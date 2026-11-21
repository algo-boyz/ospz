package ospz

// Compression boundary for SPZ load/save.
//
// Gzip (legacy v1-3): core:compress/deflate (inflate_gzip)
// ZSTD (v4 streams):  ozstd one-shot frames
//
// Batch parallel compress/decompress of independent attribute streams
// mirrors Niantic zstd-batch (thread pool over frames, not ZSTD internal MT).

import "core:bytes"
import "core:compress/deflate"
import "core:mem"
import "core:sync"
import "core:thread"
import "../ozstd"

// --- Gzip (legacy SPZ) -------------------------------------------------------

gzip_decompress :: proc(
	src: []u8,
	allocator := context.allocator,
) -> (dst: []u8, err: Spz_Error) {
	if len(src) < 2 {
		return nil, .Decompression_Failed
	}

	out: [dynamic]u8
	out.allocator = allocator
	gerr := deflate.inflate_gzip(&out, src)
	if gerr != nil {
		delete(out)
		return nil, .Decompression_Failed
	}
	return out[:], .None
}

// --- ZSTD single-frame -------------------------------------------------------

zstd_decompress_into :: proc(src, dst: []u8) -> Spz_Error {
	if len(src) == 0 {
		return .Decompression_Failed if len(dst) != 0 else .None
	}

	dctx: ozstd.DCtx
	written, err := ozstd.decompress_dctx(
		&dctx,
		raw_data(dst), len(dst),
		raw_data(src), len(src),
	)
	if err != .None || written != len(dst) {
		return .Decompression_Failed
	}
	return .None
}

zstd_decompress :: proc(
	src: []u8,
	uncompressed_size: int,
	allocator := context.allocator,
) -> (dst: []u8, err: Spz_Error) {
	if uncompressed_size < 0 {
		return nil, .Invalid_Data
	}
	dst = make([]u8, uncompressed_size, allocator)
	if e := zstd_decompress_into(src, dst); e != .None {
		delete(dst, allocator)
		return nil, e
	}
	return dst, .None
}

DEFAULT_ZSTD_LEVEL :: 3

zstd_compress :: proc(
	src: []u8,
	level: int = DEFAULT_ZSTD_LEVEL,
	allocator := context.allocator,
) -> (dst: []u8, err: Spz_Error) {
	bound := ozstd.compress_bound(len(src))
	buf := make([]u8, bound, allocator)
	written, cerr := ozstd.compress(raw_data(buf), len(buf), raw_data(src), len(src), level)
	if cerr != .None {
		delete(buf, allocator)
		return nil, .Compression_Failed
	}
	return buf[:written], .None
}

// --- Batch parallel (independent frames) -------------------------------------

// One decompress job: src frame → pre-sized dst (uncompressedSize from TOC).
Decompress_Task :: struct {
	src: []u8,
	dst: []u8,
}

// Max workers for a batch. Upstream caps at hardware_concurrency.
zstd_batch_workers :: proc(task_count: int) -> int {
	if task_count <= 1 do return 1
	n := min(task_count, 8)
	return max(n, 1)
}

// Compress every src into its own ZSTD frame, in parallel when beneficial.
// Caller owns each out[i]. On any failure all allocated chunks are freed and
// out is nil.
zstd_compress_all :: proc(
	srcs: [][]u8,
	level: int = DEFAULT_ZSTD_LEVEL,
	allocator := context.allocator,
) -> (out: [][]u8, err: Spz_Error) {
	n := len(srcs)
	if n == 0 {
		return {}, .None
	}

	// Sequential path for a single stream (or no thread support).
	if n == 1 || !thread.IS_SUPPORTED {
		out = make([][]u8, n, allocator)
		for i in 0 ..< n {
			chunk, cerr := zstd_compress(srcs[i], level, allocator)
			if cerr != .None {
				for j in 0 ..< i {
					delete(out[j], allocator)
				}
				delete(out, allocator)
				return nil, cerr
			}
			out[i] = chunk
		}
		return out, .None
	}

	out = make([][]u8, n, allocator)
	errs := make([]Spz_Error, n, context.temp_allocator)

	Compress_Shared :: struct {
		srcs:      [][]u8,
		out:       [][]u8,
		errs:      []Spz_Error,
		level:     int,
		allocator: mem.Allocator,
	}
	shared := Compress_Shared{
		srcs      = srcs,
		out       = out,
		errs      = errs,
		level     = level,
		allocator = allocator,
	}

	workers := zstd_batch_workers(n)
	pool: thread.Pool
	thread.pool_init(&pool, context.allocator, workers)
	thread.pool_start(&pool)
	defer thread.pool_destroy(&pool)

	worker :: proc(task: thread.Task) {
		s := (^Compress_Shared)(task.data)
		i := task.user_index
		chunk, cerr := zstd_compress(s.srcs[i], s.level, s.allocator)
		s.out[i] = chunk
		s.errs[i] = cerr
	}

	for i in 0 ..< n {
		thread.pool_add_task(&pool, context.allocator, worker, &shared, i)
	}
	thread.pool_finish(&pool)

	for i in 0 ..< n {
		if errs[i] != .None {
			for j in 0 ..< n {
				if out[j] != nil {
					delete(out[j], allocator)
				}
			}
			delete(out, allocator)
			return nil, errs[i]
		}
	}
	return out, .None
}

// Decompress every task in parallel. Returns one success flag per task
// (true when exactly dstSize bytes were written). Matches upstream decompressAll.
zstd_decompress_all :: proc(tasks: []Decompress_Task) -> []bool {
	n := len(tasks)
	succeeded := make([]bool, n, context.temp_allocator)
	if n == 0 {
		return succeeded
	}

	if n == 1 || !thread.IS_SUPPORTED {
		for i in 0 ..< n {
			succeeded[i] = zstd_decompress_into(tasks[i].src, tasks[i].dst) == .None
		}
		return succeeded
	}

	Decompress_Shared :: struct {
		tasks:     []Decompress_Task,
		succeeded: []bool,
	}
	shared := Decompress_Shared{tasks = tasks, succeeded = succeeded}

	workers := zstd_batch_workers(n)
	pool: thread.Pool
	thread.pool_init(&pool, context.allocator, workers)
	thread.pool_start(&pool)
	defer thread.pool_destroy(&pool)

	worker :: proc(task: thread.Task) {
		s := (^Decompress_Shared)(task.data)
		i := task.user_index
		s.succeeded[i] = zstd_decompress_into(s.tasks[i].src, s.tasks[i].dst) == .None
	}

	for i in 0 ..< n {
		thread.pool_add_task(&pool, context.allocator, worker, &shared, i)
	}
	thread.pool_finish(&pool)
	return succeeded
}