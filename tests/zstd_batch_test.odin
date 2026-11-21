package testospz

import "core:bytes"
import "core:mem"
import "core:slice"
import "core:testing"
import "core:thread"
import spz "../"

// Distinct compressible payloads so parallel workers can't silently swap results.
make_stream_payload :: proc(seed: int, nbytes: int, allocator := context.allocator) -> []u8 {
	buf := make([]u8, nbytes, allocator)
	for i in 0 ..< nbytes {
		// Mix of runs + seed so each stream is unique and still compresses.
		v := u8((i / 17 + seed * 31) & 0xff)
		if (i % 64) < 40 {
			v = u8(seed & 0xff) // longer runs for better ratio
		}
		buf[i] = v
	}
	return buf
}

@(test)
test_zstd_compress_decompress_all_roundtrip :: proc(t: ^testing.T) {
	// Typical SPZ attribute count (positions, alphas, colors, scales, rotations, sh).
	N :: 6
	sizes := [N]int{4096, 1024, 3072, 3072, 4096, 8192}

	srcs := make([][]u8, N, context.allocator)
	defer {
		for s in srcs do delete(s)
		delete(srcs)
	}
	for i in 0 ..< N {
		srcs[i] = make_stream_payload(i + 1, sizes[i])
	}

	compressed, cerr := spz.zstd_compress_all(srcs, spz.DEFAULT_ZSTD_LEVEL, context.allocator)
	testing.expect_value(t, cerr, spz.Spz_Error.None)
	testing.expect(t, len(compressed) == N)
	defer {
		for c in compressed do delete(c)
		delete(compressed)
	}

	for i in 0 ..< N {
		testing.expect(t, len(compressed[i]) > 0)
		testing.expect(t, len(compressed[i]) < len(srcs[i])) // should compress
	}

	dests := make([][]u8, N, context.allocator)
	defer {
		for d in dests do delete(d)
		delete(dests)
	}
	tasks := make([]spz.Decompress_Task, N, context.temp_allocator)
	for i in 0 ..< N {
		dests[i] = make([]u8, len(srcs[i]), context.allocator)
		tasks[i] = {src = compressed[i], dst = dests[i]}
	}

	succeeded := spz.zstd_decompress_all(tasks)
	testing.expect(t, len(succeeded) == N)
	for ok in succeeded {
		testing.expect(t, ok)
	}

	for i in 0 ..< N {
        testing.expect(t, bytes.equal(dests[i], srcs[i]), "stream round-trip mismatch")
    }
}

@(test)
test_zstd_compress_all_single_stream_sequential_path :: proc(t: ^testing.T) {
	// n == 1 must take the sequential branch and still work.
	src := make_stream_payload(42, 2048)
	defer delete(src)

	srcs := [][]u8{src}
	compressed, cerr := spz.zstd_compress_all(srcs, spz.DEFAULT_ZSTD_LEVEL, context.allocator)
	testing.expect_value(t, cerr, spz.Spz_Error.None)
	testing.expect(t, len(compressed) == 1)
	defer {
		delete(compressed[0])
		delete(compressed)
	}

	dst := make([]u8, len(src), context.allocator)
	defer delete(dst)
	tasks := []spz.Decompress_Task{{src = compressed[0], dst = dst}}
	succeeded := spz.zstd_decompress_all(tasks)
	testing.expect(t, len(succeeded) == 1 && succeeded[0])
	testing.expect(t, bytes.equal(dst, src))
}

@(test)
test_zstd_decompress_all_empty :: proc(t: ^testing.T) {
	succeeded := spz.zstd_decompress_all({})
	testing.expect(t, len(succeeded) == 0)

	compressed, cerr := spz.zstd_compress_all({}, spz.DEFAULT_ZSTD_LEVEL)
	testing.expect_value(t, cerr, spz.Spz_Error.None)
	testing.expect(t, len(compressed) == 0)
}

@(test)
test_zstd_batch_workers_cap :: proc(t: ^testing.T) {
	testing.expect_value(t, spz.zstd_batch_workers(0), 1)
	testing.expect_value(t, spz.zstd_batch_workers(1), 1)
	testing.expect_value(t, spz.zstd_batch_workers(4), 4)
	testing.expect_value(t, spz.zstd_batch_workers(16), 8) // hard cap
}

@(test)
test_zstd_parallel_path_exercised_when_supported :: proc(t: ^testing.T) {
	// Smoke that multi-stream path is reachable; if threads aren't supported
	// the sequential fallback still has to be correct.
	N :: 4
	srcs := make([][]u8, N, context.allocator)
	defer {
		for s in srcs do delete(s)
		delete(srcs)
	}
	for i in 0 ..< N {
		srcs[i] = make_stream_payload(100 + i, 16 * 1024)
	}

	compressed, cerr := spz.zstd_compress_all(srcs)
	testing.expect_value(t, cerr, spz.Spz_Error.None)
	defer {
		for c in compressed do delete(c)
		delete(compressed)
	}

	dests := make([][]u8, N, context.allocator)
	defer {
		for d in dests do delete(d)
		delete(dests)
	}
	tasks := make([]spz.Decompress_Task, N, context.temp_allocator)
	for i in 0 ..< N {
		dests[i] = make([]u8, len(srcs[i]), context.allocator)
		tasks[i] = {src = compressed[i], dst = dests[i]}
	}

	succeeded := spz.zstd_decompress_all(tasks)
	for i in 0 ..< N {
		testing.expect(t, succeeded[i])
		testing.expect(t, bytes.equal(dests[i], srcs[i]))
	}

	// Optional visibility when running with -test-name or verbose.
	if thread.IS_SUPPORTED {
		testing.expect(t, spz.zstd_batch_workers(N) > 1)
	}
}