package ospz

// Compression boundary for SPZ load/save.
//
// Gzip (legacy v1-3): core:compress/deflate (inflate_gzip)
// ZSTD (v4 streams):  ozstd — wire in zstd_decompress_into
//
// Multi-threaded batch decompress can land later without changing callers.

import "core:bytes"
import "core:compress/deflate"
import "core:mem"
import "../ozstd"

// --- Gzip (legacy SPZ) -------------------------------------------------------

// Decompress a gzip payload in memory. Returns owned slice on success.
gzip_decompress :: proc(
	src: []u8,
	allocator := context.allocator,
) -> (dst: []u8, err: Spz_Error) {
	if len(src) < 2 {
		return nil, .Decompression_Failed
	}

	out: [dynamic]u8
	out.allocator = allocator
	// Prefer the current deflate API (gzip package is deprecated).
	gerr := deflate.inflate_gzip(&out, src)
	if gerr != nil {
		delete(out)
		return nil, .Decompression_Failed
	}
	return out[:], .None
}

// --- ZSTD (SPZ v4 attribute streams) ----------------------------------------

// Decompress one ZSTD frame into a pre-sized destination buffer.
// dst must be exactly the uncompressed size from the TOC.
// Returns .None on success.
zstd_decompress_into :: proc(src, dst: []u8) -> Spz_Error {
	if len(src) == 0 {
		return .Decompression_Failed if len(dst) != 0 else .None
	}

	// Zeroed DCtx is valid; no allocation needed for a single-shot decode.
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

// Convenience: allocate and decompress. Caller owns the result.
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

// --- ZSTD compress (SPZ v4 attribute streams) --------------------------------

// Default compression level for SPZ attribute streams. ozstd supports 3, 6, 22;
// level 3 matches typical upstream ZSTD defaults for interactive save latency.
DEFAULT_ZSTD_LEVEL :: 3

// Compress src into a newly allocated buffer. Caller owns the result.
// Uses ozstd one-shot compress (levels 3 / 6 / 22 only).
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