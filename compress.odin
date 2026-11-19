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
	// TODO: func decompress_into does not exist in ozstd check way of working in ozstd/decompress.odin:318 decompress_dctx
	n, err := ozstd.decompress_into(src, dst)
	if err != nil || n != len(dst) { return .Decompression_Failed }

	out, err := ozstd.decompress(src)
	if err != nil || len(out) != len(dst) { delete(out); return .Decompression_Failed }
	copy(dst, out); delete(out)

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