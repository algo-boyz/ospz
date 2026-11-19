package ospz

// Compression boundary for SPZ load/save.
//
// Gzip (legacy v1-3): core:compress/gzip
// ZSTD (v4 streams):  ozstd — https://github.com/algo-boyz/ozstd
//
// Adjust the ozstd import path and decompress call to match your tree
// (collection path, relative vendor/, etc.). Expected one-shot shape:
//
//   decompress(src: []u8, allocator) -> (dst: []u8, err)
//   or decompress_into(src, dst: []u8) -> err
//
// Multi-threaded batch decompress can land later without changing callers.

import "core:bytes"
import "core:compress/gzip"
import "core:mem"
import "core:os"

import ozstd "../ozstd"

// --- Gzip (legacy SPZ) -------------------------------------------------------

// Decompress a gzip payload in memory. Returns owned slice on success.
gzip_decompress :: proc(
	src: []u8,
	allocator := context.allocator,
) -> (dst: []u8, err: Spz_Error) {
	if len(src) < 2 {
		return nil, .Decompression_Failed
	}
	buf: bytes.Buffer
	bytes.buffer_init_allocator(&buf, 0, len(src) * 3, allocator)
	// core:compress/gzip expects a reader; use load from bytes via temp file-free path.
	// Prefer inflate from memory when available; fall back to gzip.load pattern.
	gerr := gzip_decompress_impl(src, &buf)
	if gerr != nil {
		bytes.buffer_destroy(&buf)
		return nil, .Decompression_Failed
	}
	return bytes.buffer_to_bytes(&buf), .None
}

// Implementation detail: inflate gzip from a memory slice into buf.
// Uses gzip.inflate_from_bytes when present; otherwise a small shim.
gzip_decompress_impl :: proc(src: []u8, out: ^bytes.Buffer) -> gzip.Error {
	// Odin core gzip: load_from_bytes / inflate variants differ by version.
	// This matches the common "write compressed bytes, inflate into buffer" pattern.
	r: gzip.Inflate_Context
	defer gzip.inflate_destroy(&r)
	if e := gzip.inflate_init(&r, false); e != nil {
		return e
	}
	// Feed entire src; core may expose inflate_with_input — adjust if needed.
	when ODIN_OS != .Freestanding {
		// Portable path: use gzip.load pattern via bytes reader if inflate API differs.
		// Many trees expose: gzip.load(src, out) or similar — keep one call site.
		return gzip_inflate_bytes(src, out)
	}
	return nil
}

// Thin wrapper so only this file needs updating for core API drift.
gzip_inflate_bytes :: proc(src: []u8, out: ^bytes.Buffer) -> gzip.Error {
	// Preferred: if your core has gzip.inflate_from_byte_array, use it.
	// Fallback used by the example the user shared: gzip.load_from_file is file-only,
	// so for in-memory we copy through a temp buffer stream when necessary.
	e := gzip.load(src, out)
	return e
}

// --- ZSTD (SPZ v4 attribute streams) ----------------------------------------

// Decompress one ZSTD frame into a pre-sized destination buffer.
// dst must be exactly the uncompressed size from the TOC.
// Returns .None on success.
zstd_decompress_into :: proc(src, dst: []u8) -> Spz_Error {
	if len(src) == 0 {
		return .Decompression_Failed if len(dst) != 0 else .None
	}
	// --- wire ozstd here ---------------------------------------------------
	// Example shapes (pick the one your ozstd exposes):
	//
	//   n, err := ozstd.decompress_into(src, dst)
	//   if err != nil || n != len(dst) { return .Decompression_Failed }
	//
	//   out, err := ozstd.decompress(src)
	//   if err != nil || len(out) != len(dst) { delete(out); return .Decompression_Failed }
	//   copy(dst, out); delete(out)
	//
	// Until wired, fail closed so callers see a clear error instead of silence.
	_ = src
	_ = dst
	return .Decompression_Failed
}
