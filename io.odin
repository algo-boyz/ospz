package ospz

// SPZ file load path (v4 ZSTD multi-stream + legacy gzip single-stream).
// Mirrors Niantic load-spz.cc loadSpzPacked / unpackGaussians / loadSpz.

import "core:encoding/endian"
import "core:mem"
import "core:os"

FLAG_ANTIALIASED :: u8(0x1)
FLAG_HAS_EXTENSIONS :: u8(0x2)

MAX_COMPRESSION_RATIO :: u64(1024)
MIN_BYTES_PER_POINT :: u64(9)
TOC_ENTRY_BYTES :: 16 // compressedSize u64 + uncompressedSize u64

Ngsp_File_Header :: struct #packed {
	magic:           u32,
	version:         u32,
	num_points:      u32,
	sh_degree:       u8,
	fractional_bits: u8,
	flags:           u8,
	num_streams:     u8,
	toc_byte_offset: u32,
	reserved:        [12]u8,
}

#assert(size_of(Ngsp_File_Header) == 32)

Legacy_Packed_Header :: struct #packed {
	magic:           u32,
	version:         u32,
	num_points:      u32,
	sh_degree:       u8,
	fractional_bits: u8,
	flags:           u8,
	reserved:        u8,
}

#assert(size_of(Legacy_Packed_Header) == 16)

Ngsp_Stream_Info :: struct {
	compressed_size:   u64,
	uncompressed_size: u64,
	compressed_offset: int,
}

// Known extension type IDs (vendor << 16 | extension). Unknown types are skipped.
// See https://github.com/nianticlabs/spz/blob/main/extensions/README.md
Spz_Extension_Type :: enum u32 {
	ADOBE_Safe_Orbit_Camera  = 0xADBE0002,
	ADOBE_Coordinate_System  = 0xADBE0003,
	NIANTIC_Georeference     = 0x4E530001,
	NIANTIC_Custom_Attribute = 0x4E530002,
}

// One length-delimited extension record in the plaintext zone:
// [ u32 type ][ u32 byteLength ][ payload... ]
Extension_Record :: struct {
	type:        u32,
	byte_length: u32,
	payload:     []u8, // view into the file buffer; not owned
}

// Walk the ILV extension zone between the 32-byte header and the TOC.
// Unknown types are skipped; the zone must be an exact sequence of records
// (no trailing junk). Returns ok=false if the structure is invalid.
// Core Gaussian data still loads when ok=true and had_any is true (MVP:
// we do not parse known payloads yet — callers see had_skipped_extensions).
skip_extension_ilv_zone :: proc(zone: []u8) -> (records: []Extension_Record, had_any: bool, ok: bool) {
	if len(zone) == 0 {
		return nil, false, true
	}
	list: [dynamic]Extension_Record
	list.allocator = context.temp_allocator
	off := 0
	for off < len(zone) {
		if off + 8 > len(zone) {
			return nil, false, false
		}
		typ := endian.unchecked_get_u32le(zone[off:])
		blen := endian.unchecked_get_u32le(zone[off + 4:])
		off += 8
		if u64(blen) > u64(len(zone) - off) {
			return nil, false, false
		}
		payload := zone[off:off + int(blen)]
		off += int(blen)
		append(&list, Extension_Record{type = typ, byte_length = blen, payload = payload})
	}
	if off != len(zone) {
		return nil, false, false
	}
	return list[:], len(list) > 0, true
}

// --- Public API --------------------------------------------------------------

load_spz_packed_from_bytes :: proc(
	data: []u8,
	options: Unpack_Options = {},
	allocator := context.allocator,
) -> (Packed_Gaussians, Spz_Error) {
	if len(data) < 2 {
		return {}, .Invalid_Header
	}
	// v4 plaintext header
	if len(data) >= 4 &&
	   data[0] == 'N' && data[1] == 'G' && data[2] == 'S' && data[3] == 'P' {
		return load_packed_from_ngsp(data, options, allocator)
	}
	// legacy gzip
	if data[0] == 0x1f && data[1] == 0x8b {
		return load_packed_from_gzip(data, options, allocator)
	}
	return {}, .Invalid_Header
}

load_spz_packed_from_file :: proc(
	path: string,
	options: Unpack_Options = {},
	allocator := context.allocator,
) -> (Packed_Gaussians, Spz_Error) {
	data, read_err := os.read_entire_file(path, allocator)
	if read_err != nil {
		return {}, .Io_Failed
	}
	defer delete(data, allocator)
	return load_spz_packed_from_bytes(data, options, allocator)
}

load_spz_from_bytes :: proc(
	data: []u8,
	options: Unpack_Options = {},
	allocator := context.allocator,
) -> (Gaussian_Cloud, Spz_Error) {
	packed, err := load_spz_packed_from_bytes(data, options, allocator)
	if err != .None {
		return {}, err
	}
	cloud, uerr := unpack_gaussians(&packed, options, allocator)
	release_packed_gaussians(&packed, allocator)
	return cloud, uerr
}

load_spz_from_file :: proc(
	path: string,
	options: Unpack_Options = {},
	allocator := context.allocator,
) -> (Gaussian_Cloud, Spz_Error) {
	data, read_err := os.read_entire_file(path, allocator)
	if read_err != nil {
		return {}, .Io_Failed
	}
	defer delete(data, allocator)
	return load_spz_from_bytes(data, options, allocator)
}

// --- Public save API ---------------------------------------------------------

// Pack a Gaussian_Cloud and serialize it as SPZ v4 (NGSP + ZSTD streams).
// Caller owns the returned byte slice. On error the slice is nil.
// Extensions are not written yet (empty extension zone).
save_spz_to_bytes :: proc(
	g: ^Gaussian_Cloud,
	options: Pack_Options = {},
	allocator := context.allocator,
) -> (out: []u8, err: Spz_Error) {
	opts := options
	if opts.version == 0 {
		opts.version = LATEST_SPZ_HEADER_VERSION
	}
	if opts.sh1_bits == 0 {
		opts.sh1_bits = DEFAULT_SH1_BITS
	}
	if opts.sh_rest_bits == 0 {
		opts.sh_rest_bits = DEFAULT_SH_REST_BITS
	}

	if opts.version < MIN_ZSTD_SPZ_HEADER_VERSION {
		// Legacy gzip path not ported in this change set.
		return nil, .Unsupported_Version
	}
	if opts.version > LATEST_SPZ_HEADER_VERSION {
		return nil, .Unsupported_Version
	}

	packed := pack_gaussians(g, opts)
	if g.num_points > 0 && packed.num_points == 0 {
		return nil, .Invalid_Data
	}
	defer release_packed_gaussians(&packed, allocator)

	return save_packed_to_ngsp(&packed, opts.version, allocator)
}

save_spz_to_file :: proc(
	g: ^Gaussian_Cloud,
	options: Pack_Options = {},
	path: string,
	allocator := context.allocator,
) -> Spz_Error {
	data, err := save_spz_to_bytes(g, options, allocator)
	if err != .None {
		return err
	}
	defer delete(data, allocator)
	if werr := os.write_entire_file(path, data); werr != nil {
		return .Io_Failed
	}
	return .None
}

// Serialize already-packed attribute streams as NGSP v4.
// extension zone is empty; FLAG_HAS_EXTENSIONS is clear.
save_packed_to_ngsp :: proc(
	packed: ^Packed_Gaussians,
	version: u32,
	allocator := context.allocator,
) -> (out: []u8, err: Spz_Error) {
	if packed == nil || !packed_sizes_valid(packed) {
		return nil, .Invalid_Data
	}

	// Core streams in the order loaders expect (skip empty SH at degree 0).
	srcs := core_streams(packed)
	if len(srcs) == 0 && packed.num_points > 0 {
		return nil, .Invalid_Data
	}

	// Compress each stream sequentially (parallel batch can land later).
	compressed, cerr := zstd_compress_all(srcs, DEFAULT_ZSTD_LEVEL, context.temp_allocator)
	if cerr != .None {
		return nil, cerr
	}

	return write_ngsp(packed, version, {}, compressed, srcs, 0, allocator)
}

// Non-empty core attribute buffers, load order: positions, alphas, colors,
// scales, rotations, sh.
core_streams :: proc(packed: ^Packed_Gaussians) -> [][]u8 {
	attrs := [6]Splat_Attribute{
		.Positions, .Alphas, .Colors, .Scales, .Rotations, .Sh,
	}
	out: [dynamic][]u8
	out.allocator = context.temp_allocator
	for attr in attrs {
		buf := packed_buffer(packed, attr)
		if len(buf) > 0 {
			append(&out, buf)
		}
	}
	return out[:]
}

// Assemble [header][extension zone][TOC][compressed streams].
// host_extra_bytes is added to the last TOC compressedSize (extension frames);
// pass 0 when no frames are appended.
write_ngsp :: proc(
	packed: ^Packed_Gaussians,
	version: u32,
	extension_zone: []u8,
	compressed: [][]u8,
	uncompressed: [][]u8,
	host_extra_bytes: u64,
	allocator := context.allocator,
) -> (out: []u8, err: Spz_Error) {
	if len(compressed) != len(uncompressed) {
		return nil, .Invalid_Data
	}
	num_streams := len(compressed)
	if num_streams > 255 {
		return nil, .Invalid_Data
	}

	toc_byte_offset := u64(size_of(Ngsp_File_Header)) + u64(len(extension_zone))
	if toc_byte_offset > u64(max(u32)) {
		return nil, .Invalid_Data
	}

	total := toc_byte_offset + u64(num_streams) * TOC_ENTRY_BYTES + host_extra_bytes
	for chunk in compressed {
		total += u64(len(chunk))
	}
	if total > u64(max(int)) {
		return nil, .Allocation_Failed
	}

	buf := make([]u8, int(total), allocator)

	header: Ngsp_File_Header
	header.magic = NGSP_MAGIC
	header.version = version
	header.num_points = u32(packed.num_points)
	header.sh_degree = u8(packed.sh_degree)
	header.fractional_bits = u8(packed.fractional_bits)
	flags: u8 = 0
	if packed.antialiased {
		flags |= FLAG_ANTIALIASED
	}
	if len(extension_zone) > 0 {
		flags |= FLAG_HAS_EXTENSIONS
	}
	header.flags = flags
	header.num_streams = u8(num_streams)
	header.toc_byte_offset = u32(toc_byte_offset)

	endian.unchecked_put_u32le(buf[0:], header.magic)
	endian.unchecked_put_u32le(buf[4:], header.version)
	endian.unchecked_put_u32le(buf[8:], header.num_points)
	buf[12] = header.sh_degree
	buf[13] = header.fractional_bits
	buf[14] = header.flags
	buf[15] = header.num_streams
	endian.unchecked_put_u32le(buf[16:], header.toc_byte_offset)
	// reserved[12] already zero from make

	if len(extension_zone) > 0 {
		copy(buf[size_of(Ngsp_File_Header):], extension_zone)
	}

	for i in 0 ..< num_streams {
		cs := u64(len(compressed[i]))
		if i + 1 == num_streams {
			cs += host_extra_bytes
		}
		us := u64(len(uncompressed[i]))
		e := int(toc_byte_offset) + i * TOC_ENTRY_BYTES
		endian.unchecked_put_u64le(buf[e:], cs)
		endian.unchecked_put_u64le(buf[e + 8:], us)
	}

	off := int(toc_byte_offset) + num_streams * TOC_ENTRY_BYTES
	for chunk in compressed {
		copy(buf[off:], chunk)
		off += len(chunk)
	}
	return buf, .None
}

// Bulk decode Packed_Gaussians -> Gaussian_Cloud (coord convert RUB -> options.to).
unpack_gaussians :: proc(
	p: ^Packed_Gaussians,
	options: Unpack_Options,
	allocator := context.allocator,
) -> (Gaussian_Cloud, Spz_Error) {
	result: Gaussian_Cloud
	if p == nil || !packed_sizes_valid(p) {
		return result, .Invalid_Data
	}
	n := int(p.num_points)
	sh_dim := int(dim_for_degree(p.sh_degree))
	uses_f16 := packed_uses_float16(p)
	uses_st := p.uses_quaternion_smallest_three

	result.num_points = p.num_points
	result.sh_degree = p.sh_degree
	result.antialiased = p.antialiased
	result.positions = make([]f32, n * 3, allocator)
	result.scales = make([]f32, n * 3, allocator)
	result.rotations = make([]f32, n * 4, allocator)
	result.alphas = make([]f32, n, allocator)
	result.colors = make([]f32, n * 3, allocator)
	result.sh = make([]f32, n * sh_dim * 3, allocator)

	// Positions
	if uses_f16 {
		for i in 0 ..< n * 3 {
			bits := u16(p.positions[i * 2]) | (u16(p.positions[i * 2 + 1]) << 8)
			result.positions[i] = half_to_float(Half(bits))
		}
	} else {
		scale := 1.0 / f32(u32(1) << u32(p.fractional_bits))
		for i in 0 ..< n * 3 {
			fixed := i32(p.positions[i * 3 + 0]) |
				(i32(p.positions[i * 3 + 1]) << 8) |
				(i32(p.positions[i * 3 + 2]) << 16)
			if (fixed & 0x800000) != 0 {
				fixed |= ~i32(0xFFFFFF) // sign extend 24-bit
			}
			result.positions[i] = f32(fixed) * scale
		}
	}

	for i in 0 ..< n * 3 {
		result.scales[i] = f32(p.scales[i]) / 16.0 - 10.0
		result.colors[i] = ((f32(p.colors[i]) / 255.0) - 0.5) / COLOR_SCALE
	}
	for i in 0 ..< n {
		result.alphas[i] = inv_sigmoid(f32(p.alphas[i]) / 255.0)
	}

	// Identity converter first; convert_coordinates does RUB -> to below.
	id := identity_coordinate_converter()
	for i in 0 ..< n {
		if uses_st {
			unpack_quaternion_smallest_three(
				cast(^[4]f32)&result.rotations[i * 4],
				p.rotations[i * 4:i * 4 + 4],
				&id,
			)
		} else {
			unpack_quaternion_first_three(
				cast(^[4]f32)&result.rotations[i * 4],
				p.rotations[i * 3:i * 3 + 3],
				&id,
			)
		}
	}

	for i in 0 ..< len(p.sh) {
		result.sh[i] = unquantize_sh(p.sh[i])
	}

	// Storage is RUB; convert to caller's system.
	convert_coordinates(&result, .RUB, options.to)
	return result, .None
}

release_packed_gaussians :: proc(p: ^Packed_Gaussians, allocator := context.allocator) {
	if p == nil do return
	delete(p.positions, allocator)
	delete(p.scales, allocator)
	delete(p.rotations, allocator)
	delete(p.alphas, allocator)
	delete(p.colors, allocator)
	delete(p.sh, allocator)
	p^ = {}
}

release_gaussian_cloud :: proc(g: ^Gaussian_Cloud, allocator := context.allocator) {
	if g == nil do return
	delete(g.positions, allocator)
	delete(g.scales, allocator)
	delete(g.rotations, allocator)
	delete(g.alphas, allocator)
	delete(g.colors, allocator)
	delete(g.sh, allocator)
	g^ = {}
}

// --- v4 NGSP -----------------------------------------------------------------

load_packed_from_ngsp :: proc(
	data: []u8,
	options: Unpack_Options,
	allocator: mem.Allocator,
) -> (Packed_Gaussians, Spz_Error) {
	_ = options
	if len(data) < size_of(Ngsp_File_Header) {
		return {}, .Invalid_Header
	}
	header := read_ngsp_header(data)
	if header.magic != NGSP_MAGIC {
		return {}, .Invalid_Header
	}
	if header.version < MIN_ZSTD_SPZ_HEADER_VERSION ||
	   header.version > LATEST_SPZ_HEADER_VERSION {
		return {}, .Unsupported_Version
	}
	if header.sh_degree > u8(SH_MAX_DEGREE) {
		return {}, .Invalid_Data
	}
	if !num_points_plausible(header.num_points, len(data)) {
		return {}, .Invalid_Data
	}

	num_points := i32(header.num_points)
	sh_dim := dim_for_degree(i32(header.sh_degree))
	uses_st := header.version >= MIN_SMALLEST_THREE_QUATERNIONS_VERSION

	result: Packed_Gaussians
	result.version = header.version
	result.num_points = num_points
	result.sh_degree = i32(header.sh_degree)
	result.fractional_bits = i32(header.fractional_bits)
	result.antialiased = (header.flags & FLAG_ANTIALIASED) != 0
	result.uses_quaternion_smallest_three = uses_st

	if has_extension_zone(header, len(data)) {
		zone := data[size_of(Ngsp_File_Header):header.toc_byte_offset]
		_, had_any, zone_ok := skip_extension_ilv_zone(zone)
		if !zone_ok {
			// Malformed extension zone: still attempt core load (robustness),
			// but mark that extensions were present and not fully trusted.
			result.had_skipped_extensions = true
		} else if had_any {
			// MVP: known types are not decoded yet; mark skipped so callers
			// know coordinate-system / georef / custom attrs were ignored.
			result.had_skipped_extensions = true
		}
	}

	// Pre-size attribute buffers (order must match save / TOC streams).
	result.positions = make([]u8, int(num_points) * 9, allocator)
	result.alphas = make([]u8, int(num_points), allocator)
	result.colors = make([]u8, int(num_points) * 3, allocator)
	result.scales = make([]u8, int(num_points) * 3, allocator)
	rot_bytes := 4 if uses_st else 3
	result.rotations = make([]u8, int(num_points) * rot_bytes, allocator)
	result.sh = make([]u8, int(num_points) * int(sh_dim) * 3, allocator)

	dests := make([dynamic][]u8, 0, 6, context.temp_allocator)
	if len(result.positions) > 0 do append(&dests, result.positions)
	if len(result.alphas) > 0 do append(&dests, result.alphas)
	if len(result.colors) > 0 do append(&dests, result.colors)
	if len(result.scales) > 0 do append(&dests, result.scales)
	if len(result.rotations) > 0 do append(&dests, result.rotations)
	if len(result.sh) > 0 do append(&dests, result.sh)

	toc, toc_ok := read_ngsp_toc(data, header)
	if !toc_ok {
		release_packed_gaussians(&result, allocator)
		return {}, .Invalid_Data
	}
	if len(toc) != len(dests) || len(toc) == 0 {
		release_packed_gaussians(&result, allocator)
		return {}, .Invalid_Data
	}

	tasks := make([]Decompress_Task, len(dests), context.temp_allocator)
	for i in 0 ..< len(dests) {
		info := toc[i]
		if int(info.uncompressed_size) != len(dests[i]) {
			release_packed_gaussians(&result, allocator)
			return {}, .Invalid_Data
		}
		off := info.compressed_offset
		end := off + int(info.compressed_size)
		if off < 0 || end > len(data) {
			release_packed_gaussians(&result, allocator)
			return {}, .Invalid_Data
		}
		// Last stream may host skippable extension frames after its zstd frame;
		// ozstd steps over them so the whole TOC span is safe to pass.
		tasks[i] = {src = data[off:end], dst = dests[i]}
	}

	succeeded := zstd_decompress_all(tasks)
	for ok in succeeded {
		if !ok {
			release_packed_gaussians(&result, allocator)
			return {}, .Decompression_Failed
		}
	}

	if !packed_sizes_valid(&result) {
		release_packed_gaussians(&result, allocator)
		return {}, .Invalid_Data
	}
	return result, .None
}

read_ngsp_header :: proc(data: []u8) -> Ngsp_File_Header {
	h: Ngsp_File_Header
	h.magic = endian.unchecked_get_u32le(data[0:])
	h.version = endian.unchecked_get_u32le(data[4:])
	h.num_points = endian.unchecked_get_u32le(data[8:])
	h.sh_degree = data[12]
	h.fractional_bits = data[13]
	h.flags = data[14]
	h.num_streams = data[15]
	h.toc_byte_offset = endian.unchecked_get_u32le(data[16:])
	copy(h.reserved[:], data[20:32])
	return h
}

read_ngsp_toc :: proc(data: []u8, header: Ngsp_File_Header) -> ([]Ngsp_Stream_Info, bool) {
	if header.toc_byte_offset < u32(size_of(Ngsp_File_Header)) {
		return nil, false
	}
	toc_end := u64(header.toc_byte_offset) + u64(header.num_streams) * TOC_ENTRY_BYTES
	if toc_end > u64(len(data)) {
		return nil, false
	}
	infos := make([]Ngsp_Stream_Info, header.num_streams, context.temp_allocator)
	compressed_offset := int(toc_end)
	for i in 0 ..< int(header.num_streams) {
		e := int(header.toc_byte_offset) + i * TOC_ENTRY_BYTES
		cs := endian.unchecked_get_u64le(data[e:])
		us := endian.unchecked_get_u64le(data[e + 8:])
		infos[i] = {
			compressed_size   = cs,
			uncompressed_size = us,
			compressed_offset = compressed_offset,
		}
		if cs > u64(len(data) - compressed_offset) {
			return nil, false
		}
		compressed_offset += int(cs)
	}
	if compressed_offset != len(data) {
		return nil, false
	}
	return infos, true
}

has_extension_zone :: proc(header: Ngsp_File_Header, size: int) -> bool {
	return (header.flags & FLAG_HAS_EXTENSIONS) != 0 &&
		u32(size_of(Ngsp_File_Header)) < header.toc_byte_offset &&
		int(header.toc_byte_offset) <= size
}

num_points_plausible :: proc(num_points: u32, file_size: int) -> bool {
	if num_points == 0 {
		return true
	}
	if num_points > u32(max(i32)) {
		return false
	}
	max_points := (u64(file_size) * MAX_COMPRESSION_RATIO) / MIN_BYTES_PER_POINT
	return u64(num_points) <= max_points
}

// --- Legacy gzip -------------------------------------------------------------

load_packed_from_gzip :: proc(
	data: []u8,
	options: Unpack_Options,
	allocator: mem.Allocator,
) -> (Packed_Gaussians, Spz_Error) {
	_ = options
	raw, err := gzip_decompress(data, allocator)
	if err != .None {
		return {}, err
	}
	defer delete(raw, allocator)
	return deserialize_packed_gaussians(raw, allocator)
}

deserialize_packed_gaussians :: proc(
	data: []u8,
	allocator: mem.Allocator,
) -> (Packed_Gaussians, Spz_Error) {
	if len(data) < size_of(Legacy_Packed_Header) {
		return {}, .Invalid_Header
	}
	magic := endian.unchecked_get_u32le(data[0:])
	version := endian.unchecked_get_u32le(data[4:])
	num_points_u := endian.unchecked_get_u32le(data[8:])
	sh_degree := data[12]
	fractional_bits := data[13]
	flags := data[14]

	if magic != NGSP_MAGIC {
		return {}, .Invalid_Header
	}
	if version < 1 || version > LATEST_SPZ_HEADER_VERSION {
		return {}, .Unsupported_Version
	}
	if sh_degree > u8(SH_MAX_DEGREE) {
		return {}, .Invalid_Data
	}
	remaining := len(data) - size_of(Legacy_Packed_Header)
	if num_points_u == 0 ||
	   num_points_u > u32(max(i32)) ||
	   (remaining > 0 && int(num_points_u) > remaining / int(MIN_BYTES_PER_POINT)) {
		return {}, .Invalid_Data
	}

	num_points := i32(num_points_u)
	sh_dim := dim_for_degree(i32(sh_degree))
	uses_f16 := version == 1
	uses_st := version >= MIN_SMALLEST_THREE_QUATERNIONS_VERSION

	result: Packed_Gaussians
	result.version = version
	result.num_points = num_points
	result.sh_degree = i32(sh_degree)
	result.fractional_bits = i32(fractional_bits)
	result.antialiased = (flags & FLAG_ANTIALIASED) != 0
	result.uses_quaternion_smallest_three = uses_st
	if (flags & FLAG_HAS_EXTENSIONS) != 0 {
		result.had_skipped_extensions = true
	}

	pos_b := 6 if uses_f16 else 9
	rot_b := 4 if uses_st else 3
	n := int(num_points)

	need := n * pos_b + n + n * 3 + n * 3 + n * rot_b + n * int(sh_dim) * 3
	off := size_of(Legacy_Packed_Header)
	if off + need > len(data) {
		return {}, .Invalid_Data
	}

	take :: proc(data: []u8, off: ^int, count: int, allocator: mem.Allocator) -> []u8 {
		s := data[off^:off^ + count]
		off^ += count
		out := make([]u8, count, allocator)
		copy(out, s)
		return out
	}

	result.positions = take(data, &off, n * pos_b, allocator)
	result.alphas = take(data, &off, n, allocator)
	result.colors = take(data, &off, n * 3, allocator)
	result.scales = take(data, &off, n * 3, allocator)
	result.rotations = take(data, &off, n * rot_b, allocator)
	result.sh = take(data, &off, n * int(sh_dim) * 3, allocator)

	if !packed_sizes_valid(&result) {
		release_packed_gaussians(&result, allocator)
		return {}, .Invalid_Data
	}
	return result, .None
}