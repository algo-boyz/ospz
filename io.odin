package ospz

// SPZ file load path (v4 ZSTD multi-stream + legacy gzip single-stream).
// Mirrors Niantic load-spz.cc loadSpzPacked / unpackGaussians / loadSpz.

import "core:encoding/endian"
import "core:mem"
import "core:os"
import "core:slice"

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
	data, ok := os.read_entire_file(path, allocator)
	if !ok {
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
	// packed buffers are owned; unpack copies into cloud then we free packed
	cloud, uerr := unpack_gaussians(&packed, options, allocator)
	release_packed_gaussians(&packed, allocator)
	return cloud, uerr
}

load_spz_from_file :: proc(
	path: string,
	options: Unpack_Options = {},
	allocator := context.allocator,
) -> (Gaussian_Cloud, Spz_Error) {
	data, ok := os.read_entire_file(path, allocator)
	if !ok {
		return {}, .Io_Failed
	}
	defer delete(data, allocator)
	return load_spz_from_bytes(data, options, allocator)
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
		// Skip ILVs for now; core streams still load.
		result.had_skipped_extensions = true
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
	append_nonempty :: proc(d: ^[dynamic][]u8, buf: []u8) {
		if len(buf) > 0 do append(d, buf)
	}
	append_nonempty(&dests, result.positions)
	append_nonempty(&dests, result.alphas)
	append_nonempty(&dests, result.colors)
	append_nonempty(&dests, result.scales)
	append_nonempty(&dests, result.rotations)
	append_nonempty(&dests, result.sh)

	toc, toc_ok := read_ngsp_toc(data, header)
	if !toc_ok {
		release_packed_gaussians(&result, allocator)
		return {}, .Invalid_Data
	}
	if len(toc) != len(dests) || len(toc) == 0 {
		release_packed_gaussians(&result, allocator)
		return {}, .Invalid_Data
	}

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
		// Last stream may host extension frames after its zstd frame; zstd
		// decoders stop after one frame, so decompressing the whole span is OK
		// when ozstd stops at frame end. Prefer feeding only the core frame
		// once ZSTD_findFrameCompressedSize is available.
		src := data[off:end]
		if e := zstd_decompress_into(src, dests[i]); e != .None {
			release_packed_gaussians(&result, allocator)
			return {}, e
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
	// Little-endian field load
	h.magic = endian.unchecked_get_u32(data[0:], .Little)
	h.version = endian.unchecked_get_u32(data[4:], .Little)
	h.num_points = endian.unchecked_get_u32(data[8:], .Little)
	h.sh_degree = data[12]
	h.fractional_bits = data[13]
	h.flags = data[14]
	h.num_streams = data[15]
	h.toc_byte_offset = endian.unchecked_get_u32(data[16:], .Little)
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
		cs := endian.unchecked_get_u64(data[e:], .Little)
		us := endian.unchecked_get_u64(data[e + 8:], .Little)
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