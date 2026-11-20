package ospz

// PLY interchange for 3D Gaussian splat clouds.
// Mirrors Niantic loadSplatFromPly / saveSplatToPly (binary little-endian only).
//
// Load assumes source PLY is RDF (typical 3DGS export) and converts to options.to.
// Save converts from options.from to RDF for the written file.
//
// Rotations in Gaussian_Cloud are xyzw; PLY property order is wxyz (rot_0 = w).

import "core:fmt"
import "core:mem"
import "core:os"
import "core:strconv"
import "core:strings"

// Extra non-vertex elements encountered in the header (skipped on load).
Ply_Extra_Element :: struct {
	name:              string,
	count:             i32,
	bytes_per_element: int,
	is_known:          bool, // reserved for future extension elements
}

// --- Public API --------------------------------------------------------------

load_splat_from_ply :: proc(
	path: string,
	options: Unpack_Options = {},
	allocator := context.allocator,
) -> (Gaussian_Cloud, Spz_Error) {
	data, read_err := os.read_entire_file(path, allocator)
	if read_err != nil {
		return {}, .Io_Failed
	}
	defer delete(data, allocator)
	return load_splat_from_ply_bytes(data, options, allocator)
}

load_splat_from_ply_bytes :: proc(
	data: []u8,
	options: Unpack_Options = {},
	allocator := context.allocator,
) -> (cloud: Gaussian_Cloud, err: Spz_Error) {
	if len(data) < 4 {
		return {}, .Invalid_Header
	}

	cursor := 0
	line: string
	ok: bool

	line, cursor, ok = ply_next_header_line(data, cursor)
	if !ok || line != "ply" {
		return {}, .Invalid_Header
	}

	line, cursor, ok = ply_next_header_line(data, cursor)
	if !ok || line != "format binary_little_endian 1.0" {
		return {}, .Unsupported_Version
	}

	line, cursor, ok = ply_next_header_line(data, cursor)
	if !ok || !strings.has_prefix(line, "element vertex ") {
		return {}, .Invalid_Header
	}
	num_points_str := strings.trim_space(line[len("element vertex "):])
	num_points64, parse_ok := strconv.parse_i64(num_points_str)
	if !parse_ok || num_points64 <= 0 || num_points64 > i64(max(i32)) {
		return {}, .Invalid_Data
	}
	num_points := i32(num_points64)

	fields := make(map[string]int, allocator)
	defer delete(fields)

	extra_elements := make([dynamic]Ply_Extra_Element, 0, 4, allocator)
	defer {
		for e in extra_elements {
			delete(e.name, allocator)
		}
		delete(extra_elements)
	}

	Parse_State :: enum { In_Vertex, In_Extra }
	state := Parse_State.In_Vertex
	current_name: string
	current_count: i32
	current_bytes: int
	current_known: bool
	prop_index := 0

	for {
		line, cursor, ok = ply_next_header_line(data, cursor)
		if !ok {
			return {}, .Invalid_Header
		}
		if line == "end_header" {
			if state == .In_Extra && current_count > 0 {
				append(&extra_elements, Ply_Extra_Element{
					name = strings.clone(current_name, allocator),
					count = current_count,
					bytes_per_element = current_bytes,
					is_known = current_known,
				})
			}
			break
		}

		// New non-vertex element
		if strings.has_prefix(line, "element ") && !strings.has_prefix(line, "element vertex ") {
			if state == .In_Extra && current_count > 0 {
				append(&extra_elements, Ply_Extra_Element{
					name = strings.clone(current_name, allocator),
					count = current_count,
					bytes_per_element = current_bytes,
					is_known = current_known,
				})
			}
			rest := line[len("element "):]
			space := strings.index_byte(rest, ' ')
			if space < 0 {
				return {}, .Invalid_Header
			}
			current_name = rest[:space]
			count_str := strings.trim_space(rest[space+1:])
			c64, cok := strconv.parse_i64(count_str)
			if !cok || c64 < 0 {
				return {}, .Invalid_Header
			}
			current_count = i32(c64)
			current_bytes = 0
			current_known = false // extensions later
			state = .In_Extra
			continue
		}

		if state == .In_Extra {
			if strings.has_prefix(line, "property ") {
				current_bytes += ply_property_size(line)
			}
			continue
		}

		// Vertex properties: only float supported (matches upstream)
		if !strings.has_prefix(line, "property float ") {
			return {}, .Invalid_Data
		}
		name := line[len("property float "):]
		fields[strings.clone(name, allocator)] = prop_index
		prop_index += 1
	}
	defer {
		for k in fields {
			delete(k, allocator)
		}
	}

	index :: proc(fields: map[string]int, name: string) -> (int, bool) {
		v, ok := fields[name]
		return v, ok
	}

	px, okx := index(fields, "x")
	py, oky := index(fields, "y")
	pz, okz := index(fields, "z")
	if !okx || !oky || !okz {
		return {}, .Invalid_Data
	}
	s0, oks0 := index(fields, "scale_0")
	s1, oks1 := index(fields, "scale_1")
	s2, oks2 := index(fields, "scale_2")
	if !oks0 || !oks1 || !oks2 {
		return {}, .Invalid_Data
	}
	// PLY rot_0=w, rot_1=x, rot_2=y, rot_3=z → store xyzw
	r1, okr1 := index(fields, "rot_1")
	r2, okr2 := index(fields, "rot_2")
	r3, okr3 := index(fields, "rot_3")
	r0, okr0 := index(fields, "rot_0")
	if !okr0 || !okr1 || !okr2 || !okr3 {
		return {}, .Invalid_Data
	}
	a_idx, oka := index(fields, "opacity")
	if !oka {
		return {}, .Invalid_Data
	}
	c0, okc0 := index(fields, "f_dc_0")
	c1, okc1 := index(fields, "f_dc_1")
	c2, okc2 := index(fields, "f_dc_2")
	if !okc0 || !okc1 || !okc2 {
		return {}, .Invalid_Data
	}

	sh_idx := make([dynamic]int, 0, SH_MAX_COEFFS * 3, context.temp_allocator)
	for i in 0 ..< SH_MAX_COEFFS * 3 {
		key := fmt.tprintf("f_rest_%d", i)
		if v, ok := fields[key]; ok {
			append(&sh_idx, v)
		} else {
			break
		}
	}
	if len(sh_idx) % 3 != 0 {
		return {}, .Invalid_Data
	}
	sh_dim := len(sh_idx) / 3
	sh_degree := degree_for_dim(i32(sh_dim))

	num_fields := prop_index
	if num_fields <= 0 {
		return {}, .Invalid_Data
	}

	body_bytes := int(num_points) * num_fields * 4
	if cursor + body_bytes > len(data) {
		return {}, .Invalid_Data
	}

	// Read vertex floats (little-endian)
	values := make([]f32, int(num_points) * num_fields, allocator)
	defer delete(values, allocator)
	src := data[cursor:cursor+body_bytes]
	for i in 0 ..< len(values) {
		b0 := u32(src[i*4+0])
		b1 := u32(src[i*4+1])
		b2 := u32(src[i*4+2])
		b3 := u32(src[i*4+3])
		bits := b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
		values[i] = transmute(f32)bits
	}
	cursor += body_bytes

	// Skip extra element payloads
	for e in extra_elements {
		skip := int(e.count) * e.bytes_per_element
		if skip > 0 {
			if cursor + skip > len(data) {
				return {}, .Invalid_Data
			}
			cursor += skip
		}
	}

	n := int(num_points)
	cloud.num_points = num_points
	cloud.sh_degree = sh_degree
	cloud.antialiased = false
	cloud.positions = make([]f32, n * 3, allocator)
	cloud.scales = make([]f32, n * 3, allocator)
	cloud.rotations = make([]f32, n * 4, allocator)
	cloud.alphas = make([]f32, n, allocator)
	cloud.colors = make([]f32, n * 3, allocator)
	cloud.sh = make([]f32, n * sh_dim * 3, allocator)

	for i in 0 ..< n {
		base := i * num_fields
		cloud.positions[i*3+0] = values[base+px]
		cloud.positions[i*3+1] = values[base+py]
		cloud.positions[i*3+2] = values[base+pz]

		cloud.scales[i*3+0] = values[base+s0]
		cloud.scales[i*3+1] = values[base+s1]
		cloud.scales[i*3+2] = values[base+s2]

		// xyzw
		cloud.rotations[i*4+0] = values[base+r1]
		cloud.rotations[i*4+1] = values[base+r2]
		cloud.rotations[i*4+2] = values[base+r3]
		cloud.rotations[i*4+3] = values[base+r0]

		cloud.alphas[i] = values[base+a_idx]

		cloud.colors[i*3+0] = values[base+c0]
		cloud.colors[i*3+1] = values[base+c1]
		cloud.colors[i*3+2] = values[base+c2]

		// File is channel-major groups of f_rest_*; store coefficient-major RGB
		sh_base := i * sh_dim * 3
		for j in 0 ..< sh_dim {
			cloud.sh[sh_base+j*3+0] = values[base+sh_idx[j]]
			cloud.sh[sh_base+j*3+1] = values[base+sh_idx[j+sh_dim]]
			cloud.sh[sh_base+j*3+2] = values[base+sh_idx[j+2*sh_dim]]
		}
	}

	convert_coordinates(&cloud, .RDF, options.to)
	return cloud, .None
}

save_splat_to_ply :: proc(
	cloud: ^Gaussian_Cloud,
	options: Pack_Options,
	path: string,
) -> Spz_Error {
	if cloud == nil {
		return .Invalid_Data
	}
	n := i64(cloud.num_points)
	if n < 0 {
		return .Invalid_Data
	}
	if len(cloud.positions) != int(n)*3 ||
	   len(cloud.scales) != int(n)*3 ||
	   len(cloud.rotations) != int(n)*4 ||
	   len(cloud.alphas) != int(n) ||
	   len(cloud.colors) != int(n)*3 {
		return .Invalid_Data
	}
	sh_dim := i64(0)
	if n > 0 {
		sh_dim = i64(len(cloud.sh)) / n / 3
	}
	if sh_dim < 0 || (n > 0 && len(cloud.sh) != int(n)*int(sh_dim)*3) {
		return .Invalid_Data
	}
	D := 17 + sh_dim*3

	c := coordinate_converter(options.from, .RDF, cloud.sh_degree)

	b: strings.Builder
	strings.builder_init(&b)
	defer strings.builder_destroy(&b)

	fmt.sbprintf(&b, "ply\n")
	fmt.sbprintf(&b, "format binary_little_endian 1.0\n")
	fmt.sbprintf(&b, "element vertex %d\n", n)
	fmt.sbprintf(&b, "property float x\n")
	fmt.sbprintf(&b, "property float y\n")
	fmt.sbprintf(&b, "property float z\n")
	fmt.sbprintf(&b, "property float nx\n")
	fmt.sbprintf(&b, "property float ny\n")
	fmt.sbprintf(&b, "property float nz\n")
	fmt.sbprintf(&b, "property float f_dc_0\n")
	fmt.sbprintf(&b, "property float f_dc_1\n")
	fmt.sbprintf(&b, "property float f_dc_2\n")
	for i in 0 ..< sh_dim * 3 {
		fmt.sbprintf(&b, "property float f_rest_%d\n", i)
	}
	fmt.sbprintf(&b, "property float opacity\n")
	fmt.sbprintf(&b, "property float scale_0\n")
	fmt.sbprintf(&b, "property float scale_1\n")
	fmt.sbprintf(&b, "property float scale_2\n")
	fmt.sbprintf(&b, "property float rot_0\n")
	fmt.sbprintf(&b, "property float rot_1\n")
	fmt.sbprintf(&b, "property float rot_2\n")
	fmt.sbprintf(&b, "property float rot_3\n")
	fmt.sbprintf(&b, "end_header\n")

	header := strings.to_string(b)

	out_buf: [dynamic]u8
	defer delete(out_buf)
	append(&out_buf, ..transmute([]u8)header)

	CHUNK :: i64(1_000_000)
	row := make([]f32, int(min(CHUNK, n) * D))
    defer delete(row)

	tmp_sh_len := max(int(sh_dim) * 3, 1)
	tmp_sh := make([]f32, tmp_sh_len)
    defer delete(tmp_sh)

	for start: i64 = 0; start < n; start += CHUNK {
		end := min(start + CHUNK, n)
		row_count := end - start
		needed := int(row_count * D)

		out_idx := 0
		for i in start ..< end {
			i3 := int(i) * 3
			i4 := int(i) * 4

			pos := [3]f32{
				cloud.positions[i3 + 0],
				cloud.positions[i3 + 1],
				cloud.positions[i3 + 2],
			}
			convert_direction(c, pos[:])
			row[out_idx + 0] = pos[0]
			row[out_idx + 1] = pos[1]
			row[out_idx + 2] = pos[2]
			out_idx += 3

			row[out_idx + 0] = 0
			row[out_idx + 1] = 0
			row[out_idx + 2] = 0
			out_idx += 3

			row[out_idx + 0] = cloud.colors[i3 + 0]
			row[out_idx + 1] = cloud.colors[i3 + 1]
			row[out_idx + 2] = cloud.colors[i3 + 2]
			out_idx += 3

			if sh_dim > 0 {
				point_sh_base := int(i) * int(sh_dim) * 3
				sh_n := int(sh_dim) * 3
				for si in 0 ..< sh_n {
					tmp_sh[si] = cloud.sh[point_sh_base + si]
				}
				convert_sh_coefficients(c, tmp_sh[:sh_n], cloud.sh_degree)
				for k in 0 ..< 3 {
					for j in 0 ..< int(sh_dim) {
						row[out_idx + j] = tmp_sh[j * 3 + k]
					}
					out_idx += int(sh_dim)
				}
			}

			row[out_idx] = cloud.alphas[int(i)]
			out_idx += 1

			row[out_idx + 0] = cloud.scales[i3 + 0]
			row[out_idx + 1] = cloud.scales[i3 + 1]
			row[out_idx + 2] = cloud.scales[i3 + 2]
			out_idx += 3

			quat := [4]f32{
				cloud.rotations[i4 + 0],
				cloud.rotations[i4 + 1],
				cloud.rotations[i4 + 2],
				cloud.rotations[i4 + 3],
			}
			convert_quaternion_xyzw(c, quat[:])
			row[out_idx + 0] = quat[3]
			row[out_idx + 1] = quat[0]
			row[out_idx + 2] = quat[1]
			row[out_idx + 3] = quat[2]
			out_idx += 4
		}

		if out_idx != needed {
			return .Invalid_Data
		}

		for fi in 0 ..< needed {
			bits := transmute(u32)row[fi]
			append(&out_buf, u8(bits))
			append(&out_buf, u8(bits >> 8))
			append(&out_buf, u8(bits >> 16))
			append(&out_buf, u8(bits >> 24))
		}
	}

	werr := os.write_entire_file(path, out_buf[:])
	if werr != nil {
		return .Io_Failed
	}
	return .None
}

// --- Helpers -----------------------------------------------------------------

ply_next_header_line :: proc(data: []u8, start: int) -> (line: string, next: int, ok: bool) {
	i := start
	for i < len(data) {
		line_start := i
		for i < len(data) && data[i] != '\n' && data[i] != '\r' {
			i += 1
		}
		raw := string(data[line_start:i])
		if i < len(data) && data[i] == '\r' {
			i += 1
		}
		if i < len(data) && data[i] == '\n' {
			i += 1
		}
		trimmed := strings.trim_space(raw)
		if len(trimmed) == 0 {
			continue
		}
		if strings.has_prefix(trimmed, "comment") {
			continue
		}
		return trimmed, i, true
	}
	return "", i, false
}

ply_property_size :: proc(line: string) -> int {
	if strings.has_prefix(line, "property float ") ||
	   strings.has_prefix(line, "property int ") ||
	   strings.has_prefix(line, "property uint ") {
		return 4
	}
	if strings.has_prefix(line, "property double ") {
		return 8
	}
	if strings.has_prefix(line, "property char ") ||
	   strings.has_prefix(line, "property uchar ") {
		return 1
	}
	if strings.has_prefix(line, "property short ") ||
	   strings.has_prefix(line, "property ushort ") {
		return 2
	}
	return 4
}