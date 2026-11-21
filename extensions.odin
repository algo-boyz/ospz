package ospz

import "core:encoding/endian"
import "core:math"
import "core:mem"
import "core:slice"

// Extension type IDs (vendor << 16 | id)
EXT_ADOBE_SAFE_ORBIT_CAMERA  :: u32(0xADBE0002)
EXT_ADOBE_COORDINATE_SYSTEM  :: u32(0xADBE0003)
EXT_NIANTIC_GEOREFERENCE     :: u32(0x4E530001)
// EXT_NIANTIC_CUSTOM_ATTRIBUTE :: u32(0x4E530002)  // later

// ---- Concrete extension payloads ----------------------------------------

Extension_Safe_Orbit_Camera :: struct {
	elevation_min: f32,   // radians
	elevation_max: f32,
	radius_min:    f32,
}

Extension_Coordinate_System :: struct {
	system: Coordinate_System,   // 1..16; 0 == UNSPECIFIED is invalid for storage
}

Crs_Encoding :: enum u8 {
	Authority_Code = 0,
	Projjson       = 1,
}

Extension_Georeference :: struct {
	crs:          string,          // owned
	crs_encoding: Crs_Encoding,
	origin:       [3]f64,          // meters
	rotation:     [4]f64,          // unit quaternion x,y,z,w
	scale:        f64,             // > 0
	epoch:        f64,             // decimal year; NaN when absent
	has_epoch:    bool,
}

// Tagged union used by both Packed_Gaussians and Gaussian_Cloud
Extension :: union {
	Extension_Safe_Orbit_Camera,
	Extension_Coordinate_System,
	Extension_Georeference,
	// future: Extension_Custom_Attribute, ...
}

// ---- Parse ILV zone -----------------------------------------------------

// Parse the plaintext extension zone [32 .. toc_byte_offset).
// Unknown types are skipped; known ones are decoded.
// Returns owned slice of Extension values (caller frees with release_extensions).
parse_extension_zone :: proc(
	data: []u8,
	toc_byte_offset: u32,
	allocator := context.allocator,
) -> (exts: []Extension, had_unknown: bool, err: Spz_Error) {
	if toc_byte_offset <= 32 || int(toc_byte_offset) > len(data) {
		return nil, false, .None
	}
	zone := data[32:toc_byte_offset]
	out: [dynamic]Extension
	out.allocator = allocator

	i := 0
	for i + 8 <= len(zone) {
		typ := endian.unchecked_get_u32le(zone[i:])
		len_payload := endian.unchecked_get_u32le(zone[i+4:])
		i += 8
		if i + int(len_payload) > len(zone) {
			// truncated zone – stop, treat remaining as unknown
			had_unknown = true
			break
		}
		payload := zone[i:i+int(len_payload)]
		i += int(len_payload)

		switch typ {
		case EXT_ADOBE_SAFE_ORBIT_CAMERA:
			if e, ok := decode_safe_orbit(payload); ok {
				append(&out, e)
			} else {
				had_unknown = true
			}
		case EXT_ADOBE_COORDINATE_SYSTEM:
			if e, ok := decode_coordinate_system(payload); ok {
				append(&out, e)
			} else {
				had_unknown = true
			}
		case EXT_NIANTIC_GEOREFERENCE:
			if e, ok := decode_georeference(payload, allocator); ok {
				append(&out, e)
			} else {
				had_unknown = true
			}
		case:
			had_unknown = true
		}
	}
	return out[:], had_unknown, .None
}

// ---- Decoders ----------------------------------------------------------

decode_safe_orbit :: proc(p: []u8) -> (Extension_Safe_Orbit_Camera, bool) {
	if len(p) != 12 do return {}, false
	return {
		elevation_min = transmute(f32)endian.unchecked_get_u32le(p[0:]),
		elevation_max = transmute(f32)endian.unchecked_get_u32le(p[4:]),
		radius_min    = transmute(f32)endian.unchecked_get_u32le(p[8:]),
	}, true
}

decode_coordinate_system :: proc(p: []u8) -> (Extension_Coordinate_System, bool) {
	if len(p) != 4 do return {}, false
	v := endian.unchecked_get_u32le(p)
	if v == 0 || v > 16 do return {}, false   // UNSPECIFIED or unknown
	return {system = Coordinate_System(v)}, true
}

decode_georeference :: proc(p: []u8, allocator: mem.Allocator) -> (Extension_Georeference, bool) {
	if len(p) < 76 do return {}, false
	ext_version := p[0]
	flags       := p[1]
	crs_len     := endian.unchecked_get_u16le(p[2:])
	if ext_version != 1 do return {}, false
	if 76 + int(crs_len) > len(p) do return {}, false

	crs_enc := Crs_Encoding((flags >> 1) & 0x3)
	if crs_enc > .Projjson do return {}, false

	g: Extension_Georeference
	g.crs_encoding = crs_enc
	g.has_epoch    = (flags & 1) != 0

	// origin (3×f64), rotation (4×f64), scale, epoch – all little-endian
	off := 4
	for i in 0..<3 {
		g.origin[i] = transmute(f64)endian.unchecked_get_u64le(p[off:])
		off += 8
	}
	for i in 0..<4 {
		g.rotation[i] = transmute(f64)endian.unchecked_get_u64le(p[off:])
		off += 8
	}
	g.scale = transmute(f64)endian.unchecked_get_u64le(p[off:]); off += 8
	g.epoch = transmute(f64)endian.unchecked_get_u64le(p[off:]); off += 8

	if !g.has_epoch {
		g.epoch = math.nan_f64()
	} else if math.is_nan(g.epoch) {
		return {}, false
	}

	// crs string
	if crs_len > 0 {
		g.crs = string(p[76:76+crs_len])   // temporary view
		// own it
		g.crs = string(slice.clone(transmute([]u8)g.crs, allocator))
	}

	// basic validation (matches upstream isValid)
	if len(g.crs) == 0 do return {}, false
	if g.scale <= 0 || math.is_nan(g.scale) || math.is_inf(g.scale) {
        return {}, false
    }
	// unit-quaternion check can be added later if desired
	return g, true
}

// ---- Emit helpers -------------------------------------------------------

// Write one ILV record into a dynamic byte buffer.
write_extension_ilv :: proc(buf: ^[dynamic]u8, ext: Extension) {
	switch e in ext {
	case Extension_Safe_Orbit_Camera:
		write_u32le(buf, EXT_ADOBE_SAFE_ORBIT_CAMERA)
		write_u32le(buf, 12)
		write_f32le(buf, e.elevation_min)
		write_f32le(buf, e.elevation_max)
		write_f32le(buf, e.radius_min)

	case Extension_Coordinate_System:
		write_u32le(buf, EXT_ADOBE_COORDINATE_SYSTEM)
		write_u32le(buf, 4)
		write_u32le(buf, u32(e.system))

	case Extension_Georeference:
		// fixed 76 + crs
		crs_bytes := transmute([]u8)e.crs
		crs_len   := u16(min(len(crs_bytes), 0xFFFF))
		payload_len := 76 + int(crs_len)

		write_u32le(buf, EXT_NIANTIC_GEOREFERENCE)
		write_u32le(buf, u32(payload_len))

		append(buf, 1)   // ext_version
		flags: u8 = 0
		if e.has_epoch do flags |= 1
		flags |= u8(e.crs_encoding) << 1
		append(buf, flags)
		write_u16le(buf, crs_len)

		for v in e.origin   do write_f64le(buf, v)
		// normalize rotation lightly (upstream does this)
		q := e.rotation
		n := math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3])
		if abs(n-1.0) < 1e-3 && n > 0 {
			for &c in q do c /= n
		}
		for v in q do write_f64le(buf, v)
		write_f64le(buf, e.scale)
		write_f64le(buf, e.epoch if e.has_epoch else math.nan_f64())
		if crs_len > 0 {
			append(buf, ..crs_bytes[:crs_len])
		}
	}
}

// ---- Coordinate-system resolution ---------------------------------------

// Returns the storage coordinate system recorded by any Coordinate_System
// extension, or .RUB if none / UNSPECIFIED.
get_packed_coordinate_system :: proc(exts: []Extension) -> Coordinate_System {
	for e in exts {
		if cs, ok := e.(Extension_Coordinate_System); ok {
			if cs.system != .UNSPECIFIED {
				return cs.system
			}
		}
	}
	return .RUB
}

// ---- Ownership ------------------------------------------------------------

release_extensions :: proc(exts: []Extension, allocator := context.allocator) {
	for &e in exts {
		if g, ok := &e.(Extension_Georeference); ok {
			delete(g.crs, allocator)
		}
	}
	delete(exts, allocator)
}

// tiny helpers
write_u32le :: proc(b: ^[dynamic]u8, v: u32) {
	tmp: [4]u8
	endian.unchecked_put_u32le(tmp[:], v)
	append(b, ..tmp[:])
}
write_u16le :: proc(b: ^[dynamic]u8, v: u16) {
	tmp: [2]u8
	endian.unchecked_put_u16le(tmp[:], v)
	append(b, ..tmp[:])
}
write_f32le :: proc(b: ^[dynamic]u8, v: f32) {
	write_u32le(b, transmute(u32)v)
}
write_f64le :: proc(b: ^[dynamic]u8, v: f64) {
	tmp: [8]u8
	endian.unchecked_put_u64le(tmp[:], transmute(u64)v)
	append(b, ..tmp[:])
}