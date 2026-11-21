package testospz

import "core:math"
import "core:slice"
import "core:testing"
import spz "../"

@(test)
test_extension_safe_orbit_roundtrip :: proc(t: ^testing.T) {
	src := spz.Extension_Safe_Orbit_Camera{
		elevation_min = -0.5,
		elevation_max =  1.2,
		radius_min    =  3.5,
	}

	buf: [dynamic]u8
	spz.write_extension_ilv(&buf, src)
	defer delete(buf)

	// Fake a minimal zone: type+len already written by write_extension_ilv
	exts, had_unknown, err := spz.parse_extension_zone(
		// pad 32-byte header so zone starts at offset 32
		make_zone_with_header(buf[:]),
		u32(32 + len(buf)),
	)
	testing.expect_value(t, err, spz.Spz_Error.None)
	testing.expect(t, !had_unknown)
	testing.expect_value(t, len(exts), 1)
	defer spz.release_extensions(exts)

	got, ok := exts[0].(spz.Extension_Safe_Orbit_Camera)
	testing.expect(t, ok)
	testing.expect(t, pack_test_close(got.elevation_min, src.elevation_min, 1e-6))
	testing.expect(t, pack_test_close(got.elevation_max, src.elevation_max, 1e-6))
	testing.expect(t, pack_test_close(got.radius_min,    src.radius_min,    1e-6))
}

@(test)
test_extension_coordinate_system_roundtrip :: proc(t: ^testing.T) {
	src := spz.Extension_Coordinate_System{system = .RDF}

	buf: [dynamic]u8
	spz.write_extension_ilv(&buf, src)
	defer delete(buf)

	exts, had_unknown, err := spz.parse_extension_zone(
		make_zone_with_header(buf[:]),
		u32(32 + len(buf)),
	)
	testing.expect_value(t, err, spz.Spz_Error.None)
	testing.expect(t, !had_unknown)
	testing.expect_value(t, len(exts), 1)
	defer spz.release_extensions(exts)

	got, ok := exts[0].(spz.Extension_Coordinate_System)
	testing.expect(t, ok)
	testing.expect_value(t, got.system, spz.Coordinate_System.RDF)

	// Resolution helper
	cs := spz.get_packed_coordinate_system(exts)
	testing.expect_value(t, cs, spz.Coordinate_System.RDF)
	testing.expect_value(t, spz.get_packed_coordinate_system(nil), spz.Coordinate_System.RUB)
}

@(test)
test_extension_georeference_roundtrip :: proc(t: ^testing.T) {
	src := spz.Extension_Georeference{
		crs          = "EPSG:4978",
		crs_encoding = .Authority_Code,
		origin       = {1.0, 2.0, 3.0},
		rotation     = {0, 0, 0, 1},
		scale        = 1.5,
		epoch        = 2024.5,
		has_epoch    = true,
	}

	buf: [dynamic]u8
	spz.write_extension_ilv(&buf, src)
	defer delete(buf)

	exts, had_unknown, err := spz.parse_extension_zone(
		make_zone_with_header(buf[:]),
		u32(32 + len(buf)),
	)
	testing.expect_value(t, err, spz.Spz_Error.None)
	testing.expect(t, !had_unknown)
	testing.expect_value(t, len(exts), 1)
	defer spz.release_extensions(exts)

	got, ok := exts[0].(spz.Extension_Georeference)
	testing.expect(t, ok)
	testing.expect_value(t, got.crs, "EPSG:4978")
	testing.expect_value(t, got.crs_encoding, spz.Crs_Encoding.Authority_Code)
	testing.expect(t, got.has_epoch)
	testing.expect(t, math.abs(got.scale - 1.5) < 1e-12)
	testing.expect(t, math.abs(got.epoch - 2024.5) < 1e-12)
	for i in 0..<3 {
		testing.expect(t, math.abs(got.origin[i] - src.origin[i]) < 1e-12)
	}
}

@(test)
test_extension_unknown_skipped :: proc(t: ^testing.T) {
	// Unknown type 0xDEAD0001, 4-byte payload
	zone_payload: [dynamic]u8
	spz.write_u32le(&zone_payload, 0xDEAD0001)
	spz.write_u32le(&zone_payload, 4)
	append(&zone_payload, 0xAA, 0xBB, 0xCC, 0xDD)
	defer delete(zone_payload)

	// Also one known record after it
	known := spz.Extension_Coordinate_System{system = .RUF}
	spz.write_extension_ilv(&zone_payload, known)

	exts, had_unknown, err := spz.parse_extension_zone(
		make_zone_with_header(zone_payload[:]),
		u32(32 + len(zone_payload)),
	)
	testing.expect_value(t, err, spz.Spz_Error.None)
	testing.expect(t, had_unknown)
	testing.expect_value(t, len(exts), 1) // only the known one
	defer spz.release_extensions(exts)

	got, ok := exts[0].(spz.Extension_Coordinate_System)
	testing.expect(t, ok)
	testing.expect_value(t, got.system, spz.Coordinate_System.RUF)
}

@(test)
test_extension_save_load_packed_roundtrip :: proc(t: ^testing.T) {
	// Build a minimal valid packed cloud + attach extensions
	cloud := pack_test_make_cloud(0)
	defer pack_test_delete_cloud(&cloud)

	opts := spz.pack_options_default()
	opts.from = .UNSPECIFIED
	packed := spz.pack_gaussians(&cloud, opts)
	// attach extensions after pack
	exts := make([]spz.Extension, 2)
	exts[0] = spz.Extension_Coordinate_System{system = .RDF}
	exts[1] = spz.Extension_Safe_Orbit_Camera{
		elevation_min = -0.25,
		elevation_max =  0.75,
		radius_min    =  2.0,
	}
	packed.extensions = exts

	bytes, serr := spz.save_packed_to_ngsp(&packed, 4)
	testing.expect_value(t, serr, spz.Spz_Error.None)
	if serr != .None do return
	defer delete(bytes)

	// FLAG_HAS_EXTENSIONS must be set
	testing.expect(t, (bytes[14] & 0x2) != 0)
	// toc_byte_offset > 32
	toc := u32(bytes[16]) | (u32(bytes[17]) << 8) | (u32(bytes[18]) << 16) | (u32(bytes[19]) << 24)
	testing.expect(t, toc > 32)

	loaded, lerr := spz.load_spz_packed_from_bytes(bytes)
	testing.expect_value(t, lerr, spz.Spz_Error.None)
	if lerr != .None do return
	defer spz.release_packed_gaussians(&loaded)

	testing.expect(t, !loaded.had_skipped_extensions)
	testing.expect_value(t, len(loaded.extensions), 2)

	cs := spz.get_packed_coordinate_system(loaded.extensions)
	testing.expect_value(t, cs, spz.Coordinate_System.RDF)

	found_orbit := false
	for e in loaded.extensions {
		if o, ok := e.(spz.Extension_Safe_Orbit_Camera); ok {
			found_orbit = true
			testing.expect(t, pack_test_close(o.radius_min, 2.0, 1e-6))
		}
	}
	testing.expect(t, found_orbit)

	// cleanup attached slice on the original packed
	delete(exts)
	pack_test_delete_packed(&packed)
}

// ---- helpers ---------------------------------------------------------------

// Build a fake file prefix: 32 zero bytes (header) + zone payload.
// parse_extension_zone only reads [32 .. toc).
make_zone_with_header :: proc(zone: []u8) -> []u8 {
	out := make([]u8, 32 + len(zone))
	copy(out[32:], zone)
	return out
}