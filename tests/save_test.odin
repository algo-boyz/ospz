package testospz

import "core:math"
import "core:testing"
import spz "../"

@(test)
test_save_load_roundtrip_v4 :: proc(t: ^testing.T) {
	for degree in 0 ..= 3 {
		cloud := pack_test_make_cloud(i32(degree))
		defer pack_test_delete_cloud(&cloud)

		opts := spz.pack_options_default()
		opts.from = .UNSPECIFIED
		opts.version = spz.LATEST_SPZ_HEADER_VERSION

		bytes, serr := spz.save_spz_to_bytes(&cloud, opts)
		testing.expect_value(t, serr, spz.Spz_Error.None)
		if serr != .None {
			return
		}
		defer delete(bytes)

		testing.expect(t, len(bytes) >= 32)
		testing.expect(t, bytes[0] == 'N' && bytes[1] == 'G' && bytes[2] == 'S' && bytes[3] == 'P')

		uopts := spz.unpack_options_default()
		uopts.to = .UNSPECIFIED
		loaded, lerr := spz.load_spz_from_bytes(bytes, uopts)
		testing.expect_value(t, lerr, spz.Spz_Error.None)
		if lerr != .None {
			return
		}
		defer spz.release_gaussian_cloud(&loaded)

		testing.expect_value(t, loaded.num_points, cloud.num_points)
		testing.expect_value(t, loaded.sh_degree, cloud.sh_degree)
		testing.expect_value(t, loaded.antialiased, cloud.antialiased)

		for i in 0 ..< 3 {
			testing.expect(t, pack_test_close(loaded.positions[i], cloud.positions[i], 0.0003))
		}
		for i in 0 ..< 3 {
			testing.expect(t, pack_test_close(loaded.scales[i], cloud.scales[i], 0.03126))
		}
		for i in 0 ..< 3 {
			testing.expect(t, pack_test_close(loaded.colors[i], cloud.colors[i], 0.014))
		}
		testing.expect(t, pack_test_close(loaded.alphas[0], cloud.alphas[0], 0.02))

		q_src := spz.normalized_quat({
			cloud.rotations[0], cloud.rotations[1], cloud.rotations[2], cloud.rotations[3],
		})
		q_dst := [4]f32{
			loaded.rotations[0], loaded.rotations[1], loaded.rotations[2], loaded.rotations[3],
		}
		testing.expect(t, pack_test_quaternion_similarity(q_src, q_dst) > 0.999)

		sh_dim := int(spz.dim_for_degree(i32(degree)))
		for coeff in 0 ..< sh_dim {
			for channel in 0 ..< 3 {
				src := cloud.sh[coeff * 3 + channel]
				act := loaded.sh[coeff * 3 + channel]
				tol: f32 = 0.065
				if coeff < 3 do tol = 0.035
				testing.expect(t, pack_test_close(act, src, tol))
			}
		}
	}
}

@(test)
test_save_header_fields :: proc(t: ^testing.T) {
	cloud := pack_test_make_cloud(2)
	defer pack_test_delete_cloud(&cloud)
	cloud.antialiased = true

	opts := spz.pack_options_default()
	opts.version = 4
	bytes, err := spz.save_spz_to_bytes(&cloud, opts)
	testing.expect_value(t, err, spz.Spz_Error.None)
	if err != .None do return
	defer delete(bytes)

	magic := u32(bytes[0]) | (u32(bytes[1]) << 8) | (u32(bytes[2]) << 16) | (u32(bytes[3]) << 24)
	testing.expect_value(t, magic, spz.NGSP_MAGIC)
	version := u32(bytes[4]) | (u32(bytes[5]) << 8) | (u32(bytes[6]) << 16) | (u32(bytes[7]) << 24)
	testing.expect_value(t, version, u32(4))
	np := u32(bytes[8]) | (u32(bytes[9]) << 8) | (u32(bytes[10]) << 16) | (u32(bytes[11]) << 24)
	testing.expect_value(t, np, u32(1))
	testing.expect_value(t, bytes[12], u8(2))
	testing.expect_value(t, bytes[13], u8(12))
	testing.expect(t, (bytes[14] & 0x1) != 0)
	testing.expect(t, (bytes[14] & 0x2) == 0)
	testing.expect_value(t, bytes[15], u8(6))
}

@(test)
test_save_load_packed_only :: proc(t: ^testing.T) {
	cloud := pack_test_make_cloud(1)
	defer pack_test_delete_cloud(&cloud)

	opts := spz.pack_options_default()
	bytes, serr := spz.save_spz_to_bytes(&cloud, opts)
	testing.expect_value(t, serr, spz.Spz_Error.None)
	if serr != .None do return
	defer delete(bytes)

	packed, perr := spz.load_spz_packed_from_bytes(bytes)
	testing.expect_value(t, perr, spz.Spz_Error.None)
	if perr != .None do return
	defer spz.release_packed_gaussians(&packed)

	testing.expect_value(t, packed.num_points, i32(1))
	testing.expect_value(t, packed.sh_degree, i32(1))
	testing.expect(t, packed.uses_quaternion_smallest_three)
	testing.expect(t, spz.packed_sizes_valid(&packed))
	testing.expect_value(t, len(packed.positions), 9)
	testing.expect_value(t, len(packed.alphas), 1)
	testing.expect_value(t, len(packed.colors), 3)
	testing.expect_value(t, len(packed.scales), 3)
	testing.expect_value(t, len(packed.rotations), 4)
	testing.expect_value(t, len(packed.sh), 9)
}