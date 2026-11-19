package testospz

import "core:math"
import "core:testing"
import spz "../"

pack_test_close :: proc(a, b: f32, epsilon: f32) -> bool {
    return math.abs(a - b) <= epsilon
}

pack_test_make_cloud :: proc(degree: i32) -> spz.Gaussian_Cloud {
    sh_dim := int(spz.dim_for_degree(degree))
    cloud := spz.Gaussian_Cloud{
        num_points = 1,
        sh_degree = degree,
        antialiased = true,
        positions = make([]f32, 3),
        scales = make([]f32, 3),
        rotations = make([]f32, 4),
        alphas = make([]f32, 1),
        colors = make([]f32, 3),
        sh = make([]f32, sh_dim * 3),
    }

    cloud.positions[0] = 1.2345
    cloud.positions[1] = -2.3456
    cloud.positions[2] = 3.4567

    cloud.scales[0] = 0.125
    cloud.scales[1] = 0.5
    cloud.scales[2] = 1.25

    cloud.rotations[0] = 0.2
    cloud.rotations[1] = -0.3
    cloud.rotations[2] = 0.4
    cloud.rotations[3] = 0.8

    cloud.alphas[0] = -1.25

    cloud.colors[0] = -0.7
    cloud.colors[1] = 0.0
    cloud.colors[2] = 0.8

    for i in 0..<len(cloud.sh) {
        cloud.sh[i] = f32(i) * 0.017 - 0.21
    }
    return cloud
}

pack_test_delete_cloud :: proc(cloud: ^spz.Gaussian_Cloud) {
    delete(cloud.positions, context.allocator)
    delete(cloud.scales, context.allocator)
    delete(cloud.rotations, context.allocator)
    delete(cloud.alphas, context.allocator)
    delete(cloud.colors, context.allocator)
    delete(cloud.sh, context.allocator)
}

pack_test_delete_packed :: proc(p: ^spz.Packed_Gaussians) {
    spz.release_packed(p, .Positions, context.allocator)
    spz.release_packed(p, .Scales, context.allocator)
    spz.release_packed(p, .Rotations, context.allocator)
    spz.release_packed(p, .Alphas, context.allocator)
    spz.release_packed(p, .Colors, context.allocator)
    spz.release_packed(p, .Sh, context.allocator)
}

pack_test_quaternion_similarity :: proc(a, b: [4]f32) -> f32 {
    // q and -q represent the same orientation.
    dot := a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
    return math.abs(dot)
}

@(test)
test_pack_unpack_attributes_all_sh_degrees :: proc(t: ^testing.T) {
    for degree in 0..=3 {
        cloud := pack_test_make_cloud(i32(degree))
        options := spz.pack_options_default()
        options.from = .UNSPECIFIED
        packed := spz.pack_gaussians(&cloud, options)
        assert(packed.num_points == 1)
        assert(packed.sh_degree == i32(degree))
        assert(spz.packed_sizes_valid(&packed))

        converter := spz.coordinate_converter(.RUB, .UNSPECIFIED, i32(degree))
        decoded := spz.packed_gaussians_unpack(&packed, 0, &converter)

        // Fixed-point positions have a 1/4096-unit quantization step.
        for axis in 0..<3 {
            assert(pack_test_close(decoded.position[axis], cloud.positions[axis], 0.0003))
            // Scale is stored in sixteenths.
            assert(pack_test_close(decoded.scale[axis], cloud.scales[axis], 0.03126))
            // Color is quantized to one byte over its encoded range.
            assert(pack_test_close(decoded.color[axis], cloud.colors[axis], 0.014))
        }

        // Alpha is sigmoid-encoded then quantized to 8 bits; compare in logit space.
        assert(pack_test_close(decoded.alpha, cloud.alphas[0], 0.02))
        assert(pack_test_quaternion_similarity(decoded.rotation, spz.normalized_quat({
            cloud.rotations[0], cloud.rotations[1], cloud.rotations[2], cloud.rotations[3],
        })) > 0.999)

        // SH coefficient 0 uses sh1_bits; all remaining active coefficients
        // use sh_rest_bits. Degree 0 has no stored SH coefficients in this API.
        for coeff in 0..<int(spz.dim_for_degree(i32(degree))) {
            for channel in 0..<3 {
                source := cloud.sh[coeff*3+channel]
                actual: f32
                switch channel {
                case 0: actual = decoded.sh_r[coeff]
                case 1: actual = decoded.sh_g[coeff]
                case 2: actual = decoded.sh_b[coeff]
                }
                tolerance: f32 = 0.065
                if coeff < 3 do tolerance = 0.035
                assert(pack_test_close(actual, source, tolerance))
            }
        }

        pack_test_delete_packed(&packed)
        pack_test_delete_cloud(&cloud)
    }
}

@(test)
test_pack_unpack_coordinate_system_round_trip :: proc(t: ^testing.T) {
    // Exercise the actual pack/unpack path for identity (UNSPECIFIED) and RUB.
    // Existing converter-only tests already cover the broad coordinate matrix.
    targets := [3]spz.Coordinate_System{
        spz.Coordinate_System.UNSPECIFIED,
        spz.Coordinate_System.RUB,
        spz.Coordinate_System.RDF,
    }

    for target in targets {
        cloud := pack_test_make_cloud(3)
        original := pack_test_make_cloud(3)

        options := spz.pack_options_default()
        options.from = target
        packed := spz.pack_gaussians(&cloud, options)
        converter := spz.coordinate_converter(.RUB, target, cloud.sh_degree)
        decoded := spz.packed_gaussians_unpack(&packed, 0, &converter)

        for axis in 0..<3 {
            assert(pack_test_close(decoded.position[axis], original.positions[axis], 0.0003))
        }
        assert(pack_test_quaternion_similarity(decoded.rotation, spz.normalized_quat({
            original.rotations[0], original.rotations[1], original.rotations[2], original.rotations[3],
        })) > 0.999)
        for coeff in 0..<int(spz.dim_for_degree(3)) {
            for channel in 0..<3 {
                source := original.sh[coeff*3+channel]
                actual: f32
                switch channel {
                case 0: actual = decoded.sh_r[coeff]
                case 1: actual = decoded.sh_g[coeff]
                case 2: actual = decoded.sh_b[coeff]
                }
                tolerance: f32 = 0.065
                if coeff < 3 do tolerance = 0.035
                assert(pack_test_close(actual, source, tolerance))
            }
        }

        pack_test_delete_packed(&packed)
        pack_test_delete_cloud(&cloud)
        pack_test_delete_cloud(&original)
    }
}

@(test)
test_pack_versions_select_quaternion_encoding :: proc(t: ^testing.T) {
    expected_smallest_three := [3]bool{false, true, true}
    versions := [3]u32{2, 3, 4}

    for i in 0..<3 {
        cloud := pack_test_make_cloud(0)
        options := spz.pack_options_default()
        options.version = versions[i]
        packed := spz.pack_gaussians(&cloud, options)

        assert(packed.version == versions[i])
        assert(packed.uses_quaternion_smallest_three == expected_smallest_three[i])
        expected_rotation_bytes := 3
        if expected_smallest_three[i] do expected_rotation_bytes = 4
        assert(len(packed.rotations) == expected_rotation_bytes)
        assert(spz.packed_sizes_valid(&packed))

        pack_test_delete_packed(&packed)
        pack_test_delete_cloud(&cloud)
    }
}
