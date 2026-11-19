package testospz

import "core:math"
import "core:testing"
import spz "../"

regression_close :: proc(a, b: f32, epsilon: f32 = 1e-5) -> bool {
    return math.abs(a - b) <= epsilon
}

@(test)
test_coordinate_converter_identity_cases :: proc(t: ^testing.T) {
    cases := [2][2]spz.Coordinate_System{
        {.UNSPECIFIED, .RUB},
        {.RDF, .UNSPECIFIED},
    }

    for pair in cases {
        c := spz.coordinate_converter(pair[0], pair[1], 4)
        assert(!c.cross_family)
        for i in 0..<3 {
            assert(c.flip_p[i] == 1)
            assert(c.flip_q[i] == 1)
        }
        for i in 0..<24 {
            assert(c.flip_sh[i] == 1)
        }
    }

    c := spz.coordinate_converter(.RDF, .RDF, 4)
    assert(!c.cross_family)
    assert(c.flip_p == [3]f32{1, 1, 1})
}

@(test)
test_coordinate_within_family_signs :: proc(t: ^testing.T) {
    // LDB -> RDB changes only the X-axis handedness.
    c := spz.coordinate_converter(.LDB, .RDB, 4)

    assert(c.flip_p == [3]f32{-1, 1, 1})
    assert(c.flip_q == [3]f32{1, -1, -1})

    expected_sh := [24]f32{
        1, 1, -1, -1, 1, 1, -1, 1,
        1, -1, 1, 1, -1, 1, -1, -1,
        1, -1, 1, 1, -1, 1, -1, 1,
    }
    assert(c.flip_sh == expected_sh)
}

@(test)
test_cross_family_direction_order :: proc(t: ^testing.T) {
    // These fixtures exercise the two distinct transform orders:
    // backward = flip then -90 degrees around X;
    // forward  = +90 degrees around X then flip.
    backward := spz.coordinate_converter(.RFU, .RUB, 4)
    assert(backward.cross_family)
    assert(backward.backward)

    p_backward := [3]f32{1, 2, 3}
    spz.convert_direction(backward, p_backward[:])
    assert(regression_close(p_backward[0], 1))
    assert(regression_close(p_backward[1], 3))
    assert(regression_close(p_backward[2], -2))

    forward := spz.coordinate_converter(.RUB, .RFU, 4)
    assert(forward.cross_family)
    assert(!forward.backward)

    p_forward := [3]f32{1, 2, 3}
    spz.convert_direction(forward, p_forward[:])
    assert(regression_close(p_forward[0], 1))
    assert(regression_close(p_forward[1], -3))
    assert(regression_close(p_forward[2], 2))
}

@(test)
test_sh_round_trip_each_degree :: proc(t: ^testing.T) {
    for degree in 1..=4 {
        for from_value in u32(1)..=16 {
            for to_value in u32(1)..=16 {
                from := spz.Coordinate_System(from_value)
                to := spz.Coordinate_System(to_value)
                forward := spz.coordinate_converter(from, to, i32(degree))
                backward := spz.coordinate_converter(to, from, i32(degree))

                // Four SH bands, 24 coefficients, RGB-interleaved.
                coefficients: [72]f32
                for i in 0..<72 {
                    coefficients[i] = f32(i + 1) * 0.013 - 0.25
                }
                original := coefficients

                spz.convert_sh_coefficients(forward, coefficients[:], i32(degree))
                spz.convert_sh_coefficients(backward, coefficients[:], i32(degree))

                for i in 0..<72 {
                    assert(regression_close(coefficients[i], original[i], 3e-4))
                }
            }
        }
    }
}

@(test)
test_convert_coordinates_preserves_scales :: proc(t: ^testing.T) {
    positions := []f32{1, 2, 3}
    scales := []f32{0.1, 0.2, 0.3}
    rotations := []f32{0.2, -0.3, 0.4, 0.8}
    original_scales := [3]f32{scales[0], scales[1], scales[2]}

    cloud := spz.Gaussian_Cloud{
        num_points = 1,
        sh_degree = 0,
        positions = positions,
        scales = scales,
        rotations = rotations,
    }

    spz.convert_coordinates(&cloud, .RUB, .RFU)

    for i in 0..<3 {
        assert(cloud.scales[i] == original_scales[i])
    }
}
