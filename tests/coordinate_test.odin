package testospz

import "core:math"
import ospz "../"

test_close :: proc(a, b: f32, epsilon: f32 = 1e-5) -> bool {
    return math.abs(a - b) <= epsilon
}

test_rub_to_rdf :: proc() {
    c := spz.coordinate_converter(.RUB, .RDF, 4)

    position := [3]f32{1, 2, 3}
    spz.convert_direction(c, position[:])

    assert(test_close(position[0], 1))
    assert(test_close(position[1], -2))
    assert(test_close(position[2], -3))
}

test_coordinate_round_trip :: proc() {
    // Exercise every defined coordinate-system pair.
    for from_value in u32(1)..=16 {
        for to_value in u32(1)..=16 {
            from := spz.Coordinate_System(from_value)
            to := spz.Coordinate_System(to_value)

            forward := spz.coordinate_converter(from, to, 4)
            backward := spz.coordinate_converter(to, from, 4)

            position := [3]f32{1.25, -2.5, 3.75}
            original_position := position

            spz.convert_direction(forward, position[:])
            spz.convert_direction(backward, position[:])

            for i in 0..<3 {
                assert(test_close(position[i], original_position[i]))
            }

            quaternion := [4]f32{0.2, -0.3, 0.4, 0.8}
            original_quaternion := quaternion

            spz.convert_quaternion_xyzw(forward, quaternion[:])
            spz.convert_quaternion_xyzw(backward, quaternion[:])

            for i in 0..<4 {
                assert(test_close(quaternion[i], original_quaternion[i]))
            }
        }
    }
}

test_sh_round_trip :: proc() {
    for from_value in u32(1)..=16 {
        for to_value in u32(1)..=16 {
            forward := spz.coordinate_converter(
                spz.Coordinate_System(from_value),
                spz.Coordinate_System(to_value),
                4,
            )

            backward := spz.coordinate_converter(
                spz.Coordinate_System(to_value),
                spz.Coordinate_System(from_value),
                4,
            )

            // 24 coefficients × 3 RGB channels.
            sh: [72]f32
            for i in 0..<72 {
                sh[i] = f32(i + 1) * 0.013
            }
            original := sh

            spz.convert_sh_coefficients(forward, sh[:], 4)
            spz.convert_sh_coefficients(backward, sh[:], 4)

            for i in 0..<72 {
                assert(test_close(sh[i], original[i], 2e-4))
            }
        }
    }
}