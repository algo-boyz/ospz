package ospz

import "core:math"

SH_MAX_DEGREE :: 4

Coordinate_System :: enum u32 {
    UNSPECIFIED = 0,
    LDB         = 1,  // Left Down Back
    RDB         = 2,  // Right Down Back
    LUB         = 3,  // Left Up Back
    RUB         = 4,  // Right Up Back
    LDF         = 5,  // Left Down Front
    RDF         = 6,  // Right Down Front
    LUF         = 7,  // Left Up Front
    RUF         = 8,  // Right Up Front
    LFD         = 9,  // Left Front Down
    RFD         = 10, // Right Front Down
    LFU         = 11, // Left Front Up
    RFU         = 12, // Right Front Up
    LBD         = 13, // Left Back Down
    RBD         = 14, // Right Back Down
    LBU         = 15, // Left Back Up
    RBU         = 16, // Right Back Up
}

Gaussian_Cloud :: struct {
    num_points: i32,
    sh_degree:  i32,
    antialiased: bool,

    positions: []f32,
    scales:    []f32,
    rotations: []f32,
    alphas:    []f32,
    colors:    []f32,
    sh:        []f32,
}

Coordinate_Converter :: struct {
    flip_p:       [3]f32,
    flip_q:       [3]f32,
    flip_sh:      [24]f32,

    // Cross-family conversions combine a ±90° X rotation with
    // the within-family sign flips.
    cross_family: bool,
    backward:     bool,

    cross_flip_p:  [3]f32,
    cross_flip_q:  [3]f32,
    cross_flip_sh: [24]f32,
}

identity_coordinate_converter :: proc() -> Coordinate_Converter {
    result := Coordinate_Converter{
        flip_p       = {1, 1, 1},
        flip_q       = {1, 1, 1},
        cross_family = false,
        backward     = false,
    }

    for i in 0..<24 {
        result.flip_sh[i] = 1
    }

    return result
}

axes_match :: proc(a, b: Coordinate_System) -> [3]bool {
    a_num := i32(a) - 1
    b_num := i32(b) - 1

    if a_num < 0 || b_num < 0 {
        return {true, true, true}
    }

    return {
        ((a_num >> 0) & 1) == ((b_num >> 0) & 1),
        ((a_num >> 1) & 1) == ((b_num >> 1) & 1),
        ((a_num >> 2) & 1) == ((b_num >> 2) & 1),
    }
}

need_rotation :: proc(a, b: Coordinate_System) -> bool {
    a_num := i32(a) - 1
    b_num := i32(b) - 1

    if a_num < 0 || b_num < 0 {
        return false
    }

    return ((a_num >> 3) & 1) != ((b_num >> 3) & 1)
}

Gaussian_Cloud :: struct {
    num_points: i32,
    sh_degree:  i32,
    antialiased: bool,

    positions: []f32,
    scales:    []f32,
    rotations: []f32,
    alphas:    []f32,
    colors:    []f32,
    sh:        []f32,
}

convert_coordinates :: proc(
    cloud: ^Gaussian_Cloud,
    from, to: Coordinate_System,
) {
    if cloud.num_points == 0 {
        return
    }

    c := coordinate_converter(from, to, cloud.sh_degree)

    // Positions: XYZ.
    for i := 0; i + 3 <= len(cloud.positions); i += 3 {
        convert_direction(c, cloud.positions[i:i+3])
    }

    // Rotations: XYZW.
    if len(cloud.rotations) >= 4 {
        for i := 0; i + 4 <= len(cloud.rotations); i += 4 {
            convert_quaternion_xyzw(c, cloud.rotations[i:i+4])
        }
    }

    // SH: coefficient-major, RGB-interleaved.
    num_coeffs := len(cloud.sh) / 3
    num_coeffs_per_point := num_coeffs / int(cloud.num_points)

    if num_coeffs_per_point > 0 {
        for coeff_base := 0; coeff_base < num_coeffs;
             coeff_base += num_coeffs_per_point {

            point_sh := cloud.sh[
                coeff_base*3:(coeff_base+num_coeffs_per_point)*3
            ]

            convert_sh_coefficients(
                c,
                point_sh,
                cloud.sh_degree,
            )
        }
    }
}

is_identity_position_transform :: proc(c: Coordinate_Converter) -> bool {
    return !c.cross_family &&
        c.flip_p[0] == 1 &&
        c.flip_p[1] == 1 &&
        c.flip_p[2] == 1
}

// Mutates an XYZ position or direction in place.
convert_direction :: proc(c: Coordinate_Converter, xyz: []f32) {
    assert(len(xyz) >= 3)

    if !c.cross_family {
        xyz[0] *= c.flip_p[0]
        xyz[1] *= c.flip_p[1]
        xyz[2] *= c.flip_p[2]
        return
    }

    if c.backward {
        xyz[0] *= c.cross_flip_p[0]
        y := xyz[1] * c.cross_flip_p[1]
        z := xyz[2] * c.cross_flip_p[2]
        xyz[1] = z
        xyz[2] = -y
    } else {
        y, z := xyz[1], xyz[2]
        xyz[1] = -z
        xyz[2] = y

        xyz[0] *= c.cross_flip_p[0]
        xyz[1] *= c.cross_flip_p[1]
        xyz[2] *= c.cross_flip_p[2]
    }
}

// Mutates an XYZW quaternion in place.
// This is the layout used by GaussianCloud.rotations, not Quat4f (WXYZ).
convert_quaternion_xyzw :: proc(c: Coordinate_Converter, q: []f32) {
    assert(len(q) >= 4)

    if !c.cross_family {
        q[0] *= c.flip_q[0]
        q[1] *= c.flip_q[1]
        q[2] *= c.flip_q[2]
        return
    }

    x, y, z, w := q[0], q[1], q[2], q[3]
    s := f32(0.7071067811865476)

    if c.backward {
        x *= c.cross_flip_q[0]
        y *= c.cross_flip_q[1]
        z *= c.cross_flip_q[2]

        q[0] = s * (-w + x)
        q[1] = s * (y + z)
        q[2] = s * (-y + z)
        q[3] = s * (w + x)
    } else {
        q[0] = s * (w + x) * c.cross_flip_q[0]
        q[1] = s * (y - z) * c.cross_flip_q[1]
        q[2] = s * (y + z) * c.cross_flip_q[2]
        q[3] = s * (w - x)
    }
}

rotate_sh_band_plus_x :: proc(band: i32, p: []f32) {
    switch band {
    case 0:
        t0, t1, t2 := p[0], p[1], p[2]
        p[0] = t1
        p[1] = -t0
        p[2] = t2

    case 1:
        s := [5]f32{p[0], p[1], p[2], p[3], p[4]}
        s3 := f32(1.7320508075688772)

        p[0] = s[3]
        p[1] = -s[1]
        p[2] = -0.5 * s[2] - (s3 / 2) * s[4]
        p[3] = -s[0]
        p[4] = -(s3 / 2) * s[2] + 0.5 * s[4]

    case 2:
        s := [7]f32{p[0], p[1], p[2], p[3], p[4], p[5], p[6]}
        s15 := f32(3.872983346207417)

        p[0] = -f32(math.sqrt(5.0 / 8.0)) * s[3] + f32(math.sqrt(3.0 / 8.0)) * s[5]
        p[1] = -s[1]
        p[2] = -f32(math.sqrt(3.0 / 8.0)) * s[3] - f32(math.sqrt(5.0 / 8.0)) * s[5]
        p[3] = f32(math.sqrt(5.0 / 8.0)) * s[0] + f32(math.sqrt(3.0 / 8.0)) * s[2]
        p[4] = -0.25 * s[4] - (s15 / 4) * s[6]
        p[5] = -f32(math.sqrt(3.0 / 8.0)) * s[0] + f32(math.sqrt(5.0 / 8.0)) * s[2]
        p[6] = -(s15 / 4) * s[4] + 0.25 * s[6]

    case 3:
        s := [9]f32{p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]}
        s2  := f32(1.4142135623730951)
        s5  := f32(2.23606797749979)
        s7  := f32(2.6457513105737893)
        s14 := f32(3.7416573867739413)
        s35 := f32(5.916079783099616)

        p[0] = -(s14 / 4) * s[5] + (s2 / 4) * s[7]
        p[1] = -0.75 * s[1] + (s7 / 4) * s[3]
        p[2] = -(s2 / 4) * s[5] - (s14 / 4) * s[7]
        p[3] = (s7 / 4) * s[1] + 0.75 * s[3]
        p[4] = (3.0 / 8.0) * s[4] + (s5 / 4) * s[6] + (s35 / 8) * s[8]
        p[5] = (s14 / 4) * s[0] + (s2 / 4) * s[2]
        p[6] = (s5 / 4) * s[4] + 0.5 * s[6] - (s7 / 4) * s[8]
        p[7] = -(s2 / 4) * s[0] + (s14 / 4) * s[2]
        p[8] = (s35 / 8) * s[4] - (s7 / 4) * s[6] + 0.125 * s[8]
    }
}

rotate_sh_band_minus_x :: proc(band: i32, p: []f32) {
    switch band {
    case 0:
        t0, t1, t2 := p[0], p[1], p[2]
        p[0] = -t1
        p[1] = t0
        p[2] = t2

    case 1:
        s := [5]f32{p[0], p[1], p[2], p[3], p[4]}
        s3 := f32(1.7320508075688772)

        p[0] = -s[3]
        p[1] = -s[1]
        p[2] = -0.5 * s[2] - (s3 / 2) * s[4]
        p[3] = s[0]
        p[4] = -(s3 / 2) * s[2] + 0.5 * s[4]

    case 2:
        s := [7]f32{p[0], p[1], p[2], p[3], p[4], p[5], p[6]}
        s15 := f32(3.872983346207417)

        p[0] = f32(math.sqrt(5.0 / 8.0)) * s[3] - f32(math.sqrt(3.0 / 8.0)) * s[5]
        p[1] = -s[1]
        p[2] = f32(math.sqrt(3.0 / 8.0)) * s[3] + f32(math.sqrt(5.0 / 8.0)) * s[5]
        p[3] = -f32(math.sqrt(5.0 / 8.0)) * s[0] - f32(math.sqrt(3.0 / 8.0)) * s[2]
        p[4] = -0.25 * s[4] - (s15 / 4) * s[6]
        p[5] = f32(math.sqrt(3.0 / 8.0)) * s[0] - f32(math.sqrt(5.0 / 8.0)) * s[2]
        p[6] = -(s15 / 4) * s[4] + 0.25 * s[6]

    case 3:
        s := [9]f32{p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]}
        s2  := f32(1.4142135623730951)
        s5  := f32(2.23606797749979)
        s7  := f32(2.6457513105737893)
        s14 := f32(3.7416573867739413)
        s35 := f32(5.916079783099616)

        p[0] = (s14 / 4) * s[5] - (s2 / 4) * s[7]
        p[1] = -0.75 * s[1] + (s7 / 4) * s[3]
        p[2] = (s2 / 4) * s[5] + (s14 / 4) * s[7]
        p[3] = (s7 / 4) * s[1] + 0.75 * s[3]
        p[4] = (3.0 / 8.0) * s[4] + (s5 / 4) * s[6] + (s35 / 8) * s[8]
        p[5] = -(s14 / 4) * s[0] - (s2 / 4) * s[2]
        p[6] = (s5 / 4) * s[4] + 0.5 * s[6] - (s7 / 4) * s[8]
        p[7] = (s2 / 4) * s[0] - (s14 / 4) * s[2]
        p[8] = (s35 / 8) * s[4] - (s7 / 4) * s[6] + 0.125 * s[8]
    }
}

convert_sh_coefficients :: proc(
    c: Coordinate_Converter,
    coeffs: []f32,
    sh_degree: i32,
) {
    degree := min(sh_degree, SH_MAX_DEGREE)
    available_coeffs := len(coeffs) / 3

    flip_sh := c.flip_sh
    if c.cross_family {
        flip_sh = c.cross_flip_sh
    }

    for band in 0..<degree {
        band_start := band * (band + 2)
        band_size := 2 * band + 3

        if band_start + band_size > available_coeffs {
            break
        }

        for channel in 0..<3 {
            tmp: [9]f32

            for k in 0..<band_size {
                tmp[k] = coeffs[(band_start + k) * 3 + channel]
            }

            if c.cross_family {
                if c.backward {
                    for k in 0..<band_size {
                        tmp[k] *= flip_sh[band_start + k]
                    }
                    rotate_sh_band_minus_x(band, tmp[:band_size])
                } else {
                    rotate_sh_band_plus_x(band, tmp[:band_size])
                    for k in 0..<band_size {
                        tmp[k] *= flip_sh[band_start + k]
                    }
                }
            } else {
                for k in 0..<band_size {
                    tmp[k] *= flip_sh[band_start + k]
                }
            }

            for k in 0..<band_size {
                coeffs[(band_start + k) * 3 + channel] = tmp[k]
            }
        }
    }
}