package ospz

import "core:math"

PACKED_FRACTIONAL_BITS :: 12

// Packed_Gaussian is the fixed-size, per-point representation used by SPZ.
// SH channels are stored separately, matching Niantic's C++ struct.
to_u8 :: proc(x: f32) -> u8 {
    rounded := i32(math.round(x))
    if rounded < 0 do rounded = 0
    if rounded > 255 do rounded = 255
    return u8(rounded)
}

quantize_sh :: proc(x: f32, bucket_size: i32) -> u8 {
    q := i32(math.round(x * 128.0)) + 128
    q = ((q + bucket_size / 2) / bucket_size) * bucket_size
    if q < 0 do q = 0
    if q > 255 do q = 255
    return u8(q)
}

sigmoid_f32 :: proc(x: f32) -> f32 {
    return 1.0 / (1.0 + math.exp_f32(-x))
}

// Encodes a normalized XYZW quaternion using the legacy three-byte layout.
pack_quaternion_first_three :: proc(dst: []u8, rotation: []f32, c: ^Coordinate_Converter) {
    q := normalized_quat({rotation[0], rotation[1], rotation[2], rotation[3]})
    convert_quaternion_xyzw(c^, q[:])
    sign: f32 = 127.5
    if q[3] < 0 do sign = -127.5
    for i in 0..<3 {
        dst[i] = to_u8(q[i] * sign + 127.5)
    }
}

// Encodes a quaternion using two bits for the largest component and three
// signed 9-bit magnitudes. The packed word is explicitly little-endian.
pack_quaternion_smallest_three :: proc(dst: []u8, rotation: []f32, c: ^Coordinate_Converter) {
    q := normalized_quat({rotation[0], rotation[1], rotation[2], rotation[3]})
    convert_quaternion_xyzw(c^, q[:])

    largest := 0
    for i in 1..<4 {
        if math.abs(q[i]) > math.abs(q[largest]) do largest = i
    }
    negate := q[largest] < 0
    comp := u32(largest)
    for i in 0..<4 {
        if i != largest {
            negbit := q[i] < 0
            if negate do negbit = !negbit
            mag := u32(math.abs(q[i]) / SQRT_1_2 * 511.0 + 0.5)
            comp = (comp << 10) | (u32(negbit) << 9) | mag
        }
    }
    dst[0] = u8(comp & 0xff)
    dst[1] = u8((comp >> 8) & 0xff)
    dst[2] = u8((comp >> 16) & 0xff)
    dst[3] = u8((comp >> 24) & 0xff)
}

// Decode one packed Gaussian. Coordinate conversion is applied after decoding
// position, rotation and SH, matching PackedGaussian::unpack in load-spz.cc.
unpack :: proc(
    p: Packed_Gaussian,
    uses_float16: bool,
    uses_quaternion_smallest_three: bool,
    fractional_bits: i32,
    c: ^Coordinate_Converter,
) -> Unpacked_Gaussian {
    result: Unpacked_Gaussian

    if uses_float16 {
        for i in 0..<3 {
            bits := u16(p.position[i*2]) | (u16(p.position[i*2+1]) << 8)
            result.position[i] = half_to_float(Half(bits))
        }
    } else {
        divisor := math.pow_f32(2.0, f32(fractional_bits))
        for i in 0..<3 {
            fixed := i32(p.position[i*3]) |
                (i32(p.position[i*3+1]) << 8) |
                (i32(p.position[i*3+2]) << 16)
            if (fixed & 0x800000) != 0 do fixed |= -0x1000000
            result.position[i] = f32(fixed) / divisor
        }
    }
    convert_direction(c^, result.position[:])

    for i in 0..<3 {
        result.scale[i] = f32(p.scale[i]) / 16.0 - 10.0
        result.color[i] = ((f32(p.color[i]) / 255.0) - 0.5) / COLOR_SCALE
    }
    result.alpha = inv_sigmoid(f32(p.alpha) / 255.0)

    rotation_bytes := p.rotation
    if uses_quaternion_smallest_three {
        unpack_quaternion_smallest_three(&result.rotation, rotation_bytes[:], c)
    } else {
        unpack_quaternion_first_three(&result.rotation, rotation_bytes[:], c)
    }

    sh: [72]f32
    for i in 0..<24 {
        sh[i*3+0] = unquantize_sh(p.sh_r[i])
        sh[i*3+1] = unquantize_sh(p.sh_g[i])
        sh[i*3+2] = unquantize_sh(p.sh_b[i])
    }
    convert_sh_coefficients(c^, sh[:], 4)
    for i in 0..<24 {
        result.sh_r[i] = sh[i*3+0]
        result.sh_g[i] = sh[i*3+1]
        result.sh_b[i] = sh[i*3+2]
    }
    return result
}

packed_gaussians_unpack :: proc(
    p: ^Packed_Gaussians,
    index: i32,
    c: ^Coordinate_Converter,
) -> Unpacked_Gaussian {
    result: Unpacked_Gaussian

    packed, ok := packed_gaussian_at(p, index)
    if !ok {
        return result
    }

    return unpack(
        packed,
        packed_uses_float16(p),
        p.uses_quaternion_smallest_three,
        p.fractional_bits,
        c,
    )
}

// Convert a Gaussian_Cloud to SPZ's independent packed attribute streams.
// Invalid dimensions/options return an empty Packed_Gaussians value.
pack_gaussians :: proc(g: ^Gaussian_Cloud, options: Pack_Options) -> Packed_Gaussians {
    result: Packed_Gaussians
    sh_dim := int(dim_for_degree(g.sh_degree))
    n := int(g.num_points)
    if n < 0 || g.sh_degree < 0 || g.sh_degree > SH_MAX_DEGREE do return result

    if len(g.positions) != n*3 || len(g.scales) != n*3 || len(g.rotations) != n*4 || len(g.alphas) != n || len(g.colors) != n*3 || len(g.sh) != n*sh_dim*3 do return result

    if options.sh1_bits < 1 || options.sh1_bits > 8 || options.sh_rest_bits < 1 || options.sh_rest_bits > 8 do return result 

    result.version = options.version
    result.num_points = g.num_points
    result.sh_degree = g.sh_degree
    result.fractional_bits = PACKED_FRACTIONAL_BITS
    result.antialiased = g.antialiased
    result.uses_quaternion_smallest_three = options.version >= MIN_SMALLEST_THREE_QUATERNIONS_VERSION
    rotation_bytes := 4
    if !result.uses_quaternion_smallest_three do rotation_bytes = 3

    result.positions = make([]u8, n*9)
    result.scales = make([]u8, n*3)
    result.rotations = make([]u8, n*rotation_bytes)
    result.alphas = make([]u8, n)
    result.colors = make([]u8, n*3)
    result.sh = make([]u8, n*sh_dim*3)

    c := coordinate_converter(options.from, .RUB, g.sh_degree)
    coordinate_scale := math.pow_f32(2.0, f32(result.fractional_bits))
    for point in 0..<n {
        pos := Vec3f{
            g.positions[point*3+0] * coordinate_scale,
            g.positions[point*3+1] * coordinate_scale,
            g.positions[point*3+2] * coordinate_scale,
        }
        convert_direction(c, pos[:])
        for axis in 0..<3 {
            fixed := i32(math.round(pos[axis]))
            offset := (point*3+axis)*3
            result.positions[offset+0] = u8(fixed & 0xff)
            result.positions[offset+1] = u8((fixed >> 8) & 0xff)
            result.positions[offset+2] = u8((fixed >> 16) & 0xff)
            result.scales[point*3+axis] = to_u8((g.scales[point*3+axis]+10.0)*16.0)
            result.colors[point*3+axis] = to_u8(g.colors[point*3+axis]*(COLOR_SCALE*255.0)+127.5)
        }
        if result.uses_quaternion_smallest_three {
            pack_quaternion_smallest_three(result.rotations[point*4:point*4+4], g.rotations[point*4:point*4+4], &c)
        } else {
            pack_quaternion_first_three(result.rotations[point*3:point*3+3], g.rotations[point*4:point*4+4], &c)
        }
        result.alphas[point] = to_u8(sigmoid_f32(g.alphas[point])*255.0)

        if sh_dim > 0 {
            point_sh := make([]f32, sh_dim*3)
            for coeff in 0..<sh_dim {
                for channel in 0..<3 {
                    point_sh[coeff*3+channel] = g.sh[point*sh_dim*3+coeff*3+channel]
                }
            }
            convert_sh_coefficients(c, point_sh, g.sh_degree)
            for coeff in 0..<sh_dim {
                for channel in 0..<3 {
                    value := point_sh[coeff*3+channel]
                    bucket := 1 << u32(8-int(options.sh_rest_bits))
                    if coeff < 3 do bucket = 1 << u32(8-int(options.sh1_bits))
                    result.sh[point*sh_dim*3+coeff*3+channel] = quantize_sh(value, i32(bucket))
                }
            }
            delete(point_sh)
        }
    }
    return result
}
