package ospz

import "core:math"

Spz_Error :: enum {
    None,
    Invalid_Header,
    Unsupported_Version,
    Invalid_Data,
    Compression_Failed,
    Decompression_Failed,
    Allocation_Failed,
    Io_Failed,
}

// Half is an IEEE 754 binary16 floating-point representation.
Half :: u16

Vec3f :: [3]f32
Quat4f :: [4]f32


// Converts an IEEE 754 half-precision float to f32.
half_to_float :: proc(h: Half) -> f32 {
    sgn      := (h >> 15) & 0x1
    exponent := (h >> 10) & 0x1f
    mantissa := h & 0x3ff

    sign_mul: f32 = 1.0
    if sgn == 1 {
        sign_mul = -1.0
    }

    if exponent == 0 {
        // Subnormal numbers.
        return sign_mul * math.pow(2.0, -14.0) *
            f32(mantissa) / 1024.0
    }

    if exponent == 31 {
        // Infinity or NaN.
        if mantissa != 0 {
            return math.nan_f32()
        }

        return sign_mul * math.inf_f32()
    }

    // Normal numbers.
    return sign_mul *
        math.pow(2.0, f32(exponent) - 15.0) *
        (1.0 + f32(mantissa) / 1024.0)
}


// Converts f32 to IEEE 754 half-precision float.
float_to_half :: proc(f: f32) -> Half {
    f32_bits := transmute(u32)f

    sign     := i32((f32_bits >> 31) & 0x01)
    exponent := i32((f32_bits >> 23) & 0xff)
    mantissa := i32(f32_bits & 0x7fffff)

    // Handle infinity and NaN.
    if exponent == 0xff {
        if mantissa == 0 {
            return Half((sign << 15) | 0x7c00)
        }

        return Half((sign << 15) | 0x7c01)
    }

    // Exponent outside the half-precision range.
    centered_exp := exponent - 127

    if centered_exp > 15 {
        return Half((sign << 15) | 0x7c00)
    }

    // Normal numbers.
    if centered_exp > -15 {
        return Half(
            (sign << 15) |
            ((centered_exp + 15) << 10) |
            (mantissa >> 13),
        )
    }

    // Subnormal numbers.
    full_mantissa := 0x800000 | mantissa
    shift         := -(centered_exp + 14)
    new_mantissa  := full_mantissa >> shift

    return Half((sign << 15) | (new_mantissa >> 13))
}


// Squared Euclidean norm of a Vec3f.
squared_norm :: proc(a: Vec3f) -> f32 {
    return a[0]*a[0] + a[1]*a[1] + a[2]*a[2]
}


// Euclidean norm of a Vec3f.
norm_vec3 :: proc(a: Vec3f) -> f32 {
    return math.sqrt(squared_norm(a))
}


// Normalize a Vec3f.
normalized_vec3 :: proc(v: Vec3f) -> Vec3f {
    n := norm_vec3(v)

    return {
        v[0] / n,
        v[1] / n,
        v[2] / n,
    }
}


// Euclidean norm of a quaternion.
norm_quat :: proc(q: Quat4f) -> f32 {
    return math.sqrt(
        q[0]*q[0] +
        q[1]*q[1] +
        q[2]*q[2] +
        q[3]*q[3],
    )
}


// Normalize a quaternion.
normalized_quat :: proc(q: Quat4f) -> Quat4f {
    n := norm_quat(q)

    return {
        q[0] / n,
        q[1] / n,
        q[2] / n,
        q[3] / n,
    }
}


// Convert a scaled axis-angle vector to a quaternion.
//
// Quaternion layout: [w, x, y, z]
axis_angle_quat :: proc(scaled_axis: Vec3f) -> Quat4f {
    a0 := scaled_axis[0]
    a1 := scaled_axis[1]
    a2 := scaled_axis[2]

    theta_squared := a0*a0 + a1*a1 + a2*a2

    // Full conversion for non-zero rotation.
    if theta_squared > 0.0 {
        theta      := math.sqrt(theta_squared)
        half_theta := theta * 0.5
        k          := math.sin(half_theta) / theta

        return normalized_quat({
            math.cos(half_theta),
            a0 * k,
            a1 * k,
            a2 * k,
        })
    }

    // Taylor approximation around zero.
    k: f32 = 0.5

    return normalized_quat({
        1.0,
        a0 * k,
        a1 * k,
        a2 * k,
    })
}

