package ospz

import "core:math"
import "core:mem"

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

COLOR_SCALE :: f32(0.15)
SQRT_1_2    :: f32(0.707106781186547524401)


// -----------------------------------------------------------------------------
// Spherical Harmonics
// -----------------------------------------------------------------------------

dim_for_degree :: proc(degree: i32) -> i32 {
    switch degree {
    case 0: return 0
    case 1: return 3
    case 2: return 8
    case 3: return 15
    case 4: return 24
    }
    return 0
}


degree_for_dim :: proc(dim: i32) -> i32 {
    if dim < 3  do return 0
    if dim < 8  do return 1
    if dim < 15 do return 2
    if dim < 24 do return 3
    return 4
}


// Number of floating-point components per Gaussian for an attribute.
floats_per_point :: proc(attr: Splat_Attribute, sh_degree: i32) -> int {
    switch attr {
    case .Positions: return 3
    case .Alphas:    return 1
    case .Colors:    return 3
    case .Scales:    return 3
    case .Rotations: return 4
    case .Sh:        return int(dim_for_degree(sh_degree)) * 3
    }
    return 0
}


// Convert an unsigned 8-bit SH coefficient into its normalized value.
unquantize_sh :: proc(x: u8) -> f32 {
    return (f32(x) - 128.0) / 128.0
}


// Inverse sigmoid (logit).
inv_sigmoid :: proc(x: f32) -> f32 {
    return math.log_f32(
        x / (1.0 - x),
        f32(math.E),
    )
}


// -----------------------------------------------------------------------------
// Quaternion Decoding
// -----------------------------------------------------------------------------

// Decode a quaternion stored as its first three components in 8-bit
// fixed-point representation.
//
// The fourth component (w) is reconstructed using the unit-length constraint.
//
// Within-family coordinate conversion:
//     flip_q scales xyz during decoding.
//
// Cross-family coordinate conversion:
//     flip_q is identity and convert_quaternion_xyzw transforms the full quaternion.
//
// Quaternion layout: [x, y, z, w]
unpack_quaternion_first_three :: proc(
    rotation: ^[4]f32,
    r: []u8,
    c: ^Coordinate_Converter,
) {
    xyz := Vec3f{
        f32(r[0]),
        f32(r[1]),
        f32(r[2]),
    }

    xyz = {
        (xyz[0] / 127.5 - 1.0) * c.flip_q[0],
        (xyz[1] / 127.5 - 1.0) * c.flip_q[1],
        (xyz[2] / 127.5 - 1.0) * c.flip_q[2],
    }

    rotation[0] = xyz[0]
    rotation[1] = xyz[1]
    rotation[2] = xyz[2]

    rotation[3] = math.sqrt(
        max(0.0, 1.0 - squared_norm(xyz)),
    )

    convert_quaternion_xyzw(c^, rotation[:])
}


// Decode a quaternion using the "smallest three" encoding.
//
// 2 bits identify the largest component.
// The other three components are stored as 10-bit signed magnitudes.
//
// Components are scaled by 1/sqrt(2).
//
// Quaternion layout: [x, y, z, w]
unpack_quaternion_smallest_three :: proc(
    rotation: ^[4]f32,
    r: []u8,
    c: ^Coordinate_Converter,
) {
    comp := u32(r[0]) |
            (u32(r[1]) << 8) |
            (u32(r[2]) << 16) |
            (u32(r[3]) << 24)

    C_MASK :: (u32(1) << 9) - 1

    i_largest := int(comp >> 30)
    sum_squares: f32 = 0.0

    for i := 4; i > 0; i -= 1 {
        idx := i - 1

        if idx != i_largest {
            mag    := comp & C_MASK
            negbit := (comp >> 9) & 0x1

            comp >>= 10

            rotation[idx] =
                SQRT_1_2 * f32(mag) / f32(C_MASK)

            if negbit == 1 {
                rotation[idx] = -rotation[idx]
            }

            sum_squares += rotation[idx] * rotation[idx]
        }
    }

    // Reconstruct the largest component.
    rotation[i_largest] = math.sqrt(1.0 - sum_squares)

    // Within-family coordinate conversion.
    for i in 0..<3 {
        rotation[i] *= c.flip_q[i]
    }

    // Cross-family coordinate conversion.
    convert_quaternion_xyzw(c^, rotation[:])
}


// -----------------------------------------------------------------------------
// Packed Gaussian Attribute Access
// -----------------------------------------------------------------------------

// Access a Packed_Gaussians attribute buffer by enum tag.
//
// This allows callers to iterate over attributes instead of hard-coding
// the six individual buffer names.
packed_buffer :: proc(
    p: ^Packed_Gaussians,
    attr: Splat_Attribute,
) -> []u8 {
    switch attr {
    case .Positions: return p.positions
    case .Alphas:    return p.alphas
    case .Colors:    return p.colors
    case .Scales:    return p.scales
    case .Rotations: return p.rotations
    case .Sh:        return p.sh
    }

    // Unreachable fallback.
    return p.positions
}


// Number of floating-point components in an attribute buffer.
float_count :: proc(
    p: ^Packed_Gaussians,
    attr: Splat_Attribute,
) -> int {
    return floats_per_point(attr, p.sh_degree) * int(p.num_points)
}


// Release a packed attribute buffer.
//
// Odin requires explicit allocator ownership, unlike std::vector.
// The allocator must be the same allocator used to allocate the buffer.
release_packed :: proc(
    p: ^Packed_Gaussians,
    attr: Splat_Attribute,
    allocator: mem.Allocator,
) {
    switch attr {
    case .Positions:
        delete(p.positions, allocator)
        p.positions = nil

    case .Alphas:
        delete(p.alphas, allocator)
        p.alphas = nil

    case .Colors:
        delete(p.colors, allocator)
        p.colors = nil

    case .Scales:
        delete(p.scales, allocator)
        p.scales = nil

    case .Rotations:
        delete(p.rotations, allocator)
        p.rotations = nil

    case .Sh:
        delete(p.sh, allocator)
        p.sh = nil
    }
}
