package ospz

// SPZ packed-format constants. Keep these in sync with Niantic's load-spz.h.
SH_MAX_COEFFS :: 24
DEFAULT_SH1_BITS :: 5
DEFAULT_SH_REST_BITS :: 4
LATEST_SPZ_HEADER_VERSION :: 4
MIN_ZSTD_SPZ_HEADER_VERSION :: 4
MIN_SMALLEST_THREE_QUATERNIONS_VERSION :: 3
NGSP_MAGIC :: u32(0x5053474e)

Splat_Attribute :: enum {
    Positions,
    Alphas,
    Colors,
    Scales,
    Rotations,
    Sh,
}

// Packed attributes use independent byte streams. Do not interleave SH channels here:
// the packed SPZ stream is coefficient-major RGB, while Packed_Gaussian stores channels
// in separate fixed-size arrays for convenient point access.
Packed_Gaussians :: struct {
    version: u32,
    num_points: i32,
    sh_degree: i32,
    fractional_bits: i32,
    antialiased: bool,
    uses_quaternion_smallest_three: bool,
    had_skipped_extensions: bool,

    positions: []u8,
    scales:    []u8,
    rotations: []u8,
    alphas:    []u8,
    colors:    []u8,
    sh:        []u8,
}

Packed_Gaussian :: struct {
    position: [9]u8,
    rotation: [4]u8,
    scale:    [3]u8,
    color:    [3]u8,
    alpha:    u8,
    sh_r:     [SH_MAX_COEFFS]u8,
    sh_g:     [SH_MAX_COEFFS]u8,
    sh_b:     [SH_MAX_COEFFS]u8,
}

Unpacked_Gaussian :: struct {
    position: [3]f32,
    rotation: [4]f32,
    scale:    [3]f32,
    color:    [3]f32,
    alpha:    f32,
    sh_r:     [SH_MAX_COEFFS]f32,
    sh_g:     [SH_MAX_COEFFS]f32,
    sh_b:     [SH_MAX_COEFFS]f32,
}

Pack_Options :: struct {
    version:    u32,
    from:       Coordinate_System,
    sh1_bits:   u8,
    sh_rest_bits: u8,
}

Unpack_Options :: struct {
    to: Coordinate_System,
}

pack_options_default :: proc() -> Pack_Options {
    return {
        version = LATEST_SPZ_HEADER_VERSION,
        from = .UNSPECIFIED,
        sh1_bits = DEFAULT_SH1_BITS,
        sh_rest_bits = DEFAULT_SH_REST_BITS,
    }
}

unpack_options_default :: proc() -> Unpack_Options {
    return {to = .UNSPECIFIED}
}

// True for the unreleased legacy representation with three binary16 coordinates per point.
packed_uses_float16 :: proc(p: ^Packed_Gaussians) -> bool {
    return len(p.positions) == int(p.num_points) * 3 * 2
}

// Number of decoded floating-point components in an attribute stream.
packed_float_count :: proc(p: ^Packed_Gaussians, attr: Splat_Attribute) -> int {
    return floats_per_point(attr, p.sh_degree) * int(p.num_points)
}

// Validate stream sizes before indexing or allocating decoded data.
packed_sizes_valid :: proc(p: ^Packed_Gaussians) -> bool {
    if p.num_points < 0 || p.sh_degree < 0 || p.sh_degree > SH_MAX_DEGREE {
        return false
    }

    n := int(p.num_points)
    position_bytes := 9
    if packed_uses_float16(p) {
        position_bytes = 6
    }
    rotation_bytes := 4
    if !p.uses_quaternion_smallest_three {
        rotation_bytes = 3
    }

    return len(p.positions) == n * position_bytes &&
        len(p.scales) == n * 3 &&
        len(p.rotations) == n * rotation_bytes &&
        len(p.alphas) == n &&
        len(p.colors) == n * 3 &&
        len(p.sh) == n * int(dim_for_degree(p.sh_degree)) * 3
}

// Copy one point from the packed, non-interleaved streams into the fixed-size
// per-Gaussian representation. Unused SH coefficients are neutral (128).
packed_gaussian_at :: proc(p: ^Packed_Gaussians, point_index: i32) -> (result: Packed_Gaussian, ok: bool) {
    if point_index < 0 || point_index >= p.num_points || !packed_sizes_valid(p) {
        return result, false
    }

    i := int(point_index)
    position_bytes := 9
    if packed_uses_float16(p) {
        position_bytes = 6
    }
    for j in 0..<position_bytes {
        result.position[j] = p.positions[i * position_bytes + j]
    }

    for j in 0..<3 {
        result.scale[j] = p.scales[i * 3 + j]
        result.color[j] = p.colors[i * 3 + j]
    }

    rotation_bytes := 4
    if !p.uses_quaternion_smallest_three {
        rotation_bytes = 3
    }
    for j in 0..<rotation_bytes {
        result.rotation[j] = p.rotations[i * rotation_bytes + j]
    }
    result.alpha = p.alphas[i]

    for j in 0..<SH_MAX_COEFFS {
        result.sh_r[j] = 128
        result.sh_g[j] = 128
        result.sh_b[j] = 128
    }

    sh_dim := int(dim_for_degree(p.sh_degree))
    sh_base := i * sh_dim * 3
    for j in 0..<sh_dim {
        result.sh_r[j] = p.sh[sh_base + j * 3 + 0]
        result.sh_g[j] = p.sh[sh_base + j * 3 + 1]
        result.sh_b[j] = p.sh[sh_base + j * 3 + 2]
    }

    return result, true
}