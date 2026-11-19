package ospz

Splat_Attribute :: enum {
    Positions,
    Alphas,
    Colors,
    Scales,
    Rotations,
    Sh,
}

Packed_Gaussians :: struct {
    positions: []u8,
    alphas:    []u8,
    colors:    []u8,
    scales:    []u8,
    rotations: []u8,
    sh:        []u8,

    sh_degree: i32,
    num_points: i32,
}