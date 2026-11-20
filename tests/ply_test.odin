package testospz

import "core:fmt"
import "core:math"
import "core:os"
import "core:testing"
import ospz ".."

@(test)
ply_roundtrip_identity :: proc(t: ^testing.T) {
	// Synthetic cloud in RDF (PLY native). Save with from=RDF, load with to=RDF.
	n :: 3
	sh_degree :: i32(1)
	sh_dim := int(ospz.dim_for_degree(sh_degree))

	cloud: ospz.Gaussian_Cloud
	cloud.num_points = n
	cloud.sh_degree = sh_degree
	cloud.positions = make([]f32, n * 3)
	cloud.scales = make([]f32, n * 3)
	cloud.rotations = make([]f32, n * 4)
	cloud.alphas = make([]f32, n)
	cloud.colors = make([]f32, n * 3)
	cloud.sh = make([]f32, n * sh_dim * 3)
	defer ospz.release_gaussian_cloud(&cloud)

	for i in 0 ..< n {
		cloud.positions[i*3+0] = f32(i) * 1.1
		cloud.positions[i*3+1] = f32(i) * 2.2
		cloud.positions[i*3+2] = f32(i) * 3.3
		cloud.scales[i*3+0] = -1.0
		cloud.scales[i*3+1] = -0.5
		cloud.scales[i*3+2] = 0.0
		// xyzw
		cloud.rotations[i*4+0] = 0.0
		cloud.rotations[i*4+1] = 0.0
		cloud.rotations[i*4+2] = 0.0
		cloud.rotations[i*4+3] = 1.0
		cloud.alphas[i] = 0.5
		cloud.colors[i*3+0] = 0.1
		cloud.colors[i*3+1] = 0.2
		cloud.colors[i*3+2] = 0.3
		for j in 0 ..< sh_dim {
			cloud.sh[(i*sh_dim+j)*3+0] = f32(j) * 0.01
			cloud.sh[(i*sh_dim+j)*3+1] = f32(j) * 0.02
			cloud.sh[(i*sh_dim+j)*3+2] = f32(j) * 0.03
		}
	}

	path := "test_roundtrip.ply"
	defer os.remove(path)

	pack := ospz.Pack_Options{
		from = .RDF,
	}
	err := ospz.save_splat_to_ply(&cloud, pack, path)
	testing.expect_value(t, err, ospz.Spz_Error.None)

	loaded, lerr := ospz.load_splat_from_ply(path, {to = .RDF})
	defer ospz.release_gaussian_cloud(&loaded)
	testing.expect_value(t, lerr, ospz.Spz_Error.None)
	testing.expect_value(t, loaded.num_points, cloud.num_points)
	testing.expect_value(t, loaded.sh_degree, cloud.sh_degree)

	eps :: f32(1e-5)
	for i in 0 ..< n * 3 {
		testing.expect(t, math.abs(loaded.positions[i] - cloud.positions[i]) < eps)
		testing.expect(t, math.abs(loaded.scales[i] - cloud.scales[i]) < eps)
		testing.expect(t, math.abs(loaded.colors[i] - cloud.colors[i]) < eps)
	}
	for i in 0 ..< n * 4 {
		testing.expect(t, math.abs(loaded.rotations[i] - cloud.rotations[i]) < eps)
	}
	for i in 0 ..< n {
		testing.expect(t, math.abs(loaded.alphas[i] - cloud.alphas[i]) < eps)
	}
	for i in 0 ..< len(cloud.sh) {
		testing.expect(t, math.abs(loaded.sh[i] - cloud.sh[i]) < eps)
	}
}

@(test)
ply_roundtrip_rdf_to_rub :: proc(t: ^testing.T) {
	// Save as RDF, load into RUB — exercises cross-family conversion.
	n :: 1
	cloud: ospz.Gaussian_Cloud
	cloud.num_points = n
	cloud.sh_degree = 0
	cloud.positions = make([]f32, 3)
	cloud.scales = make([]f32, 3)
	cloud.rotations = make([]f32, 4)
	cloud.alphas = make([]f32, 1)
	cloud.colors = make([]f32, 3)
	cloud.sh = make([]f32, 0)
	defer ospz.release_gaussian_cloud(&cloud)

    cloud.positions[0] = 1
	cloud.positions[1] = 2
	cloud.positions[2] = 3
	cloud.scales[0] = 0
	cloud.scales[1] = 0
	cloud.scales[2] = 0
	cloud.rotations[0] = 0
	cloud.rotations[1] = 0
	cloud.rotations[2] = 0
	cloud.rotations[3] = 1
	cloud.alphas[0] = 0
	cloud.colors[0] = 0
	cloud.colors[1] = 0
	cloud.colors[2] = 0

	path := "test_rdf_rub.ply"
	defer os.remove(path)

	err := ospz.save_splat_to_ply(&cloud, {from = .RDF}, path)
	testing.expect_value(t, err, ospz.Spz_Error.None)

	loaded, lerr := ospz.load_splat_from_ply(path, {to = .RUB})
	defer ospz.release_gaussian_cloud(&loaded)
	testing.expect_value(t, lerr, ospz.Spz_Error.None)

    testing.expect(t, loaded.positions[0] == loaded.positions[0]) // not NaN
	testing.expect(t, loaded.positions[1] == loaded.positions[1])
	testing.expect(t, loaded.positions[2] == loaded.positions[2])

	testing.expect(t, math.abs(loaded.positions[0]) < 1e20)
	testing.expect(t, math.abs(loaded.positions[1]) < 1e20)
	testing.expect(t, math.abs(loaded.positions[2]) < 1e20)

	fmt.println("RDF(1,2,3) → RUB:", loaded.positions[0], loaded.positions[1], loaded.positions[2])
}