package testospz

import "core:testing"
import "core:fmt"
import ospz ".."

@(test)
test_load_boat_spz :: proc(t: ^testing.T) {
    cloud, err := ospz.load_spz_from_file("models/boat.spz")
    testing.expect_value(t, err, ospz.Spz_Error.None)
    if err != .None {
        fmt.eprintln("load failed:", err)
        return
    }
    defer ospz.release_gaussian_cloud(&cloud)

    testing.expect(t, cloud.num_points > 0, "expected points")
    testing.expect(t, len(cloud.positions) == int(cloud.num_points) * 3)
    testing.expect(t, len(cloud.alphas) == int(cloud.num_points))
    testing.expect(t, len(cloud.rotations) == int(cloud.num_points) * 4)
    fmt.println("boat.spz:", cloud.num_points, "points, SH degree", cloud.sh_degree,
        "antialiased", cloud.antialiased)
}