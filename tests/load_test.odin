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

@(test)
test_skip_extension_ilv_empty :: proc(t: ^testing.T) {
    recs, had, ok := ospz.skip_extension_ilv_zone(nil)
    testing.expect(t, ok)
    testing.expect(t, !had)
    testing.expect(t, len(recs) == 0)
}

@(test)
test_skip_extension_ilv_records :: proc(t: ^testing.T) {
    // two records: type 0xADBE0003 len 4 payload 01 02 03 04; type 0x4E530001 len 0
    zone := [?]u8{
        0x03, 0x00, 0xBE, 0xAD, // type LE
        0x04, 0x00, 0x00, 0x00, // len
        0x01, 0x02, 0x03, 0x04,
        0x01, 0x00, 0x53, 0x4E, // type
        0x00, 0x00, 0x00, 0x00, // len 0
    }
    recs, had, ok := ospz.skip_extension_ilv_zone(zone[:])
    testing.expect(t, ok)
    testing.expect(t, had)
    testing.expect(t, len(recs) == 2)
    testing.expect_value(t, recs[0].type, u32(ospz.Spz_Extension_Type.ADOBE_Coordinate_System))
    testing.expect_value(t, recs[0].byte_length, u32(4))
    testing.expect(t, len(recs[0].payload) == 4)
    testing.expect_value(t, recs[1].type, u32(ospz.Spz_Extension_Type.NIANTIC_Georeference))
    testing.expect_value(t, recs[1].byte_length, u32(0))
}

@(test)
test_skip_extension_ilv_truncated :: proc(t: ^testing.T) {
    // claims 10-byte payload but only 2 bytes remain
    zone := [?]u8{
        0x01, 0x00, 0x00, 0x00,
        0x0A, 0x00, 0x00, 0x00,
        0xAA, 0xBB,
    }
    _, _, ok := ospz.skip_extension_ilv_zone(zone[:])
    testing.expect(t, !ok)
}