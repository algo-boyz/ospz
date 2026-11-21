package main

import "core:fmt"
import "core:os"
import "core:strings"
import "core:path/filepath"
import ospz ".."

main :: proc() {
    args := os.args[1:]
    if len(args) == 0 {
        print_usage()
        os.exit(1)
    }

    from_path, to_path: string
    from_coord := ospz.Coordinate_System.UNSPECIFIED
    to_coord   := ospz.Coordinate_System.UNSPECIFIED

    for arg in args {
        if strings.has_prefix(arg, "from=") {
            from_path = arg[5:]
        } else if strings.has_prefix(arg, "to=") {
            to_path = arg[3:]
        } else if strings.has_prefix(arg, "--coord=") {
            // simple parse, expand later
            name := strings.to_upper(arg[8:])
            // map "RUB", "RDF", etc. → enum
            fmt.printf("using %s coord system\n", name)
        } else if arg == "-h" || arg == "--help" {
            print_usage()
            return
        }
    }

    if from_path == "" || to_path == "" {
        fmt.eprintln("error: both from= and to= are required")
        print_usage()
        os.exit(1)
    }

    from_ext := strings.to_lower(filepath.ext(from_path))
    to_ext   := strings.to_lower(filepath.ext(to_path))

    // ---------- PLY → SPZ ----------
    if from_ext == ".ply" && to_ext == ".spz" {
        unpack_opts := ospz.Unpack_Options{to = .RUB} // or user override
        cloud, err := ospz.load_splat_from_ply(from_path, unpack_opts)
        if err != .None {
            fmt.eprintf("failed to load PLY: %v\n", err)
            os.exit(1)
        }
        defer ospz.release_gaussian_cloud(&cloud) // if you have a release helper

        pack_opts := ospz.Pack_Options{from = .RUB}
        err = ospz.save_spz_to_file(&cloud, pack_opts, to_path)
        if err != .None {
            fmt.eprintf("failed to save SPZ: %v\n", err)
            os.exit(1)
        }
        fmt.printf("wrote %s (%d points)\n", to_path, cloud.num_points)
        return
    }

    // ---------- SPZ → PLY ----------
    if from_ext == ".spz" && to_ext == ".ply" {
        unpack_opts := ospz.Unpack_Options{to = .RDF}
        cloud, err := ospz.load_spz_from_file(from_path, unpack_opts)
        if err != .None {
            fmt.eprintf("failed to load SPZ: %v\n", err)
            os.exit(1)
        }
        defer ospz.release_gaussian_cloud(&cloud)

        pack_opts := ospz.Pack_Options{from = .RDF}
        err = ospz.save_splat_to_ply(&cloud, pack_opts, to_path)
        if err != .None {
            fmt.eprintf("failed to save PLY: %v\n", err)
            os.exit(1)
        }
        fmt.printf("wrote %s (%d points)\n", to_path, cloud.num_points)
        return
    }

    fmt.eprintf("unsupported conversion: %s → %s\n", from_ext, to_ext)
    os.exit(1)
}

print_usage :: proc() {
    fmt.println(`ospz – one-shot Gaussian splat converter

Usage:
  ospz from=<file> to=<file> [options]

Examples:
  ospz from=scene.ply  to=scene.spz
  ospz from=scene.spz  to=scene.ply

Supported:
  .ply  →  .spz
  .spz  →  .ply
`)
}