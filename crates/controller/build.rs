use bindgen;
use std::path::PathBuf;

fn main() {
    let libdir_path = PathBuf::from("./betaflight")
        .canonicalize()
        .expect("Could not canonicalize betaflight dir path");
    let outdir_path = PathBuf::from("./src/generated/")
        .canonicalize()
        .expect("Could not canonicalize generated dir");
    let bind_header = libdir_path.join("src/main/bindgen.h");
    let libdir_path_str = libdir_path.to_str().expect("No betaflight directory");
    let bind_header_str = bind_header.to_str().expect("Bind header does not exists");

    // command line arguments are coming from bear. To generate a compile_commands.json you can
    // run bear -- make TARGET=SITL
    let bindings = bindgen::Builder::default()
        .clang_args(vec![
            "-c",
            &format!("-I{libdir_path_str}/src/main"),
            &format!("-I{libdir_path_str}/src/platform/SITL"),
            &format!("-I{libdir_path_str}/lib/main/dyad"),
            &format!("-I{libdir_path_str}/lib/main/MAVLink"),
            &format!("-I{libdir_path_str}/src/platform/SITL/target/SITL"),
            &format!("-I{libdir_path_str}/lib/main/google/olc"),
            "-std=gnu17",
            "-Wall",
            "-Wextra",
            "-Werror",
            // "-Wunsafe-loop-optimizations",
            "-Wdouble-promotion",
            "-Wold-style-definition",
            "-ffunction-sections",
            "-fdata-sections",
            "-fno-common",
            "-DTARGET_FLASH_SIZE=2048",
            "-DHSE_VALUE=8000000",
            "-D_GNU_SOURCE",
            "-DUSE_STDPERIPH_DRIVER",
            "-DSITL",
            "-DSIMULATOR_BUILD",
            "-DSITL",
            "-DSITL",
            "-DSITL",
            "-D__FORKNAME__=\"betaflight\"",
            "-D__TARGET__=\"SITL\"",
            "-D__REVISION__=\"norevision\"",
            "-pipe",
            "-flto=auto",
            // "-fuse-linker-plugin",
            "-ffast-math",
            "-fmerge-all-constants",
            "-Ofast",
            "-o",
            "obj/main/SITL/sitl.o",
        ])
        .header(bind_header_str)
        .generate()
        .expect("Unable to generate bindings");
    let out_path = outdir_path.join("bf_bindings.rs");
    bindings
        .write_to_file(out_path)
        .expect("Couldn't write bindings!");
}
