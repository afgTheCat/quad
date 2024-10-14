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
            &format!("-I{libdir_path_str}/src/main"),
            &format!("-I{libdir_path_str}/src/platform/SITL"),
            &format!("-I{libdir_path_str}/lib/main/dyad"),
            &format!("-I{libdir_path_str}/lib/main/MAVLink"),
            &format!("-I{libdir_path_str}/src/platform/SITL/target/SITL"),
            &format!("-I{libdir_path_str}/lib/main/google/olc"),
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
            "-Ofast",
        ])
        .header(bind_header_str)
        .generate()
        .expect("Unable to generate bindings");
    let out_path = outdir_path.join("bf_bindings.rs");
    bindings
        .write_to_file(out_path)
        .expect("Couldn't write bindings!");

    // Linker flags
    let betaflight_lib_include = format!("cargo:rustc-link-search={libdir_path_str}/lib");
    println!("{}", betaflight_lib_include); // -L./lib
    println!("cargo:rustc-link-lib=static=betaflight"); // -lbetaflight
    println!("cargo:rustc-link-lib=m"); // -lm
    println!("cargo:rustc-link-lib=pthread"); // -lpthread
    println!("cargo:rustc-link-lib=c"); // -lc
    println!("cargo:rustc-link-lib=rt"); // -lrt

    // Additional linker options
    println!("cargo:rustc-link-arg=-flto=auto");
    println!("cargo:rustc-link-arg=-fuse-linker-plugin");
    println!("cargo:rustc-link-arg=-ffast-math");
    println!("cargo:rustc-link-arg=-fmerge-all-constants");
    println!("cargo:rustc-link-arg=-Ofast");
    println!("cargo:rustc-link-arg=-Wl,-gc-sections");
    let betaflight_sitl_map_include =
        format!("cargo:rustc-link-arg=-Wl,-Map,{libdir_path_str}/obj/main/betaflight_SITL.map");
    println!("{}", betaflight_sitl_map_include);
    let betaflight_link_include =
        format!("cargo:rustc-link-arg=-Wl,-L{libdir_path_str}/src/platform/SITL/link");
    println!("{}", betaflight_link_include);
    println!("cargo:rustc-link-arg=-Wl,--cref");
    let betaflight_ld_include =
        format!("cargo:rustc-link-arg=-Wl,-T{libdir_path_str}/src/platform/SITL/link/sitl.ld");
    println!("{}", betaflight_ld_include);
}
