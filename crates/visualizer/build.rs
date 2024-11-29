use std::path::PathBuf;

fn main() {
    let libdir_path = PathBuf::from("../flight_controller/betaflight")
        .canonicalize()
        .expect("Could not canonicalize betaflight dir path");
    let libdir_path_str = libdir_path.to_str().expect("No betaflight directory");

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
    println!("cargo:rustc-link-arg=-Wl,-Map,{libdir_path_str}/obj/main/betaflight_SITL.map");
    println!("cargo:rustc-link-arg=-Wl,-L{libdir_path_str}/src/platform/SITL/link");
    println!("cargo:rustc-link-arg=-Wl,--cref");
    println!("cargo:rustc-link-arg=-Wl,-T{libdir_path_str}/src/main/target/SITL/pg.ld");
    println!("cargo:rustc-link-arg=-fuse-ld=bfd");
}
