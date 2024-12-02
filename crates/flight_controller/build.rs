use bindgen;
use std::path::PathBuf;

fn main() {
    let bf_path = PathBuf::from("./betaflight")
        .canonicalize()
        .expect("Could not canonicalize betaflight dir path");
    // let sitl_path = PathBuf::from("./sitl")
    //     .canonicalize()
    //     .expect("Could not canonicalize sitl dir path");
    let bindings_path = PathBuf::from("./src/bindings/")
        .canonicalize()
        .expect("Could not canonicalize generated dir");
    let betaflight_bind_header = bf_path.join("src/main/bindgen.h");
    // let sitl_bind_header = sitl_path.join("sitl.h");
    let bf_dir_path_str = bf_path.to_str().expect("No betaflight directory");
    // let sitl_dir_path_str = sitl_path.to_str().expect("No sitl directory");
    let bf_bind_header_str = betaflight_bind_header
        .to_str()
        .expect("Betaflight header does not exists");
    // let sitl_bind_header_str = sitl_bind_header
    //     .to_str()
    //     .expect("Sitl header does not exists");

    // command line arguments are coming from bear. To generate a compile_commands.json you can
    // run bear -- make TARGET=SITL
    let bf_bindings = bindgen::Builder::default()
        .clang_args(vec![
            &format!("-I{bf_dir_path_str}/src/main"),
            &format!("-I{bf_dir_path_str}/src/main/target"),
            &format!("-I{bf_dir_path_str}/src/main/startup"),
            &format!("-I{bf_dir_path_str}/lib/main/dyad"),
            &format!("-I{bf_dir_path_str}/lib/main/MAVLink"),
            &format!("-I{bf_dir_path_str}/src/main/target/SITL"),
            &format!("-I{bf_dir_path_str}/lib/main/google/olc"),
            "-std=gnu17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wpedantic",
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
            "-D__FORKNAME__=\"betaflight\"",
            "-D__TARGET__=\"SITL\"",
            "-D__REVISION__=\"norevision\"",
            "-pipe",
            "-fPIC",
            "-g",
            "-flto",
            "-ffast-math",
            "-fmerge-all-constants",
            "-Os",
        ])
        .header(bf_bind_header_str)
        .generate_inline_functions(true) // Make all bindings public
        .generate_comments(true)
        .generate()
        .expect("Unable to generate bindings");
    let bf_generated_path = bindings_path.join("bf_bindings_generated.rs");
    bf_bindings
        .write_to_file(bf_generated_path)
        .expect("Couldn't write bindings!");
    // let sitl_bindings = bindgen::Builder::default()
    //     .clang_args(vec![
    //         "-DLWS_BUILDING_STATIC",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/include",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/secure-streams",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/plat/unix/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/core/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/misc/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/misc/misc/jrpc",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/system/./async-dns",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/system/smd/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/system/metrics/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/core-net/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/roles/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/roles/http/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/roles/http/./compression",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/roles/h1/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/roles/h2/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/roles/ws/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/roles/raw-skt/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/roles/raw-file/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/roles/listen/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/event-libs/.",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/event-libs/poll/../poll",
    //         "-I/home/gabor/ascent/sitl/extern/libwebsockets/lib/secure-streams/.",
    //         "-Wno-deprecated-declarations",
    //         "-Wno-deprecated",
    //         "-Wall",
    //         "-Wextra",
    //         "-Wno-unused-parameter",
    //         "-Wconversion",
    //         "-Wsign-compare",
    //         "-Wstrict-aliasing",
    //         "-fvisibility=hidden",
    //         "-Wundef",
    //         "-Wuninitialized",
    //         "-Wtype-limits",
    //         "-Wignored-qualifiers",
    //         "-Werror",
    //         "-fPIC",
    //         "-c",
    //         "-o",
    //         "CMakeFiles/websockets.dir/plat/unix/unix-caps.c.o",
    //         "/home/gabor/ascent/sitl/extern/libwebsockets/lib/plat/unix/unix-caps.c",
    //     ])
    //     .header(sitl_bind_header_str)
    //     .generate_inline_functions(true) // Make all bindings public
    //     .generate_comments(true)
    //     .generate()
    //     .expect("Unable to generate bindings");
    // let sitl_generated_path = bindings_path.join("sitl.rs");
    // sitl_bindings
    //     .write_to_file(sitl_generated_path)
    //     .expect("Couldn't write bindings!");

    // Linker flags
    let betaflight_lib_include = format!("cargo:rustc-link-search={bf_dir_path_str}/lib");
    println!("{}", betaflight_lib_include); // -L./lib
    println!("cargo:rustc-link-lib=dylib=betaflight"); // -lbetaflight
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
        format!("cargo:rustc-link-arg=-Wl,-Map,{bf_dir_path_str}/obj/main/betaflight_SITL.map");
    println!("{}", betaflight_sitl_map_include);
    let betaflight_link_include =
        format!("cargo:rustc-link-arg=-Wl,-L{bf_dir_path_str}/src/platform/SITL/link");
    println!("{}", betaflight_link_include);
    println!("cargo:rustc-link-arg=-Wl,--cref");
    let betaflight_ld_include =
        format!("cargo:rustc-link-arg=-Wl,-T{bf_dir_path_str}/src/main/target/SITL/pg.ld");
    println!("{}", betaflight_ld_include);
    println!("cargo:rustc-link-arg=-fuse-ld=bfd");
}
