use bindgen;
use std::path::PathBuf;

fn main() {
    let libdir_path = PathBuf::from("./sitl")
        .canonicalize()
        .expect("Could not canonicalize betaflight dir path");
    let outdir_path = PathBuf::from("./src/bindings/")
        .canonicalize()
        .expect("Could not canonicalize generated dir");
    let bind_header = libdir_path.join("sitl.h");
    let libdir_path_str = libdir_path.to_str().expect("No betaflight directory");
    let bind_header_str = bind_header.to_str().expect("Bind header does not exists");
    // command line arguments are coming from bear. To generate a compile_commands.json you can
    // run bear -- make TARGET=SITL
    let bindings = bindgen::Builder::default()
        .clang_args(vec![
            &format!("-I{libdir_path_str}/extern/betaflightext/src/main"),
            &format!("-I{libdir_path_str}/extern/betaflightext/src/main/target/simITL"),
            &format!("-I{libdir_path_str}/extern/betaflight/lib/main/dyad"),
            &format!("-I{libdir_path_str}/extern/betaflight/src/main"),
            &format!("-I{libdir_path_str}/extern/libwebsockets/include"),
            &format!("-I{libdir_path_str}/extern/libwebsockets/lib/../include"),
            "-std=gnu17",
            "-DFLASH_SIZE=2048",
            "-DHSE_VALUE=8000000",
            "-DSIMULATOR_BUILD",
            "-DSITL",
            "-D__REVISION__=\"1\"",
            "-D__TARGET__=\"SimITL\"",
            "-Dsitl_EXPORTS",
            "-fPIC",
            // "-c",
            // "-o",
        ])
        .header(bind_header_str)
        .generate_inline_functions(true) // Make all bindings public
        .generate_comments(true)
        .generate()
        .expect("Unable to generate bindings");
    let out_path = outdir_path.join("sitl_generated.rs");
    bindings
        .write_to_file(out_path)
        .expect("Couldn't write bindings!");
}
