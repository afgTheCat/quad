#[cfg(feature = "betaflight")]
fn main() {
    use std::env;
    use std::process::Command;
    let target = env::var("TARGET").unwrap_or_else(|_| "SITL".to_string());

    let status = Command::new("make")
        .arg(format!("TARGET={}", target))
        .current_dir("./betaflight")
        .status()
        .expect("Failed to run make");

    if !status.success() {
        panic!("Betaflight build failed");
    }
}

// If betaflight is not enabled, there is no reason to use it!
fn main() {}
