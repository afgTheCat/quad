use std::env;
use std::process::Command;

fn main() {
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
