fn main() {
    println!("cargo:rerun-if-changed=resources/release.json");
    let manifest = if std::env::var_os("CARGO_FEATURE_BUNDLED_RELEASE").is_some() {
        std::fs::read("resources/release.json")
            .expect("Run the release preparation script before building a bundled release")
    } else {
        br#"{"channel":"development","commit":"local","version":"development","firmware_sha256":null,"updater":false}"#.to_vec()
    };
    std::fs::write(
        std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap()).join("release.json"),
        manifest,
    )
    .unwrap();
    tauri_build::build();
}
