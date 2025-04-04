# Description: Install build dependencies for Windows. This script is meant to be run on MacOS.
cargo install cargo-xwin
rustup target add x86_64-pc-windows-msvc
rustup target add wasm32-unknown-emscripten