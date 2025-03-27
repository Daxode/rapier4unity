@echo off
REM Description: Install build dependencies for Windows

REM Install Rust targets
rustup target add x86_64-pc-windows-msvc

REM Note: For building macOS binaries, you'll need to use the original shell scripts on a Mac
echo Windows build dependencies have been installed.