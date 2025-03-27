@echo off
REM Build Windows DLLs in release mode
cargo build --release
IF NOT EXIST "..\..\build_bin" mkdir "..\..\build_bin"
copy /Y "target\release\rapier_c_bind.dll" "..\..\build_bin\"
copy /Y "target\release\rapier_c_bind.pdb" "..\..\build_bin\"

echo Windows release build completed.
echo Note: For macOS builds, run the shell scripts on a Mac system.