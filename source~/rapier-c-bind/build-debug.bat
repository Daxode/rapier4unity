@echo off
REM Build Windows DLLs
cargo build
IF NOT EXIST "..\..\build_bin" mkdir "..\..\build_bin"
copy /Y "target\debug\rapier_c_bind.dll" "..\..\build_bin\"
copy /Y "target\debug\rapier_c_bind.pdb" "..\..\build_bin\"
copy /Y "target\debug\unitybridge.dll" "..\..\build_bin\"
copy /Y "target\debug\unitybridge.pdb" "..\..\build_bin\"
cargo run -- ./rapierbind/src

echo Windows debug build completed.
echo Note: For macOS builds, run the shell scripts on a Mac system.
