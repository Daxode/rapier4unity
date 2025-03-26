# Build MacOS dylibs
cargo build -r
mv target/release/librapier_c_bind.dylib ../../build_bin/
mv target/release/librapier_c_bind.d ../../build_bin/

# Build Windows DLLs
cargo xwin build --target x86_64-pc-windows-msvc -r
mv target/x86_64-pc-windows-msvc/release/rapier_c_bind.dll ../../build_bin/
mv target/x86_64-pc-windows-msvc/release/rapier_c_bind.pdb ../../build_bin/