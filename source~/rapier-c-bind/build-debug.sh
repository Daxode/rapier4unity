# Build MacOS dylibs
cargo build
mv target/debug/librapier_c_bind.dylib ../../build_bin/
mv target/debug/librapier_c_bind.d ../../build_bin/

# Build Windows DLLs
cargo xwin build --target x86_64-pc-windows-msvc
mv target/x86_64-pc-windows-msvc/debug/rapier_c_bind.dll ../../build_bin/
mv target/x86_64-pc-windows-msvc/debug/rapier_c_bind.pdb ../../build_bin/