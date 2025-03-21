
Build the dylib file and move it to the build_bin folder
```shell
cargo build -r
mkdir ../../build_bin
mv target/release/librapier_c_bind.dylib ../../build_bin/
```

Build the dylib file and move it to the build_bin folder
```shell
cargo build
mv target/debug/librapier_c_bind.dylib ../../build_bin/
mv target/debug/librapier_c_bind.d ../../build_bin/
```

Find the symbols in the dylib file
```shell
nm -gU target/release/librapier_c_bind.dylib
```