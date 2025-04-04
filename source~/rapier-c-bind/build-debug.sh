# Build MacOS dylibs
LIB="rapier_c_bind"
UNITY_LIB="unitybridge"

# Build MacOS dylibs
TARGET_ARM="aarch64-apple-darwin"
TARGET_X86="x86_64-apple-darwin"
cargo build --target=$TARGET_ARM
cargo build --target=$TARGET_X86
lipo -create -output ${LIB}.bundle \
  target/${TARGET_ARM}/debug/lib${LIB}.dylib \
  target/${TARGET_X86}/debug/lib${LIB}.dylib
lipo -create -output ${UNITY_LIB}.bundle \
  target/${TARGET_ARM}/debug/lib${UNITY_LIB}.dylib \
  target/${TARGET_X86}/debug/lib${UNITY_LIB}.dylib
mkdir ../../build_bin/macOS
cp ${LIB}.bundle ../../build_bin/macOS/
cp ${UNITY_LIB}.bundle ../../build_bin/macOS/

# Build IOS libs
TARGET_IOS="aarch64-apple-ios"
cargo build --target=$TARGET_IOS
mkdir ../../build_bin/iOS
cp target/${TARGET_IOS}/debug/lib${LIB}.a ../../build_bin/iOS/
cp target/${TARGET_IOS}/debug/lib${UNITY_LIB}.a ../../build_bin/iOS/

# Build Windows DLLs
TARGET_WINDOWS="x86_64-pc-windows-msvc"
cargo xwin build --target $TARGET_WINDOWS
mkdir ../../build_bin/Windows
cp target/$TARGET_WINDOWS/debug/$LIB.dll ../../build_bin/Windows
cp target/$TARGET_WINDOWS/debug/$LIB.pdb ../../build_bin/Windows
cp target/$TARGET_WINDOWS/debug/$UNITY_LIB.dll ../../build_bin/Windows
cp target/$TARGET_WINDOWS/debug/$UNITY_LIB.pdb ../../build_bin/Windows

# Build Android
TARGET_ANDROID="aarch64-linux-android"
ANDROID_NDK="/Applications/Unity/Hub/Editor/6000.0.40f1/PlaybackEngines/AndroidPlayer/NDK"
if [ -z "$ANDROID_NDK" ]; then
    echo "** Android build error: $ANDROID_NDK is not defined."
    exit 1
fi
export CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER="${ANDROID_NDK}/toolchains/llvm/prebuilt/darwin-x86_64/bin/aarch64-linux-android35-clang"
cargo build --target=$TARGET_ANDROID
mkdir ../../build_bin/Android
cp target/${TARGET_ANDROID}/debug/lib${LIB}.so ../../build_bin/Android/
cp target/${TARGET_ANDROID}/debug/lib${UNITY_LIB}.so ../../build_bin/Android/

# Build WebGL
TARGET_WEBGL="wasm32-unknown-unknown"
export PATH=/Applications/Unity/Hub/Editor/6000.0.44f1/PlaybackEngines/WebGLSupport/BuildTools/Emscripten/emscripten:$PATH
#export CARGO_TARGET_WASM32_UNKNOWN_EMSCRIPTEN_LINKER="emcc"
export EMCC_CFLAGS="-Oz -s ALLOW_MEMORY_GROWTH=1"
cargo build -p unitybridge --target=$TARGET_WEBGL -r
cargo build -p rapier-c-bind --target=$TARGET_WEBGL -r
mkdir ../../build_bin/WebGL
emar x target/${TARGET_WEBGL}/release/lib${LIB}.a --output=extracted_objs
rm -f extracted_objs/allocator_api2-c117db5a17780b90.allocator_api2.86497b8fc7b9cc69-cgu.0.rcgu.o
emar rcs ../../build_bin/WebGL/lib${LIB}.a extracted_objs/*.o
rm -rf ./extracted_objs
emar x target/${TARGET_WEBGL}/release/lib${UNITY_LIB}.a --output=extracted_objs/
rm -f extracted_objs/allocator_api2-c117db5a17780b90.allocator_api2.86497b8fc7b9cc69-cgu.0.rcgu.o
emar rcs ../../build_bin/WebGL/lib${UNITY_LIB}.a extracted_objs/*.o
rm -rf ./extracted_objs

#emar rcs ../../build_bin/WebGL/lib${LIB}.a ../../build_bin/WebGL/lib${LIB}.a

#cp target/${TARGET_WEBGL}/debug/${LIB}.js ../../build_bin/WebGL/
#cp target/${TARGET_WEBGL}/debug/${LIB}.wasm ../../build_bin/WebGL/

# Generate C# bindings
cargo run -- ./rapierbind/src