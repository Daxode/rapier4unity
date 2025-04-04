# Build MacOS dylibs
LIB="rapier_c_bind"
UNITY_LIB="unitybridge"

# Build MacOS dylibs
TARGET_ARM="aarch64-apple-darwin"
TARGET_X86="x86_64-apple-darwin"
cargo build -r --target=$TARGET_ARM
cargo build -r --target=$TARGET_X86
lipo -create -output ${LIB}.bundle \
  target/${TARGET_ARM}/release/lib${LIB}.dylib \
  target/${TARGET_X86}/release/lib${LIB}.dylib
lipo -create -output ${UNITY_LIB}.bundle \
  target/${TARGET_ARM}/release/lib${UNITY_LIB}.dylib \
  target/${TARGET_X86}/release/lib${UNITY_LIB}.dylib
mkdir ../../build_bin/macOS
cp ${LIB}.bundle ../../build_bin/macOS/
cp ${UNITY_LIB}.bundle ../../build_bin/macOS/

# Build IOS libs
TARGET_IOS="aarch64-apple-ios"
cargo build -r --target=$TARGET_IOS
mkdir ../../build_bin/iOS
cp target/${TARGET_IOS}/release/lib${LIB}.a ../../build_bin/iOS/
cp target/${TARGET_IOS}/release/lib${UNITY_LIB}.a ../../build_bin/iOS/

# Build Windows DLLs
TARGET_WINDOWS="x86_64-pc-windows-msvc"
cargo xwin build -r --target $TARGET_WINDOWS
mkdir ../../build_bin/Windows
cp target/$TARGET_WINDOWS/release/$LIB.dll ../../build_bin/Windows
cp target/$TARGET_WINDOWS/release/$LIB.pdb ../../build_bin/Windows
cp target/$TARGET_WINDOWS/release/$UNITY_LIB.dll ../../build_bin/Windows
cp target/$TARGET_WINDOWS/release/$UNITY_LIB.pdb ../../build_bin/Windows

# Build Android
TARGET_ANDROID="aarch64-linux-android"
ANDROID_NDK="/Applications/Unity/Hub/Editor/6000.0.40f1/PlaybackEngines/AndroidPlayer/NDK"
if [ -z "$ANDROID_NDK" ]; then
    echo "** Android build error: $ANDROID_NDK is not defined."
    exit 1
fi
export CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER="${ANDROID_NDK}/toolchains/llvm/prebuilt/darwin-x86_64/bin/aarch64-linux-android35-clang"
cargo build -r --target=$TARGET_ANDROID
mkdir ../../build_bin/Android
cp target/${TARGET_ANDROID}/release/lib${LIB}.so ../../build_bin/Android/
cp target/${TARGET_ANDROID}/release/lib${UNITY_LIB}.so ../../build_bin/Android/

# Generate C# bindings
cargo run -- ./rapierbind/src