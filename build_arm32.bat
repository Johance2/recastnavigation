rd /s/q build
mkdir build && cd build
set NDK_ROOT=C:/Microsoft/AndroidNDK/android-ndk-r23c
set ANDROID_API=22
::set TOOLCHAIN=%NDK_ROOT%/build/cmake/android.toolchain.cmake
set TOOLCHAIN=%NDK_ROOT%/toolchains/llvm/prebuilt/windows-x86_64
set ANDROID_ABI="armeabi-v7a"
set BUILD_TYPE="Release"
cmake -A ARM ../ -DCMAKE_ANDROID_API=%ANDROID_API% -DCMAKE_ANDROID_ARCH_ABI=%ANDROID_ABI% -DCMAKE_ANDROID_NDK=%NDK_ROOT% -DCMAKE_SYSTEM_NAME=Android -DCMAKE_ANDROID_STANDALONE_TOOLCHAIN=%TOOLCHAIN%
cmake --build ./ --config %BUILD_TYPE%
mkdir "../Plugins/Android/%ANDROID_ABI%"
copy ".\Unity\NavMeshWrapper\%BUILD_TYPE%\libNavMeshWrapper.so" "../Plugins/Android/%ANDROID_ABI%/libNavMeshWrapper.so"
copy ".\Unity\RecastWrapper\%BUILD_TYPE%\libRecastWrapper.so" "../Plugins/Android/%ANDROID_ABI%/libRecastWrapper.so"
copy ".\Unity\AStarWrapper\%BUILD_TYPE%\libAStarWrapper.so" "../Plugins/Android/%ANDROID_ABI%/libAStarWrapper.so"
