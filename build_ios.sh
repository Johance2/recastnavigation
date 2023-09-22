rm -r build
mkdir build && cd build
cmake ../ -GXcode \
    -DCMAKE_SYSTEM_NAME=iOS \
    "-DCMAKE_OSX_ARCHITECTURES=arm64" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=9.3 \
    -DCMAKE_INSTALL_PREFIX=`pwd`/_install \
    -DCMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH=NO \
    -DCMAKE_XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED=NO \
    -DCMAKE_IOS_INSTALL_COMBINED=YES
sudo xcode-select -switch /Applications/Xcode.app/Contents/Developer
cmake --build . --config Release
mkdir -p "../Plugins/iOS"
cp "./Unity/NavMeshWrapper/Release-iphoneos/libNavMeshWrapper.a" "../Plugins/iOS/NavMeshWrapper.a"
cp "./Unity/RecastWrapper/Release-iphoneos/libRecastWrapper.a" "../Plugins/iOS/RecastWrapper.a"
cp "./Unity/AStarWrapper/Release-iphoneos/libAStarWrapper.a" "../Plugins/iOS/AStarWrapper.a"