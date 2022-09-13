rm -r build
mkdir build && cd build
cmake ../
make
mkdir -p "../Plugins/OSX"
cp "./Unity/RecastWrapper/libRecastWrapper.dylib" "../Plugins/OSX/RecastWrapper.dylib"
cp "./Unity/NavMeshWrapper/libNavMeshWrapper.dylib" "../Plugins/OSX/NavMeshWrapper.dylib"
cp "./Unity/AStarWrapper/libAStarWrapper.dylib" "../Plugins/OSX/AStarWrapper.dylib"