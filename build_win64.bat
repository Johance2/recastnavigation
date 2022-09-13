rd /s/q build
rd /s/q .\Unity\Bin\Release
mkdir build && cd build
set BUILD_TYPE="Release"
cmake -A x64 ../
cmake --build ./ --config %BUILD_TYPE%
mkdir "../Plugins/x86_64"
copy "..\Unity\Bin\Release\RecastWrapper.dll" "../Plugins/x86_64/RecastWrapper.dll"
copy "..\Unity\Bin\Release\NavMeshWrapper.dll" "../Plugins/x86_64/NavMeshWrapper.dll"
copy "..\Unity\Bin\Release\AStarWrapper.dll" "../Plugins/x86_64/AStarWrapper.dll"
