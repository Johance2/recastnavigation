rd /s/q build
rd /s/q .\Unity\Bin\Release
mkdir build && cd build
set BUILD_TYPE="Release"
cmake -A Win32 ../
cmake --build ./ --config %BUILD_TYPE%
mkdir "../Plugins/x86"
copy "..\Unity\Bin\Release\RecastWrapper.dll" "../Plugins/x86/RecastWrapper.dll"
copy "..\Unity\Bin\Release\NavMeshWrapper.dll" "../Plugins/x86/NavMeshWrapper.dll"
copy "..\Unity\Bin\Release\AStarWrapper.dll" "../Plugins/x86/AStarWrapper.dll"
