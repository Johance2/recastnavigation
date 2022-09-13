rd /s/q build
mkdir build && cd build
set BUILD_TYPE="Release"
cmake -A x64 ../ -DRECASTNAVIGATION_DEMO="ON"
cmake --build ./ --config %BUILD_TYPE%
