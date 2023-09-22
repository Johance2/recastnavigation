file(GLOB RECASTWRAPPER_SOURCES *.cpp ../../RecastDemo/Source/MeshLoaderObj.cpp ../../RecastDemo/Source/ChunkyTriMesh.cpp ../../RecastDemo/Contrib/fastlz/fastlz.c)
include_directories(../../Recast/Include)
include_directories(../../RecastDemo/Include)
include_directories(../../RecastDemo/Contrib/fastlz)
include_directories(../../DetourTileCache/Include)
include_directories(../../Detour/Include)
if(IOS)
	add_library(RecastWrapper ${RECASTWRAPPER_SOURCES})
	add_custom_command(TARGET RecastWrapper POST_BUILD
		COMMAND /usr/bin/libtool -static -o $<TARGET_FILE:RecastWrapper> $<TARGET_FILE:RecastWrapper> $<TARGET_FILE:Detour>
		COMMAND /usr/bin/libtool -static -o $<TARGET_FILE:RecastWrapper> $<TARGET_FILE:RecastWrapper> $<TARGET_FILE:DetourTileCache>
	)
else()
	add_library(RecastWrapper SHARED ${RECASTWRAPPER_SOURCES})
endif()

target_link_libraries(RecastWrapper Recast Detour DetourTileCache -lm)
if(MSVC)
	file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/Bin)
	set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/Unity/Bin)
endif()
