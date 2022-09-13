file(GLOB NavMeshWrapper_SOURCES *.cpp ../../RecastDemo/Contrib/fastlz/fastlz.c)
include_directories(../../Detour/Include)
include_directories(../../RecastDemo/Contrib/fastlz)
include_directories(../../DetourTileCache/Include)
if(IOS)
	add_library(NavMeshWrapper ${NavMeshWrapper_SOURCES})
	add_custom_command(TARGET NavMeshWrapper POST_BUILD
		COMMAND /usr/bin/libtool -static -o $<TARGET_FILE:NavMeshWrapper>
		$<TARGET_FILE:NavMeshWrapper> $<TARGET_FILE:Detour>
	)
else()
	add_library(NavMeshWrapper SHARED ${NavMeshWrapper_SOURCES})
endif()

SET_TARGET_PROPERTIES(NavMeshWrapper PROPERTIES MACOSX_BUNDLE TRUE)
target_link_libraries(NavMeshWrapper Detour DetourTileCache -lm)
if(MSVC)
	file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/Bin)
	set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/Unity/Bin)
endif()
