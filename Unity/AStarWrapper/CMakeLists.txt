file(GLOB AStarWrapper_SOURCES *.cpp)
if(IOS)
	add_library(AStarWrapper ${AStarWrapper_SOURCES})
	add_custom_command(TARGET AStarWrapper POST_BUILD
		COMMAND /usr/bin/libtool -static -o $<TARGET_FILE:AStarWrapper>
		$<TARGET_FILE:AStarWrapper>
	)
else()
	add_library(AStarWrapper SHARED ${AStarWrapper_SOURCES})
endif()

SET_TARGET_PROPERTIES(AStarWrapper PROPERTIES MACOSX_BUNDLE TRUE)
target_link_libraries(AStarWrapper -lm)
if(MSVC)
	file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/Bin)
	set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/Unity/Bin)
endif()
