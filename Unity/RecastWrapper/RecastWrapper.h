#pragma once
#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif


extern "C"
{
	EXPORT_API int BuildSoloMesh(const char* objPath, const char* binPath, const char* param);
	EXPORT_API int BuildTempObstacles(const char* objPath, const char* binPath, const char* param);
	EXPORT_API int BuildBlockData(const char* binPath, const char* blockPath, int width, int height, bool bDebugFile = false);
	EXPORT_API int BuildBlockDataObstacles(const char* binPath, const char* blockPath, int width, int height, bool bDebugFile = false);
}