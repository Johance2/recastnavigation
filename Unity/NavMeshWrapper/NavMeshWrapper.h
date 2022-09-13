#pragma once
#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif


extern "C"
{
	EXPORT_API bool LoadNavMesh(unsigned char* pucValue, unsigned int uiLength);
	EXPORT_API int FindStraightPath(float startX, float startY, float endX, float endY);
	EXPORT_API bool GetPathPoint(int index, float& x, float& y);
	EXPORT_API bool PathRaycast(float startX, float startY, float endX, float endY, float& hitX, float& hitY);	
	EXPORT_API void UnLoadNavMesh();
	// 动态阻挡专用函数
	EXPORT_API bool LoadObstaclesMesh(unsigned char* pucValue, unsigned int uiLength);
	EXPORT_API bool AddObstacles(float x, float y, float z, float radius, float height, unsigned int &id, bool update);
	EXPORT_API bool AddBoxObstacles(float minx, float miny, float minz, float maxx, float maxy, float maxz, unsigned int& id, bool update);
	EXPORT_API bool RemoveObstacles(unsigned int id, bool update);
	EXPORT_API bool UpdateObstaclesMesh();
}