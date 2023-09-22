#pragma once
#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif


extern "C"
{
	class NavMeshInstance;

	EXPORT_API NavMeshInstance* LoadNavMesh(unsigned char* pucValue, unsigned int uiLength);
	EXPORT_API int FindStraightPath(NavMeshInstance* inst, float startX, float startY, float endX, float endY);
	EXPORT_API bool GetPathPoint(NavMeshInstance* inst, int index, float& x, float& y);
	EXPORT_API bool PathRaycast(NavMeshInstance* inst, float startX, float startY, float endX, float endY, float& hitX, float& hitY);
	EXPORT_API void UnLoadNavMesh(NavMeshInstance* inst);
	// 动态阻挡专用函数
	EXPORT_API NavMeshInstance* LoadObstaclesMesh(unsigned char* pucValue, unsigned int uiLength);
	EXPORT_API bool AddObstacles(NavMeshInstance* inst, float x, float y, float z, float radius, float height, unsigned int &id, bool update);
	EXPORT_API bool AddBoxObstacles(NavMeshInstance* inst, float minx, float miny, float minz, float maxx, float maxy, float maxz, unsigned int& id, bool update);
	EXPORT_API bool RemoveObstacles(NavMeshInstance* inst, unsigned int id, bool update);
	EXPORT_API bool UpdateObstaclesMesh(NavMeshInstance* inst);
	// 集群寻路
	EXPORT_API bool InitCrowd(NavMeshInstance* inst, int max_agent = 128, float agent_radius = 0.7);
	EXPORT_API bool AddCrowdAgent(NavMeshInstance* inst, float x, float y, float z, float radius, float height, float maxAcceleration, float maxSpeed, unsigned int& id, int update_flag = 0);
	EXPORT_API bool RemoveCrowdAgent(NavMeshInstance* inst, unsigned int id);
	EXPORT_API bool UpdateCrowdAgent(NavMeshInstance* inst, float dt);
	EXPORT_API bool GetCrowdAgentPos(NavMeshInstance* inst, int index, float& x, float& y);
	EXPORT_API bool ResetCrowdAgentTarget(NavMeshInstance* inst, int index);
	EXPORT_API bool SetCrowdAgentTarget(NavMeshInstance* inst, int index, float x, float y);
	EXPORT_API void ClearNavMesh();
}