
#include <stdio.h>
#include <iostream>

extern "C"
{
	int BuildSoloMesh(const char* objPath, const char* binPath, const char* param);
	int BuildTempObstacles(const char* objPath, const char* binPath, const char* param);
	int BuildBlockData(const char* binPath, const char* blockPath, int width, int height, bool bDebugFile = false);
	int BuildBlockDataObstacles(const char* binPath, const char* blockPath, int width, int height, bool bDebugFile = false);
	bool LoadNavMesh(unsigned char* pucValue, unsigned int uiLength);
	int FindStraightPath(float startX, float startY, float endX, float endY);
	bool GetPathPoint(int index, float& x, float& y);
	bool PathRaycast(float startX, float startY, float endX, float endY, float& hitX, float& hitY);
	void UnLoadNavMesh();
	// 动态阻挡专用函数
	bool LoadObstaclesMesh(unsigned char* pucValue, unsigned int uiLength);
	bool AddObstacles(float x, float y, float z, float radius, float height, unsigned int& id, bool update);
	bool AddBoxObstacles(float minx, float miny, float minz, float maxx, float maxy, float maxz, unsigned int& id, bool update);
	bool RemoveObstacles(unsigned int id, bool update);
	bool UpdateObstaclesMesh();
}

int main(int argc, char* argv[])
{
	//BuildSoloMesh("D:/HT/client/Assets/BundleAssets/Map_Source/NavMesh/dijingditu_Walkable_NavMesh.obj",
	//	"D:/HT/client/Assets/BundleAssets/Map_Source/NavMesh/dijingditu_Walkable_NavMesh.obj.bytes",
	//	"0.3 0.2 2 0.7 0.9 45 8 20 12 1.3 6 6 1");

	//{
	//	std::cout << "build nav mesh" << std::endl;
	//	BuildSoloMesh("./dijingditu_Walkable_NavMesh.obj",
	//		"./dijingditu_Walkable_NavMesh.obj.bytes",
	//		"0.3 0.2 2 0.7 0.9 45 8 20 12 1.3 6 6 1");

	//	std::cout << "build nav mesh block data" << std::endl;
	//	BuildBlockData("./dijingditu_Walkable_NavMesh.obj.bytes", "./dijingditu_Walkable_NavMesh.block.bytes", 128, 128, true);


	//	std::cout << "test nav mesh block data" << std::endl;
	//	{
	//		FILE* fp = fopen("././dijingditu_Walkable_NavMesh.block.bytes", "rb");
	//		int width, height;
	//		fread(&width, sizeof(width), 1, fp);
	//		fread(&height, sizeof(height), 1, fp);
	//		char* block = new char[width * height];
	//		fread(block, width * height, 1, fp);

	//		{
	//			float x = 10.4;
	//			float y = 10.8;
	//			int index = (int)y * width + (int)x;
	//			bool is_block = block[index] == 0;
	//			std::cout << "(" << x << "," << y << ")" << "block:" << is_block << std::endl;
	//		}
	//		{
	//			float x = 0;
	//			float y = 0;
	//			int index = (int)y * width + (int)x;
	//			bool is_block = block[index] == 0;
	//			std::cout << "(" << x << "," << y << ")" << "block:" << is_block << std::endl;
	//		}

	//		delete[] block;
	//	}
	//}

	{
		std::cout << "build Obstacles nav mesh" << std::endl;
		BuildTempObstacles("./dijingditu_Walkable_NavMesh2.obj",
			"../../RecastDemo/Bin/all_tiles_tilecache.bin",
			"0.3 0.2 2 0.0 0.9 45 8 20 12 1.3 6 6 1 48");

		BuildTempObstacles("./dijingditu_Walkable_NavMesh2.obj",
			"./coc_Walkable_NavMesh.bytes",
			"0.3 0.2 2 0.0 0.9 45 8 20 12 1.3 6 6 1 48");


		unsigned char* pucValue = 0;
		FILE* fp = fopen("coc_Walkable_NavMesh.bytes", "rb");
		if (!fp)
			return false;
		if (fseek(fp, 0, SEEK_END) != 0)
		{
			fclose(fp);
			return false;
		}
		long uiLength = ftell(fp);
		if (uiLength < 0)
		{
			fclose(fp);
			return false;
		}
		if (fseek(fp, 0, SEEK_SET) != 0)
		{
			fclose(fp);
			return false;
		}
		pucValue = (unsigned char*)malloc(uiLength);
		if (!pucValue)
		{
			fclose(fp);
			return false;
		}
		size_t readLen = fread(pucValue, uiLength, 1, fp);
		fclose(fp);
		LoadObstaclesMesh(pucValue, uiLength);




		//std::cout << "build nav mesh block data" << std::endl;
		//BuildBlockDataObstacles("./coc_Walkable_NavMesh.bytes", "./coc_Walkable_NavMesh.block.bytes", 64, 64, true);

		//std::cout << "test nav mesh block data" << std::endl;
		//{
		//	FILE* fp = fopen("././coc_Walkable_NavMesh.block.bytes", "rb");
		//	int width, height;
		//	fread(&width, sizeof(width), 1, fp);
		//	fread(&height, sizeof(height), 1, fp);
		//	char* block = new char[width * height];
		//	fread(block, width * height, 1, fp);

		//	{
		//		float x = 10.4;
		//		float y = 10.8;
		//		int index = (int)y * width + (int)x;
		//		bool is_block = block[index] == 0;
		//		std::cout << "(" << x << "," << y << ")" << "block:" << is_block << std::endl;
		//	}
		//	{
		//		float x = 0;
		//		float y = 0;
		//		int index = (int)y * width + (int)x;
		//		bool is_block = block[index] == 0;
		//		std::cout << "(" << x << "," << y << ")" << "block:" << is_block << std::endl;
		//	}

		//	delete[] block;
		//}
	}

	system("pause");
	return 0;
}