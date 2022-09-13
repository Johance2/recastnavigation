#include "AStarWrapper.h"
#include <stdio.h>
#include <stdint.h>
#include <memory>
#include <fstream>
#include "AStarTile.h"

CAStarTile* g_pAStarTitle = nullptr;
int g_Astar_Offset_X = 0;
int g_Astar_Offset_Y = 0;

void AstarCreate(int width, int height, bool enable45, int offset_x, int offset_y)
{
	if (g_pAStarTitle == nullptr)
	{
		g_pAStarTitle = new CAStarTile();
	}
	g_pAStarTitle->Init(width, height, enable45);
	g_Astar_Offset_X = offset_x;
	g_Astar_Offset_Y = offset_y;
}

bool AStarSetCost(int sx, int sy, int lost)
{
	if (g_pAStarTitle == nullptr)
	{
		return false;
	}
	sx = sx - g_Astar_Offset_X;
	sy = sy - g_Astar_Offset_X;
	if (sx < 0 || sx >= g_pAStarTitle->GetWidth() || sy < 0 || sy >= g_pAStarTitle->GetHeight())
	{
		return false;
	}

	int idx = sx + sy * g_pAStarTitle->GetWidth();
	auto &nodes = g_pAStarTitle->GetTileNode();
	if (idx < 0 || idx >= nodes.size())
	{
		return false;
	}
	nodes[idx].loss = lost;
	return true;
}

int AStarSearch(int sx, int sy, int ex, int ey)
{
	if (g_pAStarTitle == nullptr)
	{
		return 0;
	}



	sx = sx - g_Astar_Offset_X;
	sy = sy - g_Astar_Offset_X;
	ex = ex - g_Astar_Offset_X;
	ey = ey - g_Astar_Offset_X;

	if (sx < 0)
	{
		sx = 0;
	}
	else
	{
		sx = std::min<int>(sx, g_pAStarTitle->GetWidth()-1);
	}
	if (sy < 0)
	{
		sy = 0;
	}
	else
	{
		sy = std::min<int>(sy, g_pAStarTitle->GetHeight() - 1);
	}

	if (ex < 0)
	{
		ex = 0;
	}
	else
	{
		ex = std::min<int>(ex, g_pAStarTitle->GetWidth() - 1);
	}
	if (ey < 0)
	{
		ey = 0;
	}
	else
	{
		ey = std::min<int>(ey, g_pAStarTitle->GetHeight() - 1);
	}


	if (!g_pAStarTitle->Search(sx, sy, ex, ey))
	{
		return 0;
	}
	return g_pAStarTitle->GetPath().size();
}

bool AStarGetPath(int index, int &x, int &y)
{
	if (g_pAStarTitle == nullptr)
	{
		return false;
	}

	auto &paths = g_pAStarTitle->GetPath();
	if (index < 0 || index >= paths.size())
		return false;

	int idx = ((CAStarTileNode*)paths[index])->index;
	x = idx % g_pAStarTitle->GetWidth() + g_Astar_Offset_X;
	y = idx / g_pAStarTitle->GetWidth() + g_Astar_Offset_Y;

	return true;
}

void AstarRelease()
{
	if (g_pAStarTitle == nullptr)
	{
		delete g_pAStarTitle;
		g_pAStarTitle = nullptr;
	}
}