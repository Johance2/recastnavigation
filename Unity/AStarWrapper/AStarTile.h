#pragma once
#include "AStarImpl.h"
#include <vector>

class CAStarTileNode : public CAStarNode
{
public:
	int index;
};

class CAStarTile :
	public CAStarImpl
{
public:
	CAStarTile();
	~CAStarTile();
	void Init(int width, int height, bool bEnable45 = false);
	bool Search(int sx, int sy, int ex, int ey);

	inline int GetWidth() { return m_nWidth; }
	inline int GetHeight() { return m_nHeight; }

	inline std::vector<CAStarTileNode> &GetTileNode() { return m_vecTileNode;  }
	
protected:
	virtual int Hn(CAStarNode *pCurrentNode, CAStarNode *pPrevNode, CAStarNode *pEndNode);

	std::vector<CAStarTileNode> m_vecTileNode;
	int m_nWidth;
	int m_nHeight;
};

