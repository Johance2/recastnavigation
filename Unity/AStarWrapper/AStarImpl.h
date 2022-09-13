#pragma once
#include <map>
#include <list>
#include <set>
#include <vector>

enum AStarNodeState
{
	ASS_NONE,
	ASS_OPEN,
	ASS_CLOSE,
};

class CAStarNode;

struct Neighbor
{
	int loss; //
	CAStarNode *node;
};

// AStar算法的节点
class CAStarNode
{
public:
	CAStarNode();
	CAStarNode *parent; // 上一个节点
	std::list<Neighbor> neighbor; // 相邻的节点
	int f; // 评估值
	int g; // 实际消耗
	int h; // 预计消耗return false;
	int loss; // 主档消耗值 如果为-1 表示不可行走
	char state;
}; 

typedef CAStarNode* CAStarNodePtr;

struct CAStarNodeComp {
	bool operator()(const CAStarNodePtr &k1, const CAStarNodePtr &k2) const {
		return k1->f < k2->f;
	}
};

class CAStarImpl
{
public:
	CAStarImpl();
	~CAStarImpl();
	// 寻路
	virtual bool Search(CAStarNode *pStart, CAStarNode *pEnd);
	const std::vector<CAStarNode*> &GetPath();

protected:
	// 深度
	virtual int Gn(CAStarNode *pPrevNode, CAStarNode *pNode);
	// 启发函数 如果返回值为 -1 表示 不可用
	virtual int Hn(CAStarNode *pCurrentNode, CAStarNode *pPrevNode, CAStarNode *pEndNode);

private:
	void AddToOpen(CAStarNode *pNode);
	void AddToClose(CAStarNode *pNode);
	// 是否在关闭节点列表
	bool IsInClose(CAStarNode *pNode);
	// 是否在打开节点列表
	bool IsInOpen(CAStarNode *pNode);

protected:
	std::multiset<CAStarNodePtr, CAStarNodeComp> m_setOpen;  // 打开列表
	std::list<CAStarNode*> m_listPath; // 最终路径
	std::vector<CAStarNode*> m_vecPath; // 最终路径
};

