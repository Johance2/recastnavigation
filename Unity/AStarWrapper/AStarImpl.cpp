#include "AStarImpl.h"
#include <algorithm>


CAStarNode::CAStarNode()
{
	parent = NULL;
	f = 0;
	g = 0;
	h = 0;
	loss = 1;
	state = ASS_NONE;
}

CAStarImpl::CAStarImpl()
{
}


CAStarImpl::~CAStarImpl()
{
}

bool UDgreater(CAStarNode *elem1, CAStarNode *elem2)
{
	return elem1->f < elem2->f;
}

bool CAStarImpl::Search(CAStarNode *pStart, CAStarNode *pEnd)
{
	m_listPath.clear();
	m_setOpen.clear();
	m_vecPath.clear();

	//if(pEnd->loss == -1)
	//	return false;

	if (pStart == pEnd)
	{
		m_vecPath.push_back(pStart);
		m_vecPath.push_back(pEnd);
		return true;
	}

	pStart->g = 0;
	pStart->h = Hn(pStart, NULL, pEnd);
	pStart->f = pStart->g + pStart->h;
	pStart->parent = 0;

	AddToOpen(pStart);
	bool bFind = false;
	// 有打开的节点就一直寻直到走到终点，或者没有点了
	while (!bFind)
	{
		if (m_setOpen.size() == 0)
			break;

		// 每次都取第一个 第一个永远是最好的
		CAStarNode *pCurrentNode = *m_setOpen.begin();
		m_setOpen.erase(m_setOpen.begin());
		AddToClose(pCurrentNode);

		// 给相邻节点计算期望值
		for (auto itr = pCurrentNode->neighbor.begin(); itr != pCurrentNode->neighbor.end(); ++itr)
		{
			CAStarNode *pNextNode = (*itr).node;
			// 到达终点
			if (pNextNode == pEnd)
			{
				pNextNode->g = Gn(pCurrentNode, pNextNode) + (*itr).loss;
				pNextNode->h = Hn(pNextNode, pCurrentNode, pEnd);
				pNextNode->f = pNextNode->g + pNextNode->h;
				pNextNode->parent = pCurrentNode;

				CAStarNode *pParend = pNextNode;
				while (pParend)
				{
					m_listPath.push_front(pParend);
					pParend = pParend->parent;
				}
				bFind = true;
				break;
			}

			// 在关闭列表 则把该节点
			if (IsInClose(pNextNode))
			{
				continue;
			}

			int h = Hn(pNextNode, pCurrentNode, pEnd);
			if (h == -1)
			{
				AddToClose(pCurrentNode);
				continue;
			}
			int g = Gn(pCurrentNode, pNextNode)+(*itr).loss;
			int f = g + h;

			// 如果在打开列表 则更新该节点的期望值
			if (IsInOpen(pNextNode))
			{
				// 消耗比较少则使用它
				if (f < pNextNode->f)
				{
					pNextNode->parent = pCurrentNode;
					pNextNode->g = g;
					pNextNode->h = h;
					pNextNode->f = f;

					auto itr2 = m_setOpen.find(pNextNode);
					// 因为返回的是 value 相同的第一个 迭代器，所以需要遍历后面了
					while (itr2 != m_setOpen.end() && (*itr2)->f == pNextNode->f)
					{
						if (*itr2 == pNextNode)
						{
							m_setOpen.erase(itr2);
							AddToOpen(pNextNode);
							break;
						}

						++itr2;
					}
				}
			}
			// 如果不在打开列表 则添加该节点
			else
			{
				pNextNode->parent = pCurrentNode;
				pNextNode->g = g;
				pNextNode->h = h;
				pNextNode->f = f;
				AddToOpen(pNextNode);
			}
		}
	};

	if (bFind)
	{
		for (auto itr = m_listPath.begin(); itr != m_listPath.end(); ++itr)
		{
			m_vecPath.push_back(*itr);
		}
	}

	return bFind;
}

const std::vector<CAStarNode*> &CAStarImpl::GetPath()
{
	return m_vecPath;
}

int CAStarImpl::Gn(CAStarNode *pPrevNode, CAStarNode *pNode)
{
	return pPrevNode->g + pNode->loss;
}

int CAStarImpl::Hn(CAStarNode *pCurrentNode, CAStarNode *pPrevNode, CAStarNode *pEndNode)
{
	return 0;
}

void CAStarImpl::AddToOpen(CAStarNode *pNode)
{
	pNode->state = ASS_OPEN;
	m_setOpen.insert(pNode);
}

void CAStarImpl::AddToClose(CAStarNode *pNode)
{
	pNode->state = ASS_CLOSE;
}

// 是否在关闭节点列表
bool CAStarImpl::IsInClose(CAStarNode *pNode)
{
	return pNode->state == ASS_CLOSE;
}
// 是否在打开节点列表
bool CAStarImpl::IsInOpen(CAStarNode *pNode)
{
	return pNode->state == ASS_OPEN;
}