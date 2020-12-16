// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "box2d/b2_broad_phase.h"
#include <string.h>

b2BroadPhase::b2BroadPhase()
{
	m_proxyCount = 0;

	for (int32 i = 0; i < b2_maxThreads; ++i)
	{
		m_perThreadData[i].m_queryProxyId = -1;
	}
}

b2BroadPhase::~b2BroadPhase()
{

}

#ifdef b2_dynamicTreeOfTrees
void b2BroadPhase::Reset(float32 subTreeWidth, float32 subTreeHeight)
{
	m_tree.Reset(subTreeWidth, subTreeHeight);
	m_proxyCount = 0;
	b2Assert(m_moveBuffer.size() == 0);
}
#endif

int32 b2BroadPhase::CreateProxy(const b2AABB& aabb, void* userData)
{
	int32 proxyId = m_tree.CreateProxy(aabb, userData);
	++m_proxyCount;
	BufferMove(proxyId);
	return proxyId;
}

void b2BroadPhase::DestroyProxy(int32 proxyId)
{
	UnBufferMove(proxyId);
	--m_proxyCount;
	m_tree.DestroyProxy(proxyId);
}

void b2BroadPhase::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
{
	bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
	if (buffer)
	{
		BufferMove(proxyId);
	}
}

void b2BroadPhase::TouchProxy(int32 proxyId)
{
	BufferMove(proxyId);
}

void b2BroadPhase::BufferMove(int32 proxyId)
{
	m_moveBuffer.push_back(proxyId);
}

void b2BroadPhase::UnBufferMove(int32 proxyId)
{
	for (uint32 i = 0; i < m_moveBuffer.size(); ++i)
	{
		if (m_moveBuffer[i] == proxyId)
		{
			m_moveBuffer[i] = e_nullProxy;
		}
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
bool b2BroadPhasePerThreadData::QueryCallback(int32 proxyId)
{
	// A proxy cannot form a pair with itself.
	if (proxyId == m_queryProxyId)
	{
		return true;
	}

	b2Pair pair;
	pair.proxyIdA = b2Min(proxyId, m_queryProxyId);
	pair.proxyIdB = b2Max(proxyId, m_queryProxyId);

	m_pairBuffer.push_back(pair);

	return true;
}
