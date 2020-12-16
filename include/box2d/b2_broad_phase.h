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

#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

#include "b2_api.h"
#include "b2_settings.h"
#include "b2_collision.h"
#include "b2_dynamic_tree.h"

struct B2_API b2Pair
{
	int32 proxyIdA;
	int32 proxyIdB;
};

struct B2_API b2BroadPhasePerThreadData
{
	bool QueryCallback(int32 proxyId);

	b2GrowableArray<b2Pair> m_pairBuffer;
	int32 m_queryProxyId;

	uint8 m_padding[b2_cacheLineSize];
};

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
class B2_API b2BroadPhase
{
public:

	enum
	{
		e_nullProxy = -1
	};

	b2BroadPhase();
	~b2BroadPhase();

#ifdef b2_dynamicTreeOfTrees
	/// Destroy all proxies and set the sub-tree dimensions.
	void Reset(float subTreeWidth, float subTreeHeight);

	/// Visit every leaf in the base tree.
	template <typename T>
	void VisitBaseTree(T* callback) const;
#endif

	/// Create a proxy with an initial AABB. Pairs are not reported until
	/// UpdatePairs is called.
	int32 CreateProxy(const b2AABB& aabb, void* userData);

	/// Destroy a proxy. It is up to the client to remove any pairs.
	void DestroyProxy(int32 proxyId);

	/// Call MoveProxy as many times as you like, then when you are done
	/// call UpdatePairs to finalized the proxy pairs (for your time step).
	void MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement);

	/// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
	void TouchProxy(int32 proxyId);

	/// Get the fat AABB for a proxy.
	const b2AABB& GetFatAABB(int32 proxyId) const;

	/// Get user data from a proxy. Returns nullptr if the id is invalid.
	void* GetUserData(int32 proxyId) const;

	/// Test overlap of fat AABBs.
	bool TestOverlap(int32 proxyIdA, int32 proxyIdB) const;

	/// Get the number of proxies.
	int32 GetProxyCount() const;

	/// Update the pairs. This results in pair callbacks. This can only add pairs.
	/// Note: This can be called from multiple threads on separate ranges of the
	/// move buffer. After all threads have finished, ResetBuffers must be called
	/// from a single thread before the next call to UpdatePairs.
	template <typename T>
	void UpdatePairs(int32 moveBegin, int32 moveEnd, T* callback, uint32 threadId);

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	template <typename T>
	void Query(T* callback, const b2AABB& aabb, uint32 threadId);

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	template <typename T>
	void RayCast(T* callback, const b2RayCastInput& input, uint32 threadId);

	/// Get the height of the embedded tree.
	int32 GetTreeHeight() const;

	/// Get the balance of the embedded tree.
	int32 GetTreeBalance() const;

	/// Get the quality metric of the embedded tree.
	float GetTreeQuality() const;

	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const b2Vec2& newOrigin);

	/// Reset the pair buffers and move buffer.
	void ResetBuffers();

	/// Get the number of proxies in the move buffer.
	int32 GetMoveCount() const;

private:

	friend class b2DynamicTree;

	void BufferMove(int32 proxyId);
	void UnBufferMove(int32 proxyId);
#ifdef b2_dynamicTreeOfTrees
	b2DynamicTreeOfTrees m_tree;
#else
	b2DynamicTree m_tree;
#endif

	int32 m_proxyCount;
	b2GrowableArray<int32> m_moveBuffer;

	b2BroadPhasePerThreadData m_perThreadData[b2_maxThreads];
};

/// This is used to sort pairs.
inline bool b2PairLessThan(const b2Pair& pair1, const b2Pair& pair2)
{
	if (pair1.proxyIdA < pair2.proxyIdA)
	{
		return true;
	}

	if (pair1.proxyIdA == pair2.proxyIdA)
	{
		return pair1.proxyIdB < pair2.proxyIdB;
	}

	return false;
}

inline void* b2BroadPhase::GetUserData(int32 proxyId) const
{
	return m_tree.GetUserData(proxyId);
}

inline bool b2BroadPhase::TestOverlap(int32 proxyIdA, int32 proxyIdB) const
{
	const b2AABB& aabbA = m_tree.GetFatAABB(proxyIdA);
	const b2AABB& aabbB = m_tree.GetFatAABB(proxyIdB);
	return b2TestOverlap(aabbA, aabbB);
}

inline const b2AABB& b2BroadPhase::GetFatAABB(int32 proxyId) const
{
	return m_tree.GetFatAABB(proxyId);
}

inline int32 b2BroadPhase::GetProxyCount() const
{
	return m_proxyCount;
}

inline int32 b2BroadPhase::GetTreeHeight() const
{
	return m_tree.GetHeight();
}

inline int32 b2BroadPhase::GetTreeBalance() const
{
	return m_tree.GetMaxBalance();
}

inline float b2BroadPhase::GetTreeQuality() const
{
	return m_tree.GetAreaRatio();
}

template <typename T>
void b2BroadPhase::UpdatePairs(int32 moveBegin, int32 moveEnd, T* callback, uint32 threadId)
{
	b2BroadPhasePerThreadData* td = m_perThreadData + threadId;

	// Perform tree queries for all moving proxies.
	for (int32 i = moveBegin; i < moveEnd; ++i)
	{
		td->m_queryProxyId = m_moveBuffer[i];
		if (td->m_queryProxyId == e_nullProxy)
		{
			continue;
		}

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
		const b2AABB fatAABB = m_tree.GetFatAABB(td->m_queryProxyId);

		// Query the tree, create pairs and add them pair buffer.
#ifdef b2_dynamicTreeOfTrees
		m_tree.Query(td, fatAABB, threadId);
#else
		m_tree.Query(td, fatAABB);
#endif
	}

	// Sort the pair buffer to expose duplicates.
	std::sort(td->m_pairBuffer.data(), td->m_pairBuffer.data() + td->m_pairBuffer.size(), b2PairLessThan);

	// Send the pairs back to the client.
	uint32 i = 0;
	while (i < td->m_pairBuffer.size())
	{
		b2Pair& primaryPair = td->m_pairBuffer[i];

		void* userDataA = m_tree.GetUserData(primaryPair.proxyIdA);
		void* userDataB = m_tree.GetUserData(primaryPair.proxyIdB);

		callback->AddPair(userDataA, userDataB, threadId);

		++i;

		// Skip any duplicate pairs.
		while (i < td->m_pairBuffer.size())
		{
			b2Pair& pair = td->m_pairBuffer[i];
			if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB)
			{
				break;
			}
			++i;
		}
	}

	// Try to keep the tree balanced.
	//m_tree.Rebalance(4);
}

template <typename T>
inline void b2BroadPhase::Query(T* callback, const b2AABB& aabb, uint32 threadId)
{
#ifdef b2_dynamicTreeOfTrees
	m_tree.Query(callback, aabb, threadId);
#else
	B2_NOT_USED(threadId);
	m_tree.Query(callback, aabb);
#endif
}

template <typename T>
inline void b2BroadPhase::RayCast(T* callback, const b2RayCastInput& input, uint32 threadId)
{
#ifdef b2_dynamicTreeOfTrees
	m_tree.RayCast(callback, input, threadId);
#else
	B2_NOT_USED(threadId);
	m_tree.RayCast(callback, input);
#endif
}

#ifdef b2_dynamicTreeOfTrees
template <typename T>
inline void b2BroadPhase::VisitBaseTree(T* callback) const
{
	m_tree.VisitBaseTree(callback);
}
#endif

inline void b2BroadPhase::ShiftOrigin(const b2Vec2& newOrigin)
{
	m_tree.ShiftOrigin(newOrigin);
}

inline void b2BroadPhase::ResetBuffers()
{
	m_moveBuffer.clear();

	for (int32 i = 0; i < b2_maxThreads; ++i)
	{
		m_perThreadData[i].m_pairBuffer.clear();
	}
}

inline int32 b2BroadPhase::GetMoveCount() const
{
	return m_moveBuffer.size();
}

#endif
