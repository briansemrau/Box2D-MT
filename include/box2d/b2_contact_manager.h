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

#ifndef B2_CONTACT_MANAGER_H
#define B2_CONTACT_MANAGER_H

#include "b2_api.h"
#include "b2_broad_phase.h"

class b2BlockAllocator;
class b2Body;
class b2ContactFilter;
class b2ContactListener;
class b2StackAllocator;
class b2TaskExecutor;
class b2TaskGroup;
struct b2FixtureProxy;

struct B2_API b2DeferredContactCreate
{
	b2Fixture* fixtureA;
	b2Fixture* fixtureB;
	int32 indexA;
	int32 indexB;
	b2ContactProxyIds proxyIds;
};

struct B2_API b2DeferredMoveProxy
{
	b2AABB aabb;
	b2Vec2 displacement;
	int32 proxyId;
};

struct B2_API b2DeferredPreSolve
{
	b2Contact* contact;
	b2Manifold oldManifold;
};

struct B2_API b2DeferredPostSolve
{
	b2Contact* contact;
	b2ContactImpulse impulse;
};

/// These are used to sort deferred events so their effects are applied in a deterministic order.
bool b2ContactPointerLessThan(const b2Contact* l, const b2Contact* r);
bool b2DeferredContactCreateLessThan(const b2DeferredContactCreate& l, const b2DeferredContactCreate& r);
bool b2DeferredMoveProxyLessThan(const b2DeferredMoveProxy& l, const b2DeferredMoveProxy& r);
bool b2DeferredPreSolveLessThan(const b2DeferredPreSolve& l, const b2DeferredPreSolve& r);
bool b2DeferredPostSolveLessThan(const b2DeferredPostSolve& l, const b2DeferredPostSolve& r);

struct B2_API b2ContactManagerPerThreadData
{
	b2GrowableArray<b2Contact*> m_beginContacts;
	b2GrowableArray<b2Contact*> m_endContacts;
	b2GrowableArray<b2DeferredPreSolve> m_preSolves;
	b2GrowableArray<b2DeferredPostSolve> m_postSolves;
	b2GrowableArray<b2Contact*> m_awakes;
	b2GrowableArray<b2Contact*> m_destroys;
	b2GrowableArray<b2DeferredContactCreate> m_creates;
	b2GrowableArray<b2DeferredMoveProxy> m_moveProxies;
	b2Profile m_profile;

	uint8 _padding[b2_cacheLineSize];
};

// Delegate of b2World.
class B2_API b2ContactManager
{
public:
	b2ContactManager();

	// Broad-phase callback.
	void AddPair(void* proxyUserDataA, void* proxyUserDataB, uint32 threadId);

	// These are called from multithreaded tasks.
	void FindNewContacts(uint32 moveBegin, uint32 moveEnd, uint32 threadId);
	void Collide(uint32 contactsBegin, uint32 contactsEnd, uint32 threadId);
	void Destroy(b2Contact* contact);
	void SynchronizeFixtures(b2Body** bodies, uint32 count, uint32 threadId);

	// Finish multithreaded work with consistency sorting.
	void FinishFindNewContacts(b2TaskExecutor& executor, b2TaskGroup* taskGroup, b2StackAllocator& allocator);
	void FinishCollide(b2TaskExecutor& executor, b2TaskGroup* taskGroup, b2StackAllocator& allocator);
	void FinishSynchronizeFixtures(b2TaskExecutor& executor, b2TaskGroup* taskGroup, b2StackAllocator& allocator);
	void FinishSolve(b2TaskExecutor& executor, b2TaskGroup* taskGroup, b2StackAllocator& allocator);

	// Finish multithreaded work without consistency sorting.
	void FinishFindNewContacts();
	void FinishCollide();
	void FinishSynchronizeFixtures();
	void FinishSolve();

	// Contacts are partitioned, with TOI eligible contacts ordered before TOI ineligible
	// contacts. This speeds up traversal during TOI solving.
	b2Contact** GetToiBegin();
	b2Contact** GetNonToiBegin();
	uint32 GetNonToiCount();

	// Reorder contacts when TOI eligibility changes.
	void RecalculateToiCandidacy(b2Body* body);
	void RecalculateToiCandidacy(b2Fixture* fixture);

	// Update the active flag for this body's contacts.
	void RecalculateSleeping(b2Body* body);

	b2BroadPhase m_broadPhase;
	b2Contact* m_contactList;
	b2ContactFilter* m_contactFilter;
	b2ContactListener* m_contactListener;
	b2BlockAllocator* m_allocator;

	// This contacts array makes it easier to assign ranges of contacts to different tasks.
	// Note: TOI partitioning is also done in this array rather than in the contact list,
	// but it might be better to do that in the contact list.
	b2GrowableArray<b2Contact*> m_contacts;
	uint32 m_toiCount;

	b2ContactManagerPerThreadData m_perThreadData[b2_maxThreads];

	bool m_deferCreates;

private:
	static bool IsContactActive(b2Contact* contact);

	void ConsumeAwakes();
	void ConsumeCreate(const b2DeferredContactCreate& create);

	void RecalculateToiCandidacy(b2Contact* contact);
	void OnContactCreate(b2Contact* contact, b2ContactProxyIds proxyIds);
	void AddToContactArray(b2Contact* contact);
	void RemoveFromContactArray(b2Contact* contact);
	void AddToContactList(b2Contact* contact);
	void RemoveFromContactList(b2Contact* contact);

	void SanityCheck();
};

inline b2Contact** b2ContactManager::GetToiBegin()
{
	return m_contacts.begin();
}

inline b2Contact** b2ContactManager::GetNonToiBegin()
{
	return m_contacts.begin() + m_toiCount;
}

inline uint32 b2ContactManager::GetNonToiCount()
{
	return m_contacts.size() - m_toiCount;
}

#endif
