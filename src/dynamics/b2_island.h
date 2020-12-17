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

#ifndef B2_ISLAND_H
#define B2_ISLAND_H

#include "Box2D/Common/b2GrowableArray.h"
#include "box2d/b2_body.h"
#include "box2d/b2_math.h"
#include "box2d/b2_time_step.h"

class b2Contact;
class b2Joint;
class b2StackAllocator;
class b2ContactListener;
struct b2ContactVelocityConstraint;
struct b2Profile;
struct b2DeferredPostSolve;

/// This is an internal class.
class b2Island
{
public:
    b2Island();
	b2Island(b2Body** bodies, b2Contact** contacts,
		b2Velocity* velocities, b2Position* positions);
	b2Island(int32 bodyCount, int32 contactCount, int32 jointCount,
		b2Body** bodies, b2Contact** contacts, b2Joint** joints,
		b2Velocity* velocities, b2Position* positions);

	void Clear()
	{
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}

	void Solve(b2Profile* profile, const b2TimeStep& step, const b2Vec2& gravity, b2StackAllocator* allocator,
		b2ContactListener* listener, uint32 threadId, bool allowSleep, b2GrowableArray<b2DeferredPostSolve>& postSolves);

	void SolveTOI(const b2TimeStep& subStep, int32 toiIndexA, int32 toiIndexB, b2StackAllocator* allocator,
		b2ContactListener* listener);

	void Add(b2Body* body)
	{
		body->SetIslandIndex(m_bodyCount, 0);
		m_bodies[m_bodyCount] = body;
		++m_bodyCount;
	}

	void Add(b2Contact* contact)
	{
		m_contacts[m_contactCount++] = contact;
	}

	void Add(b2Joint* joint)
	{
		m_joints[m_jointCount++] = joint;
	}

	template<bool isSingleThread>
	void Report(const b2ContactVelocityConstraint* constraints, b2ContactListener* listener, uint32 threadId,
		b2GrowableArray<b2DeferredPostSolve>* postSolves);

	b2Body** m_bodies;
	b2Contact** m_contacts;
	b2Joint** m_joints;

	b2Position* m_positions;
	b2Velocity* m_velocities;

	int32 m_bodyCount;
	int32 m_jointCount;
	int32 m_contactCount;
};

#endif
