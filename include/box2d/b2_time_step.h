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
#ifndef B2_TIME_STEP_H
#define B2_TIME_STEP_H

#include "b2_api.h"
#include "b2_math.h"

/// Profiling data. Times are in milliseconds.
struct B2_API b2Profile
{
	float step;
	float collide;
	float solve;
	float solveTraversal;
	float solveInit;
	float solveVelocity;
	float solvePosition;
	float solveTOI;
	float solveTOIFindMinContact;
	float broadphase;
	float broadphaseSyncFixtures;
	float broadphaseFindContacts;
	float locking;
};

/// This is an internal structure.
struct B2_API b2TimeStep
{
	float dt;			// time step
	float inv_dt;		// inverse time step (0 if dt == 0).
	float dtRatio;	// dt * inv_dt0
	int32 velocityIterations;
	int32 positionIterations;
	bool warmStarting;
};

/// This is an internal structure.
struct B2_API b2Position
{
	b2Vec2 c;
	float a;
};

/// This is an internal structure.
struct B2_API b2Velocity
{
	b2Vec2 v;
	float w;
};

/// Solver Data
struct B2_API b2SolverData
{
	b2TimeStep step;
	b2Position* positions;
	b2Velocity* velocities;
	int32 threadId;
};

inline void b2AddProfile(b2Profile& dest, const b2Profile& src, float32 scale)
{
    dest.step += scale * src.step;
    dest.collide += scale * src.collide;
    dest.solve += scale * src.solve;
    dest.solveTraversal += scale * src.solveTraversal;
    dest.solveInit += scale * src.solveInit;
    dest.solveVelocity += scale * src.solveVelocity;
    dest.solvePosition += scale * src.solvePosition;
    dest.solveTOI += scale * src.solveTOI;
    dest.solveTOIFindMinContact += scale * src.solveTOIFindMinContact;
    dest.broadphase += scale * src.broadphase;
    dest.broadphaseSyncFixtures += scale * src.broadphaseSyncFixtures;
    dest.broadphaseFindContacts += scale * src.broadphaseFindContacts;
    dest.locking += scale * src.locking;
}

#endif
