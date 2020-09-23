/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
import { b2Assert, b2_angularSleepTolerance, b2_linearSleepTolerance, b2_maxFloat, b2_maxRotation, b2_maxRotationSquared, b2_maxTranslation, b2_maxTranslationSquared, b2_timeToSleep, } from '../common/b2Settings';
import { b2Abs, b2MaxInt, b2Min, b2Vec2 } from '../common/b2Math';
import { b2Timer } from '../common/b2Timer';
import { b2ContactSolver, b2ContactSolverDef, } from './contacts/b2ContactSolver';
import { b2Position, b2SolverData, b2Velocity } from './b2TimeStep';
import { b2ContactImpulse, b2ContactListener } from './b2WorldCallbacks';
/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/
const s_timer = new b2Timer();
const s_solverData = new b2SolverData();
const s_contactSolverDef = new b2ContactSolverDef();
const s_contactSolver = new b2ContactSolver();
const s_translation = new b2Vec2();
/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/
/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/
export class b2Island {
    constructor() {
        this.m_listener = b2ContactListener.b2_defaultListener;
        this.m_bodies = [null]; // TODO: b2Settings
        this.m_contacts = [null]; // TODO: b2Settings
        this.m_joints = [null]; // TODO: b2Settings
        this.m_positions = b2Position.MakeArray(1024); // TODO: b2Settings
        this.m_velocities = b2Velocity.MakeArray(1024); // TODO: b2Settings
        this.m_bodyCount = 0;
        this.m_jointCount = 0;
        this.m_contactCount = 0;
        this.m_bodyCapacity = 0;
        this.m_contactCapacity = 0;
        this.m_jointCapacity = 0;
    }
    Initialize(bodyCapacity, contactCapacity, jointCapacity, listener) {
        this.m_bodyCapacity = bodyCapacity | 0;
        this.m_contactCapacity = contactCapacity | 0;
        this.m_jointCapacity = jointCapacity | 0;
        this.m_bodyCount = 0;
        this.m_contactCount = 0;
        this.m_jointCount = 0;
        this.m_listener = listener;
        // TODO:
        while (this.m_bodies.length < bodyCapacity) {
            this.m_bodies[this.m_bodies.length] = null;
        }
        // TODO:
        while (this.m_contacts.length < contactCapacity) {
            this.m_contacts[this.m_contacts.length] = null;
        }
        // TODO:
        while (this.m_joints.length < jointCapacity) {
            this.m_joints[this.m_joints.length] = null;
        }
        // TODO:
        if (this.m_positions.length < bodyCapacity) {
            const new_length = b2MaxInt(this.m_positions.length << 1, bodyCapacity);
            while (this.m_positions.length < new_length) {
                this.m_positions[this.m_positions.length] = new b2Position();
            }
        }
        // TODO:
        if (this.m_velocities.length < bodyCapacity) {
            const new_length = b2MaxInt(this.m_velocities.length << 1, bodyCapacity);
            while (this.m_velocities.length < new_length) {
                this.m_velocities[this.m_velocities.length] = new b2Velocity();
            }
        }
    }
    Clear() {
        this.m_bodyCount = 0;
        this.m_contactCount = 0;
        this.m_jointCount = 0;
    }
    AddBody(body) {
        !!B2_DEBUG && b2Assert(this.m_bodyCount < this.m_bodyCapacity);
        body.m_islandIndex = this.m_bodyCount;
        this.m_bodies[this.m_bodyCount++] = body;
    }
    AddContact(contact) {
        !!B2_DEBUG && b2Assert(this.m_contactCount < this.m_contactCapacity);
        this.m_contacts[this.m_contactCount++] = contact;
    }
    AddJoint(joint) {
        !!B2_DEBUG && b2Assert(this.m_jointCount < this.m_jointCapacity);
        this.m_joints[this.m_jointCount++] = joint;
    }
    Solve(profile, step, gravity, allowSleep) {
        const timer = s_timer.Reset();
        const bodyCount = this.m_bodyCount;
        const m_bodies = this.m_bodies;
        const m_positions = this.m_positions;
        const m_velocities = this.m_velocities;
        let flags = 0;
        const h = step.dt;
        // Integrate velocities and apply damping. Initialize the body state.
        for (let i = 0; i < bodyCount; ++i) {
            const b = m_bodies[i];
            // const c: b2Vec2 =
            m_positions[i].c.Copy(b.m_sweep.c);
            const a = b.m_sweep.a;
            const v = m_velocities[i].v.Copy(b.m_linearVelocity);
            let w = b.m_angularVelocity;
            // Store positions for continuous collision.
            b.m_sweep.c0.Copy(b.m_sweep.c);
            b.m_sweep.a0 = b.m_sweep.a;
            if (b.m_type === 2 /* b2_dynamicBody */) {
                // Integrate velocities.
                v.x += h * (b.m_gravityScale * gravity.x + b.m_invMass * b.m_force.x);
                v.y += h * (b.m_gravityScale * gravity.y + b.m_invMass * b.m_force.y);
                w += h * b.m_invI * b.m_torque;
                // Apply damping.
                // ODE: dv/dt + c * v = 0
                // Solution: v(t) = v0 * exp(-c * t)
                // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                // v2 = exp(-c * dt) * v1
                // Pade approximation:
                // v2 = v1 * 1 / (1 + c * dt)
                v.SelfMul(1.0 / (1.0 + h * b.m_linearDamping));
                w *= 1.0 / (1.0 + h * b.m_angularDamping);
            }
            // this.m_positions[i].c = c;
            m_positions[i].a = a;
            // this.m_velocities[i].v = v;
            m_velocities[i].w = w;
        }
        timer.Reset();
        // Solver data
        const solverData = s_solverData;
        solverData.step.Copy(step);
        solverData.positions = this.m_positions;
        solverData.velocities = this.m_velocities;
        // Initialize velocity constraints.
        const contactSolverDef = s_contactSolverDef;
        contactSolverDef.step.Copy(step);
        contactSolverDef.contacts = this.m_contacts;
        contactSolverDef.count = this.m_contactCount;
        contactSolverDef.positions = this.m_positions;
        contactSolverDef.velocities = this.m_velocities;
        const contactSolver = s_contactSolver.Initialize(contactSolverDef);
        contactSolver.InitializeVelocityConstraints();
        if (step.warmStarting) {
            contactSolver.WarmStart();
        }
        this._SolveInitJoints(solverData);
        profile.solveInit += timer.GetMilliseconds();
        // Solve velocity constraints.
        timer.Reset();
        for (let i = 0; i < step.velocityIterations; ++i) {
            for (let j = 0; j < this.m_jointCount; ++j) {
                this.m_joints[j].SolveVelocityConstraints(solverData);
            }
            contactSolver.SolveVelocityConstraints();
        }
        // Store impulses for warm starting
        contactSolver.StoreImpulses();
        profile.solveVelocity += timer.GetMilliseconds();
        // Integrate positions.
        for (let i = 0; i < bodyCount; ++i) {
            const c = m_positions[i].c;
            let a = m_positions[i].a;
            const v = m_velocities[i].v;
            let w = m_velocities[i].w;
            // Check for large velocities
            const translation = b2Vec2.MulSV(h, v, s_translation);
            if (b2Vec2.DotVV(translation, translation) > b2_maxTranslationSquared) {
                const ratio = b2_maxTranslation / translation.Length();
                v.SelfMul(ratio);
            }
            const rotation = h * w;
            if (rotation * rotation > b2_maxRotationSquared) {
                const ratio = b2_maxRotation / b2Abs(rotation);
                w *= ratio;
            }
            // Integrate
            c.x += h * v.x;
            c.y += h * v.y;
            a += h * w;
            // this.m_positions[i].c = c;
            m_positions[i].a = a;
            // this.m_velocities[i].v = v;
            m_velocities[i].w = w;
        }
        // Solve position constraints
        timer.Reset();
        flags |= this._SolvePositionsConstraits(step.positionIterations, contactSolver, solverData)
            ? 1
            : 0;
        // Copy state buffers back to the bodies
        for (let i = 0; i < bodyCount; ++i) {
            const body = m_bodies[i];
            body.m_sweep.c.Copy(m_positions[i].c);
            body.m_sweep.a = m_positions[i].a;
            body.m_linearVelocity.Copy(m_velocities[i].v);
            body.m_angularVelocity = m_velocities[i].w;
            body.SynchronizeTransform();
        }
        profile.solvePosition += timer.GetMilliseconds();
        this.Report(contactSolver.m_velocityConstraints);
        flags |= allowSleep && this._UpdateSleepTime(h) ? 2 : 0;
        return flags;
    }
    _SolvePositionsConstraits(iterations, contactSolver, solverData) {
        const jointsCount = this.m_jointCount;
        const joints = this.m_joints;
        for (let i = 0; i < iterations; ++i) {
            const contactsOkay = contactSolver.SolvePositionConstraints();
            let jointsOkay = true;
            for (let j = 0; j < jointsCount; ++j) {
                const jointOkay = joints[j].SolvePositionConstraints(solverData);
                jointsOkay = jointsOkay && jointOkay;
            }
            if (contactsOkay && jointsOkay) {
                // Exit early if the position errors are small.
                return true;
            }
        }
        return false;
    }
    _SolveInitJoints(solverData) {
        const count = this.m_jointCount;
        const list = this.m_joints;
        for (let i = 0; i < count; ++i) {
            list[i].InitVelocityConstraints(solverData);
        }
    }
    _UpdateSleepTime(h) {
        let minSleepTime = b2_maxFloat;
        const linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
        const angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;
        const count = this.m_bodyCount;
        const list = this.m_bodies;
        for (let i = 0; i < count; ++i) {
            const b = list[i];
            if (b.GetType() === 0 /* b2_staticBody */) {
                continue;
            }
            if (!b.m_autoSleepFlag ||
                b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
                b2Vec2.DotVV(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
                b.m_sleepTime = 0;
                minSleepTime = 0;
            }
            else {
                b.m_sleepTime += h;
                minSleepTime = b2Min(minSleepTime, b.m_sleepTime);
            }
        }
        return minSleepTime >= b2_timeToSleep;
    }
    SleepAll() {
        for (let i = 0; i < this.m_bodyCount; ++i) {
            this.m_bodies[i].SetAwake(false);
        }
    }
    SolveTOI(subStep, toiIndexA, toiIndexB) {
        !!B2_DEBUG && b2Assert(toiIndexA < this.m_bodyCount);
        !!B2_DEBUG && b2Assert(toiIndexB < this.m_bodyCount);
        // Initialize the body state.
        for (let i = 0; i < this.m_bodyCount; ++i) {
            const b = this.m_bodies[i];
            this.m_positions[i].c.Copy(b.m_sweep.c);
            this.m_positions[i].a = b.m_sweep.a;
            this.m_velocities[i].v.Copy(b.m_linearVelocity);
            this.m_velocities[i].w = b.m_angularVelocity;
        }
        const contactSolverDef = s_contactSolverDef;
        contactSolverDef.contacts = this.m_contacts;
        contactSolverDef.count = this.m_contactCount;
        contactSolverDef.step.Copy(subStep);
        contactSolverDef.positions = this.m_positions;
        contactSolverDef.velocities = this.m_velocities;
        const contactSolver = s_contactSolver.Initialize(contactSolverDef);
        // Solve position constraints.
        for (let i = 0; i < subStep.positionIterations; ++i) {
            const contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
            if (contactsOkay) {
                break;
            }
        }
        /*
            #if 0
              // Is the new position really safe?
              for (int32 i = 0; i < this.m_contactCount; ++i) {
                b2Contact* c = this.m_contacts[i];
                b2Fixture* fA = c.GetFixtureA();
                b2Fixture* fB = c.GetFixtureB();
    
                b2Body* bA = fA.GetBody();
                b2Body* bB = fB.GetBody();
    
                int32 indexA = c.GetChildIndexA();
                int32 indexB = c.GetChildIndexB();
    
                b2DistanceInput input;
                input.proxyA.Set(fA.GetShape(), indexA);
                input.proxyB.Set(fB.GetShape(), indexB);
                input.transformA = bA.GetTransform();
                input.transformB = bB.GetTransform();
                input.useRadii = false;
    
                b2DistanceOutput output;
                b2SimplexCache cache;
                cache.count = 0;
                b2Distance(&output, &cache, &input);
    
                if (output.distance === 0 || cache.count === 3) {
                  cache.count += 0;
                }
              }
            #endif
            */
        // Leap of faith to new safe state.
        this.m_bodies[toiIndexA].m_sweep.c0.Copy(this.m_positions[toiIndexA].c);
        this.m_bodies[toiIndexA].m_sweep.a0 = this.m_positions[toiIndexA].a;
        this.m_bodies[toiIndexB].m_sweep.c0.Copy(this.m_positions[toiIndexB].c);
        this.m_bodies[toiIndexB].m_sweep.a0 = this.m_positions[toiIndexB].a;
        // No warm starting is needed for TOI events because warm
        // starting impulses were applied in the discrete solver.
        contactSolver.InitializeVelocityConstraints();
        // Solve velocity constraints.
        for (let i = 0; i < subStep.velocityIterations; ++i) {
            contactSolver.SolveVelocityConstraints();
        }
        // Don't store the TOI contact forces for warm starting
        // because they can be quite large.
        const h = subStep.dt;
        // Integrate positions
        for (let i = 0; i < this.m_bodyCount; ++i) {
            const c = this.m_positions[i].c;
            let a = this.m_positions[i].a;
            const v = this.m_velocities[i].v;
            let w = this.m_velocities[i].w;
            // Check for large velocities
            const translation = b2Vec2.MulSV(h, v, s_translation);
            if (b2Vec2.DotVV(translation, translation) > b2_maxTranslationSquared) {
                const ratio = b2_maxTranslation / translation.Length();
                v.SelfMul(ratio);
            }
            const rotation = h * w;
            if (rotation * rotation > b2_maxRotationSquared) {
                const ratio = b2_maxRotation / b2Abs(rotation);
                w *= ratio;
            }
            // Integrate
            c.SelfMulAdd(h, v);
            a += h * w;
            // this.m_positions[i].c = c;
            this.m_positions[i].a = a;
            // this.m_velocities[i].v = v;
            this.m_velocities[i].w = w;
            // Sync bodies
            const body = this.m_bodies[i];
            body.m_sweep.c.Copy(c);
            body.m_sweep.a = a;
            body.m_linearVelocity.Copy(v);
            body.m_angularVelocity = w;
            body.SynchronizeTransform();
        }
        this.Report(contactSolver.m_velocityConstraints);
    }
    Report(constraints) {
        for (let i = 0; i < this.m_contactCount; ++i) {
            const c = this.m_contacts[i];
            if (!c) {
                continue;
            }
            const vc = constraints[i];
            const impulse = b2Island.s_impulse;
            impulse.count = vc.pointCount;
            for (let j = 0; j < vc.pointCount; ++j) {
                impulse.normalImpulses[j] = vc.points[j].normalImpulse;
                impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
            }
            this.m_listener.PostSolve(c, impulse);
        }
    }
}
b2Island.s_impulse = new b2ContactImpulse();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJJc2xhbmQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi9zcmMvZHluYW1pY3MvYjJJc2xhbmQudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7QUFFSCxPQUFPLEVBQ0wsUUFBUSxFQUNSLHdCQUF3QixFQUN4Qix1QkFBdUIsRUFDdkIsV0FBVyxFQUNYLGNBQWMsRUFDZCxxQkFBcUIsRUFDckIsaUJBQWlCLEVBQ2pCLHdCQUF3QixFQUN4QixjQUFjLEdBQ2YsTUFBTSxzQkFBc0IsQ0FBQztBQUM5QixPQUFPLEVBQUUsS0FBSyxFQUFFLFFBQVEsRUFBRSxLQUFLLEVBQUUsTUFBTSxFQUFFLE1BQU0sa0JBQWtCLENBQUM7QUFDbEUsT0FBTyxFQUFFLE9BQU8sRUFBRSxNQUFNLG1CQUFtQixDQUFDO0FBRTVDLE9BQU8sRUFDTCxlQUFlLEVBQ2Ysa0JBQWtCLEdBRW5CLE1BQU0sNEJBQTRCLENBQUM7QUFHcEMsT0FBTyxFQUFFLFVBQVUsRUFBYSxZQUFZLEVBQWMsVUFBVSxFQUFFLE1BQU0sY0FBYyxDQUFDO0FBQzNGLE9BQU8sRUFBRSxnQkFBZ0IsRUFBRSxpQkFBaUIsRUFBRSxNQUFNLG9CQUFvQixDQUFDO0FBRXpFOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7OztFQTZFRTtBQUVGLE1BQU0sT0FBTyxHQUFHLElBQUksT0FBTyxFQUFFLENBQUM7QUFDOUIsTUFBTSxZQUFZLEdBQUcsSUFBSSxZQUFZLEVBQUUsQ0FBQztBQUN4QyxNQUFNLGtCQUFrQixHQUFHLElBQUksa0JBQWtCLEVBQUUsQ0FBQztBQUNwRCxNQUFNLGVBQWUsR0FBRyxJQUFJLGVBQWUsRUFBRSxDQUFDO0FBQzlDLE1BQU0sYUFBYSxHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFFbkM7Ozs7Ozs7Ozs7Ozs7O0VBY0U7QUFFRjs7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QUMvQyxLQUFLLElBQUksQ0FBQyxHQUFHLENBQUMsRUFBRSxDQUFDLEdBQUcsSUFBSSxDQUFDLGNBQWMsRUFBRSxFQUFFLENBQUMsRUFBRTtZQUM1QyxNQUFNLENBQUMsR0FBYyxJQUFJLENBQUMsVUFBVSxDQUFDLENBQUMsQ0FBQyxDQUFDO1lBRXhDLElBQUksQ0FBQyxDQUFDLEVBQUU7Z0JBQ04sU0FBUzthQUNWO1lBRUQsTUFBTSxFQUFFLEdBQUcsV0FBVyxDQUFDLENBQUMsQ0FBQyxDQUFDO1lBRTFCLE1BQU0sT0FBTyxHQUFHLFFBQVEsQ0FBQyxTQUFTLENBQUM7WUFDbkMsT0FBTyxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUMsVUFBVSxDQUFDO1lBQzlCLEtBQUssSUFBSSxDQUFDLEdBQUcsQ0FBQyxFQUFFLENBQUMsR0FBRyxFQUFFLENBQUMsVUFBVSxFQUFFLEVBQUUsQ0FBQyxFQUFFO2dCQUN0QyxPQUFPLENBQUMsY0FBYyxDQUFDLENBQUMsQ0FBQyxHQUFHLEVBQUUsQ0FBQyxNQUFNLENBQUMsQ0FBQyxDQUFDLENBQUMsYUFBYSxDQUFDO2dCQUN2RCxPQUFPLENBQUMsZUFBZSxDQUFDLENBQUMsQ0FBQyxHQUFHLEVBQUUsQ0FBQyxNQUFNLENBQUMsQ0FBQyxDQUFDLENBQUMsY0FBYyxDQUFDO2FBQzFEO1lBRUQsSUFBSSxDQUFDLFVBQVUsQ0FBQyxTQUFTLENBQUMsQ0FBQyxFQUFFLE9BQU8sQ0FBQyxDQUFDO1NBQ3ZDO0lBQ0gsQ0FBQzs7QUFyQmMsa0JBQVMsR0FBRyxJQUFJLGdCQUFnQixFQUFFLENBQUMiLCJzb3VyY2VzQ29udGVudCI6WyIvKlxyXG4gKiBDb3B5cmlnaHQgKGMpIDIwMDYtMjAwOSBFcmluIENhdHRvIGh0dHA6Ly93d3cuYm94MmQub3JnXHJcbiAqXHJcbiAqIFRoaXMgc29mdHdhcmUgaXMgcHJvdmlkZWQgJ2FzLWlzJywgd2l0aG91dCBhbnkgZXhwcmVzcyBvciBpbXBsaWVkXHJcbiAqIHdhcnJhbnR5LiAgSW4gbm8gZXZlbnQgd2lsbCB0aGUgYXV0aG9ycyBiZSBoZWxkIGxpYWJsZSBmb3IgYW55IGRhbWFnZXNcclxuICogYXJpc2luZyBmcm9tIHRoZSB1c2Ugb2YgdGhpcyBzb2Z0d2FyZS5cclxuICogUGVybWlzc2lvbiBpcyBncmFudGVkIHRvIGFueW9uZSB0byB1c2UgdGhpcyBzb2Z0d2FyZSBmb3IgYW55IHB1cnBvc2UsXHJcbiAqIGluY2x1ZGluZyBjb21tZXJjaWFsIGFwcGxpY2F0aW9ucywgYW5kIHRvIGFsdGVyIGl0IGFuZCByZWRpc3RyaWJ1dGUgaXRcclxuICogZnJlZWx5LCBzdWJqZWN0IHRvIHRoZSBmb2xsb3dpbmcgcmVzdHJpY3Rpb25zOlxyXG4gKiAxLiBUaGUgb3JpZ2luIG9mIHRoaXMgc29mdHdhcmUgbXVzdCBub3QgYmUgbWlzcmVwcmVzZW50ZWQ7IHlvdSBtdXN0IG5vdFxyXG4gKiBjbGFpbSB0aGF0IHlvdSB3cm90ZSB0aGUgb3JpZ2luYWwgc29mdHdhcmUuIElmIHlvdSB1c2UgdGhpcyBzb2Z0d2FyZVxyXG4gKiBpbiBhIHByb2R1Y3QsIGFuIGFja25vd2xlZGdtZW50IGluIHRoZSBwcm9kdWN0IGRvY3VtZW50YXRpb24gd291bGQgYmVcclxuICogYXBwcmVjaWF0ZWQgYnV0IGlzIG5vdCByZXF1aXJlZC5cclxuICogMi4gQWx0ZXJlZCBzb3VyY2UgdmVyc2lvbnMgbXVzdCBiZSBwbGFpbmx5IG1hcmtlZCBhcyBzdWNoLCBhbmQgbXVzdCBub3QgYmVcclxuICogbWlzcmVwcmVzZW50ZWQgYXMgYmVpbmcgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLlxyXG4gKiAzLiBUaGlzIG5vdGljZSBtYXkgbm90IGJlIHJlbW92ZWQgb3IgYWx0ZXJlZCBmcm9tIGFueSBzb3VyY2UgZGlzdHJpYnV0aW9uLlxyXG4gKi9cclxuXHJcbmltcG9ydCB7XHJcbiAgYjJBc3NlcnQsXHJcbiAgYjJfYW5ndWxhclNsZWVwVG9sZXJhbmNlLFxyXG4gIGIyX2xpbmVhclNsZWVwVG9sZXJhbmNlLFxyXG4gIGIyX21heEZsb2F0LFxyXG4gIGIyX21heFJvdGF0aW9uLFxyXG4gIGIyX21heFJvdGF0aW9uU3F1YXJlZCxcclxuICBiMl9tYXhUcmFuc2xhdGlvbixcclxuICBiMl9tYXhUcmFuc2xhdGlvblNxdWFyZWQsXHJcbiAgYjJfdGltZVRvU2xlZXAsXHJcbn0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQgeyBiMkFicywgYjJNYXhJbnQsIGIyTWluLCBiMlZlYzIgfSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJUaW1lciB9IGZyb20gJy4uL2NvbW1vbi9iMlRpbWVyJztcclxuaW1wb3J0IHsgYjJDb250YWN0IH0gZnJvbSAnLi9jb250YWN0cy9iMkNvbnRhY3QnO1xyXG5pbXBvcnQge1xyXG4gIGIyQ29udGFjdFNvbHZlcixcclxuICBiMkNvbnRhY3RTb2x2ZXJEZWYsXHJcbiAgYjJDb250YWN0VmVsb2NpdHlDb25zdHJhaW50LFxyXG59IGZyb20gJy4vY29udGFjdHMvYjJDb250YWN0U29sdmVyJztcclxuaW1wb3J0IHsgYjJKb2ludCB9IGZyb20gJy4vam9pbnRzL2IySm9pbnQnO1xyXG5pbXBvcnQgeyBiMkJvZHksIGIyQm9keVR5cGUgfSBmcm9tICcuL2IyQm9keSc7XHJcbmltcG9ydCB7IGIyUG9zaXRpb24sIGIyUHJvZmlsZSwgYjJTb2x2ZXJEYXRhLCBiMlRpbWVTdGVwLCBiMlZlbG9jaXR5IH0gZnJvbSAnLi9iMlRpbWVTdGVwJztcclxuaW1wb3J0IHsgYjJDb250YWN0SW1wdWxzZSwgYjJDb250YWN0TGlzdGVuZXIgfSBmcm9tICcuL2IyV29ybGRDYWxsYmFja3MnO1xyXG5cclxuLypcclxuUG9zaXRpb24gQ29ycmVjdGlvbiBOb3Rlc1xyXG49PT09PT09PT09PT09PT09PT09PT09PT09XHJcbkkgdHJpZWQgdGhlIHNldmVyYWwgYWxnb3JpdGhtcyBmb3IgcG9zaXRpb24gY29ycmVjdGlvbiBvZiB0aGUgMkQgcmV2b2x1dGUgam9pbnQuXHJcbkkgbG9va2VkIGF0IHRoZXNlIHN5c3RlbXM6XHJcbi0gc2ltcGxlIHBlbmR1bHVtICgxbSBkaWFtZXRlciBzcGhlcmUgb24gbWFzc2xlc3MgNW0gc3RpY2spIHdpdGggaW5pdGlhbCBhbmd1bGFyIHZlbG9jaXR5IG9mIDEwMCByYWQvcy5cclxuLSBzdXNwZW5zaW9uIGJyaWRnZSB3aXRoIDMwIDFtIGxvbmcgcGxhbmtzIG9mIGxlbmd0aCAxbS5cclxuLSBtdWx0aS1saW5rIGNoYWluIHdpdGggMzAgMW0gbG9uZyBsaW5rcy5cclxuXHJcbkhlcmUgYXJlIHRoZSBhbGdvcml0aG1zOlxyXG5cclxuQmF1bWdhcnRlIC0gQSBmcmFjdGlvbiBvZiB0aGUgcG9zaXRpb24gZXJyb3IgaXMgYWRkZWQgdG8gdGhlIHZlbG9jaXR5IGVycm9yLiBUaGVyZSBpcyBub1xyXG5zZXBhcmF0ZSBwb3NpdGlvbiBzb2x2ZXIuXHJcblxyXG5Qc2V1ZG8gVmVsb2NpdGllcyAtIEFmdGVyIHRoZSB2ZWxvY2l0eSBzb2x2ZXIgYW5kIHBvc2l0aW9uIGludGVncmF0aW9uLFxyXG50aGUgcG9zaXRpb24gZXJyb3IsIEphY29iaWFuLCBhbmQgZWZmZWN0aXZlIG1hc3MgYXJlIHJlY29tcHV0ZWQuIFRoZW5cclxudGhlIHZlbG9jaXR5IGNvbnN0cmFpbnRzIGFyZSBzb2x2ZWQgd2l0aCBwc2V1ZG8gdmVsb2NpdGllcyBhbmQgYSBmcmFjdGlvblxyXG5vZiB0aGUgcG9zaXRpb24gZXJyb3IgaXMgYWRkZWQgdG8gdGhlIHBzZXVkbyB2ZWxvY2l0eSBlcnJvci4gVGhlIHBzZXVkb1xyXG52ZWxvY2l0aWVzIGFyZSBpbml0aWFsaXplZCB0byB6ZXJvIGFuZCB0aGVyZSBpcyBubyB3YXJtLXN0YXJ0aW5nLiBBZnRlclxyXG50aGUgcG9zaXRpb24gc29sdmVyLCB0aGUgcHNldWRvIHZlbG9jaXRpZXMgYXJlIGFkZGVkIHRvIHRoZSBwb3NpdGlvbnMuXHJcblRoaXMgaXMgYWxzbyBjYWxsZWQgdGhlIEZpcnN0IE9yZGVyIFdvcmxkIG1ldGhvZCBvciB0aGUgUG9zaXRpb24gTENQIG1ldGhvZC5cclxuXHJcbk1vZGlmaWVkIE5vbmxpbmVhciBHYXVzcy1TZWlkZWwgKE5HUykgLSBMaWtlIFBzZXVkbyBWZWxvY2l0aWVzIGV4Y2VwdCB0aGVcclxucG9zaXRpb24gZXJyb3IgaXMgcmUtY29tcHV0ZWQgZm9yIGVhY2ggY29uc3RyYWludCBhbmQgdGhlIHBvc2l0aW9ucyBhcmUgdXBkYXRlZFxyXG5hZnRlciB0aGUgY29uc3RyYWludCBpcyBzb2x2ZWQuIFRoZSByYWRpdXMgdmVjdG9ycyAoYWthIEphY29iaWFucykgYXJlXHJcbnJlLWNvbXB1dGVkIHRvbyAob3RoZXJ3aXNlIHRoZSBhbGdvcml0aG0gaGFzIGhvcnJpYmxlIGluc3RhYmlsaXR5KS4gVGhlIHBzZXVkb1xyXG52ZWxvY2l0eSBzdGF0ZXMgYXJlIG5vdCBuZWVkZWQgYmVjYXVzZSB0aGV5IGFyZSBlZmZlY3RpdmVseSB6ZXJvIGF0IHRoZSBiZWdpbm5pbmdcclxub2YgZWFjaCBpdGVyYXRpb24uIFNpbmNlIHdlIGhhdmUgdGhlIGN1cnJlbnQgcG9zaXRpb24gZXJyb3IsIHdlIGFsbG93IHRoZVxyXG5pdGVyYXRpb25zIHRvIHRlcm1pbmF0ZSBlYXJseSBpZiB0aGUgZXJyb3IgYmVjb21lcyBzbWFsbGVyIHRoYW4gYjJfbGluZWFyU2xvcC5cclxuXHJcbkZ1bGwgTkdTIG9yIGp1c3QgTkdTIC0gTGlrZSBNb2RpZmllZCBOR1MgZXhjZXB0IHRoZSBlZmZlY3RpdmUgbWFzcyBhcmUgcmUtY29tcHV0ZWRcclxuZWFjaCB0aW1lIGEgY29uc3RyYWludCBpcyBzb2x2ZWQuXHJcblxyXG5IZXJlIGFyZSB0aGUgcmVzdWx0czpcclxuQmF1bWdhcnRlIC0gdGhpcyBpcyB0aGUgY2hlYXBlc3QgYWxnb3JpdGhtIGJ1dCBpdCBoYXMgc29tZSBzdGFiaWxpdHkgcHJvYmxlbXMsXHJcbmVzcGVjaWFsbHkgd2l0aCB0aGUgYnJpZGdlLiBUaGUgY2hhaW4gbGlua3Mgc2VwYXJhdGUgZWFzaWx5IGNsb3NlIHRvIHRoZSByb290XHJcbmFuZCB0aGV5IGppdHRlciBhcyB0aGV5IHN0cnVnZ2xlIHRvIHB1bGwgdG9nZXRoZXIuIFRoaXMgaXMgb25lIG9mIHRoZSBtb3N0IGNvbW1vblxyXG5tZXRob2RzIGluIHRoZSBmaWVsZC4gVGhlIGJpZyBkcmF3YmFjayBpcyB0aGF0IHRoZSBwb3NpdGlvbiBjb3JyZWN0aW9uIGFydGlmaWNpYWxseVxyXG5hZmZlY3RzIHRoZSBtb21lbnR1bSwgdGh1cyBsZWFkaW5nIHRvIGluc3RhYmlsaXRpZXMgYW5kIGZhbHNlIGJvdW5jZS4gSSB1c2VkIGFcclxuYmlhcyBmYWN0b3Igb2YgMC4yLiBBIGxhcmdlciBiaWFzIGZhY3RvciBtYWtlcyB0aGUgYnJpZGdlIGxlc3Mgc3RhYmxlLCBhIHNtYWxsZXJcclxuZmFjdG9yIG1ha2VzIGpvaW50cyBhbmQgY29udGFjdHMgbW9yZSBzcG9uZ3kuXHJcblxyXG5Qc2V1ZG8gVmVsb2NpdGllcyAtIHRoZSBpcyBtb3JlIHN0YWJsZSB0aGFuIHRoZSBCYXVtZ2FydGUgbWV0aG9kLiBUaGUgYnJpZGdlIGlzXHJcbnN0YWJsZS4gSG93ZXZlciwgam9pbnRzIHN0aWxsIHNlcGFyYXRlIHdpdGggbGFyZ2UgYW5ndWxhciB2ZWxvY2l0aWVzLiBEcmFnIHRoZVxyXG5zaW1wbGUgcGVuZHVsdW0gaW4gYSBjaXJjbGUgcXVpY2tseSBhbmQgdGhlIGpvaW50IHdpbGwgc2VwYXJhdGUuIFRoZSBjaGFpbiBzZXBhcmF0ZXNcclxuZWFzaWx5IGFuZCBkb2VzIG5vdCByZWNvdmVyLiBJIHVzZWQgYSBiaWFzIGZhY3RvciBvZiAwLjIuIEEgbGFyZ2VyIHZhbHVlIGxlYWQgdG9cclxudGhlIGJyaWRnZSBjb2xsYXBzaW5nIHdoZW4gYSBoZWF2eSBjdWJlIGRyb3BzIG9uIGl0LlxyXG5cclxuTW9kaWZpZWQgTkdTIC0gdGhpcyBhbGdvcml0aG0gaXMgYmV0dGVyIGluIHNvbWUgd2F5cyB0aGFuIEJhdW1nYXJ0ZSBhbmQgUHNldWRvXHJcblZlbG9jaXRpZXMsIGJ1dCBpbiBvdGhlciB3YXlzIGl0IGlzIHdvcnNlLiBUaGUgYnJpZGdlIGFuZCBjaGFpbiBhcmUgbXVjaCBtb3JlXHJcbnN0YWJsZSwgYnV0IHRoZSBzaW1wbGUgcGVuZHVsdW0gZ29lcyB1bnN0YWJsZSBhdCBoaWdoIGFuZ3VsYXIgdmVsb2NpdGllcy5cclxuXHJcbkZ1bGwgTkdTIC0gc3RhYmxlIGluIGFsbCB0ZXN0cy4gVGhlIGpvaW50cyBkaXNwbGF5IGdvb2Qgc3RpZmZuZXNzLiBUaGUgYnJpZGdlXHJcbnN0aWxsIHNhZ3MsIGJ1dCB0aGlzIGlzIGJldHRlciB0aGFuIGluZmluaXRlIGZvcmNlcy5cclxuXHJcblJlY29tbWVuZGF0aW9uc1xyXG5Qc2V1ZG8gVmVsb2NpdGllcyBhcmUgbm90IHJlYWxseSB3b3J0aHdoaWxlIGJlY2F1c2UgdGhlIGJyaWRnZSBhbmQgY2hhaW4gY2Fubm90XHJcbnJlY292ZXIgZnJvbSBqb2ludCBzZXBhcmF0aW9uLiBJbiBvdGhlciBjYXNlcyB0aGUgYmVuZWZpdCBvdmVyIEJhdW1nYXJ0ZSBpcyBzbWFsbC5cclxuXHJcbk1vZGlmaWVkIE5HUyBpcyBub3QgYSByb2J1c3QgbWV0aG9kIGZvciB0aGUgcmV2b2x1dGUgam9pbnQgZHVlIHRvIHRoZSB2aW9sZW50XHJcbmluc3RhYmlsaXR5IHNlZW4gaW4gdGhlIHNpbXBsZSBwZW5kdWx1bS4gUGVyaGFwcyBpdCBpcyB2aWFibGUgd2l0aCBvdGhlciBjb25zdHJhaW50XHJcbnR5cGVzLCBlc3BlY2lhbGx5IHNjYWxhciBjb25zdHJhaW50cyB3aGVyZSB0aGUgZWZmZWN0aXZlIG1hc3MgaXMgYSBzY2FsYXIuXHJcblxyXG5UaGlzIGxlYXZlcyBCYXVtZ2FydGUgYW5kIEZ1bGwgTkdTLiBCYXVtZ2FydGUgaGFzIHNtYWxsLCBidXQgbWFuYWdlYWJsZSBpbnN0YWJpbGl0aWVzXHJcbmFuZCBpcyB2ZXJ5IGZhc3QuIEkgZG9uJ3QgdGhpbmsgd2UgY2FuIGVzY2FwZSBCYXVtZ2FydGUsIGVzcGVjaWFsbHkgaW4gaGlnaGx5XHJcbmRlbWFuZGluZyBjYXNlcyB3aGVyZSBoaWdoIGNvbnN0cmFpbnQgZmlkZWxpdHkgaXMgbm90IG5lZWRlZC5cclxuXHJcbkZ1bGwgTkdTIGlzIHJvYnVzdCBhbmQgZWFzeSBvbiB0aGUgZXllcy4gSSByZWNvbW1lbmQgdGhpcyBhcyBhbiBvcHRpb24gZm9yXHJcbmhpZ2hlciBmaWRlbGl0eSBzaW11bGF0aW9uIGFuZCBjZXJ0YWlubHkgZm9yIHN1c3BlbnNpb24gYnJpZGdlcyBhbmQgbG9uZyBjaGFpbnMuXHJcbkZ1bGwgTkdTIG1pZ2h0IGJlIGEgZ29vZCBjaG9pY2UgZm9yIHJhZ2RvbGxzLCBlc3BlY2lhbGx5IG1vdG9yaXplZCByYWdkb2xscyB3aGVyZVxyXG5qb2ludCBzZXBhcmF0aW9uIGNhbiBiZSBwcm9ibGVtYXRpYy4gVGhlIG51bWJlciBvZiBOR1MgaXRlcmF0aW9ucyBjYW4gYmUgcmVkdWNlZFxyXG5mb3IgYmV0dGVyIHBlcmZvcm1hbmNlIHdpdGhvdXQgaGFybWluZyByb2J1c3RuZXNzIG11Y2guXHJcblxyXG5FYWNoIGpvaW50IGluIGEgY2FuIGJlIGhhbmRsZWQgZGlmZmVyZW50bHkgaW4gdGhlIHBvc2l0aW9uIHNvbHZlci4gU28gSSByZWNvbW1lbmRcclxuYSBzeXN0ZW0gd2hlcmUgdGhlIHVzZXIgY2FuIHNlbGVjdCB0aGUgYWxnb3JpdGhtIG9uIGEgcGVyIGpvaW50IGJhc2lzLiBJIHdvdWxkXHJcbnByb2JhYmx5IGRlZmF1bHQgdG8gdGhlIHNsb3dlciBGdWxsIE5HUyBhbmQgbGV0IHRoZSB1c2VyIHNlbGVjdCB0aGUgZmFzdGVyXHJcbkJhdW1nYXJ0ZSBtZXRob2QgaW4gcGVyZm9ybWFuY2UgY3JpdGljYWwgc2NlbmFyaW9zLlxyXG4qL1xyXG5cclxuY29uc3Qgc190aW1lciA9IG5ldyBiMlRpbWVyKCk7XHJcbmNvbnN0IHNfc29sdmVyRGF0YSA9IG5ldyBiMlNvbHZlckRhdGEoKTtcclxuY29uc3Qgc19jb250YWN0U29sdmVyRGVmID0gbmV3IGIyQ29udGFjdFNvbHZlckRlZigpO1xyXG5jb25zdCBzX2NvbnRhY3RTb2x2ZXIgPSBuZXcgYjJDb250YWN0U29sdmVyKCk7XHJcbmNvbnN0IHNfdHJhbnNsYXRpb24gPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4vKlxyXG5DYWNoZSBQZXJmb3JtYW5jZVxyXG5cclxuVGhlIEJveDJEIHNvbHZlcnMgYXJlIGRvbWluYXRlZCBieSBjYWNoZSBtaXNzZXMuIERhdGEgc3RydWN0dXJlcyBhcmUgZGVzaWduZWRcclxudG8gaW5jcmVhc2UgdGhlIG51bWJlciBvZiBjYWNoZSBoaXRzLiBNdWNoIG9mIG1pc3NlcyBhcmUgZHVlIHRvIHJhbmRvbSBhY2Nlc3NcclxudG8gYm9keSBkYXRhLiBUaGUgY29uc3RyYWludCBzdHJ1Y3R1cmVzIGFyZSBpdGVyYXRlZCBvdmVyIGxpbmVhcmx5LCB3aGljaCBsZWFkc1xyXG50byBmZXcgY2FjaGUgbWlzc2VzLlxyXG5cclxuVGhlIGJvZGllcyBhcmUgbm90IGFjY2Vzc2VkIGR1cmluZyBpdGVyYXRpb24uIEluc3RlYWQgcmVhZCBvbmx5IGRhdGEsIHN1Y2ggYXNcclxudGhlIG1hc3MgdmFsdWVzIGFyZSBzdG9yZWQgd2l0aCB0aGUgY29uc3RyYWludHMuIFRoZSBtdXRhYmxlIGRhdGEgYXJlIHRoZSBjb25zdHJhaW50XHJcbmltcHVsc2VzIGFuZCB0aGUgYm9kaWVzIHZlbG9jaXRpZXMvcG9zaXRpb25zLiBUaGUgaW1wdWxzZXMgYXJlIGhlbGQgaW5zaWRlIHRoZVxyXG5jb25zdHJhaW50IHN0cnVjdHVyZXMuIFRoZSBib2R5IHZlbG9jaXRpZXMvcG9zaXRpb25zIGFyZSBoZWxkIGluIGNvbXBhY3QsIHRlbXBvcmFyeVxyXG5hcnJheXMgdG8gaW5jcmVhc2UgdGhlIG51bWJlciBvZiBjYWNoZSBoaXRzLiBMaW5lYXIgYW5kIGFuZ3VsYXIgdmVsb2NpdHkgYXJlXHJcbnN0b3JlZCBpbiBhIHNpbmdsZSBhcnJheSBzaW5jZSBtdWx0aXBsZSBhcnJheXMgbGVhZCB0byBtdWx0aXBsZSBtaXNzZXMuXHJcbiovXHJcblxyXG4vKlxyXG4yRCBSb3RhdGlvblxyXG5cclxuUiA9IFtjb3ModGhldGEpIC1zaW4odGhldGEpXVxyXG4gICAgW3Npbih0aGV0YSkgY29zKHRoZXRhKSBdXHJcblxyXG50aGV0YURvdCA9IG9tZWdhXHJcblxyXG5MZXQgcTEgPSBjb3ModGhldGEpLCBxMiA9IHNpbih0aGV0YSkuXHJcblIgPSBbcTEgLXEyXVxyXG4gICAgW3EyICBxMV1cclxuXHJcbnExRG90ID0gLXRoZXRhRG90ICogcTJcclxucTJEb3QgPSB0aGV0YURvdCAqIHExXHJcblxyXG5xMV9uZXcgPSBxMV9vbGQgLSBkdCAqIHcgKiBxMlxyXG5xMl9uZXcgPSBxMl9vbGQgKyBkdCAqIHcgKiBxMVxyXG50aGVuIG5vcm1hbGl6ZS5cclxuXHJcblRoaXMgbWlnaHQgYmUgZmFzdGVyIHRoYW4gY29tcHV0aW5nIHNpbitjb3MuXHJcbkhvd2V2ZXIsIHdlIGNhbiBjb21wdXRlIHNpbitjb3Mgb2YgdGhlIHNhbWUgYW5nbGUgZmFzdC5cclxuKi9cclxuXHJcbmV4cG9ydCBjbGFzcyBiMklzbGFuZCB7XHJcbiAgbV9saXN0ZW5lciA9IGIyQ29udGFjdExpc3RlbmVyLmIyX2RlZmF1bHRMaXN0ZW5lcjtcclxuXHJcbiAgcmVhZG9ubHkgbV9ib2RpZXM6IGIyQm9keVtdID0gLyoxMDI0Ki8gKFtudWxsXSBhcyB1bmtub3duKSBhcyBiMkJvZHlbXTsgLy8gVE9ETzogYjJTZXR0aW5nc1xyXG4gIHJlYWRvbmx5IG1fY29udGFjdHM6IGIyQ29udGFjdFtdID0gLyoxMDI0Ki8gKFtudWxsXSBhcyB1bmtub3duKSBhcyBiMkNvbnRhY3RbXTsgLy8gVE9ETzogYjJTZXR0aW5nc1xyXG4gIHJlYWRvbmx5IG1fam9pbnRzOiBiMkpvaW50W10gPSAvKjEwMjQqLyAoW251bGxdIGFzIHVua25vd24pIGFzIGIySm9pbnRbXTsgLy8gVE9ETzogYjJTZXR0aW5nc1xyXG5cclxuICByZWFkb25seSBtX3Bvc2l0aW9uczogYjJQb3NpdGlvbltdID0gYjJQb3NpdGlvbi5NYWtlQXJyYXkoMTAyNCk7IC8vIFRPRE86IGIyU2V0dGluZ3NcclxuICByZWFkb25seSBtX3ZlbG9jaXRpZXM6IGIyVmVsb2NpdHlbXSA9IGIyVmVsb2NpdHkuTWFrZUFycmF5KDEwMjQpOyAvLyBUT0RPOiBiMlNldHRpbmdzXHJcblxyXG4gIG1fYm9keUNvdW50ID0gMDtcclxuICBtX2pvaW50Q291bnQgPSAwO1xyXG4gIG1fY29udGFjdENvdW50ID0gMDtcclxuXHJcbiAgbV9ib2R5Q2FwYWNpdHkgPSAwO1xyXG4gIG1fY29udGFjdENhcGFjaXR5ID0gMDtcclxuICBtX2pvaW50Q2FwYWNpdHkgPSAwO1xyXG5cclxuICBJbml0aWFsaXplKFxyXG4gICAgYm9keUNhcGFjaXR5OiBudW1iZXIsXHJcbiAgICBjb250YWN0Q2FwYWNpdHk6IG51bWJlcixcclxuICAgIGpvaW50Q2FwYWNpdHk6IG51bWJlcixcclxuICAgIGxpc3RlbmVyOiBiMkNvbnRhY3RMaXN0ZW5lcixcclxuICApOiB2b2lkIHtcclxuICAgIHRoaXMubV9ib2R5Q2FwYWNpdHkgPSBib2R5Q2FwYWNpdHkgfCAwO1xyXG4gICAgdGhpcy5tX2NvbnRhY3RDYXBhY2l0eSA9IGNvbnRhY3RDYXBhY2l0eSB8IDA7XHJcbiAgICB0aGlzLm1fam9pbnRDYXBhY2l0eSA9IGpvaW50Q2FwYWNpdHkgfCAwO1xyXG4gICAgdGhpcy5tX2JvZHlDb3VudCA9IDA7XHJcbiAgICB0aGlzLm1fY29udGFjdENvdW50ID0gMDtcclxuICAgIHRoaXMubV9qb2ludENvdW50ID0gMDtcclxuXHJcbiAgICB0aGlzLm1fbGlzdGVuZXIgPSBsaXN0ZW5lcjtcclxuXHJcbiAgICAvLyBUT0RPOlxyXG4gICAgd2hpbGUgKHRoaXMubV9ib2RpZXMubGVuZ3RoIDwgYm9keUNhcGFjaXR5KSB7XHJcbiAgICAgIHRoaXMubV9ib2RpZXNbdGhpcy5tX2JvZGllcy5sZW5ndGhdID0gKG51bGwgYXMgdW5rbm93bikgYXMgYjJCb2R5O1xyXG4gICAgfVxyXG4gICAgLy8gVE9ETzpcclxuICAgIHdoaWxlICh0aGlzLm1fY29udGFjdHMubGVuZ3RoIDwgY29udGFjdENhcGFjaXR5KSB7XHJcbiAgICAgIHRoaXMubV9jb250YWN0c1t0aGlzLm1fY29udGFjdHMubGVuZ3RoXSA9IChudWxsIGFzIHVua25vd24pIGFzIGIyQ29udGFjdDtcclxuICAgIH1cclxuICAgIC8vIFRPRE86XHJcbiAgICB3aGlsZSAodGhpcy5tX2pvaW50cy5sZW5ndGggPCBqb2ludENhcGFjaXR5KSB7XHJcbiAgICAgIHRoaXMubV9qb2ludHNbdGhpcy5tX2pvaW50cy5sZW5ndGhdID0gKG51bGwgYXMgdW5rbm93bikgYXMgYjJKb2ludDtcclxuICAgIH1cclxuXHJcbiAgICAvLyBUT0RPOlxyXG4gICAgaWYgKHRoaXMubV9wb3NpdGlvbnMubGVuZ3RoIDwgYm9keUNhcGFjaXR5KSB7XHJcbiAgICAgIGNvbnN0IG5ld19sZW5ndGggPSBiMk1heEludCh0aGlzLm1fcG9zaXRpb25zLmxlbmd0aCA8PCAxLCBib2R5Q2FwYWNpdHkpO1xyXG4gICAgICB3aGlsZSAodGhpcy5tX3Bvc2l0aW9ucy5sZW5ndGggPCBuZXdfbGVuZ3RoKSB7XHJcbiAgICAgICAgdGhpcy5tX3Bvc2l0aW9uc1t0aGlzLm1fcG9zaXRpb25zLmxlbmd0aF0gPSBuZXcgYjJQb3NpdGlvbigpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICAvLyBUT0RPOlxyXG4gICAgaWYgKHRoaXMubV92ZWxvY2l0aWVzLmxlbmd0aCA8IGJvZHlDYXBhY2l0eSkge1xyXG4gICAgICBjb25zdCBuZXdfbGVuZ3RoID0gYjJNYXhJbnQodGhpcy5tX3ZlbG9jaXRpZXMubGVuZ3RoIDw8IDEsIGJvZHlDYXBhY2l0eSk7XHJcbiAgICAgIHdoaWxlICh0aGlzLm1fdmVsb2NpdGllcy5sZW5ndGggPCBuZXdfbGVuZ3RoKSB7XHJcbiAgICAgICAgdGhpcy5tX3ZlbG9jaXRpZXNbdGhpcy5tX3ZlbG9jaXRpZXMubGVuZ3RoXSA9IG5ldyBiMlZlbG9jaXR5KCk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIENsZWFyKCk6IHZvaWQge1xyXG4gICAgdGhpcy5tX2JvZHlDb3VudCA9IDA7XHJcbiAgICB0aGlzLm1fY29udGFjdENvdW50ID0gMDtcclxuICAgIHRoaXMubV9qb2ludENvdW50ID0gMDtcclxuICB9XHJcblxyXG4gIEFkZEJvZHkoYm9keTogYjJCb2R5KTogdm9pZCB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRoaXMubV9ib2R5Q291bnQgPCB0aGlzLm1fYm9keUNhcGFjaXR5KTtcclxuICAgIGJvZHkubV9pc2xhbmRJbmRleCA9IHRoaXMubV9ib2R5Q291bnQ7XHJcbiAgICB0aGlzLm1fYm9kaWVzW3RoaXMubV9ib2R5Q291bnQrK10gPSBib2R5O1xyXG4gIH1cclxuXHJcbiAgQWRkQ29udGFjdChjb250YWN0OiBiMkNvbnRhY3QpOiB2b2lkIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2NvbnRhY3RDb3VudCA8IHRoaXMubV9jb250YWN0Q2FwYWNpdHkpO1xyXG4gICAgdGhpcy5tX2NvbnRhY3RzW3RoaXMubV9jb250YWN0Q291bnQrK10gPSBjb250YWN0O1xyXG4gIH1cclxuXHJcbiAgQWRkSm9pbnQoam9pbnQ6IGIySm9pbnQpOiB2b2lkIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2pvaW50Q291bnQgPCB0aGlzLm1fam9pbnRDYXBhY2l0eSk7XHJcbiAgICB0aGlzLm1fam9pbnRzW3RoaXMubV9qb2ludENvdW50KytdID0gam9pbnQ7XHJcbiAgfVxyXG5cclxuICBTb2x2ZShwcm9maWxlOiBiMlByb2ZpbGUsIHN0ZXA6IGIyVGltZVN0ZXAsIGdyYXZpdHk6IGIyVmVjMiwgYWxsb3dTbGVlcDogYm9vbGVhbik6IG51bWJlciB7XHJcbiAgICBjb25zdCB0aW1lciA9IHNfdGltZXIuUmVzZXQoKTtcclxuXHJcbiAgICBjb25zdCBib2R5Q291bnQgPSB0aGlzLm1fYm9keUNvdW50O1xyXG4gICAgY29uc3QgbV9ib2RpZXMgPSB0aGlzLm1fYm9kaWVzO1xyXG4gICAgY29uc3QgbV9wb3NpdGlvbnMgPSB0aGlzLm1fcG9zaXRpb25zO1xyXG4gICAgY29uc3QgbV92ZWxvY2l0aWVzID0gdGhpcy5tX3ZlbG9jaXRpZXM7XHJcbiAgICBsZXQgZmxhZ3MgPSAwO1xyXG5cclxuICAgIGNvbnN0IGggPSBzdGVwLmR0O1xyXG5cclxuICAgIC8vIEludGVncmF0ZSB2ZWxvY2l0aWVzIGFuZCBhcHBseSBkYW1waW5nLiBJbml0aWFsaXplIHRoZSBib2R5IHN0YXRlLlxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBib2R5Q291bnQ7ICsraSkge1xyXG4gICAgICBjb25zdCBiID0gbV9ib2RpZXNbaV07XHJcblxyXG4gICAgICAvLyBjb25zdCBjOiBiMlZlYzIgPVxyXG4gICAgICBtX3Bvc2l0aW9uc1tpXS5jLkNvcHkoYi5tX3N3ZWVwLmMpO1xyXG4gICAgICBjb25zdCBhOiBudW1iZXIgPSBiLm1fc3dlZXAuYTtcclxuICAgICAgY29uc3QgdjogYjJWZWMyID0gbV92ZWxvY2l0aWVzW2ldLnYuQ29weShiLm1fbGluZWFyVmVsb2NpdHkpO1xyXG4gICAgICBsZXQgdzogbnVtYmVyID0gYi5tX2FuZ3VsYXJWZWxvY2l0eTtcclxuXHJcbiAgICAgIC8vIFN0b3JlIHBvc2l0aW9ucyBmb3IgY29udGludW91cyBjb2xsaXNpb24uXHJcbiAgICAgIGIubV9zd2VlcC5jMC5Db3B5KGIubV9zd2VlcC5jKTtcclxuICAgICAgYi5tX3N3ZWVwLmEwID0gYi5tX3N3ZWVwLmE7XHJcblxyXG4gICAgICBpZiAoYi5tX3R5cGUgPT09IGIyQm9keVR5cGUuYjJfZHluYW1pY0JvZHkpIHtcclxuICAgICAgICAvLyBJbnRlZ3JhdGUgdmVsb2NpdGllcy5cclxuICAgICAgICB2LnggKz0gaCAqIChiLm1fZ3Jhdml0eVNjYWxlICogZ3Jhdml0eS54ICsgYi5tX2ludk1hc3MgKiBiLm1fZm9yY2UueCk7XHJcbiAgICAgICAgdi55ICs9IGggKiAoYi5tX2dyYXZpdHlTY2FsZSAqIGdyYXZpdHkueSArIGIubV9pbnZNYXNzICogYi5tX2ZvcmNlLnkpO1xyXG4gICAgICAgIHcgKz0gaCAqIGIubV9pbnZJICogYi5tX3RvcnF1ZTtcclxuXHJcbiAgICAgICAgLy8gQXBwbHkgZGFtcGluZy5cclxuICAgICAgICAvLyBPREU6IGR2L2R0ICsgYyAqIHYgPSAwXHJcbiAgICAgICAgLy8gU29sdXRpb246IHYodCkgPSB2MCAqIGV4cCgtYyAqIHQpXHJcbiAgICAgICAgLy8gVGltZSBzdGVwOiB2KHQgKyBkdCkgPSB2MCAqIGV4cCgtYyAqICh0ICsgZHQpKSA9IHYwICogZXhwKC1jICogdCkgKiBleHAoLWMgKiBkdCkgPSB2ICogZXhwKC1jICogZHQpXHJcbiAgICAgICAgLy8gdjIgPSBleHAoLWMgKiBkdCkgKiB2MVxyXG4gICAgICAgIC8vIFBhZGUgYXBwcm94aW1hdGlvbjpcclxuICAgICAgICAvLyB2MiA9IHYxICogMSAvICgxICsgYyAqIGR0KVxyXG4gICAgICAgIHYuU2VsZk11bCgxLjAgLyAoMS4wICsgaCAqIGIubV9saW5lYXJEYW1waW5nKSk7XHJcbiAgICAgICAgdyAqPSAxLjAgLyAoMS4wICsgaCAqIGIubV9hbmd1bGFyRGFtcGluZyk7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIHRoaXMubV9wb3NpdGlvbnNbaV0uYyA9IGM7XHJcbiAgICAgIG1fcG9zaXRpb25zW2ldLmEgPSBhO1xyXG4gICAgICAvLyB0aGlzLm1fdmVsb2NpdGllc1tpXS52ID0gdjtcclxuICAgICAgbV92ZWxvY2l0aWVzW2ldLncgPSB3O1xyXG4gICAgfVxyXG5cclxuICAgIHRpbWVyLlJlc2V0KCk7XHJcblxyXG4gICAgLy8gU29sdmVyIGRhdGFcclxuICAgIGNvbnN0IHNvbHZlckRhdGEgPSBzX3NvbHZlckRhdGE7XHJcbiAgICBzb2x2ZXJEYXRhLnN0ZXAuQ29weShzdGVwKTtcclxuICAgIHNvbHZlckRhdGEucG9zaXRpb25zID0gdGhpcy5tX3Bvc2l0aW9ucztcclxuICAgIHNvbHZlckRhdGEudmVsb2NpdGllcyA9IHRoaXMubV92ZWxvY2l0aWVzO1xyXG5cclxuICAgIC8vIEluaXRpYWxpemUgdmVsb2NpdHkgY29uc3RyYWludHMuXHJcbiAgICBjb25zdCBjb250YWN0U29sdmVyRGVmID0gc19jb250YWN0U29sdmVyRGVmO1xyXG4gICAgY29udGFjdFNvbHZlckRlZi5zdGVwLkNvcHkoc3RlcCk7XHJcbiAgICBjb250YWN0U29sdmVyRGVmLmNvbnRhY3RzID0gdGhpcy5tX2NvbnRhY3RzO1xyXG4gICAgY29udGFjdFNvbHZlckRlZi5jb3VudCA9IHRoaXMubV9jb250YWN0Q291bnQ7XHJcbiAgICBjb250YWN0U29sdmVyRGVmLnBvc2l0aW9ucyA9IHRoaXMubV9wb3NpdGlvbnM7XHJcbiAgICBjb250YWN0U29sdmVyRGVmLnZlbG9jaXRpZXMgPSB0aGlzLm1fdmVsb2NpdGllcztcclxuXHJcbiAgICBjb25zdCBjb250YWN0U29sdmVyID0gc19jb250YWN0U29sdmVyLkluaXRpYWxpemUoY29udGFjdFNvbHZlckRlZik7XHJcbiAgICBjb250YWN0U29sdmVyLkluaXRpYWxpemVWZWxvY2l0eUNvbnN0cmFpbnRzKCk7XHJcblxyXG4gICAgaWYgKHN0ZXAud2FybVN0YXJ0aW5nKSB7XHJcbiAgICAgIGNvbnRhY3RTb2x2ZXIuV2FybVN0YXJ0KCk7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5fU29sdmVJbml0Sm9pbnRzKHNvbHZlckRhdGEpO1xyXG5cclxuICAgIHByb2ZpbGUuc29sdmVJbml0ICs9IHRpbWVyLkdldE1pbGxpc2Vjb25kcygpO1xyXG5cclxuICAgIC8vIFNvbHZlIHZlbG9jaXR5IGNvbnN0cmFpbnRzLlxyXG4gICAgdGltZXIuUmVzZXQoKTtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgc3RlcC52ZWxvY2l0eUl0ZXJhdGlvbnM7ICsraSkge1xyXG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMubV9qb2ludENvdW50OyArK2opIHtcclxuICAgICAgICB0aGlzLm1fam9pbnRzW2pdLlNvbHZlVmVsb2NpdHlDb25zdHJhaW50cyhzb2x2ZXJEYXRhKTtcclxuICAgICAgfVxyXG5cclxuICAgICAgY29udGFjdFNvbHZlci5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHMoKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBTdG9yZSBpbXB1bHNlcyBmb3Igd2FybSBzdGFydGluZ1xyXG4gICAgY29udGFjdFNvbHZlci5TdG9yZUltcHVsc2VzKCk7XHJcbiAgICBwcm9maWxlLnNvbHZlVmVsb2NpdHkgKz0gdGltZXIuR2V0TWlsbGlzZWNvbmRzKCk7XHJcblxyXG4gICAgLy8gSW50ZWdyYXRlIHBvc2l0aW9ucy5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgYm9keUNvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgYyA9IG1fcG9zaXRpb25zW2ldLmM7XHJcbiAgICAgIGxldCBhID0gbV9wb3NpdGlvbnNbaV0uYTtcclxuICAgICAgY29uc3QgdiA9IG1fdmVsb2NpdGllc1tpXS52O1xyXG4gICAgICBsZXQgdyA9IG1fdmVsb2NpdGllc1tpXS53O1xyXG5cclxuICAgICAgLy8gQ2hlY2sgZm9yIGxhcmdlIHZlbG9jaXRpZXNcclxuICAgICAgY29uc3QgdHJhbnNsYXRpb24gPSBiMlZlYzIuTXVsU1YoaCwgdiwgc190cmFuc2xhdGlvbik7XHJcbiAgICAgIGlmIChiMlZlYzIuRG90VlYodHJhbnNsYXRpb24sIHRyYW5zbGF0aW9uKSA+IGIyX21heFRyYW5zbGF0aW9uU3F1YXJlZCkge1xyXG4gICAgICAgIGNvbnN0IHJhdGlvOiBudW1iZXIgPSBiMl9tYXhUcmFuc2xhdGlvbiAvIHRyYW5zbGF0aW9uLkxlbmd0aCgpO1xyXG4gICAgICAgIHYuU2VsZk11bChyYXRpbyk7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGNvbnN0IHJvdGF0aW9uOiBudW1iZXIgPSBoICogdztcclxuICAgICAgaWYgKHJvdGF0aW9uICogcm90YXRpb24gPiBiMl9tYXhSb3RhdGlvblNxdWFyZWQpIHtcclxuICAgICAgICBjb25zdCByYXRpbzogbnVtYmVyID0gYjJfbWF4Um90YXRpb24gLyBiMkFicyhyb3RhdGlvbik7XHJcbiAgICAgICAgdyAqPSByYXRpbztcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gSW50ZWdyYXRlXHJcbiAgICAgIGMueCArPSBoICogdi54O1xyXG4gICAgICBjLnkgKz0gaCAqIHYueTtcclxuICAgICAgYSArPSBoICogdztcclxuXHJcbiAgICAgIC8vIHRoaXMubV9wb3NpdGlvbnNbaV0uYyA9IGM7XHJcbiAgICAgIG1fcG9zaXRpb25zW2ldLmEgPSBhO1xyXG4gICAgICAvLyB0aGlzLm1fdmVsb2NpdGllc1tpXS52ID0gdjtcclxuICAgICAgbV92ZWxvY2l0aWVzW2ldLncgPSB3O1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFNvbHZlIHBvc2l0aW9uIGNvbnN0cmFpbnRzXHJcbiAgICB0aW1lci5SZXNldCgpO1xyXG4gICAgZmxhZ3MgfD0gdGhpcy5fU29sdmVQb3NpdGlvbnNDb25zdHJhaXRzKHN0ZXAucG9zaXRpb25JdGVyYXRpb25zLCBjb250YWN0U29sdmVyLCBzb2x2ZXJEYXRhKVxyXG4gICAgICA/IDFcclxuICAgICAgOiAwO1xyXG5cclxuICAgIC8vIENvcHkgc3RhdGUgYnVmZmVycyBiYWNrIHRvIHRoZSBib2RpZXNcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgYm9keUNvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgYm9keTogYjJCb2R5ID0gbV9ib2RpZXNbaV07XHJcbiAgICAgIGJvZHkubV9zd2VlcC5jLkNvcHkobV9wb3NpdGlvbnNbaV0uYyk7XHJcbiAgICAgIGJvZHkubV9zd2VlcC5hID0gbV9wb3NpdGlvbnNbaV0uYTtcclxuICAgICAgYm9keS5tX2xpbmVhclZlbG9jaXR5LkNvcHkobV92ZWxvY2l0aWVzW2ldLnYpO1xyXG4gICAgICBib2R5Lm1fYW5ndWxhclZlbG9jaXR5ID0gbV92ZWxvY2l0aWVzW2ldLnc7XHJcbiAgICAgIGJvZHkuU3luY2hyb25pemVUcmFuc2Zvcm0oKTtcclxuICAgIH1cclxuXHJcbiAgICBwcm9maWxlLnNvbHZlUG9zaXRpb24gKz0gdGltZXIuR2V0TWlsbGlzZWNvbmRzKCk7XHJcblxyXG4gICAgdGhpcy5SZXBvcnQoY29udGFjdFNvbHZlci5tX3ZlbG9jaXR5Q29uc3RyYWludHMpO1xyXG5cclxuICAgIGZsYWdzIHw9IGFsbG93U2xlZXAgJiYgdGhpcy5fVXBkYXRlU2xlZXBUaW1lKGgpID8gMiA6IDA7XHJcbiAgICByZXR1cm4gZmxhZ3M7XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIF9Tb2x2ZVBvc2l0aW9uc0NvbnN0cmFpdHMoXHJcbiAgICBpdGVyYXRpb25zOiBudW1iZXIsXHJcbiAgICBjb250YWN0U29sdmVyOiBiMkNvbnRhY3RTb2x2ZXIsXHJcbiAgICBzb2x2ZXJEYXRhOiBiMlNvbHZlckRhdGEsXHJcbiAgKTogYm9vbGVhbiB7XHJcbiAgICBjb25zdCBqb2ludHNDb3VudCA9IHRoaXMubV9qb2ludENvdW50O1xyXG4gICAgY29uc3Qgam9pbnRzID0gdGhpcy5tX2pvaW50cztcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgaXRlcmF0aW9uczsgKytpKSB7XHJcbiAgICAgIGNvbnN0IGNvbnRhY3RzT2theTogYm9vbGVhbiA9IGNvbnRhY3RTb2x2ZXIuU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzKCk7XHJcblxyXG4gICAgICBsZXQgam9pbnRzT2theSA9IHRydWU7XHJcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgam9pbnRzQ291bnQ7ICsraikge1xyXG4gICAgICAgIGNvbnN0IGpvaW50T2theTogYm9vbGVhbiA9IGpvaW50c1tqXS5Tb2x2ZVBvc2l0aW9uQ29uc3RyYWludHMoc29sdmVyRGF0YSk7XHJcbiAgICAgICAgam9pbnRzT2theSA9IGpvaW50c09rYXkgJiYgam9pbnRPa2F5O1xyXG4gICAgICB9XHJcblxyXG4gICAgICBpZiAoY29udGFjdHNPa2F5ICYmIGpvaW50c09rYXkpIHtcclxuICAgICAgICAvLyBFeGl0IGVhcmx5IGlmIHRoZSBwb3NpdGlvbiBlcnJvcnMgYXJlIHNtYWxsLlxyXG4gICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICByZXR1cm4gZmFsc2U7XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIF9Tb2x2ZUluaXRKb2ludHMoc29sdmVyRGF0YTogYjJTb2x2ZXJEYXRhKSB7XHJcbiAgICBjb25zdCBjb3VudCA9IHRoaXMubV9qb2ludENvdW50O1xyXG4gICAgY29uc3QgbGlzdCA9IHRoaXMubV9qb2ludHM7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IGNvdW50OyArK2kpIHtcclxuICAgICAgbGlzdFtpXS5Jbml0VmVsb2NpdHlDb25zdHJhaW50cyhzb2x2ZXJEYXRhKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHByaXZhdGUgX1VwZGF0ZVNsZWVwVGltZShoOiBudW1iZXIpIHtcclxuICAgIGxldCBtaW5TbGVlcFRpbWUgPSBiMl9tYXhGbG9hdDtcclxuXHJcbiAgICBjb25zdCBsaW5Ub2xTcXIgPSBiMl9saW5lYXJTbGVlcFRvbGVyYW5jZSAqIGIyX2xpbmVhclNsZWVwVG9sZXJhbmNlO1xyXG4gICAgY29uc3QgYW5nVG9sU3FyID0gYjJfYW5ndWxhclNsZWVwVG9sZXJhbmNlICogYjJfYW5ndWxhclNsZWVwVG9sZXJhbmNlO1xyXG5cclxuICAgIGNvbnN0IGNvdW50ID0gdGhpcy5tX2JvZHlDb3VudDtcclxuICAgIGNvbnN0IGxpc3QgPSB0aGlzLm1fYm9kaWVzO1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgY291bnQ7ICsraSkge1xyXG4gICAgICBjb25zdCBiOiBiMkJvZHkgPSBsaXN0W2ldO1xyXG4gICAgICBpZiAoYi5HZXRUeXBlKCkgPT09IGIyQm9keVR5cGUuYjJfc3RhdGljQm9keSkge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBpZiAoXHJcbiAgICAgICAgIWIubV9hdXRvU2xlZXBGbGFnIHx8XHJcbiAgICAgICAgYi5tX2FuZ3VsYXJWZWxvY2l0eSAqIGIubV9hbmd1bGFyVmVsb2NpdHkgPiBhbmdUb2xTcXIgfHxcclxuICAgICAgICBiMlZlYzIuRG90VlYoYi5tX2xpbmVhclZlbG9jaXR5LCBiLm1fbGluZWFyVmVsb2NpdHkpID4gbGluVG9sU3FyXHJcbiAgICAgICkge1xyXG4gICAgICAgIGIubV9zbGVlcFRpbWUgPSAwO1xyXG4gICAgICAgIG1pblNsZWVwVGltZSA9IDA7XHJcbiAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgYi5tX3NsZWVwVGltZSArPSBoO1xyXG4gICAgICAgIG1pblNsZWVwVGltZSA9IGIyTWluKG1pblNsZWVwVGltZSwgYi5tX3NsZWVwVGltZSk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gbWluU2xlZXBUaW1lID49IGIyX3RpbWVUb1NsZWVwO1xyXG4gIH1cclxuXHJcbiAgU2xlZXBBbGwoKSB7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9ib2R5Q291bnQ7ICsraSkge1xyXG4gICAgICB0aGlzLm1fYm9kaWVzW2ldLlNldEF3YWtlKGZhbHNlKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIFNvbHZlVE9JKHN1YlN0ZXA6IGIyVGltZVN0ZXAsIHRvaUluZGV4QTogbnVtYmVyLCB0b2lJbmRleEI6IG51bWJlcik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCh0b2lJbmRleEEgPCB0aGlzLm1fYm9keUNvdW50KTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodG9pSW5kZXhCIDwgdGhpcy5tX2JvZHlDb3VudCk7XHJcblxyXG4gICAgLy8gSW5pdGlhbGl6ZSB0aGUgYm9keSBzdGF0ZS5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2JvZHlDb3VudDsgKytpKSB7XHJcbiAgICAgIGNvbnN0IGI6IGIyQm9keSA9IHRoaXMubV9ib2RpZXNbaV07XHJcbiAgICAgIHRoaXMubV9wb3NpdGlvbnNbaV0uYy5Db3B5KGIubV9zd2VlcC5jKTtcclxuICAgICAgdGhpcy5tX3Bvc2l0aW9uc1tpXS5hID0gYi5tX3N3ZWVwLmE7XHJcbiAgICAgIHRoaXMubV92ZWxvY2l0aWVzW2ldLnYuQ29weShiLm1fbGluZWFyVmVsb2NpdHkpO1xyXG4gICAgICB0aGlzLm1fdmVsb2NpdGllc1tpXS53ID0gYi5tX2FuZ3VsYXJWZWxvY2l0eTtcclxuICAgIH1cclxuXHJcbiAgICBjb25zdCBjb250YWN0U29sdmVyRGVmID0gc19jb250YWN0U29sdmVyRGVmO1xyXG4gICAgY29udGFjdFNvbHZlckRlZi5jb250YWN0cyA9IHRoaXMubV9jb250YWN0cztcclxuICAgIGNvbnRhY3RTb2x2ZXJEZWYuY291bnQgPSB0aGlzLm1fY29udGFjdENvdW50O1xyXG4gICAgY29udGFjdFNvbHZlckRlZi5zdGVwLkNvcHkoc3ViU3RlcCk7XHJcbiAgICBjb250YWN0U29sdmVyRGVmLnBvc2l0aW9ucyA9IHRoaXMubV9wb3NpdGlvbnM7XHJcbiAgICBjb250YWN0U29sdmVyRGVmLnZlbG9jaXRpZXMgPSB0aGlzLm1fdmVsb2NpdGllcztcclxuICAgIGNvbnN0IGNvbnRhY3RTb2x2ZXIgPSBzX2NvbnRhY3RTb2x2ZXIuSW5pdGlhbGl6ZShjb250YWN0U29sdmVyRGVmKTtcclxuXHJcbiAgICAvLyBTb2x2ZSBwb3NpdGlvbiBjb25zdHJhaW50cy5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgc3ViU3RlcC5wb3NpdGlvbkl0ZXJhdGlvbnM7ICsraSkge1xyXG4gICAgICBjb25zdCBjb250YWN0c09rYXk6IGJvb2xlYW4gPSBjb250YWN0U29sdmVyLlNvbHZlVE9JUG9zaXRpb25Db25zdHJhaW50cyh0b2lJbmRleEEsIHRvaUluZGV4Qik7XHJcbiAgICAgIGlmIChjb250YWN0c09rYXkpIHtcclxuICAgICAgICBicmVhaztcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIC8qXHJcbiAgICAgICAgI2lmIDBcclxuICAgICAgICAgIC8vIElzIHRoZSBuZXcgcG9zaXRpb24gcmVhbGx5IHNhZmU/XHJcbiAgICAgICAgICBmb3IgKGludDMyIGkgPSAwOyBpIDwgdGhpcy5tX2NvbnRhY3RDb3VudDsgKytpKSB7XHJcbiAgICAgICAgICAgIGIyQ29udGFjdCogYyA9IHRoaXMubV9jb250YWN0c1tpXTtcclxuICAgICAgICAgICAgYjJGaXh0dXJlKiBmQSA9IGMuR2V0Rml4dHVyZUEoKTtcclxuICAgICAgICAgICAgYjJGaXh0dXJlKiBmQiA9IGMuR2V0Rml4dHVyZUIoKTtcclxuXHJcbiAgICAgICAgICAgIGIyQm9keSogYkEgPSBmQS5HZXRCb2R5KCk7XHJcbiAgICAgICAgICAgIGIyQm9keSogYkIgPSBmQi5HZXRCb2R5KCk7XHJcblxyXG4gICAgICAgICAgICBpbnQzMiBpbmRleEEgPSBjLkdldENoaWxkSW5kZXhBKCk7XHJcbiAgICAgICAgICAgIGludDMyIGluZGV4QiA9IGMuR2V0Q2hpbGRJbmRleEIoKTtcclxuXHJcbiAgICAgICAgICAgIGIyRGlzdGFuY2VJbnB1dCBpbnB1dDtcclxuICAgICAgICAgICAgaW5wdXQucHJveHlBLlNldChmQS5HZXRTaGFwZSgpLCBpbmRleEEpO1xyXG4gICAgICAgICAgICBpbnB1dC5wcm94eUIuU2V0KGZCLkdldFNoYXBlKCksIGluZGV4Qik7XHJcbiAgICAgICAgICAgIGlucHV0LnRyYW5zZm9ybUEgPSBiQS5HZXRUcmFuc2Zvcm0oKTtcclxuICAgICAgICAgICAgaW5wdXQudHJhbnNmb3JtQiA9IGJCLkdldFRyYW5zZm9ybSgpO1xyXG4gICAgICAgICAgICBpbnB1dC51c2VSYWRpaSA9IGZhbHNlO1xyXG5cclxuICAgICAgICAgICAgYjJEaXN0YW5jZU91dHB1dCBvdXRwdXQ7XHJcbiAgICAgICAgICAgIGIyU2ltcGxleENhY2hlIGNhY2hlO1xyXG4gICAgICAgICAgICBjYWNoZS5jb3VudCA9IDA7XHJcbiAgICAgICAgICAgIGIyRGlzdGFuY2UoJm91dHB1dCwgJmNhY2hlLCAmaW5wdXQpO1xyXG5cclxuICAgICAgICAgICAgaWYgKG91dHB1dC5kaXN0YW5jZSA9PT0gMCB8fCBjYWNoZS5jb3VudCA9PT0gMykge1xyXG4gICAgICAgICAgICAgIGNhY2hlLmNvdW50ICs9IDA7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgIH1cclxuICAgICAgICAjZW5kaWZcclxuICAgICAgICAqL1xyXG5cclxuICAgIC8vIExlYXAgb2YgZmFpdGggdG8gbmV3IHNhZmUgc3RhdGUuXHJcbiAgICB0aGlzLm1fYm9kaWVzW3RvaUluZGV4QV0ubV9zd2VlcC5jMC5Db3B5KHRoaXMubV9wb3NpdGlvbnNbdG9pSW5kZXhBXS5jKTtcclxuICAgIHRoaXMubV9ib2RpZXNbdG9pSW5kZXhBXS5tX3N3ZWVwLmEwID0gdGhpcy5tX3Bvc2l0aW9uc1t0b2lJbmRleEFdLmE7XHJcbiAgICB0aGlzLm1fYm9kaWVzW3RvaUluZGV4Ql0ubV9zd2VlcC5jMC5Db3B5KHRoaXMubV9wb3NpdGlvbnNbdG9pSW5kZXhCXS5jKTtcclxuICAgIHRoaXMubV9ib2RpZXNbdG9pSW5kZXhCXS5tX3N3ZWVwLmEwID0gdGhpcy5tX3Bvc2l0aW9uc1t0b2lJbmRleEJdLmE7XHJcblxyXG4gICAgLy8gTm8gd2FybSBzdGFydGluZyBpcyBuZWVkZWQgZm9yIFRPSSBldmVudHMgYmVjYXVzZSB3YXJtXHJcbiAgICAvLyBzdGFydGluZyBpbXB1bHNlcyB3ZXJlIGFwcGxpZWQgaW4gdGhlIGRpc2NyZXRlIHNvbHZlci5cclxuICAgIGNvbnRhY3RTb2x2ZXIuSW5pdGlhbGl6ZVZlbG9jaXR5Q29uc3RyYWludHMoKTtcclxuXHJcbiAgICAvLyBTb2x2ZSB2ZWxvY2l0eSBjb25zdHJhaW50cy5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgc3ViU3RlcC52ZWxvY2l0eUl0ZXJhdGlvbnM7ICsraSkge1xyXG4gICAgICBjb250YWN0U29sdmVyLlNvbHZlVmVsb2NpdHlDb25zdHJhaW50cygpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIERvbid0IHN0b3JlIHRoZSBUT0kgY29udGFjdCBmb3JjZXMgZm9yIHdhcm0gc3RhcnRpbmdcclxuICAgIC8vIGJlY2F1c2UgdGhleSBjYW4gYmUgcXVpdGUgbGFyZ2UuXHJcblxyXG4gICAgY29uc3QgaDogbnVtYmVyID0gc3ViU3RlcC5kdDtcclxuXHJcbiAgICAvLyBJbnRlZ3JhdGUgcG9zaXRpb25zXHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9ib2R5Q291bnQ7ICsraSkge1xyXG4gICAgICBjb25zdCBjID0gdGhpcy5tX3Bvc2l0aW9uc1tpXS5jO1xyXG4gICAgICBsZXQgYSA9IHRoaXMubV9wb3NpdGlvbnNbaV0uYTtcclxuICAgICAgY29uc3QgdiA9IHRoaXMubV92ZWxvY2l0aWVzW2ldLnY7XHJcbiAgICAgIGxldCB3ID0gdGhpcy5tX3ZlbG9jaXRpZXNbaV0udztcclxuXHJcbiAgICAgIC8vIENoZWNrIGZvciBsYXJnZSB2ZWxvY2l0aWVzXHJcbiAgICAgIGNvbnN0IHRyYW5zbGF0aW9uID0gYjJWZWMyLk11bFNWKGgsIHYsIHNfdHJhbnNsYXRpb24pO1xyXG4gICAgICBpZiAoYjJWZWMyLkRvdFZWKHRyYW5zbGF0aW9uLCB0cmFuc2xhdGlvbikgPiBiMl9tYXhUcmFuc2xhdGlvblNxdWFyZWQpIHtcclxuICAgICAgICBjb25zdCByYXRpbzogbnVtYmVyID0gYjJfbWF4VHJhbnNsYXRpb24gLyB0cmFuc2xhdGlvbi5MZW5ndGgoKTtcclxuICAgICAgICB2LlNlbGZNdWwocmF0aW8pO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBjb25zdCByb3RhdGlvbjogbnVtYmVyID0gaCAqIHc7XHJcbiAgICAgIGlmIChyb3RhdGlvbiAqIHJvdGF0aW9uID4gYjJfbWF4Um90YXRpb25TcXVhcmVkKSB7XHJcbiAgICAgICAgY29uc3QgcmF0aW86IG51bWJlciA9IGIyX21heFJvdGF0aW9uIC8gYjJBYnMocm90YXRpb24pO1xyXG4gICAgICAgIHcgKj0gcmF0aW87XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIEludGVncmF0ZVxyXG4gICAgICBjLlNlbGZNdWxBZGQoaCwgdik7XHJcbiAgICAgIGEgKz0gaCAqIHc7XHJcblxyXG4gICAgICAvLyB0aGlzLm1fcG9zaXRpb25zW2ldLmMgPSBjO1xyXG4gICAgICB0aGlzLm1fcG9zaXRpb25zW2ldLmEgPSBhO1xyXG4gICAgICAvLyB0aGlzLm1fdmVsb2NpdGllc1tpXS52ID0gdjtcclxuICAgICAgdGhpcy5tX3ZlbG9jaXRpZXNbaV0udyA9IHc7XHJcblxyXG4gICAgICAvLyBTeW5jIGJvZGllc1xyXG4gICAgICBjb25zdCBib2R5OiBiMkJvZHkgPSB0aGlzLm1fYm9kaWVzW2ldO1xyXG4gICAgICBib2R5Lm1fc3dlZXAuYy5Db3B5KGMpO1xyXG4gICAgICBib2R5Lm1fc3dlZXAuYSA9IGE7XHJcbiAgICAgIGJvZHkubV9saW5lYXJWZWxvY2l0eS5Db3B5KHYpO1xyXG4gICAgICBib2R5Lm1fYW5ndWxhclZlbG9jaXR5ID0gdztcclxuICAgICAgYm9keS5TeW5jaHJvbml6ZVRyYW5zZm9ybSgpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMuUmVwb3J0KGNvbnRhY3RTb2x2ZXIubV92ZWxvY2l0eUNvbnN0cmFpbnRzKTtcclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIHNfaW1wdWxzZSA9IG5ldyBiMkNvbnRhY3RJbXB1bHNlKCk7XHJcblxyXG4gIFJlcG9ydChjb25zdHJhaW50czogYjJDb250YWN0VmVsb2NpdHlDb25zdHJhaW50W10pOiB2b2lkIHtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvbnRhY3RDb3VudDsgKytpKSB7XHJcbiAgICAgIGNvbnN0IGM6IGIyQ29udGFjdCA9IHRoaXMubV9jb250YWN0c1tpXTtcclxuXHJcbiAgICAgIGlmICghYykge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBjb25zdCB2YyA9IGNvbnN0cmFpbnRzW2ldO1xyXG5cclxuICAgICAgY29uc3QgaW1wdWxzZSA9IGIySXNsYW5kLnNfaW1wdWxzZTtcclxuICAgICAgaW1wdWxzZS5jb3VudCA9IHZjLnBvaW50Q291bnQ7XHJcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdmMucG9pbnRDb3VudDsgKytqKSB7XHJcbiAgICAgICAgaW1wdWxzZS5ub3JtYWxJbXB1bHNlc1tqXSA9IHZjLnBvaW50c1tqXS5ub3JtYWxJbXB1bHNlO1xyXG4gICAgICAgIGltcHVsc2UudGFuZ2VudEltcHVsc2VzW2pdID0gdmMucG9pbnRzW2pdLnRhbmdlbnRJbXB1bHNlO1xyXG4gICAgICB9XHJcblxyXG4gICAgICB0aGlzLm1fbGlzdGVuZXIuUG9zdFNvbHZlKGMsIGltcHVsc2UpO1xyXG4gICAgfVxyXG4gIH1cclxufVxyXG4iXX0=