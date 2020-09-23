/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
import { b2_epsilon, b2_maxFloat, b2_maxSubSteps, b2_maxTOIContacts, b2Assert, } from '../common/b2Settings';
import { b2Min, b2Sweep, b2Vec2 } from '../common/b2Math';
import { b2Timer } from '../common/b2Timer';
import { b2AABB, b2RayCastInput, b2RayCastOutput, b2TestOverlapShape, } from '../collision/b2Collision';
import { b2TimeOfImpact, b2TOIInput, b2TOIOutput, } from '../collision/b2TimeOfImpact';
import { b2AreaJoint } from './joints/b2AreaJoint';
import { b2DistanceJoint } from './joints/b2DistanceJoint';
import { b2FrictionJoint } from './joints/b2FrictionJoint';
import { b2GearJoint } from './joints/b2GearJoint';
import { b2MotorJoint } from './joints/b2MotorJoint';
import { b2MouseJoint } from './joints/b2MouseJoint';
import { b2PrismaticJoint } from './joints/b2PrismaticJoint';
import { b2PulleyJoint } from './joints/b2PulleyJoint';
import { b2RevoluteJoint } from './joints/b2RevoluteJoint';
import { b2RopeJoint } from './joints/b2RopeJoint';
import { b2WeldJoint } from './joints/b2WeldJoint';
import { b2WheelJoint } from './joints/b2WheelJoint';
import { b2Body } from './b2Body';
import { b2ContactManager } from './b2ContactManager';
import { b2FixtureProxy } from './b2Fixture';
import { b2Island } from './b2Island';
import { b2Profile, b2TimeStep } from './b2TimeStep';
import { b2QueryCallback, b2RayCastCallback, } from './b2WorldCallbacks';
import { b2CalculateParticleIterations } from '../particle/b2Particle';
import { b2ParticleSystem } from '../particle/b2ParticleSystem';
const SolveTOI_s_subStep = new b2TimeStep();
const SolveTOI_s_backup = new b2Sweep();
const SolveTOI_s_backup1 = new b2Sweep();
const SolveTOI_s_backup2 = new b2Sweep();
const SolveTOI_s_toi_input = new b2TOIInput();
const SolveTOI_s_toi_output = new b2TOIOutput();
/// Take a time step. This performs collision detection, integration,
/// and constraint solution.
/// @param timeStep the amount of time to simulate, this should not vary.
/// @param velocityIterations for the velocity constraint solver.
/// @param positionIterations for the position constraint solver.
const Step_s_step = new b2TimeStep();
const Step_s_stepTimer = new b2Timer();
const Step_s_timer = new b2Timer();
const Step_s_broadphaseTimer = new b2Timer();
/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
export class b2World {
    // #endif
    /// Construct a world object.
    /// @param gravity the world gravity vector.
    constructor(gravity) {
        this.m_newFixture = false;
        this.m_locked = false;
        this.m_clearForces = true;
        this.m_contactManager = new b2ContactManager();
        this.m_bodyList = null;
        this.m_jointList = null;
        // #if B2_ENABLE_PARTICLE
        this.m_particleSystemList = null;
        // #endif
        this.m_bodyCount = 0;
        this.m_jointCount = 0;
        this.m_gravity = new b2Vec2();
        this.m_allowSleep = true;
        this.m_destructionListener = null;
        // This is used to compute the time step ratio to
        // support a variable time step.
        this.m_inv_dt0 = NaN;
        // These are for debugging the solver.
        this.m_warmStarting = true;
        this.m_continuousPhysics = true;
        this.m_subStepping = false;
        this.m_stepComplete = true;
        this.m_profile = new b2Profile();
        this.m_island = new b2Island();
        this.s_stack = [null];
        // #if B2_ENABLE_CONTROLLER
        this.m_controllerList = null;
        this.m_controllerCount = 0;
        this.m_inv_dt0 = 0.0;
        this.m_gravity.Copy(gravity);
    }
    /// Register a destruction listener. The listener is owned by you and must
    /// remain in scope.
    SetDestructionListener(listener) {
        this.m_destructionListener = listener;
    }
    /// Register a contact filter to provide specific control over collision.
    /// Otherwise the default filter is used (b2_defaultFilter). The listener is
    /// owned by you and must remain in scope.
    SetContactFilter(filter) {
        this.m_contactManager.m_contactFilter = filter;
    }
    /// Register a contact event listener. The listener is owned by you and must
    /// remain in scope.
    SetContactListener(listener) {
        this.m_contactManager.m_contactListener = listener;
    }
    /// Create a rigid body given a definition. No reference to the definition
    /// is retained.
    /// @warning This function is locked during callbacks.
    CreateBody(def = {}) {
        if (this.IsLocked()) {
            throw new Error();
        }
        const b = new b2Body(def, this);
        // Add to world doubly linked list.
        b.m_prev = null;
        b.m_next = this.m_bodyList;
        if (this.m_bodyList) {
            this.m_bodyList.m_prev = b;
        }
        this.m_bodyList = b;
        ++this.m_bodyCount;
        return b;
    }
    /// Destroy a rigid body given a definition. No reference to the definition
    /// is retained. This function is locked during callbacks.
    /// @warning This automatically deletes all associated shapes and joints.
    /// @warning This function is locked during callbacks.
    DestroyBody(b) {
        !!B2_DEBUG && b2Assert(this.m_bodyCount > 0);
        if (this.IsLocked()) {
            throw new Error();
        }
        // Delete the attached joints.
        let je = b.m_jointList;
        while (je) {
            const je0 = je;
            je = je.next;
            if (this.m_destructionListener) {
                this.m_destructionListener.SayGoodbyeJoint(je0.joint);
            }
            this.DestroyJoint(je0.joint);
            b.m_jointList = je;
        }
        b.m_jointList = null;
        if (B2_ENABLE_CONTROLLER) {
            // @see b2Controller list
            let coe = b.m_controllerList;
            while (coe) {
                const coe0 = coe;
                coe = coe.nextController;
                coe0.controller.RemoveBody(b);
            }
        }
        // Delete the attached contacts.
        let ce = b.m_contactList;
        while (ce) {
            const ce0 = ce;
            ce = ce.next;
            this.m_contactManager.Destroy(ce0.contact);
        }
        b.m_contactList = null;
        // Delete the attached fixtures. This destroys broad-phase proxies.
        let f = b.m_fixtureList;
        while (f) {
            const f0 = f;
            f = f.m_next;
            if (this.m_destructionListener) {
                this.m_destructionListener.SayGoodbyeFixture(f0);
            }
            f0.DestroyProxies();
            f0.Reset();
            b.m_fixtureList = f;
            b.m_fixtureCount -= 1;
        }
        b.m_fixtureList = null;
        b.m_fixtureCount = 0;
        // Remove world body list.
        if (b.m_prev) {
            b.m_prev.m_next = b.m_next;
        }
        if (b.m_next) {
            b.m_next.m_prev = b.m_prev;
        }
        if (b === this.m_bodyList) {
            this.m_bodyList = b.m_next;
        }
        --this.m_bodyCount;
    }
    static _Joint_Create(def) {
        switch (def.type) {
            case 3 /* e_distanceJoint */:
                return new b2DistanceJoint(def);
            case 5 /* e_mouseJoint */:
                return new b2MouseJoint(def);
            case 2 /* e_prismaticJoint */:
                return new b2PrismaticJoint(def);
            case 1 /* e_revoluteJoint */:
                return new b2RevoluteJoint(def);
            case 4 /* e_pulleyJoint */:
                return new b2PulleyJoint(def);
            case 6 /* e_gearJoint */:
                return new b2GearJoint(def);
            case 7 /* e_wheelJoint */:
                return new b2WheelJoint(def);
            case 8 /* e_weldJoint */:
                return new b2WeldJoint(def);
            case 9 /* e_frictionJoint */:
                return new b2FrictionJoint(def);
            case 10 /* e_ropeJoint */:
                return new b2RopeJoint(def);
            case 11 /* e_motorJoint */:
                return new b2MotorJoint(def);
            case 12 /* e_areaJoint */:
                return new b2AreaJoint(def);
        }
        throw new Error();
    }
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    static _Joint_Destroy(joint) { }
    CreateJoint(def) {
        if (this.IsLocked()) {
            throw new Error();
        }
        const j = b2World._Joint_Create(def);
        // Connect to the world list.
        j.m_prev = null;
        j.m_next = this.m_jointList;
        if (this.m_jointList) {
            this.m_jointList.m_prev = j;
        }
        this.m_jointList = j;
        ++this.m_jointCount;
        // Connect to the bodies' doubly linked lists.
        // j.m_edgeA.other = j.m_bodyB; // done in b2Joint constructor
        j.m_edgeA.prev = null;
        j.m_edgeA.next = j.m_bodyA.m_jointList;
        if (j.m_bodyA.m_jointList) {
            j.m_bodyA.m_jointList.prev = j.m_edgeA;
        }
        j.m_bodyA.m_jointList = j.m_edgeA;
        // j.m_edgeB.other = j.m_bodyA; // done in b2Joint constructor
        j.m_edgeB.prev = null;
        j.m_edgeB.next = j.m_bodyB.m_jointList;
        if (j.m_bodyB.m_jointList) {
            j.m_bodyB.m_jointList.prev = j.m_edgeB;
        }
        j.m_bodyB.m_jointList = j.m_edgeB;
        const bodyA = j.m_bodyA;
        const bodyB = j.m_bodyB;
        const collideConnected = j.m_collideConnected;
        // If the joint prevents collisions, then flag any contacts for filtering.
        if (!collideConnected) {
            let edge = bodyB.GetContactList();
            while (edge) {
                if (edge.other === bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact.FlagForFiltering();
                }
                edge = edge.next;
            }
        }
        // Note: creating a joint doesn't wake the bodies.
        return j;
    }
    /// Destroy a joint. This may cause the connected bodies to begin colliding.
    /// @warning This function is locked during callbacks.
    DestroyJoint(j) {
        if (this.IsLocked()) {
            throw new Error();
        }
        // Remove from the doubly linked list.
        if (j.m_prev) {
            j.m_prev.m_next = j.m_next;
        }
        if (j.m_next) {
            j.m_next.m_prev = j.m_prev;
        }
        if (j === this.m_jointList) {
            this.m_jointList = j.m_next;
        }
        // Disconnect from island graph.
        const bodyA = j.m_bodyA;
        const bodyB = j.m_bodyB;
        const collideConnected = j.m_collideConnected;
        // Wake up connected bodies.
        bodyA.SetAwake(true);
        bodyB.SetAwake(true);
        // Remove from body 1.
        if (j.m_edgeA.prev) {
            j.m_edgeA.prev.next = j.m_edgeA.next;
        }
        if (j.m_edgeA.next) {
            j.m_edgeA.next.prev = j.m_edgeA.prev;
        }
        if (j.m_edgeA === bodyA.m_jointList) {
            bodyA.m_jointList = j.m_edgeA.next;
        }
        j.m_edgeA.Reset();
        // Remove from body 2
        if (j.m_edgeB.prev) {
            j.m_edgeB.prev.next = j.m_edgeB.next;
        }
        if (j.m_edgeB.next) {
            j.m_edgeB.next.prev = j.m_edgeB.prev;
        }
        if (j.m_edgeB === bodyB.m_jointList) {
            bodyB.m_jointList = j.m_edgeB.next;
        }
        j.m_edgeB.Reset();
        b2World._Joint_Destroy(j);
        !!B2_DEBUG && b2Assert(this.m_jointCount > 0);
        --this.m_jointCount;
        // If the joint prevents collisions, then flag any contacts for filtering.
        if (!collideConnected) {
            let edge = bodyB.GetContactList();
            while (edge) {
                if (edge.other === bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact.FlagForFiltering();
                }
                edge = edge.next;
            }
        }
    }
    CreateParticleSystem(def) {
        if (!B2_ENABLE_PARTICLE) {
            throw new Error();
        }
        if (this.IsLocked()) {
            throw new Error();
        }
        const p = new b2ParticleSystem(def, this);
        // Add to world doubly linked list.
        p.m_prev = null;
        p.m_next = this.m_particleSystemList;
        if (this.m_particleSystemList) {
            this.m_particleSystemList.m_prev = p;
        }
        this.m_particleSystemList = p;
        return p;
    }
    DestroyParticleSystem(p) {
        if (!B2_ENABLE_PARTICLE) {
            throw new Error();
        }
        if (this.IsLocked()) {
            throw new Error();
        }
        // Remove world particleSystem list.
        if (p.m_prev) {
            p.m_prev.m_next = p.m_next;
        }
        if (p.m_next) {
            p.m_next.m_prev = p.m_prev;
        }
        if (p === this.m_particleSystemList) {
            this.m_particleSystemList = p.m_next;
        }
    }
    CalculateReasonableParticleIterations(timeStep) {
        if (!B2_ENABLE_PARTICLE || this.m_particleSystemList === null) {
            return 1;
        }
        // todo:
        function GetSmallestRadius(world) {
            let smallestRadius = b2_maxFloat;
            for (let system = world.GetParticleSystemList(); system !== null; system = system.m_next) {
                smallestRadius = b2Min(smallestRadius, system.GetRadius());
            }
            return smallestRadius;
        }
        // Use the smallest radius, since that represents the worst-case.
        return b2CalculateParticleIterations(this.m_gravity.Length(), GetSmallestRadius(this), timeStep);
    }
    Step(dt, velocityIterations, positionIterations, particleIterations = this.CalculateReasonableParticleIterations(dt)) {
        const stepTimer = Step_s_stepTimer.Reset();
        // If new fixtures were added, we need to find the new contacts.
        if (this.m_newFixture) {
            this.m_contactManager.FindNewContacts();
            this.m_newFixture = false;
        }
        this.m_locked = true;
        const step = Step_s_step;
        step.dt = dt;
        step.velocityIterations = velocityIterations;
        step.positionIterations = positionIterations;
        if (B2_ENABLE_PARTICLE) {
            step.particleIterations = particleIterations;
        }
        if (dt > 0) {
            step.inv_dt = 1 / dt;
        }
        else {
            step.inv_dt = 0;
        }
        step.dtRatio = this.m_inv_dt0 * dt;
        step.warmStarting = this.m_warmStarting;
        // Update contacts. This is where some contacts are destroyed.
        const timer = Step_s_timer.Reset();
        this.m_contactManager.Collide();
        this.m_profile.collide = timer.GetMilliseconds();
        // Integrate velocities, solve velocity constraints, and integrate positions.
        if (this.m_stepComplete && step.dt > 0) {
            const timer = Step_s_timer.Reset();
            if (B2_ENABLE_PARTICLE) {
                for (let p = this.m_particleSystemList; p; p = p.m_next) {
                    p.Solve(step); // Particle Simulation
                }
            }
            this.Solve(step);
            this.m_profile.solve = timer.GetMilliseconds();
        }
        // Handle TOI events.
        if (this.m_continuousPhysics && step.dt > 0) {
            const timer = Step_s_timer.Reset();
            this.SolveTOI(step);
            this.m_profile.solveTOI = timer.GetMilliseconds();
        }
        if (step.dt > 0) {
            this.m_inv_dt0 = step.inv_dt;
        }
        if (this.m_clearForces) {
            this.ClearForces();
        }
        this.m_locked = false;
        this.m_profile.step = stepTimer.GetMilliseconds();
    }
    /// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
    /// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
    /// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
    /// a fixed sized time step under a variable frame-rate.
    /// When you perform sub-stepping you will disable auto clearing of forces and instead call
    /// ClearForces after all sub-steps are complete in one pass of your game loop.
    /// @see SetAutoClearForces
    ClearForces() {
        for (let body = this.m_bodyList; body; body = body.m_next) {
            body.m_force.SetZero();
            body.m_torque = 0;
        }
    }
    QueryAABB(...args) {
        if (args[0] instanceof b2QueryCallback) {
            this._QueryAABB(args[0], args[1]);
        }
        else {
            this._QueryAABB(null, args[0], args[1]);
        }
    }
    _QueryAABB(callback, aabb, fn) {
        this.m_contactManager.m_broadPhase.Query(aabb, (proxy) => {
            const fixture_proxy = proxy.userData;
            !!B2_DEBUG && b2Assert(fixture_proxy instanceof b2FixtureProxy);
            const fixture = fixture_proxy.fixture;
            if (callback) {
                return callback.ReportFixture(fixture);
            }
            else if (fn) {
                return fn(fixture);
            }
            return true;
        });
        if (B2_ENABLE_PARTICLE && callback instanceof b2QueryCallback) {
            for (let p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.QueryAABB(callback, aabb);
                }
            }
        }
    }
    QueryAllAABB(aabb, out = []) {
        this.QueryAABB(aabb, (fixture) => {
            out.push(fixture);
            return true;
        });
        return out;
    }
    QueryPointAABB(...args) {
        if (args[0] instanceof b2QueryCallback) {
            this._QueryPointAABB(args[0], args[1]);
        }
        else {
            this._QueryPointAABB(null, args[0], args[1]);
        }
    }
    _QueryPointAABB(callback, point, fn) {
        this.m_contactManager.m_broadPhase.QueryPoint(point, (proxy) => {
            const fixture_proxy = proxy.userData;
            !!B2_DEBUG && b2Assert(fixture_proxy instanceof b2FixtureProxy);
            const fixture = fixture_proxy.fixture;
            if (callback) {
                return callback.ReportFixture(fixture);
            }
            else if (fn) {
                return fn(fixture);
            }
            return true;
        });
        if (B2_ENABLE_PARTICLE && callback instanceof b2QueryCallback) {
            for (let p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.QueryPointAABB(callback, point);
                }
            }
        }
    }
    QueryAllPointAABB(point, out = []) {
        this.QueryPointAABB(point, (fixture) => {
            out.push(fixture);
            return true;
        });
        return out;
    }
    QueryFixtureShape(...args) {
        if (args[0] instanceof b2QueryCallback) {
            this._QueryFixtureShape(args[0], args[1], args[2], args[3]);
        }
        else {
            this._QueryFixtureShape(null, args[0], args[1], args[2], args[3]);
        }
    }
    _QueryFixtureShape(callback, shape, index, transform, fn) {
        const aabb = b2World.QueryFixtureShape_s_aabb;
        shape.ComputeAABB(aabb, transform, index);
        this.m_contactManager.m_broadPhase.Query(aabb, (proxy) => {
            const fixture_proxy = proxy.userData;
            !!B2_DEBUG && b2Assert(fixture_proxy instanceof b2FixtureProxy);
            const fixture = fixture_proxy.fixture;
            if (b2TestOverlapShape(shape, index, fixture.GetShape(), fixture_proxy.childIndex, transform, fixture.GetBody().GetTransform())) {
                if (callback) {
                    return callback.ReportFixture(fixture);
                }
                else if (fn) {
                    return fn(fixture);
                }
            }
            return true;
        });
        if (B2_ENABLE_PARTICLE && callback instanceof b2QueryCallback) {
            for (let p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.QueryAABB(callback, aabb);
                }
            }
        }
    }
    QueryAllFixtureShape(shape, index, transform, out = []) {
        this.QueryFixtureShape(shape, index, transform, (fixture) => {
            out.push(fixture);
            return true;
        });
        return out;
    }
    QueryFixturePoint(...args) {
        if (args[0] instanceof b2QueryCallback) {
            this._QueryFixturePoint(args[0], args[1]);
        }
        else {
            this._QueryFixturePoint(null, args[0], args[1]);
        }
    }
    _QueryFixturePoint(callback, point, fn) {
        this.m_contactManager.m_broadPhase.QueryPoint(point, (proxy) => {
            const fixture_proxy = proxy.userData;
            !!B2_DEBUG && b2Assert(fixture_proxy instanceof b2FixtureProxy);
            const fixture = fixture_proxy.fixture;
            if (fixture.TestPoint(point)) {
                if (callback) {
                    return callback.ReportFixture(fixture);
                }
                else if (fn) {
                    return fn(fixture);
                }
            }
            return true;
        });
        if (B2_ENABLE_PARTICLE && callback) {
            for (let p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.QueryPointAABB(callback, point);
                }
            }
        }
    }
    QueryAllFixturePoint(point, out = []) {
        this.QueryFixturePoint(point, (fixture) => {
            out.push(fixture);
            return true;
        });
        return out;
    }
    RayCast(...args) {
        if (args[0] instanceof b2RayCastCallback) {
            this._RayCast(args[0], args[1], args[2]);
        }
        else {
            this._RayCast(null, args[0], args[1], args[2]);
        }
    }
    _RayCast(callback, point1, point2, fn) {
        const input = b2World.RayCast_s_input;
        input.maxFraction = 1;
        input.p1.Copy(point1);
        input.p2.Copy(point2);
        this.m_contactManager.m_broadPhase.RayCast(input, (input, proxy) => {
            const fixture_proxy = proxy.userData;
            !!B2_DEBUG && b2Assert(fixture_proxy instanceof b2FixtureProxy);
            const fixture = fixture_proxy.fixture;
            const index = fixture_proxy.childIndex;
            const output = b2World.RayCast_s_output;
            const hit = fixture.RayCast(output, input, index);
            if (hit) {
                const fraction = output.fraction;
                const point = b2World.RayCast_s_point;
                point.Set((1 - fraction) * point1.x + fraction * point2.x, (1 - fraction) * point1.y + fraction * point2.y);
                if (callback) {
                    return callback.ReportFixture(fixture, point, output.normal, fraction);
                }
                else if (fn) {
                    return fn(fixture, point, output.normal, fraction);
                }
            }
            return input.maxFraction;
        });
        if (B2_ENABLE_PARTICLE && callback) {
            for (let p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.RayCast(callback, point1, point2);
                }
            }
        }
    }
    RayCastOne(point1, point2) {
        let result = null;
        let min_fraction = 1;
        this.RayCast(point1, point2, (fixture, point, normal, fraction) => {
            if (fraction < min_fraction) {
                min_fraction = fraction;
                result = fixture;
            }
            return min_fraction;
        });
        return result;
    }
    RayCastAll(point1, point2, out = []) {
        this.RayCast(point1, point2, (fixture, point, normal, fraction) => {
            out.push(fixture);
            return 1;
        });
        return out;
    }
    /// Get the world body list. With the returned body, use b2Body::GetNext to get
    /// the next body in the world list. A NULL body indicates the end of the list.
    /// @return the head of the world body list.
    GetBodyList() {
        return this.m_bodyList;
    }
    /// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
    /// the next joint in the world list. A NULL joint indicates the end of the list.
    /// @return the head of the world joint list.
    GetJointList() {
        return this.m_jointList;
    }
    GetParticleSystemList() {
        return this.m_particleSystemList;
    }
    /// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
    /// the next contact in the world list. A NULL contact indicates the end of the list.
    /// @return the head of the world contact list.
    /// @warning contacts are created and destroyed in the middle of a time step.
    /// Use b2ContactListener to avoid missing contacts.
    GetContactList() {
        return this.m_contactManager.m_contactList;
    }
    /// Enable/disable sleep.
    SetAllowSleeping(flag) {
        if (flag === this.m_allowSleep) {
            return;
        }
        this.m_allowSleep = flag;
        if (!this.m_allowSleep) {
            for (let b = this.m_bodyList; b; b = b.m_next) {
                b.SetAwake(true);
            }
        }
    }
    GetAllowSleeping() {
        return this.m_allowSleep;
    }
    /// Enable/disable warm starting. For testing.
    SetWarmStarting(flag) {
        this.m_warmStarting = flag;
    }
    GetWarmStarting() {
        return this.m_warmStarting;
    }
    /// Enable/disable continuous physics. For testing.
    SetContinuousPhysics(flag) {
        this.m_continuousPhysics = flag;
    }
    GetContinuousPhysics() {
        return this.m_continuousPhysics;
    }
    /// Enable/disable single stepped continuous physics. For testing.
    SetSubStepping(flag) {
        this.m_subStepping = flag;
    }
    GetSubStepping() {
        return this.m_subStepping;
    }
    /// Get the number of broad-phase proxies.
    GetProxyCount() {
        return this.m_contactManager.m_broadPhase.GetProxyCount();
    }
    /// Get the number of bodies.
    GetBodyCount() {
        return this.m_bodyCount;
    }
    /// Get the number of joints.
    GetJointCount() {
        return this.m_jointCount;
    }
    /// Get the number of contacts (each may have 0 or more contact points).
    GetContactCount() {
        return this.m_contactManager.m_contactCount;
    }
    /// Get the height of the dynamic tree.
    GetTreeHeight() {
        return this.m_contactManager.m_broadPhase.GetTreeHeight();
    }
    /// Get the balance of the dynamic tree.
    GetTreeBalance() {
        return this.m_contactManager.m_broadPhase.GetTreeBalance();
    }
    /// Get the quality metric of the dynamic tree. The smaller the better.
    /// The minimum is 1.
    GetTreeQuality() {
        return this.m_contactManager.m_broadPhase.GetTreeQuality();
    }
    /// Change the global gravity vector.
    SetGravity(gravity, wake = true) {
        if (!b2Vec2.IsEqualToV(this.m_gravity, gravity)) {
            this.m_gravity.Copy(gravity);
            if (wake) {
                for (let b = this.m_bodyList; b; b = b.m_next) {
                    b.SetAwake(true);
                }
            }
        }
    }
    /// Get the global gravity vector.
    GetGravity() {
        return this.m_gravity;
    }
    /// Is the world locked (in the middle of a time step).
    IsLocked() {
        return this.m_locked;
    }
    /// Set flag to control automatic clearing of forces after each time step.
    SetAutoClearForces(flag) {
        this.m_clearForces = flag;
    }
    /// Get the flag that controls automatic clearing of forces after each time step.
    GetAutoClearForces() {
        return this.m_clearForces;
    }
    /// Shift the world origin. Useful for large worlds.
    /// The body shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    ShiftOrigin(newOrigin) {
        if (this.IsLocked()) {
            throw new Error();
        }
        for (let b = this.m_bodyList; b; b = b.m_next) {
            b.m_xf.p.SelfSub(newOrigin);
            b.m_sweep.c0.SelfSub(newOrigin);
            b.m_sweep.c.SelfSub(newOrigin);
        }
        for (let j = this.m_jointList; j; j = j.m_next) {
            j.ShiftOrigin(newOrigin);
        }
        this.m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
    }
    /// Get the contact manager for testing.
    GetContactManager() {
        return this.m_contactManager;
    }
    /// Get the current profile.
    GetProfile() {
        return this.m_profile;
    }
    Solve(step) {
        if (B2_ENABLE_PARTICLE) {
            // update previous transforms
            for (let b = this.m_bodyList; b; b = b.m_next) {
                b.m_xf0.Copy(b.m_xf);
            }
        }
        if (B2_ENABLE_CONTROLLER) {
            // @see b2Controller list
            for (let controller = this.m_controllerList; controller; controller = controller.m_next) {
                controller.Step(step);
            }
        }
        this.m_profile.solveInit = 0;
        this.m_profile.solveVelocity = 0;
        this.m_profile.solvePosition = 0;
        // Size the island for the worst case.
        const island = this.m_island;
        island.Initialize(this.m_bodyCount, this.m_contactManager.m_contactCount, this.m_jointCount, this.m_contactManager.m_contactListener);
        // Clear all the island flags.
        for (let b = this.m_bodyList; b; b = b.m_next) {
            b.m_islandFlag = false;
        }
        for (let c = this.m_contactManager.m_contactList; c; c = c.m_next) {
            c.m_islandFlag = false;
        }
        for (let j = this.m_jointList; j; j = j.m_next) {
            j.m_islandFlag = false;
        }
        // Build and simulate all awake islands.
        const stackSize = this.m_bodyCount; // DEBUG
        const stack = this.s_stack;
        for (let seed = this.m_bodyList; seed; seed = seed.m_next) {
            if (seed.m_islandFlag) {
                continue;
            }
            if (!seed.IsAwake() || !seed.IsActive()) {
                continue;
            }
            // The seed can be dynamic or kinematic.
            if (seed.GetType() === 0 /* b2_staticBody */) {
                continue;
            }
            // Reset island and stack.
            island.Clear();
            let stackCount = 0;
            stack[stackCount++] = seed;
            seed.m_islandFlag = true;
            // Perform a depth first search (DFS) on the constraint graph.
            while (stackCount > 0) {
                // Grab the next body off the stack and add it to the island.
                const b = stack[--stackCount];
                if (!b) {
                    throw new Error();
                }
                !!B2_DEBUG && b2Assert(b.IsActive());
                island.AddBody(b);
                // Make sure the body is awake. (without resetting sleep timer).
                b.m_awakeFlag = true;
                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.GetType() === 0 /* b2_staticBody */) {
                    continue;
                }
                // Search all contacts connected to this body.
                for (let ce = b.m_contactList; ce; ce = ce.next) {
                    const contact = ce.contact;
                    // Has this contact already been added to an island?
                    if (contact.m_islandFlag) {
                        continue;
                    }
                    // Is this contact solid and touching?
                    if (!contact.IsEnabled() || !contact.IsTouching()) {
                        continue;
                    }
                    // Skip sensors.
                    const sensorA = contact.m_fixtureA.m_isSensor;
                    const sensorB = contact.m_fixtureB.m_isSensor;
                    if (sensorA || sensorB) {
                        continue;
                    }
                    island.AddContact(contact);
                    contact.m_islandFlag = true;
                    const other = ce.other;
                    // Was the other body already added to this island?
                    if (other.m_islandFlag) {
                        continue;
                    }
                    !!B2_DEBUG && b2Assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_islandFlag = true;
                }
                // Search all joints connect to this body.
                for (let je = b.m_jointList; je; je = je.next) {
                    if (je.joint.m_islandFlag) {
                        continue;
                    }
                    const other = je.other;
                    // Don't simulate joints connected to inactive bodies.
                    if (!other.IsActive()) {
                        continue;
                    }
                    island.AddJoint(je.joint);
                    je.joint.m_islandFlag = true;
                    if (other.m_islandFlag) {
                        continue;
                    }
                    !!B2_DEBUG && b2Assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_islandFlag = true;
                }
            }
            if ((island.Solve(this.m_profile, step, this.m_gravity, this.m_allowSleep) & 3) === 3) {
                island.SleepAll();
            }
            // Post solve cleanup.
            for (let i = 0; i < island.m_bodyCount; ++i) {
                // Allow static bodies to participate in other islands.
                const b = island.m_bodies[i];
                if (b.GetType() === 0 /* b2_staticBody */) {
                    b.m_islandFlag = false;
                }
            }
        }
        for (let i = 0; i < stack.length; ++i) {
            if (!stack[i]) {
                break;
            }
            stack[i] = null;
        }
        const timer = Step_s_broadphaseTimer.Reset();
        // Synchronize fixtures, check for out of range bodies.
        this._SynchronizeFixturesCheck();
        // Look for new contacts.
        this.m_contactManager.FindNewContacts();
        this.m_profile.broadphase = timer.GetMilliseconds();
    }
    _SynchronizeFixturesCheck() {
        // Synchronize fixtures, check for out of range bodies.
        for (let b = this.m_bodyList; b; b = b.m_next) {
            // If a body was not in an island then it did not move.
            if (!b.m_islandFlag) {
                continue;
            }
            if (b.GetType() === 0 /* b2_staticBody */) {
                continue;
            }
            // Update fixtures (for broad-phase).
            b.SynchronizeFixtures();
        }
    }
    SolveTOI(step) {
        const island = this.m_island;
        island.Initialize(b2_maxTOIContacts << 1, b2_maxTOIContacts, 0, this.m_contactManager.m_contactListener);
        if (this.m_stepComplete) {
            for (let b = this.m_bodyList; b; b = b.m_next) {
                b.m_islandFlag = false;
                b.m_sweep.alpha0 = 0;
            }
            for (let c = this.m_contactManager.m_contactList; c; c = c.m_next) {
                // Invalidate TOI
                c.m_toiFlag = false;
                c.m_islandFlag = false;
                c.m_toiCount = 0;
                c.m_toi = 1.0;
            }
        }
        // Find TOI events and solve them.
        for (;;) {
            // Find the first TOI.
            let minContact = null;
            let minAlpha = 1.0;
            for (let c = this.m_contactManager.m_contactList; c; c = c.m_next) {
                // Is this contact disabled?
                if (!c.IsEnabled()) {
                    continue;
                }
                // Prevent excessive sub-stepping.
                if (c.m_toiCount > b2_maxSubSteps) {
                    continue;
                }
                let alpha = 1;
                if (c.m_toiFlag) {
                    // This contact has a valid cached TOI.
                    alpha = c.m_toi;
                }
                else {
                    const fA = c.GetFixtureA();
                    const fB = c.GetFixtureB();
                    // Is there a sensor?
                    if (fA.IsSensor() || fB.IsSensor()) {
                        continue;
                    }
                    const bA = fA.GetBody();
                    const bB = fB.GetBody();
                    const typeA = bA.m_type;
                    const typeB = bB.m_type;
                    !!B2_DEBUG &&
                        b2Assert(typeA !== 0 /* b2_staticBody */ || typeB !== 0 /* b2_staticBody */);
                    const activeA = bA.IsAwake() && typeA !== 0 /* b2_staticBody */;
                    const activeB = bB.IsAwake() && typeB !== 0 /* b2_staticBody */;
                    // Is at least one body active (awake and dynamic or kinematic)?
                    if (!activeA && !activeB) {
                        continue;
                    }
                    const collideA = bA.IsBullet() || typeA !== 2 /* b2_dynamicBody */;
                    const collideB = bB.IsBullet() || typeB !== 2 /* b2_dynamicBody */;
                    // Are these two non-bullet dynamic bodies?
                    if (!collideA && !collideB) {
                        continue;
                    }
                    // Compute the TOI for this contact.
                    // Put the sweeps onto the same time interval.
                    let alpha0 = bA.m_sweep.alpha0;
                    if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0) {
                        alpha0 = bB.m_sweep.alpha0;
                        bA.m_sweep.Advance(alpha0);
                    }
                    else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0) {
                        alpha0 = bA.m_sweep.alpha0;
                        bB.m_sweep.Advance(alpha0);
                    }
                    !!B2_DEBUG && b2Assert(alpha0 < 1);
                    const indexA = c.GetChildIndexA();
                    const indexB = c.GetChildIndexB();
                    // Compute the time of impact in interval [0, minTOI]
                    const input = SolveTOI_s_toi_input;
                    input.proxyA.SetShape(fA.GetShape(), indexA);
                    input.proxyB.SetShape(fB.GetShape(), indexB);
                    input.sweepA.Copy(bA.m_sweep);
                    input.sweepB.Copy(bB.m_sweep);
                    input.tMax = 1;
                    const output = SolveTOI_s_toi_output;
                    b2TimeOfImpact(output, input);
                    // Beta is the fraction of the remaining portion of the .
                    const beta = output.t;
                    if (output.state === 3 /* e_touching */) {
                        alpha = b2Min(alpha0 + (1 - alpha0) * beta, 1);
                    }
                    else {
                        alpha = 1;
                    }
                    c.m_toi = alpha;
                    c.m_toiFlag = true;
                }
                if (alpha < minAlpha) {
                    // This is the minimum TOI found so far.
                    minContact = c;
                    minAlpha = alpha;
                }
            }
            if (minContact === null || 1 - 10 * b2_epsilon < minAlpha) {
                // No more TOI events. Done!
                this.m_stepComplete = true;
                break;
            }
            // Advance the bodies to the TOI.
            const fA = minContact.GetFixtureA();
            const fB = minContact.GetFixtureB();
            const bA = fA.GetBody();
            const bB = fB.GetBody();
            const backup1 = SolveTOI_s_backup1.Copy(bA.m_sweep);
            const backup2 = SolveTOI_s_backup2.Copy(bB.m_sweep);
            bA.Advance(minAlpha);
            bB.Advance(minAlpha);
            // The TOI contact likely has some new contact points.
            minContact.Update(this.m_contactManager.m_contactListener);
            minContact.m_toiFlag = false;
            ++minContact.m_toiCount;
            // Is the contact solid?
            if (!minContact.IsEnabled() || !minContact.IsTouching()) {
                // Restore the sweeps.
                minContact.SetEnabled(false);
                bA.m_sweep.Copy(backup1);
                bB.m_sweep.Copy(backup2);
                bA.SynchronizeTransform();
                bB.SynchronizeTransform();
                continue;
            }
            bA.SetAwake(true);
            bB.SetAwake(true);
            // Build the island
            island.Clear();
            island.AddBody(bA);
            island.AddBody(bB);
            island.AddContact(minContact);
            bA.m_islandFlag = true;
            bB.m_islandFlag = true;
            minContact.m_islandFlag = true;
            // Get contacts on bodyA and bodyB.
            // const bodies: b2Body[] = [bA, bB];
            for (let i = 0; i < 2; ++i) {
                const body = i === 0 ? bA : bB; // bodies[i];
                if (body.m_type === 2 /* b2_dynamicBody */) {
                    for (let ce = body.m_contactList; ce; ce = ce.next) {
                        if (island.m_bodyCount === island.m_bodyCapacity) {
                            break;
                        }
                        if (island.m_contactCount === island.m_contactCapacity) {
                            break;
                        }
                        const contact = ce.contact;
                        // Has this contact already been added to the island?
                        if (contact.m_islandFlag) {
                            continue;
                        }
                        // Only add static, kinematic, or bullet bodies.
                        const other = ce.other;
                        if (other.m_type === 2 /* b2_dynamicBody */ &&
                            !body.IsBullet() &&
                            !other.IsBullet()) {
                            continue;
                        }
                        // Skip sensors.
                        const sensorA = contact.m_fixtureA.m_isSensor;
                        const sensorB = contact.m_fixtureB.m_isSensor;
                        if (sensorA || sensorB) {
                            continue;
                        }
                        // Tentatively advance the body to the TOI.
                        const backup = SolveTOI_s_backup.Copy(other.m_sweep);
                        if (!other.m_islandFlag) {
                            other.Advance(minAlpha);
                        }
                        // Update the contact points
                        contact.Update(this.m_contactManager.m_contactListener);
                        // Was the contact disabled by the user?
                        if (!contact.IsEnabled()) {
                            other.m_sweep.Copy(backup);
                            other.SynchronizeTransform();
                            continue;
                        }
                        // Are there contact points?
                        if (!contact.IsTouching()) {
                            other.m_sweep.Copy(backup);
                            other.SynchronizeTransform();
                            continue;
                        }
                        // Add the contact to the island
                        contact.m_islandFlag = true;
                        island.AddContact(contact);
                        // Has the other body already been added to the island?
                        if (other.m_islandFlag) {
                            continue;
                        }
                        // Add the other body to the island.
                        other.m_islandFlag = true;
                        if (other.m_type !== 0 /* b2_staticBody */) {
                            other.SetAwake(true);
                        }
                        island.AddBody(other);
                    }
                }
            }
            const subStep = SolveTOI_s_subStep;
            subStep.dt = (1 - minAlpha) * step.dt;
            subStep.inv_dt = 1 / subStep.dt;
            subStep.dtRatio = 1;
            subStep.positionIterations = 20;
            subStep.velocityIterations = step.velocityIterations;
            if (B2_ENABLE_PARTICLE) {
                subStep.particleIterations = step.particleIterations;
            }
            subStep.warmStarting = false;
            island.SolveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);
            // Reset island flags and synchronize broad-phase proxies.
            for (let i = 0; i < island.m_bodyCount; ++i) {
                const body = island.m_bodies[i];
                body.m_islandFlag = false;
                if (body.m_type !== 2 /* b2_dynamicBody */) {
                    continue;
                }
                body.SynchronizeFixtures();
                // Invalidate all contact TOIs on this displaced body.
                for (let ce = body.m_contactList; ce; ce = ce.next) {
                    ce.contact.m_toiFlag = false;
                    ce.contact.m_islandFlag = false;
                }
            }
            // Commit fixture proxy movements to the broad-phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            this.m_contactManager.FindNewContacts();
            if (this.m_subStepping) {
                this.m_stepComplete = false;
                break;
            }
        }
    }
    AddController(controller) {
        if (!B2_ENABLE_CONTROLLER) {
            return controller;
        }
        // b2Assert(controller.m_world === null, "Controller can only be a member of one world");
        // controller.m_world = this;
        controller.m_next = this.m_controllerList;
        controller.m_prev = null;
        if (this.m_controllerList) {
            this.m_controllerList.m_prev = controller;
        }
        this.m_controllerList = controller;
        ++this.m_controllerCount;
        return controller;
    }
    RemoveController(controller) {
        if (!B2_ENABLE_CONTROLLER) {
            return controller;
        }
        // b2Assert(controller.m_world === this, "Controller is not a member of this world");
        if (controller.m_prev) {
            controller.m_prev.m_next = controller.m_next;
        }
        if (controller.m_next) {
            controller.m_next.m_prev = controller.m_prev;
        }
        if (this.m_controllerList === controller) {
            this.m_controllerList = controller.m_next;
        }
        --this.m_controllerCount;
        controller.m_prev = null;
        controller.m_next = null;
        // delete controller.m_world; // = null;
        return controller;
    }
}
b2World.QueryFixtureShape_s_aabb = new b2AABB();
b2World.RayCast_s_input = new b2RayCastInput();
b2World.RayCast_s_output = new b2RayCastOutput();
b2World.RayCast_s_point = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJXb3JsZC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9keW5hbWljcy9iMldvcmxkLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsT0FBTyxFQUNMLFVBQVUsRUFDVixXQUFXLEVBQ1gsY0FBYyxFQUNkLGlCQUFpQixFQUNqQixRQUFRLEdBQ1QsTUFBTSxzQkFBc0IsQ0FBQztBQUM5QixPQUFPLEVBQUUsS0FBSyxFQUFFLE9BQU8sRUFBZSxNQUFNLEVBQU0sTUFBTSxrQkFBa0IsQ0FBQztBQUMzRSxPQUFPLEVBQUUsT0FBTyxFQUFFLE1BQU0sbUJBQW1CLENBQUM7QUFDNUMsT0FBTyxFQUNMLE1BQU0sRUFDTixjQUFjLEVBQ2QsZUFBZSxFQUNmLGtCQUFrQixHQUNuQixNQUFNLDBCQUEwQixDQUFDO0FBRWxDLE9BQU8sRUFDTCxjQUFjLEVBQ2QsVUFBVSxFQUNWLFdBQVcsR0FFWixNQUFNLDZCQUE2QixDQUFDO0FBSXJDLE9BQU8sRUFBRSxXQUFXLEVBQW1CLE1BQU0sc0JBQXNCLENBQUM7QUFDcEUsT0FBTyxFQUFFLGVBQWUsRUFBdUIsTUFBTSwwQkFBMEIsQ0FBQztBQUNoRixPQUFPLEVBQUUsZUFBZSxFQUF1QixNQUFNLDBCQUEwQixDQUFDO0FBQ2hGLE9BQU8sRUFBRSxXQUFXLEVBQW1CLE1BQU0sc0JBQXNCLENBQUM7QUFDcEUsT0FBTyxFQUFvQixZQUFZLEVBQUUsTUFBTSx1QkFBdUIsQ0FBQztBQUN2RSxPQUFPLEVBQW9CLFlBQVksRUFBRSxNQUFNLHVCQUF1QixDQUFDO0FBQ3ZFLE9BQU8sRUFBd0IsZ0JBQWdCLEVBQUUsTUFBTSwyQkFBMkIsQ0FBQztBQUNuRixPQUFPLEVBQXFCLGFBQWEsRUFBRSxNQUFNLHdCQUF3QixDQUFDO0FBQzFFLE9BQU8sRUFBdUIsZUFBZSxFQUFFLE1BQU0sMEJBQTBCLENBQUM7QUFDaEYsT0FBTyxFQUFtQixXQUFXLEVBQUUs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QUcsSUFBSSxDQUFDO1FBQ3pCLFVBQVUsQ0FBQyxNQUFNLEdBQUcsSUFBSSxDQUFDO1FBQ3pCLHdDQUF3QztRQUN4QyxPQUFPLFVBQVUsQ0FBQztJQUNwQixDQUFDOztBQXozQmMsZ0NBQXdCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQXVIeEMsdUJBQWUsR0FBRyxJQUFJLGNBQWMsRUFBRSxDQUFDO0FBQ3ZDLHdCQUFnQixHQUFHLElBQUksZUFBZSxFQUFFLENBQUM7QUFDekMsdUJBQWUsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDIiwic291cmNlc0NvbnRlbnQiOlsiLypcclxuICogQ29weXJpZ2h0IChjKSAyMDA2LTIwMTEgRXJpbiBDYXR0byBodHRwOi8vd3d3LmJveDJkLm9yZ1xyXG4gKlxyXG4gKiBUaGlzIHNvZnR3YXJlIGlzIHByb3ZpZGVkICdhcy1pcycsIHdpdGhvdXQgYW55IGV4cHJlc3Mgb3IgaW1wbGllZFxyXG4gKiB3YXJyYW50eS4gIEluIG5vIGV2ZW50IHdpbGwgdGhlIGF1dGhvcnMgYmUgaGVsZCBsaWFibGUgZm9yIGFueSBkYW1hZ2VzXHJcbiAqIGFyaXNpbmcgZnJvbSB0aGUgdXNlIG9mIHRoaXMgc29mdHdhcmUuXHJcbiAqIFBlcm1pc3Npb24gaXMgZ3JhbnRlZCB0byBhbnlvbmUgdG8gdXNlIHRoaXMgc29mdHdhcmUgZm9yIGFueSBwdXJwb3NlLFxyXG4gKiBpbmNsdWRpbmcgY29tbWVyY2lhbCBhcHBsaWNhdGlvbnMsIGFuZCB0byBhbHRlciBpdCBhbmQgcmVkaXN0cmlidXRlIGl0XHJcbiAqIGZyZWVseSwgc3ViamVjdCB0byB0aGUgZm9sbG93aW5nIHJlc3RyaWN0aW9uczpcclxuICogMS4gVGhlIG9yaWdpbiBvZiB0aGlzIHNvZnR3YXJlIG11c3Qgbm90IGJlIG1pc3JlcHJlc2VudGVkOyB5b3UgbXVzdCBub3RcclxuICogY2xhaW0gdGhhdCB5b3Ugd3JvdGUgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLiBJZiB5b3UgdXNlIHRoaXMgc29mdHdhcmVcclxuICogaW4gYSBwcm9kdWN0LCBhbiBhY2tub3dsZWRnbWVudCBpbiB0aGUgcHJvZHVjdCBkb2N1bWVudGF0aW9uIHdvdWxkIGJlXHJcbiAqIGFwcHJlY2lhdGVkIGJ1dCBpcyBub3QgcmVxdWlyZWQuXHJcbiAqIDIuIEFsdGVyZWQgc291cmNlIHZlcnNpb25zIG11c3QgYmUgcGxhaW5seSBtYXJrZWQgYXMgc3VjaCwgYW5kIG11c3Qgbm90IGJlXHJcbiAqIG1pc3JlcHJlc2VudGVkIGFzIGJlaW5nIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS5cclxuICogMy4gVGhpcyBub3RpY2UgbWF5IG5vdCBiZSByZW1vdmVkIG9yIGFsdGVyZWQgZnJvbSBhbnkgc291cmNlIGRpc3RyaWJ1dGlvbi5cclxuICovXHJcblxyXG5pbXBvcnQge1xyXG4gIGIyX2Vwc2lsb24sXHJcbiAgYjJfbWF4RmxvYXQsXHJcbiAgYjJfbWF4U3ViU3RlcHMsXHJcbiAgYjJfbWF4VE9JQ29udGFjdHMsXHJcbiAgYjJBc3NlcnQsXHJcbn0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQgeyBiMk1pbiwgYjJTd2VlcCwgYjJUcmFuc2Zvcm0sIGIyVmVjMiwgWFkgfSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJUaW1lciB9IGZyb20gJy4uL2NvbW1vbi9iMlRpbWVyJztcclxuaW1wb3J0IHtcclxuICBiMkFBQkIsXHJcbiAgYjJSYXlDYXN0SW5wdXQsXHJcbiAgYjJSYXlDYXN0T3V0cHV0LFxyXG4gIGIyVGVzdE92ZXJsYXBTaGFwZSxcclxufSBmcm9tICcuLi9jb2xsaXNpb24vYjJDb2xsaXNpb24nO1xyXG5pbXBvcnQgeyBiMlRyZWVOb2RlIH0gZnJvbSAnLi4vY29sbGlzaW9uL2IyRHluYW1pY1RyZWUnO1xyXG5pbXBvcnQge1xyXG4gIGIyVGltZU9mSW1wYWN0LFxyXG4gIGIyVE9JSW5wdXQsXHJcbiAgYjJUT0lPdXRwdXQsXHJcbiAgYjJUT0lPdXRwdXRTdGF0ZSxcclxufSBmcm9tICcuLi9jb2xsaXNpb24vYjJUaW1lT2ZJbXBhY3QnO1xyXG5pbXBvcnQgeyBiMlNoYXBlIH0gZnJvbSAnLi4vY29sbGlzaW9uL3NoYXBlcy9iMlNoYXBlJztcclxuaW1wb3J0IHsgYjJDb250YWN0LCBiMkNvbnRhY3RFZGdlIH0gZnJvbSAnLi9jb250YWN0cy9iMkNvbnRhY3QnO1xyXG5pbXBvcnQgeyBiMklKb2ludERlZiwgYjJKb2ludCwgYjJKb2ludEVkZ2UsIGIySm9pbnRUeXBlIH0gZnJvbSAnLi9qb2ludHMvYjJKb2ludCc7XHJcbmltcG9ydCB7IGIyQXJlYUpvaW50LCBiMklBcmVhSm9pbnREZWYgfSBmcm9tICcuL2pvaW50cy9iMkFyZWFKb2ludCc7XHJcbmltcG9ydCB7IGIyRGlzdGFuY2VKb2ludCwgYjJJRGlzdGFuY2VKb2ludERlZiB9IGZyb20gJy4vam9pbnRzL2IyRGlzdGFuY2VKb2ludCc7XHJcbmltcG9ydCB7IGIyRnJpY3Rpb25Kb2ludCwgYjJJRnJpY3Rpb25Kb2ludERlZiB9IGZyb20gJy4vam9pbnRzL2IyRnJpY3Rpb25Kb2ludCc7XHJcbmltcG9ydCB7IGIyR2VhckpvaW50LCBiMklHZWFySm9pbnREZWYgfSBmcm9tICcuL2pvaW50cy9iMkdlYXJKb2ludCc7XHJcbmltcG9ydCB7IGIySU1vdG9ySm9pbnREZWYsIGIyTW90b3JKb2ludCB9IGZyb20gJy4vam9pbnRzL2IyTW90b3JKb2ludCc7XHJcbmltcG9ydCB7IGIySU1vdXNlSm9pbnREZWYsIGIyTW91c2VKb2ludCB9IGZyb20gJy4vam9pbnRzL2IyTW91c2VKb2ludCc7XHJcbmltcG9ydCB7IGIySVByaXNtYXRpY0pvaW50RGVmLCBiMlByaXNtYXRpY0pvaW50IH0gZnJvbSAnLi9qb2ludHMvYjJQcmlzbWF0aWNKb2ludCc7XHJcbmltcG9ydCB7IGIySVB1bGxleUpvaW50RGVmLCBiMlB1bGxleUpvaW50IH0gZnJvbSAnLi9qb2ludHMvYjJQdWxsZXlKb2ludCc7XHJcbmltcG9ydCB7IGIySVJldm9sdXRlSm9pbnREZWYsIGIyUmV2b2x1dGVKb2ludCB9IGZyb20gJy4vam9pbnRzL2IyUmV2b2x1dGVKb2ludCc7XHJcbmltcG9ydCB7IGIySVJvcGVKb2ludERlZiwgYjJSb3BlSm9pbnQgfSBmcm9tICcuL2pvaW50cy9iMlJvcGVKb2ludCc7XHJcbmltcG9ydCB7IGIySVdlbGRKb2ludERlZiwgYjJXZWxkSm9pbnQgfSBmcm9tICcuL2pvaW50cy9iMldlbGRKb2ludCc7XHJcbmltcG9ydCB7IGIySVdoZWVsSm9pbnREZWYsIGIyV2hlZWxKb2ludCB9IGZyb20gJy4vam9pbnRzL2IyV2hlZWxKb2ludCc7XHJcbmltcG9ydCB7IGIyQm9keSwgYjJCb2R5VHlwZSwgYjJJQm9keURlZiB9IGZyb20gJy4vYjJCb2R5JztcclxuaW1wb3J0IHsgYjJDb250YWN0TWFuYWdlciB9IGZyb20gJy4vYjJDb250YWN0TWFuYWdlcic7XHJcbmltcG9ydCB7IGIyRml4dHVyZSwgYjJGaXh0dXJlUHJveHkgfSBmcm9tICcuL2IyRml4dHVyZSc7XHJcbmltcG9ydCB7IGIySXNsYW5kIH0gZnJvbSAnLi9iMklzbGFuZCc7XHJcbmltcG9ydCB7IGIyUHJvZmlsZSwgYjJUaW1lU3RlcCB9IGZyb20gJy4vYjJUaW1lU3RlcCc7XHJcbmltcG9ydCB7XHJcbiAgYjJDb250YWN0RmlsdGVyLFxyXG4gIGIyQ29udGFjdExpc3RlbmVyLFxyXG4gIGIyRGVzdHJ1Y3Rpb25MaXN0ZW5lcixcclxuICBiMlF1ZXJ5Q2FsbGJhY2ssXHJcbiAgYjJRdWVyeUNhbGxiYWNrRnVuY3Rpb24sXHJcbiAgYjJSYXlDYXN0Q2FsbGJhY2ssXHJcbiAgYjJSYXlDYXN0Q2FsbGJhY2tGdW5jdGlvbixcclxufSBmcm9tICcuL2IyV29ybGRDYWxsYmFja3MnO1xyXG5pbXBvcnQgeyBiMkNhbGN1bGF0ZVBhcnRpY2xlSXRlcmF0aW9ucyB9IGZyb20gJy4uL3BhcnRpY2xlL2IyUGFydGljbGUnO1xyXG5pbXBvcnQgeyBiMlBhcnRpY2xlU3lzdGVtLCBiMlBhcnRpY2xlU3lzdGVtRGVmIH0gZnJvbSAnLi4vcGFydGljbGUvYjJQYXJ0aWNsZVN5c3RlbSc7XHJcbmltcG9ydCB7IGIyQ29udHJvbGxlciwgYjJDb250cm9sbGVyRWRnZSB9IGZyb20gJy4uL2NvbnRyb2xsZXJzL2IyQ29udHJvbGxlcic7XHJcblxyXG5jb25zdCBTb2x2ZVRPSV9zX3N1YlN0ZXAgPSBuZXcgYjJUaW1lU3RlcCgpO1xyXG5jb25zdCBTb2x2ZVRPSV9zX2JhY2t1cCA9IG5ldyBiMlN3ZWVwKCk7XHJcbmNvbnN0IFNvbHZlVE9JX3NfYmFja3VwMSA9IG5ldyBiMlN3ZWVwKCk7XHJcbmNvbnN0IFNvbHZlVE9JX3NfYmFja3VwMiA9IG5ldyBiMlN3ZWVwKCk7XHJcbmNvbnN0IFNvbHZlVE9JX3NfdG9pX2lucHV0ID0gbmV3IGIyVE9JSW5wdXQoKTtcclxuY29uc3QgU29sdmVUT0lfc190b2lfb3V0cHV0ID0gbmV3IGIyVE9JT3V0cHV0KCk7XHJcblxyXG4vLy8gVGFrZSBhIHRpbWUgc3RlcC4gVGhpcyBwZXJmb3JtcyBjb2xsaXNpb24gZGV0ZWN0aW9uLCBpbnRlZ3JhdGlvbixcclxuLy8vIGFuZCBjb25zdHJhaW50IHNvbHV0aW9uLlxyXG4vLy8gQHBhcmFtIHRpbWVTdGVwIHRoZSBhbW91bnQgb2YgdGltZSB0byBzaW11bGF0ZSwgdGhpcyBzaG91bGQgbm90IHZhcnkuXHJcbi8vLyBAcGFyYW0gdmVsb2NpdHlJdGVyYXRpb25zIGZvciB0aGUgdmVsb2NpdHkgY29uc3RyYWludCBzb2x2ZXIuXHJcbi8vLyBAcGFyYW0gcG9zaXRpb25JdGVyYXRpb25zIGZvciB0aGUgcG9zaXRpb24gY29uc3RyYWludCBzb2x2ZXIuXHJcbmNvbnN0IFN0ZXBfc19zdGVwID0gbmV3IGIyVGltZVN0ZXAoKTtcclxuY29uc3QgU3RlcF9zX3N0ZXBUaW1lciA9IG5ldyBiMlRpbWVyKCk7XHJcbmNvbnN0IFN0ZXBfc190aW1lciA9IG5ldyBiMlRpbWVyKCk7XHJcbmNvbnN0IFN0ZXBfc19icm9hZHBoYXNlVGltZXIgPSBuZXcgYjJUaW1lcigpO1xyXG5cclxuLy8vIFRoZSB3b3JsZCBjbGFzcyBtYW5hZ2VzIGFsbCBwaHlzaWNzIGVudGl0aWVzLCBkeW5hbWljIHNpbXVsYXRpb24sXHJcbi8vLyBhbmQgYXN5bmNocm9ub3VzIHF1ZXJpZXMuIFRoZSB3b3JsZCBhbHNvIGNvbnRhaW5zIGVmZmljaWVudCBtZW1vcnlcclxuLy8vIG1hbmFnZW1lbnQgZmFjaWxpdGllcy5cclxuZXhwb3J0IGNsYXNzIGIyV29ybGQge1xyXG4gIG1fbmV3Rml4dHVyZSA9IGZhbHNlO1xyXG4gIG1fbG9ja2VkID0gZmFsc2U7XHJcbiAgbV9jbGVhckZvcmNlcyA9IHRydWU7XHJcblxyXG4gIHJlYWRvbmx5IG1fY29udGFjdE1hbmFnZXIgPSBuZXcgYjJDb250YWN0TWFuYWdlcigpO1xyXG5cclxuICBtX2JvZHlMaXN0OiBiMkJvZHkgfCBudWxsID0gbnVsbDtcclxuICBtX2pvaW50TGlzdDogYjJKb2ludCB8IG51bGwgPSBudWxsO1xyXG5cclxuICAvLyAjaWYgQjJfRU5BQkxFX1BBUlRJQ0xFXHJcbiAgbV9wYXJ0aWNsZVN5c3RlbUxpc3Q6IGIyUGFydGljbGVTeXN0ZW0gfCBudWxsID0gbnVsbDtcclxuICAvLyAjZW5kaWZcclxuXHJcbiAgbV9ib2R5Q291bnQgPSAwO1xyXG4gIG1fam9pbnRDb3VudCA9IDA7XHJcblxyXG4gIHJlYWRvbmx5IG1fZ3Jhdml0eSA9IG5ldyBiMlZlYzIoKTtcclxuICBtX2FsbG93U2xlZXAgPSB0cnVlO1xyXG5cclxuICBtX2Rlc3RydWN0aW9uTGlzdGVuZXI6IGIyRGVzdHJ1Y3Rpb25MaXN0ZW5lciB8IG51bGwgPSBudWxsO1xyXG5cclxuICAvLyBUaGlzIGlzIHVzZWQgdG8gY29tcHV0ZSB0aGUgdGltZSBzdGVwIHJhdGlvIHRvXHJcbiAgLy8gc3VwcG9ydCBhIHZhcmlhYmxlIHRpbWUgc3RlcC5cclxuICBtX2ludl9kdDAgPSBOYU47XHJcblxyXG4gIC8vIFRoZXNlIGFyZSBmb3IgZGVidWdnaW5nIHRoZSBzb2x2ZXIuXHJcbiAgbV93YXJtU3RhcnRpbmcgPSB0cnVlO1xyXG4gIG1fY29udGludW91c1BoeXNpY3MgPSB0cnVlO1xyXG4gIG1fc3ViU3RlcHBpbmcgPSBmYWxzZTtcclxuXHJcbiAgbV9zdGVwQ29tcGxldGUgPSB0cnVlO1xyXG5cclxuICByZWFkb25seSBtX3Byb2ZpbGUgPSBuZXcgYjJQcm9maWxlKCk7XHJcbiAgcmVhZG9ubHkgbV9pc2xhbmQgPSBuZXcgYjJJc2xhbmQoKTtcclxuICByZWFkb25seSBzX3N0YWNrOiBBcnJheTxiMkJvZHkgfCBudWxsPiA9IFtudWxsXTtcclxuXHJcbiAgLy8gI2lmIEIyX0VOQUJMRV9DT05UUk9MTEVSXHJcbiAgbV9jb250cm9sbGVyTGlzdDogYjJDb250cm9sbGVyIHwgbnVsbCA9IG51bGw7XHJcbiAgbV9jb250cm9sbGVyQ291bnQgPSAwO1xyXG4gIC8vICNlbmRpZlxyXG5cclxuICAvLy8gQ29uc3RydWN0IGEgd29ybGQgb2JqZWN0LlxyXG4gIC8vLyBAcGFyYW0gZ3Jhdml0eSB0aGUgd29ybGQgZ3Jhdml0eSB2ZWN0b3IuXHJcbiAgY29uc3RydWN0b3IoZ3Jhdml0eTogWFkpIHtcclxuICAgIHRoaXMubV9pbnZfZHQwID0gMC4wO1xyXG4gICAgdGhpcy5tX2dyYXZpdHkuQ29weShncmF2aXR5KTtcclxuICB9XHJcblxyXG4gIC8vLyBSZWdpc3RlciBhIGRlc3RydWN0aW9uIGxpc3RlbmVyLiBUaGUgbGlzdGVuZXIgaXMgb3duZWQgYnkgeW91IGFuZCBtdXN0XHJcbiAgLy8vIHJlbWFpbiBpbiBzY29wZS5cclxuICBTZXREZXN0cnVjdGlvbkxpc3RlbmVyKGxpc3RlbmVyOiBiMkRlc3RydWN0aW9uTGlzdGVuZXIgfCBudWxsKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fZGVzdHJ1Y3Rpb25MaXN0ZW5lciA9IGxpc3RlbmVyO1xyXG4gIH1cclxuXHJcbiAgLy8vIFJlZ2lzdGVyIGEgY29udGFjdCBmaWx0ZXIgdG8gcHJvdmlkZSBzcGVjaWZpYyBjb250cm9sIG92ZXIgY29sbGlzaW9uLlxyXG4gIC8vLyBPdGhlcndpc2UgdGhlIGRlZmF1bHQgZmlsdGVyIGlzIHVzZWQgKGIyX2RlZmF1bHRGaWx0ZXIpLiBUaGUgbGlzdGVuZXIgaXNcclxuICAvLy8gb3duZWQgYnkgeW91IGFuZCBtdXN0IHJlbWFpbiBpbiBzY29wZS5cclxuICBTZXRDb250YWN0RmlsdGVyKGZpbHRlcjogYjJDb250YWN0RmlsdGVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fY29udGFjdE1hbmFnZXIubV9jb250YWN0RmlsdGVyID0gZmlsdGVyO1xyXG4gIH1cclxuXHJcbiAgLy8vIFJlZ2lzdGVyIGEgY29udGFjdCBldmVudCBsaXN0ZW5lci4gVGhlIGxpc3RlbmVyIGlzIG93bmVkIGJ5IHlvdSBhbmQgbXVzdFxyXG4gIC8vLyByZW1haW4gaW4gc2NvcGUuXHJcbiAgU2V0Q29udGFjdExpc3RlbmVyKGxpc3RlbmVyOiBiMkNvbnRhY3RMaXN0ZW5lcik6IHZvaWQge1xyXG4gICAgdGhpcy5tX2NvbnRhY3RNYW5hZ2VyLm1fY29udGFjdExpc3RlbmVyID0gbGlzdGVuZXI7XHJcbiAgfVxyXG5cclxuICAvLy8gQ3JlYXRlIGEgcmlnaWQgYm9keSBnaXZlbiBhIGRlZmluaXRpb24uIE5vIHJlZmVyZW5jZSB0byB0aGUgZGVmaW5pdGlvblxyXG4gIC8vLyBpcyByZXRhaW5lZC5cclxuICAvLy8gQHdhcm5pbmcgVGhpcyBmdW5jdGlvbiBpcyBsb2NrZWQgZHVyaW5nIGNhbGxiYWNrcy5cclxuICBDcmVhdGVCb2R5KGRlZjogYjJJQm9keURlZiA9IHt9KTogYjJCb2R5IHtcclxuICAgIGlmICh0aGlzLklzTG9ja2VkKCkpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgYjogYjJCb2R5ID0gbmV3IGIyQm9keShkZWYsIHRoaXMpO1xyXG5cclxuICAgIC8vIEFkZCB0byB3b3JsZCBkb3VibHkgbGlua2VkIGxpc3QuXHJcbiAgICBiLm1fcHJldiA9IG51bGw7XHJcbiAgICBiLm1fbmV4dCA9IHRoaXMubV9ib2R5TGlzdDtcclxuICAgIGlmICh0aGlzLm1fYm9keUxpc3QpIHtcclxuICAgICAgdGhpcy5tX2JvZHlMaXN0Lm1fcHJldiA9IGI7XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fYm9keUxpc3QgPSBiO1xyXG4gICAgKyt0aGlzLm1fYm9keUNvdW50O1xyXG5cclxuICAgIHJldHVybiBiO1xyXG4gIH1cclxuXHJcbiAgLy8vIERlc3Ryb3kgYSByaWdpZCBib2R5IGdpdmVuIGEgZGVmaW5pdGlvbi4gTm8gcmVmZXJlbmNlIHRvIHRoZSBkZWZpbml0aW9uXHJcbiAgLy8vIGlzIHJldGFpbmVkLiBUaGlzIGZ1bmN0aW9uIGlzIGxvY2tlZCBkdXJpbmcgY2FsbGJhY2tzLlxyXG4gIC8vLyBAd2FybmluZyBUaGlzIGF1dG9tYXRpY2FsbHkgZGVsZXRlcyBhbGwgYXNzb2NpYXRlZCBzaGFwZXMgYW5kIGpvaW50cy5cclxuICAvLy8gQHdhcm5pbmcgVGhpcyBmdW5jdGlvbiBpcyBsb2NrZWQgZHVyaW5nIGNhbGxiYWNrcy5cclxuICBEZXN0cm95Qm9keShiOiBiMkJvZHkpOiB2b2lkIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2JvZHlDb3VudCA+IDApO1xyXG4gICAgaWYgKHRoaXMuSXNMb2NrZWQoKSkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBEZWxldGUgdGhlIGF0dGFjaGVkIGpvaW50cy5cclxuICAgIGxldCBqZTogYjJKb2ludEVkZ2UgfCBudWxsID0gYi5tX2pvaW50TGlzdDtcclxuICAgIHdoaWxlIChqZSkge1xyXG4gICAgICBjb25zdCBqZTA6IGIySm9pbnRFZGdlID0gamU7XHJcbiAgICAgIGplID0gamUubmV4dDtcclxuXHJcbiAgICAgIGlmICh0aGlzLm1fZGVzdHJ1Y3Rpb25MaXN0ZW5lcikge1xyXG4gICAgICAgIHRoaXMubV9kZXN0cnVjdGlvbkxpc3RlbmVyLlNheUdvb2RieWVKb2ludChqZTAuam9pbnQpO1xyXG4gICAgICB9XHJcblxyXG4gICAgICB0aGlzLkRlc3Ryb3lKb2ludChqZTAuam9pbnQpO1xyXG5cclxuICAgICAgYi5tX2pvaW50TGlzdCA9IGplO1xyXG4gICAgfVxyXG4gICAgYi5tX2pvaW50TGlzdCA9IG51bGw7XHJcblxyXG4gICAgaWYgKEIyX0VOQUJMRV9DT05UUk9MTEVSKSB7XHJcbiAgICAgIC8vIEBzZWUgYjJDb250cm9sbGVyIGxpc3RcclxuICAgICAgbGV0IGNvZTogYjJDb250cm9sbGVyRWRnZSB8IG51bGwgPSBiLm1fY29udHJvbGxlckxpc3Q7XHJcbiAgICAgIHdoaWxlIChjb2UpIHtcclxuICAgICAgICBjb25zdCBjb2UwOiBiMkNvbnRyb2xsZXJFZGdlID0gY29lO1xyXG4gICAgICAgIGNvZSA9IGNvZS5uZXh0Q29udHJvbGxlcjtcclxuICAgICAgICBjb2UwLmNvbnRyb2xsZXIuUmVtb3ZlQm9keShiKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIC8vIERlbGV0ZSB0aGUgYXR0YWNoZWQgY29udGFjdHMuXHJcbiAgICBsZXQgY2U6IGIyQ29udGFjdEVkZ2UgfCBudWxsID0gYi5tX2NvbnRhY3RMaXN0O1xyXG4gICAgd2hpbGUgKGNlKSB7XHJcbiAgICAgIGNvbnN0IGNlMDogYjJDb250YWN0RWRnZSA9IGNlO1xyXG4gICAgICBjZSA9IGNlLm5leHQ7XHJcbiAgICAgIHRoaXMubV9jb250YWN0TWFuYWdlci5EZXN0cm95KGNlMC5jb250YWN0KTtcclxuICAgIH1cclxuICAgIGIubV9jb250YWN0TGlzdCA9IG51bGw7XHJcblxyXG4gICAgLy8gRGVsZXRlIHRoZSBhdHRhY2hlZCBmaXh0dXJlcy4gVGhpcyBkZXN0cm95cyBicm9hZC1waGFzZSBwcm94aWVzLlxyXG4gICAgbGV0IGY6IGIyRml4dHVyZSB8IG51bGwgPSBiLm1fZml4dHVyZUxpc3Q7XHJcbiAgICB3aGlsZSAoZikge1xyXG4gICAgICBjb25zdCBmMDogYjJGaXh0dXJlID0gZjtcclxuICAgICAgZiA9IGYubV9uZXh0O1xyXG5cclxuICAgICAgaWYgKHRoaXMubV9kZXN0cnVjdGlvbkxpc3RlbmVyKSB7XHJcbiAgICAgICAgdGhpcy5tX2Rlc3RydWN0aW9uTGlzdGVuZXIuU2F5R29vZGJ5ZUZpeHR1cmUoZjApO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBmMC5EZXN0cm95UHJveGllcygpO1xyXG4gICAgICBmMC5SZXNldCgpO1xyXG5cclxuICAgICAgYi5tX2ZpeHR1cmVMaXN0ID0gZjtcclxuICAgICAgYi5tX2ZpeHR1cmVDb3VudCAtPSAxO1xyXG4gICAgfVxyXG4gICAgYi5tX2ZpeHR1cmVMaXN0ID0gbnVsbDtcclxuICAgIGIubV9maXh0dXJlQ291bnQgPSAwO1xyXG5cclxuICAgIC8vIFJlbW92ZSB3b3JsZCBib2R5IGxpc3QuXHJcbiAgICBpZiAoYi5tX3ByZXYpIHtcclxuICAgICAgYi5tX3ByZXYubV9uZXh0ID0gYi5tX25leHQ7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGIubV9uZXh0KSB7XHJcbiAgICAgIGIubV9uZXh0Lm1fcHJldiA9IGIubV9wcmV2O1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChiID09PSB0aGlzLm1fYm9keUxpc3QpIHtcclxuICAgICAgdGhpcy5tX2JvZHlMaXN0ID0gYi5tX25leHQ7XHJcbiAgICB9XHJcblxyXG4gICAgLS10aGlzLm1fYm9keUNvdW50O1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgX0pvaW50X0NyZWF0ZShkZWY6IGIySUpvaW50RGVmKTogYjJKb2ludCB7XHJcbiAgICBzd2l0Y2ggKGRlZi50eXBlKSB7XHJcbiAgICAgIGNhc2UgYjJKb2ludFR5cGUuZV9kaXN0YW5jZUpvaW50OlxyXG4gICAgICAgIHJldHVybiBuZXcgYjJEaXN0YW5jZUpvaW50KGRlZiBhcyBiMklEaXN0YW5jZUpvaW50RGVmKTtcclxuICAgICAgY2FzZSBiMkpvaW50VHlwZS5lX21vdXNlSm9pbnQ6XHJcbiAgICAgICAgcmV0dXJuIG5ldyBiMk1vdXNlSm9pbnQoZGVmIGFzIGIySU1vdXNlSm9pbnREZWYpO1xyXG4gICAgICBjYXNlIGIySm9pbnRUeXBlLmVfcHJpc21hdGljSm9pbnQ6XHJcbiAgICAgICAgcmV0dXJuIG5ldyBiMlByaXNtYXRpY0pvaW50KGRlZiBhcyBiMklQcmlzbWF0aWNKb2ludERlZik7XHJcbiAgICAgIGNhc2UgYjJKb2ludFR5cGUuZV9yZXZvbHV0ZUpvaW50OlxyXG4gICAgICAgIHJldHVybiBuZXcgYjJSZXZvbHV0ZUpvaW50KGRlZiBhcyBiMklSZXZvbHV0ZUpvaW50RGVmKTtcclxuICAgICAgY2FzZSBiMkpvaW50VHlwZS5lX3B1bGxleUpvaW50OlxyXG4gICAgICAgIHJldHVybiBuZXcgYjJQdWxsZXlKb2ludChkZWYgYXMgYjJJUHVsbGV5Sm9pbnREZWYpO1xyXG4gICAgICBjYXNlIGIySm9pbnRUeXBlLmVfZ2VhckpvaW50OlxyXG4gICAgICAgIHJldHVybiBuZXcgYjJHZWFySm9pbnQoZGVmIGFzIGIySUdlYXJKb2ludERlZik7XHJcbiAgICAgIGNhc2UgYjJKb2ludFR5cGUuZV93aGVlbEpvaW50OlxyXG4gICAgICAgIHJldHVybiBuZXcgYjJXaGVlbEpvaW50KGRlZiBhcyBiMklXaGVlbEpvaW50RGVmKTtcclxuICAgICAgY2FzZSBiMkpvaW50VHlwZS5lX3dlbGRKb2ludDpcclxuICAgICAgICByZXR1cm4gbmV3IGIyV2VsZEpvaW50KGRlZiBhcyBiMklXZWxkSm9pbnREZWYpO1xyXG4gICAgICBjYXNlIGIySm9pbnRUeXBlLmVfZnJpY3Rpb25Kb2ludDpcclxuICAgICAgICByZXR1cm4gbmV3IGIyRnJpY3Rpb25Kb2ludChkZWYgYXMgYjJJRnJpY3Rpb25Kb2ludERlZik7XHJcbiAgICAgIGNhc2UgYjJKb2ludFR5cGUuZV9yb3BlSm9pbnQ6XHJcbiAgICAgICAgcmV0dXJuIG5ldyBiMlJvcGVKb2ludChkZWYgYXMgYjJJUm9wZUpvaW50RGVmKTtcclxuICAgICAgY2FzZSBiMkpvaW50VHlwZS5lX21vdG9ySm9pbnQ6XHJcbiAgICAgICAgcmV0dXJuIG5ldyBiMk1vdG9ySm9pbnQoZGVmIGFzIGIySU1vdG9ySm9pbnREZWYpO1xyXG4gICAgICBjYXNlIGIySm9pbnRUeXBlLmVfYXJlYUpvaW50OlxyXG4gICAgICAgIHJldHVybiBuZXcgYjJBcmVhSm9pbnQoZGVmIGFzIGIySUFyZWFKb2ludERlZik7XHJcbiAgICB9XHJcbiAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICB9XHJcblxyXG4gIC8vIGVzbGludC1kaXNhYmxlLW5leHQtbGluZSBAdHlwZXNjcmlwdC1lc2xpbnQvbm8tZW1wdHktZnVuY3Rpb25cclxuICBwcml2YXRlIHN0YXRpYyBfSm9pbnRfRGVzdHJveShqb2ludDogYjJKb2ludCk6IHZvaWQge31cclxuXHJcbiAgLy8vIENyZWF0ZSBhIGpvaW50IHRvIGNvbnN0cmFpbiBib2RpZXMgdG9nZXRoZXIuIE5vIHJlZmVyZW5jZSB0byB0aGUgZGVmaW5pdGlvblxyXG4gIC8vLyBpcyByZXRhaW5lZC4gVGhpcyBtYXkgY2F1c2UgdGhlIGNvbm5lY3RlZCBib2RpZXMgdG8gY2Vhc2UgY29sbGlkaW5nLlxyXG4gIC8vLyBAd2FybmluZyBUaGlzIGZ1bmN0aW9uIGlzIGxvY2tlZCBkdXJpbmcgY2FsbGJhY2tzLlxyXG4gIENyZWF0ZUpvaW50KGRlZjogYjJJQXJlYUpvaW50RGVmKTogYjJBcmVhSm9pbnQ7XHJcbiAgQ3JlYXRlSm9pbnQoZGVmOiBiMklEaXN0YW5jZUpvaW50RGVmKTogYjJEaXN0YW5jZUpvaW50O1xyXG4gIENyZWF0ZUpvaW50KGRlZjogYjJJRnJpY3Rpb25Kb2ludERlZik6IGIyRnJpY3Rpb25Kb2ludDtcclxuICBDcmVhdGVKb2ludChkZWY6IGIySUdlYXJKb2ludERlZik6IGIyR2VhckpvaW50O1xyXG4gIENyZWF0ZUpvaW50KGRlZjogYjJJTW90b3JKb2ludERlZik6IGIyTW90b3JKb2ludDtcclxuICBDcmVhdGVKb2ludChkZWY6IGIySU1vdXNlSm9pbnREZWYpOiBiMk1vdXNlSm9pbnQ7XHJcbiAgQ3JlYXRlSm9pbnQoZGVmOiBiMklQcmlzbWF0aWNKb2ludERlZik6IGIyUHJpc21hdGljSm9pbnQ7XHJcbiAgQ3JlYXRlSm9pbnQoZGVmOiBiMklQdWxsZXlKb2ludERlZik6IGIyUHVsbGV5Sm9pbnQ7XHJcbiAgQ3JlYXRlSm9pbnQoZGVmOiBiMklSZXZvbHV0ZUpvaW50RGVmKTogYjJSZXZvbHV0ZUpvaW50O1xyXG4gIENyZWF0ZUpvaW50KGRlZjogYjJJUm9wZUpvaW50RGVmKTogYjJSb3BlSm9pbnQ7XHJcbiAgQ3JlYXRlSm9pbnQoZGVmOiBiMklXZWxkSm9pbnREZWYpOiBiMldlbGRKb2ludDtcclxuICBDcmVhdGVKb2ludChkZWY6IGIySVdoZWVsSm9pbnREZWYpOiBiMldoZWVsSm9pbnQ7XHJcbiAgQ3JlYXRlSm9pbnQoZGVmOiBiMklKb2ludERlZik6IGIySm9pbnQge1xyXG4gICAgaWYgKHRoaXMuSXNMb2NrZWQoKSkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuXHJcbiAgICBjb25zdCBqOiBiMkpvaW50ID0gYjJXb3JsZC5fSm9pbnRfQ3JlYXRlKGRlZik7XHJcblxyXG4gICAgLy8gQ29ubmVjdCB0byB0aGUgd29ybGQgbGlzdC5cclxuICAgIGoubV9wcmV2ID0gbnVsbDtcclxuICAgIGoubV9uZXh0ID0gdGhpcy5tX2pvaW50TGlzdDtcclxuICAgIGlmICh0aGlzLm1fam9pbnRMaXN0KSB7XHJcbiAgICAgIHRoaXMubV9qb2ludExpc3QubV9wcmV2ID0gajtcclxuICAgIH1cclxuICAgIHRoaXMubV9qb2ludExpc3QgPSBqO1xyXG4gICAgKyt0aGlzLm1fam9pbnRDb3VudDtcclxuXHJcbiAgICAvLyBDb25uZWN0IHRvIHRoZSBib2RpZXMnIGRvdWJseSBsaW5rZWQgbGlzdHMuXHJcbiAgICAvLyBqLm1fZWRnZUEub3RoZXIgPSBqLm1fYm9keUI7IC8vIGRvbmUgaW4gYjJKb2ludCBjb25zdHJ1Y3RvclxyXG4gICAgai5tX2VkZ2VBLnByZXYgPSBudWxsO1xyXG4gICAgai5tX2VkZ2VBLm5leHQgPSBqLm1fYm9keUEubV9qb2ludExpc3Q7XHJcbiAgICBpZiAoai5tX2JvZHlBLm1fam9pbnRMaXN0KSB7XHJcbiAgICAgIGoubV9ib2R5QS5tX2pvaW50TGlzdC5wcmV2ID0gai5tX2VkZ2VBO1xyXG4gICAgfVxyXG4gICAgai5tX2JvZHlBLm1fam9pbnRMaXN0ID0gai5tX2VkZ2VBO1xyXG5cclxuICAgIC8vIGoubV9lZGdlQi5vdGhlciA9IGoubV9ib2R5QTsgLy8gZG9uZSBpbiBiMkpvaW50IGNvbnN0cnVjdG9yXHJcbiAgICBqLm1fZWRnZUIucHJldiA9IG51bGw7XHJcbiAgICBqLm1fZWRnZUIubmV4dCA9IGoubV9ib2R5Qi5tX2pvaW50TGlzdDtcclxuICAgIGlmIChqLm1fYm9keUIubV9qb2ludExpc3QpIHtcclxuICAgICAgai5tX2JvZHlCLm1fam9pbnRMaXN0LnByZXYgPSBqLm1fZWRnZUI7XHJcbiAgICB9XHJcbiAgICBqLm1fYm9keUIubV9qb2ludExpc3QgPSBqLm1fZWRnZUI7XHJcblxyXG4gICAgY29uc3QgYm9keUE6IGIyQm9keSA9IGoubV9ib2R5QTtcclxuICAgIGNvbnN0IGJvZHlCOiBiMkJvZHkgPSBqLm1fYm9keUI7XHJcbiAgICBjb25zdCBjb2xsaWRlQ29ubmVjdGVkOiBib29sZWFuID0gai5tX2NvbGxpZGVDb25uZWN0ZWQ7XHJcblxyXG4gICAgLy8gSWYgdGhlIGpvaW50IHByZXZlbnRzIGNvbGxpc2lvbnMsIHRoZW4gZmxhZyBhbnkgY29udGFjdHMgZm9yIGZpbHRlcmluZy5cclxuICAgIGlmICghY29sbGlkZUNvbm5lY3RlZCkge1xyXG4gICAgICBsZXQgZWRnZTogYjJDb250YWN0RWRnZSB8IG51bGwgPSBib2R5Qi5HZXRDb250YWN0TGlzdCgpO1xyXG4gICAgICB3aGlsZSAoZWRnZSkge1xyXG4gICAgICAgIGlmIChlZGdlLm90aGVyID09PSBib2R5QSkge1xyXG4gICAgICAgICAgLy8gRmxhZyB0aGUgY29udGFjdCBmb3IgZmlsdGVyaW5nIGF0IHRoZSBuZXh0IHRpbWUgc3RlcCAod2hlcmUgZWl0aGVyXHJcbiAgICAgICAgICAvLyBib2R5IGlzIGF3YWtlKS5cclxuICAgICAgICAgIGVkZ2UuY29udGFjdC5GbGFnRm9yRmlsdGVyaW5nKCk7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBlZGdlID0gZWRnZS5uZXh0O1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gTm90ZTogY3JlYXRpbmcgYSBqb2ludCBkb2Vzbid0IHdha2UgdGhlIGJvZGllcy5cclxuXHJcbiAgICByZXR1cm4gajtcclxuICB9XHJcblxyXG4gIC8vLyBEZXN0cm95IGEgam9pbnQuIFRoaXMgbWF5IGNhdXNlIHRoZSBjb25uZWN0ZWQgYm9kaWVzIHRvIGJlZ2luIGNvbGxpZGluZy5cclxuICAvLy8gQHdhcm5pbmcgVGhpcyBmdW5jdGlvbiBpcyBsb2NrZWQgZHVyaW5nIGNhbGxiYWNrcy5cclxuICBEZXN0cm95Sm9pbnQoajogYjJKb2ludCk6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMuSXNMb2NrZWQoKSkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBSZW1vdmUgZnJvbSB0aGUgZG91Ymx5IGxpbmtlZCBsaXN0LlxyXG4gICAgaWYgKGoubV9wcmV2KSB7XHJcbiAgICAgIGoubV9wcmV2Lm1fbmV4dCA9IGoubV9uZXh0O1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChqLm1fbmV4dCkge1xyXG4gICAgICBqLm1fbmV4dC5tX3ByZXYgPSBqLm1fcHJldjtcclxuICAgIH1cclxuXHJcbiAgICBpZiAoaiA9PT0gdGhpcy5tX2pvaW50TGlzdCkge1xyXG4gICAgICB0aGlzLm1fam9pbnRMaXN0ID0gai5tX25leHQ7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gRGlzY29ubmVjdCBmcm9tIGlzbGFuZCBncmFwaC5cclxuICAgIGNvbnN0IGJvZHlBOiBiMkJvZHkgPSBqLm1fYm9keUE7XHJcbiAgICBjb25zdCBib2R5QjogYjJCb2R5ID0gai5tX2JvZHlCO1xyXG4gICAgY29uc3QgY29sbGlkZUNvbm5lY3RlZDogYm9vbGVhbiA9IGoubV9jb2xsaWRlQ29ubmVjdGVkO1xyXG5cclxuICAgIC8vIFdha2UgdXAgY29ubmVjdGVkIGJvZGllcy5cclxuICAgIGJvZHlBLlNldEF3YWtlKHRydWUpO1xyXG4gICAgYm9keUIuU2V0QXdha2UodHJ1ZSk7XHJcblxyXG4gICAgLy8gUmVtb3ZlIGZyb20gYm9keSAxLlxyXG4gICAgaWYgKGoubV9lZGdlQS5wcmV2KSB7XHJcbiAgICAgIGoubV9lZGdlQS5wcmV2Lm5leHQgPSBqLm1fZWRnZUEubmV4dDtcclxuICAgIH1cclxuXHJcbiAgICBpZiAoai5tX2VkZ2VBLm5leHQpIHtcclxuICAgICAgai5tX2VkZ2VBLm5leHQucHJldiA9IGoubV9lZGdlQS5wcmV2O1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChqLm1fZWRnZUEgPT09IGJvZHlBLm1fam9pbnRMaXN0KSB7XHJcbiAgICAgIGJvZHlBLm1fam9pbnRMaXN0ID0gai5tX2VkZ2VBLm5leHQ7XHJcbiAgICB9XHJcblxyXG4gICAgai5tX2VkZ2VBLlJlc2V0KCk7XHJcblxyXG4gICAgLy8gUmVtb3ZlIGZyb20gYm9keSAyXHJcbiAgICBpZiAoai5tX2VkZ2VCLnByZXYpIHtcclxuICAgICAgai5tX2VkZ2VCLnByZXYubmV4dCA9IGoubV9lZGdlQi5uZXh0O1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChqLm1fZWRnZUIubmV4dCkge1xyXG4gICAgICBqLm1fZWRnZUIubmV4dC5wcmV2ID0gai5tX2VkZ2VCLnByZXY7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGoubV9lZGdlQiA9PT0gYm9keUIubV9qb2ludExpc3QpIHtcclxuICAgICAgYm9keUIubV9qb2ludExpc3QgPSBqLm1fZWRnZUIubmV4dDtcclxuICAgIH1cclxuXHJcbiAgICBqLm1fZWRnZUIuUmVzZXQoKTtcclxuXHJcbiAgICBiMldvcmxkLl9Kb2ludF9EZXN0cm95KGopO1xyXG5cclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2pvaW50Q291bnQgPiAwKTtcclxuICAgIC0tdGhpcy5tX2pvaW50Q291bnQ7XHJcblxyXG4gICAgLy8gSWYgdGhlIGpvaW50IHByZXZlbnRzIGNvbGxpc2lvbnMsIHRoZW4gZmxhZyBhbnkgY29udGFjdHMgZm9yIGZpbHRlcmluZy5cclxuICAgIGlmICghY29sbGlkZUNvbm5lY3RlZCkge1xyXG4gICAgICBsZXQgZWRnZTogYjJDb250YWN0RWRnZSB8IG51bGwgPSBib2R5Qi5HZXRDb250YWN0TGlzdCgpO1xyXG4gICAgICB3aGlsZSAoZWRnZSkge1xyXG4gICAgICAgIGlmIChlZGdlLm90aGVyID09PSBib2R5QSkge1xyXG4gICAgICAgICAgLy8gRmxhZyB0aGUgY29udGFjdCBmb3IgZmlsdGVyaW5nIGF0IHRoZSBuZXh0IHRpbWUgc3RlcCAod2hlcmUgZWl0aGVyXHJcbiAgICAgICAgICAvLyBib2R5IGlzIGF3YWtlKS5cclxuICAgICAgICAgIGVkZ2UuY29udGFjdC5GbGFnRm9yRmlsdGVyaW5nKCk7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBlZGdlID0gZWRnZS5uZXh0O1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBDcmVhdGVQYXJ0aWNsZVN5c3RlbShkZWY6IGIyUGFydGljbGVTeXN0ZW1EZWYpOiBiMlBhcnRpY2xlU3lzdGVtIHtcclxuICAgIGlmICghQjJfRU5BQkxFX1BBUlRJQ0xFKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG5cclxuICAgIGlmICh0aGlzLklzTG9ja2VkKCkpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgcCA9IG5ldyBiMlBhcnRpY2xlU3lzdGVtKGRlZiwgdGhpcyk7XHJcblxyXG4gICAgLy8gQWRkIHRvIHdvcmxkIGRvdWJseSBsaW5rZWQgbGlzdC5cclxuICAgIHAubV9wcmV2ID0gbnVsbDtcclxuICAgIHAubV9uZXh0ID0gdGhpcy5tX3BhcnRpY2xlU3lzdGVtTGlzdDtcclxuICAgIGlmICh0aGlzLm1fcGFydGljbGVTeXN0ZW1MaXN0KSB7XHJcbiAgICAgIHRoaXMubV9wYXJ0aWNsZVN5c3RlbUxpc3QubV9wcmV2ID0gcDtcclxuICAgIH1cclxuICAgIHRoaXMubV9wYXJ0aWNsZVN5c3RlbUxpc3QgPSBwO1xyXG5cclxuICAgIHJldHVybiBwO1xyXG4gIH1cclxuXHJcbiAgRGVzdHJveVBhcnRpY2xlU3lzdGVtKHA6IGIyUGFydGljbGVTeXN0ZW0pOiB2b2lkIHtcclxuICAgIGlmICghQjJfRU5BQkxFX1BBUlRJQ0xFKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG5cclxuICAgIGlmICh0aGlzLklzTG9ja2VkKCkpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gUmVtb3ZlIHdvcmxkIHBhcnRpY2xlU3lzdGVtIGxpc3QuXHJcbiAgICBpZiAocC5tX3ByZXYpIHtcclxuICAgICAgcC5tX3ByZXYubV9uZXh0ID0gcC5tX25leHQ7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHAubV9uZXh0KSB7XHJcbiAgICAgIHAubV9uZXh0Lm1fcHJldiA9IHAubV9wcmV2O1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChwID09PSB0aGlzLm1fcGFydGljbGVTeXN0ZW1MaXN0KSB7XHJcbiAgICAgIHRoaXMubV9wYXJ0aWNsZVN5c3RlbUxpc3QgPSBwLm1fbmV4dDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIENhbGN1bGF0ZVJlYXNvbmFibGVQYXJ0aWNsZUl0ZXJhdGlvbnModGltZVN0ZXA6IG51bWJlcik6IG51bWJlciB7XHJcbiAgICBpZiAoIUIyX0VOQUJMRV9QQVJUSUNMRSB8fCB0aGlzLm1fcGFydGljbGVTeXN0ZW1MaXN0ID09PSBudWxsKSB7XHJcbiAgICAgIHJldHVybiAxO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIHRvZG86XHJcbiAgICBmdW5jdGlvbiBHZXRTbWFsbGVzdFJhZGl1cyh3b3JsZDogYjJXb3JsZCk6IG51bWJlciB7XHJcbiAgICAgIGxldCBzbWFsbGVzdFJhZGl1cyA9IGIyX21heEZsb2F0O1xyXG4gICAgICBmb3IgKGxldCBzeXN0ZW0gPSB3b3JsZC5HZXRQYXJ0aWNsZVN5c3RlbUxpc3QoKTsgc3lzdGVtICE9PSBudWxsOyBzeXN0ZW0gPSBzeXN0ZW0ubV9uZXh0KSB7XHJcbiAgICAgICAgc21hbGxlc3RSYWRpdXMgPSBiMk1pbihzbWFsbGVzdFJhZGl1cywgc3lzdGVtLkdldFJhZGl1cygpKTtcclxuICAgICAgfVxyXG4gICAgICByZXR1cm4gc21hbGxlc3RSYWRpdXM7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gVXNlIHRoZSBzbWFsbGVzdCByYWRpdXMsIHNpbmNlIHRoYXQgcmVwcmVzZW50cyB0aGUgd29yc3QtY2FzZS5cclxuICAgIHJldHVybiBiMkNhbGN1bGF0ZVBhcnRpY2xlSXRlcmF0aW9ucyhcclxuICAgICAgdGhpcy5tX2dyYXZpdHkuTGVuZ3RoKCksXHJcbiAgICAgIEdldFNtYWxsZXN0UmFkaXVzKHRoaXMpLFxyXG4gICAgICB0aW1lU3RlcCxcclxuICAgICk7XHJcbiAgfVxyXG5cclxuICBTdGVwKFxyXG4gICAgZHQ6IG51bWJlcixcclxuICAgIHZlbG9jaXR5SXRlcmF0aW9uczogbnVtYmVyLFxyXG4gICAgcG9zaXRpb25JdGVyYXRpb25zOiBudW1iZXIsXHJcbiAgICBwYXJ0aWNsZUl0ZXJhdGlvbnM6IG51bWJlciA9IHRoaXMuQ2FsY3VsYXRlUmVhc29uYWJsZVBhcnRpY2xlSXRlcmF0aW9ucyhkdCksXHJcbiAgKTogdm9pZCB7XHJcbiAgICBjb25zdCBzdGVwVGltZXIgPSBTdGVwX3Nfc3RlcFRpbWVyLlJlc2V0KCk7XHJcblxyXG4gICAgLy8gSWYgbmV3IGZpeHR1cmVzIHdlcmUgYWRkZWQsIHdlIG5lZWQgdG8gZmluZCB0aGUgbmV3IGNvbnRhY3RzLlxyXG4gICAgaWYgKHRoaXMubV9uZXdGaXh0dXJlKSB7XHJcbiAgICAgIHRoaXMubV9jb250YWN0TWFuYWdlci5GaW5kTmV3Q29udGFjdHMoKTtcclxuICAgICAgdGhpcy5tX25ld0ZpeHR1cmUgPSBmYWxzZTtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLm1fbG9ja2VkID0gdHJ1ZTtcclxuXHJcbiAgICBjb25zdCBzdGVwID0gU3RlcF9zX3N0ZXA7XHJcbiAgICBzdGVwLmR0ID0gZHQ7XHJcbiAgICBzdGVwLnZlbG9jaXR5SXRlcmF0aW9ucyA9IHZlbG9jaXR5SXRlcmF0aW9ucztcclxuICAgIHN0ZXAucG9zaXRpb25JdGVyYXRpb25zID0gcG9zaXRpb25JdGVyYXRpb25zO1xyXG4gICAgaWYgKEIyX0VOQUJMRV9QQVJUSUNMRSkge1xyXG4gICAgICBzdGVwLnBhcnRpY2xlSXRlcmF0aW9ucyA9IHBhcnRpY2xlSXRlcmF0aW9ucztcclxuICAgIH1cclxuICAgIGlmIChkdCA+IDApIHtcclxuICAgICAgc3RlcC5pbnZfZHQgPSAxIC8gZHQ7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICBzdGVwLmludl9kdCA9IDA7XHJcbiAgICB9XHJcblxyXG4gICAgc3RlcC5kdFJhdGlvID0gdGhpcy5tX2ludl9kdDAgKiBkdDtcclxuXHJcbiAgICBzdGVwLndhcm1TdGFydGluZyA9IHRoaXMubV93YXJtU3RhcnRpbmc7XHJcblxyXG4gICAgLy8gVXBkYXRlIGNvbnRhY3RzLiBUaGlzIGlzIHdoZXJlIHNvbWUgY29udGFjdHMgYXJlIGRlc3Ryb3llZC5cclxuICAgIGNvbnN0IHRpbWVyID0gU3RlcF9zX3RpbWVyLlJlc2V0KCk7XHJcbiAgICB0aGlzLm1fY29udGFjdE1hbmFnZXIuQ29sbGlkZSgpO1xyXG4gICAgdGhpcy5tX3Byb2ZpbGUuY29sbGlkZSA9IHRpbWVyLkdldE1pbGxpc2Vjb25kcygpO1xyXG5cclxuICAgIC8vIEludGVncmF0ZSB2ZWxvY2l0aWVzLCBzb2x2ZSB2ZWxvY2l0eSBjb25zdHJhaW50cywgYW5kIGludGVncmF0ZSBwb3NpdGlvbnMuXHJcbiAgICBpZiAodGhpcy5tX3N0ZXBDb21wbGV0ZSAmJiBzdGVwLmR0ID4gMCkge1xyXG4gICAgICBjb25zdCB0aW1lciA9IFN0ZXBfc190aW1lci5SZXNldCgpO1xyXG4gICAgICBpZiAoQjJfRU5BQkxFX1BBUlRJQ0xFKSB7XHJcbiAgICAgICAgZm9yIChsZXQgcCA9IHRoaXMubV9wYXJ0aWNsZVN5c3RlbUxpc3Q7IHA7IHAgPSBwLm1fbmV4dCkge1xyXG4gICAgICAgICAgcC5Tb2x2ZShzdGVwKTsgLy8gUGFydGljbGUgU2ltdWxhdGlvblxyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgICB0aGlzLlNvbHZlKHN0ZXApO1xyXG4gICAgICB0aGlzLm1fcHJvZmlsZS5zb2x2ZSA9IHRpbWVyLkdldE1pbGxpc2Vjb25kcygpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIEhhbmRsZSBUT0kgZXZlbnRzLlxyXG4gICAgaWYgKHRoaXMubV9jb250aW51b3VzUGh5c2ljcyAmJiBzdGVwLmR0ID4gMCkge1xyXG4gICAgICBjb25zdCB0aW1lciA9IFN0ZXBfc190aW1lci5SZXNldCgpO1xyXG4gICAgICB0aGlzLlNvbHZlVE9JKHN0ZXApO1xyXG4gICAgICB0aGlzLm1fcHJvZmlsZS5zb2x2ZVRPSSA9IHRpbWVyLkdldE1pbGxpc2Vjb25kcygpO1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChzdGVwLmR0ID4gMCkge1xyXG4gICAgICB0aGlzLm1faW52X2R0MCA9IHN0ZXAuaW52X2R0O1xyXG4gICAgfVxyXG5cclxuICAgIGlmICh0aGlzLm1fY2xlYXJGb3JjZXMpIHtcclxuICAgICAgdGhpcy5DbGVhckZvcmNlcygpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9sb2NrZWQgPSBmYWxzZTtcclxuXHJcbiAgICB0aGlzLm1fcHJvZmlsZS5zdGVwID0gc3RlcFRpbWVyLkdldE1pbGxpc2Vjb25kcygpO1xyXG4gIH1cclxuXHJcbiAgLy8vIE1hbnVhbGx5IGNsZWFyIHRoZSBmb3JjZSBidWZmZXIgb24gYWxsIGJvZGllcy4gQnkgZGVmYXVsdCwgZm9yY2VzIGFyZSBjbGVhcmVkIGF1dG9tYXRpY2FsbHlcclxuICAvLy8gYWZ0ZXIgZWFjaCBjYWxsIHRvIFN0ZXAuIFRoZSBkZWZhdWx0IGJlaGF2aW9yIGlzIG1vZGlmaWVkIGJ5IGNhbGxpbmcgU2V0QXV0b0NsZWFyRm9yY2VzLlxyXG4gIC8vLyBUaGUgcHVycG9zZSBvZiB0aGlzIGZ1bmN0aW9uIGlzIHRvIHN1cHBvcnQgc3ViLXN0ZXBwaW5nLiBTdWItc3RlcHBpbmcgaXMgb2Z0ZW4gdXNlZCB0byBtYWludGFpblxyXG4gIC8vLyBhIGZpeGVkIHNpemVkIHRpbWUgc3RlcCB1bmRlciBhIHZhcmlhYmxlIGZyYW1lLXJhdGUuXHJcbiAgLy8vIFdoZW4geW91IHBlcmZvcm0gc3ViLXN0ZXBwaW5nIHlvdSB3aWxsIGRpc2FibGUgYXV0byBjbGVhcmluZyBvZiBmb3JjZXMgYW5kIGluc3RlYWQgY2FsbFxyXG4gIC8vLyBDbGVhckZvcmNlcyBhZnRlciBhbGwgc3ViLXN0ZXBzIGFyZSBjb21wbGV0ZSBpbiBvbmUgcGFzcyBvZiB5b3VyIGdhbWUgbG9vcC5cclxuICAvLy8gQHNlZSBTZXRBdXRvQ2xlYXJGb3JjZXNcclxuICBDbGVhckZvcmNlcygpOiB2b2lkIHtcclxuICAgIGZvciAobGV0IGJvZHkgPSB0aGlzLm1fYm9keUxpc3Q7IGJvZHk7IGJvZHkgPSBib2R5Lm1fbmV4dCkge1xyXG4gICAgICBib2R5Lm1fZm9yY2UuU2V0WmVybygpO1xyXG4gICAgICBib2R5Lm1fdG9ycXVlID0gMDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vLyBRdWVyeSB0aGUgd29ybGQgZm9yIGFsbCBmaXh0dXJlcyB0aGF0IHBvdGVudGlhbGx5IG92ZXJsYXAgdGhlXHJcbiAgLy8vIHByb3ZpZGVkIEFBQkIuXHJcbiAgLy8vIEBwYXJhbSBjYWxsYmFjayBhIHVzZXIgaW1wbGVtZW50ZWQgY2FsbGJhY2sgY2xhc3MuXHJcbiAgLy8vIEBwYXJhbSBhYWJiIHRoZSBxdWVyeSBib3guXHJcbiAgUXVlcnlBQUJCKGNhbGxiYWNrOiBiMlF1ZXJ5Q2FsbGJhY2ssIGFhYmI6IGIyQUFCQik6IHZvaWQ7XHJcbiAgUXVlcnlBQUJCKGFhYmI6IGIyQUFCQiwgZm46IGIyUXVlcnlDYWxsYmFja0Z1bmN0aW9uKTogdm9pZDtcclxuICBRdWVyeUFBQkIoLi4uYXJnczogYW55W10pOiB2b2lkIHtcclxuICAgIGlmIChhcmdzWzBdIGluc3RhbmNlb2YgYjJRdWVyeUNhbGxiYWNrKSB7XHJcbiAgICAgIHRoaXMuX1F1ZXJ5QUFCQihhcmdzWzBdLCBhcmdzWzFdKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMuX1F1ZXJ5QUFCQihudWxsLCBhcmdzWzBdLCBhcmdzWzFdKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHByaXZhdGUgX1F1ZXJ5QUFCQihcclxuICAgIGNhbGxiYWNrOiBiMlF1ZXJ5Q2FsbGJhY2sgfCBudWxsLFxyXG4gICAgYWFiYjogYjJBQUJCLFxyXG4gICAgZm4/OiBiMlF1ZXJ5Q2FsbGJhY2tGdW5jdGlvbixcclxuICApOiB2b2lkIHtcclxuICAgIHRoaXMubV9jb250YWN0TWFuYWdlci5tX2Jyb2FkUGhhc2UuUXVlcnkoYWFiYiwgKHByb3h5OiBiMlRyZWVOb2RlPGIyRml4dHVyZVByb3h5Pik6IGJvb2xlYW4gPT4ge1xyXG4gICAgICBjb25zdCBmaXh0dXJlX3Byb3h5OiBiMkZpeHR1cmVQcm94eSA9IHByb3h5LnVzZXJEYXRhO1xyXG4gICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZpeHR1cmVfcHJveHkgaW5zdGFuY2VvZiBiMkZpeHR1cmVQcm94eSk7XHJcbiAgICAgIGNvbnN0IGZpeHR1cmU6IGIyRml4dHVyZSA9IGZpeHR1cmVfcHJveHkuZml4dHVyZTtcclxuICAgICAgaWYgKGNhbGxiYWNrKSB7XHJcbiAgICAgICAgcmV0dXJuIGNhbGxiYWNrLlJlcG9ydEZpeHR1cmUoZml4dHVyZSk7XHJcbiAgICAgIH0gZWxzZSBpZiAoZm4pIHtcclxuICAgICAgICByZXR1cm4gZm4oZml4dHVyZSk7XHJcbiAgICAgIH1cclxuICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICB9KTtcclxuICAgIGlmIChCMl9FTkFCTEVfUEFSVElDTEUgJiYgY2FsbGJhY2sgaW5zdGFuY2VvZiBiMlF1ZXJ5Q2FsbGJhY2spIHtcclxuICAgICAgZm9yIChsZXQgcCA9IHRoaXMubV9wYXJ0aWNsZVN5c3RlbUxpc3Q7IHA7IHAgPSBwLm1fbmV4dCkge1xyXG4gICAgICAgIGlmIChjYWxsYmFjay5TaG91bGRRdWVyeVBhcnRpY2xlU3lzdGVtKHApKSB7XHJcbiAgICAgICAgICBwLlF1ZXJ5QUFCQihjYWxsYmFjaywgYWFiYik7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBRdWVyeUFsbEFBQkIoYWFiYjogYjJBQUJCLCBvdXQ6IGIyRml4dHVyZVtdID0gW10pOiBiMkZpeHR1cmVbXSB7XHJcbiAgICB0aGlzLlF1ZXJ5QUFCQihhYWJiLCAoZml4dHVyZTogYjJGaXh0dXJlKTogYm9vbGVhbiA9PiB7XHJcbiAgICAgIG91dC5wdXNoKGZpeHR1cmUpO1xyXG4gICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH0pO1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIC8vLyBRdWVyeSB0aGUgd29ybGQgZm9yIGFsbCBmaXh0dXJlcyB0aGF0IHBvdGVudGlhbGx5IG92ZXJsYXAgdGhlXHJcbiAgLy8vIHByb3ZpZGVkIHBvaW50LlxyXG4gIC8vLyBAcGFyYW0gY2FsbGJhY2sgYSB1c2VyIGltcGxlbWVudGVkIGNhbGxiYWNrIGNsYXNzLlxyXG4gIC8vLyBAcGFyYW0gcG9pbnQgdGhlIHF1ZXJ5IHBvaW50LlxyXG4gIFF1ZXJ5UG9pbnRBQUJCKGNhbGxiYWNrOiBiMlF1ZXJ5Q2FsbGJhY2ssIHBvaW50OiBYWSk6IHZvaWQ7XHJcbiAgUXVlcnlQb2ludEFBQkIocG9pbnQ6IFhZLCBmbjogYjJRdWVyeUNhbGxiYWNrRnVuY3Rpb24pOiB2b2lkO1xyXG4gIFF1ZXJ5UG9pbnRBQUJCKC4uLmFyZ3M6IGFueVtdKTogdm9pZCB7XHJcbiAgICBpZiAoYXJnc1swXSBpbnN0YW5jZW9mIGIyUXVlcnlDYWxsYmFjaykge1xyXG4gICAgICB0aGlzLl9RdWVyeVBvaW50QUFCQihhcmdzWzBdLCBhcmdzWzFdKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMuX1F1ZXJ5UG9pbnRBQUJCKG51bGwsIGFyZ3NbMF0sIGFyZ3NbMV0pO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBfUXVlcnlQb2ludEFBQkIoXHJcbiAgICBjYWxsYmFjazogYjJRdWVyeUNhbGxiYWNrIHwgbnVsbCxcclxuICAgIHBvaW50OiBYWSxcclxuICAgIGZuPzogYjJRdWVyeUNhbGxiYWNrRnVuY3Rpb24sXHJcbiAgKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fY29udGFjdE1hbmFnZXIubV9icm9hZFBoYXNlLlF1ZXJ5UG9pbnQoXHJcbiAgICAgIHBvaW50LFxyXG4gICAgICAocHJveHk6IGIyVHJlZU5vZGU8YjJGaXh0dXJlUHJveHk+KTogYm9vbGVhbiA9PiB7XHJcbiAgICAgICAgY29uc3QgZml4dHVyZV9wcm94eTogYjJGaXh0dXJlUHJveHkgPSBwcm94eS51c2VyRGF0YTtcclxuICAgICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZpeHR1cmVfcHJveHkgaW5zdGFuY2VvZiBiMkZpeHR1cmVQcm94eSk7XHJcbiAgICAgICAgY29uc3QgZml4dHVyZTogYjJGaXh0dXJlID0gZml4dHVyZV9wcm94eS5maXh0dXJlO1xyXG4gICAgICAgIGlmIChjYWxsYmFjaykge1xyXG4gICAgICAgICAgcmV0dXJuIGNhbGxiYWNrLlJlcG9ydEZpeHR1cmUoZml4dHVyZSk7XHJcbiAgICAgICAgfSBlbHNlIGlmIChmbikge1xyXG4gICAgICAgICAgcmV0dXJuIGZuKGZpeHR1cmUpO1xyXG4gICAgICAgIH1cclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgICAgfSxcclxuICAgICk7XHJcbiAgICBpZiAoQjJfRU5BQkxFX1BBUlRJQ0xFICYmIGNhbGxiYWNrIGluc3RhbmNlb2YgYjJRdWVyeUNhbGxiYWNrKSB7XHJcbiAgICAgIGZvciAobGV0IHAgPSB0aGlzLm1fcGFydGljbGVTeXN0ZW1MaXN0OyBwOyBwID0gcC5tX25leHQpIHtcclxuICAgICAgICBpZiAoY2FsbGJhY2suU2hvdWxkUXVlcnlQYXJ0aWNsZVN5c3RlbShwKSkge1xyXG4gICAgICAgICAgcC5RdWVyeVBvaW50QUFCQihjYWxsYmFjaywgcG9pbnQpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgUXVlcnlBbGxQb2ludEFBQkIocG9pbnQ6IFhZLCBvdXQ6IGIyRml4dHVyZVtdID0gW10pOiBiMkZpeHR1cmVbXSB7XHJcbiAgICB0aGlzLlF1ZXJ5UG9pbnRBQUJCKHBvaW50LCAoZml4dHVyZTogYjJGaXh0dXJlKTogYm9vbGVhbiA9PiB7XHJcbiAgICAgIG91dC5wdXNoKGZpeHR1cmUpO1xyXG4gICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH0pO1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIFF1ZXJ5Rml4dHVyZVNoYXBlKFxyXG4gICAgY2FsbGJhY2s6IGIyUXVlcnlDYWxsYmFjayxcclxuICAgIHNoYXBlOiBiMlNoYXBlLFxyXG4gICAgaW5kZXg6IG51bWJlcixcclxuICAgIHRyYW5zZm9ybTogYjJUcmFuc2Zvcm0sXHJcbiAgKTogdm9pZDtcclxuICBRdWVyeUZpeHR1cmVTaGFwZShcclxuICAgIHNoYXBlOiBiMlNoYXBlLFxyXG4gICAgaW5kZXg6IG51bWJlcixcclxuICAgIHRyYW5zZm9ybTogYjJUcmFuc2Zvcm0sXHJcbiAgICBmbjogYjJRdWVyeUNhbGxiYWNrRnVuY3Rpb24sXHJcbiAgKTogdm9pZDtcclxuICBRdWVyeUZpeHR1cmVTaGFwZSguLi5hcmdzOiBhbnlbXSk6IHZvaWQge1xyXG4gICAgaWYgKGFyZ3NbMF0gaW5zdGFuY2VvZiBiMlF1ZXJ5Q2FsbGJhY2spIHtcclxuICAgICAgdGhpcy5fUXVlcnlGaXh0dXJlU2hhcGUoYXJnc1swXSwgYXJnc1sxXSwgYXJnc1syXSwgYXJnc1szXSk7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICB0aGlzLl9RdWVyeUZpeHR1cmVTaGFwZShudWxsLCBhcmdzWzBdLCBhcmdzWzFdLCBhcmdzWzJdLCBhcmdzWzNdKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIFF1ZXJ5Rml4dHVyZVNoYXBlX3NfYWFiYiA9IG5ldyBiMkFBQkIoKTtcclxuXHJcbiAgcHJpdmF0ZSBfUXVlcnlGaXh0dXJlU2hhcGUoXHJcbiAgICBjYWxsYmFjazogYjJRdWVyeUNhbGxiYWNrIHwgbnVsbCxcclxuICAgIHNoYXBlOiBiMlNoYXBlLFxyXG4gICAgaW5kZXg6IG51bWJlcixcclxuICAgIHRyYW5zZm9ybTogYjJUcmFuc2Zvcm0sXHJcbiAgICBmbj86IGIyUXVlcnlDYWxsYmFja0Z1bmN0aW9uLFxyXG4gICk6IHZvaWQge1xyXG4gICAgY29uc3QgYWFiYjogYjJBQUJCID0gYjJXb3JsZC5RdWVyeUZpeHR1cmVTaGFwZV9zX2FhYmI7XHJcbiAgICBzaGFwZS5Db21wdXRlQUFCQihhYWJiLCB0cmFuc2Zvcm0sIGluZGV4KTtcclxuICAgIHRoaXMubV9jb250YWN0TWFuYWdlci5tX2Jyb2FkUGhhc2UuUXVlcnkoYWFiYiwgKHByb3h5OiBiMlRyZWVOb2RlPGIyRml4dHVyZVByb3h5Pik6IGJvb2xlYW4gPT4ge1xyXG4gICAgICBjb25zdCBmaXh0dXJlX3Byb3h5OiBiMkZpeHR1cmVQcm94eSA9IHByb3h5LnVzZXJEYXRhO1xyXG4gICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZpeHR1cmVfcHJveHkgaW5zdGFuY2VvZiBiMkZpeHR1cmVQcm94eSk7XHJcbiAgICAgIGNvbnN0IGZpeHR1cmU6IGIyRml4dHVyZSA9IGZpeHR1cmVfcHJveHkuZml4dHVyZTtcclxuICAgICAgaWYgKFxyXG4gICAgICAgIGIyVGVzdE92ZXJsYXBTaGFwZShcclxuICAgICAgICAgIHNoYXBlLFxyXG4gICAgICAgICAgaW5kZXgsXHJcbiAgICAgICAgICBmaXh0dXJlLkdldFNoYXBlKCksXHJcbiAgICAgICAgICBmaXh0dXJlX3Byb3h5LmNoaWxkSW5kZXgsXHJcbiAgICAgICAgICB0cmFuc2Zvcm0sXHJcbiAgICAgICAgICBmaXh0dXJlLkdldEJvZHkoKS5HZXRUcmFuc2Zvcm0oKSxcclxuICAgICAgICApXHJcbiAgICAgICkge1xyXG4gICAgICAgIGlmIChjYWxsYmFjaykge1xyXG4gICAgICAgICAgcmV0dXJuIGNhbGxiYWNrLlJlcG9ydEZpeHR1cmUoZml4dHVyZSk7XHJcbiAgICAgICAgfSBlbHNlIGlmIChmbikge1xyXG4gICAgICAgICAgcmV0dXJuIGZuKGZpeHR1cmUpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH0pO1xyXG4gICAgaWYgKEIyX0VOQUJMRV9QQVJUSUNMRSAmJiBjYWxsYmFjayBpbnN0YW5jZW9mIGIyUXVlcnlDYWxsYmFjaykge1xyXG4gICAgICBmb3IgKGxldCBwID0gdGhpcy5tX3BhcnRpY2xlU3lzdGVtTGlzdDsgcDsgcCA9IHAubV9uZXh0KSB7XHJcbiAgICAgICAgaWYgKGNhbGxiYWNrLlNob3VsZFF1ZXJ5UGFydGljbGVTeXN0ZW0ocCkpIHtcclxuICAgICAgICAgIHAuUXVlcnlBQUJCKGNhbGxiYWNrLCBhYWJiKTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIFF1ZXJ5QWxsRml4dHVyZVNoYXBlKFxyXG4gICAgc2hhcGU6IGIyU2hhcGUsXHJcbiAgICBpbmRleDogbnVtYmVyLFxyXG4gICAgdHJhbnNmb3JtOiBiMlRyYW5zZm9ybSxcclxuICAgIG91dDogYjJGaXh0dXJlW10gPSBbXSxcclxuICApOiBiMkZpeHR1cmVbXSB7XHJcbiAgICB0aGlzLlF1ZXJ5Rml4dHVyZVNoYXBlKHNoYXBlLCBpbmRleCwgdHJhbnNmb3JtLCAoZml4dHVyZTogYjJGaXh0dXJlKTogYm9vbGVhbiA9PiB7XHJcbiAgICAgIG91dC5wdXNoKGZpeHR1cmUpO1xyXG4gICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH0pO1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIFF1ZXJ5Rml4dHVyZVBvaW50KGNhbGxiYWNrOiBiMlF1ZXJ5Q2FsbGJhY2ssIHBvaW50OiBYWSk6IHZvaWQ7XHJcbiAgUXVlcnlGaXh0dXJlUG9pbnQocG9pbnQ6IFhZLCBmbjogYjJRdWVyeUNhbGxiYWNrRnVuY3Rpb24pOiB2b2lkO1xyXG4gIFF1ZXJ5Rml4dHVyZVBvaW50KC4uLmFyZ3M6IGFueVtdKTogdm9pZCB7XHJcbiAgICBpZiAoYXJnc1swXSBpbnN0YW5jZW9mIGIyUXVlcnlDYWxsYmFjaykge1xyXG4gICAgICB0aGlzLl9RdWVyeUZpeHR1cmVQb2ludChhcmdzWzBdLCBhcmdzWzFdKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMuX1F1ZXJ5Rml4dHVyZVBvaW50KG51bGwsIGFyZ3NbMF0sIGFyZ3NbMV0pO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBfUXVlcnlGaXh0dXJlUG9pbnQoXHJcbiAgICBjYWxsYmFjazogYjJRdWVyeUNhbGxiYWNrIHwgbnVsbCxcclxuICAgIHBvaW50OiBYWSxcclxuICAgIGZuPzogYjJRdWVyeUNhbGxiYWNrRnVuY3Rpb24sXHJcbiAgKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fY29udGFjdE1hbmFnZXIubV9icm9hZFBoYXNlLlF1ZXJ5UG9pbnQoXHJcbiAgICAgIHBvaW50LFxyXG4gICAgICAocHJveHk6IGIyVHJlZU5vZGU8YjJGaXh0dXJlUHJveHk+KTogYm9vbGVhbiA9PiB7XHJcbiAgICAgICAgY29uc3QgZml4dHVyZV9wcm94eTogYjJGaXh0dXJlUHJveHkgPSBwcm94eS51c2VyRGF0YTtcclxuICAgICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZpeHR1cmVfcHJveHkgaW5zdGFuY2VvZiBiMkZpeHR1cmVQcm94eSk7XHJcbiAgICAgICAgY29uc3QgZml4dHVyZTogYjJGaXh0dXJlID0gZml4dHVyZV9wcm94eS5maXh0dXJlO1xyXG4gICAgICAgIGlmIChmaXh0dXJlLlRlc3RQb2ludChwb2ludCkpIHtcclxuICAgICAgICAgIGlmIChjYWxsYmFjaykge1xyXG4gICAgICAgICAgICByZXR1cm4gY2FsbGJhY2suUmVwb3J0Rml4dHVyZShmaXh0dXJlKTtcclxuICAgICAgICAgIH0gZWxzZSBpZiAoZm4pIHtcclxuICAgICAgICAgICAgcmV0dXJuIGZuKGZpeHR1cmUpO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgICAgfSxcclxuICAgICk7XHJcbiAgICBpZiAoQjJfRU5BQkxFX1BBUlRJQ0xFICYmIGNhbGxiYWNrKSB7XHJcbiAgICAgIGZvciAobGV0IHAgPSB0aGlzLm1fcGFydGljbGVTeXN0ZW1MaXN0OyBwOyBwID0gcC5tX25leHQpIHtcclxuICAgICAgICBpZiAoY2FsbGJhY2suU2hvdWxkUXVlcnlQYXJ0aWNsZVN5c3RlbShwKSkge1xyXG4gICAgICAgICAgcC5RdWVyeVBvaW50QUFCQihjYWxsYmFjaywgcG9pbnQpO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgUXVlcnlBbGxGaXh0dXJlUG9pbnQocG9pbnQ6IFhZLCBvdXQ6IGIyRml4dHVyZVtdID0gW10pOiBiMkZpeHR1cmVbXSB7XHJcbiAgICB0aGlzLlF1ZXJ5Rml4dHVyZVBvaW50KHBvaW50LCAoZml4dHVyZTogYjJGaXh0dXJlKTogYm9vbGVhbiA9PiB7XHJcbiAgICAgIG91dC5wdXNoKGZpeHR1cmUpO1xyXG4gICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH0pO1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIC8vLyBSYXktY2FzdCB0aGUgd29ybGQgZm9yIGFsbCBmaXh0dXJlcyBpbiB0aGUgcGF0aCBvZiB0aGUgcmF5LiBZb3VyIGNhbGxiYWNrXHJcbiAgLy8vIGNvbnRyb2xzIHdoZXRoZXIgeW91IGdldCB0aGUgY2xvc2VzdCBwb2ludCwgYW55IHBvaW50LCBvciBuLXBvaW50cy5cclxuICAvLy8gVGhlIHJheS1jYXN0IGlnbm9yZXMgc2hhcGVzIHRoYXQgY29udGFpbiB0aGUgc3RhcnRpbmcgcG9pbnQuXHJcbiAgLy8vIEBwYXJhbSBjYWxsYmFjayBhIHVzZXIgaW1wbGVtZW50ZWQgY2FsbGJhY2sgY2xhc3MuXHJcbiAgLy8vIEBwYXJhbSBwb2ludDEgdGhlIHJheSBzdGFydGluZyBwb2ludFxyXG4gIC8vLyBAcGFyYW0gcG9pbnQyIHRoZSByYXkgZW5kaW5nIHBvaW50XHJcbiAgUmF5Q2FzdChjYWxsYmFjazogYjJSYXlDYXN0Q2FsbGJhY2ssIHBvaW50MTogWFksIHBvaW50MjogWFkpOiB2b2lkO1xyXG4gIFJheUNhc3QocG9pbnQxOiBYWSwgcG9pbnQyOiBYWSwgZm46IGIyUmF5Q2FzdENhbGxiYWNrRnVuY3Rpb24pOiB2b2lkO1xyXG4gIFJheUNhc3QoLi4uYXJnczogYW55W10pOiB2b2lkIHtcclxuICAgIGlmIChhcmdzWzBdIGluc3RhbmNlb2YgYjJSYXlDYXN0Q2FsbGJhY2spIHtcclxuICAgICAgdGhpcy5fUmF5Q2FzdChhcmdzWzBdLCBhcmdzWzFdLCBhcmdzWzJdKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMuX1JheUNhc3QobnVsbCwgYXJnc1swXSwgYXJnc1sxXSwgYXJnc1syXSk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBSYXlDYXN0X3NfaW5wdXQgPSBuZXcgYjJSYXlDYXN0SW5wdXQoKTtcclxuICBwcml2YXRlIHN0YXRpYyBSYXlDYXN0X3Nfb3V0cHV0ID0gbmV3IGIyUmF5Q2FzdE91dHB1dCgpO1xyXG4gIHByaXZhdGUgc3RhdGljIFJheUNhc3Rfc19wb2ludCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgcHJpdmF0ZSBfUmF5Q2FzdChcclxuICAgIGNhbGxiYWNrOiBiMlJheUNhc3RDYWxsYmFjayB8IG51bGwsXHJcbiAgICBwb2ludDE6IFhZLFxyXG4gICAgcG9pbnQyOiBYWSxcclxuICAgIGZuPzogYjJSYXlDYXN0Q2FsbGJhY2tGdW5jdGlvbixcclxuICApOiB2b2lkIHtcclxuICAgIGNvbnN0IGlucHV0OiBiMlJheUNhc3RJbnB1dCA9IGIyV29ybGQuUmF5Q2FzdF9zX2lucHV0O1xyXG4gICAgaW5wdXQubWF4RnJhY3Rpb24gPSAxO1xyXG4gICAgaW5wdXQucDEuQ29weShwb2ludDEpO1xyXG4gICAgaW5wdXQucDIuQ29weShwb2ludDIpO1xyXG4gICAgdGhpcy5tX2NvbnRhY3RNYW5hZ2VyLm1fYnJvYWRQaGFzZS5SYXlDYXN0KFxyXG4gICAgICBpbnB1dCxcclxuICAgICAgKGlucHV0OiBiMlJheUNhc3RJbnB1dCwgcHJveHk6IGIyVHJlZU5vZGU8YjJGaXh0dXJlUHJveHk+KTogbnVtYmVyID0+IHtcclxuICAgICAgICBjb25zdCBmaXh0dXJlX3Byb3h5OiBiMkZpeHR1cmVQcm94eSA9IHByb3h5LnVzZXJEYXRhO1xyXG4gICAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZml4dHVyZV9wcm94eSBpbnN0YW5jZW9mIGIyRml4dHVyZVByb3h5KTtcclxuICAgICAgICBjb25zdCBmaXh0dXJlOiBiMkZpeHR1cmUgPSBmaXh0dXJlX3Byb3h5LmZpeHR1cmU7XHJcbiAgICAgICAgY29uc3QgaW5kZXg6IG51bWJlciA9IGZpeHR1cmVfcHJveHkuY2hpbGRJbmRleDtcclxuICAgICAgICBjb25zdCBvdXRwdXQ6IGIyUmF5Q2FzdE91dHB1dCA9IGIyV29ybGQuUmF5Q2FzdF9zX291dHB1dDtcclxuICAgICAgICBjb25zdCBoaXQ6IGJvb2xlYW4gPSBmaXh0dXJlLlJheUNhc3Qob3V0cHV0LCBpbnB1dCwgaW5kZXgpO1xyXG4gICAgICAgIGlmIChoaXQpIHtcclxuICAgICAgICAgIGNvbnN0IGZyYWN0aW9uOiBudW1iZXIgPSBvdXRwdXQuZnJhY3Rpb247XHJcbiAgICAgICAgICBjb25zdCBwb2ludDogYjJWZWMyID0gYjJXb3JsZC5SYXlDYXN0X3NfcG9pbnQ7XHJcbiAgICAgICAgICBwb2ludC5TZXQoXHJcbiAgICAgICAgICAgICgxIC0gZnJhY3Rpb24pICogcG9pbnQxLnggKyBmcmFjdGlvbiAqIHBvaW50Mi54LFxyXG4gICAgICAgICAgICAoMSAtIGZyYWN0aW9uKSAqIHBvaW50MS55ICsgZnJhY3Rpb24gKiBwb2ludDIueSxcclxuICAgICAgICAgICk7XHJcbiAgICAgICAgICBpZiAoY2FsbGJhY2spIHtcclxuICAgICAgICAgICAgcmV0dXJuIGNhbGxiYWNrLlJlcG9ydEZpeHR1cmUoZml4dHVyZSwgcG9pbnQsIG91dHB1dC5ub3JtYWwsIGZyYWN0aW9uKTtcclxuICAgICAgICAgIH0gZWxzZSBpZiAoZm4pIHtcclxuICAgICAgICAgICAgcmV0dXJuIGZuKGZpeHR1cmUsIHBvaW50LCBvdXRwdXQubm9ybWFsLCBmcmFjdGlvbik7XHJcbiAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHJldHVybiBpbnB1dC5tYXhGcmFjdGlvbjtcclxuICAgICAgfSxcclxuICAgICk7XHJcbiAgICBpZiAoQjJfRU5BQkxFX1BBUlRJQ0xFICYmIGNhbGxiYWNrKSB7XHJcbiAgICAgIGZvciAobGV0IHAgPSB0aGlzLm1fcGFydGljbGVTeXN0ZW1MaXN0OyBwOyBwID0gcC5tX25leHQpIHtcclxuICAgICAgICBpZiAoY2FsbGJhY2suU2hvdWxkUXVlcnlQYXJ0aWNsZVN5c3RlbShwKSkge1xyXG4gICAgICAgICAgcC5SYXlDYXN0KGNhbGxiYWNrLCBwb2ludDEsIHBvaW50Mik7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBSYXlDYXN0T25lKHBvaW50MTogWFksIHBvaW50MjogWFkpOiBiMkZpeHR1cmUgfCBudWxsIHtcclxuICAgIGxldCByZXN1bHQ6IGIyRml4dHVyZSB8IG51bGwgPSBudWxsO1xyXG4gICAgbGV0IG1pbl9mcmFjdGlvbiA9IDE7XHJcbiAgICB0aGlzLlJheUNhc3QoXHJcbiAgICAgIHBvaW50MSxcclxuICAgICAgcG9pbnQyLFxyXG4gICAgICAoZml4dHVyZTogYjJGaXh0dXJlLCBwb2ludDogYjJWZWMyLCBub3JtYWw6IGIyVmVjMiwgZnJhY3Rpb246IG51bWJlcik6IG51bWJlciA9PiB7XHJcbiAgICAgICAgaWYgKGZyYWN0aW9uIDwgbWluX2ZyYWN0aW9uKSB7XHJcbiAgICAgICAgICBtaW5fZnJhY3Rpb24gPSBmcmFjdGlvbjtcclxuICAgICAgICAgIHJlc3VsdCA9IGZpeHR1cmU7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHJldHVybiBtaW5fZnJhY3Rpb247XHJcbiAgICAgIH0sXHJcbiAgICApO1xyXG4gICAgcmV0dXJuIHJlc3VsdDtcclxuICB9XHJcblxyXG4gIFJheUNhc3RBbGwocG9pbnQxOiBYWSwgcG9pbnQyOiBYWSwgb3V0OiBiMkZpeHR1cmVbXSA9IFtdKTogYjJGaXh0dXJlW10ge1xyXG4gICAgdGhpcy5SYXlDYXN0KFxyXG4gICAgICBwb2ludDEsXHJcbiAgICAgIHBvaW50MixcclxuICAgICAgKGZpeHR1cmU6IGIyRml4dHVyZSwgcG9pbnQ6IGIyVmVjMiwgbm9ybWFsOiBiMlZlYzIsIGZyYWN0aW9uOiBudW1iZXIpOiBudW1iZXIgPT4ge1xyXG4gICAgICAgIG91dC5wdXNoKGZpeHR1cmUpO1xyXG4gICAgICAgIHJldHVybiAxO1xyXG4gICAgICB9LFxyXG4gICAgKTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSB3b3JsZCBib2R5IGxpc3QuIFdpdGggdGhlIHJldHVybmVkIGJvZHksIHVzZSBiMkJvZHk6OkdldE5leHQgdG8gZ2V0XHJcbiAgLy8vIHRoZSBuZXh0IGJvZHkgaW4gdGhlIHdvcmxkIGxpc3QuIEEgTlVMTCBib2R5IGluZGljYXRlcyB0aGUgZW5kIG9mIHRoZSBsaXN0LlxyXG4gIC8vLyBAcmV0dXJuIHRoZSBoZWFkIG9mIHRoZSB3b3JsZCBib2R5IGxpc3QuXHJcbiAgR2V0Qm9keUxpc3QoKTogYjJCb2R5IHwgbnVsbCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2JvZHlMaXN0O1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgd29ybGQgam9pbnQgbGlzdC4gV2l0aCB0aGUgcmV0dXJuZWQgam9pbnQsIHVzZSBiMkpvaW50OjpHZXROZXh0IHRvIGdldFxyXG4gIC8vLyB0aGUgbmV4dCBqb2ludCBpbiB0aGUgd29ybGQgbGlzdC4gQSBOVUxMIGpvaW50IGluZGljYXRlcyB0aGUgZW5kIG9mIHRoZSBsaXN0LlxyXG4gIC8vLyBAcmV0dXJuIHRoZSBoZWFkIG9mIHRoZSB3b3JsZCBqb2ludCBsaXN0LlxyXG4gIEdldEpvaW50TGlzdCgpOiBiMkpvaW50IHwgbnVsbCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2pvaW50TGlzdDtcclxuICB9XHJcblxyXG4gIEdldFBhcnRpY2xlU3lzdGVtTGlzdCgpOiBiMlBhcnRpY2xlU3lzdGVtIHwgbnVsbCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3BhcnRpY2xlU3lzdGVtTGlzdDtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIHdvcmxkIGNvbnRhY3QgbGlzdC4gV2l0aCB0aGUgcmV0dXJuZWQgY29udGFjdCwgdXNlIGIyQ29udGFjdDo6R2V0TmV4dCB0byBnZXRcclxuICAvLy8gdGhlIG5leHQgY29udGFjdCBpbiB0aGUgd29ybGQgbGlzdC4gQSBOVUxMIGNvbnRhY3QgaW5kaWNhdGVzIHRoZSBlbmQgb2YgdGhlIGxpc3QuXHJcbiAgLy8vIEByZXR1cm4gdGhlIGhlYWQgb2YgdGhlIHdvcmxkIGNvbnRhY3QgbGlzdC5cclxuICAvLy8gQHdhcm5pbmcgY29udGFjdHMgYXJlIGNyZWF0ZWQgYW5kIGRlc3Ryb3llZCBpbiB0aGUgbWlkZGxlIG9mIGEgdGltZSBzdGVwLlxyXG4gIC8vLyBVc2UgYjJDb250YWN0TGlzdGVuZXIgdG8gYXZvaWQgbWlzc2luZyBjb250YWN0cy5cclxuICBHZXRDb250YWN0TGlzdCgpOiBiMkNvbnRhY3QgfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fY29udGFjdE1hbmFnZXIubV9jb250YWN0TGlzdDtcclxuICB9XHJcblxyXG4gIC8vLyBFbmFibGUvZGlzYWJsZSBzbGVlcC5cclxuICBTZXRBbGxvd1NsZWVwaW5nKGZsYWc6IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIGlmIChmbGFnID09PSB0aGlzLm1fYWxsb3dTbGVlcCkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5tX2FsbG93U2xlZXAgPSBmbGFnO1xyXG4gICAgaWYgKCF0aGlzLm1fYWxsb3dTbGVlcCkge1xyXG4gICAgICBmb3IgKGxldCBiID0gdGhpcy5tX2JvZHlMaXN0OyBiOyBiID0gYi5tX25leHQpIHtcclxuICAgICAgICBiLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBHZXRBbGxvd1NsZWVwaW5nKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9hbGxvd1NsZWVwO1xyXG4gIH1cclxuXHJcbiAgLy8vIEVuYWJsZS9kaXNhYmxlIHdhcm0gc3RhcnRpbmcuIEZvciB0ZXN0aW5nLlxyXG4gIFNldFdhcm1TdGFydGluZyhmbGFnOiBib29sZWFuKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fd2FybVN0YXJ0aW5nID0gZmxhZztcclxuICB9XHJcblxyXG4gIEdldFdhcm1TdGFydGluZygpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0aGlzLm1fd2FybVN0YXJ0aW5nO1xyXG4gIH1cclxuXHJcbiAgLy8vIEVuYWJsZS9kaXNhYmxlIGNvbnRpbnVvdXMgcGh5c2ljcy4gRm9yIHRlc3RpbmcuXHJcbiAgU2V0Q29udGludW91c1BoeXNpY3MoZmxhZzogYm9vbGVhbik6IHZvaWQge1xyXG4gICAgdGhpcy5tX2NvbnRpbnVvdXNQaHlzaWNzID0gZmxhZztcclxuICB9XHJcblxyXG4gIEdldENvbnRpbnVvdXNQaHlzaWNzKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9jb250aW51b3VzUGh5c2ljcztcclxuICB9XHJcblxyXG4gIC8vLyBFbmFibGUvZGlzYWJsZSBzaW5nbGUgc3RlcHBlZCBjb250aW51b3VzIHBoeXNpY3MuIEZvciB0ZXN0aW5nLlxyXG4gIFNldFN1YlN0ZXBwaW5nKGZsYWc6IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIHRoaXMubV9zdWJTdGVwcGluZyA9IGZsYWc7XHJcbiAgfVxyXG5cclxuICBHZXRTdWJTdGVwcGluZygpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0aGlzLm1fc3ViU3RlcHBpbmc7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBudW1iZXIgb2YgYnJvYWQtcGhhc2UgcHJveGllcy5cclxuICBHZXRQcm94eUNvdW50KCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2NvbnRhY3RNYW5hZ2VyLm1fYnJvYWRQaGFzZS5HZXRQcm94eUNvdW50KCk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBudW1iZXIgb2YgYm9kaWVzLlxyXG4gIEdldEJvZHlDb3VudCgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ib2R5Q291bnQ7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBudW1iZXIgb2Ygam9pbnRzLlxyXG4gIEdldEpvaW50Q291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fam9pbnRDb3VudDtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIG51bWJlciBvZiBjb250YWN0cyAoZWFjaCBtYXkgaGF2ZSAwIG9yIG1vcmUgY29udGFjdCBwb2ludHMpLlxyXG4gIEdldENvbnRhY3RDb3VudCgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9jb250YWN0TWFuYWdlci5tX2NvbnRhY3RDb3VudDtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGhlaWdodCBvZiB0aGUgZHluYW1pYyB0cmVlLlxyXG4gIEdldFRyZWVIZWlnaHQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fY29udGFjdE1hbmFnZXIubV9icm9hZFBoYXNlLkdldFRyZWVIZWlnaHQoKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGJhbGFuY2Ugb2YgdGhlIGR5bmFtaWMgdHJlZS5cclxuICBHZXRUcmVlQmFsYW5jZSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9jb250YWN0TWFuYWdlci5tX2Jyb2FkUGhhc2UuR2V0VHJlZUJhbGFuY2UoKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIHF1YWxpdHkgbWV0cmljIG9mIHRoZSBkeW5hbWljIHRyZWUuIFRoZSBzbWFsbGVyIHRoZSBiZXR0ZXIuXHJcbiAgLy8vIFRoZSBtaW5pbXVtIGlzIDEuXHJcbiAgR2V0VHJlZVF1YWxpdHkoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fY29udGFjdE1hbmFnZXIubV9icm9hZFBoYXNlLkdldFRyZWVRdWFsaXR5KCk7XHJcbiAgfVxyXG5cclxuICAvLy8gQ2hhbmdlIHRoZSBnbG9iYWwgZ3Jhdml0eSB2ZWN0b3IuXHJcbiAgU2V0R3Jhdml0eShncmF2aXR5OiBYWSwgd2FrZSA9IHRydWUpIHtcclxuICAgIGlmICghYjJWZWMyLklzRXF1YWxUb1YodGhpcy5tX2dyYXZpdHksIGdyYXZpdHkpKSB7XHJcbiAgICAgIHRoaXMubV9ncmF2aXR5LkNvcHkoZ3Jhdml0eSk7XHJcblxyXG4gICAgICBpZiAod2FrZSkge1xyXG4gICAgICAgIGZvciAobGV0IGI6IGIyQm9keSB8IG51bGwgPSB0aGlzLm1fYm9keUxpc3Q7IGI7IGIgPSBiLm1fbmV4dCkge1xyXG4gICAgICAgICAgYi5TZXRBd2FrZSh0cnVlKTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGdsb2JhbCBncmF2aXR5IHZlY3Rvci5cclxuICBHZXRHcmF2aXR5KCk6IFJlYWRvbmx5PGIyVmVjMj4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ncmF2aXR5O1xyXG4gIH1cclxuXHJcbiAgLy8vIElzIHRoZSB3b3JsZCBsb2NrZWQgKGluIHRoZSBtaWRkbGUgb2YgYSB0aW1lIHN0ZXApLlxyXG4gIElzTG9ja2VkKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9sb2NrZWQ7XHJcbiAgfVxyXG5cclxuICAvLy8gU2V0IGZsYWcgdG8gY29udHJvbCBhdXRvbWF0aWMgY2xlYXJpbmcgb2YgZm9yY2VzIGFmdGVyIGVhY2ggdGltZSBzdGVwLlxyXG4gIFNldEF1dG9DbGVhckZvcmNlcyhmbGFnOiBib29sZWFuKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fY2xlYXJGb3JjZXMgPSBmbGFnO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgZmxhZyB0aGF0IGNvbnRyb2xzIGF1dG9tYXRpYyBjbGVhcmluZyBvZiBmb3JjZXMgYWZ0ZXIgZWFjaCB0aW1lIHN0ZXAuXHJcbiAgR2V0QXV0b0NsZWFyRm9yY2VzKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9jbGVhckZvcmNlcztcclxuICB9XHJcblxyXG4gIC8vLyBTaGlmdCB0aGUgd29ybGQgb3JpZ2luLiBVc2VmdWwgZm9yIGxhcmdlIHdvcmxkcy5cclxuICAvLy8gVGhlIGJvZHkgc2hpZnQgZm9ybXVsYSBpczogcG9zaXRpb24gLT0gbmV3T3JpZ2luXHJcbiAgLy8vIEBwYXJhbSBuZXdPcmlnaW4gdGhlIG5ldyBvcmlnaW4gd2l0aCByZXNwZWN0IHRvIHRoZSBvbGQgb3JpZ2luXHJcbiAgU2hpZnRPcmlnaW4obmV3T3JpZ2luOiBYWSk6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMuSXNMb2NrZWQoKSkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuXHJcbiAgICBmb3IgKGxldCBiOiBiMkJvZHkgfCBudWxsID0gdGhpcy5tX2JvZHlMaXN0OyBiOyBiID0gYi5tX25leHQpIHtcclxuICAgICAgYi5tX3hmLnAuU2VsZlN1YihuZXdPcmlnaW4pO1xyXG4gICAgICBiLm1fc3dlZXAuYzAuU2VsZlN1YihuZXdPcmlnaW4pO1xyXG4gICAgICBiLm1fc3dlZXAuYy5TZWxmU3ViKG5ld09yaWdpbik7XHJcbiAgICB9XHJcblxyXG4gICAgZm9yIChsZXQgajogYjJKb2ludCB8IG51bGwgPSB0aGlzLm1fam9pbnRMaXN0OyBqOyBqID0gai5tX25leHQpIHtcclxuICAgICAgai5TaGlmdE9yaWdpbihuZXdPcmlnaW4pO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9jb250YWN0TWFuYWdlci5tX2Jyb2FkUGhhc2UuU2hpZnRPcmlnaW4obmV3T3JpZ2luKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGNvbnRhY3QgbWFuYWdlciBmb3IgdGVzdGluZy5cclxuICBHZXRDb250YWN0TWFuYWdlcigpOiBiMkNvbnRhY3RNYW5hZ2VyIHtcclxuICAgIHJldHVybiB0aGlzLm1fY29udGFjdE1hbmFnZXI7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBjdXJyZW50IHByb2ZpbGUuXHJcbiAgR2V0UHJvZmlsZSgpOiBiMlByb2ZpbGUge1xyXG4gICAgcmV0dXJuIHRoaXMubV9wcm9maWxlO1xyXG4gIH1cclxuXHJcbiAgU29sdmUoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgaWYgKEIyX0VOQUJMRV9QQVJUSUNMRSkge1xyXG4gICAgICAvLyB1cGRhdGUgcHJldmlvdXMgdHJhbnNmb3Jtc1xyXG4gICAgICBmb3IgKGxldCBiID0gdGhpcy5tX2JvZHlMaXN0OyBiOyBiID0gYi5tX25leHQpIHtcclxuICAgICAgICBiLm1feGYwLkNvcHkoYi5tX3hmKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIGlmIChCMl9FTkFCTEVfQ09OVFJPTExFUikge1xyXG4gICAgICAvLyBAc2VlIGIyQ29udHJvbGxlciBsaXN0XHJcbiAgICAgIGZvciAobGV0IGNvbnRyb2xsZXIgPSB0aGlzLm1fY29udHJvbGxlckxpc3Q7IGNvbnRyb2xsZXI7IGNvbnRyb2xsZXIgPSBjb250cm9sbGVyLm1fbmV4dCkge1xyXG4gICAgICAgIGNvbnRyb2xsZXIuU3RlcChzdGVwKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9wcm9maWxlLnNvbHZlSW5pdCA9IDA7XHJcbiAgICB0aGlzLm1fcHJvZmlsZS5zb2x2ZVZlbG9jaXR5ID0gMDtcclxuICAgIHRoaXMubV9wcm9maWxlLnNvbHZlUG9zaXRpb24gPSAwO1xyXG5cclxuICAgIC8vIFNpemUgdGhlIGlzbGFuZCBmb3IgdGhlIHdvcnN0IGNhc2UuXHJcbiAgICBjb25zdCBpc2xhbmQgPSB0aGlzLm1faXNsYW5kO1xyXG4gICAgaXNsYW5kLkluaXRpYWxpemUoXHJcbiAgICAgIHRoaXMubV9ib2R5Q291bnQsXHJcbiAgICAgIHRoaXMubV9jb250YWN0TWFuYWdlci5tX2NvbnRhY3RDb3VudCxcclxuICAgICAgdGhpcy5tX2pvaW50Q291bnQsXHJcbiAgICAgIHRoaXMubV9jb250YWN0TWFuYWdlci5tX2NvbnRhY3RMaXN0ZW5lcixcclxuICAgICk7XHJcblxyXG4gICAgLy8gQ2xlYXIgYWxsIHRoZSBpc2xhbmQgZmxhZ3MuXHJcbiAgICBmb3IgKGxldCBiID0gdGhpcy5tX2JvZHlMaXN0OyBiOyBiID0gYi5tX25leHQpIHtcclxuICAgICAgYi5tX2lzbGFuZEZsYWcgPSBmYWxzZTtcclxuICAgIH1cclxuICAgIGZvciAobGV0IGMgPSB0aGlzLm1fY29udGFjdE1hbmFnZXIubV9jb250YWN0TGlzdDsgYzsgYyA9IGMubV9uZXh0KSB7XHJcbiAgICAgIGMubV9pc2xhbmRGbGFnID0gZmFsc2U7XHJcbiAgICB9XHJcbiAgICBmb3IgKGxldCBqID0gdGhpcy5tX2pvaW50TGlzdDsgajsgaiA9IGoubV9uZXh0KSB7XHJcbiAgICAgIGoubV9pc2xhbmRGbGFnID0gZmFsc2U7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gQnVpbGQgYW5kIHNpbXVsYXRlIGFsbCBhd2FrZSBpc2xhbmRzLlxyXG4gICAgY29uc3Qgc3RhY2tTaXplID0gdGhpcy5tX2JvZHlDb3VudDsgLy8gREVCVUdcclxuICAgIGNvbnN0IHN0YWNrID0gdGhpcy5zX3N0YWNrO1xyXG4gICAgZm9yIChsZXQgc2VlZCA9IHRoaXMubV9ib2R5TGlzdDsgc2VlZDsgc2VlZCA9IHNlZWQubV9uZXh0KSB7XHJcbiAgICAgIGlmIChzZWVkLm1faXNsYW5kRmxhZykge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBpZiAoIXNlZWQuSXNBd2FrZSgpIHx8ICFzZWVkLklzQWN0aXZlKCkpIHtcclxuICAgICAgICBjb250aW51ZTtcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gVGhlIHNlZWQgY2FuIGJlIGR5bmFtaWMgb3Iga2luZW1hdGljLlxyXG4gICAgICBpZiAoc2VlZC5HZXRUeXBlKCkgPT09IGIyQm9keVR5cGUuYjJfc3RhdGljQm9keSkge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBSZXNldCBpc2xhbmQgYW5kIHN0YWNrLlxyXG4gICAgICBpc2xhbmQuQ2xlYXIoKTtcclxuICAgICAgbGV0IHN0YWNrQ291bnQgPSAwO1xyXG4gICAgICBzdGFja1tzdGFja0NvdW50KytdID0gc2VlZDtcclxuICAgICAgc2VlZC5tX2lzbGFuZEZsYWcgPSB0cnVlO1xyXG5cclxuICAgICAgLy8gUGVyZm9ybSBhIGRlcHRoIGZpcnN0IHNlYXJjaCAoREZTKSBvbiB0aGUgY29uc3RyYWludCBncmFwaC5cclxuICAgICAgd2hpbGUgKHN0YWNrQ291bnQgPiAwKSB7XHJcbiAgICAgICAgLy8gR3JhYiB0aGUgbmV4dCBib2R5IG9mZiB0aGUgc3RhY2sgYW5kIGFkZCBpdCB0byB0aGUgaXNsYW5kLlxyXG4gICAgICAgIGNvbnN0IGI6IGIyQm9keSB8IG51bGwgPSBzdGFja1stLXN0YWNrQ291bnRdO1xyXG4gICAgICAgIGlmICghYikge1xyXG4gICAgICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoYi5Jc0FjdGl2ZSgpKTtcclxuICAgICAgICBpc2xhbmQuQWRkQm9keShiKTtcclxuXHJcbiAgICAgICAgLy8gTWFrZSBzdXJlIHRoZSBib2R5IGlzIGF3YWtlLiAod2l0aG91dCByZXNldHRpbmcgc2xlZXAgdGltZXIpLlxyXG4gICAgICAgIGIubV9hd2FrZUZsYWcgPSB0cnVlO1xyXG5cclxuICAgICAgICAvLyBUbyBrZWVwIGlzbGFuZHMgYXMgc21hbGwgYXMgcG9zc2libGUsIHdlIGRvbid0XHJcbiAgICAgICAgLy8gcHJvcGFnYXRlIGlzbGFuZHMgYWNyb3NzIHN0YXRpYyBib2RpZXMuXHJcbiAgICAgICAgaWYgKGIuR2V0VHlwZSgpID09PSBiMkJvZHlUeXBlLmIyX3N0YXRpY0JvZHkpIHtcclxuICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgLy8gU2VhcmNoIGFsbCBjb250YWN0cyBjb25uZWN0ZWQgdG8gdGhpcyBib2R5LlxyXG4gICAgICAgIGZvciAobGV0IGNlID0gYi5tX2NvbnRhY3RMaXN0OyBjZTsgY2UgPSBjZS5uZXh0KSB7XHJcbiAgICAgICAgICBjb25zdCBjb250YWN0ID0gY2UuY29udGFjdDtcclxuXHJcbiAgICAgICAgICAvLyBIYXMgdGhpcyBjb250YWN0IGFscmVhZHkgYmVlbiBhZGRlZCB0byBhbiBpc2xhbmQ/XHJcbiAgICAgICAgICBpZiAoY29udGFjdC5tX2lzbGFuZEZsYWcpIHtcclxuICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgLy8gSXMgdGhpcyBjb250YWN0IHNvbGlkIGFuZCB0b3VjaGluZz9cclxuICAgICAgICAgIGlmICghY29udGFjdC5Jc0VuYWJsZWQoKSB8fCAhY29udGFjdC5Jc1RvdWNoaW5nKCkpIHtcclxuICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgLy8gU2tpcCBzZW5zb3JzLlxyXG4gICAgICAgICAgY29uc3Qgc2Vuc29yQSA9IGNvbnRhY3QubV9maXh0dXJlQS5tX2lzU2Vuc29yO1xyXG4gICAgICAgICAgY29uc3Qgc2Vuc29yQiA9IGNvbnRhY3QubV9maXh0dXJlQi5tX2lzU2Vuc29yO1xyXG4gICAgICAgICAgaWYgKHNlbnNvckEgfHwgc2Vuc29yQikge1xyXG4gICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICBpc2xhbmQuQWRkQ29udGFjdChjb250YWN0KTtcclxuICAgICAgICAgIGNvbnRhY3QubV9pc2xhbmRGbGFnID0gdHJ1ZTtcclxuXHJcbiAgICAgICAgICBjb25zdCBvdGhlciA9IGNlLm90aGVyO1xyXG5cclxuICAgICAgICAgIC8vIFdhcyB0aGUgb3RoZXIgYm9keSBhbHJlYWR5IGFkZGVkIHRvIHRoaXMgaXNsYW5kP1xyXG4gICAgICAgICAgaWYgKG90aGVyLm1faXNsYW5kRmxhZykge1xyXG4gICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHN0YWNrQ291bnQgPCBzdGFja1NpemUpO1xyXG4gICAgICAgICAgc3RhY2tbc3RhY2tDb3VudCsrXSA9IG90aGVyO1xyXG4gICAgICAgICAgb3RoZXIubV9pc2xhbmRGbGFnID0gdHJ1ZTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIC8vIFNlYXJjaCBhbGwgam9pbnRzIGNvbm5lY3QgdG8gdGhpcyBib2R5LlxyXG4gICAgICAgIGZvciAobGV0IGplID0gYi5tX2pvaW50TGlzdDsgamU7IGplID0gamUubmV4dCkge1xyXG4gICAgICAgICAgaWYgKGplLmpvaW50Lm1faXNsYW5kRmxhZykge1xyXG4gICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICBjb25zdCBvdGhlciA9IGplLm90aGVyO1xyXG5cclxuICAgICAgICAgIC8vIERvbid0IHNpbXVsYXRlIGpvaW50cyBjb25uZWN0ZWQgdG8gaW5hY3RpdmUgYm9kaWVzLlxyXG4gICAgICAgICAgaWYgKCFvdGhlci5Jc0FjdGl2ZSgpKSB7XHJcbiAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgfVxyXG5cclxuICAgICAgICAgIGlzbGFuZC5BZGRKb2ludChqZS5qb2ludCk7XHJcbiAgICAgICAgICBqZS5qb2ludC5tX2lzbGFuZEZsYWcgPSB0cnVlO1xyXG5cclxuICAgICAgICAgIGlmIChvdGhlci5tX2lzbGFuZEZsYWcpIHtcclxuICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChzdGFja0NvdW50IDwgc3RhY2tTaXplKTtcclxuICAgICAgICAgIHN0YWNrW3N0YWNrQ291bnQrK10gPSBvdGhlcjtcclxuICAgICAgICAgIG90aGVyLm1faXNsYW5kRmxhZyA9IHRydWU7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcblxyXG4gICAgICBpZiAoKGlzbGFuZC5Tb2x2ZSh0aGlzLm1fcHJvZmlsZSwgc3RlcCwgdGhpcy5tX2dyYXZpdHksIHRoaXMubV9hbGxvd1NsZWVwKSAmIDMpID09PSAzKSB7XHJcbiAgICAgICAgaXNsYW5kLlNsZWVwQWxsKCk7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIFBvc3Qgc29sdmUgY2xlYW51cC5cclxuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCBpc2xhbmQubV9ib2R5Q291bnQ7ICsraSkge1xyXG4gICAgICAgIC8vIEFsbG93IHN0YXRpYyBib2RpZXMgdG8gcGFydGljaXBhdGUgaW4gb3RoZXIgaXNsYW5kcy5cclxuICAgICAgICBjb25zdCBiID0gaXNsYW5kLm1fYm9kaWVzW2ldO1xyXG4gICAgICAgIGlmIChiLkdldFR5cGUoKSA9PT0gYjJCb2R5VHlwZS5iMl9zdGF0aWNCb2R5KSB7XHJcbiAgICAgICAgICBiLm1faXNsYW5kRmxhZyA9IGZhbHNlO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgc3RhY2subGVuZ3RoOyArK2kpIHtcclxuICAgICAgaWYgKCFzdGFja1tpXSkge1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgICB9XHJcbiAgICAgIHN0YWNrW2ldID0gbnVsbDtcclxuICAgIH1cclxuXHJcbiAgICBjb25zdCB0aW1lciA9IFN0ZXBfc19icm9hZHBoYXNlVGltZXIuUmVzZXQoKTtcclxuXHJcbiAgICAvLyBTeW5jaHJvbml6ZSBmaXh0dXJlcywgY2hlY2sgZm9yIG91dCBvZiByYW5nZSBib2RpZXMuXHJcbiAgICB0aGlzLl9TeW5jaHJvbml6ZUZpeHR1cmVzQ2hlY2soKTtcclxuXHJcbiAgICAvLyBMb29rIGZvciBuZXcgY29udGFjdHMuXHJcbiAgICB0aGlzLm1fY29udGFjdE1hbmFnZXIuRmluZE5ld0NvbnRhY3RzKCk7XHJcbiAgICB0aGlzLm1fcHJvZmlsZS5icm9hZHBoYXNlID0gdGltZXIuR2V0TWlsbGlzZWNvbmRzKCk7XHJcbiAgfVxyXG5cclxuICBfU3luY2hyb25pemVGaXh0dXJlc0NoZWNrKCkge1xyXG4gICAgLy8gU3luY2hyb25pemUgZml4dHVyZXMsIGNoZWNrIGZvciBvdXQgb2YgcmFuZ2UgYm9kaWVzLlxyXG4gICAgZm9yIChsZXQgYiA9IHRoaXMubV9ib2R5TGlzdDsgYjsgYiA9IGIubV9uZXh0KSB7XHJcbiAgICAgIC8vIElmIGEgYm9keSB3YXMgbm90IGluIGFuIGlzbGFuZCB0aGVuIGl0IGRpZCBub3QgbW92ZS5cclxuICAgICAgaWYgKCFiLm1faXNsYW5kRmxhZykge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBpZiAoYi5HZXRUeXBlKCkgPT09IGIyQm9keVR5cGUuYjJfc3RhdGljQm9keSkge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBVcGRhdGUgZml4dHVyZXMgKGZvciBicm9hZC1waGFzZSkuXHJcbiAgICAgIGIuU3luY2hyb25pemVGaXh0dXJlcygpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgU29sdmVUT0koc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQge1xyXG4gICAgY29uc3QgaXNsYW5kOiBiMklzbGFuZCA9IHRoaXMubV9pc2xhbmQ7XHJcbiAgICBpc2xhbmQuSW5pdGlhbGl6ZShcclxuICAgICAgYjJfbWF4VE9JQ29udGFjdHMgPDwgMSxcclxuICAgICAgYjJfbWF4VE9JQ29udGFjdHMsXHJcbiAgICAgIDAsXHJcbiAgICAgIHRoaXMubV9jb250YWN0TWFuYWdlci5tX2NvbnRhY3RMaXN0ZW5lcixcclxuICAgICk7XHJcblxyXG4gICAgaWYgKHRoaXMubV9zdGVwQ29tcGxldGUpIHtcclxuICAgICAgZm9yIChsZXQgYjogYjJCb2R5IHwgbnVsbCA9IHRoaXMubV9ib2R5TGlzdDsgYjsgYiA9IGIubV9uZXh0KSB7XHJcbiAgICAgICAgYi5tX2lzbGFuZEZsYWcgPSBmYWxzZTtcclxuICAgICAgICBiLm1fc3dlZXAuYWxwaGEwID0gMDtcclxuICAgICAgfVxyXG5cclxuICAgICAgZm9yIChsZXQgYzogYjJDb250YWN0IHwgbnVsbCA9IHRoaXMubV9jb250YWN0TWFuYWdlci5tX2NvbnRhY3RMaXN0OyBjOyBjID0gYy5tX25leHQpIHtcclxuICAgICAgICAvLyBJbnZhbGlkYXRlIFRPSVxyXG4gICAgICAgIGMubV90b2lGbGFnID0gZmFsc2U7XHJcbiAgICAgICAgYy5tX2lzbGFuZEZsYWcgPSBmYWxzZTtcclxuICAgICAgICBjLm1fdG9pQ291bnQgPSAwO1xyXG4gICAgICAgIGMubV90b2kgPSAxLjA7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICAvLyBGaW5kIFRPSSBldmVudHMgYW5kIHNvbHZlIHRoZW0uXHJcbiAgICBmb3IgKDs7KSB7XHJcbiAgICAgIC8vIEZpbmQgdGhlIGZpcnN0IFRPSS5cclxuICAgICAgbGV0IG1pbkNvbnRhY3Q6IGIyQ29udGFjdCB8IG51bGwgPSBudWxsO1xyXG4gICAgICBsZXQgbWluQWxwaGEgPSAxLjA7XHJcblxyXG4gICAgICBmb3IgKGxldCBjID0gdGhpcy5tX2NvbnRhY3RNYW5hZ2VyLm1fY29udGFjdExpc3Q7IGM7IGMgPSBjLm1fbmV4dCkge1xyXG4gICAgICAgIC8vIElzIHRoaXMgY29udGFjdCBkaXNhYmxlZD9cclxuICAgICAgICBpZiAoIWMuSXNFbmFibGVkKCkpIHtcclxuICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgLy8gUHJldmVudCBleGNlc3NpdmUgc3ViLXN0ZXBwaW5nLlxyXG4gICAgICAgIGlmIChjLm1fdG9pQ291bnQgPiBiMl9tYXhTdWJTdGVwcykge1xyXG4gICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBsZXQgYWxwaGEgPSAxO1xyXG4gICAgICAgIGlmIChjLm1fdG9pRmxhZykge1xyXG4gICAgICAgICAgLy8gVGhpcyBjb250YWN0IGhhcyBhIHZhbGlkIGNhY2hlZCBUT0kuXHJcbiAgICAgICAgICBhbHBoYSA9IGMubV90b2k7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgIGNvbnN0IGZBOiBiMkZpeHR1cmUgPSBjLkdldEZpeHR1cmVBKCk7XHJcbiAgICAgICAgICBjb25zdCBmQjogYjJGaXh0dXJlID0gYy5HZXRGaXh0dXJlQigpO1xyXG5cclxuICAgICAgICAgIC8vIElzIHRoZXJlIGEgc2Vuc29yP1xyXG4gICAgICAgICAgaWYgKGZBLklzU2Vuc29yKCkgfHwgZkIuSXNTZW5zb3IoKSkge1xyXG4gICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICBjb25zdCBiQTogYjJCb2R5ID0gZkEuR2V0Qm9keSgpO1xyXG4gICAgICAgICAgY29uc3QgYkI6IGIyQm9keSA9IGZCLkdldEJvZHkoKTtcclxuXHJcbiAgICAgICAgICBjb25zdCB0eXBlQTogYjJCb2R5VHlwZSA9IGJBLm1fdHlwZTtcclxuICAgICAgICAgIGNvbnN0IHR5cGVCOiBiMkJvZHlUeXBlID0gYkIubV90eXBlO1xyXG4gICAgICAgICAgISFCMl9ERUJVRyAmJlxyXG4gICAgICAgICAgICBiMkFzc2VydCh0eXBlQSAhPT0gYjJCb2R5VHlwZS5iMl9zdGF0aWNCb2R5IHx8IHR5cGVCICE9PSBiMkJvZHlUeXBlLmIyX3N0YXRpY0JvZHkpO1xyXG5cclxuICAgICAgICAgIGNvbnN0IGFjdGl2ZUE6IGJvb2xlYW4gPSBiQS5Jc0F3YWtlKCkgJiYgdHlwZUEgIT09IGIyQm9keVR5cGUuYjJfc3RhdGljQm9keTtcclxuICAgICAgICAgIGNvbnN0IGFjdGl2ZUI6IGJvb2xlYW4gPSBiQi5Jc0F3YWtlKCkgJiYgdHlwZUIgIT09IGIyQm9keVR5cGUuYjJfc3RhdGljQm9keTtcclxuXHJcbiAgICAgICAgICAvLyBJcyBhdCBsZWFzdCBvbmUgYm9keSBhY3RpdmUgKGF3YWtlIGFuZCBkeW5hbWljIG9yIGtpbmVtYXRpYyk/XHJcbiAgICAgICAgICBpZiAoIWFjdGl2ZUEgJiYgIWFjdGl2ZUIpIHtcclxuICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgY29uc3QgY29sbGlkZUE6IGJvb2xlYW4gPSBiQS5Jc0J1bGxldCgpIHx8IHR5cGVBICE9PSBiMkJvZHlUeXBlLmIyX2R5bmFtaWNCb2R5O1xyXG4gICAgICAgICAgY29uc3QgY29sbGlkZUI6IGJvb2xlYW4gPSBiQi5Jc0J1bGxldCgpIHx8IHR5cGVCICE9PSBiMkJvZHlUeXBlLmIyX2R5bmFtaWNCb2R5O1xyXG5cclxuICAgICAgICAgIC8vIEFyZSB0aGVzZSB0d28gbm9uLWJ1bGxldCBkeW5hbWljIGJvZGllcz9cclxuICAgICAgICAgIGlmICghY29sbGlkZUEgJiYgIWNvbGxpZGVCKSB7XHJcbiAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgfVxyXG5cclxuICAgICAgICAgIC8vIENvbXB1dGUgdGhlIFRPSSBmb3IgdGhpcyBjb250YWN0LlxyXG4gICAgICAgICAgLy8gUHV0IHRoZSBzd2VlcHMgb250byB0aGUgc2FtZSB0aW1lIGludGVydmFsLlxyXG4gICAgICAgICAgbGV0IGFscGhhMDogbnVtYmVyID0gYkEubV9zd2VlcC5hbHBoYTA7XHJcblxyXG4gICAgICAgICAgaWYgKGJBLm1fc3dlZXAuYWxwaGEwIDwgYkIubV9zd2VlcC5hbHBoYTApIHtcclxuICAgICAgICAgICAgYWxwaGEwID0gYkIubV9zd2VlcC5hbHBoYTA7XHJcbiAgICAgICAgICAgIGJBLm1fc3dlZXAuQWR2YW5jZShhbHBoYTApO1xyXG4gICAgICAgICAgfSBlbHNlIGlmIChiQi5tX3N3ZWVwLmFscGhhMCA8IGJBLm1fc3dlZXAuYWxwaGEwKSB7XHJcbiAgICAgICAgICAgIGFscGhhMCA9IGJBLm1fc3dlZXAuYWxwaGEwO1xyXG4gICAgICAgICAgICBiQi5tX3N3ZWVwLkFkdmFuY2UoYWxwaGEwKTtcclxuICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGFscGhhMCA8IDEpO1xyXG5cclxuICAgICAgICAgIGNvbnN0IGluZGV4QTogbnVtYmVyID0gYy5HZXRDaGlsZEluZGV4QSgpO1xyXG4gICAgICAgICAgY29uc3QgaW5kZXhCOiBudW1iZXIgPSBjLkdldENoaWxkSW5kZXhCKCk7XHJcblxyXG4gICAgICAgICAgLy8gQ29tcHV0ZSB0aGUgdGltZSBvZiBpbXBhY3QgaW4gaW50ZXJ2YWwgWzAsIG1pblRPSV1cclxuICAgICAgICAgIGNvbnN0IGlucHV0OiBiMlRPSUlucHV0ID0gU29sdmVUT0lfc190b2lfaW5wdXQ7XHJcbiAgICAgICAgICBpbnB1dC5wcm94eUEuU2V0U2hhcGUoZkEuR2V0U2hhcGUoKSwgaW5kZXhBKTtcclxuICAgICAgICAgIGlucHV0LnByb3h5Qi5TZXRTaGFwZShmQi5HZXRTaGFwZSgpLCBpbmRleEIpO1xyXG4gICAgICAgICAgaW5wdXQuc3dlZXBBLkNvcHkoYkEubV9zd2VlcCk7XHJcbiAgICAgICAgICBpbnB1dC5zd2VlcEIuQ29weShiQi5tX3N3ZWVwKTtcclxuICAgICAgICAgIGlucHV0LnRNYXggPSAxO1xyXG5cclxuICAgICAgICAgIGNvbnN0IG91dHB1dDogYjJUT0lPdXRwdXQgPSBTb2x2ZVRPSV9zX3RvaV9vdXRwdXQ7XHJcbiAgICAgICAgICBiMlRpbWVPZkltcGFjdChvdXRwdXQsIGlucHV0KTtcclxuXHJcbiAgICAgICAgICAvLyBCZXRhIGlzIHRoZSBmcmFjdGlvbiBvZiB0aGUgcmVtYWluaW5nIHBvcnRpb24gb2YgdGhlIC5cclxuICAgICAgICAgIGNvbnN0IGJldGE6IG51bWJlciA9IG91dHB1dC50O1xyXG4gICAgICAgICAgaWYgKG91dHB1dC5zdGF0ZSA9PT0gYjJUT0lPdXRwdXRTdGF0ZS5lX3RvdWNoaW5nKSB7XHJcbiAgICAgICAgICAgIGFscGhhID0gYjJNaW4oYWxwaGEwICsgKDEgLSBhbHBoYTApICogYmV0YSwgMSk7XHJcbiAgICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgICBhbHBoYSA9IDE7XHJcbiAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgYy5tX3RvaSA9IGFscGhhO1xyXG4gICAgICAgICAgYy5tX3RvaUZsYWcgPSB0cnVlO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgaWYgKGFscGhhIDwgbWluQWxwaGEpIHtcclxuICAgICAgICAgIC8vIFRoaXMgaXMgdGhlIG1pbmltdW0gVE9JIGZvdW5kIHNvIGZhci5cclxuICAgICAgICAgIG1pbkNvbnRhY3QgPSBjO1xyXG4gICAgICAgICAgbWluQWxwaGEgPSBhbHBoYTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGlmIChtaW5Db250YWN0ID09PSBudWxsIHx8IDEgLSAxMCAqIGIyX2Vwc2lsb24gPCBtaW5BbHBoYSkge1xyXG4gICAgICAgIC8vIE5vIG1vcmUgVE9JIGV2ZW50cy4gRG9uZSFcclxuICAgICAgICB0aGlzLm1fc3RlcENvbXBsZXRlID0gdHJ1ZTtcclxuICAgICAgICBicmVhaztcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gQWR2YW5jZSB0aGUgYm9kaWVzIHRvIHRoZSBUT0kuXHJcbiAgICAgIGNvbnN0IGZBOiBiMkZpeHR1cmUgPSBtaW5Db250YWN0LkdldEZpeHR1cmVBKCk7XHJcbiAgICAgIGNvbnN0IGZCOiBiMkZpeHR1cmUgPSBtaW5Db250YWN0LkdldEZpeHR1cmVCKCk7XHJcbiAgICAgIGNvbnN0IGJBOiBiMkJvZHkgPSBmQS5HZXRCb2R5KCk7XHJcbiAgICAgIGNvbnN0IGJCOiBiMkJvZHkgPSBmQi5HZXRCb2R5KCk7XHJcblxyXG4gICAgICBjb25zdCBiYWNrdXAxID0gU29sdmVUT0lfc19iYWNrdXAxLkNvcHkoYkEubV9zd2VlcCk7XHJcbiAgICAgIGNvbnN0IGJhY2t1cDIgPSBTb2x2ZVRPSV9zX2JhY2t1cDIuQ29weShiQi5tX3N3ZWVwKTtcclxuXHJcbiAgICAgIGJBLkFkdmFuY2UobWluQWxwaGEpO1xyXG4gICAgICBiQi5BZHZhbmNlKG1pbkFscGhhKTtcclxuXHJcbiAgICAgIC8vIFRoZSBUT0kgY29udGFjdCBsaWtlbHkgaGFzIHNvbWUgbmV3IGNvbnRhY3QgcG9pbnRzLlxyXG4gICAgICBtaW5Db250YWN0LlVwZGF0ZSh0aGlzLm1fY29udGFjdE1hbmFnZXIubV9jb250YWN0TGlzdGVuZXIpO1xyXG4gICAgICBtaW5Db250YWN0Lm1fdG9pRmxhZyA9IGZhbHNlO1xyXG4gICAgICArK21pbkNvbnRhY3QubV90b2lDb3VudDtcclxuXHJcbiAgICAgIC8vIElzIHRoZSBjb250YWN0IHNvbGlkP1xyXG4gICAgICBpZiAoIW1pbkNvbnRhY3QuSXNFbmFibGVkKCkgfHwgIW1pbkNvbnRhY3QuSXNUb3VjaGluZygpKSB7XHJcbiAgICAgICAgLy8gUmVzdG9yZSB0aGUgc3dlZXBzLlxyXG4gICAgICAgIG1pbkNvbnRhY3QuU2V0RW5hYmxlZChmYWxzZSk7XHJcbiAgICAgICAgYkEubV9zd2VlcC5Db3B5KGJhY2t1cDEpO1xyXG4gICAgICAgIGJCLm1fc3dlZXAuQ29weShiYWNrdXAyKTtcclxuICAgICAgICBiQS5TeW5jaHJvbml6ZVRyYW5zZm9ybSgpO1xyXG4gICAgICAgIGJCLlN5bmNocm9uaXplVHJhbnNmb3JtKCk7XHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGJBLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICBiQi5TZXRBd2FrZSh0cnVlKTtcclxuXHJcbiAgICAgIC8vIEJ1aWxkIHRoZSBpc2xhbmRcclxuICAgICAgaXNsYW5kLkNsZWFyKCk7XHJcbiAgICAgIGlzbGFuZC5BZGRCb2R5KGJBKTtcclxuICAgICAgaXNsYW5kLkFkZEJvZHkoYkIpO1xyXG4gICAgICBpc2xhbmQuQWRkQ29udGFjdChtaW5Db250YWN0KTtcclxuXHJcbiAgICAgIGJBLm1faXNsYW5kRmxhZyA9IHRydWU7XHJcbiAgICAgIGJCLm1faXNsYW5kRmxhZyA9IHRydWU7XHJcbiAgICAgIG1pbkNvbnRhY3QubV9pc2xhbmRGbGFnID0gdHJ1ZTtcclxuXHJcbiAgICAgIC8vIEdldCBjb250YWN0cyBvbiBib2R5QSBhbmQgYm9keUIuXHJcbiAgICAgIC8vIGNvbnN0IGJvZGllczogYjJCb2R5W10gPSBbYkEsIGJCXTtcclxuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCAyOyArK2kpIHtcclxuICAgICAgICBjb25zdCBib2R5OiBiMkJvZHkgPSBpID09PSAwID8gYkEgOiBiQjsgLy8gYm9kaWVzW2ldO1xyXG4gICAgICAgIGlmIChib2R5Lm1fdHlwZSA9PT0gYjJCb2R5VHlwZS5iMl9keW5hbWljQm9keSkge1xyXG4gICAgICAgICAgZm9yIChsZXQgY2U6IGIyQ29udGFjdEVkZ2UgfCBudWxsID0gYm9keS5tX2NvbnRhY3RMaXN0OyBjZTsgY2UgPSBjZS5uZXh0KSB7XHJcbiAgICAgICAgICAgIGlmIChpc2xhbmQubV9ib2R5Q291bnQgPT09IGlzbGFuZC5tX2JvZHlDYXBhY2l0eSkge1xyXG4gICAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICBpZiAoaXNsYW5kLm1fY29udGFjdENvdW50ID09PSBpc2xhbmQubV9jb250YWN0Q2FwYWNpdHkpIHtcclxuICAgICAgICAgICAgICBicmVhaztcclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgY29uc3QgY29udGFjdDogYjJDb250YWN0ID0gY2UuY29udGFjdDtcclxuXHJcbiAgICAgICAgICAgIC8vIEhhcyB0aGlzIGNvbnRhY3QgYWxyZWFkeSBiZWVuIGFkZGVkIHRvIHRoZSBpc2xhbmQ/XHJcbiAgICAgICAgICAgIGlmIChjb250YWN0Lm1faXNsYW5kRmxhZykge1xyXG4gICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICAvLyBPbmx5IGFkZCBzdGF0aWMsIGtpbmVtYXRpYywgb3IgYnVsbGV0IGJvZGllcy5cclxuICAgICAgICAgICAgY29uc3Qgb3RoZXIgPSBjZS5vdGhlcjtcclxuICAgICAgICAgICAgaWYgKFxyXG4gICAgICAgICAgICAgIG90aGVyLm1fdHlwZSA9PT0gYjJCb2R5VHlwZS5iMl9keW5hbWljQm9keSAmJlxyXG4gICAgICAgICAgICAgICFib2R5LklzQnVsbGV0KCkgJiZcclxuICAgICAgICAgICAgICAhb3RoZXIuSXNCdWxsZXQoKVxyXG4gICAgICAgICAgICApIHtcclxuICAgICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgLy8gU2tpcCBzZW5zb3JzLlxyXG4gICAgICAgICAgICBjb25zdCBzZW5zb3JBID0gY29udGFjdC5tX2ZpeHR1cmVBLm1faXNTZW5zb3I7XHJcbiAgICAgICAgICAgIGNvbnN0IHNlbnNvckIgPSBjb250YWN0Lm1fZml4dHVyZUIubV9pc1NlbnNvcjtcclxuICAgICAgICAgICAgaWYgKHNlbnNvckEgfHwgc2Vuc29yQikge1xyXG4gICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICAvLyBUZW50YXRpdmVseSBhZHZhbmNlIHRoZSBib2R5IHRvIHRoZSBUT0kuXHJcbiAgICAgICAgICAgIGNvbnN0IGJhY2t1cCA9IFNvbHZlVE9JX3NfYmFja3VwLkNvcHkob3RoZXIubV9zd2VlcCk7XHJcbiAgICAgICAgICAgIGlmICghb3RoZXIubV9pc2xhbmRGbGFnKSB7XHJcbiAgICAgICAgICAgICAgb3RoZXIuQWR2YW5jZShtaW5BbHBoYSk7XHJcbiAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgIC8vIFVwZGF0ZSB0aGUgY29udGFjdCBwb2ludHNcclxuICAgICAgICAgICAgY29udGFjdC5VcGRhdGUodGhpcy5tX2NvbnRhY3RNYW5hZ2VyLm1fY29udGFjdExpc3RlbmVyKTtcclxuXHJcbiAgICAgICAgICAgIC8vIFdhcyB0aGUgY29udGFjdCBkaXNhYmxlZCBieSB0aGUgdXNlcj9cclxuICAgICAgICAgICAgaWYgKCFjb250YWN0LklzRW5hYmxlZCgpKSB7XHJcbiAgICAgICAgICAgICAgb3RoZXIubV9zd2VlcC5Db3B5KGJhY2t1cCk7XHJcbiAgICAgICAgICAgICAgb3RoZXIuU3luY2hyb25pemVUcmFuc2Zvcm0oKTtcclxuICAgICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgLy8gQXJlIHRoZXJlIGNvbnRhY3QgcG9pbnRzP1xyXG4gICAgICAgICAgICBpZiAoIWNvbnRhY3QuSXNUb3VjaGluZygpKSB7XHJcbiAgICAgICAgICAgICAgb3RoZXIubV9zd2VlcC5Db3B5KGJhY2t1cCk7XHJcbiAgICAgICAgICAgICAgb3RoZXIuU3luY2hyb25pemVUcmFuc2Zvcm0oKTtcclxuICAgICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgLy8gQWRkIHRoZSBjb250YWN0IHRvIHRoZSBpc2xhbmRcclxuICAgICAgICAgICAgY29udGFjdC5tX2lzbGFuZEZsYWcgPSB0cnVlO1xyXG4gICAgICAgICAgICBpc2xhbmQuQWRkQ29udGFjdChjb250YWN0KTtcclxuXHJcbiAgICAgICAgICAgIC8vIEhhcyB0aGUgb3RoZXIgYm9keSBhbHJlYWR5IGJlZW4gYWRkZWQgdG8gdGhlIGlzbGFuZD9cclxuICAgICAgICAgICAgaWYgKG90aGVyLm1faXNsYW5kRmxhZykge1xyXG4gICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICAvLyBBZGQgdGhlIG90aGVyIGJvZHkgdG8gdGhlIGlzbGFuZC5cclxuICAgICAgICAgICAgb3RoZXIubV9pc2xhbmRGbGFnID0gdHJ1ZTtcclxuXHJcbiAgICAgICAgICAgIGlmIChvdGhlci5tX3R5cGUgIT09IGIyQm9keVR5cGUuYjJfc3RhdGljQm9keSkge1xyXG4gICAgICAgICAgICAgIG90aGVyLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICBpc2xhbmQuQWRkQm9keShvdGhlcik7XHJcbiAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcblxyXG4gICAgICBjb25zdCBzdWJTdGVwID0gU29sdmVUT0lfc19zdWJTdGVwO1xyXG4gICAgICBzdWJTdGVwLmR0ID0gKDEgLSBtaW5BbHBoYSkgKiBzdGVwLmR0O1xyXG4gICAgICBzdWJTdGVwLmludl9kdCA9IDEgLyBzdWJTdGVwLmR0O1xyXG4gICAgICBzdWJTdGVwLmR0UmF0aW8gPSAxO1xyXG4gICAgICBzdWJTdGVwLnBvc2l0aW9uSXRlcmF0aW9ucyA9IDIwO1xyXG4gICAgICBzdWJTdGVwLnZlbG9jaXR5SXRlcmF0aW9ucyA9IHN0ZXAudmVsb2NpdHlJdGVyYXRpb25zO1xyXG4gICAgICBpZiAoQjJfRU5BQkxFX1BBUlRJQ0xFKSB7XHJcbiAgICAgICAgc3ViU3RlcC5wYXJ0aWNsZUl0ZXJhdGlvbnMgPSBzdGVwLnBhcnRpY2xlSXRlcmF0aW9ucztcclxuICAgICAgfVxyXG4gICAgICBzdWJTdGVwLndhcm1TdGFydGluZyA9IGZhbHNlO1xyXG4gICAgICBpc2xhbmQuU29sdmVUT0koc3ViU3RlcCwgYkEubV9pc2xhbmRJbmRleCwgYkIubV9pc2xhbmRJbmRleCk7XHJcblxyXG4gICAgICAvLyBSZXNldCBpc2xhbmQgZmxhZ3MgYW5kIHN5bmNocm9uaXplIGJyb2FkLXBoYXNlIHByb3hpZXMuXHJcbiAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgaXNsYW5kLm1fYm9keUNvdW50OyArK2kpIHtcclxuICAgICAgICBjb25zdCBib2R5OiBiMkJvZHkgPSBpc2xhbmQubV9ib2RpZXNbaV07XHJcbiAgICAgICAgYm9keS5tX2lzbGFuZEZsYWcgPSBmYWxzZTtcclxuXHJcbiAgICAgICAgaWYgKGJvZHkubV90eXBlICE9PSBiMkJvZHlUeXBlLmIyX2R5bmFtaWNCb2R5KSB7XHJcbiAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIGJvZHkuU3luY2hyb25pemVGaXh0dXJlcygpO1xyXG5cclxuICAgICAgICAvLyBJbnZhbGlkYXRlIGFsbCBjb250YWN0IFRPSXMgb24gdGhpcyBkaXNwbGFjZWQgYm9keS5cclxuICAgICAgICBmb3IgKGxldCBjZTogYjJDb250YWN0RWRnZSB8IG51bGwgPSBib2R5Lm1fY29udGFjdExpc3Q7IGNlOyBjZSA9IGNlLm5leHQpIHtcclxuICAgICAgICAgIGNlLmNvbnRhY3QubV90b2lGbGFnID0gZmFsc2U7XHJcbiAgICAgICAgICBjZS5jb250YWN0Lm1faXNsYW5kRmxhZyA9IGZhbHNlO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gQ29tbWl0IGZpeHR1cmUgcHJveHkgbW92ZW1lbnRzIHRvIHRoZSBicm9hZC1waGFzZSBzbyB0aGF0IG5ldyBjb250YWN0cyBhcmUgY3JlYXRlZC5cclxuICAgICAgLy8gQWxzbywgc29tZSBjb250YWN0cyBjYW4gYmUgZGVzdHJveWVkLlxyXG4gICAgICB0aGlzLm1fY29udGFjdE1hbmFnZXIuRmluZE5ld0NvbnRhY3RzKCk7XHJcblxyXG4gICAgICBpZiAodGhpcy5tX3N1YlN0ZXBwaW5nKSB7XHJcbiAgICAgICAgdGhpcy5tX3N0ZXBDb21wbGV0ZSA9IGZhbHNlO1xyXG4gICAgICAgIGJyZWFrO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBBZGRDb250cm9sbGVyKGNvbnRyb2xsZXI6IGIyQ29udHJvbGxlcik6IGIyQ29udHJvbGxlciB7XHJcbiAgICBpZiAoIUIyX0VOQUJMRV9DT05UUk9MTEVSKSB7XHJcbiAgICAgIHJldHVybiBjb250cm9sbGVyO1xyXG4gICAgfVxyXG4gICAgLy8gYjJBc3NlcnQoY29udHJvbGxlci5tX3dvcmxkID09PSBudWxsLCBcIkNvbnRyb2xsZXIgY2FuIG9ubHkgYmUgYSBtZW1iZXIgb2Ygb25lIHdvcmxkXCIpO1xyXG4gICAgLy8gY29udHJvbGxlci5tX3dvcmxkID0gdGhpcztcclxuICAgIGNvbnRyb2xsZXIubV9uZXh0ID0gdGhpcy5tX2NvbnRyb2xsZXJMaXN0O1xyXG4gICAgY29udHJvbGxlci5tX3ByZXYgPSBudWxsO1xyXG4gICAgaWYgKHRoaXMubV9jb250cm9sbGVyTGlzdCkge1xyXG4gICAgICB0aGlzLm1fY29udHJvbGxlckxpc3QubV9wcmV2ID0gY29udHJvbGxlcjtcclxuICAgIH1cclxuICAgIHRoaXMubV9jb250cm9sbGVyTGlzdCA9IGNvbnRyb2xsZXI7XHJcbiAgICArK3RoaXMubV9jb250cm9sbGVyQ291bnQ7XHJcbiAgICByZXR1cm4gY29udHJvbGxlcjtcclxuICB9XHJcblxyXG4gIFJlbW92ZUNvbnRyb2xsZXIoY29udHJvbGxlcjogYjJDb250cm9sbGVyKTogYjJDb250cm9sbGVyIHtcclxuICAgIGlmICghQjJfRU5BQkxFX0NPTlRST0xMRVIpIHtcclxuICAgICAgcmV0dXJuIGNvbnRyb2xsZXI7XHJcbiAgICB9XHJcbiAgICAvLyBiMkFzc2VydChjb250cm9sbGVyLm1fd29ybGQgPT09IHRoaXMsIFwiQ29udHJvbGxlciBpcyBub3QgYSBtZW1iZXIgb2YgdGhpcyB3b3JsZFwiKTtcclxuICAgIGlmIChjb250cm9sbGVyLm1fcHJldikge1xyXG4gICAgICBjb250cm9sbGVyLm1fcHJldi5tX25leHQgPSBjb250cm9sbGVyLm1fbmV4dDtcclxuICAgIH1cclxuICAgIGlmIChjb250cm9sbGVyLm1fbmV4dCkge1xyXG4gICAgICBjb250cm9sbGVyLm1fbmV4dC5tX3ByZXYgPSBjb250cm9sbGVyLm1fcHJldjtcclxuICAgIH1cclxuICAgIGlmICh0aGlzLm1fY29udHJvbGxlckxpc3QgPT09IGNvbnRyb2xsZXIpIHtcclxuICAgICAgdGhpcy5tX2NvbnRyb2xsZXJMaXN0ID0gY29udHJvbGxlci5tX25leHQ7XHJcbiAgICB9XHJcbiAgICAtLXRoaXMubV9jb250cm9sbGVyQ291bnQ7XHJcbiAgICBjb250cm9sbGVyLm1fcHJldiA9IG51bGw7XHJcbiAgICBjb250cm9sbGVyLm1fbmV4dCA9IG51bGw7XHJcbiAgICAvLyBkZWxldGUgY29udHJvbGxlci5tX3dvcmxkOyAvLyA9IG51bGw7XHJcbiAgICByZXR1cm4gY29udHJvbGxlcjtcclxuICB9XHJcbn1cclxuIl19