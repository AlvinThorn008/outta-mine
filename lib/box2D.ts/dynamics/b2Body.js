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
import { b2Assert } from '../common/b2Settings';
import { b2IsValid, b2Rot, b2Sweep, b2Transform, b2Vec2 } from '../common/b2Math';
import { b2MassData, b2Shape } from '../collision/shapes/b2Shape';
import { b2Fixture, b2FixtureDef } from './b2Fixture';
/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
export class b2BodyDef {
    constructor() {
        /// The body type: static, kinematic, or dynamic.
        /// Note: if a dynamic body would have zero mass, the mass is set to one.
        this.type = 0 /* b2_staticBody */;
        /// The world position of the body. Avoid creating bodies at the origin
        /// since this can lead to many overlapping shapes.
        this.position = new b2Vec2();
        /// The world angle of the body in radians.
        this.angle = NaN;
        /// The linear velocity of the body's origin in world co-ordinates.
        this.linearVelocity = new b2Vec2();
        /// The angular velocity of the body.
        this.angularVelocity = NaN;
        /// Linear damping is use to reduce the linear velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        this.linearDamping = NaN;
        /// Angular damping is use to reduce the angular velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        this.angularDamping = NaN;
        /// Set this flag to false if this body should never fall asleep. Note that
        /// this increases CPU usage.
        this.allowSleep = true;
        /// Is this body initially awake or sleeping?
        this.awake = true;
        /// Should this body be prevented from rotating? Useful for characters.
        this.fixedRotation = false;
        /// Is this a fast moving body that should be prevented from tunneling through
        /// other moving bodies? Note that all bodies are prevented from tunneling through
        /// kinematic and static bodies. This setting is only considered on dynamic bodies.
        /// @warning You should use this flag sparingly since it increases processing time.
        this.bullet = false;
        /// Does this body start out active?
        this.active = true;
        /// Use this to store application specific body data.
        this.userData = null;
        /// Scale the gravity applied to this body.
        this.gravityScale = NaN; // 1.0
        this.angle = 0.0;
        this.angularVelocity = 0.0;
        this.linearDamping = 0.0;
        this.angularDamping = 0.0;
        this.gravityScale = 1.0;
    }
}
/// A rigid body. These are created via b2World::CreateBody.
export class b2Body {
    // #endif
    constructor(bd, world) {
        this.m_islandFlag = false;
        this.m_awakeFlag = false;
        this.m_autoSleepFlag = false;
        this.m_bulletFlag = false;
        this.m_fixedRotationFlag = false;
        this.m_activeFlag = false;
        this.m_toiFlag = false;
        this.m_islandIndex = 0;
        this.m_xf = new b2Transform(); // the body origin transform
        // #if B2_ENABLE_PARTICLE
        this.m_xf0 = new b2Transform();
        // #endif
        this.m_sweep = new b2Sweep(); // the swept motion for CCD
        this.m_linearVelocity = new b2Vec2();
        this.m_angularVelocity = NaN;
        this.m_force = new b2Vec2();
        this.m_torque = NaN;
        this.m_prev = null;
        this.m_next = null;
        this.m_fixtureList = null;
        this.m_fixtureCount = 0;
        this.m_jointList = null;
        this.m_contactList = null;
        this.m_mass = NaN;
        this.m_invMass = NaN;
        // Rotational inertia about the center of mass.
        this.m_I = NaN;
        this.m_invI = NaN;
        this.m_linearDamping = NaN;
        this.m_angularDamping = NaN;
        this.m_gravityScale = NaN;
        this.m_sleepTime = NaN;
        this.m_userData = null;
        // #if B2_ENABLE_CONTROLLER
        this.m_controllerList = null;
        this.m_controllerCount = 0;
        this.m_bulletFlag = bd.bullet ?? false;
        this.m_fixedRotationFlag = bd.fixedRotation ?? false;
        this.m_autoSleepFlag = bd.allowSleep ?? true;
        this.m_awakeFlag = bd.awake ?? true;
        this.m_activeFlag = bd.active ?? true;
        this.m_world = world;
        this.m_xf.p.Copy(bd.position ?? b2Vec2.ZERO);
        !!B2_DEBUG && b2Assert(this.m_xf.p.IsValid());
        this.m_xf.q.SetAngle(bd.angle ?? 0.0);
        !!B2_DEBUG && b2Assert(b2IsValid(this.m_xf.q.GetAngle()));
        if (B2_ENABLE_PARTICLE) {
            this.m_xf0.Copy(this.m_xf);
        }
        this.m_sweep.localCenter.SetZero();
        this.m_sweep.c0.Copy(this.m_xf.p);
        this.m_sweep.c.Copy(this.m_xf.p);
        this.m_sweep.a0 = this.m_sweep.a = this.m_xf.q.GetAngle();
        this.m_sweep.alpha0 = 0;
        this.m_linearVelocity.Copy(bd.linearVelocity ?? b2Vec2.ZERO);
        !!B2_DEBUG && b2Assert(this.m_linearVelocity.IsValid());
        this.m_angularVelocity = bd.angularVelocity ?? 0;
        !!B2_DEBUG && b2Assert(b2IsValid(this.m_angularVelocity));
        this.m_linearDamping = bd.linearDamping ?? 0.0;
        this.m_angularDamping = bd.angularDamping ?? 0.0;
        this.m_gravityScale = bd.gravityScale ?? 1.0;
        !!B2_DEBUG && b2Assert(b2IsValid(this.m_gravityScale) && this.m_gravityScale >= 0);
        !!B2_DEBUG && b2Assert(b2IsValid(this.m_angularDamping) && this.m_angularDamping >= 0);
        !!B2_DEBUG && b2Assert(b2IsValid(this.m_linearDamping) && this.m_linearDamping >= 0);
        this.m_force.SetZero();
        this.m_torque = 0.0;
        this.m_sleepTime = 0.0;
        this.m_type = bd.type ?? 0 /* b2_staticBody */;
        if (bd.type === 2 /* b2_dynamicBody */) {
            this.m_mass = 1.0;
            this.m_invMass = 1.0;
        }
        else {
            this.m_mass = 0.0;
            this.m_invMass = 0.0;
        }
        this.m_I = 0.0;
        this.m_invI = 0.0;
        this.m_userData = bd.userData;
    }
    CreateFixture(a, b = 0) {
        if (a instanceof b2Shape) {
            return this.CreateFixtureShapeDensity(a, b);
        }
        else {
            return this.CreateFixtureDef(a);
        }
    }
    /// Creates a fixture and attach it to this body. Use this function if you need
    /// to set some fixture parameters, like friction. Otherwise you can create the
    /// fixture directly from a shape.
    /// If the density is non-zero, this function automatically updates the mass of the body.
    /// Contacts are not created until the next time step.
    /// @param def the fixture definition.
    /// @warning This function is locked during callbacks.
    CreateFixtureDef(def) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        const fixture = new b2Fixture(this, def);
        if (this.m_activeFlag) {
            fixture.CreateProxies();
        }
        fixture.m_next = this.m_fixtureList;
        this.m_fixtureList = fixture;
        ++this.m_fixtureCount;
        // fixture.m_body = this;
        // Adjust mass properties if needed.
        if (fixture.m_density > 0) {
            this.ResetMassData();
        }
        // Let the world know we have a new fixture. This will cause new contacts
        // to be created at the beginning of the next time step.
        this.m_world.m_newFixture = true;
        return fixture;
    }
    CreateFixtureShapeDensity(shape, density = 0) {
        const def = b2Body.CreateFixtureShapeDensity_s_def;
        def.shape = shape;
        def.density = density;
        return this.CreateFixtureDef(def);
    }
    /// Destroy a fixture. This removes the fixture from the broad-phase and
    /// destroys all contacts associated with this fixture. This will
    /// automatically adjust the mass of the body if the body is dynamic and the
    /// fixture has positive density.
    /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
    /// @param fixture the fixture to be removed.
    /// @warning This function is locked during callbacks.
    DestroyFixture(fixture) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        !!B2_DEBUG && b2Assert(fixture.m_body === this);
        // Remove the fixture from this body's singly linked list.
        !!B2_DEBUG && b2Assert(this.m_fixtureCount > 0);
        let node = this.m_fixtureList;
        let ppF = null;
        // DEBUG: let found: boolean = false;
        while (node !== null) {
            if (node === fixture) {
                if (ppF) {
                    ppF.m_next = fixture.m_next;
                }
                else {
                    this.m_fixtureList = fixture.m_next;
                }
                // DEBUG: found = true;
                break;
            }
            ppF = node;
            node = node.m_next;
        }
        // You tried to remove a shape that is not attached to this body.
        // TODO: debug
        //!!B2_DEBUG && b2Assert(found);
        // Destroy any contacts associated with the fixture.
        let edge = this.m_contactList;
        while (edge) {
            const c = edge.contact;
            edge = edge.next;
            const fixtureA = c.GetFixtureA();
            const fixtureB = c.GetFixtureB();
            if (fixture === fixtureA || fixture === fixtureB) {
                // This destroys the contact and removes it from
                // this body's contact list.
                this.m_world.m_contactManager.Destroy(c);
            }
        }
        if (this.m_activeFlag) {
            fixture.DestroyProxies();
        }
        // fixture.m_body = null;
        fixture.m_next = null;
        fixture.Reset();
        --this.m_fixtureCount;
        // Reset the mass data.
        this.ResetMassData();
    }
    /// Set the position of the body's origin and rotation.
    /// This breaks any contacts and wakes the other bodies.
    /// Manipulating a body's transform may cause non-physical behavior.
    /// @param position the world position of the body's local origin.
    /// @param angle the world rotation in radians.
    SetTransformVec(position, angle) {
        this.SetTransformXY(position.x, position.y, angle);
    }
    SetTransformXY(x, y, angle) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        this.m_xf.q.SetAngle(angle);
        this.m_xf.p.Set(x, y);
        if (B2_ENABLE_PARTICLE) {
            this.m_xf0.Copy(this.m_xf);
        }
        b2Transform.MulXV(this.m_xf, this.m_sweep.localCenter, this.m_sweep.c);
        this.m_sweep.a = angle;
        this.m_sweep.c0.Copy(this.m_sweep.c);
        this.m_sweep.a0 = angle;
        for (let f = this.m_fixtureList; f; f = f.m_next) {
            f.SynchronizeProxies(this.m_xf, this.m_xf, b2Vec2.ZERO);
        }
        this.m_world.m_contactManager.FindNewContacts();
    }
    SetTransform(xf) {
        this.SetTransformVec(xf.p, xf.GetAngle());
    }
    /// Get the body transform for the body's origin.
    /// @return the world transform of the body's origin.
    GetTransform() {
        return this.m_xf;
    }
    /// Get the world body origin position.
    /// @return the world position of the body's origin.
    GetPosition() {
        return this.m_xf.p;
    }
    SetPosition(position) {
        this.SetTransformVec(position, this.GetAngle());
    }
    SetPositionXY(x, y) {
        this.SetTransformXY(x, y, this.GetAngle());
    }
    /// Get the angle in radians.
    /// @return the current world rotation angle in radians.
    GetAngle() {
        return this.m_sweep.a;
    }
    SetAngle(angle) {
        this.SetTransformVec(this.GetPosition(), angle);
    }
    /// Get the world position of the center of mass.
    GetWorldCenter() {
        return this.m_sweep.c;
    }
    /// Get the local position of the center of mass.
    GetLocalCenter() {
        return this.m_sweep.localCenter;
    }
    /// Set the linear velocity of the center of mass.
    /// @param v the new linear velocity of the center of mass.
    SetLinearVelocity(v) {
        this.SetLinearVelocityXY(v.x, v.y);
    }
    SetLinearVelocityXY(x, y) {
        if (this.m_type === 0 /* b2_staticBody */) {
            return;
        }
        if (x * x + y * y > 0) {
            this.SetAwake(true);
        }
        this.m_linearVelocity.Set(x, y);
    }
    /// Get the linear velocity of the center of mass.
    /// @return the linear velocity of the center of mass.
    GetLinearVelocity() {
        return this.m_linearVelocity;
    }
    /// Set the angular velocity.
    /// @param omega the new angular velocity in radians/second.
    SetAngularVelocity(w) {
        if (this.m_type === 0 /* b2_staticBody */) {
            return;
        }
        if (w * w > 0) {
            this.SetAwake(true);
        }
        this.m_angularVelocity = w;
    }
    /// Get the angular velocity.
    /// @return the angular velocity in radians/second.
    GetAngularVelocity() {
        return this.m_angularVelocity;
    }
    GetDefinition(bd) {
        bd.type = this.GetType();
        bd.allowSleep = this.m_autoSleepFlag;
        bd.angle = this.GetAngle();
        bd.angularDamping = this.m_angularDamping;
        bd.gravityScale = this.m_gravityScale;
        bd.angularVelocity = this.m_angularVelocity;
        bd.fixedRotation = this.m_fixedRotationFlag;
        bd.bullet = this.m_bulletFlag;
        bd.awake = this.m_awakeFlag;
        bd.linearDamping = this.m_linearDamping;
        bd.linearVelocity.Copy(this.GetLinearVelocity());
        bd.position.Copy(this.GetPosition());
        bd.userData = this.GetUserData();
        return bd;
    }
    /// Apply a force at a world point. If the force is not
    /// applied at the center of mass, it will generate a torque and
    /// affect the angular velocity. This wakes up the body.
    /// @param force the world force vector, usually in Newtons (N).
    /// @param point the world position of the point of application.
    /// @param wake also wake up the body
    ApplyForce(force, point, wake = true) {
        if (this.m_type !== 2 /* b2_dynamicBody */) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_force.x += force.x;
            this.m_force.y += force.y;
            this.m_torque +=
                (point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x;
        }
    }
    /// Apply a force to the center of mass. This wakes up the body.
    /// @param force the world force vector, usually in Newtons (N).
    /// @param wake also wake up the body
    ApplyForceToCenter(force, wake = true) {
        if (this.m_type !== 2 /* b2_dynamicBody */) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_force.x += force.x;
            this.m_force.y += force.y;
        }
    }
    /// Apply a torque. This affects the angular velocity
    /// without affecting the linear velocity of the center of mass.
    /// @param torque about the z-axis (out of the screen), usually in N-m.
    /// @param wake also wake up the body
    ApplyTorque(torque, wake = true) {
        if (this.m_type !== 2 /* b2_dynamicBody */) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_torque += torque;
        }
    }
    /// Apply an impulse at a point. This immediately modifies the velocity.
    /// It also modifies the angular velocity if the point of application
    /// is not at the center of mass. This wakes up the body.
    /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
    /// @param point the world position of the point of application.
    /// @param wake also wake up the body
    ApplyLinearImpulse(impulse, point, wake = true) {
        if (this.m_type !== 2 /* b2_dynamicBody */) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_linearVelocity.x += this.m_invMass * impulse.x;
            this.m_linearVelocity.y += this.m_invMass * impulse.y;
            this.m_angularVelocity +=
                this.m_invI *
                    ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
        }
    }
    /// Apply an impulse at the center of gravity. This immediately modifies the velocity.
    /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
    /// @param wake also wake up the body
    ApplyLinearImpulseToCenter(impulse, wake = true) {
        if (this.m_type !== 2 /* b2_dynamicBody */) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_linearVelocity.x += this.m_invMass * impulse.x;
            this.m_linearVelocity.y += this.m_invMass * impulse.y;
        }
    }
    /// Apply an angular impulse.
    /// @param impulse the angular impulse in units of kg*m*m/s
    /// @param wake also wake up the body
    ApplyAngularImpulse(impulse, wake = true) {
        if (this.m_type !== 2 /* b2_dynamicBody */) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_angularVelocity += this.m_invI * impulse;
        }
    }
    /// Get the total mass of the body.
    /// @return the mass, usually in kilograms (kg).
    GetMass() {
        return this.m_mass;
    }
    /// Get the rotational inertia of the body about the local origin.
    /// @return the rotational inertia, usually in kg-m^2.
    GetInertia() {
        return (this.m_I + this.m_mass * b2Vec2.DotVV(this.m_sweep.localCenter, this.m_sweep.localCenter));
    }
    /// Get the mass data of the body.
    /// @return a struct containing the mass, inertia and center of the body.
    GetMassData(data) {
        data.mass = this.m_mass;
        data.I =
            this.m_I + this.m_mass * b2Vec2.DotVV(this.m_sweep.localCenter, this.m_sweep.localCenter);
        data.center.Copy(this.m_sweep.localCenter);
        return data;
    }
    SetMassData(massData) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        if (this.m_type !== 2 /* b2_dynamicBody */) {
            return;
        }
        this.m_invMass = 0;
        this.m_I = 0;
        this.m_invI = 0;
        this.m_mass = massData.mass;
        if (this.m_mass <= 0) {
            this.m_mass = 1;
        }
        this.m_invMass = 1 / this.m_mass;
        if (massData.I > 0 && !this.m_fixedRotationFlag) {
            this.m_I = massData.I - this.m_mass * b2Vec2.DotVV(massData.center, massData.center);
            !!B2_DEBUG && b2Assert(this.m_I > 0);
            this.m_invI = 1 / this.m_I;
        }
        // Move center of mass.
        const oldCenter = b2Body.SetMassData_s_oldCenter.Copy(this.m_sweep.c);
        this.m_sweep.localCenter.Copy(massData.center);
        b2Transform.MulXV(this.m_xf, this.m_sweep.localCenter, this.m_sweep.c);
        this.m_sweep.c0.Copy(this.m_sweep.c);
        // Update center of mass velocity.
        b2Vec2.AddVCrossSV(this.m_linearVelocity, this.m_angularVelocity, b2Vec2.SubVV(this.m_sweep.c, oldCenter, b2Vec2.s_t0), this.m_linearVelocity);
    }
    ResetMassData() {
        // Compute mass data from shapes. Each shape has its own density.
        this.m_mass = 0;
        this.m_invMass = 0;
        this.m_I = 0;
        this.m_invI = 0;
        this.m_sweep.localCenter.SetZero();
        // Static and kinematic bodies have zero mass.
        if (this.m_type === 0 /* b2_staticBody */ || this.m_type === 1 /* b2_kinematicBody */) {
            this.m_sweep.c0.Copy(this.m_xf.p);
            this.m_sweep.c.Copy(this.m_xf.p);
            this.m_sweep.a0 = this.m_sweep.a;
            return;
        }
        !!B2_DEBUG && b2Assert(this.m_type === 2 /* b2_dynamicBody */);
        // Accumulate mass over all fixtures.
        const localCenter = b2Body.ResetMassData_s_localCenter.SetZero();
        for (let f = this.m_fixtureList; f; f = f.m_next) {
            if (f.m_density === 0) {
                continue;
            }
            const massData = f.GetMassData(b2Body.ResetMassData_s_massData);
            this.m_mass += massData.mass;
            localCenter.x += massData.center.x * massData.mass;
            localCenter.y += massData.center.y * massData.mass;
            this.m_I += massData.I;
        }
        // Compute center of mass.
        if (this.m_mass > 0) {
            this.m_invMass = 1 / this.m_mass;
            localCenter.x *= this.m_invMass;
            localCenter.y *= this.m_invMass;
        }
        else {
            // Force all dynamic bodies to have a positive mass.
            this.m_mass = 1;
            this.m_invMass = 1;
        }
        if (this.m_I > 0 && !this.m_fixedRotationFlag) {
            // Center the inertia about the center of mass.
            this.m_I -= this.m_mass * b2Vec2.DotVV(localCenter, localCenter);
            !!B2_DEBUG && b2Assert(this.m_I > 0);
            this.m_invI = 1 / this.m_I;
        }
        else {
            this.m_I = 0;
            this.m_invI = 0;
        }
        // Move center of mass.
        const oldCenter = b2Body.ResetMassData_s_oldCenter.Copy(this.m_sweep.c);
        this.m_sweep.localCenter.Copy(localCenter);
        b2Transform.MulXV(this.m_xf, this.m_sweep.localCenter, this.m_sweep.c);
        this.m_sweep.c0.Copy(this.m_sweep.c);
        // Update center of mass velocity.
        b2Vec2.AddVCrossSV(this.m_linearVelocity, this.m_angularVelocity, b2Vec2.SubVV(this.m_sweep.c, oldCenter, b2Vec2.s_t0), this.m_linearVelocity);
    }
    /// Get the world coordinates of a point given the local coordinates.
    /// @param localPoint a point on the body measured relative the the body's origin.
    /// @return the same point expressed in world coordinates.
    GetWorldPoint(localPoint, out) {
        return b2Transform.MulXV(this.m_xf, localPoint, out);
    }
    /// Get the world coordinates of a vector given the local coordinates.
    /// @param localVector a vector fixed in the body.
    /// @return the same vector expressed in world coordinates.
    GetWorldVector(localVector, out) {
        return b2Rot.MulRV(this.m_xf.q, localVector, out);
    }
    /// Gets a local point relative to the body's origin given a world point.
    /// @param a point in world coordinates.
    /// @return the corresponding local point relative to the body's origin.
    GetLocalPoint(worldPoint, out) {
        return b2Transform.MulTXV(this.m_xf, worldPoint, out);
    }
    /// Gets a local vector given a world vector.
    /// @param a vector in world coordinates.
    /// @return the corresponding local vector.
    GetLocalVector(worldVector, out) {
        return b2Rot.MulTRV(this.m_xf.q, worldVector, out);
    }
    /// Get the world linear velocity of a world point attached to this body.
    /// @param a point in world coordinates.
    /// @return the world velocity of a point.
    GetLinearVelocityFromWorldPoint(worldPoint, out) {
        return b2Vec2.AddVCrossSV(this.m_linearVelocity, this.m_angularVelocity, b2Vec2.SubVV(worldPoint, this.m_sweep.c, b2Vec2.s_t0), out);
    }
    /// Get the world velocity of a local point.
    /// @param a point in local coordinates.
    /// @return the world velocity of a point.
    GetLinearVelocityFromLocalPoint(localPoint, out) {
        return this.GetLinearVelocityFromWorldPoint(this.GetWorldPoint(localPoint, out), out);
    }
    /// Get the linear damping of the body.
    GetLinearDamping() {
        return this.m_linearDamping;
    }
    /// Set the linear damping of the body.
    SetLinearDamping(linearDamping) {
        this.m_linearDamping = linearDamping;
    }
    /// Get the angular damping of the body.
    GetAngularDamping() {
        return this.m_angularDamping;
    }
    /// Set the angular damping of the body.
    SetAngularDamping(angularDamping) {
        this.m_angularDamping = angularDamping;
    }
    /// Get the gravity scale of the body.
    GetGravityScale() {
        return this.m_gravityScale;
    }
    /// Set the gravity scale of the body.
    SetGravityScale(scale) {
        this.m_gravityScale = scale;
    }
    /// Set the type of this body. This may alter the mass and velocity.
    SetType(type) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        if (this.m_type === type) {
            return;
        }
        this.m_type = type;
        this.ResetMassData();
        if (this.m_type === 0 /* b2_staticBody */) {
            this.m_linearVelocity.SetZero();
            this.m_angularVelocity = 0;
            this.m_sweep.a0 = this.m_sweep.a;
            this.m_sweep.c0.Copy(this.m_sweep.c);
            this.SynchronizeFixtures();
        }
        this.SetAwake(true);
        this.m_force.SetZero();
        this.m_torque = 0;
        // Delete the attached contacts.
        let ce = this.m_contactList;
        while (ce) {
            const ce0 = ce;
            ce = ce.next;
            this.m_world.m_contactManager.Destroy(ce0.contact);
        }
        this.m_contactList = null;
        // Touch the proxies so that new contacts will be created (when appropriate)
        for (let f = this.m_fixtureList; f; f = f.m_next) {
            f.TouchProxies();
        }
    }
    /// Get the type of this body.
    GetType() {
        return this.m_type;
    }
    /// Should this body be treated like a bullet for continuous collision detection?
    SetBullet(flag) {
        this.m_bulletFlag = flag;
    }
    /// Is this body treated like a bullet for continuous collision detection?
    IsBullet() {
        return this.m_bulletFlag;
    }
    /// You can disable sleeping on this body. If you disable sleeping, the
    /// body will be woken.
    SetSleepingAllowed(flag) {
        this.m_autoSleepFlag = flag;
        if (!flag) {
            this.SetAwake(true);
        }
    }
    /// Is this body allowed to sleep
    IsSleepingAllowed() {
        return this.m_autoSleepFlag;
    }
    /// Set the sleep state of the body. A sleeping body has very
    /// low CPU cost.
    /// @param flag set to true to wake the body, false to put it to sleep.
    SetAwake(flag) {
        if (flag) {
            this.m_awakeFlag = true;
            this.m_sleepTime = 0;
        }
        else {
            this.m_awakeFlag = false;
            this.m_sleepTime = 0;
            this.m_linearVelocity.SetZero();
            this.m_angularVelocity = 0;
            this.m_force.SetZero();
            this.m_torque = 0;
        }
    }
    /// Get the sleeping state of this body.
    /// @return true if the body is sleeping.
    IsAwake() {
        return this.m_awakeFlag;
    }
    /// Set the active state of the body. An inactive body is not
    /// simulated and cannot be collided with or woken up.
    /// If you pass a flag of true, all fixtures will be added to the
    /// broad-phase.
    /// If you pass a flag of false, all fixtures will be removed from
    /// the broad-phase and all contacts will be destroyed.
    /// Fixtures and joints are otherwise unaffected. You may continue
    /// to create/destroy fixtures and joints on inactive bodies.
    /// Fixtures on an inactive body are implicitly inactive and will
    /// not participate in collisions, ray-casts, or queries.
    /// Joints connected to an inactive body are implicitly inactive.
    /// An inactive body is still owned by a b2World object and remains
    /// in the body list.
    SetActive(flag) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        if (flag === this.IsActive()) {
            return;
        }
        this.m_activeFlag = flag;
        if (flag) {
            // Create all proxies.
            for (let f = this.m_fixtureList; f; f = f.m_next) {
                f.CreateProxies();
            }
            // Contacts are created the next time step.
        }
        else {
            // Destroy all proxies.
            for (let f = this.m_fixtureList; f; f = f.m_next) {
                f.DestroyProxies();
            }
            // Destroy the attached contacts.
            let ce = this.m_contactList;
            while (ce) {
                const ce0 = ce;
                ce = ce.next;
                this.m_world.m_contactManager.Destroy(ce0.contact);
            }
            this.m_contactList = null;
        }
    }
    /// Get the active state of the body.
    IsActive() {
        return this.m_activeFlag;
    }
    /// Set this body to have fixed rotation. This causes the mass
    /// to be reset.
    SetFixedRotation(flag) {
        if (this.m_fixedRotationFlag === flag) {
            return;
        }
        this.m_fixedRotationFlag = flag;
        this.m_angularVelocity = 0;
        this.ResetMassData();
    }
    /// Does this body have fixed rotation?
    IsFixedRotation() {
        return this.m_fixedRotationFlag;
    }
    /// Get the list of all fixtures attached to this body.
    GetFixtureList() {
        return this.m_fixtureList;
    }
    /// Get the list of all joints attached to this body.
    GetJointList() {
        return this.m_jointList;
    }
    /// Get the list of all contacts attached to this body.
    /// @warning this list changes during the time step and you may
    /// miss some collisions if you don't use b2ContactListener.
    GetContactList() {
        return this.m_contactList;
    }
    /// Get the next body in the world's body list.
    GetNext() {
        return this.m_next;
    }
    /// Get the user data pointer that was provided in the body definition.
    GetUserData() {
        return this.m_userData;
    }
    /// Set the user data. Use this to store your application specific data.
    SetUserData(data) {
        this.m_userData = data;
    }
    /// Get the parent world of this body.
    GetWorld() {
        return this.m_world;
    }
    SynchronizeFixtures() {
        const xf1 = b2Body.SynchronizeFixtures_s_xf1;
        xf1.q.SetAngle(this.m_sweep.a0);
        b2Rot.MulRV(xf1.q, this.m_sweep.localCenter, xf1.p);
        b2Vec2.SubVV(this.m_sweep.c0, xf1.p, xf1.p);
        // const displacement: b2Vec2 = b2Vec2.SubVV(this.m_xf.p, xf1.p, b2Body.SynchronizeFixtures_s_displacement);
        const displacement = b2Vec2.SubVV(this.m_sweep.c, this.m_sweep.c0, b2Body.SynchronizeFixtures_s_displacement);
        for (let f = this.m_fixtureList; f; f = f.m_next) {
            f.SynchronizeProxies(xf1, this.m_xf, displacement);
        }
    }
    SynchronizeTransform() {
        this.m_xf.q.SetAngle(this.m_sweep.a);
        b2Rot.MulRV(this.m_xf.q, this.m_sweep.localCenter, this.m_xf.p);
        b2Vec2.SubVV(this.m_sweep.c, this.m_xf.p, this.m_xf.p);
    }
    // This is used to prevent connected bodies from colliding.
    // It may lie, depending on the collideConnected flag.
    ShouldCollide(other) {
        // At least one body should be dynamic or kinematic.
        if (this.m_type === 0 /* b2_staticBody */ && other.m_type === 0 /* b2_staticBody */) {
            return false;
        }
        return this.ShouldCollideConnected(other);
    }
    ShouldCollideConnected(other) {
        // Does a joint prevent collision?
        for (let jn = this.m_jointList; jn; jn = jn.next) {
            if (jn.other === other) {
                if (!jn.joint.m_collideConnected) {
                    return false;
                }
            }
        }
        return true;
    }
    Advance(alpha) {
        // Advance to the new safe time. This doesn't sync the broad-phase.
        this.m_sweep.Advance(alpha);
        this.m_sweep.c.Copy(this.m_sweep.c0);
        this.m_sweep.a = this.m_sweep.a0;
        this.m_xf.q.SetAngle(this.m_sweep.a);
        b2Rot.MulRV(this.m_xf.q, this.m_sweep.localCenter, this.m_xf.p);
        b2Vec2.SubVV(this.m_sweep.c, this.m_xf.p, this.m_xf.p);
    }
    // #if B2_ENABLE_CONTROLLER
    GetControllerList() {
        return this.m_controllerList;
    }
    GetControllerCount() {
        return this.m_controllerCount;
    }
}
/// Creates a fixture from a shape and attach it to this body.
/// This is a convenience function. Use b2FixtureDef if you need to set parameters
/// like friction, restitution, user data, or filtering.
/// If the density is non-zero, this function automatically updates the mass of the body.
/// @param shape the shape to be cloned.
/// @param density the shape density (set to zero for static bodies).
/// @warning This function is locked during callbacks.
b2Body.CreateFixtureShapeDensity_s_def = new b2FixtureDef();
/// Set the mass properties to override the mass properties of the fixtures.
/// Note that this changes the center of mass position.
/// Note that creating or destroying fixtures can also alter the mass.
/// This function has no effect if the body isn't dynamic.
/// @param massData the mass properties.
b2Body.SetMassData_s_oldCenter = new b2Vec2();
/// This resets the mass properties to the sum of the mass properties of the fixtures.
/// This normally does not need to be called unless you called SetMassData to override
/// the mass and you later want to reset the mass.
b2Body.ResetMassData_s_localCenter = new b2Vec2();
b2Body.ResetMassData_s_oldCenter = new b2Vec2();
b2Body.ResetMassData_s_massData = new b2MassData();
b2Body.SynchronizeFixtures_s_xf1 = new b2Transform();
b2Body.SynchronizeFixtures_s_displacement = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJCb2R5LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL2R5bmFtaWNzL2IyQm9keS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFBRSxRQUFRLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUNoRCxPQUFPLEVBQUUsU0FBUyxFQUFFLEtBQUssRUFBRSxPQUFPLEVBQUUsV0FBVyxFQUFFLE1BQU0sRUFBTSxNQUFNLGtCQUFrQixDQUFDO0FBQ3RGLE9BQU8sRUFBRSxVQUFVLEVBQUUsT0FBTyxFQUFFLE1BQU0sNkJBQTZCLENBQUM7QUFHbEUsT0FBTyxFQUFFLFNBQVMsRUFBRSxZQUFZLEVBQWlCLE1BQU0sYUFBYSxDQUFDO0FBMEVyRSwwRUFBMEU7QUFDMUUsMEZBQTBGO0FBQzFGLE1BQU0sT0FBTyxTQUFTO0lBcURwQjtRQXBEQSxpREFBaUQ7UUFDakQseUVBQXlFO1FBQ3pFLFNBQUkseUJBQXdDO1FBRTVDLHVFQUF1RTtRQUN2RSxtREFBbUQ7UUFDMUMsYUFBUSxHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7UUFFakMsMkNBQTJDO1FBQzNDLFVBQUssR0FBRyxHQUFHLENBQUM7UUFFWixtRUFBbUU7UUFDMUQsbUJBQWMsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBRXZDLHFDQUFxQztRQUNyQyxvQkFBZSxHQUFHLEdBQUcsQ0FBQztRQUV0Qiw4RUFBOEU7UUFDOUUsMkVBQTJFO1FBQzNFLGtEQUFrRDtRQUNsRCxrQkFBYSxHQUFHLEdBQUcsQ0FBQztRQUVwQixnRkFBZ0Y7UUFDaEYsMkVBQTJFO1FBQzNFLGtEQUFrRDtRQUNsRCxtQkFBYyxHQUFHLEdBQUcsQ0FBQztRQUVyQiwyRUFBMkU7UUFDM0UsNkJBQTZCO1FBQzdCLGVBQVUsR0FBRyxJQUFJLENBQUM7UUFFbEIsNkNBQTZDO1FBQzdDLFVBQUssR0FBRyxJQUFJLENBQUM7UUFFYix1RUFBdUU7UUFD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LENBQUMsRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsQ0FBQyxDQUFDO0lBQ3pELENBQUM7SUFFRCwyQkFBMkI7SUFDM0IsaUJBQWlCO1FBQ2YsT0FBTyxJQUFJLENBQUMsZ0JBQWdCLENBQUM7SUFDL0IsQ0FBQztJQUVELGtCQUFrQjtRQUNoQixPQUFPLElBQUksQ0FBQyxpQkFBaUIsQ0FBQztJQUNoQyxDQUFDOztBQXQwQkQsOERBQThEO0FBQzlELGtGQUFrRjtBQUNsRix3REFBd0Q7QUFDeEQseUZBQXlGO0FBQ3pGLHdDQUF3QztBQUN4QyxxRUFBcUU7QUFDckUsc0RBQXNEO0FBQ3ZDLHNDQUErQixHQUFpQixJQUFJLFlBQVksRUFBRSxDQUFDO0FBMldsRiw0RUFBNEU7QUFDNUUsdURBQXVEO0FBQ3ZELHNFQUFzRTtBQUN0RSwwREFBMEQ7QUFDMUQsd0NBQXdDO0FBQ3pCLDhCQUF1QixHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7QUEyQzlELHNGQUFzRjtBQUN0RixzRkFBc0Y7QUFDdEYsa0RBQWtEO0FBQ25DLGtDQUEyQixHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7QUFDbkQsZ0NBQXlCLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNqRCwrQkFBd0IsR0FBZSxJQUFJLFVBQVUsRUFBRSxDQUFDO0FBNFZ4RCxnQ0FBeUIsR0FBZ0IsSUFBSSxXQUFXLEVBQUUsQ0FBQztBQUMzRCx5Q0FBa0MsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDIiwic291cmNlc0NvbnRlbnQiOlsiLypcclxuICogQ29weXJpZ2h0IChjKSAyMDA2LTIwMTEgRXJpbiBDYXR0byBodHRwOi8vd3d3LmJveDJkLm9yZ1xyXG4gKlxyXG4gKiBUaGlzIHNvZnR3YXJlIGlzIHByb3ZpZGVkICdhcy1pcycsIHdpdGhvdXQgYW55IGV4cHJlc3Mgb3IgaW1wbGllZFxyXG4gKiB3YXJyYW50eS4gIEluIG5vIGV2ZW50IHdpbGwgdGhlIGF1dGhvcnMgYmUgaGVsZCBsaWFibGUgZm9yIGFueSBkYW1hZ2VzXHJcbiAqIGFyaXNpbmcgZnJvbSB0aGUgdXNlIG9mIHRoaXMgc29mdHdhcmUuXHJcbiAqIFBlcm1pc3Npb24gaXMgZ3JhbnRlZCB0byBhbnlvbmUgdG8gdXNlIHRoaXMgc29mdHdhcmUgZm9yIGFueSBwdXJwb3NlLFxyXG4gKiBpbmNsdWRpbmcgY29tbWVyY2lhbCBhcHBsaWNhdGlvbnMsIGFuZCB0byBhbHRlciBpdCBhbmQgcmVkaXN0cmlidXRlIGl0XHJcbiAqIGZyZWVseSwgc3ViamVjdCB0byB0aGUgZm9sbG93aW5nIHJlc3RyaWN0aW9uczpcclxuICogMS4gVGhlIG9yaWdpbiBvZiB0aGlzIHNvZnR3YXJlIG11c3Qgbm90IGJlIG1pc3JlcHJlc2VudGVkOyB5b3UgbXVzdCBub3RcclxuICogY2xhaW0gdGhhdCB5b3Ugd3JvdGUgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLiBJZiB5b3UgdXNlIHRoaXMgc29mdHdhcmVcclxuICogaW4gYSBwcm9kdWN0LCBhbiBhY2tub3dsZWRnbWVudCBpbiB0aGUgcHJvZHVjdCBkb2N1bWVudGF0aW9uIHdvdWxkIGJlXHJcbiAqIGFwcHJlY2lhdGVkIGJ1dCBpcyBub3QgcmVxdWlyZWQuXHJcbiAqIDIuIEFsdGVyZWQgc291cmNlIHZlcnNpb25zIG11c3QgYmUgcGxhaW5seSBtYXJrZWQgYXMgc3VjaCwgYW5kIG11c3Qgbm90IGJlXHJcbiAqIG1pc3JlcHJlc2VudGVkIGFzIGJlaW5nIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS5cclxuICogMy4gVGhpcyBub3RpY2UgbWF5IG5vdCBiZSByZW1vdmVkIG9yIGFsdGVyZWQgZnJvbSBhbnkgc291cmNlIGRpc3RyaWJ1dGlvbi5cclxuICovXHJcblxyXG5pbXBvcnQgeyBiMkFzc2VydCB9IGZyb20gJy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJJc1ZhbGlkLCBiMlJvdCwgYjJTd2VlcCwgYjJUcmFuc2Zvcm0sIGIyVmVjMiwgWFkgfSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJNYXNzRGF0YSwgYjJTaGFwZSB9IGZyb20gJy4uL2NvbGxpc2lvbi9zaGFwZXMvYjJTaGFwZSc7XHJcbmltcG9ydCB7IGIyQ29udGFjdEVkZ2UgfSBmcm9tICcuL2NvbnRhY3RzL2IyQ29udGFjdCc7XHJcbmltcG9ydCB7IGIySm9pbnRFZGdlIH0gZnJvbSAnLi9qb2ludHMvYjJKb2ludCc7XHJcbmltcG9ydCB7IGIyRml4dHVyZSwgYjJGaXh0dXJlRGVmLCBiMklGaXh0dXJlRGVmIH0gZnJvbSAnLi9iMkZpeHR1cmUnO1xyXG5pbXBvcnQgeyBiMldvcmxkIH0gZnJvbSAnLi9iMldvcmxkJztcclxuaW1wb3J0IHsgYjJDb250cm9sbGVyRWRnZSB9IGZyb20gJy4uL2NvbnRyb2xsZXJzL2IyQ29udHJvbGxlcic7XHJcblxyXG4vLy8gVGhlIGJvZHkgdHlwZS5cclxuLy8vIHN0YXRpYzogemVybyBtYXNzLCB6ZXJvIHZlbG9jaXR5LCBtYXkgYmUgbWFudWFsbHkgbW92ZWRcclxuLy8vIGtpbmVtYXRpYzogemVybyBtYXNzLCBub24temVybyB2ZWxvY2l0eSBzZXQgYnkgdXNlciwgbW92ZWQgYnkgc29sdmVyXHJcbi8vLyBkeW5hbWljOiBwb3NpdGl2ZSBtYXNzLCBub24temVybyB2ZWxvY2l0eSBkZXRlcm1pbmVkIGJ5IGZvcmNlcywgbW92ZWQgYnkgc29sdmVyXHJcbmV4cG9ydCBjb25zdCBlbnVtIGIyQm9keVR5cGUge1xyXG4gIGIyX3Vua25vd24gPSAtMSxcclxuICBiMl9zdGF0aWNCb2R5ID0gMCxcclxuICBiMl9raW5lbWF0aWNCb2R5ID0gMSxcclxuICBiMl9keW5hbWljQm9keSA9IDIsXHJcblxyXG4gIC8vIFRPRE9fRVJJTlxyXG4gIC8vIGIyX2J1bGxldEJvZHkgPSAzXHJcbn1cclxuXHJcbmV4cG9ydCBpbnRlcmZhY2UgYjJJQm9keURlZiB7XHJcbiAgLy8vIFRoZSBib2R5IHR5cGU6IHN0YXRpYywga2luZW1hdGljLCBvciBkeW5hbWljLlxyXG4gIC8vLyBOb3RlOiBpZiBhIGR5bmFtaWMgYm9keSB3b3VsZCBoYXZlIHplcm8gbWFzcywgdGhlIG1hc3MgaXMgc2V0IHRvIG9uZS5cclxuICB0eXBlPzogYjJCb2R5VHlwZTtcclxuXHJcbiAgLy8vIFRoZSB3b3JsZCBwb3NpdGlvbiBvZiB0aGUgYm9keS4gQXZvaWQgY3JlYXRpbmcgYm9kaWVzIGF0IHRoZSBvcmlnaW5cclxuICAvLy8gc2luY2UgdGhpcyBjYW4gbGVhZCB0byBtYW55IG92ZXJsYXBwaW5nIHNoYXBlcy5cclxuICBwb3NpdGlvbj86IFhZO1xyXG5cclxuICAvLy8gVGhlIHdvcmxkIGFuZ2xlIG9mIHRoZSBib2R5IGluIHJhZGlhbnMuXHJcbiAgYW5nbGU/OiBudW1iZXI7XHJcblxyXG4gIC8vLyBUaGUgbGluZWFyIHZlbG9jaXR5IG9mIHRoZSBib2R5J3Mgb3JpZ2luIGluIHdvcmxkIGNvLW9yZGluYXRlcy5cclxuICBsaW5lYXJWZWxvY2l0eT86IFhZO1xyXG5cclxuICAvLy8gVGhlIGFuZ3VsYXIgdmVsb2NpdHkgb2YgdGhlIGJvZHkuXHJcbiAgYW5ndWxhclZlbG9jaXR5PzogbnVtYmVyO1xyXG5cclxuICAvLy8gTGluZWFyIGRhbXBpbmcgaXMgdXNlIHRvIHJlZHVjZSB0aGUgbGluZWFyIHZlbG9jaXR5LiBUaGUgZGFtcGluZyBwYXJhbWV0ZXJcclxuICAvLy8gY2FuIGJlIGxhcmdlciB0aGFuIDEuMGYgYnV0IHRoZSBkYW1waW5nIGVmZmVjdCBiZWNvbWVzIHNlbnNpdGl2ZSB0byB0aGVcclxuICAvLy8gdGltZSBzdGVwIHdoZW4gdGhlIGRhbXBpbmcgcGFyYW1ldGVyIGlzIGxhcmdlLlxyXG4gIC8vLyBVbml0cyBhcmUgMS90aW1lXHJcbiAgbGluZWFyRGFtcGluZz86IG51bWJlcjtcclxuXHJcbiAgLy8vIEFuZ3VsYXIgZGFtcGluZyBpcyB1c2UgdG8gcmVkdWNlIHRoZSBhbmd1bGFyIHZlbG9jaXR5LiBUaGUgZGFtcGluZyBwYXJhbWV0ZXJcclxuICAvLy8gY2FuIGJlIGxhcmdlciB0aGFuIDEuMGYgYnV0IHRoZSBkYW1waW5nIGVmZmVjdCBiZWNvbWVzIHNlbnNpdGl2ZSB0byB0aGVcclxuICAvLy8gdGltZSBzdGVwIHdoZW4gdGhlIGRhbXBpbmcgcGFyYW1ldGVyIGlzIGxhcmdlLlxyXG4gIC8vLyBVbml0cyBhcmUgMS90aW1lXHJcbiAgYW5ndWxhckRhbXBpbmc/OiBudW1iZXI7XHJcblxyXG4gIC8vLyBTZXQgdGhpcyBmbGFnIHRvIGZhbHNlIGlmIHRoaXMgYm9keSBzaG91bGQgbmV2ZXIgZmFsbCBhc2xlZXAuIE5vdGUgdGhhdFxyXG4gIC8vLyB0aGlzIGluY3JlYXNlcyBDUFUgdXNhZ2UuXHJcbiAgYWxsb3dTbGVlcD86IGJvb2xlYW47XHJcblxyXG4gIC8vLyBJcyB0aGlzIGJvZHkgaW5pdGlhbGx5IGF3YWtlIG9yIHNsZWVwaW5nP1xyXG4gIGF3YWtlPzogYm9vbGVhbjtcclxuXHJcbiAgLy8vIFNob3VsZCB0aGlzIGJvZHkgYmUgcHJldmVudGVkIGZyb20gcm90YXRpbmc/IFVzZWZ1bCBmb3IgY2hhcmFjdGVycy5cclxuICBmaXhlZFJvdGF0aW9uPzogYm9vbGVhbjtcclxuXHJcbiAgLy8vIElzIHRoaXMgYSBmYXN0IG1vdmluZyBib2R5IHRoYXQgc2hvdWxkIGJlIHByZXZlbnRlZCBmcm9tIHR1bm5lbGluZyB0aHJvdWdoXHJcbiAgLy8vIG90aGVyIG1vdmluZyBib2RpZXM/IE5vdGUgdGhhdCBhbGwgYm9kaWVzIGFyZSBwcmV2ZW50ZWQgZnJvbSB0dW5uZWxpbmcgdGhyb3VnaFxyXG4gIC8vLyBraW5lbWF0aWMgYW5kIHN0YXRpYyBib2RpZXMuIFRoaXMgc2V0dGluZyBpcyBvbmx5IGNvbnNpZGVyZWQgb24gZHluYW1pYyBib2RpZXMuXHJcbiAgLy8vIEB3YXJuaW5nIFlvdSBzaG91bGQgdXNlIHRoaXMgZmxhZyBzcGFyaW5nbHkgc2luY2UgaXQgaW5jcmVhc2VzIHByb2Nlc3NpbmcgdGltZS5cclxuICBidWxsZXQ/OiBib29sZWFuO1xyXG5cclxuICAvLy8gRG9lcyB0aGlzIGJvZHkgc3RhcnQgb3V0IGFjdGl2ZT9cclxuICBhY3RpdmU/OiBib29sZWFuO1xyXG5cclxuICAvLy8gVXNlIHRoaXMgdG8gc3RvcmUgYXBwbGljYXRpb24gc3BlY2lmaWMgYm9keSBkYXRhLlxyXG4gIHVzZXJEYXRhPzogYW55O1xyXG5cclxuICAvLy8gU2NhbGUgdGhlIGdyYXZpdHkgYXBwbGllZCB0byB0aGlzIGJvZHkuXHJcbiAgZ3Jhdml0eVNjYWxlPzogbnVtYmVyO1xyXG59XHJcblxyXG4vLy8gQSBib2R5IGRlZmluaXRpb24gaG9sZHMgYWxsIHRoZSBkYXRhIG5lZWRlZCB0byBjb25zdHJ1Y3QgYSByaWdpZCBib2R5LlxyXG4vLy8gWW91IGNhbiBzYWZlbHkgcmUtdXNlIGJvZHkgZGVmaW5pdGlvbnMuIFNoYXBlcyBhcmUgYWRkZWQgdG8gYSBib2R5IGFmdGVyIGNvbnN0cnVjdGlvbi5cclxuZXhwb3J0IGNsYXNzIGIyQm9keURlZiBpbXBsZW1lbnRzIGIySUJvZHlEZWYge1xyXG4gIC8vLyBUaGUgYm9keSB0eXBlOiBzdGF0aWMsIGtpbmVtYXRpYywgb3IgZHluYW1pYy5cclxuICAvLy8gTm90ZTogaWYgYSBkeW5hbWljIGJvZHkgd291bGQgaGF2ZSB6ZXJvIG1hc3MsIHRoZSBtYXNzIGlzIHNldCB0byBvbmUuXHJcbiAgdHlwZTogYjJCb2R5VHlwZSA9IGIyQm9keVR5cGUuYjJfc3RhdGljQm9keTtcclxuXHJcbiAgLy8vIFRoZSB3b3JsZCBwb3NpdGlvbiBvZiB0aGUgYm9keS4gQXZvaWQgY3JlYXRpbmcgYm9kaWVzIGF0IHRoZSBvcmlnaW5cclxuICAvLy8gc2luY2UgdGhpcyBjYW4gbGVhZCB0byBtYW55IG92ZXJsYXBwaW5nIHNoYXBlcy5cclxuICByZWFkb25seSBwb3NpdGlvbiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgLy8vIFRoZSB3b3JsZCBhbmdsZSBvZiB0aGUgYm9keSBpbiByYWRpYW5zLlxyXG4gIGFuZ2xlID0gTmFOO1xyXG5cclxuICAvLy8gVGhlIGxpbmVhciB2ZWxvY2l0eSBvZiB0aGUgYm9keSdzIG9yaWdpbiBpbiB3b3JsZCBjby1vcmRpbmF0ZXMuXHJcbiAgcmVhZG9ubHkgbGluZWFyVmVsb2NpdHkgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIC8vLyBUaGUgYW5ndWxhciB2ZWxvY2l0eSBvZiB0aGUgYm9keS5cclxuICBhbmd1bGFyVmVsb2NpdHkgPSBOYU47XHJcblxyXG4gIC8vLyBMaW5lYXIgZGFtcGluZyBpcyB1c2UgdG8gcmVkdWNlIHRoZSBsaW5lYXIgdmVsb2NpdHkuIFRoZSBkYW1waW5nIHBhcmFtZXRlclxyXG4gIC8vLyBjYW4gYmUgbGFyZ2VyIHRoYW4gMS4wZiBidXQgdGhlIGRhbXBpbmcgZWZmZWN0IGJlY29tZXMgc2Vuc2l0aXZlIHRvIHRoZVxyXG4gIC8vLyB0aW1lIHN0ZXAgd2hlbiB0aGUgZGFtcGluZyBwYXJhbWV0ZXIgaXMgbGFyZ2UuXHJcbiAgbGluZWFyRGFtcGluZyA9IE5hTjtcclxuXHJcbiAgLy8vIEFuZ3VsYXIgZGFtcGluZyBpcyB1c2UgdG8gcmVkdWNlIHRoZSBhbmd1bGFyIHZlbG9jaXR5LiBUaGUgZGFtcGluZyBwYXJhbWV0ZXJcclxuICAvLy8gY2FuIGJlIGxhcmdlciB0aGFuIDEuMGYgYnV0IHRoZSBkYW1waW5nIGVmZmVjdCBiZWNvbWVzIHNlbnNpdGl2ZSB0byB0aGVcclxuICAvLy8gdGltZSBzdGVwIHdoZW4gdGhlIGRhbXBpbmcgcGFyYW1ldGVyIGlzIGxhcmdlLlxyXG4gIGFuZ3VsYXJEYW1waW5nID0gTmFOO1xyXG5cclxuICAvLy8gU2V0IHRoaXMgZmxhZyB0byBmYWxzZSBpZiB0aGlzIGJvZHkgc2hvdWxkIG5ldmVyIGZhbGwgYXNsZWVwLiBOb3RlIHRoYXRcclxuICAvLy8gdGhpcyBpbmNyZWFzZXMgQ1BVIHVzYWdlLlxyXG4gIGFsbG93U2xlZXAgPSB0cnVlO1xyXG5cclxuICAvLy8gSXMgdGhpcyBib2R5IGluaXRpYWxseSBhd2FrZSBvciBzbGVlcGluZz9cclxuICBhd2FrZSA9IHRydWU7XHJcblxyXG4gIC8vLyBTaG91bGQgdGhpcyBib2R5IGJlIHByZXZlbnRlZCBmcm9tIHJvdGF0aW5nPyBVc2VmdWwgZm9yIGNoYXJhY3RlcnMuXHJcbiAgZml4ZWRSb3RhdGlvbiA9IGZhbHNlO1xyXG5cclxuICAvLy8gSXMgdGhpcyBhIGZhc3QgbW92aW5nIGJvZHkgdGhhdCBzaG91bGQgYmUgcHJldmVudGVkIGZyb20gdHVubmVsaW5nIHRocm91Z2hcclxuICAvLy8gb3RoZXIgbW92aW5nIGJvZGllcz8gTm90ZSB0aGF0IGFsbCBib2RpZXMgYXJlIHByZXZlbnRlZCBmcm9tIHR1bm5lbGluZyB0aHJvdWdoXHJcbiAgLy8vIGtpbmVtYXRpYyBhbmQgc3RhdGljIGJvZGllcy4gVGhpcyBzZXR0aW5nIGlzIG9ubHkgY29uc2lkZXJlZCBvbiBkeW5hbWljIGJvZGllcy5cclxuICAvLy8gQHdhcm5pbmcgWW91IHNob3VsZCB1c2UgdGhpcyBmbGFnIHNwYXJpbmdseSBzaW5jZSBpdCBpbmNyZWFzZXMgcHJvY2Vzc2luZyB0aW1lLlxyXG4gIGJ1bGxldCA9IGZhbHNlO1xyXG5cclxuICAvLy8gRG9lcyB0aGlzIGJvZHkgc3RhcnQgb3V0IGFjdGl2ZT9cclxuICBhY3RpdmUgPSB0cnVlO1xyXG5cclxuICAvLy8gVXNlIHRoaXMgdG8gc3RvcmUgYXBwbGljYXRpb24gc3BlY2lmaWMgYm9keSBkYXRhLlxyXG4gIHVzZXJEYXRhOiBhbnkgPSBudWxsO1xyXG5cclxuICAvLy8gU2NhbGUgdGhlIGdyYXZpdHkgYXBwbGllZCB0byB0aGlzIGJvZHkuXHJcbiAgZ3Jhdml0eVNjYWxlID0gTmFOOyAvLyAxLjBcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLmFuZ2xlID0gMC4wO1xyXG4gICAgdGhpcy5hbmd1bGFyVmVsb2NpdHkgPSAwLjA7XHJcbiAgICB0aGlzLmxpbmVhckRhbXBpbmcgPSAwLjA7XHJcbiAgICB0aGlzLmFuZ3VsYXJEYW1waW5nID0gMC4wO1xyXG4gICAgdGhpcy5ncmF2aXR5U2NhbGUgPSAxLjA7XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gQSByaWdpZCBib2R5LiBUaGVzZSBhcmUgY3JlYXRlZCB2aWEgYjJXb3JsZDo6Q3JlYXRlQm9keS5cclxuZXhwb3J0IGNsYXNzIGIyQm9keSB7XHJcbiAgbV90eXBlOiBiMkJvZHlUeXBlO1xyXG5cclxuICBtX2lzbGFuZEZsYWcgPSBmYWxzZTtcclxuICBtX2F3YWtlRmxhZyA9IGZhbHNlO1xyXG4gIG1fYXV0b1NsZWVwRmxhZyA9IGZhbHNlO1xyXG4gIG1fYnVsbGV0RmxhZyA9IGZhbHNlO1xyXG4gIG1fZml4ZWRSb3RhdGlvbkZsYWcgPSBmYWxzZTtcclxuICBtX2FjdGl2ZUZsYWcgPSBmYWxzZTtcclxuICBtX3RvaUZsYWcgPSBmYWxzZTtcclxuXHJcbiAgbV9pc2xhbmRJbmRleCA9IDA7XHJcblxyXG4gIHJlYWRvbmx5IG1feGYgPSBuZXcgYjJUcmFuc2Zvcm0oKTsgLy8gdGhlIGJvZHkgb3JpZ2luIHRyYW5zZm9ybVxyXG4gIC8vICNpZiBCMl9FTkFCTEVfUEFSVElDTEVcclxuICByZWFkb25seSBtX3hmMCA9IG5ldyBiMlRyYW5zZm9ybSgpO1xyXG4gIC8vICNlbmRpZlxyXG4gIHJlYWRvbmx5IG1fc3dlZXAgPSBuZXcgYjJTd2VlcCgpOyAvLyB0aGUgc3dlcHQgbW90aW9uIGZvciBDQ0RcclxuXHJcbiAgcmVhZG9ubHkgbV9saW5lYXJWZWxvY2l0eSA9IG5ldyBiMlZlYzIoKTtcclxuICBtX2FuZ3VsYXJWZWxvY2l0eSA9IE5hTjtcclxuXHJcbiAgcmVhZG9ubHkgbV9mb3JjZSA9IG5ldyBiMlZlYzIoKTtcclxuICBtX3RvcnF1ZSA9IE5hTjtcclxuXHJcbiAgbV93b3JsZDogYjJXb3JsZDtcclxuICBtX3ByZXY6IGIyQm9keSB8IG51bGwgPSBudWxsO1xyXG4gIG1fbmV4dDogYjJCb2R5IHwgbnVsbCA9IG51bGw7XHJcblxyXG4gIG1fZml4dHVyZUxpc3Q6IGIyRml4dHVyZSB8IG51bGwgPSBudWxsO1xyXG4gIG1fZml4dHVyZUNvdW50ID0gMDtcclxuXHJcbiAgbV9qb2ludExpc3Q6IGIySm9pbnRFZGdlIHwgbnVsbCA9IG51bGw7XHJcbiAgbV9jb250YWN0TGlzdDogYjJDb250YWN0RWRnZSB8IG51bGwgPSBudWxsO1xyXG5cclxuICBtX21hc3MgPSBOYU47XHJcbiAgbV9pbnZNYXNzID0gTmFOO1xyXG5cclxuICAvLyBSb3RhdGlvbmFsIGluZXJ0aWEgYWJvdXQgdGhlIGNlbnRlciBvZiBtYXNzLlxyXG4gIG1fSSA9IE5hTjtcclxuICBtX2ludkkgPSBOYU47XHJcblxyXG4gIG1fbGluZWFyRGFtcGluZyA9IE5hTjtcclxuICBtX2FuZ3VsYXJEYW1waW5nID0gTmFOO1xyXG4gIG1fZ3Jhdml0eVNjYWxlID0gTmFOO1xyXG5cclxuICBtX3NsZWVwVGltZSA9IE5hTjtcclxuXHJcbiAgbV91c2VyRGF0YTogYW55ID0gbnVsbDtcclxuXHJcbiAgLy8gI2lmIEIyX0VOQUJMRV9DT05UUk9MTEVSXHJcbiAgbV9jb250cm9sbGVyTGlzdDogYjJDb250cm9sbGVyRWRnZSB8IG51bGwgPSBudWxsO1xyXG4gIG1fY29udHJvbGxlckNvdW50ID0gMDtcclxuICAvLyAjZW5kaWZcclxuXHJcbiAgY29uc3RydWN0b3IoYmQ6IGIySUJvZHlEZWYsIHdvcmxkOiBiMldvcmxkKSB7XHJcbiAgICB0aGlzLm1fYnVsbGV0RmxhZyA9IGJkLmJ1bGxldCA/PyBmYWxzZTtcclxuICAgIHRoaXMubV9maXhlZFJvdGF0aW9uRmxhZyA9IGJkLmZpeGVkUm90YXRpb24gPz8gZmFsc2U7XHJcbiAgICB0aGlzLm1fYXV0b1NsZWVwRmxhZyA9IGJkLmFsbG93U2xlZXAgPz8gdHJ1ZTtcclxuICAgIHRoaXMubV9hd2FrZUZsYWcgPSBiZC5hd2FrZSA/PyB0cnVlO1xyXG4gICAgdGhpcy5tX2FjdGl2ZUZsYWcgPSBiZC5hY3RpdmUgPz8gdHJ1ZTtcclxuXHJcbiAgICB0aGlzLm1fd29ybGQgPSB3b3JsZDtcclxuXHJcbiAgICB0aGlzLm1feGYucC5Db3B5KGJkLnBvc2l0aW9uID8/IGIyVmVjMi5aRVJPKTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX3hmLnAuSXNWYWxpZCgpKTtcclxuICAgIHRoaXMubV94Zi5xLlNldEFuZ2xlKGJkLmFuZ2xlID8/IDAuMCk7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGIySXNWYWxpZCh0aGlzLm1feGYucS5HZXRBbmdsZSgpKSk7XHJcbiAgICBpZiAoQjJfRU5BQkxFX1BBUlRJQ0xFKSB7XHJcbiAgICAgIHRoaXMubV94ZjAuQ29weSh0aGlzLm1feGYpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9zd2VlcC5sb2NhbENlbnRlci5TZXRaZXJvKCk7XHJcbiAgICB0aGlzLm1fc3dlZXAuYzAuQ29weSh0aGlzLm1feGYucCk7XHJcbiAgICB0aGlzLm1fc3dlZXAuYy5Db3B5KHRoaXMubV94Zi5wKTtcclxuICAgIHRoaXMubV9zd2VlcC5hMCA9IHRoaXMubV9zd2VlcC5hID0gdGhpcy5tX3hmLnEuR2V0QW5nbGUoKTtcclxuICAgIHRoaXMubV9zd2VlcC5hbHBoYTAgPSAwO1xyXG5cclxuICAgIHRoaXMubV9saW5lYXJWZWxvY2l0eS5Db3B5KGJkLmxpbmVhclZlbG9jaXR5ID8/IGIyVmVjMi5aRVJPKTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2xpbmVhclZlbG9jaXR5LklzVmFsaWQoKSk7XHJcbiAgICB0aGlzLm1fYW5ndWxhclZlbG9jaXR5ID0gYmQuYW5ndWxhclZlbG9jaXR5ID8/IDA7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGIySXNWYWxpZCh0aGlzLm1fYW5ndWxhclZlbG9jaXR5KSk7XHJcblxyXG4gICAgdGhpcy5tX2xpbmVhckRhbXBpbmcgPSBiZC5saW5lYXJEYW1waW5nID8/IDAuMDtcclxuICAgIHRoaXMubV9hbmd1bGFyRGFtcGluZyA9IGJkLmFuZ3VsYXJEYW1waW5nID8/IDAuMDtcclxuICAgIHRoaXMubV9ncmF2aXR5U2NhbGUgPSBiZC5ncmF2aXR5U2NhbGUgPz8gMS4wO1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChiMklzVmFsaWQodGhpcy5tX2dyYXZpdHlTY2FsZSkgJiYgdGhpcy5tX2dyYXZpdHlTY2FsZSA+PSAwKTtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoYjJJc1ZhbGlkKHRoaXMubV9hbmd1bGFyRGFtcGluZykgJiYgdGhpcy5tX2FuZ3VsYXJEYW1waW5nID49IDApO1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChiMklzVmFsaWQodGhpcy5tX2xpbmVhckRhbXBpbmcpICYmIHRoaXMubV9saW5lYXJEYW1waW5nID49IDApO1xyXG5cclxuICAgIHRoaXMubV9mb3JjZS5TZXRaZXJvKCk7XHJcbiAgICB0aGlzLm1fdG9ycXVlID0gMC4wO1xyXG5cclxuICAgIHRoaXMubV9zbGVlcFRpbWUgPSAwLjA7XHJcblxyXG4gICAgdGhpcy5tX3R5cGUgPSBiZC50eXBlID8/IGIyQm9keVR5cGUuYjJfc3RhdGljQm9keTtcclxuXHJcbiAgICBpZiAoYmQudHlwZSA9PT0gYjJCb2R5VHlwZS5iMl9keW5hbWljQm9keSkge1xyXG4gICAgICB0aGlzLm1fbWFzcyA9IDEuMDtcclxuICAgICAgdGhpcy5tX2ludk1hc3MgPSAxLjA7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICB0aGlzLm1fbWFzcyA9IDAuMDtcclxuICAgICAgdGhpcy5tX2ludk1hc3MgPSAwLjA7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5tX0kgPSAwLjA7XHJcbiAgICB0aGlzLm1faW52SSA9IDAuMDtcclxuXHJcbiAgICB0aGlzLm1fdXNlckRhdGEgPSBiZC51c2VyRGF0YTtcclxuICB9XHJcblxyXG4gIENyZWF0ZUZpeHR1cmUoZGVmOiBiMklGaXh0dXJlRGVmKTogYjJGaXh0dXJlO1xyXG4gIENyZWF0ZUZpeHR1cmUoc2hhcGU6IGIyU2hhcGUpOiBiMkZpeHR1cmU7XHJcbiAgQ3JlYXRlRml4dHVyZShzaGFwZTogYjJTaGFwZSwgZGVuc2l0eTogbnVtYmVyKTogYjJGaXh0dXJlO1xyXG4gIENyZWF0ZUZpeHR1cmUoYTogYjJJRml4dHVyZURlZiB8IGIyU2hhcGUsIGIgPSAwKTogYjJGaXh0dXJlIHtcclxuICAgIGlmIChhIGluc3RhbmNlb2YgYjJTaGFwZSkge1xyXG4gICAgICByZXR1cm4gdGhpcy5DcmVhdGVGaXh0dXJlU2hhcGVEZW5zaXR5KGEsIGIpO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgcmV0dXJuIHRoaXMuQ3JlYXRlRml4dHVyZURlZihhKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vLyBDcmVhdGVzIGEgZml4dHVyZSBhbmQgYXR0YWNoIGl0IHRvIHRoaXMgYm9keS4gVXNlIHRoaXMgZnVuY3Rpb24gaWYgeW91IG5lZWRcclxuICAvLy8gdG8gc2V0IHNvbWUgZml4dHVyZSBwYXJhbWV0ZXJzLCBsaWtlIGZyaWN0aW9uLiBPdGhlcndpc2UgeW91IGNhbiBjcmVhdGUgdGhlXHJcbiAgLy8vIGZpeHR1cmUgZGlyZWN0bHkgZnJvbSBhIHNoYXBlLlxyXG4gIC8vLyBJZiB0aGUgZGVuc2l0eSBpcyBub24temVybywgdGhpcyBmdW5jdGlvbiBhdXRvbWF0aWNhbGx5IHVwZGF0ZXMgdGhlIG1hc3Mgb2YgdGhlIGJvZHkuXHJcbiAgLy8vIENvbnRhY3RzIGFyZSBub3QgY3JlYXRlZCB1bnRpbCB0aGUgbmV4dCB0aW1lIHN0ZXAuXHJcbiAgLy8vIEBwYXJhbSBkZWYgdGhlIGZpeHR1cmUgZGVmaW5pdGlvbi5cclxuICAvLy8gQHdhcm5pbmcgVGhpcyBmdW5jdGlvbiBpcyBsb2NrZWQgZHVyaW5nIGNhbGxiYWNrcy5cclxuICBDcmVhdGVGaXh0dXJlRGVmKGRlZjogYjJJRml4dHVyZURlZik6IGIyRml4dHVyZSB7XHJcbiAgICBpZiAodGhpcy5tX3dvcmxkLklzTG9ja2VkKCkpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgZml4dHVyZTogYjJGaXh0dXJlID0gbmV3IGIyRml4dHVyZSh0aGlzLCBkZWYpO1xyXG5cclxuICAgIGlmICh0aGlzLm1fYWN0aXZlRmxhZykge1xyXG4gICAgICBmaXh0dXJlLkNyZWF0ZVByb3hpZXMoKTtcclxuICAgIH1cclxuXHJcbiAgICBmaXh0dXJlLm1fbmV4dCA9IHRoaXMubV9maXh0dXJlTGlzdDtcclxuICAgIHRoaXMubV9maXh0dXJlTGlzdCA9IGZpeHR1cmU7XHJcbiAgICArK3RoaXMubV9maXh0dXJlQ291bnQ7XHJcblxyXG4gICAgLy8gZml4dHVyZS5tX2JvZHkgPSB0aGlzO1xyXG5cclxuICAgIC8vIEFkanVzdCBtYXNzIHByb3BlcnRpZXMgaWYgbmVlZGVkLlxyXG4gICAgaWYgKGZpeHR1cmUubV9kZW5zaXR5ID4gMCkge1xyXG4gICAgICB0aGlzLlJlc2V0TWFzc0RhdGEoKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBMZXQgdGhlIHdvcmxkIGtub3cgd2UgaGF2ZSBhIG5ldyBmaXh0dXJlLiBUaGlzIHdpbGwgY2F1c2UgbmV3IGNvbnRhY3RzXHJcbiAgICAvLyB0byBiZSBjcmVhdGVkIGF0IHRoZSBiZWdpbm5pbmcgb2YgdGhlIG5leHQgdGltZSBzdGVwLlxyXG4gICAgdGhpcy5tX3dvcmxkLm1fbmV3Rml4dHVyZSA9IHRydWU7XHJcblxyXG4gICAgcmV0dXJuIGZpeHR1cmU7XHJcbiAgfVxyXG5cclxuICAvLy8gQ3JlYXRlcyBhIGZpeHR1cmUgZnJvbSBhIHNoYXBlIGFuZCBhdHRhY2ggaXQgdG8gdGhpcyBib2R5LlxyXG4gIC8vLyBUaGlzIGlzIGEgY29udmVuaWVuY2UgZnVuY3Rpb24uIFVzZSBiMkZpeHR1cmVEZWYgaWYgeW91IG5lZWQgdG8gc2V0IHBhcmFtZXRlcnNcclxuICAvLy8gbGlrZSBmcmljdGlvbiwgcmVzdGl0dXRpb24sIHVzZXIgZGF0YSwgb3IgZmlsdGVyaW5nLlxyXG4gIC8vLyBJZiB0aGUgZGVuc2l0eSBpcyBub24temVybywgdGhpcyBmdW5jdGlvbiBhdXRvbWF0aWNhbGx5IHVwZGF0ZXMgdGhlIG1hc3Mgb2YgdGhlIGJvZHkuXHJcbiAgLy8vIEBwYXJhbSBzaGFwZSB0aGUgc2hhcGUgdG8gYmUgY2xvbmVkLlxyXG4gIC8vLyBAcGFyYW0gZGVuc2l0eSB0aGUgc2hhcGUgZGVuc2l0eSAoc2V0IHRvIHplcm8gZm9yIHN0YXRpYyBib2RpZXMpLlxyXG4gIC8vLyBAd2FybmluZyBUaGlzIGZ1bmN0aW9uIGlzIGxvY2tlZCBkdXJpbmcgY2FsbGJhY2tzLlxyXG4gIHByaXZhdGUgc3RhdGljIENyZWF0ZUZpeHR1cmVTaGFwZURlbnNpdHlfc19kZWY6IGIyRml4dHVyZURlZiA9IG5ldyBiMkZpeHR1cmVEZWYoKTtcclxuXHJcbiAgQ3JlYXRlRml4dHVyZVNoYXBlRGVuc2l0eShzaGFwZTogYjJTaGFwZSwgZGVuc2l0eSA9IDApOiBiMkZpeHR1cmUge1xyXG4gICAgY29uc3QgZGVmOiBiMkZpeHR1cmVEZWYgPSBiMkJvZHkuQ3JlYXRlRml4dHVyZVNoYXBlRGVuc2l0eV9zX2RlZjtcclxuICAgIGRlZi5zaGFwZSA9IHNoYXBlO1xyXG4gICAgZGVmLmRlbnNpdHkgPSBkZW5zaXR5O1xyXG4gICAgcmV0dXJuIHRoaXMuQ3JlYXRlRml4dHVyZURlZihkZWYpO1xyXG4gIH1cclxuXHJcbiAgLy8vIERlc3Ryb3kgYSBmaXh0dXJlLiBUaGlzIHJlbW92ZXMgdGhlIGZpeHR1cmUgZnJvbSB0aGUgYnJvYWQtcGhhc2UgYW5kXHJcbiAgLy8vIGRlc3Ryb3lzIGFsbCBjb250YWN0cyBhc3NvY2lhdGVkIHdpdGggdGhpcyBmaXh0dXJlLiBUaGlzIHdpbGxcclxuICAvLy8gYXV0b21hdGljYWxseSBhZGp1c3QgdGhlIG1hc3Mgb2YgdGhlIGJvZHkgaWYgdGhlIGJvZHkgaXMgZHluYW1pYyBhbmQgdGhlXHJcbiAgLy8vIGZpeHR1cmUgaGFzIHBvc2l0aXZlIGRlbnNpdHkuXHJcbiAgLy8vIEFsbCBmaXh0dXJlcyBhdHRhY2hlZCB0byBhIGJvZHkgYXJlIGltcGxpY2l0bHkgZGVzdHJveWVkIHdoZW4gdGhlIGJvZHkgaXMgZGVzdHJveWVkLlxyXG4gIC8vLyBAcGFyYW0gZml4dHVyZSB0aGUgZml4dHVyZSB0byBiZSByZW1vdmVkLlxyXG4gIC8vLyBAd2FybmluZyBUaGlzIGZ1bmN0aW9uIGlzIGxvY2tlZCBkdXJpbmcgY2FsbGJhY2tzLlxyXG4gIERlc3Ryb3lGaXh0dXJlKGZpeHR1cmU6IGIyRml4dHVyZSk6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMubV93b3JsZC5Jc0xvY2tlZCgpKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG5cclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoZml4dHVyZS5tX2JvZHkgPT09IHRoaXMpO1xyXG5cclxuICAgIC8vIFJlbW92ZSB0aGUgZml4dHVyZSBmcm9tIHRoaXMgYm9keSdzIHNpbmdseSBsaW5rZWQgbGlzdC5cclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2ZpeHR1cmVDb3VudCA+IDApO1xyXG4gICAgbGV0IG5vZGU6IGIyRml4dHVyZSB8IG51bGwgPSB0aGlzLm1fZml4dHVyZUxpc3Q7XHJcbiAgICBsZXQgcHBGOiBiMkZpeHR1cmUgfCBudWxsID0gbnVsbDtcclxuICAgIC8vIERFQlVHOiBsZXQgZm91bmQ6IGJvb2xlYW4gPSBmYWxzZTtcclxuICAgIHdoaWxlIChub2RlICE9PSBudWxsKSB7XHJcbiAgICAgIGlmIChub2RlID09PSBmaXh0dXJlKSB7XHJcbiAgICAgICAgaWYgKHBwRikge1xyXG4gICAgICAgICAgcHBGLm1fbmV4dCA9IGZpeHR1cmUubV9uZXh0O1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICB0aGlzLm1fZml4dHVyZUxpc3QgPSBmaXh0dXJlLm1fbmV4dDtcclxuICAgICAgICB9XHJcbiAgICAgICAgLy8gREVCVUc6IGZvdW5kID0gdHJ1ZTtcclxuICAgICAgICBicmVhaztcclxuICAgICAgfVxyXG5cclxuICAgICAgcHBGID0gbm9kZTtcclxuICAgICAgbm9kZSA9IG5vZGUubV9uZXh0O1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFlvdSB0cmllZCB0byByZW1vdmUgYSBzaGFwZSB0aGF0IGlzIG5vdCBhdHRhY2hlZCB0byB0aGlzIGJvZHkuXHJcbiAgICAvLyBUT0RPOiBkZWJ1Z1xyXG4gICAgLy8hIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGZvdW5kKTtcclxuXHJcbiAgICAvLyBEZXN0cm95IGFueSBjb250YWN0cyBhc3NvY2lhdGVkIHdpdGggdGhlIGZpeHR1cmUuXHJcbiAgICBsZXQgZWRnZTogYjJDb250YWN0RWRnZSB8IG51bGwgPSB0aGlzLm1fY29udGFjdExpc3Q7XHJcbiAgICB3aGlsZSAoZWRnZSkge1xyXG4gICAgICBjb25zdCBjID0gZWRnZS5jb250YWN0O1xyXG4gICAgICBlZGdlID0gZWRnZS5uZXh0O1xyXG5cclxuICAgICAgY29uc3QgZml4dHVyZUE6IGIyRml4dHVyZSA9IGMuR2V0Rml4dHVyZUEoKTtcclxuICAgICAgY29uc3QgZml4dHVyZUI6IGIyRml4dHVyZSA9IGMuR2V0Rml4dHVyZUIoKTtcclxuXHJcbiAgICAgIGlmIChmaXh0dXJlID09PSBmaXh0dXJlQSB8fCBmaXh0dXJlID09PSBmaXh0dXJlQikge1xyXG4gICAgICAgIC8vIFRoaXMgZGVzdHJveXMgdGhlIGNvbnRhY3QgYW5kIHJlbW92ZXMgaXQgZnJvbVxyXG4gICAgICAgIC8vIHRoaXMgYm9keSdzIGNvbnRhY3QgbGlzdC5cclxuICAgICAgICB0aGlzLm1fd29ybGQubV9jb250YWN0TWFuYWdlci5EZXN0cm95KGMpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHRoaXMubV9hY3RpdmVGbGFnKSB7XHJcbiAgICAgIGZpeHR1cmUuRGVzdHJveVByb3hpZXMoKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBmaXh0dXJlLm1fYm9keSA9IG51bGw7XHJcbiAgICBmaXh0dXJlLm1fbmV4dCA9IG51bGw7XHJcbiAgICBmaXh0dXJlLlJlc2V0KCk7XHJcblxyXG4gICAgLS10aGlzLm1fZml4dHVyZUNvdW50O1xyXG5cclxuICAgIC8vIFJlc2V0IHRoZSBtYXNzIGRhdGEuXHJcbiAgICB0aGlzLlJlc2V0TWFzc0RhdGEoKTtcclxuICB9XHJcblxyXG4gIC8vLyBTZXQgdGhlIHBvc2l0aW9uIG9mIHRoZSBib2R5J3Mgb3JpZ2luIGFuZCByb3RhdGlvbi5cclxuICAvLy8gVGhpcyBicmVha3MgYW55IGNvbnRhY3RzIGFuZCB3YWtlcyB0aGUgb3RoZXIgYm9kaWVzLlxyXG4gIC8vLyBNYW5pcHVsYXRpbmcgYSBib2R5J3MgdHJhbnNmb3JtIG1heSBjYXVzZSBub24tcGh5c2ljYWwgYmVoYXZpb3IuXHJcbiAgLy8vIEBwYXJhbSBwb3NpdGlvbiB0aGUgd29ybGQgcG9zaXRpb24gb2YgdGhlIGJvZHkncyBsb2NhbCBvcmlnaW4uXHJcbiAgLy8vIEBwYXJhbSBhbmdsZSB0aGUgd29ybGQgcm90YXRpb24gaW4gcmFkaWFucy5cclxuICBTZXRUcmFuc2Zvcm1WZWMocG9zaXRpb246IFhZLCBhbmdsZTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLlNldFRyYW5zZm9ybVhZKHBvc2l0aW9uLngsIHBvc2l0aW9uLnksIGFuZ2xlKTtcclxuICB9XHJcblxyXG4gIFNldFRyYW5zZm9ybVhZKHg6IG51bWJlciwgeTogbnVtYmVyLCBhbmdsZTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICBpZiAodGhpcy5tX3dvcmxkLklzTG9ja2VkKCkpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5tX3hmLnEuU2V0QW5nbGUoYW5nbGUpO1xyXG4gICAgdGhpcy5tX3hmLnAuU2V0KHgsIHkpO1xyXG4gICAgaWYgKEIyX0VOQUJMRV9QQVJUSUNMRSkge1xyXG4gICAgICB0aGlzLm1feGYwLkNvcHkodGhpcy5tX3hmKTtcclxuICAgIH1cclxuXHJcbiAgICBiMlRyYW5zZm9ybS5NdWxYVih0aGlzLm1feGYsIHRoaXMubV9zd2VlcC5sb2NhbENlbnRlciwgdGhpcy5tX3N3ZWVwLmMpO1xyXG4gICAgdGhpcy5tX3N3ZWVwLmEgPSBhbmdsZTtcclxuXHJcbiAgICB0aGlzLm1fc3dlZXAuYzAuQ29weSh0aGlzLm1fc3dlZXAuYyk7XHJcbiAgICB0aGlzLm1fc3dlZXAuYTAgPSBhbmdsZTtcclxuXHJcbiAgICBmb3IgKGxldCBmOiBiMkZpeHR1cmUgfCBudWxsID0gdGhpcy5tX2ZpeHR1cmVMaXN0OyBmOyBmID0gZi5tX25leHQpIHtcclxuICAgICAgZi5TeW5jaHJvbml6ZVByb3hpZXModGhpcy5tX3hmLCB0aGlzLm1feGYsIGIyVmVjMi5aRVJPKTtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLm1fd29ybGQubV9jb250YWN0TWFuYWdlci5GaW5kTmV3Q29udGFjdHMoKTtcclxuICB9XHJcblxyXG4gIFNldFRyYW5zZm9ybSh4ZjogYjJUcmFuc2Zvcm0pOiB2b2lkIHtcclxuICAgIHRoaXMuU2V0VHJhbnNmb3JtVmVjKHhmLnAsIHhmLkdldEFuZ2xlKCkpO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgYm9keSB0cmFuc2Zvcm0gZm9yIHRoZSBib2R5J3Mgb3JpZ2luLlxyXG4gIC8vLyBAcmV0dXJuIHRoZSB3b3JsZCB0cmFuc2Zvcm0gb2YgdGhlIGJvZHkncyBvcmlnaW4uXHJcbiAgR2V0VHJhbnNmb3JtKCk6IFJlYWRvbmx5PGIyVHJhbnNmb3JtPiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3hmO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgd29ybGQgYm9keSBvcmlnaW4gcG9zaXRpb24uXHJcbiAgLy8vIEByZXR1cm4gdGhlIHdvcmxkIHBvc2l0aW9uIG9mIHRoZSBib2R5J3Mgb3JpZ2luLlxyXG4gIEdldFBvc2l0aW9uKCk6IFJlYWRvbmx5PGIyVmVjMj4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV94Zi5wO1xyXG4gIH1cclxuXHJcbiAgU2V0UG9zaXRpb24ocG9zaXRpb246IFhZKTogdm9pZCB7XHJcbiAgICB0aGlzLlNldFRyYW5zZm9ybVZlYyhwb3NpdGlvbiwgdGhpcy5HZXRBbmdsZSgpKTtcclxuICB9XHJcblxyXG4gIFNldFBvc2l0aW9uWFkoeDogbnVtYmVyLCB5OiBudW1iZXIpOiB2b2lkIHtcclxuICAgIHRoaXMuU2V0VHJhbnNmb3JtWFkoeCwgeSwgdGhpcy5HZXRBbmdsZSgpKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGFuZ2xlIGluIHJhZGlhbnMuXHJcbiAgLy8vIEByZXR1cm4gdGhlIGN1cnJlbnQgd29ybGQgcm90YXRpb24gYW5nbGUgaW4gcmFkaWFucy5cclxuICBHZXRBbmdsZSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9zd2VlcC5hO1xyXG4gIH1cclxuXHJcbiAgU2V0QW5nbGUoYW5nbGU6IG51bWJlcik6IHZvaWQge1xyXG4gICAgdGhpcy5TZXRUcmFuc2Zvcm1WZWModGhpcy5HZXRQb3NpdGlvbigpLCBhbmdsZSk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSB3b3JsZCBwb3NpdGlvbiBvZiB0aGUgY2VudGVyIG9mIG1hc3MuXHJcbiAgR2V0V29ybGRDZW50ZXIoKTogUmVhZG9ubHk8YjJWZWMyPiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3N3ZWVwLmM7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBsb2NhbCBwb3NpdGlvbiBvZiB0aGUgY2VudGVyIG9mIG1hc3MuXHJcbiAgR2V0TG9jYWxDZW50ZXIoKTogUmVhZG9ubHk8YjJWZWMyPiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3N3ZWVwLmxvY2FsQ2VudGVyO1xyXG4gIH1cclxuXHJcbiAgLy8vIFNldCB0aGUgbGluZWFyIHZlbG9jaXR5IG9mIHRoZSBjZW50ZXIgb2YgbWFzcy5cclxuICAvLy8gQHBhcmFtIHYgdGhlIG5ldyBsaW5lYXIgdmVsb2NpdHkgb2YgdGhlIGNlbnRlciBvZiBtYXNzLlxyXG4gIFNldExpbmVhclZlbG9jaXR5KHY6IFhZKTogdm9pZCB7XHJcbiAgICB0aGlzLlNldExpbmVhclZlbG9jaXR5WFkodi54LCB2LnkpO1xyXG4gIH1cclxuXHJcbiAgU2V0TGluZWFyVmVsb2NpdHlYWSh4OiBudW1iZXIsIHk6IG51bWJlcik6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMubV90eXBlID09PSBiMkJvZHlUeXBlLmIyX3N0YXRpY0JvZHkpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIGlmICh4ICogeCArIHkgKiB5ID4gMCkge1xyXG4gICAgICB0aGlzLlNldEF3YWtlKHRydWUpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9saW5lYXJWZWxvY2l0eS5TZXQoeCwgeSk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBsaW5lYXIgdmVsb2NpdHkgb2YgdGhlIGNlbnRlciBvZiBtYXNzLlxyXG4gIC8vLyBAcmV0dXJuIHRoZSBsaW5lYXIgdmVsb2NpdHkgb2YgdGhlIGNlbnRlciBvZiBtYXNzLlxyXG4gIEdldExpbmVhclZlbG9jaXR5KCk6IFJlYWRvbmx5PGIyVmVjMj4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9saW5lYXJWZWxvY2l0eTtcclxuICB9XHJcblxyXG4gIC8vLyBTZXQgdGhlIGFuZ3VsYXIgdmVsb2NpdHkuXHJcbiAgLy8vIEBwYXJhbSBvbWVnYSB0aGUgbmV3IGFuZ3VsYXIgdmVsb2NpdHkgaW4gcmFkaWFucy9zZWNvbmQuXHJcbiAgU2V0QW5ndWxhclZlbG9jaXR5KHc6IG51bWJlcik6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMubV90eXBlID09PSBiMkJvZHlUeXBlLmIyX3N0YXRpY0JvZHkpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIGlmICh3ICogdyA+IDApIHtcclxuICAgICAgdGhpcy5TZXRBd2FrZSh0cnVlKTtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLm1fYW5ndWxhclZlbG9jaXR5ID0gdztcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGFuZ3VsYXIgdmVsb2NpdHkuXHJcbiAgLy8vIEByZXR1cm4gdGhlIGFuZ3VsYXIgdmVsb2NpdHkgaW4gcmFkaWFucy9zZWNvbmQuXHJcbiAgR2V0QW5ndWxhclZlbG9jaXR5KCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2FuZ3VsYXJWZWxvY2l0eTtcclxuICB9XHJcblxyXG4gIEdldERlZmluaXRpb24oYmQ6IGIyQm9keURlZik6IGIyQm9keURlZiB7XHJcbiAgICBiZC50eXBlID0gdGhpcy5HZXRUeXBlKCk7XHJcbiAgICBiZC5hbGxvd1NsZWVwID0gdGhpcy5tX2F1dG9TbGVlcEZsYWc7XHJcbiAgICBiZC5hbmdsZSA9IHRoaXMuR2V0QW5nbGUoKTtcclxuICAgIGJkLmFuZ3VsYXJEYW1waW5nID0gdGhpcy5tX2FuZ3VsYXJEYW1waW5nO1xyXG4gICAgYmQuZ3Jhdml0eVNjYWxlID0gdGhpcy5tX2dyYXZpdHlTY2FsZTtcclxuICAgIGJkLmFuZ3VsYXJWZWxvY2l0eSA9IHRoaXMubV9hbmd1bGFyVmVsb2NpdHk7XHJcbiAgICBiZC5maXhlZFJvdGF0aW9uID0gdGhpcy5tX2ZpeGVkUm90YXRpb25GbGFnO1xyXG4gICAgYmQuYnVsbGV0ID0gdGhpcy5tX2J1bGxldEZsYWc7XHJcbiAgICBiZC5hd2FrZSA9IHRoaXMubV9hd2FrZUZsYWc7XHJcbiAgICBiZC5saW5lYXJEYW1waW5nID0gdGhpcy5tX2xpbmVhckRhbXBpbmc7XHJcbiAgICBiZC5saW5lYXJWZWxvY2l0eS5Db3B5KHRoaXMuR2V0TGluZWFyVmVsb2NpdHkoKSk7XHJcbiAgICBiZC5wb3NpdGlvbi5Db3B5KHRoaXMuR2V0UG9zaXRpb24oKSk7XHJcbiAgICBiZC51c2VyRGF0YSA9IHRoaXMuR2V0VXNlckRhdGEoKTtcclxuICAgIHJldHVybiBiZDtcclxuICB9XHJcblxyXG4gIC8vLyBBcHBseSBhIGZvcmNlIGF0IGEgd29ybGQgcG9pbnQuIElmIHRoZSBmb3JjZSBpcyBub3RcclxuICAvLy8gYXBwbGllZCBhdCB0aGUgY2VudGVyIG9mIG1hc3MsIGl0IHdpbGwgZ2VuZXJhdGUgYSB0b3JxdWUgYW5kXHJcbiAgLy8vIGFmZmVjdCB0aGUgYW5ndWxhciB2ZWxvY2l0eS4gVGhpcyB3YWtlcyB1cCB0aGUgYm9keS5cclxuICAvLy8gQHBhcmFtIGZvcmNlIHRoZSB3b3JsZCBmb3JjZSB2ZWN0b3IsIHVzdWFsbHkgaW4gTmV3dG9ucyAoTikuXHJcbiAgLy8vIEBwYXJhbSBwb2ludCB0aGUgd29ybGQgcG9zaXRpb24gb2YgdGhlIHBvaW50IG9mIGFwcGxpY2F0aW9uLlxyXG4gIC8vLyBAcGFyYW0gd2FrZSBhbHNvIHdha2UgdXAgdGhlIGJvZHlcclxuICBBcHBseUZvcmNlKGZvcmNlOiBYWSwgcG9pbnQ6IFhZLCB3YWtlID0gdHJ1ZSk6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMubV90eXBlICE9PSBiMkJvZHlUeXBlLmIyX2R5bmFtaWNCb2R5KSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBpZiAod2FrZSAmJiAhdGhpcy5tX2F3YWtlRmxhZykge1xyXG4gICAgICB0aGlzLlNldEF3YWtlKHRydWUpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIERvbid0IGFjY3VtdWxhdGUgYSBmb3JjZSBpZiB0aGUgYm9keSBpcyBzbGVlcGluZy5cclxuICAgIGlmICh0aGlzLm1fYXdha2VGbGFnKSB7XHJcbiAgICAgIHRoaXMubV9mb3JjZS54ICs9IGZvcmNlLng7XHJcbiAgICAgIHRoaXMubV9mb3JjZS55ICs9IGZvcmNlLnk7XHJcbiAgICAgIHRoaXMubV90b3JxdWUgKz1cclxuICAgICAgICAocG9pbnQueCAtIHRoaXMubV9zd2VlcC5jLngpICogZm9yY2UueSAtIChwb2ludC55IC0gdGhpcy5tX3N3ZWVwLmMueSkgKiBmb3JjZS54O1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLy8vIEFwcGx5IGEgZm9yY2UgdG8gdGhlIGNlbnRlciBvZiBtYXNzLiBUaGlzIHdha2VzIHVwIHRoZSBib2R5LlxyXG4gIC8vLyBAcGFyYW0gZm9yY2UgdGhlIHdvcmxkIGZvcmNlIHZlY3RvciwgdXN1YWxseSBpbiBOZXd0b25zIChOKS5cclxuICAvLy8gQHBhcmFtIHdha2UgYWxzbyB3YWtlIHVwIHRoZSBib2R5XHJcbiAgQXBwbHlGb3JjZVRvQ2VudGVyKGZvcmNlOiBYWSwgd2FrZSA9IHRydWUpOiB2b2lkIHtcclxuICAgIGlmICh0aGlzLm1fdHlwZSAhPT0gYjJCb2R5VHlwZS5iMl9keW5hbWljQm9keSkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHdha2UgJiYgIXRoaXMubV9hd2FrZUZsYWcpIHtcclxuICAgICAgdGhpcy5TZXRBd2FrZSh0cnVlKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBEb24ndCBhY2N1bXVsYXRlIGEgZm9yY2UgaWYgdGhlIGJvZHkgaXMgc2xlZXBpbmcuXHJcbiAgICBpZiAodGhpcy5tX2F3YWtlRmxhZykge1xyXG4gICAgICB0aGlzLm1fZm9yY2UueCArPSBmb3JjZS54O1xyXG4gICAgICB0aGlzLm1fZm9yY2UueSArPSBmb3JjZS55O1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLy8vIEFwcGx5IGEgdG9ycXVlLiBUaGlzIGFmZmVjdHMgdGhlIGFuZ3VsYXIgdmVsb2NpdHlcclxuICAvLy8gd2l0aG91dCBhZmZlY3RpbmcgdGhlIGxpbmVhciB2ZWxvY2l0eSBvZiB0aGUgY2VudGVyIG9mIG1hc3MuXHJcbiAgLy8vIEBwYXJhbSB0b3JxdWUgYWJvdXQgdGhlIHotYXhpcyAob3V0IG9mIHRoZSBzY3JlZW4pLCB1c3VhbGx5IGluIE4tbS5cclxuICAvLy8gQHBhcmFtIHdha2UgYWxzbyB3YWtlIHVwIHRoZSBib2R5XHJcbiAgQXBwbHlUb3JxdWUodG9ycXVlOiBudW1iZXIsIHdha2UgPSB0cnVlKTogdm9pZCB7XHJcbiAgICBpZiAodGhpcy5tX3R5cGUgIT09IGIyQm9keVR5cGUuYjJfZHluYW1pY0JvZHkpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIGlmICh3YWtlICYmICF0aGlzLm1fYXdha2VGbGFnKSB7XHJcbiAgICAgIHRoaXMuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gRG9uJ3QgYWNjdW11bGF0ZSBhIGZvcmNlIGlmIHRoZSBib2R5IGlzIHNsZWVwaW5nLlxyXG4gICAgaWYgKHRoaXMubV9hd2FrZUZsYWcpIHtcclxuICAgICAgdGhpcy5tX3RvcnF1ZSArPSB0b3JxdWU7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICAvLy8gQXBwbHkgYW4gaW1wdWxzZSBhdCBhIHBvaW50LiBUaGlzIGltbWVkaWF0ZWx5IG1vZGlmaWVzIHRoZSB2ZWxvY2l0eS5cclxuICAvLy8gSXQgYWxzbyBtb2RpZmllcyB0aGUgYW5ndWxhciB2ZWxvY2l0eSBpZiB0aGUgcG9pbnQgb2YgYXBwbGljYXRpb25cclxuICAvLy8gaXMgbm90IGF0IHRoZSBjZW50ZXIgb2YgbWFzcy4gVGhpcyB3YWtlcyB1cCB0aGUgYm9keS5cclxuICAvLy8gQHBhcmFtIGltcHVsc2UgdGhlIHdvcmxkIGltcHVsc2UgdmVjdG9yLCB1c3VhbGx5IGluIE4tc2Vjb25kcyBvciBrZy1tL3MuXHJcbiAgLy8vIEBwYXJhbSBwb2ludCB0aGUgd29ybGQgcG9zaXRpb24gb2YgdGhlIHBvaW50IG9mIGFwcGxpY2F0aW9uLlxyXG4gIC8vLyBAcGFyYW0gd2FrZSBhbHNvIHdha2UgdXAgdGhlIGJvZHlcclxuICBBcHBseUxpbmVhckltcHVsc2UoaW1wdWxzZTogWFksIHBvaW50OiBYWSwgd2FrZSA9IHRydWUpOiB2b2lkIHtcclxuICAgIGlmICh0aGlzLm1fdHlwZSAhPT0gYjJCb2R5VHlwZS5iMl9keW5hbWljQm9keSkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHdha2UgJiYgIXRoaXMubV9hd2FrZUZsYWcpIHtcclxuICAgICAgdGhpcy5TZXRBd2FrZSh0cnVlKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBEb24ndCBhY2N1bXVsYXRlIGEgZm9yY2UgaWYgdGhlIGJvZHkgaXMgc2xlZXBpbmcuXHJcbiAgICBpZiAodGhpcy5tX2F3YWtlRmxhZykge1xyXG4gICAgICB0aGlzLm1fbGluZWFyVmVsb2NpdHkueCArPSB0aGlzLm1faW52TWFzcyAqIGltcHVsc2UueDtcclxuICAgICAgdGhpcy5tX2xpbmVhclZlbG9jaXR5LnkgKz0gdGhpcy5tX2ludk1hc3MgKiBpbXB1bHNlLnk7XHJcbiAgICAgIHRoaXMubV9hbmd1bGFyVmVsb2NpdHkgKz1cclxuICAgICAgICB0aGlzLm1faW52SSAqXHJcbiAgICAgICAgKChwb2ludC54IC0gdGhpcy5tX3N3ZWVwLmMueCkgKiBpbXB1bHNlLnkgLSAocG9pbnQueSAtIHRoaXMubV9zd2VlcC5jLnkpICogaW1wdWxzZS54KTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vLyBBcHBseSBhbiBpbXB1bHNlIGF0IHRoZSBjZW50ZXIgb2YgZ3Jhdml0eS4gVGhpcyBpbW1lZGlhdGVseSBtb2RpZmllcyB0aGUgdmVsb2NpdHkuXHJcbiAgLy8vIEBwYXJhbSBpbXB1bHNlIHRoZSB3b3JsZCBpbXB1bHNlIHZlY3RvciwgdXN1YWxseSBpbiBOLXNlY29uZHMgb3Iga2ctbS9zLlxyXG4gIC8vLyBAcGFyYW0gd2FrZSBhbHNvIHdha2UgdXAgdGhlIGJvZHlcclxuICBBcHBseUxpbmVhckltcHVsc2VUb0NlbnRlcihpbXB1bHNlOiBYWSwgd2FrZSA9IHRydWUpOiB2b2lkIHtcclxuICAgIGlmICh0aGlzLm1fdHlwZSAhPT0gYjJCb2R5VHlwZS5iMl9keW5hbWljQm9keSkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHdha2UgJiYgIXRoaXMubV9hd2FrZUZsYWcpIHtcclxuICAgICAgdGhpcy5TZXRBd2FrZSh0cnVlKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBEb24ndCBhY2N1bXVsYXRlIGEgZm9yY2UgaWYgdGhlIGJvZHkgaXMgc2xlZXBpbmcuXHJcbiAgICBpZiAodGhpcy5tX2F3YWtlRmxhZykge1xyXG4gICAgICB0aGlzLm1fbGluZWFyVmVsb2NpdHkueCArPSB0aGlzLm1faW52TWFzcyAqIGltcHVsc2UueDtcclxuICAgICAgdGhpcy5tX2xpbmVhclZlbG9jaXR5LnkgKz0gdGhpcy5tX2ludk1hc3MgKiBpbXB1bHNlLnk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICAvLy8gQXBwbHkgYW4gYW5ndWxhciBpbXB1bHNlLlxyXG4gIC8vLyBAcGFyYW0gaW1wdWxzZSB0aGUgYW5ndWxhciBpbXB1bHNlIGluIHVuaXRzIG9mIGtnKm0qbS9zXHJcbiAgLy8vIEBwYXJhbSB3YWtlIGFsc28gd2FrZSB1cCB0aGUgYm9keVxyXG4gIEFwcGx5QW5ndWxhckltcHVsc2UoaW1wdWxzZTogbnVtYmVyLCB3YWtlID0gdHJ1ZSk6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMubV90eXBlICE9PSBiMkJvZHlUeXBlLmIyX2R5bmFtaWNCb2R5KSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBpZiAod2FrZSAmJiAhdGhpcy5tX2F3YWtlRmxhZykge1xyXG4gICAgICB0aGlzLlNldEF3YWtlKHRydWUpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIERvbid0IGFjY3VtdWxhdGUgYSBmb3JjZSBpZiB0aGUgYm9keSBpcyBzbGVlcGluZy5cclxuICAgIGlmICh0aGlzLm1fYXdha2VGbGFnKSB7XHJcbiAgICAgIHRoaXMubV9hbmd1bGFyVmVsb2NpdHkgKz0gdGhpcy5tX2ludkkgKiBpbXB1bHNlO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgdG90YWwgbWFzcyBvZiB0aGUgYm9keS5cclxuICAvLy8gQHJldHVybiB0aGUgbWFzcywgdXN1YWxseSBpbiBraWxvZ3JhbXMgKGtnKS5cclxuICBHZXRNYXNzKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX21hc3M7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSByb3RhdGlvbmFsIGluZXJ0aWEgb2YgdGhlIGJvZHkgYWJvdXQgdGhlIGxvY2FsIG9yaWdpbi5cclxuICAvLy8gQHJldHVybiB0aGUgcm90YXRpb25hbCBpbmVydGlhLCB1c3VhbGx5IGluIGtnLW1eMi5cclxuICBHZXRJbmVydGlhKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gKFxyXG4gICAgICB0aGlzLm1fSSArIHRoaXMubV9tYXNzICogYjJWZWMyLkRvdFZWKHRoaXMubV9zd2VlcC5sb2NhbENlbnRlciwgdGhpcy5tX3N3ZWVwLmxvY2FsQ2VudGVyKVxyXG4gICAgKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIG1hc3MgZGF0YSBvZiB0aGUgYm9keS5cclxuICAvLy8gQHJldHVybiBhIHN0cnVjdCBjb250YWluaW5nIHRoZSBtYXNzLCBpbmVydGlhIGFuZCBjZW50ZXIgb2YgdGhlIGJvZHkuXHJcbiAgR2V0TWFzc0RhdGEoZGF0YTogYjJNYXNzRGF0YSk6IGIyTWFzc0RhdGEge1xyXG4gICAgZGF0YS5tYXNzID0gdGhpcy5tX21hc3M7XHJcbiAgICBkYXRhLkkgPVxyXG4gICAgICB0aGlzLm1fSSArIHRoaXMubV9tYXNzICogYjJWZWMyLkRvdFZWKHRoaXMubV9zd2VlcC5sb2NhbENlbnRlciwgdGhpcy5tX3N3ZWVwLmxvY2FsQ2VudGVyKTtcclxuICAgIGRhdGEuY2VudGVyLkNvcHkodGhpcy5tX3N3ZWVwLmxvY2FsQ2VudGVyKTtcclxuICAgIHJldHVybiBkYXRhO1xyXG4gIH1cclxuXHJcbiAgLy8vIFNldCB0aGUgbWFzcyBwcm9wZXJ0aWVzIHRvIG92ZXJyaWRlIHRoZSBtYXNzIHByb3BlcnRpZXMgb2YgdGhlIGZpeHR1cmVzLlxyXG4gIC8vLyBOb3RlIHRoYXQgdGhpcyBjaGFuZ2VzIHRoZSBjZW50ZXIgb2YgbWFzcyBwb3NpdGlvbi5cclxuICAvLy8gTm90ZSB0aGF0IGNyZWF0aW5nIG9yIGRlc3Ryb3lpbmcgZml4dHVyZXMgY2FuIGFsc28gYWx0ZXIgdGhlIG1hc3MuXHJcbiAgLy8vIFRoaXMgZnVuY3Rpb24gaGFzIG5vIGVmZmVjdCBpZiB0aGUgYm9keSBpc24ndCBkeW5hbWljLlxyXG4gIC8vLyBAcGFyYW0gbWFzc0RhdGEgdGhlIG1hc3MgcHJvcGVydGllcy5cclxuICBwcml2YXRlIHN0YXRpYyBTZXRNYXNzRGF0YV9zX29sZENlbnRlcjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTZXRNYXNzRGF0YShtYXNzRGF0YTogYjJNYXNzRGF0YSk6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMubV93b3JsZC5Jc0xvY2tlZCgpKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG5cclxuICAgIGlmICh0aGlzLm1fdHlwZSAhPT0gYjJCb2R5VHlwZS5iMl9keW5hbWljQm9keSkge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5tX2ludk1hc3MgPSAwO1xyXG4gICAgdGhpcy5tX0kgPSAwO1xyXG4gICAgdGhpcy5tX2ludkkgPSAwO1xyXG5cclxuICAgIHRoaXMubV9tYXNzID0gbWFzc0RhdGEubWFzcztcclxuICAgIGlmICh0aGlzLm1fbWFzcyA8PSAwKSB7XHJcbiAgICAgIHRoaXMubV9tYXNzID0gMTtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLm1faW52TWFzcyA9IDEgLyB0aGlzLm1fbWFzcztcclxuXHJcbiAgICBpZiAobWFzc0RhdGEuSSA+IDAgJiYgIXRoaXMubV9maXhlZFJvdGF0aW9uRmxhZykge1xyXG4gICAgICB0aGlzLm1fSSA9IG1hc3NEYXRhLkkgLSB0aGlzLm1fbWFzcyAqIGIyVmVjMi5Eb3RWVihtYXNzRGF0YS5jZW50ZXIsIG1hc3NEYXRhLmNlbnRlcik7XHJcbiAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX0kgPiAwKTtcclxuICAgICAgdGhpcy5tX2ludkkgPSAxIC8gdGhpcy5tX0k7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gTW92ZSBjZW50ZXIgb2YgbWFzcy5cclxuICAgIGNvbnN0IG9sZENlbnRlcjogYjJWZWMyID0gYjJCb2R5LlNldE1hc3NEYXRhX3Nfb2xkQ2VudGVyLkNvcHkodGhpcy5tX3N3ZWVwLmMpO1xyXG4gICAgdGhpcy5tX3N3ZWVwLmxvY2FsQ2VudGVyLkNvcHkobWFzc0RhdGEuY2VudGVyKTtcclxuICAgIGIyVHJhbnNmb3JtLk11bFhWKHRoaXMubV94ZiwgdGhpcy5tX3N3ZWVwLmxvY2FsQ2VudGVyLCB0aGlzLm1fc3dlZXAuYyk7XHJcbiAgICB0aGlzLm1fc3dlZXAuYzAuQ29weSh0aGlzLm1fc3dlZXAuYyk7XHJcblxyXG4gICAgLy8gVXBkYXRlIGNlbnRlciBvZiBtYXNzIHZlbG9jaXR5LlxyXG4gICAgYjJWZWMyLkFkZFZDcm9zc1NWKFxyXG4gICAgICB0aGlzLm1fbGluZWFyVmVsb2NpdHksXHJcbiAgICAgIHRoaXMubV9hbmd1bGFyVmVsb2NpdHksXHJcbiAgICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fc3dlZXAuYywgb2xkQ2VudGVyLCBiMlZlYzIuc190MCksXHJcbiAgICAgIHRoaXMubV9saW5lYXJWZWxvY2l0eSxcclxuICAgICk7XHJcbiAgfVxyXG5cclxuICAvLy8gVGhpcyByZXNldHMgdGhlIG1hc3MgcHJvcGVydGllcyB0byB0aGUgc3VtIG9mIHRoZSBtYXNzIHByb3BlcnRpZXMgb2YgdGhlIGZpeHR1cmVzLlxyXG4gIC8vLyBUaGlzIG5vcm1hbGx5IGRvZXMgbm90IG5lZWQgdG8gYmUgY2FsbGVkIHVubGVzcyB5b3UgY2FsbGVkIFNldE1hc3NEYXRhIHRvIG92ZXJyaWRlXHJcbiAgLy8vIHRoZSBtYXNzIGFuZCB5b3UgbGF0ZXIgd2FudCB0byByZXNldCB0aGUgbWFzcy5cclxuICBwcml2YXRlIHN0YXRpYyBSZXNldE1hc3NEYXRhX3NfbG9jYWxDZW50ZXI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBSZXNldE1hc3NEYXRhX3Nfb2xkQ2VudGVyOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgUmVzZXRNYXNzRGF0YV9zX21hc3NEYXRhOiBiMk1hc3NEYXRhID0gbmV3IGIyTWFzc0RhdGEoKTtcclxuXHJcbiAgUmVzZXRNYXNzRGF0YSgpOiB2b2lkIHtcclxuICAgIC8vIENvbXB1dGUgbWFzcyBkYXRhIGZyb20gc2hhcGVzLiBFYWNoIHNoYXBlIGhhcyBpdHMgb3duIGRlbnNpdHkuXHJcbiAgICB0aGlzLm1fbWFzcyA9IDA7XHJcbiAgICB0aGlzLm1faW52TWFzcyA9IDA7XHJcbiAgICB0aGlzLm1fSSA9IDA7XHJcbiAgICB0aGlzLm1faW52SSA9IDA7XHJcbiAgICB0aGlzLm1fc3dlZXAubG9jYWxDZW50ZXIuU2V0WmVybygpO1xyXG5cclxuICAgIC8vIFN0YXRpYyBhbmQga2luZW1hdGljIGJvZGllcyBoYXZlIHplcm8gbWFzcy5cclxuICAgIGlmICh0aGlzLm1fdHlwZSA9PT0gYjJCb2R5VHlwZS5iMl9zdGF0aWNCb2R5IHx8IHRoaXMubV90eXBlID09PSBiMkJvZHlUeXBlLmIyX2tpbmVtYXRpY0JvZHkpIHtcclxuICAgICAgdGhpcy5tX3N3ZWVwLmMwLkNvcHkodGhpcy5tX3hmLnApO1xyXG4gICAgICB0aGlzLm1fc3dlZXAuYy5Db3B5KHRoaXMubV94Zi5wKTtcclxuICAgICAgdGhpcy5tX3N3ZWVwLmEwID0gdGhpcy5tX3N3ZWVwLmE7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRoaXMubV90eXBlID09PSBiMkJvZHlUeXBlLmIyX2R5bmFtaWNCb2R5KTtcclxuXHJcbiAgICAvLyBBY2N1bXVsYXRlIG1hc3Mgb3ZlciBhbGwgZml4dHVyZXMuXHJcbiAgICBjb25zdCBsb2NhbENlbnRlcjogYjJWZWMyID0gYjJCb2R5LlJlc2V0TWFzc0RhdGFfc19sb2NhbENlbnRlci5TZXRaZXJvKCk7XHJcbiAgICBmb3IgKGxldCBmOiBiMkZpeHR1cmUgfCBudWxsID0gdGhpcy5tX2ZpeHR1cmVMaXN0OyBmOyBmID0gZi5tX25leHQpIHtcclxuICAgICAgaWYgKGYubV9kZW5zaXR5ID09PSAwKSB7XHJcbiAgICAgICAgY29udGludWU7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGNvbnN0IG1hc3NEYXRhOiBiMk1hc3NEYXRhID0gZi5HZXRNYXNzRGF0YShiMkJvZHkuUmVzZXRNYXNzRGF0YV9zX21hc3NEYXRhKTtcclxuICAgICAgdGhpcy5tX21hc3MgKz0gbWFzc0RhdGEubWFzcztcclxuICAgICAgbG9jYWxDZW50ZXIueCArPSBtYXNzRGF0YS5jZW50ZXIueCAqIG1hc3NEYXRhLm1hc3M7XHJcbiAgICAgIGxvY2FsQ2VudGVyLnkgKz0gbWFzc0RhdGEuY2VudGVyLnkgKiBtYXNzRGF0YS5tYXNzO1xyXG4gICAgICB0aGlzLm1fSSArPSBtYXNzRGF0YS5JO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIENvbXB1dGUgY2VudGVyIG9mIG1hc3MuXHJcbiAgICBpZiAodGhpcy5tX21hc3MgPiAwKSB7XHJcbiAgICAgIHRoaXMubV9pbnZNYXNzID0gMSAvIHRoaXMubV9tYXNzO1xyXG4gICAgICBsb2NhbENlbnRlci54ICo9IHRoaXMubV9pbnZNYXNzO1xyXG4gICAgICBsb2NhbENlbnRlci55ICo9IHRoaXMubV9pbnZNYXNzO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgLy8gRm9yY2UgYWxsIGR5bmFtaWMgYm9kaWVzIHRvIGhhdmUgYSBwb3NpdGl2ZSBtYXNzLlxyXG4gICAgICB0aGlzLm1fbWFzcyA9IDE7XHJcbiAgICAgIHRoaXMubV9pbnZNYXNzID0gMTtcclxuICAgIH1cclxuXHJcbiAgICBpZiAodGhpcy5tX0kgPiAwICYmICF0aGlzLm1fZml4ZWRSb3RhdGlvbkZsYWcpIHtcclxuICAgICAgLy8gQ2VudGVyIHRoZSBpbmVydGlhIGFib3V0IHRoZSBjZW50ZXIgb2YgbWFzcy5cclxuICAgICAgdGhpcy5tX0kgLT0gdGhpcy5tX21hc3MgKiBiMlZlYzIuRG90VlYobG9jYWxDZW50ZXIsIGxvY2FsQ2VudGVyKTtcclxuICAgICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCh0aGlzLm1fSSA+IDApO1xyXG4gICAgICB0aGlzLm1faW52SSA9IDEgLyB0aGlzLm1fSTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV9JID0gMDtcclxuICAgICAgdGhpcy5tX2ludkkgPSAwO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIE1vdmUgY2VudGVyIG9mIG1hc3MuXHJcbiAgICBjb25zdCBvbGRDZW50ZXI6IGIyVmVjMiA9IGIyQm9keS5SZXNldE1hc3NEYXRhX3Nfb2xkQ2VudGVyLkNvcHkodGhpcy5tX3N3ZWVwLmMpO1xyXG4gICAgdGhpcy5tX3N3ZWVwLmxvY2FsQ2VudGVyLkNvcHkobG9jYWxDZW50ZXIpO1xyXG4gICAgYjJUcmFuc2Zvcm0uTXVsWFYodGhpcy5tX3hmLCB0aGlzLm1fc3dlZXAubG9jYWxDZW50ZXIsIHRoaXMubV9zd2VlcC5jKTtcclxuICAgIHRoaXMubV9zd2VlcC5jMC5Db3B5KHRoaXMubV9zd2VlcC5jKTtcclxuXHJcbiAgICAvLyBVcGRhdGUgY2VudGVyIG9mIG1hc3MgdmVsb2NpdHkuXHJcbiAgICBiMlZlYzIuQWRkVkNyb3NzU1YoXHJcbiAgICAgIHRoaXMubV9saW5lYXJWZWxvY2l0eSxcclxuICAgICAgdGhpcy5tX2FuZ3VsYXJWZWxvY2l0eSxcclxuICAgICAgYjJWZWMyLlN1YlZWKHRoaXMubV9zd2VlcC5jLCBvbGRDZW50ZXIsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgdGhpcy5tX2xpbmVhclZlbG9jaXR5LFxyXG4gICAgKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIHdvcmxkIGNvb3JkaW5hdGVzIG9mIGEgcG9pbnQgZ2l2ZW4gdGhlIGxvY2FsIGNvb3JkaW5hdGVzLlxyXG4gIC8vLyBAcGFyYW0gbG9jYWxQb2ludCBhIHBvaW50IG9uIHRoZSBib2R5IG1lYXN1cmVkIHJlbGF0aXZlIHRoZSB0aGUgYm9keSdzIG9yaWdpbi5cclxuICAvLy8gQHJldHVybiB0aGUgc2FtZSBwb2ludCBleHByZXNzZWQgaW4gd29ybGQgY29vcmRpbmF0ZXMuXHJcbiAgR2V0V29ybGRQb2ludDxUIGV4dGVuZHMgWFk+KGxvY2FsUG9pbnQ6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIHJldHVybiBiMlRyYW5zZm9ybS5NdWxYVih0aGlzLm1feGYsIGxvY2FsUG9pbnQsIG91dCk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSB3b3JsZCBjb29yZGluYXRlcyBvZiBhIHZlY3RvciBnaXZlbiB0aGUgbG9jYWwgY29vcmRpbmF0ZXMuXHJcbiAgLy8vIEBwYXJhbSBsb2NhbFZlY3RvciBhIHZlY3RvciBmaXhlZCBpbiB0aGUgYm9keS5cclxuICAvLy8gQHJldHVybiB0aGUgc2FtZSB2ZWN0b3IgZXhwcmVzc2VkIGluIHdvcmxkIGNvb3JkaW5hdGVzLlxyXG4gIEdldFdvcmxkVmVjdG9yPFQgZXh0ZW5kcyBYWT4obG9jYWxWZWN0b3I6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIHJldHVybiBiMlJvdC5NdWxSVih0aGlzLm1feGYucSwgbG9jYWxWZWN0b3IsIG91dCk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0cyBhIGxvY2FsIHBvaW50IHJlbGF0aXZlIHRvIHRoZSBib2R5J3Mgb3JpZ2luIGdpdmVuIGEgd29ybGQgcG9pbnQuXHJcbiAgLy8vIEBwYXJhbSBhIHBvaW50IGluIHdvcmxkIGNvb3JkaW5hdGVzLlxyXG4gIC8vLyBAcmV0dXJuIHRoZSBjb3JyZXNwb25kaW5nIGxvY2FsIHBvaW50IHJlbGF0aXZlIHRvIHRoZSBib2R5J3Mgb3JpZ2luLlxyXG4gIEdldExvY2FsUG9pbnQ8VCBleHRlbmRzIFhZPih3b3JsZFBvaW50OiBYWSwgb3V0OiBUKTogVCB7XHJcbiAgICByZXR1cm4gYjJUcmFuc2Zvcm0uTXVsVFhWKHRoaXMubV94Ziwgd29ybGRQb2ludCwgb3V0KTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXRzIGEgbG9jYWwgdmVjdG9yIGdpdmVuIGEgd29ybGQgdmVjdG9yLlxyXG4gIC8vLyBAcGFyYW0gYSB2ZWN0b3IgaW4gd29ybGQgY29vcmRpbmF0ZXMuXHJcbiAgLy8vIEByZXR1cm4gdGhlIGNvcnJlc3BvbmRpbmcgbG9jYWwgdmVjdG9yLlxyXG4gIEdldExvY2FsVmVjdG9yPFQgZXh0ZW5kcyBYWT4od29ybGRWZWN0b3I6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIHJldHVybiBiMlJvdC5NdWxUUlYodGhpcy5tX3hmLnEsIHdvcmxkVmVjdG9yLCBvdXQpO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgd29ybGQgbGluZWFyIHZlbG9jaXR5IG9mIGEgd29ybGQgcG9pbnQgYXR0YWNoZWQgdG8gdGhpcyBib2R5LlxyXG4gIC8vLyBAcGFyYW0gYSBwb2ludCBpbiB3b3JsZCBjb29yZGluYXRlcy5cclxuICAvLy8gQHJldHVybiB0aGUgd29ybGQgdmVsb2NpdHkgb2YgYSBwb2ludC5cclxuICBHZXRMaW5lYXJWZWxvY2l0eUZyb21Xb3JsZFBvaW50PFQgZXh0ZW5kcyBYWT4od29ybGRQb2ludDogWFksIG91dDogVCk6IFQge1xyXG4gICAgcmV0dXJuIGIyVmVjMi5BZGRWQ3Jvc3NTVihcclxuICAgICAgdGhpcy5tX2xpbmVhclZlbG9jaXR5LFxyXG4gICAgICB0aGlzLm1fYW5ndWxhclZlbG9jaXR5LFxyXG4gICAgICBiMlZlYzIuU3ViVlYod29ybGRQb2ludCwgdGhpcy5tX3N3ZWVwLmMsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgb3V0LFxyXG4gICAgKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIHdvcmxkIHZlbG9jaXR5IG9mIGEgbG9jYWwgcG9pbnQuXHJcbiAgLy8vIEBwYXJhbSBhIHBvaW50IGluIGxvY2FsIGNvb3JkaW5hdGVzLlxyXG4gIC8vLyBAcmV0dXJuIHRoZSB3b3JsZCB2ZWxvY2l0eSBvZiBhIHBvaW50LlxyXG4gIEdldExpbmVhclZlbG9jaXR5RnJvbUxvY2FsUG9pbnQ8VCBleHRlbmRzIFhZPihsb2NhbFBvaW50OiBYWSwgb3V0OiBUKTogVCB7XHJcbiAgICByZXR1cm4gdGhpcy5HZXRMaW5lYXJWZWxvY2l0eUZyb21Xb3JsZFBvaW50KHRoaXMuR2V0V29ybGRQb2ludChsb2NhbFBvaW50LCBvdXQpLCBvdXQpO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgbGluZWFyIGRhbXBpbmcgb2YgdGhlIGJvZHkuXHJcbiAgR2V0TGluZWFyRGFtcGluZygpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9saW5lYXJEYW1waW5nO1xyXG4gIH1cclxuXHJcbiAgLy8vIFNldCB0aGUgbGluZWFyIGRhbXBpbmcgb2YgdGhlIGJvZHkuXHJcbiAgU2V0TGluZWFyRGFtcGluZyhsaW5lYXJEYW1waW5nOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIHRoaXMubV9saW5lYXJEYW1waW5nID0gbGluZWFyRGFtcGluZztcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGFuZ3VsYXIgZGFtcGluZyBvZiB0aGUgYm9keS5cclxuICBHZXRBbmd1bGFyRGFtcGluZygpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9hbmd1bGFyRGFtcGluZztcclxuICB9XHJcblxyXG4gIC8vLyBTZXQgdGhlIGFuZ3VsYXIgZGFtcGluZyBvZiB0aGUgYm9keS5cclxuICBTZXRBbmd1bGFyRGFtcGluZyhhbmd1bGFyRGFtcGluZzogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fYW5ndWxhckRhbXBpbmcgPSBhbmd1bGFyRGFtcGluZztcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGdyYXZpdHkgc2NhbGUgb2YgdGhlIGJvZHkuXHJcbiAgR2V0R3Jhdml0eVNjYWxlKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2dyYXZpdHlTY2FsZTtcclxuICB9XHJcblxyXG4gIC8vLyBTZXQgdGhlIGdyYXZpdHkgc2NhbGUgb2YgdGhlIGJvZHkuXHJcbiAgU2V0R3Jhdml0eVNjYWxlKHNjYWxlOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIHRoaXMubV9ncmF2aXR5U2NhbGUgPSBzY2FsZTtcclxuICB9XHJcblxyXG4gIC8vLyBTZXQgdGhlIHR5cGUgb2YgdGhpcyBib2R5LiBUaGlzIG1heSBhbHRlciB0aGUgbWFzcyBhbmQgdmVsb2NpdHkuXHJcbiAgU2V0VHlwZSh0eXBlOiBiMkJvZHlUeXBlKTogdm9pZCB7XHJcbiAgICBpZiAodGhpcy5tX3dvcmxkLklzTG9ja2VkKCkpIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHRoaXMubV90eXBlID09PSB0eXBlKSB7XHJcbiAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLm1fdHlwZSA9IHR5cGU7XHJcblxyXG4gICAgdGhpcy5SZXNldE1hc3NEYXRhKCk7XHJcblxyXG4gICAgaWYgKHRoaXMubV90eXBlID09PSBiMkJvZHlUeXBlLmIyX3N0YXRpY0JvZHkpIHtcclxuICAgICAgdGhpcy5tX2xpbmVhclZlbG9jaXR5LlNldFplcm8oKTtcclxuICAgICAgdGhpcy5tX2FuZ3VsYXJWZWxvY2l0eSA9IDA7XHJcbiAgICAgIHRoaXMubV9zd2VlcC5hMCA9IHRoaXMubV9zd2VlcC5hO1xyXG4gICAgICB0aGlzLm1fc3dlZXAuYzAuQ29weSh0aGlzLm1fc3dlZXAuYyk7XHJcbiAgICAgIHRoaXMuU3luY2hyb25pemVGaXh0dXJlcygpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMuU2V0QXdha2UodHJ1ZSk7XHJcblxyXG4gICAgdGhpcy5tX2ZvcmNlLlNldFplcm8oKTtcclxuICAgIHRoaXMubV90b3JxdWUgPSAwO1xyXG5cclxuICAgIC8vIERlbGV0ZSB0aGUgYXR0YWNoZWQgY29udGFjdHMuXHJcbiAgICBsZXQgY2U6IGIyQ29udGFjdEVkZ2UgfCBudWxsID0gdGhpcy5tX2NvbnRhY3RMaXN0O1xyXG4gICAgd2hpbGUgKGNlKSB7XHJcbiAgICAgIGNvbnN0IGNlMDogYjJDb250YWN0RWRnZSA9IGNlO1xyXG4gICAgICBjZSA9IGNlLm5leHQ7XHJcbiAgICAgIHRoaXMubV93b3JsZC5tX2NvbnRhY3RNYW5hZ2VyLkRlc3Ryb3koY2UwLmNvbnRhY3QpO1xyXG4gICAgfVxyXG4gICAgdGhpcy5tX2NvbnRhY3RMaXN0ID0gbnVsbDtcclxuXHJcbiAgICAvLyBUb3VjaCB0aGUgcHJveGllcyBzbyB0aGF0IG5ldyBjb250YWN0cyB3aWxsIGJlIGNyZWF0ZWQgKHdoZW4gYXBwcm9wcmlhdGUpXHJcbiAgICBmb3IgKGxldCBmOiBiMkZpeHR1cmUgfCBudWxsID0gdGhpcy5tX2ZpeHR1cmVMaXN0OyBmOyBmID0gZi5tX25leHQpIHtcclxuICAgICAgZi5Ub3VjaFByb3hpZXMoKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIHR5cGUgb2YgdGhpcyBib2R5LlxyXG4gIEdldFR5cGUoKTogYjJCb2R5VHlwZSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3R5cGU7XHJcbiAgfVxyXG5cclxuICAvLy8gU2hvdWxkIHRoaXMgYm9keSBiZSB0cmVhdGVkIGxpa2UgYSBidWxsZXQgZm9yIGNvbnRpbnVvdXMgY29sbGlzaW9uIGRldGVjdGlvbj9cclxuICBTZXRCdWxsZXQoZmxhZzogYm9vbGVhbik6IHZvaWQge1xyXG4gICAgdGhpcy5tX2J1bGxldEZsYWcgPSBmbGFnO1xyXG4gIH1cclxuXHJcbiAgLy8vIElzIHRoaXMgYm9keSB0cmVhdGVkIGxpa2UgYSBidWxsZXQgZm9yIGNvbnRpbnVvdXMgY29sbGlzaW9uIGRldGVjdGlvbj9cclxuICBJc0J1bGxldCgpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0aGlzLm1fYnVsbGV0RmxhZztcclxuICB9XHJcblxyXG4gIC8vLyBZb3UgY2FuIGRpc2FibGUgc2xlZXBpbmcgb24gdGhpcyBib2R5LiBJZiB5b3UgZGlzYWJsZSBzbGVlcGluZywgdGhlXHJcbiAgLy8vIGJvZHkgd2lsbCBiZSB3b2tlbi5cclxuICBTZXRTbGVlcGluZ0FsbG93ZWQoZmxhZzogYm9vbGVhbik6IHZvaWQge1xyXG4gICAgdGhpcy5tX2F1dG9TbGVlcEZsYWcgPSBmbGFnO1xyXG4gICAgaWYgKCFmbGFnKSB7XHJcbiAgICAgIHRoaXMuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICAvLy8gSXMgdGhpcyBib2R5IGFsbG93ZWQgdG8gc2xlZXBcclxuICBJc1NsZWVwaW5nQWxsb3dlZCgpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0aGlzLm1fYXV0b1NsZWVwRmxhZztcclxuICB9XHJcblxyXG4gIC8vLyBTZXQgdGhlIHNsZWVwIHN0YXRlIG9mIHRoZSBib2R5LiBBIHNsZWVwaW5nIGJvZHkgaGFzIHZlcnlcclxuICAvLy8gbG93IENQVSBjb3N0LlxyXG4gIC8vLyBAcGFyYW0gZmxhZyBzZXQgdG8gdHJ1ZSB0byB3YWtlIHRoZSBib2R5LCBmYWxzZSB0byBwdXQgaXQgdG8gc2xlZXAuXHJcbiAgU2V0QXdha2UoZmxhZzogYm9vbGVhbik6IHZvaWQge1xyXG4gICAgaWYgKGZsYWcpIHtcclxuICAgICAgdGhpcy5tX2F3YWtlRmxhZyA9IHRydWU7XHJcbiAgICAgIHRoaXMubV9zbGVlcFRpbWUgPSAwO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgdGhpcy5tX2F3YWtlRmxhZyA9IGZhbHNlO1xyXG4gICAgICB0aGlzLm1fc2xlZXBUaW1lID0gMDtcclxuICAgICAgdGhpcy5tX2xpbmVhclZlbG9jaXR5LlNldFplcm8oKTtcclxuICAgICAgdGhpcy5tX2FuZ3VsYXJWZWxvY2l0eSA9IDA7XHJcbiAgICAgIHRoaXMubV9mb3JjZS5TZXRaZXJvKCk7XHJcbiAgICAgIHRoaXMubV90b3JxdWUgPSAwO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgc2xlZXBpbmcgc3RhdGUgb2YgdGhpcyBib2R5LlxyXG4gIC8vLyBAcmV0dXJuIHRydWUgaWYgdGhlIGJvZHkgaXMgc2xlZXBpbmcuXHJcbiAgSXNBd2FrZSgpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0aGlzLm1fYXdha2VGbGFnO1xyXG4gIH1cclxuXHJcbiAgLy8vIFNldCB0aGUgYWN0aXZlIHN0YXRlIG9mIHRoZSBib2R5LiBBbiBpbmFjdGl2ZSBib2R5IGlzIG5vdFxyXG4gIC8vLyBzaW11bGF0ZWQgYW5kIGNhbm5vdCBiZSBjb2xsaWRlZCB3aXRoIG9yIHdva2VuIHVwLlxyXG4gIC8vLyBJZiB5b3UgcGFzcyBhIGZsYWcgb2YgdHJ1ZSwgYWxsIGZpeHR1cmVzIHdpbGwgYmUgYWRkZWQgdG8gdGhlXHJcbiAgLy8vIGJyb2FkLXBoYXNlLlxyXG4gIC8vLyBJZiB5b3UgcGFzcyBhIGZsYWcgb2YgZmFsc2UsIGFsbCBmaXh0dXJlcyB3aWxsIGJlIHJlbW92ZWQgZnJvbVxyXG4gIC8vLyB0aGUgYnJvYWQtcGhhc2UgYW5kIGFsbCBjb250YWN0cyB3aWxsIGJlIGRlc3Ryb3llZC5cclxuICAvLy8gRml4dHVyZXMgYW5kIGpvaW50cyBhcmUgb3RoZXJ3aXNlIHVuYWZmZWN0ZWQuIFlvdSBtYXkgY29udGludWVcclxuICAvLy8gdG8gY3JlYXRlL2Rlc3Ryb3kgZml4dHVyZXMgYW5kIGpvaW50cyBvbiBpbmFjdGl2ZSBib2RpZXMuXHJcbiAgLy8vIEZpeHR1cmVzIG9uIGFuIGluYWN0aXZlIGJvZHkgYXJlIGltcGxpY2l0bHkgaW5hY3RpdmUgYW5kIHdpbGxcclxuICAvLy8gbm90IHBhcnRpY2lwYXRlIGluIGNvbGxpc2lvbnMsIHJheS1jYXN0cywgb3IgcXVlcmllcy5cclxuICAvLy8gSm9pbnRzIGNvbm5lY3RlZCB0byBhbiBpbmFjdGl2ZSBib2R5IGFyZSBpbXBsaWNpdGx5IGluYWN0aXZlLlxyXG4gIC8vLyBBbiBpbmFjdGl2ZSBib2R5IGlzIHN0aWxsIG93bmVkIGJ5IGEgYjJXb3JsZCBvYmplY3QgYW5kIHJlbWFpbnNcclxuICAvLy8gaW4gdGhlIGJvZHkgbGlzdC5cclxuICBTZXRBY3RpdmUoZmxhZzogYm9vbGVhbik6IHZvaWQge1xyXG4gICAgaWYgKHRoaXMubV93b3JsZC5Jc0xvY2tlZCgpKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChmbGFnID09PSB0aGlzLklzQWN0aXZlKCkpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9hY3RpdmVGbGFnID0gZmxhZztcclxuXHJcbiAgICBpZiAoZmxhZykge1xyXG4gICAgICAvLyBDcmVhdGUgYWxsIHByb3hpZXMuXHJcbiAgICAgIGZvciAobGV0IGY6IGIyRml4dHVyZSB8IG51bGwgPSB0aGlzLm1fZml4dHVyZUxpc3Q7IGY7IGYgPSBmLm1fbmV4dCkge1xyXG4gICAgICAgIGYuQ3JlYXRlUHJveGllcygpO1xyXG4gICAgICB9XHJcbiAgICAgIC8vIENvbnRhY3RzIGFyZSBjcmVhdGVkIHRoZSBuZXh0IHRpbWUgc3RlcC5cclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIC8vIERlc3Ryb3kgYWxsIHByb3hpZXMuXHJcbiAgICAgIGZvciAobGV0IGY6IGIyRml4dHVyZSB8IG51bGwgPSB0aGlzLm1fZml4dHVyZUxpc3Q7IGY7IGYgPSBmLm1fbmV4dCkge1xyXG4gICAgICAgIGYuRGVzdHJveVByb3hpZXMoKTtcclxuICAgICAgfVxyXG4gICAgICAvLyBEZXN0cm95IHRoZSBhdHRhY2hlZCBjb250YWN0cy5cclxuICAgICAgbGV0IGNlOiBiMkNvbnRhY3RFZGdlIHwgbnVsbCA9IHRoaXMubV9jb250YWN0TGlzdDtcclxuICAgICAgd2hpbGUgKGNlKSB7XHJcbiAgICAgICAgY29uc3QgY2UwOiBiMkNvbnRhY3RFZGdlID0gY2U7XHJcbiAgICAgICAgY2UgPSBjZS5uZXh0O1xyXG4gICAgICAgIHRoaXMubV93b3JsZC5tX2NvbnRhY3RNYW5hZ2VyLkRlc3Ryb3koY2UwLmNvbnRhY3QpO1xyXG4gICAgICB9XHJcbiAgICAgIHRoaXMubV9jb250YWN0TGlzdCA9IG51bGw7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBhY3RpdmUgc3RhdGUgb2YgdGhlIGJvZHkuXHJcbiAgSXNBY3RpdmUoKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2FjdGl2ZUZsYWc7XHJcbiAgfVxyXG5cclxuICAvLy8gU2V0IHRoaXMgYm9keSB0byBoYXZlIGZpeGVkIHJvdGF0aW9uLiBUaGlzIGNhdXNlcyB0aGUgbWFzc1xyXG4gIC8vLyB0byBiZSByZXNldC5cclxuICBTZXRGaXhlZFJvdGF0aW9uKGZsYWc6IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIGlmICh0aGlzLm1fZml4ZWRSb3RhdGlvbkZsYWcgPT09IGZsYWcpIHtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9maXhlZFJvdGF0aW9uRmxhZyA9IGZsYWc7XHJcblxyXG4gICAgdGhpcy5tX2FuZ3VsYXJWZWxvY2l0eSA9IDA7XHJcblxyXG4gICAgdGhpcy5SZXNldE1hc3NEYXRhKCk7XHJcbiAgfVxyXG5cclxuICAvLy8gRG9lcyB0aGlzIGJvZHkgaGF2ZSBmaXhlZCByb3RhdGlvbj9cclxuICBJc0ZpeGVkUm90YXRpb24oKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2ZpeGVkUm90YXRpb25GbGFnO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgbGlzdCBvZiBhbGwgZml4dHVyZXMgYXR0YWNoZWQgdG8gdGhpcyBib2R5LlxyXG4gIEdldEZpeHR1cmVMaXN0KCk6IGIyRml4dHVyZSB8IG51bGwge1xyXG4gICAgcmV0dXJuIHRoaXMubV9maXh0dXJlTGlzdDtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGxpc3Qgb2YgYWxsIGpvaW50cyBhdHRhY2hlZCB0byB0aGlzIGJvZHkuXHJcbiAgR2V0Sm9pbnRMaXN0KCk6IGIySm9pbnRFZGdlIHwgbnVsbCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2pvaW50TGlzdDtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGxpc3Qgb2YgYWxsIGNvbnRhY3RzIGF0dGFjaGVkIHRvIHRoaXMgYm9keS5cclxuICAvLy8gQHdhcm5pbmcgdGhpcyBsaXN0IGNoYW5nZXMgZHVyaW5nIHRoZSB0aW1lIHN0ZXAgYW5kIHlvdSBtYXlcclxuICAvLy8gbWlzcyBzb21lIGNvbGxpc2lvbnMgaWYgeW91IGRvbid0IHVzZSBiMkNvbnRhY3RMaXN0ZW5lci5cclxuICBHZXRDb250YWN0TGlzdCgpOiBiMkNvbnRhY3RFZGdlIHwgbnVsbCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2NvbnRhY3RMaXN0O1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgbmV4dCBib2R5IGluIHRoZSB3b3JsZCdzIGJvZHkgbGlzdC5cclxuICBHZXROZXh0KCk6IGIyQm9keSB8IG51bGwge1xyXG4gICAgcmV0dXJuIHRoaXMubV9uZXh0O1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgdXNlciBkYXRhIHBvaW50ZXIgdGhhdCB3YXMgcHJvdmlkZWQgaW4gdGhlIGJvZHkgZGVmaW5pdGlvbi5cclxuICBHZXRVc2VyRGF0YSgpOiBhbnkge1xyXG4gICAgcmV0dXJuIHRoaXMubV91c2VyRGF0YTtcclxuICB9XHJcblxyXG4gIC8vLyBTZXQgdGhlIHVzZXIgZGF0YS4gVXNlIHRoaXMgdG8gc3RvcmUgeW91ciBhcHBsaWNhdGlvbiBzcGVjaWZpYyBkYXRhLlxyXG4gIFNldFVzZXJEYXRhKGRhdGE6IGFueSk6IHZvaWQge1xyXG4gICAgdGhpcy5tX3VzZXJEYXRhID0gZGF0YTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIHBhcmVudCB3b3JsZCBvZiB0aGlzIGJvZHkuXHJcbiAgR2V0V29ybGQoKTogYjJXb3JsZCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3dvcmxkO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgU3luY2hyb25pemVGaXh0dXJlc19zX3hmMTogYjJUcmFuc2Zvcm0gPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICBwcml2YXRlIHN0YXRpYyBTeW5jaHJvbml6ZUZpeHR1cmVzX3NfZGlzcGxhY2VtZW50OiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFN5bmNocm9uaXplRml4dHVyZXMoKTogdm9pZCB7XHJcbiAgICBjb25zdCB4ZjE6IGIyVHJhbnNmb3JtID0gYjJCb2R5LlN5bmNocm9uaXplRml4dHVyZXNfc194ZjE7XHJcbiAgICB4ZjEucS5TZXRBbmdsZSh0aGlzLm1fc3dlZXAuYTApO1xyXG4gICAgYjJSb3QuTXVsUlYoeGYxLnEsIHRoaXMubV9zd2VlcC5sb2NhbENlbnRlciwgeGYxLnApO1xyXG4gICAgYjJWZWMyLlN1YlZWKHRoaXMubV9zd2VlcC5jMCwgeGYxLnAsIHhmMS5wKTtcclxuXHJcbiAgICAvLyBjb25zdCBkaXNwbGFjZW1lbnQ6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVih0aGlzLm1feGYucCwgeGYxLnAsIGIyQm9keS5TeW5jaHJvbml6ZUZpeHR1cmVzX3NfZGlzcGxhY2VtZW50KTtcclxuICAgIGNvbnN0IGRpc3BsYWNlbWVudDogYjJWZWMyID0gYjJWZWMyLlN1YlZWKFxyXG4gICAgICB0aGlzLm1fc3dlZXAuYyxcclxuICAgICAgdGhpcy5tX3N3ZWVwLmMwLFxyXG4gICAgICBiMkJvZHkuU3luY2hyb25pemVGaXh0dXJlc19zX2Rpc3BsYWNlbWVudCxcclxuICAgICk7XHJcblxyXG4gICAgZm9yIChsZXQgZjogYjJGaXh0dXJlIHwgbnVsbCA9IHRoaXMubV9maXh0dXJlTGlzdDsgZjsgZiA9IGYubV9uZXh0KSB7XHJcbiAgICAgIGYuU3luY2hyb25pemVQcm94aWVzKHhmMSwgdGhpcy5tX3hmLCBkaXNwbGFjZW1lbnQpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgU3luY2hyb25pemVUcmFuc2Zvcm0oKTogdm9pZCB7XHJcbiAgICB0aGlzLm1feGYucS5TZXRBbmdsZSh0aGlzLm1fc3dlZXAuYSk7XHJcbiAgICBiMlJvdC5NdWxSVih0aGlzLm1feGYucSwgdGhpcy5tX3N3ZWVwLmxvY2FsQ2VudGVyLCB0aGlzLm1feGYucCk7XHJcbiAgICBiMlZlYzIuU3ViVlYodGhpcy5tX3N3ZWVwLmMsIHRoaXMubV94Zi5wLCB0aGlzLm1feGYucCk7XHJcbiAgfVxyXG5cclxuICAvLyBUaGlzIGlzIHVzZWQgdG8gcHJldmVudCBjb25uZWN0ZWQgYm9kaWVzIGZyb20gY29sbGlkaW5nLlxyXG4gIC8vIEl0IG1heSBsaWUsIGRlcGVuZGluZyBvbiB0aGUgY29sbGlkZUNvbm5lY3RlZCBmbGFnLlxyXG4gIFNob3VsZENvbGxpZGUob3RoZXI6IGIyQm9keSk6IGJvb2xlYW4ge1xyXG4gICAgLy8gQXQgbGVhc3Qgb25lIGJvZHkgc2hvdWxkIGJlIGR5bmFtaWMgb3Iga2luZW1hdGljLlxyXG4gICAgaWYgKHRoaXMubV90eXBlID09PSBiMkJvZHlUeXBlLmIyX3N0YXRpY0JvZHkgJiYgb3RoZXIubV90eXBlID09PSBiMkJvZHlUeXBlLmIyX3N0YXRpY0JvZHkpIHtcclxuICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRoaXMuU2hvdWxkQ29sbGlkZUNvbm5lY3RlZChvdGhlcik7XHJcbiAgfVxyXG5cclxuICBTaG91bGRDb2xsaWRlQ29ubmVjdGVkKG90aGVyOiBiMkJvZHkpOiBib29sZWFuIHtcclxuICAgIC8vIERvZXMgYSBqb2ludCBwcmV2ZW50IGNvbGxpc2lvbj9cclxuICAgIGZvciAobGV0IGpuOiBiMkpvaW50RWRnZSB8IG51bGwgPSB0aGlzLm1fam9pbnRMaXN0OyBqbjsgam4gPSBqbi5uZXh0KSB7XHJcbiAgICAgIGlmIChqbi5vdGhlciA9PT0gb3RoZXIpIHtcclxuICAgICAgICBpZiAoIWpuLmpvaW50Lm1fY29sbGlkZUNvbm5lY3RlZCkge1xyXG4gICAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH1cclxuXHJcbiAgQWR2YW5jZShhbHBoYTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICAvLyBBZHZhbmNlIHRvIHRoZSBuZXcgc2FmZSB0aW1lLiBUaGlzIGRvZXNuJ3Qgc3luYyB0aGUgYnJvYWQtcGhhc2UuXHJcbiAgICB0aGlzLm1fc3dlZXAuQWR2YW5jZShhbHBoYSk7XHJcbiAgICB0aGlzLm1fc3dlZXAuYy5Db3B5KHRoaXMubV9zd2VlcC5jMCk7XHJcbiAgICB0aGlzLm1fc3dlZXAuYSA9IHRoaXMubV9zd2VlcC5hMDtcclxuICAgIHRoaXMubV94Zi5xLlNldEFuZ2xlKHRoaXMubV9zd2VlcC5hKTtcclxuICAgIGIyUm90Lk11bFJWKHRoaXMubV94Zi5xLCB0aGlzLm1fc3dlZXAubG9jYWxDZW50ZXIsIHRoaXMubV94Zi5wKTtcclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fc3dlZXAuYywgdGhpcy5tX3hmLnAsIHRoaXMubV94Zi5wKTtcclxuICB9XHJcblxyXG4gIC8vICNpZiBCMl9FTkFCTEVfQ09OVFJPTExFUlxyXG4gIEdldENvbnRyb2xsZXJMaXN0KCk6IGIyQ29udHJvbGxlckVkZ2UgfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fY29udHJvbGxlckxpc3Q7XHJcbiAgfVxyXG5cclxuICBHZXRDb250cm9sbGVyQ291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fY29udHJvbGxlckNvdW50O1xyXG4gIH1cclxuICAvLyAjZW5kaWZcclxufVxyXG4iXX0=