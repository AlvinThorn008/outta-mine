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
import { b2_baumgarte, b2_linearSlop, b2_maxLinearCorrection, b2_maxManifoldPoints, b2_toiBaumgarte, b2_velocityThreshold, b2Assert, b2MakeArray, } from '../../common/b2Settings';
import { b2Clamp, b2Mat22, b2Max, b2MaxInt, b2Min, b2Rot, b2Transform, b2Vec2, } from '../../common/b2Math';
import { b2WorldManifold, } from '../../collision/b2Collision';
import { b2TimeStep } from '../b2TimeStep';
// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
// #define B2_DEBUG_SOLVER 0
const g_blockSolve = false;
export class b2VelocityConstraintPoint {
    constructor() {
        this.rA = new b2Vec2();
        this.rB = new b2Vec2();
        this.normalImpulse = NaN;
        this.tangentImpulse = NaN;
        this.normalMass = NaN;
        this.tangentMass = NaN;
        this.velocityBias = NaN;
        this.normalImpulse = 0.0;
        this.tangentImpulse = 0.0;
        this.normalMass = 0.0;
        this.tangentMass = 0.0;
        this.velocityBias = 0.0;
    }
    static MakeArray(length) {
        return b2MakeArray(length, (i) => new b2VelocityConstraintPoint());
    }
}
export class b2ContactVelocityConstraint {
    constructor() {
        this.points = b2VelocityConstraintPoint.MakeArray(b2_maxManifoldPoints);
        this.normal = new b2Vec2();
        this.tangent = new b2Vec2();
        this.normalMass = new b2Mat22();
        this.K = new b2Mat22();
        this.indexA = 0;
        this.indexB = 0;
        this.invMassA = NaN;
        this.invMassB = NaN;
        this.invIA = NaN;
        this.invIB = NaN;
        this.friction = NaN;
        this.restitution = NaN;
        this.tangentSpeed = NaN;
        this.pointCount = 0;
        this.contactIndex = 0;
        // this.invMassA = 0.0;
        // this.invMassB = 0.0;
        // this.invIA = 0.0;
        // this.invIB = 0.0;
        // this.friction = 0.0;
        // this.restitution = 0.0;
        // this.tangentSpeed = 0.0;
    }
    static MakeArray(length) {
        return b2MakeArray(length, (i) => new b2ContactVelocityConstraint());
    }
}
export class b2ContactPositionConstraint {
    constructor() {
        this.localPoints = b2Vec2.MakeArray(b2_maxManifoldPoints);
        this.localNormal = new b2Vec2();
        this.localPoint = new b2Vec2();
        this.indexA = 0;
        this.indexB = 0;
        this.invMassA = NaN;
        this.invMassB = NaN;
        this.localCenterA = new b2Vec2();
        this.localCenterB = new b2Vec2();
        this.invIA = NaN;
        this.invIB = NaN;
        this.type = -1 /* e_unknown */;
        this.radiusA = NaN;
        this.radiusB = NaN;
        this.pointCount = 0;
        // TODO: maybe it for write first -read
        // this.invMassA = 0.0;
        // this.invMassB = 0.0;
        // this.invIA = 0.0;
        // this.invIB = 0.0;
        // this.radiusA = 0.0;
        // this.radiusB = 0.0;
    }
    static MakeArray(length) {
        return b2MakeArray(length, (i) => new b2ContactPositionConstraint());
    }
}
export class b2ContactSolverDef {
    constructor() {
        this.step = new b2TimeStep();
        this.contacts = [null];
        this.count = 0;
        this.positions = [null];
        this.velocities = [null];
    }
}
export class b2PositionSolverManifold {
    constructor() {
        this.normal = new b2Vec2();
        this.point = new b2Vec2();
        this.separation = NaN;
    }
    Initialize(pc, xfA, xfB, index) {
        const pointA = b2PositionSolverManifold.Initialize_s_pointA;
        const pointB = b2PositionSolverManifold.Initialize_s_pointB;
        const planePoint = b2PositionSolverManifold.Initialize_s_planePoint;
        const clipPoint = b2PositionSolverManifold.Initialize_s_clipPoint;
        !!B2_DEBUG && b2Assert(pc.pointCount > 0);
        if (pc.type === 0 /* e_circles */) {
            // b2Vec2 pointA = b2Mul(xfA, pc->localPoint);
            b2Transform.MulXV(xfA, pc.localPoint, pointA);
            // b2Vec2 pointB = b2Mul(xfB, pc->localPoints[0]);
            b2Transform.MulXV(xfB, pc.localPoints[0], pointB);
            // normal = pointB - pointA;
            // normal.Normalize();
            b2Vec2.SubVV(pointB, pointA, this.normal).SelfNormalize();
            // point = 0.5f * (pointA + pointB);
            b2Vec2.MidVV(pointA, pointB, this.point);
            // separation = b2Dot(pointB - pointA, normal) - pc->radius;
            this.separation =
                b2Vec2.DotVV(b2Vec2.SubVV(pointB, pointA, b2Vec2.s_t0), this.normal) -
                    pc.radiusA -
                    pc.radiusB;
        }
        else if (pc.type === 1 /* e_faceA */) {
            // normal = b2Mul(xfA.q, pc->localNormal);
            b2Rot.MulRV(xfA.q, pc.localNormal, this.normal);
            // b2Vec2 planePoint = b2Mul(xfA, pc->localPoint);
            b2Transform.MulXV(xfA, pc.localPoint, planePoint);
            // b2Vec2 clipPoint = b2Mul(xfB, pc->localPoints[index]);
            b2Transform.MulXV(xfB, pc.localPoints[index], clipPoint);
            // separation = b2Dot(clipPoint - planePoint, normal) - pc->radius;
            this.separation =
                b2Vec2.DotVV(b2Vec2.SubVV(clipPoint, planePoint, b2Vec2.s_t0), this.normal) -
                    pc.radiusA -
                    pc.radiusB;
            // point = clipPoint;
            this.point.Copy(clipPoint);
        }
        else if (pc.type === 2 /* e_faceB */) {
            // normal = b2Mul(xfB.q, pc->localNormal);
            b2Rot.MulRV(xfB.q, pc.localNormal, this.normal);
            // b2Vec2 planePoint = b2Mul(xfB, pc->localPoint);
            b2Transform.MulXV(xfB, pc.localPoint, planePoint);
            // b2Vec2 clipPoint = b2Mul(xfA, pc->localPoints[index]);
            b2Transform.MulXV(xfA, pc.localPoints[index], clipPoint);
            // separation = b2Dot(clipPoint - planePoint, normal) - pc->radius;
            this.separation =
                b2Vec2.DotVV(b2Vec2.SubVV(clipPoint, planePoint, b2Vec2.s_t0), this.normal) -
                    pc.radiusA -
                    pc.radiusB;
            // point = clipPoint;
            this.point.Copy(clipPoint);
            // Ensure normal points from A to B
            // normal = -normal;
            this.normal.SelfNeg();
        }
    }
}
b2PositionSolverManifold.Initialize_s_pointA = new b2Vec2();
b2PositionSolverManifold.Initialize_s_pointB = new b2Vec2();
b2PositionSolverManifold.Initialize_s_planePoint = new b2Vec2();
b2PositionSolverManifold.Initialize_s_clipPoint = new b2Vec2();
export class b2ContactSolver {
    constructor() {
        this.m_step = new b2TimeStep();
        this.m_positions = [null];
        this.m_velocities = [null];
        this.m_positionConstraints = b2ContactPositionConstraint.MakeArray(1024); // TODO: b2Settings
        this.m_velocityConstraints = b2ContactVelocityConstraint.MakeArray(1024); // TODO: b2Settings
        this.m_contacts = [null];
        this.m_count = 0;
    }
    Initialize(def) {
        this.m_step.Copy(def.step);
        this.m_count = def.count;
        // TODO:
        if (this.m_positionConstraints.length < this.m_count) {
            const new_length = b2MaxInt(this.m_positionConstraints.length << 1, this.m_count);
            while (this.m_positionConstraints.length < new_length) {
                this.m_positionConstraints.push(new b2ContactPositionConstraint());
            }
        }
        // TODO:
        if (this.m_velocityConstraints.length < this.m_count) {
            const new_length = b2MaxInt(this.m_velocityConstraints.length << 1, this.m_count);
            while (this.m_velocityConstraints.length < new_length) {
                this.m_velocityConstraints.push(new b2ContactVelocityConstraint());
            }
        }
        this.m_positions = def.positions;
        this.m_velocities = def.velocities;
        this.m_contacts = def.contacts;
        this.Initialize2();
        return this;
    }
    Initialize2() {
        // Initialize position independent portions of the constraints.
        for (let i = 0; i < this.m_count; ++i) {
            const contact = this.m_contacts[i];
            const fixtureA = contact.m_fixtureA;
            const fixtureB = contact.m_fixtureB;
            const radiusA = fixtureA._shapeRadius;
            const radiusB = fixtureB._shapeRadius;
            const bodyA = fixtureA.GetBody();
            const bodyB = fixtureB.GetBody();
            const manifold = contact.GetManifold();
            const pointCount = manifold.pointCount;
            !!B2_DEBUG && b2Assert(pointCount > 0);
            const vc = this.m_velocityConstraints[i];
            vc.friction = contact.m_friction;
            vc.restitution = contact.m_restitution;
            vc.tangentSpeed = contact.m_tangentSpeed;
            vc.indexA = bodyA.m_islandIndex;
            vc.indexB = bodyB.m_islandIndex;
            vc.invMassA = bodyA.m_invMass;
            vc.invMassB = bodyB.m_invMass;
            vc.invIA = bodyA.m_invI;
            vc.invIB = bodyB.m_invI;
            vc.contactIndex = i;
            vc.pointCount = pointCount;
            vc.K.SetZero();
            vc.normalMass.SetZero();
            const pc = this.m_positionConstraints[i];
            pc.indexA = bodyA.m_islandIndex;
            pc.indexB = bodyB.m_islandIndex;
            pc.invMassA = bodyA.m_invMass;
            pc.invMassB = bodyB.m_invMass;
            pc.localCenterA.Copy(bodyA.m_sweep.localCenter);
            pc.localCenterB.Copy(bodyB.m_sweep.localCenter);
            pc.invIA = bodyA.m_invI;
            pc.invIB = bodyB.m_invI;
            pc.localNormal.Copy(manifold.localNormal);
            pc.localPoint.Copy(manifold.localPoint);
            pc.pointCount = pointCount;
            pc.radiusA = radiusA;
            pc.radiusB = radiusB;
            pc.type = manifold.type;
            for (let j = 0; j < pointCount; ++j) {
                const cp = manifold.points[j];
                const vcp = vc.points[j];
                if (this.m_step.warmStarting) {
                    vcp.normalImpulse = this.m_step.dtRatio * cp.normalImpulse;
                    vcp.tangentImpulse = this.m_step.dtRatio * cp.tangentImpulse;
                }
                else {
                    vcp.normalImpulse = 0.0;
                    vcp.tangentImpulse = 0.0;
                }
                vcp.rA.SetZero();
                vcp.rB.SetZero();
                vcp.normalMass = 0.0;
                vcp.tangentMass = 0.0;
                vcp.velocityBias = 0.0;
                pc.localPoints[j].Copy(cp.localPoint);
            }
        }
    }
    InitializeVelocityConstraints() {
        const xfA = b2ContactSolver.InitializeVelocityConstraints_s_xfA;
        const xfB = b2ContactSolver.InitializeVelocityConstraints_s_xfB;
        const worldManifold = b2ContactSolver.InitializeVelocityConstraints_s_worldManifold;
        const k_maxConditionNumber = 1000.0;
        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];
            const pc = this.m_positionConstraints[i];
            const radiusA = pc.radiusA;
            const radiusB = pc.radiusB;
            const manifold = this.m_contacts[vc.contactIndex].GetManifold();
            const indexA = vc.indexA;
            const indexB = vc.indexB;
            const mA = vc.invMassA;
            const mB = vc.invMassB;
            const iA = vc.invIA;
            const iB = vc.invIB;
            const localCenterA = pc.localCenterA;
            const localCenterB = pc.localCenterB;
            const cA = this.m_positions[indexA].c;
            const aA = this.m_positions[indexA].a;
            const vA = this.m_velocities[indexA].v;
            const wA = this.m_velocities[indexA].w;
            const cB = this.m_positions[indexB].c;
            const aB = this.m_positions[indexB].a;
            const vB = this.m_velocities[indexB].v;
            const wB = this.m_velocities[indexB].w;
            !!B2_DEBUG && b2Assert(manifold.pointCount > 0);
            xfA.q.SetAngle(aA);
            xfB.q.SetAngle(aB);
            b2Vec2.SubVV(cA, b2Rot.MulRV(xfA.q, localCenterA, b2Vec2.s_t0), xfA.p);
            b2Vec2.SubVV(cB, b2Rot.MulRV(xfB.q, localCenterB, b2Vec2.s_t0), xfB.p);
            worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);
            vc.normal.Copy(worldManifold.normal);
            b2Vec2.CrossVOne(vc.normal, vc.tangent); // compute from normal
            const pointCount = vc.pointCount;
            for (let j = 0; j < pointCount; ++j) {
                const vcp = vc.points[j];
                // vcp->rA = worldManifold.points[j] - cA;
                b2Vec2.SubVV(worldManifold.points[j], cA, vcp.rA);
                // vcp->rB = worldManifold.points[j] - cB;
                b2Vec2.SubVV(worldManifold.points[j], cB, vcp.rB);
                const rnA = b2Vec2.CrossVV(vcp.rA, vc.normal);
                const rnB = b2Vec2.CrossVV(vcp.rB, vc.normal);
                const kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                vcp.normalMass = kNormal > 0 ? 1 / kNormal : 0;
                // b2Vec2 tangent = b2Cross(vc->normal, 1.0f);
                const tangent = vc.tangent; // precomputed from normal
                const rtA = b2Vec2.CrossVV(vcp.rA, tangent);
                const rtB = b2Vec2.CrossVV(vcp.rB, tangent);
                const kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                vcp.tangentMass = kTangent > 0 ? 1 / kTangent : 0;
                // Setup a velocity bias for restitution.
                vcp.velocityBias = 0;
                // float32 vRel = b2Dot(vc->normal, vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA));
                const vRel = b2Vec2.DotVV(vc.normal, b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, vcp.rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, vcp.rA, b2Vec2.s_t1), b2Vec2.s_t0));
                if (vRel < -b2_velocityThreshold) {
                    vcp.velocityBias += -vc.restitution * vRel;
                }
            }
            // If we have two points, then prepare the block solver.
            if (vc.pointCount === 2 && g_blockSolve) {
                const vcp1 = vc.points[0];
                const vcp2 = vc.points[1];
                const rn1A = b2Vec2.CrossVV(vcp1.rA, vc.normal);
                const rn1B = b2Vec2.CrossVV(vcp1.rB, vc.normal);
                const rn2A = b2Vec2.CrossVV(vcp2.rA, vc.normal);
                const rn2B = b2Vec2.CrossVV(vcp2.rB, vc.normal);
                const k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                const k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                const k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
                // Ensure a reasonable condition number.
                // float32 k_maxConditionNumber = 1000.0f;
                if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
                    // K is safe to invert.
                    vc.K.ex.Set(k11, k12);
                    vc.K.ey.Set(k12, k22);
                    vc.K.GetInverse(vc.normalMass);
                }
                else {
                    // The constraints are redundant, just use one.
                    // TODO_ERIN use deepest?
                    vc.pointCount = 1;
                }
            }
        }
    }
    WarmStart() {
        const P = b2ContactSolver.WarmStart_s_P;
        // Warm start.
        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];
            const indexA = vc.indexA;
            const indexB = vc.indexB;
            const mA = vc.invMassA;
            const iA = vc.invIA;
            const mB = vc.invMassB;
            const iB = vc.invIB;
            const pointCount = vc.pointCount;
            const vA = this.m_velocities[indexA].v;
            let wA = this.m_velocities[indexA].w;
            const vB = this.m_velocities[indexB].v;
            let wB = this.m_velocities[indexB].w;
            const normal = vc.normal;
            // b2Vec2 tangent = b2Cross(normal, 1.0f);
            const tangent = vc.tangent; // precomputed from normal
            for (let j = 0; j < pointCount; ++j) {
                const vcp = vc.points[j];
                // b2Vec2 P = vcp->normalImpulse * normal + vcp->tangentImpulse * tangent;
                b2Vec2.AddVV(b2Vec2.MulSV(vcp.normalImpulse, normal, b2Vec2.s_t0), b2Vec2.MulSV(vcp.tangentImpulse, tangent, b2Vec2.s_t1), P);
                // wA -= iA * b2Cross(vcp->rA, P);
                wA -= iA * b2Vec2.CrossVV(vcp.rA, P);
                // vA -= mA * P;
                vA.SelfMulSub(mA, P);
                // wB += iB * b2Cross(vcp->rB, P);
                wB += iB * b2Vec2.CrossVV(vcp.rB, P);
                // vB += mB * P;
                vB.SelfMulAdd(mB, P);
            }
            // this.m_velocities[indexA].v = vA;
            this.m_velocities[indexA].w = wA;
            // this.m_velocities[indexB].v = vB;
            this.m_velocities[indexB].w = wB;
        }
    }
    SolveVelocityConstraints() {
        const dv = b2ContactSolver.SolveVelocityConstraints_s_dv;
        const dv1 = b2ContactSolver.SolveVelocityConstraints_s_dv1;
        const dv2 = b2ContactSolver.SolveVelocityConstraints_s_dv2;
        const P = b2ContactSolver.SolveVelocityConstraints_s_P;
        const a = b2ContactSolver.SolveVelocityConstraints_s_a;
        const b = b2ContactSolver.SolveVelocityConstraints_s_b;
        const x = b2ContactSolver.SolveVelocityConstraints_s_x;
        const d = b2ContactSolver.SolveVelocityConstraints_s_d;
        const P1 = b2ContactSolver.SolveVelocityConstraints_s_P1;
        const P2 = b2ContactSolver.SolveVelocityConstraints_s_P2;
        const P1P2 = b2ContactSolver.SolveVelocityConstraints_s_P1P2;
        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];
            const indexA = vc.indexA;
            const indexB = vc.indexB;
            const mA = vc.invMassA;
            const iA = vc.invIA;
            const mB = vc.invMassB;
            const iB = vc.invIB;
            const pointCount = vc.pointCount;
            const vA = this.m_velocities[indexA].v;
            let wA = this.m_velocities[indexA].w;
            const vB = this.m_velocities[indexB].v;
            let wB = this.m_velocities[indexB].w;
            // b2Vec2 normal = vc->normal;
            const normal = vc.normal;
            // b2Vec2 tangent = b2Cross(normal, 1.0f);
            const tangent = vc.tangent; // precomputed from normal
            const friction = vc.friction;
            !!B2_DEBUG && b2Assert(pointCount === 1 || pointCount === 2);
            // Solve tangent constraints first because non-penetration is more important
            // than friction.
            for (let j = 0; j < pointCount; ++j) {
                const vcp = vc.points[j];
                // Relative velocity at contact
                // b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);
                b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, vcp.rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, vcp.rA, b2Vec2.s_t1), dv);
                // Compute tangent force
                // float32 vt = b2Dot(dv, tangent) - vc->tangentSpeed;
                const vt = b2Vec2.DotVV(dv, tangent) - vc.tangentSpeed;
                let lambda = vcp.tangentMass * -vt;
                // b2Clamp the accumulated force
                const maxFriction = friction * vcp.normalImpulse;
                const newImpulse = b2Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
                lambda = newImpulse - vcp.tangentImpulse;
                vcp.tangentImpulse = newImpulse;
                // Apply contact impulse
                // b2Vec2 P = lambda * tangent;
                b2Vec2.MulSV(lambda, tangent, P);
                // vA -= mA * P;
                vA.SelfMulSub(mA, P);
                // wA -= iA * b2Cross(vcp->rA, P);
                wA -= iA * b2Vec2.CrossVV(vcp.rA, P);
                // vB += mB * P;
                vB.SelfMulAdd(mB, P);
                // wB += iB * b2Cross(vcp->rB, P);
                wB += iB * b2Vec2.CrossVV(vcp.rB, P);
            }
            // Solve normal constraints
            if (vc.pointCount === 1 || g_blockSolve === false) {
                for (let j = 0; j < pointCount; ++j) {
                    const vcp = vc.points[j];
                    // Relative velocity at contact
                    // b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);
                    b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, vcp.rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, vcp.rA, b2Vec2.s_t1), dv);
                    // Compute normal impulse
                    // float32 vn = b2Dot(dv, normal);
                    const vn = b2Vec2.DotVV(dv, normal);
                    let lambda = -vcp.normalMass * (vn - vcp.velocityBias);
                    // b2Clamp the accumulated impulse
                    // float32 newImpulse = b2Max(vcp->normalImpulse + lambda, 0.0f);
                    const newImpulse = b2Max(vcp.normalImpulse + lambda, 0);
                    lambda = newImpulse - vcp.normalImpulse;
                    vcp.normalImpulse = newImpulse;
                    // Apply contact impulse
                    // b2Vec2 P = lambda * normal;
                    b2Vec2.MulSV(lambda, normal, P);
                    // vA -= mA * P;
                    vA.SelfMulSub(mA, P);
                    // wA -= iA * b2Cross(vcp->rA, P);
                    wA -= iA * b2Vec2.CrossVV(vcp.rA, P);
                    // vB += mB * P;
                    vB.SelfMulAdd(mB, P);
                    // wB += iB * b2Cross(vcp->rB, P);
                    wB += iB * b2Vec2.CrossVV(vcp.rB, P);
                }
            }
            else {
                // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                // Build the mini LCP for this contact patch
                //
                // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                //
                // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                // b = vn0 - velocityBias
                //
                // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                // solution that satisfies the problem is chosen.
                //
                // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                //
                // Substitute:
                //
                // x = a + d
                //
                // a := old total impulse
                // x := new total impulse
                // d := incremental impulse
                //
                // For the current iteration we extend the formula for the incremental impulse
                // to compute the new total impulse:
                //
                // vn = A * d + b
                //    = A * (x - a) + b
                //    = A * x + b - A * a
                //    = A * x + b'
                // b' = b - A * a;
                const cp1 = vc.points[0];
                const cp2 = vc.points[1];
                // b2Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
                a.Set(cp1.normalImpulse, cp2.normalImpulse);
                !!B2_DEBUG && b2Assert(a.x >= 0 && a.y >= 0);
                // Relative velocity at contact
                // b2Vec2 dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
                b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, cp1.rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, cp1.rA, b2Vec2.s_t1), dv1);
                // b2Vec2 dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);
                b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, cp2.rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, cp2.rA, b2Vec2.s_t1), dv2);
                // Compute normal velocity
                // float32 vn1 = b2Dot(dv1, normal);
                let vn1 = b2Vec2.DotVV(dv1, normal);
                // float32 vn2 = b2Dot(dv2, normal);
                let vn2 = b2Vec2.DotVV(dv2, normal);
                // b2Vec2 b;
                b.x = vn1 - cp1.velocityBias;
                b.y = vn2 - cp2.velocityBias;
                // Compute b'
                // b -= b2Mul(vc->K, a);
                b.SelfSub(b2Mat22.MulMV(vc.K, a, b2Vec2.s_t0));
                /*
                        #if B2_DEBUG_SOLVER === 1
                        const k_errorTol: number = 0.001;
                        #endif
                        */
                for (;;) {
                    //
                    // Case 1: vn = 0
                    //
                    // 0 = A * x + b'
                    //
                    // Solve for x:
                    //
                    // x = - inv(A) * b'
                    //
                    // b2Vec2 x = - b2Mul(vc->normalMass, b);
                    b2Mat22.MulMV(vc.normalMass, b, x).SelfNeg();
                    if (x.x >= 0 && x.y >= 0) {
                        // Get the incremental impulse
                        // b2Vec2 d = x - a;
                        b2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // b2Vec2 P1 = d.x * normal;
                        b2Vec2.MulSV(d.x, normal, P1);
                        // b2Vec2 P2 = d.y * normal;
                        b2Vec2.MulSV(d.y, normal, P2);
                        b2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
                        wA -= iA * (b2Vec2.CrossVV(cp1.rA, P1) + b2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
                        wB += iB * (b2Vec2.CrossVV(cp1.rB, P1) + b2Vec2.CrossVV(cp2.rB, P2));
                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        /*
                                    #if B2_DEBUG_SOLVER === 1
                                    // Postconditions
                                    dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
                                    dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);
            
                                    // Compute normal velocity
                                    vn1 = b2Dot(dv1, normal);
                                    vn2 = b2Dot(dv2, normal);
            
                                    b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
                                    b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
                                    #endif
                                    */
                        break;
                    }
                    //
                    // Case 2: vn1 = 0 and x2 = 0
                    //
                    //   0 = a11 * x1 + a12 * 0 + b1'
                    // vn2 = a21 * x1 + a22 * 0 + b2'
                    //
                    x.x = -cp1.normalMass * b.x;
                    x.y = 0;
                    vn1 = 0;
                    vn2 = vc.K.ex.y * x.x + b.y;
                    if (x.x >= 0 && vn2 >= 0) {
                        // Get the incremental impulse
                        // b2Vec2 d = x - a;
                        b2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // b2Vec2 P1 = d.x * normal;
                        b2Vec2.MulSV(d.x, normal, P1);
                        // b2Vec2 P2 = d.y * normal;
                        b2Vec2.MulSV(d.y, normal, P2);
                        b2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
                        wA -= iA * (b2Vec2.CrossVV(cp1.rA, P1) + b2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
                        wB += iB * (b2Vec2.CrossVV(cp1.rB, P1) + b2Vec2.CrossVV(cp2.rB, P2));
                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        /*
                                    #if B2_DEBUG_SOLVER === 1
                                    // Postconditions
                                    dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
            
                                    // Compute normal velocity
                                    vn1 = b2Dot(dv1, normal);
            
                                    b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
                                    #endif
                                    */
                        break;
                    }
                    //
                    // Case 3: vn2 = 0 and x1 = 0
                    //
                    // vn1 = a11 * 0 + a12 * x2 + b1'
                    //   0 = a21 * 0 + a22 * x2 + b2'
                    //
                    x.x = 0;
                    x.y = -cp2.normalMass * b.y;
                    vn1 = vc.K.ey.x * x.y + b.x;
                    vn2 = 0;
                    if (x.y >= 0 && vn1 >= 0) {
                        // Resubstitute for the incremental impulse
                        // b2Vec2 d = x - a;
                        b2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // b2Vec2 P1 = d.x * normal;
                        b2Vec2.MulSV(d.x, normal, P1);
                        // b2Vec2 P2 = d.y * normal;
                        b2Vec2.MulSV(d.y, normal, P2);
                        b2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
                        wA -= iA * (b2Vec2.CrossVV(cp1.rA, P1) + b2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
                        wB += iB * (b2Vec2.CrossVV(cp1.rB, P1) + b2Vec2.CrossVV(cp2.rB, P2));
                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        /*
                                    #if B2_DEBUG_SOLVER === 1
                                    // Postconditions
                                    dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);
            
                                    // Compute normal velocity
                                    vn2 = b2Dot(dv2, normal);
            
                                    b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
                                    #endif
                                    */
                        break;
                    }
                    //
                    // Case 4: x1 = 0 and x2 = 0
                    //
                    // vn1 = b1
                    // vn2 = b2;
                    x.x = 0;
                    x.y = 0;
                    vn1 = b.x;
                    vn2 = b.y;
                    if (vn1 >= 0 && vn2 >= 0) {
                        // Resubstitute for the incremental impulse
                        // b2Vec2 d = x - a;
                        b2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // b2Vec2 P1 = d.x * normal;
                        b2Vec2.MulSV(d.x, normal, P1);
                        // b2Vec2 P2 = d.y * normal;
                        b2Vec2.MulSV(d.y, normal, P2);
                        b2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
                        wA -= iA * (b2Vec2.CrossVV(cp1.rA, P1) + b2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
                        wB += iB * (b2Vec2.CrossVV(cp1.rB, P1) + b2Vec2.CrossVV(cp2.rB, P2));
                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        break;
                    }
                    // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                    break;
                }
            }
            // this.m_velocities[indexA].v = vA;
            this.m_velocities[indexA].w = wA;
            // this.m_velocities[indexB].v = vB;
            this.m_velocities[indexB].w = wB;
        }
    }
    StoreImpulses() {
        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];
            const manifold = this.m_contacts[vc.contactIndex].GetManifold();
            for (let j = 0; j < vc.pointCount; ++j) {
                manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
                manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
            }
        }
    }
    SolvePositionConstraints() {
        const xfA = b2ContactSolver.SolvePositionConstraints_s_xfA;
        const xfB = b2ContactSolver.SolvePositionConstraints_s_xfB;
        const psm = b2ContactSolver.SolvePositionConstraints_s_psm;
        const rA = b2ContactSolver.SolvePositionConstraints_s_rA;
        const rB = b2ContactSolver.SolvePositionConstraints_s_rB;
        const P = b2ContactSolver.SolvePositionConstraints_s_P;
        let minSeparation = 0;
        for (let i = 0; i < this.m_count; ++i) {
            const pc = this.m_positionConstraints[i];
            const indexA = pc.indexA;
            const indexB = pc.indexB;
            const localCenterA = pc.localCenterA;
            const mA = pc.invMassA;
            const iA = pc.invIA;
            const localCenterB = pc.localCenterB;
            const mB = pc.invMassB;
            const iB = pc.invIB;
            const pointCount = pc.pointCount;
            const cA = this.m_positions[indexA].c;
            let aA = this.m_positions[indexA].a;
            const cB = this.m_positions[indexB].c;
            let aB = this.m_positions[indexB].a;
            // Solve normal constraints
            for (let j = 0; j < pointCount; ++j) {
                xfA.q.SetAngle(aA);
                xfB.q.SetAngle(aB);
                b2Vec2.SubVV(cA, b2Rot.MulRV(xfA.q, localCenterA, b2Vec2.s_t0), xfA.p);
                b2Vec2.SubVV(cB, b2Rot.MulRV(xfB.q, localCenterB, b2Vec2.s_t0), xfB.p);
                psm.Initialize(pc, xfA, xfB, j);
                const normal = psm.normal;
                const point = psm.point;
                const separation = psm.separation;
                // b2Vec2 rA = point - cA;
                b2Vec2.SubVV(point, cA, rA);
                // b2Vec2 rB = point - cB;
                b2Vec2.SubVV(point, cB, rB);
                // Track max constraint error.
                minSeparation = b2Min(minSeparation, separation);
                // Prevent large corrections and allow slop.
                const C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0);
                // Compute the effective mass.
                // float32 rnA = b2Cross(rA, normal);
                const rnA = b2Vec2.CrossVV(rA, normal);
                // float32 rnB = b2Cross(rB, normal);
                const rnB = b2Vec2.CrossVV(rB, normal);
                // float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                const K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                // Compute normal impulse
                const impulse = K > 0 ? -C / K : 0;
                // b2Vec2 P = impulse * normal;
                b2Vec2.MulSV(impulse, normal, P);
                // cA -= mA * P;
                cA.SelfMulSub(mA, P);
                // aA -= iA * b2Cross(rA, P);
                aA -= iA * b2Vec2.CrossVV(rA, P);
                // cB += mB * P;
                cB.SelfMulAdd(mB, P);
                // aB += iB * b2Cross(rB, P);
                aB += iB * b2Vec2.CrossVV(rB, P);
            }
            // this.m_positions[indexA].c = cA;
            this.m_positions[indexA].a = aA;
            // this.m_positions[indexB].c = cB;
            this.m_positions[indexB].a = aB;
        }
        // We can't expect minSpeparation >= -b2_linearSlop because we don't
        // push the separation above -b2_linearSlop.
        return minSeparation > -3 * b2_linearSlop;
    }
    SolveTOIPositionConstraints(toiIndexA, toiIndexB) {
        const xfA = b2ContactSolver.SolveTOIPositionConstraints_s_xfA;
        const xfB = b2ContactSolver.SolveTOIPositionConstraints_s_xfB;
        const psm = b2ContactSolver.SolveTOIPositionConstraints_s_psm;
        const rA = b2ContactSolver.SolveTOIPositionConstraints_s_rA;
        const rB = b2ContactSolver.SolveTOIPositionConstraints_s_rB;
        const P = b2ContactSolver.SolveTOIPositionConstraints_s_P;
        let minSeparation = 0;
        for (let i = 0; i < this.m_count; ++i) {
            const pc = this.m_positionConstraints[i];
            const indexA = pc.indexA;
            const indexB = pc.indexB;
            const localCenterA = pc.localCenterA;
            const localCenterB = pc.localCenterB;
            const pointCount = pc.pointCount;
            let mA = 0;
            let iA = 0;
            if (indexA === toiIndexA || indexA === toiIndexB) {
                mA = pc.invMassA;
                iA = pc.invIA;
            }
            let mB = 0;
            let iB = 0;
            if (indexB === toiIndexA || indexB === toiIndexB) {
                mB = pc.invMassB;
                iB = pc.invIB;
            }
            const cA = this.m_positions[indexA].c;
            let aA = this.m_positions[indexA].a;
            const cB = this.m_positions[indexB].c;
            let aB = this.m_positions[indexB].a;
            // Solve normal constraints
            for (let j = 0; j < pointCount; ++j) {
                xfA.q.SetAngle(aA);
                xfB.q.SetAngle(aB);
                b2Vec2.SubVV(cA, b2Rot.MulRV(xfA.q, localCenterA, b2Vec2.s_t0), xfA.p);
                b2Vec2.SubVV(cB, b2Rot.MulRV(xfB.q, localCenterB, b2Vec2.s_t0), xfB.p);
                psm.Initialize(pc, xfA, xfB, j);
                const normal = psm.normal;
                const point = psm.point;
                const separation = psm.separation;
                // b2Vec2 rA = point - cA;
                b2Vec2.SubVV(point, cA, rA);
                // b2Vec2 rB = point - cB;
                b2Vec2.SubVV(point, cB, rB);
                // Track max constraint error.
                minSeparation = b2Min(minSeparation, separation);
                // Prevent large corrections and allow slop.
                const C = b2Clamp(b2_toiBaumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0);
                // Compute the effective mass.
                // float32 rnA = b2Cross(rA, normal);
                const rnA = b2Vec2.CrossVV(rA, normal);
                // float32 rnB = b2Cross(rB, normal);
                const rnB = b2Vec2.CrossVV(rB, normal);
                // float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                const K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                // Compute normal impulse
                const impulse = K > 0 ? -C / K : 0;
                // b2Vec2 P = impulse * normal;
                b2Vec2.MulSV(impulse, normal, P);
                // cA -= mA * P;
                cA.SelfMulSub(mA, P);
                // aA -= iA * b2Cross(rA, P);
                aA -= iA * b2Vec2.CrossVV(rA, P);
                // cB += mB * P;
                cB.SelfMulAdd(mB, P);
                // aB += iB * b2Cross(rB, P);
                aB += iB * b2Vec2.CrossVV(rB, P);
            }
            // this.m_positions[indexA].c = cA;
            this.m_positions[indexA].a = aA;
            // this.m_positions[indexB].c = cB;
            this.m_positions[indexB].a = aB;
        }
        // We can't expect minSpeparation >= -b2_linearSlop because we don't
        // push the separation above -b2_linearSlop.
        return minSeparation >= -1.5 * b2_linearSlop;
    }
}
b2ContactSolver.InitializeVelocityConstraints_s_xfA = new b2Transform();
b2ContactSolver.InitializeVelocityConstraints_s_xfB = new b2Transform();
b2ContactSolver.InitializeVelocityConstraints_s_worldManifold = new b2WorldManifold();
b2ContactSolver.WarmStart_s_P = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_dv = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_dv1 = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_dv2 = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_P = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_a = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_b = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_x = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_d = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_P1 = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_P2 = new b2Vec2();
b2ContactSolver.SolveVelocityConstraints_s_P1P2 = new b2Vec2();
b2ContactSolver.SolvePositionConstraints_s_xfA = new b2Transform();
b2ContactSolver.SolvePositionConstraints_s_xfB = new b2Transform();
b2ContactSolver.SolvePositionConstraints_s_psm = new b2PositionSolverManifold();
b2ContactSolver.SolvePositionConstraints_s_rA = new b2Vec2();
b2ContactSolver.SolvePositionConstraints_s_rB = new b2Vec2();
b2ContactSolver.SolvePositionConstraints_s_P = new b2Vec2();
b2ContactSolver.SolveTOIPositionConstraints_s_xfA = new b2Transform();
b2ContactSolver.SolveTOIPositionConstraints_s_xfB = new b2Transform();
b2ContactSolver.SolveTOIPositionConstraints_s_psm = new b2PositionSolverManifold();
b2ContactSolver.SolveTOIPositionConstraints_s_rA = new b2Vec2();
b2ContactSolver.SolveTOIPositionConstraints_s_rB = new b2Vec2();
b2ContactSolver.SolveTOIPositionConstraints_s_P = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb250YWN0U29sdmVyLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vc3JjL2R5bmFtaWNzL2NvbnRhY3RzL2IyQ29udGFjdFNvbHZlci50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFDTCxZQUFZLEVBQ1osYUFBYSxFQUNiLHNCQUFzQixFQUN0QixvQkFBb0IsRUFDcEIsZUFBZSxFQUNmLG9CQUFvQixFQUNwQixRQUFRLEVBQ1IsV0FBVyxHQUNaLE1BQU0seUJBQXlCLENBQUM7QUFDakMsT0FBTyxFQUNMLE9BQU8sRUFDUCxPQUFPLEVBQ1AsS0FBSyxFQUNMLFFBQVEsRUFDUixLQUFLLEVBQ0wsS0FBSyxFQUNMLFdBQVcsRUFDWCxNQUFNLEdBQ1AsTUFBTSxxQkFBcUIsQ0FBQztBQUM3QixPQUFPLEVBSUwsZUFBZSxHQUNoQixNQUFNLDZCQUE2QixDQUFDO0FBSXJDLE9BQU8sRUFBYyxVQUFVLEVBQWMsTUFBTSxlQUFlLENBQUM7QUFFbkUsd0lBQXdJO0FBQ3hJLDRCQUE0QjtBQUU1QixNQUFNLFlBQVksR0FBRyxLQUFLLENBQUM7QUFFM0IsTUFBTSxPQUFPLHlCQUF5QjtJQVNwQztRQVJTLE9BQUUsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQ2xCLE9BQUUsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQzNCLGtCQUFhLEdBQUcsR0FBRyxDQUFDO1FBQ3BCLG1CQUFjLEdBQUcsR0FBRyxDQUFDO1FBQ3JCLGVBQVUsR0FBRyxHQUFHLENBQUM7UUFDakIsZ0JBQVcsR0FBRyxHQUFHLENBQUM7UUFDbEIsaUJBQVksR0FBRyxHQUFHLENBQUM7UUFHakIsSUFBSSxDQUFDLGFBQWEsR0FBRyxHQUFHLENBQUM7UUFDekIsSUFBSSxDQUFDLGNBQWMsR0FBRyxHQUFHLENBQUM7UUFDMUIsSUFBSSxDQUFDLFVBQVUsR0FBRyxHQUFHLENBQUM7UUFDdEIsSUFB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LEVBQUUsQ0FBQyxRQUFRLENBQUM7WUFDL0IsTUFBTSxFQUFFLEdBQVcsRUFBRSxDQUFDLEtBQUssQ0FBQztZQUM1QixNQUFNLEVBQUUsR0FBVyxFQUFFLENBQUMsS0FBSyxDQUFDO1lBQzVCLE1BQU0sWUFBWSxHQUFXLEVBQUUsQ0FBQyxZQUFZLENBQUM7WUFDN0MsTUFBTSxZQUFZLEdBQVcsRUFBRSxDQUFDLFlBQVksQ0FBQztZQUU3QyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsV0FBVyxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUM5QyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsV0FBVyxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUM5QyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsWUFBWSxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUMvQyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsWUFBWSxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUUvQyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsV0FBVyxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUM5QyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsV0FBVyxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUM5QyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsWUFBWSxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUMvQyxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsWUFBWSxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUUvQyxDQUFDLENBQUMsUUFBUSxJQUFJLFFBQVEsQ0FBQyxRQUFRLENBQUMsVUFBVSxHQUFHLENBQUMsQ0FBQyxDQUFDO1lBRWhELEdBQUcsQ0FBQyxDQUFDLENBQUMsUUFBUSxDQUFDLEVBQUUsQ0FBQyxDQUFDO1lBQ25CLEdBQUcsQ0FBQyxDQUFDLENBQUMsUUFBUSxDQUFDLEVBQUUsQ0FBQyxDQUFDO1lBQ25CLE1BQU0sQ0FB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QztBQUNuRCw4Q0FBOEIsR0FBRyxJQUFJLFdBQVcsRUFBRSxDQUFDO0FBQ25ELDhDQUE4QixHQUFHLElBQUksd0JBQXdCLEVBQUUsQ0FBQztBQUNoRSw2Q0FBNkIsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQzdDLDZDQUE2QixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDN0MsNENBQTRCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQWdHNUMsaURBQWlDLEdBQUcsSUFBSSxXQUFXLEVBQUUsQ0FBQztBQUN0RCxpREFBaUMsR0FBRyxJQUFJLFdBQVcsRUFBRSxDQUFDO0FBQ3RELGlEQUFpQyxHQUFHLElBQUksd0JBQXdCLEVBQUUsQ0FBQztBQUNuRSxnREFBZ0MsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQ2hELGdEQUFnQyxHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDaEQsK0NBQStCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDA5IEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHtcclxuICBiMl9iYXVtZ2FydGUsXHJcbiAgYjJfbGluZWFyU2xvcCxcclxuICBiMl9tYXhMaW5lYXJDb3JyZWN0aW9uLFxyXG4gIGIyX21heE1hbmlmb2xkUG9pbnRzLFxyXG4gIGIyX3RvaUJhdW1nYXJ0ZSxcclxuICBiMl92ZWxvY2l0eVRocmVzaG9sZCxcclxuICBiMkFzc2VydCxcclxuICBiMk1ha2VBcnJheSxcclxufSBmcm9tICcuLi8uLi9jb21tb24vYjJTZXR0aW5ncyc7XHJcbmltcG9ydCB7XHJcbiAgYjJDbGFtcCxcclxuICBiMk1hdDIyLFxyXG4gIGIyTWF4LFxyXG4gIGIyTWF4SW50LFxyXG4gIGIyTWluLFxyXG4gIGIyUm90LFxyXG4gIGIyVHJhbnNmb3JtLFxyXG4gIGIyVmVjMixcclxufSBmcm9tICcuLi8uLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHtcclxuICBiMk1hbmlmb2xkLFxyXG4gIGIyTWFuaWZvbGRQb2ludCxcclxuICBiMk1hbmlmb2xkVHlwZSxcclxuICBiMldvcmxkTWFuaWZvbGQsXHJcbn0gZnJvbSAnLi4vLi4vY29sbGlzaW9uL2IyQ29sbGlzaW9uJztcclxuaW1wb3J0IHsgYjJDb250YWN0IH0gZnJvbSAnLi9iMkNvbnRhY3QnO1xyXG5pbXBvcnQgeyBiMkJvZHkgfSBmcm9tICcuLi9iMkJvZHknO1xyXG5pbXBvcnQgeyBiMkZpeHR1cmUgfSBmcm9tICcuLi9iMkZpeHR1cmUnO1xyXG5pbXBvcnQgeyBiMlBvc2l0aW9uLCBiMlRpbWVTdGVwLCBiMlZlbG9jaXR5IH0gZnJvbSAnLi4vYjJUaW1lU3RlcCc7XHJcblxyXG4vLyBTb2x2ZXIgZGVidWdnaW5nIGlzIG5vcm1hbGx5IGRpc2FibGVkIGJlY2F1c2UgdGhlIGJsb2NrIHNvbHZlciBzb21ldGltZXMgaGFzIHRvIGRlYWwgd2l0aCBhIHBvb3JseSBjb25kaXRpb25lZCBlZmZlY3RpdmUgbWFzcyBtYXRyaXguXHJcbi8vICNkZWZpbmUgQjJfREVCVUdfU09MVkVSIDBcclxuXHJcbmNvbnN0IGdfYmxvY2tTb2x2ZSA9IGZhbHNlO1xyXG5cclxuZXhwb3J0IGNsYXNzIGIyVmVsb2NpdHlDb25zdHJhaW50UG9pbnQge1xyXG4gIHJlYWRvbmx5IHJBID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IHJCID0gbmV3IGIyVmVjMigpO1xyXG4gIG5vcm1hbEltcHVsc2UgPSBOYU47XHJcbiAgdGFuZ2VudEltcHVsc2UgPSBOYU47XHJcbiAgbm9ybWFsTWFzcyA9IE5hTjtcclxuICB0YW5nZW50TWFzcyA9IE5hTjtcclxuICB2ZWxvY2l0eUJpYXMgPSBOYU47XHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgdGhpcy5ub3JtYWxJbXB1bHNlID0gMC4wO1xyXG4gICAgdGhpcy50YW5nZW50SW1wdWxzZSA9IDAuMDtcclxuICAgIHRoaXMubm9ybWFsTWFzcyA9IDAuMDtcclxuICAgIHRoaXMudGFuZ2VudE1hc3MgPSAwLjA7XHJcbiAgICB0aGlzLnZlbG9jaXR5QmlhcyA9IDAuMDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBNYWtlQXJyYXkobGVuZ3RoOiBudW1iZXIpOiBiMlZlbG9jaXR5Q29uc3RyYWludFBvaW50W10ge1xyXG4gICAgcmV0dXJuIGIyTWFrZUFycmF5KGxlbmd0aCwgKGk6IG51bWJlcikgPT4gbmV3IGIyVmVsb2NpdHlDb25zdHJhaW50UG9pbnQoKSk7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJDb250YWN0VmVsb2NpdHlDb25zdHJhaW50IHtcclxuICByZWFkb25seSBwb2ludHM6IGIyVmVsb2NpdHlDb25zdHJhaW50UG9pbnRbXSA9IGIyVmVsb2NpdHlDb25zdHJhaW50UG9pbnQuTWFrZUFycmF5KFxyXG4gICAgYjJfbWF4TWFuaWZvbGRQb2ludHMsXHJcbiAgKTtcclxuICByZWFkb25seSBub3JtYWw6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSB0YW5nZW50OiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbm9ybWFsTWFzczogYjJNYXQyMiA9IG5ldyBiMk1hdDIyKCk7XHJcbiAgcmVhZG9ubHkgSzogYjJNYXQyMiA9IG5ldyBiMk1hdDIyKCk7XHJcbiAgaW5kZXhBID0gMDtcclxuICBpbmRleEIgPSAwO1xyXG4gIGludk1hc3NBID0gTmFOO1xyXG4gIGludk1hc3NCID0gTmFOO1xyXG4gIGludklBID0gTmFOO1xyXG4gIGludklCID0gTmFOO1xyXG4gIGZyaWN0aW9uID0gTmFOO1xyXG4gIHJlc3RpdHV0aW9uID0gTmFOO1xyXG4gIHRhbmdlbnRTcGVlZCA9IE5hTjtcclxuICBwb2ludENvdW50ID0gMDtcclxuICBjb250YWN0SW5kZXggPSAwO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIC8vIHRoaXMuaW52TWFzc0EgPSAwLjA7XHJcbiAgICAvLyB0aGlzLmludk1hc3NCID0gMC4wO1xyXG4gICAgLy8gdGhpcy5pbnZJQSA9IDAuMDtcclxuICAgIC8vIHRoaXMuaW52SUIgPSAwLjA7XHJcbiAgICAvLyB0aGlzLmZyaWN0aW9uID0gMC4wO1xyXG4gICAgLy8gdGhpcy5yZXN0aXR1dGlvbiA9IDAuMDtcclxuICAgIC8vIHRoaXMudGFuZ2VudFNwZWVkID0gMC4wO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE1ha2VBcnJheShsZW5ndGg6IG51bWJlcik6IGIyQ29udGFjdFZlbG9jaXR5Q29uc3RyYWludFtdIHtcclxuICAgIHJldHVybiBiMk1ha2VBcnJheShsZW5ndGgsIChpOiBudW1iZXIpID0+IG5ldyBiMkNvbnRhY3RWZWxvY2l0eUNvbnN0cmFpbnQoKSk7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY2xhc3MgYjJDb250YWN0UG9zaXRpb25Db25zdHJhaW50IHtcclxuICByZWFkb25seSBsb2NhbFBvaW50czogYjJWZWMyW10gPSBiMlZlYzIuTWFrZUFycmF5KGIyX21heE1hbmlmb2xkUG9pbnRzKTtcclxuICByZWFkb25seSBsb2NhbE5vcm1hbDogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IGxvY2FsUG9pbnQ6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBpbmRleEEgPSAwO1xyXG4gIGluZGV4QiA9IDA7XHJcbiAgaW52TWFzc0EgPSBOYU47XHJcbiAgaW52TWFzc0IgPSBOYU47XHJcbiAgcmVhZG9ubHkgbG9jYWxDZW50ZXJBID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IGxvY2FsQ2VudGVyQiA9IG5ldyBiMlZlYzIoKTtcclxuICBpbnZJQSA9IE5hTjtcclxuICBpbnZJQiA9IE5hTjtcclxuICB0eXBlID0gYjJNYW5pZm9sZFR5cGUuZV91bmtub3duO1xyXG4gIHJhZGl1c0EgPSBOYU47XHJcbiAgcmFkaXVzQiA9IE5hTjtcclxuICBwb2ludENvdW50ID0gMDtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICAvLyBUT0RPOiBtYXliZSBpdCBmb3Igd3JpdGUgZmlyc3QgLXJlYWRcclxuICAgIC8vIHRoaXMuaW52TWFzc0EgPSAwLjA7XHJcbiAgICAvLyB0aGlzLmludk1hc3NCID0gMC4wO1xyXG4gICAgLy8gdGhpcy5pbnZJQSA9IDAuMDtcclxuICAgIC8vIHRoaXMuaW52SUIgPSAwLjA7XHJcbiAgICAvLyB0aGlzLnJhZGl1c0EgPSAwLjA7XHJcbiAgICAvLyB0aGlzLnJhZGl1c0IgPSAwLjA7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTWFrZUFycmF5KGxlbmd0aDogbnVtYmVyKTogYjJDb250YWN0UG9zaXRpb25Db25zdHJhaW50W10ge1xyXG4gICAgcmV0dXJuIGIyTWFrZUFycmF5KGxlbmd0aCwgKGk6IG51bWJlcikgPT4gbmV3IGIyQ29udGFjdFBvc2l0aW9uQ29uc3RyYWludCgpKTtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMkNvbnRhY3RTb2x2ZXJEZWYge1xyXG4gIHJlYWRvbmx5IHN0ZXA6IGIyVGltZVN0ZXAgPSBuZXcgYjJUaW1lU3RlcCgpO1xyXG4gIGNvbnRhY3RzOiBiMkNvbnRhY3RbXSA9IChbbnVsbF0gYXMgdW5rbm93bikgYXMgYjJDb250YWN0W107XHJcbiAgY291bnQgPSAwO1xyXG4gIHBvc2l0aW9uczogYjJQb3NpdGlvbltdID0gKFtudWxsXSBhcyB1bmtub3duKSBhcyBiMlBvc2l0aW9uW107XHJcbiAgdmVsb2NpdGllczogYjJWZWxvY2l0eVtdID0gKFtudWxsXSBhcyB1bmtub3duKSBhcyBiMlZlbG9jaXR5W107XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlBvc2l0aW9uU29sdmVyTWFuaWZvbGQge1xyXG4gIHJlYWRvbmx5IG5vcm1hbDogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IHBvaW50OiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc2VwYXJhdGlvbiA9IE5hTjtcclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgSW5pdGlhbGl6ZV9zX3BvaW50QSA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBJbml0aWFsaXplX3NfcG9pbnRCID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIEluaXRpYWxpemVfc19wbGFuZVBvaW50ID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIEluaXRpYWxpemVfc19jbGlwUG9pbnQgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIEluaXRpYWxpemUoXHJcbiAgICBwYzogYjJDb250YWN0UG9zaXRpb25Db25zdHJhaW50LFxyXG4gICAgeGZBOiBiMlRyYW5zZm9ybSxcclxuICAgIHhmQjogYjJUcmFuc2Zvcm0sXHJcbiAgICBpbmRleDogbnVtYmVyLFxyXG4gICk6IHZvaWQge1xyXG4gICAgY29uc3QgcG9pbnRBOiBiMlZlYzIgPSBiMlBvc2l0aW9uU29sdmVyTWFuaWZvbGQuSW5pdGlhbGl6ZV9zX3BvaW50QTtcclxuICAgIGNvbnN0IHBvaW50QjogYjJWZWMyID0gYjJQb3NpdGlvblNvbHZlck1hbmlmb2xkLkluaXRpYWxpemVfc19wb2ludEI7XHJcbiAgICBjb25zdCBwbGFuZVBvaW50OiBiMlZlYzIgPSBiMlBvc2l0aW9uU29sdmVyTWFuaWZvbGQuSW5pdGlhbGl6ZV9zX3BsYW5lUG9pbnQ7XHJcbiAgICBjb25zdCBjbGlwUG9pbnQ6IGIyVmVjMiA9IGIyUG9zaXRpb25Tb2x2ZXJNYW5pZm9sZC5Jbml0aWFsaXplX3NfY2xpcFBvaW50O1xyXG5cclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQocGMucG9pbnRDb3VudCA+IDApO1xyXG5cclxuICAgIGlmIChwYy50eXBlID09PSBiMk1hbmlmb2xkVHlwZS5lX2NpcmNsZXMpIHtcclxuICAgICAgLy8gYjJWZWMyIHBvaW50QSA9IGIyTXVsKHhmQSwgcGMtPmxvY2FsUG9pbnQpO1xyXG4gICAgICBiMlRyYW5zZm9ybS5NdWxYVih4ZkEsIHBjLmxvY2FsUG9pbnQsIHBvaW50QSk7XHJcbiAgICAgIC8vIGIyVmVjMiBwb2ludEIgPSBiMk11bCh4ZkIsIHBjLT5sb2NhbFBvaW50c1swXSk7XHJcbiAgICAgIGIyVHJhbnNmb3JtLk11bFhWKHhmQiwgcGMubG9jYWxQb2ludHNbMF0sIHBvaW50Qik7XHJcbiAgICAgIC8vIG5vcm1hbCA9IHBvaW50QiAtIHBvaW50QTtcclxuICAgICAgLy8gbm9ybWFsLk5vcm1hbGl6ZSgpO1xyXG4gICAgICBiMlZlYzIuU3ViVlYocG9pbnRCLCBwb2ludEEsIHRoaXMubm9ybWFsKS5TZWxmTm9ybWFsaXplKCk7XHJcbiAgICAgIC8vIHBvaW50ID0gMC41ZiAqIChwb2ludEEgKyBwb2ludEIpO1xyXG4gICAgICBiMlZlYzIuTWlkVlYocG9pbnRBLCBwb2ludEIsIHRoaXMucG9pbnQpO1xyXG4gICAgICAvLyBzZXBhcmF0aW9uID0gYjJEb3QocG9pbnRCIC0gcG9pbnRBLCBub3JtYWwpIC0gcGMtPnJhZGl1cztcclxuICAgICAgdGhpcy5zZXBhcmF0aW9uID1cclxuICAgICAgICBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKHBvaW50QiwgcG9pbnRBLCBiMlZlYzIuc190MCksIHRoaXMubm9ybWFsKSAtXHJcbiAgICAgICAgcGMucmFkaXVzQSAtXHJcbiAgICAgICAgcGMucmFkaXVzQjtcclxuICAgIH0gZWxzZSBpZiAocGMudHlwZSA9PT0gYjJNYW5pZm9sZFR5cGUuZV9mYWNlQSkge1xyXG4gICAgICAvLyBub3JtYWwgPSBiMk11bCh4ZkEucSwgcGMtPmxvY2FsTm9ybWFsKTtcclxuICAgICAgYjJSb3QuTXVsUlYoeGZBLnEsIHBjLmxvY2FsTm9ybWFsLCB0aGlzLm5vcm1hbCk7XHJcbiAgICAgIC8vIGIyVmVjMiBwbGFuZVBvaW50ID0gYjJNdWwoeGZBLCBwYy0+bG9jYWxQb2ludCk7XHJcbiAgICAgIGIyVHJhbnNmb3JtLk11bFhWKHhmQSwgcGMubG9jYWxQb2ludCwgcGxhbmVQb2ludCk7XHJcblxyXG4gICAgICAvLyBiMlZlYzIgY2xpcFBvaW50ID0gYjJNdWwoeGZCLCBwYy0+bG9jYWxQb2ludHNbaW5kZXhdKTtcclxuICAgICAgYjJUcmFuc2Zvcm0uTXVsWFYoeGZCLCBwYy5sb2NhbFBvaW50c1tpbmRleF0sIGNsaXBQb2ludCk7XHJcbiAgICAgIC8vIHNlcGFyYXRpb24gPSBiMkRvdChjbGlwUG9pbnQgLSBwbGFuZVBvaW50LCBub3JtYWwpIC0gcGMtPnJhZGl1cztcclxuICAgICAgdGhpcy5zZXBhcmF0aW9uID1cclxuICAgICAgICBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKGNsaXBQb2ludCwgcGxhbmVQb2ludCwgYjJWZWMyLnNfdDApLCB0aGlzLm5vcm1hbCkgLVxyXG4gICAgICAgIHBjLnJhZGl1c0EgLVxyXG4gICAgICAgIHBjLnJhZGl1c0I7XHJcbiAgICAgIC8vIHBvaW50ID0gY2xpcFBvaW50O1xyXG4gICAgICB0aGlzLnBvaW50LkNvcHkoY2xpcFBvaW50KTtcclxuICAgIH0gZWxzZSBpZiAocGMudHlwZSA9PT0gYjJNYW5pZm9sZFR5cGUuZV9mYWNlQikge1xyXG4gICAgICAvLyBub3JtYWwgPSBiMk11bCh4ZkIucSwgcGMtPmxvY2FsTm9ybWFsKTtcclxuICAgICAgYjJSb3QuTXVsUlYoeGZCLnEsIHBjLmxvY2FsTm9ybWFsLCB0aGlzLm5vcm1hbCk7XHJcbiAgICAgIC8vIGIyVmVjMiBwbGFuZVBvaW50ID0gYjJNdWwoeGZCLCBwYy0+bG9jYWxQb2ludCk7XHJcbiAgICAgIGIyVHJhbnNmb3JtLk11bFhWKHhmQiwgcGMubG9jYWxQb2ludCwgcGxhbmVQb2ludCk7XHJcblxyXG4gICAgICAvLyBiMlZlYzIgY2xpcFBvaW50ID0gYjJNdWwoeGZBLCBwYy0+bG9jYWxQb2ludHNbaW5kZXhdKTtcclxuICAgICAgYjJUcmFuc2Zvcm0uTXVsWFYoeGZBLCBwYy5sb2NhbFBvaW50c1tpbmRleF0sIGNsaXBQb2ludCk7XHJcbiAgICAgIC8vIHNlcGFyYXRpb24gPSBiMkRvdChjbGlwUG9pbnQgLSBwbGFuZVBvaW50LCBub3JtYWwpIC0gcGMtPnJhZGl1cztcclxuICAgICAgdGhpcy5zZXBhcmF0aW9uID1cclxuICAgICAgICBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKGNsaXBQb2ludCwgcGxhbmVQb2ludCwgYjJWZWMyLnNfdDApLCB0aGlzLm5vcm1hbCkgLVxyXG4gICAgICAgIHBjLnJhZGl1c0EgLVxyXG4gICAgICAgIHBjLnJhZGl1c0I7XHJcbiAgICAgIC8vIHBvaW50ID0gY2xpcFBvaW50O1xyXG4gICAgICB0aGlzLnBvaW50LkNvcHkoY2xpcFBvaW50KTtcclxuXHJcbiAgICAgIC8vIEVuc3VyZSBub3JtYWwgcG9pbnRzIGZyb20gQSB0byBCXHJcbiAgICAgIC8vIG5vcm1hbCA9IC1ub3JtYWw7XHJcbiAgICAgIHRoaXMubm9ybWFsLlNlbGZOZWcoKTtcclxuICAgIH1cclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMkNvbnRhY3RTb2x2ZXIge1xyXG4gIHJlYWRvbmx5IG1fc3RlcDogYjJUaW1lU3RlcCA9IG5ldyBiMlRpbWVTdGVwKCk7XHJcbiAgbV9wb3NpdGlvbnM6IGIyUG9zaXRpb25bXSA9IChbbnVsbF0gYXMgdW5rbm93bikgYXMgYjJQb3NpdGlvbltdO1xyXG4gIG1fdmVsb2NpdGllczogYjJWZWxvY2l0eVtdID0gKFtudWxsXSBhcyB1bmtub3duKSBhcyBiMlZlbG9jaXR5W107XHJcbiAgcmVhZG9ubHkgbV9wb3NpdGlvbkNvbnN0cmFpbnRzOiBiMkNvbnRhY3RQb3NpdGlvbkNvbnN0cmFpbnRbXSA9IGIyQ29udGFjdFBvc2l0aW9uQ29uc3RyYWludC5NYWtlQXJyYXkoXHJcbiAgICAxMDI0LFxyXG4gICk7IC8vIFRPRE86IGIyU2V0dGluZ3NcclxuICByZWFkb25seSBtX3ZlbG9jaXR5Q29uc3RyYWludHM6IGIyQ29udGFjdFZlbG9jaXR5Q29uc3RyYWludFtdID0gYjJDb250YWN0VmVsb2NpdHlDb25zdHJhaW50Lk1ha2VBcnJheShcclxuICAgIDEwMjQsXHJcbiAgKTsgLy8gVE9ETzogYjJTZXR0aW5nc1xyXG4gIG1fY29udGFjdHM6IGIyQ29udGFjdFtdID0gKFtudWxsXSBhcyB1bmtub3duKSBhcyBiMkNvbnRhY3RbXTtcclxuICBtX2NvdW50ID0gMDtcclxuXHJcbiAgSW5pdGlhbGl6ZShkZWY6IGIyQ29udGFjdFNvbHZlckRlZik6IGIyQ29udGFjdFNvbHZlciB7XHJcbiAgICB0aGlzLm1fc3RlcC5Db3B5KGRlZi5zdGVwKTtcclxuICAgIHRoaXMubV9jb3VudCA9IGRlZi5jb3VudDtcclxuICAgIC8vIFRPRE86XHJcbiAgICBpZiAodGhpcy5tX3Bvc2l0aW9uQ29uc3RyYWludHMubGVuZ3RoIDwgdGhpcy5tX2NvdW50KSB7XHJcbiAgICAgIGNvbnN0IG5ld19sZW5ndGg6IG51bWJlciA9IGIyTWF4SW50KHRoaXMubV9wb3NpdGlvbkNvbnN0cmFpbnRzLmxlbmd0aCA8PCAxLCB0aGlzLm1fY291bnQpO1xyXG4gICAgICB3aGlsZSAodGhpcy5tX3Bvc2l0aW9uQ29uc3RyYWludHMubGVuZ3RoIDwgbmV3X2xlbmd0aCkge1xyXG4gICAgICAgIHRoaXMubV9wb3NpdGlvbkNvbnN0cmFpbnRzLnB1c2gobmV3IGIyQ29udGFjdFBvc2l0aW9uQ29uc3RyYWludCgpKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgLy8gVE9ETzpcclxuICAgIGlmICh0aGlzLm1fdmVsb2NpdHlDb25zdHJhaW50cy5sZW5ndGggPCB0aGlzLm1fY291bnQpIHtcclxuICAgICAgY29uc3QgbmV3X2xlbmd0aDogbnVtYmVyID0gYjJNYXhJbnQodGhpcy5tX3ZlbG9jaXR5Q29uc3RyYWludHMubGVuZ3RoIDw8IDEsIHRoaXMubV9jb3VudCk7XHJcbiAgICAgIHdoaWxlICh0aGlzLm1fdmVsb2NpdHlDb25zdHJhaW50cy5sZW5ndGggPCBuZXdfbGVuZ3RoKSB7XHJcbiAgICAgICAgdGhpcy5tX3ZlbG9jaXR5Q29uc3RyYWludHMucHVzaChuZXcgYjJDb250YWN0VmVsb2NpdHlDb25zdHJhaW50KCkpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fcG9zaXRpb25zID0gZGVmLnBvc2l0aW9ucztcclxuICAgIHRoaXMubV92ZWxvY2l0aWVzID0gZGVmLnZlbG9jaXRpZXM7XHJcbiAgICB0aGlzLm1fY29udGFjdHMgPSBkZWYuY29udGFjdHM7XHJcblxyXG4gICAgdGhpcy5Jbml0aWFsaXplMigpO1xyXG5cclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgSW5pdGlhbGl6ZTIoKSB7XHJcbiAgICAvLyBJbml0aWFsaXplIHBvc2l0aW9uIGluZGVwZW5kZW50IHBvcnRpb25zIG9mIHRoZSBjb25zdHJhaW50cy5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgY29udGFjdDogYjJDb250YWN0ID0gdGhpcy5tX2NvbnRhY3RzW2ldO1xyXG5cclxuICAgICAgY29uc3QgZml4dHVyZUE6IGIyRml4dHVyZSA9IGNvbnRhY3QubV9maXh0dXJlQTtcclxuICAgICAgY29uc3QgZml4dHVyZUI6IGIyRml4dHVyZSA9IGNvbnRhY3QubV9maXh0dXJlQjtcclxuICAgICAgY29uc3QgcmFkaXVzQTogbnVtYmVyID0gZml4dHVyZUEuX3NoYXBlUmFkaXVzO1xyXG4gICAgICBjb25zdCByYWRpdXNCOiBudW1iZXIgPSBmaXh0dXJlQi5fc2hhcGVSYWRpdXM7XHJcbiAgICAgIGNvbnN0IGJvZHlBOiBiMkJvZHkgPSBmaXh0dXJlQS5HZXRCb2R5KCk7XHJcbiAgICAgIGNvbnN0IGJvZHlCOiBiMkJvZHkgPSBmaXh0dXJlQi5HZXRCb2R5KCk7XHJcbiAgICAgIGNvbnN0IG1hbmlmb2xkOiBiMk1hbmlmb2xkID0gY29udGFjdC5HZXRNYW5pZm9sZCgpO1xyXG5cclxuICAgICAgY29uc3QgcG9pbnRDb3VudDogbnVtYmVyID0gbWFuaWZvbGQucG9pbnRDb3VudDtcclxuICAgICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChwb2ludENvdW50ID4gMCk7XHJcblxyXG4gICAgICBjb25zdCB2YzogYjJDb250YWN0VmVsb2NpdHlDb25zdHJhaW50ID0gdGhpcy5tX3ZlbG9jaXR5Q29uc3RyYWludHNbaV07XHJcbiAgICAgIHZjLmZyaWN0aW9uID0gY29udGFjdC5tX2ZyaWN0aW9uO1xyXG4gICAgICB2Yy5yZXN0aXR1dGlvbiA9IGNvbnRhY3QubV9yZXN0aXR1dGlvbjtcclxuICAgICAgdmMudGFuZ2VudFNwZWVkID0gY29udGFjdC5tX3RhbmdlbnRTcGVlZDtcclxuICAgICAgdmMuaW5kZXhBID0gYm9keUEubV9pc2xhbmRJbmRleDtcclxuICAgICAgdmMuaW5kZXhCID0gYm9keUIubV9pc2xhbmRJbmRleDtcclxuICAgICAgdmMuaW52TWFzc0EgPSBib2R5QS5tX2ludk1hc3M7XHJcbiAgICAgIHZjLmludk1hc3NCID0gYm9keUIubV9pbnZNYXNzO1xyXG4gICAgICB2Yy5pbnZJQSA9IGJvZHlBLm1faW52STtcclxuICAgICAgdmMuaW52SUIgPSBib2R5Qi5tX2ludkk7XHJcbiAgICAgIHZjLmNvbnRhY3RJbmRleCA9IGk7XHJcbiAgICAgIHZjLnBvaW50Q291bnQgPSBwb2ludENvdW50O1xyXG4gICAgICB2Yy5LLlNldFplcm8oKTtcclxuICAgICAgdmMubm9ybWFsTWFzcy5TZXRaZXJvKCk7XHJcblxyXG4gICAgICBjb25zdCBwYzogYjJDb250YWN0UG9zaXRpb25Db25zdHJhaW50ID0gdGhpcy5tX3Bvc2l0aW9uQ29uc3RyYWludHNbaV07XHJcbiAgICAgIHBjLmluZGV4QSA9IGJvZHlBLm1faXNsYW5kSW5kZXg7XHJcbiAgICAgIHBjLmluZGV4QiA9IGJvZHlCLm1faXNsYW5kSW5kZXg7XHJcbiAgICAgIHBjLmludk1hc3NBID0gYm9keUEubV9pbnZNYXNzO1xyXG4gICAgICBwYy5pbnZNYXNzQiA9IGJvZHlCLm1faW52TWFzcztcclxuICAgICAgcGMubG9jYWxDZW50ZXJBLkNvcHkoYm9keUEubV9zd2VlcC5sb2NhbENlbnRlcik7XHJcbiAgICAgIHBjLmxvY2FsQ2VudGVyQi5Db3B5KGJvZHlCLm1fc3dlZXAubG9jYWxDZW50ZXIpO1xyXG4gICAgICBwYy5pbnZJQSA9IGJvZHlBLm1faW52STtcclxuICAgICAgcGMuaW52SUIgPSBib2R5Qi5tX2ludkk7XHJcbiAgICAgIHBjLmxvY2FsTm9ybWFsLkNvcHkobWFuaWZvbGQubG9jYWxOb3JtYWwpO1xyXG4gICAgICBwYy5sb2NhbFBvaW50LkNvcHkobWFuaWZvbGQubG9jYWxQb2ludCk7XHJcbiAgICAgIHBjLnBvaW50Q291bnQgPSBwb2ludENvdW50O1xyXG4gICAgICBwYy5yYWRpdXNBID0gcmFkaXVzQTtcclxuICAgICAgcGMucmFkaXVzQiA9IHJhZGl1c0I7XHJcbiAgICAgIHBjLnR5cGUgPSBtYW5pZm9sZC50eXBlO1xyXG5cclxuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCBwb2ludENvdW50OyArK2opIHtcclxuICAgICAgICBjb25zdCBjcDogYjJNYW5pZm9sZFBvaW50ID0gbWFuaWZvbGQucG9pbnRzW2pdO1xyXG4gICAgICAgIGNvbnN0IHZjcDogYjJWZWxvY2l0eUNvbnN0cmFpbnRQb2ludCA9IHZjLnBvaW50c1tqXTtcclxuXHJcbiAgICAgICAgaWYgKHRoaXMubV9zdGVwLndhcm1TdGFydGluZykge1xyXG4gICAgICAgICAgdmNwLm5vcm1hbEltcHVsc2UgPSB0aGlzLm1fc3RlcC5kdFJhdGlvICogY3Aubm9ybWFsSW1wdWxzZTtcclxuICAgICAgICAgIHZjcC50YW5nZW50SW1wdWxzZSA9IHRoaXMubV9zdGVwLmR0UmF0aW8gKiBjcC50YW5nZW50SW1wdWxzZTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgdmNwLm5vcm1hbEltcHVsc2UgPSAwLjA7XHJcbiAgICAgICAgICB2Y3AudGFuZ2VudEltcHVsc2UgPSAwLjA7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICB2Y3AuckEuU2V0WmVybygpO1xyXG4gICAgICAgIHZjcC5yQi5TZXRaZXJvKCk7XHJcbiAgICAgICAgdmNwLm5vcm1hbE1hc3MgPSAwLjA7XHJcbiAgICAgICAgdmNwLnRhbmdlbnRNYXNzID0gMC4wO1xyXG4gICAgICAgIHZjcC52ZWxvY2l0eUJpYXMgPSAwLjA7XHJcblxyXG4gICAgICAgIHBjLmxvY2FsUG9pbnRzW2pdLkNvcHkoY3AubG9jYWxQb2ludCk7XHJcbiAgICAgIH1cclxuICAgIH1cclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIEluaXRpYWxpemVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfeGZBID0gbmV3IGIyVHJhbnNmb3JtKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgSW5pdGlhbGl6ZVZlbG9jaXR5Q29uc3RyYWludHNfc194ZkIgPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICBwcml2YXRlIHN0YXRpYyBJbml0aWFsaXplVmVsb2NpdHlDb25zdHJhaW50c19zX3dvcmxkTWFuaWZvbGQgPSBuZXcgYjJXb3JsZE1hbmlmb2xkKCk7XHJcblxyXG4gIEluaXRpYWxpemVWZWxvY2l0eUNvbnN0cmFpbnRzKCk6IHZvaWQge1xyXG4gICAgY29uc3QgeGZBID0gYjJDb250YWN0U29sdmVyLkluaXRpYWxpemVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfeGZBO1xyXG4gICAgY29uc3QgeGZCID0gYjJDb250YWN0U29sdmVyLkluaXRpYWxpemVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfeGZCO1xyXG4gICAgY29uc3Qgd29ybGRNYW5pZm9sZCA9IGIyQ29udGFjdFNvbHZlci5Jbml0aWFsaXplVmVsb2NpdHlDb25zdHJhaW50c19zX3dvcmxkTWFuaWZvbGQ7XHJcblxyXG4gICAgY29uc3Qga19tYXhDb25kaXRpb25OdW1iZXIgPSAxMDAwLjA7XHJcblxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7ICsraSkge1xyXG4gICAgICBjb25zdCB2YzogYjJDb250YWN0VmVsb2NpdHlDb25zdHJhaW50ID0gdGhpcy5tX3ZlbG9jaXR5Q29uc3RyYWludHNbaV07XHJcbiAgICAgIGNvbnN0IHBjOiBiMkNvbnRhY3RQb3NpdGlvbkNvbnN0cmFpbnQgPSB0aGlzLm1fcG9zaXRpb25Db25zdHJhaW50c1tpXTtcclxuXHJcbiAgICAgIGNvbnN0IHJhZGl1c0E6IG51bWJlciA9IHBjLnJhZGl1c0E7XHJcbiAgICAgIGNvbnN0IHJhZGl1c0I6IG51bWJlciA9IHBjLnJhZGl1c0I7XHJcbiAgICAgIGNvbnN0IG1hbmlmb2xkOiBiMk1hbmlmb2xkID0gdGhpcy5tX2NvbnRhY3RzW3ZjLmNvbnRhY3RJbmRleF0uR2V0TWFuaWZvbGQoKTtcclxuXHJcbiAgICAgIGNvbnN0IGluZGV4QTogbnVtYmVyID0gdmMuaW5kZXhBO1xyXG4gICAgICBjb25zdCBpbmRleEI6IG51bWJlciA9IHZjLmluZGV4QjtcclxuXHJcbiAgICAgIGNvbnN0IG1BOiBudW1iZXIgPSB2Yy5pbnZNYXNzQTtcclxuICAgICAgY29uc3QgbUI6IG51bWJlciA9IHZjLmludk1hc3NCO1xyXG4gICAgICBjb25zdCBpQTogbnVtYmVyID0gdmMuaW52SUE7XHJcbiAgICAgIGNvbnN0IGlCOiBudW1iZXIgPSB2Yy5pbnZJQjtcclxuICAgICAgY29uc3QgbG9jYWxDZW50ZXJBOiBiMlZlYzIgPSBwYy5sb2NhbENlbnRlckE7XHJcbiAgICAgIGNvbnN0IGxvY2FsQ2VudGVyQjogYjJWZWMyID0gcGMubG9jYWxDZW50ZXJCO1xyXG5cclxuICAgICAgY29uc3QgY0E6IGIyVmVjMiA9IHRoaXMubV9wb3NpdGlvbnNbaW5kZXhBXS5jO1xyXG4gICAgICBjb25zdCBhQTogbnVtYmVyID0gdGhpcy5tX3Bvc2l0aW9uc1tpbmRleEFdLmE7XHJcbiAgICAgIGNvbnN0IHZBOiBiMlZlYzIgPSB0aGlzLm1fdmVsb2NpdGllc1tpbmRleEFdLnY7XHJcbiAgICAgIGNvbnN0IHdBOiBudW1iZXIgPSB0aGlzLm1fdmVsb2NpdGllc1tpbmRleEFdLnc7XHJcblxyXG4gICAgICBjb25zdCBjQjogYjJWZWMyID0gdGhpcy5tX3Bvc2l0aW9uc1tpbmRleEJdLmM7XHJcbiAgICAgIGNvbnN0IGFCOiBudW1iZXIgPSB0aGlzLm1fcG9zaXRpb25zW2luZGV4Ql0uYTtcclxuICAgICAgY29uc3QgdkI6IGIyVmVjMiA9IHRoaXMubV92ZWxvY2l0aWVzW2luZGV4Ql0udjtcclxuICAgICAgY29uc3Qgd0I6IG51bWJlciA9IHRoaXMubV92ZWxvY2l0aWVzW2luZGV4Ql0udztcclxuXHJcbiAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQobWFuaWZvbGQucG9pbnRDb3VudCA+IDApO1xyXG5cclxuICAgICAgeGZBLnEuU2V0QW5nbGUoYUEpO1xyXG4gICAgICB4ZkIucS5TZXRBbmdsZShhQik7XHJcbiAgICAgIGIyVmVjMi5TdWJWVihjQSwgYjJSb3QuTXVsUlYoeGZBLnEsIGxvY2FsQ2VudGVyQSwgYjJWZWMyLnNfdDApLCB4ZkEucCk7XHJcbiAgICAgIGIyVmVjMi5TdWJWVihjQiwgYjJSb3QuTXVsUlYoeGZCLnEsIGxvY2FsQ2VudGVyQiwgYjJWZWMyLnNfdDApLCB4ZkIucCk7XHJcblxyXG4gICAgICB3b3JsZE1hbmlmb2xkLkluaXRpYWxpemUobWFuaWZvbGQsIHhmQSwgcmFkaXVzQSwgeGZCLCByYWRpdXNCKTtcclxuXHJcbiAgICAgIHZjLm5vcm1hbC5Db3B5KHdvcmxkTWFuaWZvbGQubm9ybWFsKTtcclxuICAgICAgYjJWZWMyLkNyb3NzVk9uZSh2Yy5ub3JtYWwsIHZjLnRhbmdlbnQpOyAvLyBjb21wdXRlIGZyb20gbm9ybWFsXHJcblxyXG4gICAgICBjb25zdCBwb2ludENvdW50OiBudW1iZXIgPSB2Yy5wb2ludENvdW50O1xyXG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHBvaW50Q291bnQ7ICsraikge1xyXG4gICAgICAgIGNvbnN0IHZjcDogYjJWZWxvY2l0eUNvbnN0cmFpbnRQb2ludCA9IHZjLnBvaW50c1tqXTtcclxuXHJcbiAgICAgICAgLy8gdmNwLT5yQSA9IHdvcmxkTWFuaWZvbGQucG9pbnRzW2pdIC0gY0E7XHJcbiAgICAgICAgYjJWZWMyLlN1YlZWKHdvcmxkTWFuaWZvbGQucG9pbnRzW2pdLCBjQSwgdmNwLnJBKTtcclxuICAgICAgICAvLyB2Y3AtPnJCID0gd29ybGRNYW5pZm9sZC5wb2ludHNbal0gLSBjQjtcclxuICAgICAgICBiMlZlYzIuU3ViVlYod29ybGRNYW5pZm9sZC5wb2ludHNbal0sIGNCLCB2Y3AuckIpO1xyXG5cclxuICAgICAgICBjb25zdCBybkE6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKHZjcC5yQSwgdmMubm9ybWFsKTtcclxuICAgICAgICBjb25zdCBybkI6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKHZjcC5yQiwgdmMubm9ybWFsKTtcclxuXHJcbiAgICAgICAgY29uc3Qga05vcm1hbDogbnVtYmVyID0gbUEgKyBtQiArIGlBICogcm5BICogcm5BICsgaUIgKiBybkIgKiBybkI7XHJcblxyXG4gICAgICAgIHZjcC5ub3JtYWxNYXNzID0ga05vcm1hbCA+IDAgPyAxIC8ga05vcm1hbCA6IDA7XHJcblxyXG4gICAgICAgIC8vIGIyVmVjMiB0YW5nZW50ID0gYjJDcm9zcyh2Yy0+bm9ybWFsLCAxLjBmKTtcclxuICAgICAgICBjb25zdCB0YW5nZW50OiBiMlZlYzIgPSB2Yy50YW5nZW50OyAvLyBwcmVjb21wdXRlZCBmcm9tIG5vcm1hbFxyXG5cclxuICAgICAgICBjb25zdCBydEE6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKHZjcC5yQSwgdGFuZ2VudCk7XHJcbiAgICAgICAgY29uc3QgcnRCOiBudW1iZXIgPSBiMlZlYzIuQ3Jvc3NWVih2Y3AuckIsIHRhbmdlbnQpO1xyXG5cclxuICAgICAgICBjb25zdCBrVGFuZ2VudDogbnVtYmVyID0gbUEgKyBtQiArIGlBICogcnRBICogcnRBICsgaUIgKiBydEIgKiBydEI7XHJcblxyXG4gICAgICAgIHZjcC50YW5nZW50TWFzcyA9IGtUYW5nZW50ID4gMCA/IDEgLyBrVGFuZ2VudCA6IDA7XHJcblxyXG4gICAgICAgIC8vIFNldHVwIGEgdmVsb2NpdHkgYmlhcyBmb3IgcmVzdGl0dXRpb24uXHJcbiAgICAgICAgdmNwLnZlbG9jaXR5QmlhcyA9IDA7XHJcbiAgICAgICAgLy8gZmxvYXQzMiB2UmVsID0gYjJEb3QodmMtPm5vcm1hbCwgdkIgKyBiMkNyb3NzKHdCLCB2Y3AtPnJCKSAtIHZBIC0gYjJDcm9zcyh3QSwgdmNwLT5yQSkpO1xyXG4gICAgICAgIGNvbnN0IHZSZWw6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihcclxuICAgICAgICAgIHZjLm5vcm1hbCxcclxuICAgICAgICAgIGIyVmVjMi5TdWJWVihcclxuICAgICAgICAgICAgYjJWZWMyLkFkZFZDcm9zc1NWKHZCLCB3QiwgdmNwLnJCLCBiMlZlYzIuc190MCksXHJcbiAgICAgICAgICAgIGIyVmVjMi5BZGRWQ3Jvc3NTVih2QSwgd0EsIHZjcC5yQSwgYjJWZWMyLnNfdDEpLFxyXG4gICAgICAgICAgICBiMlZlYzIuc190MCxcclxuICAgICAgICAgICksXHJcbiAgICAgICAgKTtcclxuICAgICAgICBpZiAodlJlbCA8IC1iMl92ZWxvY2l0eVRocmVzaG9sZCkge1xyXG4gICAgICAgICAgdmNwLnZlbG9jaXR5QmlhcyArPSAtdmMucmVzdGl0dXRpb24gKiB2UmVsO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gSWYgd2UgaGF2ZSB0d28gcG9pbnRzLCB0aGVuIHByZXBhcmUgdGhlIGJsb2NrIHNvbHZlci5cclxuICAgICAgaWYgKHZjLnBvaW50Q291bnQgPT09IDIgJiYgZ19ibG9ja1NvbHZlKSB7XHJcbiAgICAgICAgY29uc3QgdmNwMTogYjJWZWxvY2l0eUNvbnN0cmFpbnRQb2ludCA9IHZjLnBvaW50c1swXTtcclxuICAgICAgICBjb25zdCB2Y3AyOiBiMlZlbG9jaXR5Q29uc3RyYWludFBvaW50ID0gdmMucG9pbnRzWzFdO1xyXG5cclxuICAgICAgICBjb25zdCBybjFBOiBudW1iZXIgPSBiMlZlYzIuQ3Jvc3NWVih2Y3AxLnJBLCB2Yy5ub3JtYWwpO1xyXG4gICAgICAgIGNvbnN0IHJuMUI6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKHZjcDEuckIsIHZjLm5vcm1hbCk7XHJcbiAgICAgICAgY29uc3Qgcm4yQTogbnVtYmVyID0gYjJWZWMyLkNyb3NzVlYodmNwMi5yQSwgdmMubm9ybWFsKTtcclxuICAgICAgICBjb25zdCBybjJCOiBudW1iZXIgPSBiMlZlYzIuQ3Jvc3NWVih2Y3AyLnJCLCB2Yy5ub3JtYWwpO1xyXG5cclxuICAgICAgICBjb25zdCBrMTE6IG51bWJlciA9IG1BICsgbUIgKyBpQSAqIHJuMUEgKiBybjFBICsgaUIgKiBybjFCICogcm4xQjtcclxuICAgICAgICBjb25zdCBrMjI6IG51bWJlciA9IG1BICsgbUIgKyBpQSAqIHJuMkEgKiBybjJBICsgaUIgKiBybjJCICogcm4yQjtcclxuICAgICAgICBjb25zdCBrMTI6IG51bWJlciA9IG1BICsgbUIgKyBpQSAqIHJuMUEgKiBybjJBICsgaUIgKiBybjFCICogcm4yQjtcclxuXHJcbiAgICAgICAgLy8gRW5zdXJlIGEgcmVhc29uYWJsZSBjb25kaXRpb24gbnVtYmVyLlxyXG4gICAgICAgIC8vIGZsb2F0MzIga19tYXhDb25kaXRpb25OdW1iZXIgPSAxMDAwLjBmO1xyXG4gICAgICAgIGlmIChrMTEgKiBrMTEgPCBrX21heENvbmRpdGlvbk51bWJlciAqIChrMTEgKiBrMjIgLSBrMTIgKiBrMTIpKSB7XHJcbiAgICAgICAgICAvLyBLIGlzIHNhZmUgdG8gaW52ZXJ0LlxyXG4gICAgICAgICAgdmMuSy5leC5TZXQoazExLCBrMTIpO1xyXG4gICAgICAgICAgdmMuSy5leS5TZXQoazEyLCBrMjIpO1xyXG4gICAgICAgICAgdmMuSy5HZXRJbnZlcnNlKHZjLm5vcm1hbE1hc3MpO1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICAvLyBUaGUgY29uc3RyYWludHMgYXJlIHJlZHVuZGFudCwganVzdCB1c2Ugb25lLlxyXG4gICAgICAgICAgLy8gVE9ET19FUklOIHVzZSBkZWVwZXN0P1xyXG4gICAgICAgICAgdmMucG9pbnRDb3VudCA9IDE7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBXYXJtU3RhcnRfc19QID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBXYXJtU3RhcnQoKTogdm9pZCB7XHJcbiAgICBjb25zdCBQOiBiMlZlYzIgPSBiMkNvbnRhY3RTb2x2ZXIuV2FybVN0YXJ0X3NfUDtcclxuXHJcbiAgICAvLyBXYXJtIHN0YXJ0LlxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7ICsraSkge1xyXG4gICAgICBjb25zdCB2YzogYjJDb250YWN0VmVsb2NpdHlDb25zdHJhaW50ID0gdGhpcy5tX3ZlbG9jaXR5Q29uc3RyYWludHNbaV07XHJcblxyXG4gICAgICBjb25zdCBpbmRleEE6IG51bWJlciA9IHZjLmluZGV4QTtcclxuICAgICAgY29uc3QgaW5kZXhCOiBudW1iZXIgPSB2Yy5pbmRleEI7XHJcbiAgICAgIGNvbnN0IG1BOiBudW1iZXIgPSB2Yy5pbnZNYXNzQTtcclxuICAgICAgY29uc3QgaUE6IG51bWJlciA9IHZjLmludklBO1xyXG4gICAgICBjb25zdCBtQjogbnVtYmVyID0gdmMuaW52TWFzc0I7XHJcbiAgICAgIGNvbnN0IGlCOiBudW1iZXIgPSB2Yy5pbnZJQjtcclxuICAgICAgY29uc3QgcG9pbnRDb3VudDogbnVtYmVyID0gdmMucG9pbnRDb3VudDtcclxuXHJcbiAgICAgIGNvbnN0IHZBOiBiMlZlYzIgPSB0aGlzLm1fdmVsb2NpdGllc1tpbmRleEFdLnY7XHJcbiAgICAgIGxldCB3QTogbnVtYmVyID0gdGhpcy5tX3ZlbG9jaXRpZXNbaW5kZXhBXS53O1xyXG4gICAgICBjb25zdCB2QjogYjJWZWMyID0gdGhpcy5tX3ZlbG9jaXRpZXNbaW5kZXhCXS52O1xyXG4gICAgICBsZXQgd0I6IG51bWJlciA9IHRoaXMubV92ZWxvY2l0aWVzW2luZGV4Ql0udztcclxuXHJcbiAgICAgIGNvbnN0IG5vcm1hbDogYjJWZWMyID0gdmMubm9ybWFsO1xyXG4gICAgICAvLyBiMlZlYzIgdGFuZ2VudCA9IGIyQ3Jvc3Mobm9ybWFsLCAxLjBmKTtcclxuICAgICAgY29uc3QgdGFuZ2VudDogYjJWZWMyID0gdmMudGFuZ2VudDsgLy8gcHJlY29tcHV0ZWQgZnJvbSBub3JtYWxcclxuXHJcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgcG9pbnRDb3VudDsgKytqKSB7XHJcbiAgICAgICAgY29uc3QgdmNwOiBiMlZlbG9jaXR5Q29uc3RyYWludFBvaW50ID0gdmMucG9pbnRzW2pdO1xyXG4gICAgICAgIC8vIGIyVmVjMiBQID0gdmNwLT5ub3JtYWxJbXB1bHNlICogbm9ybWFsICsgdmNwLT50YW5nZW50SW1wdWxzZSAqIHRhbmdlbnQ7XHJcbiAgICAgICAgYjJWZWMyLkFkZFZWKFxyXG4gICAgICAgICAgYjJWZWMyLk11bFNWKHZjcC5ub3JtYWxJbXB1bHNlLCBub3JtYWwsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICAgIGIyVmVjMi5NdWxTVih2Y3AudGFuZ2VudEltcHVsc2UsIHRhbmdlbnQsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICAgIFAsXHJcbiAgICAgICAgKTtcclxuICAgICAgICAvLyB3QSAtPSBpQSAqIGIyQ3Jvc3ModmNwLT5yQSwgUCk7XHJcbiAgICAgICAgd0EgLT0gaUEgKiBiMlZlYzIuQ3Jvc3NWVih2Y3AuckEsIFApO1xyXG4gICAgICAgIC8vIHZBIC09IG1BICogUDtcclxuICAgICAgICB2QS5TZWxmTXVsU3ViKG1BLCBQKTtcclxuICAgICAgICAvLyB3QiArPSBpQiAqIGIyQ3Jvc3ModmNwLT5yQiwgUCk7XHJcbiAgICAgICAgd0IgKz0gaUIgKiBiMlZlYzIuQ3Jvc3NWVih2Y3AuckIsIFApO1xyXG4gICAgICAgIC8vIHZCICs9IG1CICogUDtcclxuICAgICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQKTtcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gdGhpcy5tX3ZlbG9jaXRpZXNbaW5kZXhBXS52ID0gdkE7XHJcbiAgICAgIHRoaXMubV92ZWxvY2l0aWVzW2luZGV4QV0udyA9IHdBO1xyXG4gICAgICAvLyB0aGlzLm1fdmVsb2NpdGllc1tpbmRleEJdLnYgPSB2QjtcclxuICAgICAgdGhpcy5tX3ZlbG9jaXRpZXNbaW5kZXhCXS53ID0gd0I7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19kdiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19kdjEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfZHYyID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX1AgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfYSA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19iID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX3ggPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfZCA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19QMSA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19QMiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19QMVAyID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHMoKTogdm9pZCB7XHJcbiAgICBjb25zdCBkdjogYjJWZWMyID0gYjJDb250YWN0U29sdmVyLlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX2R2O1xyXG4gICAgY29uc3QgZHYxOiBiMlZlYzIgPSBiMkNvbnRhY3RTb2x2ZXIuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfZHYxO1xyXG4gICAgY29uc3QgZHYyOiBiMlZlYzIgPSBiMkNvbnRhY3RTb2x2ZXIuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfZHYyO1xyXG4gICAgY29uc3QgUDogYjJWZWMyID0gYjJDb250YWN0U29sdmVyLlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX1A7XHJcbiAgICBjb25zdCBhOiBiMlZlYzIgPSBiMkNvbnRhY3RTb2x2ZXIuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfYTtcclxuICAgIGNvbnN0IGI6IGIyVmVjMiA9IGIyQ29udGFjdFNvbHZlci5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19iO1xyXG4gICAgY29uc3QgeDogYjJWZWMyID0gYjJDb250YWN0U29sdmVyLlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX3g7XHJcbiAgICBjb25zdCBkOiBiMlZlYzIgPSBiMkNvbnRhY3RTb2x2ZXIuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfZDtcclxuICAgIGNvbnN0IFAxOiBiMlZlYzIgPSBiMkNvbnRhY3RTb2x2ZXIuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUDE7XHJcbiAgICBjb25zdCBQMjogYjJWZWMyID0gYjJDb250YWN0U29sdmVyLlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX1AyO1xyXG4gICAgY29uc3QgUDFQMjogYjJWZWMyID0gYjJDb250YWN0U29sdmVyLlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX1AxUDI7XHJcblxyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fY291bnQ7ICsraSkge1xyXG4gICAgICBjb25zdCB2YzogYjJDb250YWN0VmVsb2NpdHlDb25zdHJhaW50ID0gdGhpcy5tX3ZlbG9jaXR5Q29uc3RyYWludHNbaV07XHJcblxyXG4gICAgICBjb25zdCBpbmRleEE6IG51bWJlciA9IHZjLmluZGV4QTtcclxuICAgICAgY29uc3QgaW5kZXhCOiBudW1iZXIgPSB2Yy5pbmRleEI7XHJcbiAgICAgIGNvbnN0IG1BOiBudW1iZXIgPSB2Yy5pbnZNYXNzQTtcclxuICAgICAgY29uc3QgaUE6IG51bWJlciA9IHZjLmludklBO1xyXG4gICAgICBjb25zdCBtQjogbnVtYmVyID0gdmMuaW52TWFzc0I7XHJcbiAgICAgIGNvbnN0IGlCOiBudW1iZXIgPSB2Yy5pbnZJQjtcclxuICAgICAgY29uc3QgcG9pbnRDb3VudDogbnVtYmVyID0gdmMucG9pbnRDb3VudDtcclxuXHJcbiAgICAgIGNvbnN0IHZBOiBiMlZlYzIgPSB0aGlzLm1fdmVsb2NpdGllc1tpbmRleEFdLnY7XHJcbiAgICAgIGxldCB3QTogbnVtYmVyID0gdGhpcy5tX3ZlbG9jaXRpZXNbaW5kZXhBXS53O1xyXG4gICAgICBjb25zdCB2QjogYjJWZWMyID0gdGhpcy5tX3ZlbG9jaXRpZXNbaW5kZXhCXS52O1xyXG4gICAgICBsZXQgd0I6IG51bWJlciA9IHRoaXMubV92ZWxvY2l0aWVzW2luZGV4Ql0udztcclxuXHJcbiAgICAgIC8vIGIyVmVjMiBub3JtYWwgPSB2Yy0+bm9ybWFsO1xyXG4gICAgICBjb25zdCBub3JtYWw6IGIyVmVjMiA9IHZjLm5vcm1hbDtcclxuICAgICAgLy8gYjJWZWMyIHRhbmdlbnQgPSBiMkNyb3NzKG5vcm1hbCwgMS4wZik7XHJcbiAgICAgIGNvbnN0IHRhbmdlbnQ6IGIyVmVjMiA9IHZjLnRhbmdlbnQ7IC8vIHByZWNvbXB1dGVkIGZyb20gbm9ybWFsXHJcbiAgICAgIGNvbnN0IGZyaWN0aW9uOiBudW1iZXIgPSB2Yy5mcmljdGlvbjtcclxuXHJcbiAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQocG9pbnRDb3VudCA9PT0gMSB8fCBwb2ludENvdW50ID09PSAyKTtcclxuXHJcbiAgICAgIC8vIFNvbHZlIHRhbmdlbnQgY29uc3RyYWludHMgZmlyc3QgYmVjYXVzZSBub24tcGVuZXRyYXRpb24gaXMgbW9yZSBpbXBvcnRhbnRcclxuICAgICAgLy8gdGhhbiBmcmljdGlvbi5cclxuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCBwb2ludENvdW50OyArK2opIHtcclxuICAgICAgICBjb25zdCB2Y3A6IGIyVmVsb2NpdHlDb25zdHJhaW50UG9pbnQgPSB2Yy5wb2ludHNbal07XHJcblxyXG4gICAgICAgIC8vIFJlbGF0aXZlIHZlbG9jaXR5IGF0IGNvbnRhY3RcclxuICAgICAgICAvLyBiMlZlYzIgZHYgPSB2QiArIGIyQ3Jvc3Mod0IsIHZjcC0+ckIpIC0gdkEgLSBiMkNyb3NzKHdBLCB2Y3AtPnJBKTtcclxuICAgICAgICBiMlZlYzIuU3ViVlYoXHJcbiAgICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkIsIHdCLCB2Y3AuckIsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICAgIGIyVmVjMi5BZGRWQ3Jvc3NTVih2QSwgd0EsIHZjcC5yQSwgYjJWZWMyLnNfdDEpLFxyXG4gICAgICAgICAgZHYsXHJcbiAgICAgICAgKTtcclxuXHJcbiAgICAgICAgLy8gQ29tcHV0ZSB0YW5nZW50IGZvcmNlXHJcbiAgICAgICAgLy8gZmxvYXQzMiB2dCA9IGIyRG90KGR2LCB0YW5nZW50KSAtIHZjLT50YW5nZW50U3BlZWQ7XHJcbiAgICAgICAgY29uc3QgdnQ6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihkdiwgdGFuZ2VudCkgLSB2Yy50YW5nZW50U3BlZWQ7XHJcbiAgICAgICAgbGV0IGxhbWJkYTogbnVtYmVyID0gdmNwLnRhbmdlbnRNYXNzICogLXZ0O1xyXG5cclxuICAgICAgICAvLyBiMkNsYW1wIHRoZSBhY2N1bXVsYXRlZCBmb3JjZVxyXG4gICAgICAgIGNvbnN0IG1heEZyaWN0aW9uOiBudW1iZXIgPSBmcmljdGlvbiAqIHZjcC5ub3JtYWxJbXB1bHNlO1xyXG4gICAgICAgIGNvbnN0IG5ld0ltcHVsc2U6IG51bWJlciA9IGIyQ2xhbXAodmNwLnRhbmdlbnRJbXB1bHNlICsgbGFtYmRhLCAtbWF4RnJpY3Rpb24sIG1heEZyaWN0aW9uKTtcclxuICAgICAgICBsYW1iZGEgPSBuZXdJbXB1bHNlIC0gdmNwLnRhbmdlbnRJbXB1bHNlO1xyXG4gICAgICAgIHZjcC50YW5nZW50SW1wdWxzZSA9IG5ld0ltcHVsc2U7XHJcblxyXG4gICAgICAgIC8vIEFwcGx5IGNvbnRhY3QgaW1wdWxzZVxyXG4gICAgICAgIC8vIGIyVmVjMiBQID0gbGFtYmRhICogdGFuZ2VudDtcclxuICAgICAgICBiMlZlYzIuTXVsU1YobGFtYmRhLCB0YW5nZW50LCBQKTtcclxuXHJcbiAgICAgICAgLy8gdkEgLT0gbUEgKiBQO1xyXG4gICAgICAgIHZBLlNlbGZNdWxTdWIobUEsIFApO1xyXG4gICAgICAgIC8vIHdBIC09IGlBICogYjJDcm9zcyh2Y3AtPnJBLCBQKTtcclxuICAgICAgICB3QSAtPSBpQSAqIGIyVmVjMi5Dcm9zc1ZWKHZjcC5yQSwgUCk7XHJcblxyXG4gICAgICAgIC8vIHZCICs9IG1CICogUDtcclxuICAgICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQKTtcclxuICAgICAgICAvLyB3QiArPSBpQiAqIGIyQ3Jvc3ModmNwLT5yQiwgUCk7XHJcbiAgICAgICAgd0IgKz0gaUIgKiBiMlZlYzIuQ3Jvc3NWVih2Y3AuckIsIFApO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBTb2x2ZSBub3JtYWwgY29uc3RyYWludHNcclxuICAgICAgaWYgKHZjLnBvaW50Q291bnQgPT09IDEgfHwgZ19ibG9ja1NvbHZlID09PSBmYWxzZSkge1xyXG4gICAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgcG9pbnRDb3VudDsgKytqKSB7XHJcbiAgICAgICAgICBjb25zdCB2Y3A6IGIyVmVsb2NpdHlDb25zdHJhaW50UG9pbnQgPSB2Yy5wb2ludHNbal07XHJcblxyXG4gICAgICAgICAgLy8gUmVsYXRpdmUgdmVsb2NpdHkgYXQgY29udGFjdFxyXG4gICAgICAgICAgLy8gYjJWZWMyIGR2ID0gdkIgKyBiMkNyb3NzKHdCLCB2Y3AtPnJCKSAtIHZBIC0gYjJDcm9zcyh3QSwgdmNwLT5yQSk7XHJcbiAgICAgICAgICBiMlZlYzIuU3ViVlYoXHJcbiAgICAgICAgICAgIGIyVmVjMi5BZGRWQ3Jvc3NTVih2Qiwgd0IsIHZjcC5yQiwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkEsIHdBLCB2Y3AuckEsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICAgICAgZHYsXHJcbiAgICAgICAgICApO1xyXG5cclxuICAgICAgICAgIC8vIENvbXB1dGUgbm9ybWFsIGltcHVsc2VcclxuICAgICAgICAgIC8vIGZsb2F0MzIgdm4gPSBiMkRvdChkdiwgbm9ybWFsKTtcclxuICAgICAgICAgIGNvbnN0IHZuOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoZHYsIG5vcm1hbCk7XHJcbiAgICAgICAgICBsZXQgbGFtYmRhOiBudW1iZXIgPSAtdmNwLm5vcm1hbE1hc3MgKiAodm4gLSB2Y3AudmVsb2NpdHlCaWFzKTtcclxuXHJcbiAgICAgICAgICAvLyBiMkNsYW1wIHRoZSBhY2N1bXVsYXRlZCBpbXB1bHNlXHJcbiAgICAgICAgICAvLyBmbG9hdDMyIG5ld0ltcHVsc2UgPSBiMk1heCh2Y3AtPm5vcm1hbEltcHVsc2UgKyBsYW1iZGEsIDAuMGYpO1xyXG4gICAgICAgICAgY29uc3QgbmV3SW1wdWxzZTogbnVtYmVyID0gYjJNYXgodmNwLm5vcm1hbEltcHVsc2UgKyBsYW1iZGEsIDApO1xyXG4gICAgICAgICAgbGFtYmRhID0gbmV3SW1wdWxzZSAtIHZjcC5ub3JtYWxJbXB1bHNlO1xyXG4gICAgICAgICAgdmNwLm5vcm1hbEltcHVsc2UgPSBuZXdJbXB1bHNlO1xyXG5cclxuICAgICAgICAgIC8vIEFwcGx5IGNvbnRhY3QgaW1wdWxzZVxyXG4gICAgICAgICAgLy8gYjJWZWMyIFAgPSBsYW1iZGEgKiBub3JtYWw7XHJcbiAgICAgICAgICBiMlZlYzIuTXVsU1YobGFtYmRhLCBub3JtYWwsIFApO1xyXG4gICAgICAgICAgLy8gdkEgLT0gbUEgKiBQO1xyXG4gICAgICAgICAgdkEuU2VsZk11bFN1YihtQSwgUCk7XHJcbiAgICAgICAgICAvLyB3QSAtPSBpQSAqIGIyQ3Jvc3ModmNwLT5yQSwgUCk7XHJcbiAgICAgICAgICB3QSAtPSBpQSAqIGIyVmVjMi5Dcm9zc1ZWKHZjcC5yQSwgUCk7XHJcblxyXG4gICAgICAgICAgLy8gdkIgKz0gbUIgKiBQO1xyXG4gICAgICAgICAgdkIuU2VsZk11bEFkZChtQiwgUCk7XHJcbiAgICAgICAgICAvLyB3QiArPSBpQiAqIGIyQ3Jvc3ModmNwLT5yQiwgUCk7XHJcbiAgICAgICAgICB3QiArPSBpQiAqIGIyVmVjMi5Dcm9zc1ZWKHZjcC5yQiwgUCk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9IGVsc2Uge1xyXG4gICAgICAgIC8vIEJsb2NrIHNvbHZlciBkZXZlbG9wZWQgaW4gY29sbGFib3JhdGlvbiB3aXRoIERpcmsgR3JlZ29yaXVzIChiYWNrIGluIDAxLzA3IG9uIEJveDJEX0xpdGUpLlxyXG4gICAgICAgIC8vIEJ1aWxkIHRoZSBtaW5pIExDUCBmb3IgdGhpcyBjb250YWN0IHBhdGNoXHJcbiAgICAgICAgLy9cclxuICAgICAgICAvLyB2biA9IEEgKiB4ICsgYiwgdm4gPj0gMCwgeCA+PSAwIGFuZCB2bl9pICogeF9pID0gMCB3aXRoIGkgPSAxLi4yXHJcbiAgICAgICAgLy9cclxuICAgICAgICAvLyBBID0gSiAqIFcgKiBKVCBhbmQgSiA9ICggLW4sIC1yMSB4IG4sIG4sIHIyIHggbiApXHJcbiAgICAgICAgLy8gYiA9IHZuMCAtIHZlbG9jaXR5Qmlhc1xyXG4gICAgICAgIC8vXHJcbiAgICAgICAgLy8gVGhlIHN5c3RlbSBpcyBzb2x2ZWQgdXNpbmcgdGhlIFwiVG90YWwgZW51bWVyYXRpb24gbWV0aG9kXCIgKHMuIE11cnR5KS4gVGhlIGNvbXBsZW1lbnRhcnkgY29uc3RyYWludCB2bl9pICogeF9pXHJcbiAgICAgICAgLy8gaW1wbGllcyB0aGF0IHdlIG11c3QgaGF2ZSBpbiBhbnkgc29sdXRpb24gZWl0aGVyIHZuX2kgPSAwIG9yIHhfaSA9IDAuIFNvIGZvciB0aGUgMkQgY29udGFjdCBwcm9ibGVtIHRoZSBjYXNlc1xyXG4gICAgICAgIC8vIHZuMSA9IDAgYW5kIHZuMiA9IDAsIHgxID0gMCBhbmQgeDIgPSAwLCB4MSA9IDAgYW5kIHZuMiA9IDAsIHgyID0gMCBhbmQgdm4xID0gMCBuZWVkIHRvIGJlIHRlc3RlZC4gVGhlIGZpcnN0IHZhbGlkXHJcbiAgICAgICAgLy8gc29sdXRpb24gdGhhdCBzYXRpc2ZpZXMgdGhlIHByb2JsZW0gaXMgY2hvc2VuLlxyXG4gICAgICAgIC8vXHJcbiAgICAgICAgLy8gSW4gb3JkZXIgdG8gYWNjb3VudCBvZiB0aGUgYWNjdW11bGF0ZWQgaW1wdWxzZSAnYScgKGJlY2F1c2Ugb2YgdGhlIGl0ZXJhdGl2ZSBuYXR1cmUgb2YgdGhlIHNvbHZlciB3aGljaCBvbmx5IHJlcXVpcmVzXHJcbiAgICAgICAgLy8gdGhhdCB0aGUgYWNjdW11bGF0ZWQgaW1wdWxzZSBpcyBjbGFtcGVkIGFuZCBub3QgdGhlIGluY3JlbWVudGFsIGltcHVsc2UpIHdlIGNoYW5nZSB0aGUgaW1wdWxzZSB2YXJpYWJsZSAoeF9pKS5cclxuICAgICAgICAvL1xyXG4gICAgICAgIC8vIFN1YnN0aXR1dGU6XHJcbiAgICAgICAgLy9cclxuICAgICAgICAvLyB4ID0gYSArIGRcclxuICAgICAgICAvL1xyXG4gICAgICAgIC8vIGEgOj0gb2xkIHRvdGFsIGltcHVsc2VcclxuICAgICAgICAvLyB4IDo9IG5ldyB0b3RhbCBpbXB1bHNlXHJcbiAgICAgICAgLy8gZCA6PSBpbmNyZW1lbnRhbCBpbXB1bHNlXHJcbiAgICAgICAgLy9cclxuICAgICAgICAvLyBGb3IgdGhlIGN1cnJlbnQgaXRlcmF0aW9uIHdlIGV4dGVuZCB0aGUgZm9ybXVsYSBmb3IgdGhlIGluY3JlbWVudGFsIGltcHVsc2VcclxuICAgICAgICAvLyB0byBjb21wdXRlIHRoZSBuZXcgdG90YWwgaW1wdWxzZTpcclxuICAgICAgICAvL1xyXG4gICAgICAgIC8vIHZuID0gQSAqIGQgKyBiXHJcbiAgICAgICAgLy8gICAgPSBBICogKHggLSBhKSArIGJcclxuICAgICAgICAvLyAgICA9IEEgKiB4ICsgYiAtIEEgKiBhXHJcbiAgICAgICAgLy8gICAgPSBBICogeCArIGInXHJcbiAgICAgICAgLy8gYicgPSBiIC0gQSAqIGE7XHJcblxyXG4gICAgICAgIGNvbnN0IGNwMTogYjJWZWxvY2l0eUNvbnN0cmFpbnRQb2ludCA9IHZjLnBvaW50c1swXTtcclxuICAgICAgICBjb25zdCBjcDI6IGIyVmVsb2NpdHlDb25zdHJhaW50UG9pbnQgPSB2Yy5wb2ludHNbMV07XHJcblxyXG4gICAgICAgIC8vIGIyVmVjMiBhKGNwMS0+bm9ybWFsSW1wdWxzZSwgY3AyLT5ub3JtYWxJbXB1bHNlKTtcclxuICAgICAgICBhLlNldChjcDEubm9ybWFsSW1wdWxzZSwgY3AyLm5vcm1hbEltcHVsc2UpO1xyXG4gICAgICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQoYS54ID49IDAgJiYgYS55ID49IDApO1xyXG5cclxuICAgICAgICAvLyBSZWxhdGl2ZSB2ZWxvY2l0eSBhdCBjb250YWN0XHJcbiAgICAgICAgLy8gYjJWZWMyIGR2MSA9IHZCICsgYjJDcm9zcyh3QiwgY3AxLT5yQikgLSB2QSAtIGIyQ3Jvc3Mod0EsIGNwMS0+ckEpO1xyXG4gICAgICAgIGIyVmVjMi5TdWJWVihcclxuICAgICAgICAgIGIyVmVjMi5BZGRWQ3Jvc3NTVih2Qiwgd0IsIGNwMS5yQiwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgICAgYjJWZWMyLkFkZFZDcm9zc1NWKHZBLCB3QSwgY3AxLnJBLCBiMlZlYzIuc190MSksXHJcbiAgICAgICAgICBkdjEsXHJcbiAgICAgICAgKTtcclxuICAgICAgICAvLyBiMlZlYzIgZHYyID0gdkIgKyBiMkNyb3NzKHdCLCBjcDItPnJCKSAtIHZBIC0gYjJDcm9zcyh3QSwgY3AyLT5yQSk7XHJcbiAgICAgICAgYjJWZWMyLlN1YlZWKFxyXG4gICAgICAgICAgYjJWZWMyLkFkZFZDcm9zc1NWKHZCLCB3QiwgY3AyLnJCLCBiMlZlYzIuc190MCksXHJcbiAgICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkEsIHdBLCBjcDIuckEsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICAgIGR2MixcclxuICAgICAgICApO1xyXG5cclxuICAgICAgICAvLyBDb21wdXRlIG5vcm1hbCB2ZWxvY2l0eVxyXG4gICAgICAgIC8vIGZsb2F0MzIgdm4xID0gYjJEb3QoZHYxLCBub3JtYWwpO1xyXG4gICAgICAgIGxldCB2bjE6IG51bWJlciA9IGIyVmVjMi5Eb3RWVihkdjEsIG5vcm1hbCk7XHJcbiAgICAgICAgLy8gZmxvYXQzMiB2bjIgPSBiMkRvdChkdjIsIG5vcm1hbCk7XHJcbiAgICAgICAgbGV0IHZuMjogbnVtYmVyID0gYjJWZWMyLkRvdFZWKGR2Miwgbm9ybWFsKTtcclxuXHJcbiAgICAgICAgLy8gYjJWZWMyIGI7XHJcbiAgICAgICAgYi54ID0gdm4xIC0gY3AxLnZlbG9jaXR5QmlhcztcclxuICAgICAgICBiLnkgPSB2bjIgLSBjcDIudmVsb2NpdHlCaWFzO1xyXG5cclxuICAgICAgICAvLyBDb21wdXRlIGInXHJcbiAgICAgICAgLy8gYiAtPSBiMk11bCh2Yy0+SywgYSk7XHJcbiAgICAgICAgYi5TZWxmU3ViKGIyTWF0MjIuTXVsTVYodmMuSywgYSwgYjJWZWMyLnNfdDApKTtcclxuXHJcbiAgICAgICAgLypcclxuICAgICAgICAgICAgICAgICNpZiBCMl9ERUJVR19TT0xWRVIgPT09IDFcclxuICAgICAgICAgICAgICAgIGNvbnN0IGtfZXJyb3JUb2w6IG51bWJlciA9IDAuMDAxO1xyXG4gICAgICAgICAgICAgICAgI2VuZGlmXHJcbiAgICAgICAgICAgICAgICAqL1xyXG5cclxuICAgICAgICBmb3IgKDs7KSB7XHJcbiAgICAgICAgICAvL1xyXG4gICAgICAgICAgLy8gQ2FzZSAxOiB2biA9IDBcclxuICAgICAgICAgIC8vXHJcbiAgICAgICAgICAvLyAwID0gQSAqIHggKyBiJ1xyXG4gICAgICAgICAgLy9cclxuICAgICAgICAgIC8vIFNvbHZlIGZvciB4OlxyXG4gICAgICAgICAgLy9cclxuICAgICAgICAgIC8vIHggPSAtIGludihBKSAqIGInXHJcbiAgICAgICAgICAvL1xyXG4gICAgICAgICAgLy8gYjJWZWMyIHggPSAtIGIyTXVsKHZjLT5ub3JtYWxNYXNzLCBiKTtcclxuICAgICAgICAgIGIyTWF0MjIuTXVsTVYodmMubm9ybWFsTWFzcywgYiwgeCkuU2VsZk5lZygpO1xyXG5cclxuICAgICAgICAgIGlmICh4LnggPj0gMCAmJiB4LnkgPj0gMCkge1xyXG4gICAgICAgICAgICAvLyBHZXQgdGhlIGluY3JlbWVudGFsIGltcHVsc2VcclxuICAgICAgICAgICAgLy8gYjJWZWMyIGQgPSB4IC0gYTtcclxuICAgICAgICAgICAgYjJWZWMyLlN1YlZWKHgsIGEsIGQpO1xyXG5cclxuICAgICAgICAgICAgLy8gQXBwbHkgaW5jcmVtZW50YWwgaW1wdWxzZVxyXG4gICAgICAgICAgICAvLyBiMlZlYzIgUDEgPSBkLnggKiBub3JtYWw7XHJcbiAgICAgICAgICAgIGIyVmVjMi5NdWxTVihkLngsIG5vcm1hbCwgUDEpO1xyXG4gICAgICAgICAgICAvLyBiMlZlYzIgUDIgPSBkLnkgKiBub3JtYWw7XHJcbiAgICAgICAgICAgIGIyVmVjMi5NdWxTVihkLnksIG5vcm1hbCwgUDIpO1xyXG4gICAgICAgICAgICBiMlZlYzIuQWRkVlYoUDEsIFAyLCBQMVAyKTtcclxuICAgICAgICAgICAgLy8gdkEgLT0gbUEgKiAoUDEgKyBQMik7XHJcbiAgICAgICAgICAgIHZBLlNlbGZNdWxTdWIobUEsIFAxUDIpO1xyXG4gICAgICAgICAgICAvLyB3QSAtPSBpQSAqIChiMkNyb3NzKGNwMS0+ckEsIFAxKSArIGIyQ3Jvc3MoY3AyLT5yQSwgUDIpKTtcclxuICAgICAgICAgICAgd0EgLT0gaUEgKiAoYjJWZWMyLkNyb3NzVlYoY3AxLnJBLCBQMSkgKyBiMlZlYzIuQ3Jvc3NWVihjcDIuckEsIFAyKSk7XHJcblxyXG4gICAgICAgICAgICAvLyB2QiArPSBtQiAqIChQMSArIFAyKTtcclxuICAgICAgICAgICAgdkIuU2VsZk11bEFkZChtQiwgUDFQMik7XHJcbiAgICAgICAgICAgIC8vIHdCICs9IGlCICogKGIyQ3Jvc3MoY3AxLT5yQiwgUDEpICsgYjJDcm9zcyhjcDItPnJCLCBQMikpO1xyXG4gICAgICAgICAgICB3QiArPSBpQiAqIChiMlZlYzIuQ3Jvc3NWVihjcDEuckIsIFAxKSArIGIyVmVjMi5Dcm9zc1ZWKGNwMi5yQiwgUDIpKTtcclxuXHJcbiAgICAgICAgICAgIC8vIEFjY3VtdWxhdGVcclxuICAgICAgICAgICAgY3AxLm5vcm1hbEltcHVsc2UgPSB4Lng7XHJcbiAgICAgICAgICAgIGNwMi5ub3JtYWxJbXB1bHNlID0geC55O1xyXG5cclxuICAgICAgICAgICAgLypcclxuICAgICAgICAgICAgICAgICAgICAgICAgI2lmIEIyX0RFQlVHX1NPTFZFUiA9PT0gMVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAvLyBQb3N0Y29uZGl0aW9uc1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBkdjEgPSB2QiArIGIyQ3Jvc3Mod0IsIGNwMS0+ckIpIC0gdkEgLSBiMkNyb3NzKHdBLCBjcDEtPnJBKTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgZHYyID0gdkIgKyBiMkNyb3NzKHdCLCBjcDItPnJCKSAtIHZBIC0gYjJDcm9zcyh3QSwgY3AyLT5yQSk7XHJcblxyXG4gICAgICAgICAgICAgICAgICAgICAgICAvLyBDb21wdXRlIG5vcm1hbCB2ZWxvY2l0eVxyXG4gICAgICAgICAgICAgICAgICAgICAgICB2bjEgPSBiMkRvdChkdjEsIG5vcm1hbCk7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHZuMiA9IGIyRG90KGR2Miwgbm9ybWFsKTtcclxuXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIGIyQXNzZXJ0KGIyQWJzKHZuMSAtIGNwMS0+dmVsb2NpdHlCaWFzKSA8IGtfZXJyb3JUb2wpO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBiMkFzc2VydChiMkFicyh2bjIgLSBjcDItPnZlbG9jaXR5QmlhcykgPCBrX2Vycm9yVG9sKTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgI2VuZGlmXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICovXHJcbiAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgfVxyXG5cclxuICAgICAgICAgIC8vXHJcbiAgICAgICAgICAvLyBDYXNlIDI6IHZuMSA9IDAgYW5kIHgyID0gMFxyXG4gICAgICAgICAgLy9cclxuICAgICAgICAgIC8vICAgMCA9IGExMSAqIHgxICsgYTEyICogMCArIGIxJ1xyXG4gICAgICAgICAgLy8gdm4yID0gYTIxICogeDEgKyBhMjIgKiAwICsgYjInXHJcbiAgICAgICAgICAvL1xyXG4gICAgICAgICAgeC54ID0gLWNwMS5ub3JtYWxNYXNzICogYi54O1xyXG4gICAgICAgICAgeC55ID0gMDtcclxuICAgICAgICAgIHZuMSA9IDA7XHJcbiAgICAgICAgICB2bjIgPSB2Yy5LLmV4LnkgKiB4LnggKyBiLnk7XHJcblxyXG4gICAgICAgICAgaWYgKHgueCA+PSAwICYmIHZuMiA+PSAwKSB7XHJcbiAgICAgICAgICAgIC8vIEdldCB0aGUgaW5jcmVtZW50YWwgaW1wdWxzZVxyXG4gICAgICAgICAgICAvLyBiMlZlYzIgZCA9IHggLSBhO1xyXG4gICAgICAgICAgICBiMlZlYzIuU3ViVlYoeCwgYSwgZCk7XHJcblxyXG4gICAgICAgICAgICAvLyBBcHBseSBpbmNyZW1lbnRhbCBpbXB1bHNlXHJcbiAgICAgICAgICAgIC8vIGIyVmVjMiBQMSA9IGQueCAqIG5vcm1hbDtcclxuICAgICAgICAgICAgYjJWZWMyLk11bFNWKGQueCwgbm9ybWFsLCBQMSk7XHJcbiAgICAgICAgICAgIC8vIGIyVmVjMiBQMiA9IGQueSAqIG5vcm1hbDtcclxuICAgICAgICAgICAgYjJWZWMyLk11bFNWKGQueSwgbm9ybWFsLCBQMik7XHJcbiAgICAgICAgICAgIGIyVmVjMi5BZGRWVihQMSwgUDIsIFAxUDIpO1xyXG4gICAgICAgICAgICAvLyB2QSAtPSBtQSAqIChQMSArIFAyKTtcclxuICAgICAgICAgICAgdkEuU2VsZk11bFN1YihtQSwgUDFQMik7XHJcbiAgICAgICAgICAgIC8vIHdBIC09IGlBICogKGIyQ3Jvc3MoY3AxLT5yQSwgUDEpICsgYjJDcm9zcyhjcDItPnJBLCBQMikpO1xyXG4gICAgICAgICAgICB3QSAtPSBpQSAqIChiMlZlYzIuQ3Jvc3NWVihjcDEuckEsIFAxKSArIGIyVmVjMi5Dcm9zc1ZWKGNwMi5yQSwgUDIpKTtcclxuXHJcbiAgICAgICAgICAgIC8vIHZCICs9IG1CICogKFAxICsgUDIpO1xyXG4gICAgICAgICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQMVAyKTtcclxuICAgICAgICAgICAgLy8gd0IgKz0gaUIgKiAoYjJDcm9zcyhjcDEtPnJCLCBQMSkgKyBiMkNyb3NzKGNwMi0+ckIsIFAyKSk7XHJcbiAgICAgICAgICAgIHdCICs9IGlCICogKGIyVmVjMi5Dcm9zc1ZWKGNwMS5yQiwgUDEpICsgYjJWZWMyLkNyb3NzVlYoY3AyLnJCLCBQMikpO1xyXG5cclxuICAgICAgICAgICAgLy8gQWNjdW11bGF0ZVxyXG4gICAgICAgICAgICBjcDEubm9ybWFsSW1wdWxzZSA9IHgueDtcclxuICAgICAgICAgICAgY3AyLm5vcm1hbEltcHVsc2UgPSB4Lnk7XHJcblxyXG4gICAgICAgICAgICAvKlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAjaWYgQjJfREVCVUdfU09MVkVSID09PSAxXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIC8vIFBvc3Rjb25kaXRpb25zXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIGR2MSA9IHZCICsgYjJDcm9zcyh3QiwgY3AxLT5yQikgLSB2QSAtIGIyQ3Jvc3Mod0EsIGNwMS0+ckEpO1xyXG5cclxuICAgICAgICAgICAgICAgICAgICAgICAgLy8gQ29tcHV0ZSBub3JtYWwgdmVsb2NpdHlcclxuICAgICAgICAgICAgICAgICAgICAgICAgdm4xID0gYjJEb3QoZHYxLCBub3JtYWwpO1xyXG5cclxuICAgICAgICAgICAgICAgICAgICAgICAgYjJBc3NlcnQoYjJBYnModm4xIC0gY3AxLT52ZWxvY2l0eUJpYXMpIDwga19lcnJvclRvbCk7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICNlbmRpZlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAqL1xyXG4gICAgICAgICAgICBicmVhaztcclxuICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAvL1xyXG4gICAgICAgICAgLy8gQ2FzZSAzOiB2bjIgPSAwIGFuZCB4MSA9IDBcclxuICAgICAgICAgIC8vXHJcbiAgICAgICAgICAvLyB2bjEgPSBhMTEgKiAwICsgYTEyICogeDIgKyBiMSdcclxuICAgICAgICAgIC8vICAgMCA9IGEyMSAqIDAgKyBhMjIgKiB4MiArIGIyJ1xyXG4gICAgICAgICAgLy9cclxuICAgICAgICAgIHgueCA9IDA7XHJcbiAgICAgICAgICB4LnkgPSAtY3AyLm5vcm1hbE1hc3MgKiBiLnk7XHJcbiAgICAgICAgICB2bjEgPSB2Yy5LLmV5LnggKiB4LnkgKyBiLng7XHJcbiAgICAgICAgICB2bjIgPSAwO1xyXG5cclxuICAgICAgICAgIGlmICh4LnkgPj0gMCAmJiB2bjEgPj0gMCkge1xyXG4gICAgICAgICAgICAvLyBSZXN1YnN0aXR1dGUgZm9yIHRoZSBpbmNyZW1lbnRhbCBpbXB1bHNlXHJcbiAgICAgICAgICAgIC8vIGIyVmVjMiBkID0geCAtIGE7XHJcbiAgICAgICAgICAgIGIyVmVjMi5TdWJWVih4LCBhLCBkKTtcclxuXHJcbiAgICAgICAgICAgIC8vIEFwcGx5IGluY3JlbWVudGFsIGltcHVsc2VcclxuICAgICAgICAgICAgLy8gYjJWZWMyIFAxID0gZC54ICogbm9ybWFsO1xyXG4gICAgICAgICAgICBiMlZlYzIuTXVsU1YoZC54LCBub3JtYWwsIFAxKTtcclxuICAgICAgICAgICAgLy8gYjJWZWMyIFAyID0gZC55ICogbm9ybWFsO1xyXG4gICAgICAgICAgICBiMlZlYzIuTXVsU1YoZC55LCBub3JtYWwsIFAyKTtcclxuICAgICAgICAgICAgYjJWZWMyLkFkZFZWKFAxLCBQMiwgUDFQMik7XHJcbiAgICAgICAgICAgIC8vIHZBIC09IG1BICogKFAxICsgUDIpO1xyXG4gICAgICAgICAgICB2QS5TZWxmTXVsU3ViKG1BLCBQMVAyKTtcclxuICAgICAgICAgICAgLy8gd0EgLT0gaUEgKiAoYjJDcm9zcyhjcDEtPnJBLCBQMSkgKyBiMkNyb3NzKGNwMi0+ckEsIFAyKSk7XHJcbiAgICAgICAgICAgIHdBIC09IGlBICogKGIyVmVjMi5Dcm9zc1ZWKGNwMS5yQSwgUDEpICsgYjJWZWMyLkNyb3NzVlYoY3AyLnJBLCBQMikpO1xyXG5cclxuICAgICAgICAgICAgLy8gdkIgKz0gbUIgKiAoUDEgKyBQMik7XHJcbiAgICAgICAgICAgIHZCLlNlbGZNdWxBZGQobUIsIFAxUDIpO1xyXG4gICAgICAgICAgICAvLyB3QiArPSBpQiAqIChiMkNyb3NzKGNwMS0+ckIsIFAxKSArIGIyQ3Jvc3MoY3AyLT5yQiwgUDIpKTtcclxuICAgICAgICAgICAgd0IgKz0gaUIgKiAoYjJWZWMyLkNyb3NzVlYoY3AxLnJCLCBQMSkgKyBiMlZlYzIuQ3Jvc3NWVihjcDIuckIsIFAyKSk7XHJcblxyXG4gICAgICAgICAgICAvLyBBY2N1bXVsYXRlXHJcbiAgICAgICAgICAgIGNwMS5ub3JtYWxJbXB1bHNlID0geC54O1xyXG4gICAgICAgICAgICBjcDIubm9ybWFsSW1wdWxzZSA9IHgueTtcclxuXHJcbiAgICAgICAgICAgIC8qXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICNpZiBCMl9ERUJVR19TT0xWRVIgPT09IDFcclxuICAgICAgICAgICAgICAgICAgICAgICAgLy8gUG9zdGNvbmRpdGlvbnNcclxuICAgICAgICAgICAgICAgICAgICAgICAgZHYyID0gdkIgKyBiMkNyb3NzKHdCLCBjcDItPnJCKSAtIHZBIC0gYjJDcm9zcyh3QSwgY3AyLT5yQSk7XHJcblxyXG4gICAgICAgICAgICAgICAgICAgICAgICAvLyBDb21wdXRlIG5vcm1hbCB2ZWxvY2l0eVxyXG4gICAgICAgICAgICAgICAgICAgICAgICB2bjIgPSBiMkRvdChkdjIsIG5vcm1hbCk7XHJcblxyXG4gICAgICAgICAgICAgICAgICAgICAgICBiMkFzc2VydChiMkFicyh2bjIgLSBjcDItPnZlbG9jaXR5QmlhcykgPCBrX2Vycm9yVG9sKTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgI2VuZGlmXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICovXHJcbiAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgfVxyXG5cclxuICAgICAgICAgIC8vXHJcbiAgICAgICAgICAvLyBDYXNlIDQ6IHgxID0gMCBhbmQgeDIgPSAwXHJcbiAgICAgICAgICAvL1xyXG4gICAgICAgICAgLy8gdm4xID0gYjFcclxuICAgICAgICAgIC8vIHZuMiA9IGIyO1xyXG4gICAgICAgICAgeC54ID0gMDtcclxuICAgICAgICAgIHgueSA9IDA7XHJcbiAgICAgICAgICB2bjEgPSBiLng7XHJcbiAgICAgICAgICB2bjIgPSBiLnk7XHJcblxyXG4gICAgICAgICAgaWYgKHZuMSA+PSAwICYmIHZuMiA+PSAwKSB7XHJcbiAgICAgICAgICAgIC8vIFJlc3Vic3RpdHV0ZSBmb3IgdGhlIGluY3JlbWVudGFsIGltcHVsc2VcclxuICAgICAgICAgICAgLy8gYjJWZWMyIGQgPSB4IC0gYTtcclxuICAgICAgICAgICAgYjJWZWMyLlN1YlZWKHgsIGEsIGQpO1xyXG5cclxuICAgICAgICAgICAgLy8gQXBwbHkgaW5jcmVtZW50YWwgaW1wdWxzZVxyXG4gICAgICAgICAgICAvLyBiMlZlYzIgUDEgPSBkLnggKiBub3JtYWw7XHJcbiAgICAgICAgICAgIGIyVmVjMi5NdWxTVihkLngsIG5vcm1hbCwgUDEpO1xyXG4gICAgICAgICAgICAvLyBiMlZlYzIgUDIgPSBkLnkgKiBub3JtYWw7XHJcbiAgICAgICAgICAgIGIyVmVjMi5NdWxTVihkLnksIG5vcm1hbCwgUDIpO1xyXG4gICAgICAgICAgICBiMlZlYzIuQWRkVlYoUDEsIFAyLCBQMVAyKTtcclxuICAgICAgICAgICAgLy8gdkEgLT0gbUEgKiAoUDEgKyBQMik7XHJcbiAgICAgICAgICAgIHZBLlNlbGZNdWxTdWIobUEsIFAxUDIpO1xyXG4gICAgICAgICAgICAvLyB3QSAtPSBpQSAqIChiMkNyb3NzKGNwMS0+ckEsIFAxKSArIGIyQ3Jvc3MoY3AyLT5yQSwgUDIpKTtcclxuICAgICAgICAgICAgd0EgLT0gaUEgKiAoYjJWZWMyLkNyb3NzVlYoY3AxLnJBLCBQMSkgKyBiMlZlYzIuQ3Jvc3NWVihjcDIuckEsIFAyKSk7XHJcblxyXG4gICAgICAgICAgICAvLyB2QiArPSBtQiAqIChQMSArIFAyKTtcclxuICAgICAgICAgICAgdkIuU2VsZk11bEFkZChtQiwgUDFQMik7XHJcbiAgICAgICAgICAgIC8vIHdCICs9IGlCICogKGIyQ3Jvc3MoY3AxLT5yQiwgUDEpICsgYjJDcm9zcyhjcDItPnJCLCBQMikpO1xyXG4gICAgICAgICAgICB3QiArPSBpQiAqIChiMlZlYzIuQ3Jvc3NWVihjcDEuckIsIFAxKSArIGIyVmVjMi5Dcm9zc1ZWKGNwMi5yQiwgUDIpKTtcclxuXHJcbiAgICAgICAgICAgIC8vIEFjY3VtdWxhdGVcclxuICAgICAgICAgICAgY3AxLm5vcm1hbEltcHVsc2UgPSB4Lng7XHJcbiAgICAgICAgICAgIGNwMi5ub3JtYWxJbXB1bHNlID0geC55O1xyXG5cclxuICAgICAgICAgICAgYnJlYWs7XHJcbiAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgLy8gTm8gc29sdXRpb24sIGdpdmUgdXAuIFRoaXMgaXMgaGl0IHNvbWV0aW1lcywgYnV0IGl0IGRvZXNuJ3Qgc2VlbSB0byBtYXR0ZXIuXHJcbiAgICAgICAgICBicmVhaztcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIHRoaXMubV92ZWxvY2l0aWVzW2luZGV4QV0udiA9IHZBO1xyXG4gICAgICB0aGlzLm1fdmVsb2NpdGllc1tpbmRleEFdLncgPSB3QTtcclxuICAgICAgLy8gdGhpcy5tX3ZlbG9jaXRpZXNbaW5kZXhCXS52ID0gdkI7XHJcbiAgICAgIHRoaXMubV92ZWxvY2l0aWVzW2luZGV4Ql0udyA9IHdCO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgU3RvcmVJbXB1bHNlcygpOiB2b2lkIHtcclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgdmM6IGIyQ29udGFjdFZlbG9jaXR5Q29uc3RyYWludCA9IHRoaXMubV92ZWxvY2l0eUNvbnN0cmFpbnRzW2ldO1xyXG4gICAgICBjb25zdCBtYW5pZm9sZDogYjJNYW5pZm9sZCA9IHRoaXMubV9jb250YWN0c1t2Yy5jb250YWN0SW5kZXhdLkdldE1hbmlmb2xkKCk7XHJcblxyXG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHZjLnBvaW50Q291bnQ7ICsraikge1xyXG4gICAgICAgIG1hbmlmb2xkLnBvaW50c1tqXS5ub3JtYWxJbXB1bHNlID0gdmMucG9pbnRzW2pdLm5vcm1hbEltcHVsc2U7XHJcbiAgICAgICAgbWFuaWZvbGQucG9pbnRzW2pdLnRhbmdlbnRJbXB1bHNlID0gdmMucG9pbnRzW2pdLnRhbmdlbnRJbXB1bHNlO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc194ZkEgPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc194ZkIgPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc19wc20gPSBuZXcgYjJQb3NpdGlvblNvbHZlck1hbmlmb2xkKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfUCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzKCk6IGJvb2xlYW4ge1xyXG4gICAgY29uc3QgeGZBOiBiMlRyYW5zZm9ybSA9IGIyQ29udGFjdFNvbHZlci5Tb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc194ZkE7XHJcbiAgICBjb25zdCB4ZkI6IGIyVHJhbnNmb3JtID0gYjJDb250YWN0U29sdmVyLlNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX3hmQjtcclxuICAgIGNvbnN0IHBzbTogYjJQb3NpdGlvblNvbHZlck1hbmlmb2xkID0gYjJDb250YWN0U29sdmVyLlNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX3BzbTtcclxuICAgIGNvbnN0IHJBOiBiMlZlYzIgPSBiMkNvbnRhY3RTb2x2ZXIuU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckE7XHJcbiAgICBjb25zdCByQjogYjJWZWMyID0gYjJDb250YWN0U29sdmVyLlNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX3JCO1xyXG4gICAgY29uc3QgUDogYjJWZWMyID0gYjJDb250YWN0U29sdmVyLlNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX1A7XHJcblxyXG4gICAgbGV0IG1pblNlcGFyYXRpb24gPSAwO1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgcGM6IGIyQ29udGFjdFBvc2l0aW9uQ29uc3RyYWludCA9IHRoaXMubV9wb3NpdGlvbkNvbnN0cmFpbnRzW2ldO1xyXG5cclxuICAgICAgY29uc3QgaW5kZXhBOiBudW1iZXIgPSBwYy5pbmRleEE7XHJcbiAgICAgIGNvbnN0IGluZGV4QjogbnVtYmVyID0gcGMuaW5kZXhCO1xyXG4gICAgICBjb25zdCBsb2NhbENlbnRlckE6IGIyVmVjMiA9IHBjLmxvY2FsQ2VudGVyQTtcclxuICAgICAgY29uc3QgbUE6IG51bWJlciA9IHBjLmludk1hc3NBO1xyXG4gICAgICBjb25zdCBpQTogbnVtYmVyID0gcGMuaW52SUE7XHJcbiAgICAgIGNvbnN0IGxvY2FsQ2VudGVyQjogYjJWZWMyID0gcGMubG9jYWxDZW50ZXJCO1xyXG4gICAgICBjb25zdCBtQjogbnVtYmVyID0gcGMuaW52TWFzc0I7XHJcbiAgICAgIGNvbnN0IGlCOiBudW1iZXIgPSBwYy5pbnZJQjtcclxuICAgICAgY29uc3QgcG9pbnRDb3VudDogbnVtYmVyID0gcGMucG9pbnRDb3VudDtcclxuXHJcbiAgICAgIGNvbnN0IGNBOiBiMlZlYzIgPSB0aGlzLm1fcG9zaXRpb25zW2luZGV4QV0uYztcclxuICAgICAgbGV0IGFBOiBudW1iZXIgPSB0aGlzLm1fcG9zaXRpb25zW2luZGV4QV0uYTtcclxuXHJcbiAgICAgIGNvbnN0IGNCOiBiMlZlYzIgPSB0aGlzLm1fcG9zaXRpb25zW2luZGV4Ql0uYztcclxuICAgICAgbGV0IGFCOiBudW1iZXIgPSB0aGlzLm1fcG9zaXRpb25zW2luZGV4Ql0uYTtcclxuXHJcbiAgICAgIC8vIFNvbHZlIG5vcm1hbCBjb25zdHJhaW50c1xyXG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHBvaW50Q291bnQ7ICsraikge1xyXG4gICAgICAgIHhmQS5xLlNldEFuZ2xlKGFBKTtcclxuICAgICAgICB4ZkIucS5TZXRBbmdsZShhQik7XHJcbiAgICAgICAgYjJWZWMyLlN1YlZWKGNBLCBiMlJvdC5NdWxSVih4ZkEucSwgbG9jYWxDZW50ZXJBLCBiMlZlYzIuc190MCksIHhmQS5wKTtcclxuICAgICAgICBiMlZlYzIuU3ViVlYoY0IsIGIyUm90Lk11bFJWKHhmQi5xLCBsb2NhbENlbnRlckIsIGIyVmVjMi5zX3QwKSwgeGZCLnApO1xyXG5cclxuICAgICAgICBwc20uSW5pdGlhbGl6ZShwYywgeGZBLCB4ZkIsIGopO1xyXG4gICAgICAgIGNvbnN0IG5vcm1hbDogYjJWZWMyID0gcHNtLm5vcm1hbDtcclxuXHJcbiAgICAgICAgY29uc3QgcG9pbnQ6IGIyVmVjMiA9IHBzbS5wb2ludDtcclxuICAgICAgICBjb25zdCBzZXBhcmF0aW9uOiBudW1iZXIgPSBwc20uc2VwYXJhdGlvbjtcclxuXHJcbiAgICAgICAgLy8gYjJWZWMyIHJBID0gcG9pbnQgLSBjQTtcclxuICAgICAgICBiMlZlYzIuU3ViVlYocG9pbnQsIGNBLCByQSk7XHJcbiAgICAgICAgLy8gYjJWZWMyIHJCID0gcG9pbnQgLSBjQjtcclxuICAgICAgICBiMlZlYzIuU3ViVlYocG9pbnQsIGNCLCByQik7XHJcblxyXG4gICAgICAgIC8vIFRyYWNrIG1heCBjb25zdHJhaW50IGVycm9yLlxyXG4gICAgICAgIG1pblNlcGFyYXRpb24gPSBiMk1pbihtaW5TZXBhcmF0aW9uLCBzZXBhcmF0aW9uKTtcclxuXHJcbiAgICAgICAgLy8gUHJldmVudCBsYXJnZSBjb3JyZWN0aW9ucyBhbmQgYWxsb3cgc2xvcC5cclxuICAgICAgICBjb25zdCBDOiBudW1iZXIgPSBiMkNsYW1wKFxyXG4gICAgICAgICAgYjJfYmF1bWdhcnRlICogKHNlcGFyYXRpb24gKyBiMl9saW5lYXJTbG9wKSxcclxuICAgICAgICAgIC1iMl9tYXhMaW5lYXJDb3JyZWN0aW9uLFxyXG4gICAgICAgICAgMCxcclxuICAgICAgICApO1xyXG5cclxuICAgICAgICAvLyBDb21wdXRlIHRoZSBlZmZlY3RpdmUgbWFzcy5cclxuICAgICAgICAvLyBmbG9hdDMyIHJuQSA9IGIyQ3Jvc3MockEsIG5vcm1hbCk7XHJcbiAgICAgICAgY29uc3Qgcm5BOiBudW1iZXIgPSBiMlZlYzIuQ3Jvc3NWVihyQSwgbm9ybWFsKTtcclxuICAgICAgICAvLyBmbG9hdDMyIHJuQiA9IGIyQ3Jvc3MockIsIG5vcm1hbCk7XHJcbiAgICAgICAgY29uc3Qgcm5COiBudW1iZXIgPSBiMlZlYzIuQ3Jvc3NWVihyQiwgbm9ybWFsKTtcclxuICAgICAgICAvLyBmbG9hdDMyIEsgPSBtQSArIG1CICsgaUEgKiBybkEgKiBybkEgKyBpQiAqIHJuQiAqIHJuQjtcclxuICAgICAgICBjb25zdCBLOiBudW1iZXIgPSBtQSArIG1CICsgaUEgKiBybkEgKiBybkEgKyBpQiAqIHJuQiAqIHJuQjtcclxuXHJcbiAgICAgICAgLy8gQ29tcHV0ZSBub3JtYWwgaW1wdWxzZVxyXG4gICAgICAgIGNvbnN0IGltcHVsc2U6IG51bWJlciA9IEsgPiAwID8gLUMgLyBLIDogMDtcclxuXHJcbiAgICAgICAgLy8gYjJWZWMyIFAgPSBpbXB1bHNlICogbm9ybWFsO1xyXG4gICAgICAgIGIyVmVjMi5NdWxTVihpbXB1bHNlLCBub3JtYWwsIFApO1xyXG5cclxuICAgICAgICAvLyBjQSAtPSBtQSAqIFA7XHJcbiAgICAgICAgY0EuU2VsZk11bFN1YihtQSwgUCk7XHJcbiAgICAgICAgLy8gYUEgLT0gaUEgKiBiMkNyb3NzKHJBLCBQKTtcclxuICAgICAgICBhQSAtPSBpQSAqIGIyVmVjMi5Dcm9zc1ZWKHJBLCBQKTtcclxuXHJcbiAgICAgICAgLy8gY0IgKz0gbUIgKiBQO1xyXG4gICAgICAgIGNCLlNlbGZNdWxBZGQobUIsIFApO1xyXG4gICAgICAgIC8vIGFCICs9IGlCICogYjJDcm9zcyhyQiwgUCk7XHJcbiAgICAgICAgYUIgKz0gaUIgKiBiMlZlYzIuQ3Jvc3NWVihyQiwgUCk7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIHRoaXMubV9wb3NpdGlvbnNbaW5kZXhBXS5jID0gY0E7XHJcbiAgICAgIHRoaXMubV9wb3NpdGlvbnNbaW5kZXhBXS5hID0gYUE7XHJcblxyXG4gICAgICAvLyB0aGlzLm1fcG9zaXRpb25zW2luZGV4Ql0uYyA9IGNCO1xyXG4gICAgICB0aGlzLm1fcG9zaXRpb25zW2luZGV4Ql0uYSA9IGFCO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFdlIGNhbid0IGV4cGVjdCBtaW5TcGVwYXJhdGlvbiA+PSAtYjJfbGluZWFyU2xvcCBiZWNhdXNlIHdlIGRvbid0XHJcbiAgICAvLyBwdXNoIHRoZSBzZXBhcmF0aW9uIGFib3ZlIC1iMl9saW5lYXJTbG9wLlxyXG4gICAgcmV0dXJuIG1pblNlcGFyYXRpb24gPiAtMyAqIGIyX2xpbmVhclNsb3A7XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVRPSVBvc2l0aW9uQ29uc3RyYWludHNfc194ZkEgPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVRPSVBvc2l0aW9uQ29uc3RyYWludHNfc194ZkIgPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVRPSVBvc2l0aW9uQ29uc3RyYWludHNfc19wc20gPSBuZXcgYjJQb3NpdGlvblNvbHZlck1hbmlmb2xkKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVUT0lQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVUT0lQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVUT0lQb3NpdGlvbkNvbnN0cmFpbnRzX3NfUCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU29sdmVUT0lQb3NpdGlvbkNvbnN0cmFpbnRzKHRvaUluZGV4QTogbnVtYmVyLCB0b2lJbmRleEI6IG51bWJlcik6IGJvb2xlYW4ge1xyXG4gICAgY29uc3QgeGZBOiBiMlRyYW5zZm9ybSA9IGIyQ29udGFjdFNvbHZlci5Tb2x2ZVRPSVBvc2l0aW9uQ29uc3RyYWludHNfc194ZkE7XHJcbiAgICBjb25zdCB4ZkI6IGIyVHJhbnNmb3JtID0gYjJDb250YWN0U29sdmVyLlNvbHZlVE9JUG9zaXRpb25Db25zdHJhaW50c19zX3hmQjtcclxuICAgIGNvbnN0IHBzbTogYjJQb3NpdGlvblNvbHZlck1hbmlmb2xkID0gYjJDb250YWN0U29sdmVyLlNvbHZlVE9JUG9zaXRpb25Db25zdHJhaW50c19zX3BzbTtcclxuICAgIGNvbnN0IHJBOiBiMlZlYzIgPSBiMkNvbnRhY3RTb2x2ZXIuU29sdmVUT0lQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckE7XHJcbiAgICBjb25zdCByQjogYjJWZWMyID0gYjJDb250YWN0U29sdmVyLlNvbHZlVE9JUG9zaXRpb25Db25zdHJhaW50c19zX3JCO1xyXG4gICAgY29uc3QgUDogYjJWZWMyID0gYjJDb250YWN0U29sdmVyLlNvbHZlVE9JUG9zaXRpb25Db25zdHJhaW50c19zX1A7XHJcblxyXG4gICAgbGV0IG1pblNlcGFyYXRpb24gPSAwO1xyXG5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX2NvdW50OyArK2kpIHtcclxuICAgICAgY29uc3QgcGM6IGIyQ29udGFjdFBvc2l0aW9uQ29uc3RyYWludCA9IHRoaXMubV9wb3NpdGlvbkNvbnN0cmFpbnRzW2ldO1xyXG5cclxuICAgICAgY29uc3QgaW5kZXhBOiBudW1iZXIgPSBwYy5pbmRleEE7XHJcbiAgICAgIGNvbnN0IGluZGV4QjogbnVtYmVyID0gcGMuaW5kZXhCO1xyXG4gICAgICBjb25zdCBsb2NhbENlbnRlckE6IGIyVmVjMiA9IHBjLmxvY2FsQ2VudGVyQTtcclxuICAgICAgY29uc3QgbG9jYWxDZW50ZXJCOiBiMlZlYzIgPSBwYy5sb2NhbENlbnRlckI7XHJcbiAgICAgIGNvbnN0IHBvaW50Q291bnQ6IG51bWJlciA9IHBjLnBvaW50Q291bnQ7XHJcblxyXG4gICAgICBsZXQgbUEgPSAwO1xyXG4gICAgICBsZXQgaUEgPSAwO1xyXG4gICAgICBpZiAoaW5kZXhBID09PSB0b2lJbmRleEEgfHwgaW5kZXhBID09PSB0b2lJbmRleEIpIHtcclxuICAgICAgICBtQSA9IHBjLmludk1hc3NBO1xyXG4gICAgICAgIGlBID0gcGMuaW52SUE7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIGxldCBtQiA9IDA7XHJcbiAgICAgIGxldCBpQiA9IDA7XHJcbiAgICAgIGlmIChpbmRleEIgPT09IHRvaUluZGV4QSB8fCBpbmRleEIgPT09IHRvaUluZGV4Qikge1xyXG4gICAgICAgIG1CID0gcGMuaW52TWFzc0I7XHJcbiAgICAgICAgaUIgPSBwYy5pbnZJQjtcclxuICAgICAgfVxyXG5cclxuICAgICAgY29uc3QgY0E6IGIyVmVjMiA9IHRoaXMubV9wb3NpdGlvbnNbaW5kZXhBXS5jO1xyXG4gICAgICBsZXQgYUE6IG51bWJlciA9IHRoaXMubV9wb3NpdGlvbnNbaW5kZXhBXS5hO1xyXG5cclxuICAgICAgY29uc3QgY0I6IGIyVmVjMiA9IHRoaXMubV9wb3NpdGlvbnNbaW5kZXhCXS5jO1xyXG4gICAgICBsZXQgYUI6IG51bWJlciA9IHRoaXMubV9wb3NpdGlvbnNbaW5kZXhCXS5hO1xyXG5cclxuICAgICAgLy8gU29sdmUgbm9ybWFsIGNvbnN0cmFpbnRzXHJcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgcG9pbnRDb3VudDsgKytqKSB7XHJcbiAgICAgICAgeGZBLnEuU2V0QW5nbGUoYUEpO1xyXG4gICAgICAgIHhmQi5xLlNldEFuZ2xlKGFCKTtcclxuICAgICAgICBiMlZlYzIuU3ViVlYoY0EsIGIyUm90Lk11bFJWKHhmQS5xLCBsb2NhbENlbnRlckEsIGIyVmVjMi5zX3QwKSwgeGZBLnApO1xyXG4gICAgICAgIGIyVmVjMi5TdWJWVihjQiwgYjJSb3QuTXVsUlYoeGZCLnEsIGxvY2FsQ2VudGVyQiwgYjJWZWMyLnNfdDApLCB4ZkIucCk7XHJcblxyXG4gICAgICAgIHBzbS5Jbml0aWFsaXplKHBjLCB4ZkEsIHhmQiwgaik7XHJcbiAgICAgICAgY29uc3Qgbm9ybWFsOiBiMlZlYzIgPSBwc20ubm9ybWFsO1xyXG5cclxuICAgICAgICBjb25zdCBwb2ludDogYjJWZWMyID0gcHNtLnBvaW50O1xyXG4gICAgICAgIGNvbnN0IHNlcGFyYXRpb246IG51bWJlciA9IHBzbS5zZXBhcmF0aW9uO1xyXG5cclxuICAgICAgICAvLyBiMlZlYzIgckEgPSBwb2ludCAtIGNBO1xyXG4gICAgICAgIGIyVmVjMi5TdWJWVihwb2ludCwgY0EsIHJBKTtcclxuICAgICAgICAvLyBiMlZlYzIgckIgPSBwb2ludCAtIGNCO1xyXG4gICAgICAgIGIyVmVjMi5TdWJWVihwb2ludCwgY0IsIHJCKTtcclxuXHJcbiAgICAgICAgLy8gVHJhY2sgbWF4IGNvbnN0cmFpbnQgZXJyb3IuXHJcbiAgICAgICAgbWluU2VwYXJhdGlvbiA9IGIyTWluKG1pblNlcGFyYXRpb24sIHNlcGFyYXRpb24pO1xyXG5cclxuICAgICAgICAvLyBQcmV2ZW50IGxhcmdlIGNvcnJlY3Rpb25zIGFuZCBhbGxvdyBzbG9wLlxyXG4gICAgICAgIGNvbnN0IEM6IG51bWJlciA9IGIyQ2xhbXAoXHJcbiAgICAgICAgICBiMl90b2lCYXVtZ2FydGUgKiAoc2VwYXJhdGlvbiArIGIyX2xpbmVhclNsb3ApLFxyXG4gICAgICAgICAgLWIyX21heExpbmVhckNvcnJlY3Rpb24sXHJcbiAgICAgICAgICAwLFxyXG4gICAgICAgICk7XHJcblxyXG4gICAgICAgIC8vIENvbXB1dGUgdGhlIGVmZmVjdGl2ZSBtYXNzLlxyXG4gICAgICAgIC8vIGZsb2F0MzIgcm5BID0gYjJDcm9zcyhyQSwgbm9ybWFsKTtcclxuICAgICAgICBjb25zdCBybkE6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKHJBLCBub3JtYWwpO1xyXG4gICAgICAgIC8vIGZsb2F0MzIgcm5CID0gYjJDcm9zcyhyQiwgbm9ybWFsKTtcclxuICAgICAgICBjb25zdCBybkI6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKHJCLCBub3JtYWwpO1xyXG4gICAgICAgIC8vIGZsb2F0MzIgSyA9IG1BICsgbUIgKyBpQSAqIHJuQSAqIHJuQSArIGlCICogcm5CICogcm5CO1xyXG4gICAgICAgIGNvbnN0IEs6IG51bWJlciA9IG1BICsgbUIgKyBpQSAqIHJuQSAqIHJuQSArIGlCICogcm5CICogcm5CO1xyXG5cclxuICAgICAgICAvLyBDb21wdXRlIG5vcm1hbCBpbXB1bHNlXHJcbiAgICAgICAgY29uc3QgaW1wdWxzZTogbnVtYmVyID0gSyA+IDAgPyAtQyAvIEsgOiAwO1xyXG5cclxuICAgICAgICAvLyBiMlZlYzIgUCA9IGltcHVsc2UgKiBub3JtYWw7XHJcbiAgICAgICAgYjJWZWMyLk11bFNWKGltcHVsc2UsIG5vcm1hbCwgUCk7XHJcblxyXG4gICAgICAgIC8vIGNBIC09IG1BICogUDtcclxuICAgICAgICBjQS5TZWxmTXVsU3ViKG1BLCBQKTtcclxuICAgICAgICAvLyBhQSAtPSBpQSAqIGIyQ3Jvc3MockEsIFApO1xyXG4gICAgICAgIGFBIC09IGlBICogYjJWZWMyLkNyb3NzVlYockEsIFApO1xyXG5cclxuICAgICAgICAvLyBjQiArPSBtQiAqIFA7XHJcbiAgICAgICAgY0IuU2VsZk11bEFkZChtQiwgUCk7XHJcbiAgICAgICAgLy8gYUIgKz0gaUIgKiBiMkNyb3NzKHJCLCBQKTtcclxuICAgICAgICBhQiArPSBpQiAqIGIyVmVjMi5Dcm9zc1ZWKHJCLCBQKTtcclxuICAgICAgfVxyXG5cclxuICAgICAgLy8gdGhpcy5tX3Bvc2l0aW9uc1tpbmRleEFdLmMgPSBjQTtcclxuICAgICAgdGhpcy5tX3Bvc2l0aW9uc1tpbmRleEFdLmEgPSBhQTtcclxuXHJcbiAgICAgIC8vIHRoaXMubV9wb3NpdGlvbnNbaW5kZXhCXS5jID0gY0I7XHJcbiAgICAgIHRoaXMubV9wb3NpdGlvbnNbaW5kZXhCXS5hID0gYUI7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gV2UgY2FuJ3QgZXhwZWN0IG1pblNwZXBhcmF0aW9uID49IC1iMl9saW5lYXJTbG9wIGJlY2F1c2Ugd2UgZG9uJ3RcclxuICAgIC8vIHB1c2ggdGhlIHNlcGFyYXRpb24gYWJvdmUgLWIyX2xpbmVhclNsb3AuXHJcbiAgICByZXR1cm4gbWluU2VwYXJhdGlvbiA+PSAtMS41ICogYjJfbGluZWFyU2xvcDtcclxuICB9XHJcbn1cclxuIl19