/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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
import { b2Maybe } from '../../common/b2Settings';
import { b2Clamp, b2Mat22, b2Rot, b2Vec2 } from '../../common/b2Math';
import { b2Joint, b2JointDef } from './b2Joint';
/// Friction joint definition.
export class b2FrictionJointDef extends b2JointDef {
    constructor() {
        super(9 /* e_frictionJoint */);
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
        this.maxForce = 0;
        this.maxTorque = 0;
    }
    Initialize(bA, bB, anchor) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
    }
}
export class b2FrictionJoint extends b2Joint {
    constructor(def) {
        super(def);
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        // Solver shared
        this.m_linearImpulse = new b2Vec2();
        this.m_angularImpulse = 0;
        this.m_maxForce = 0;
        this.m_maxTorque = 0;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_linearMass = new b2Mat22();
        this.m_angularMass = 0;
        this.m_qA = new b2Rot();
        this.m_qB = new b2Rot();
        this.m_lalcA = new b2Vec2();
        this.m_lalcB = new b2Vec2();
        this.m_K = new b2Mat22();
        this.m_localAnchorA.Copy(def.localAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB);
        this.m_linearImpulse.SetZero();
        this.m_maxForce = b2Maybe(def.maxForce, 0);
        this.m_maxTorque = b2Maybe(def.maxTorque, 0);
        this.m_linearMass.SetZero();
    }
    InitVelocityConstraints(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;
        // const cA: b2Vec2 = data.positions[this.m_indexA].c;
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        // const cB: b2Vec2 = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        // const qA: b2Rot = new b2Rot(aA), qB: b2Rot = new b2Rot(aB);
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // Compute the effective mass matrix.
        // m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]
        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        const K = this.m_K; // new b2Mat22();
        K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
        K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
        K.GetInverse(this.m_linearMass);
        this.m_angularMass = iA + iB;
        if (this.m_angularMass > 0) {
            this.m_angularMass = 1 / this.m_angularMass;
        }
        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            // m_linearImpulse *= data.step.dtRatio;
            this.m_linearImpulse.SelfMul(data.step.dtRatio);
            this.m_angularImpulse *= data.step.dtRatio;
            // const P: b2Vec2(m_linearImpulse.x, m_linearImpulse.y);
            const P = this.m_linearImpulse;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            // wA -= iA * (b2Cross(m_rA, P) + m_angularImpulse);
            wA -= iA * (b2Vec2.CrossVV(this.m_rA, P) + this.m_angularImpulse);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            // wB += iB * (b2Cross(m_rB, P) + m_angularImpulse);
            wB += iB * (b2Vec2.CrossVV(this.m_rB, P) + this.m_angularImpulse);
        }
        else {
            this.m_linearImpulse.SetZero();
            this.m_angularImpulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolveVelocityConstraints(data) {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        const h = data.step.dt;
        // Solve angular friction
        {
            const Cdot = wB - wA;
            let impulse = -this.m_angularMass * Cdot;
            const oldImpulse = this.m_angularImpulse;
            const maxImpulse = h * this.m_maxTorque;
            this.m_angularImpulse = b2Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_angularImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        // Solve linear friction
        {
            // b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
            const Cdot_v2 = b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2Vec2.s_t1), b2FrictionJoint.SolveVelocityConstraints_s_Cdot_v2);
            // b2Vec2 impulse = -b2Mul(m_linearMass, Cdot);
            const impulseV = b2Mat22
                .MulMV(this.m_linearMass, Cdot_v2, b2FrictionJoint.SolveVelocityConstraints_s_impulseV)
                .SelfNeg();
            // b2Vec2 oldImpulse = m_linearImpulse;
            const oldImpulseV = b2FrictionJoint.SolveVelocityConstraints_s_oldImpulseV.Copy(this.m_linearImpulse);
            // m_linearImpulse += impulse;
            this.m_linearImpulse.SelfAdd(impulseV);
            const maxImpulse = h * this.m_maxForce;
            if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
                this.m_linearImpulse.Normalize();
                this.m_linearImpulse.SelfMul(maxImpulse);
            }
            // impulse = m_linearImpulse - oldImpulse;
            b2Vec2.SubVV(this.m_linearImpulse, oldImpulseV, impulseV);
            // vA -= mA * impulse;
            vA.SelfMulSub(mA, impulseV);
            // wA -= iA * b2Cross(m_rA, impulse);
            wA -= iA * b2Vec2.CrossVV(this.m_rA, impulseV);
            // vB += mB * impulse;
            vB.SelfMulAdd(mB, impulseV);
            // wB += iB * b2Cross(m_rB, impulse);
            wB += iB * b2Vec2.CrossVV(this.m_rB, impulseV);
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolvePositionConstraints(data) {
        return true;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        out.x = inv_dt * this.m_linearImpulse.x;
        out.y = inv_dt * this.m_linearImpulse.y;
        return out;
    }
    GetReactionTorque(inv_dt) {
        return inv_dt * this.m_angularImpulse;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    SetMaxForce(force) {
        this.m_maxForce = force;
    }
    GetMaxForce() {
        return this.m_maxForce;
    }
    SetMaxTorque(torque) {
        this.m_maxTorque = torque;
    }
    GetMaxTorque() {
        return this.m_maxTorque;
    }
}
b2FrictionJoint.SolveVelocityConstraints_s_Cdot_v2 = new b2Vec2();
b2FrictionJoint.SolveVelocityConstraints_s_impulseV = new b2Vec2();
b2FrictionJoint.SolveVelocityConstraints_s_oldImpulseV = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJGcmljdGlvbkpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vc3JjL2R5bmFtaWNzL2pvaW50cy9iMkZyaWN0aW9uSm9pbnQudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7QUFFSCxPQUFPLEVBQUUsT0FBTyxFQUFFLE1BQU0seUJBQXlCLENBQUM7QUFDbEQsT0FBTyxFQUFFLE9BQU8sRUFBRSxPQUFPLEVBQUUsS0FBSyxFQUFFLE1BQU0sRUFBTSxNQUFNLHFCQUFxQixDQUFDO0FBQzFFLE9BQU8sRUFBZSxPQUFPLEVBQUUsVUFBVSxFQUFlLE1BQU0sV0FBVyxDQUFDO0FBYzFFLDhCQUE4QjtBQUM5QixNQUFNLE9BQU8sa0JBQW1CLFNBQVEsVUFBVTtJQVNoRDtRQUNFLEtBQUsseUJBQTZCLENBQUM7UUFUNUIsaUJBQVksR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBRXBDLGlCQUFZLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUU3QyxhQUFRLEdBQUcsQ0FBQyxDQUFDO1FBRWIsY0FBUyxHQUFHLENBQUMsQ0FBQztJQUlkLENBQUM7SUFFRCxVQUFVLENBQUMsRUFBVSxFQUFFLEVBQVUsRUFBRSxNQUFjO1FBQy9DLElBQUksQ0FBQyxLQUFLLEdBQUcsRUFBRSxDQUFDO1FBQ2hCLElBQUksQ0FBQyxLQUFLLEdBQUcsRUFBRSxDQUFDO1FBQ2hCLElBQUksQ0FBQyxLQUFLLENBQUMsYUFBYSxDQUFDLE1BQU0sRUFBRSxJQUFJLENBQUMsWUFBWSxDQUFDLENBQUM7UUFDcEQsSUFBSSxDQUFDLEtBQUssQ0FBQyxhQUFhLENBQUMsTUFBTSxFQUFFLElBQUksQ0FBQyxZQUFZLENBQUMsQ0FBQztJQUN0RCxDQUFDO0NBQ0Y7QUFFRCxNQUFNLE9BQU8sZUFBZ0IsU0FBUSxPQUFPO0lBOEIxQyxZQUFZLEdBQXdCO1FBQ2xDLEtBQUssQ0FBQyxHQUFHLENBQUMsQ0FBQztRQTlC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LE1BQU0sR0FBRyxJQUFJLENBQUMsZUFBZSxDQUFDLENBQUMsQ0FBQztRQUN4QyxHQUFHLENBQUMsQ0FBQyxHQUFHLE1BQU0sR0FBRyxJQUFJLENBQUMsZUFBZSxDQUFDLENBQUMsQ0FBQztRQUN4QyxPQUFPLEdBQUcsQ0FBQztJQUNiLENBQUM7SUFFRCxpQkFBaUIsQ0FBQyxNQUFjO1FBQzlCLE9BQU8sTUFBTSxHQUFHLElBQUksQ0FBQyxnQkFBZ0IsQ0FBQztJQUN4QyxDQUFDO0lBRUQsZUFBZTtRQUNiLE9BQU8sSUFBSSxDQUFDLGNBQWMsQ0FBQztJQUM3QixDQUFDO0lBRUQsZUFBZTtRQUNiLE9BQU8sSUFBSSxDQUFDLGNBQWMsQ0FBQztJQUM3QixDQUFDO0lBRUQsV0FBVyxDQUFDLEtBQWE7UUFDdkIsSUFBSSxDQUFDLFVBQVUsR0FBRyxLQUFLLENBQUM7SUFDMUIsQ0FBQztJQUVELFdBQVc7UUFDVCxPQUFPLElBQUksQ0FBQyxVQUFVLENBQUM7SUFDekIsQ0FBQztJQUVELFlBQVksQ0FBQyxNQUFjO1FBQ3pCLElBQUksQ0FBQyxXQUFXLEdBQUcsTUFBTSxDQUFDO0lBQzVCLENBQUM7SUFFRCxZQUFZO1FBQ1YsT0FBTyxJQUFJLENBQUMsV0FBVyxDQUFDO0lBQzFCLENBQUM7O0FBMUhjLGtEQUFrQyxHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDbEQsbURBQW1DLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUNuRCxzREFBc0MsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDIiwic291cmNlc0NvbnRlbnQiOlsiLypcclxuICogQ29weXJpZ2h0IChjKSAyMDA2LTIwMDcgRXJpbiBDYXR0byBodHRwOi8vd3d3LmJveDJkLm9yZ1xyXG4gKlxyXG4gKiBUaGlzIHNvZnR3YXJlIGlzIHByb3ZpZGVkICdhcy1pcycsIHdpdGhvdXQgYW55IGV4cHJlc3Mgb3IgaW1wbGllZFxyXG4gKiB3YXJyYW50eS4gIEluIG5vIGV2ZW50IHdpbGwgdGhlIGF1dGhvcnMgYmUgaGVsZCBsaWFibGUgZm9yIGFueSBkYW1hZ2VzXHJcbiAqIGFyaXNpbmcgZnJvbSB0aGUgdXNlIG9mIHRoaXMgc29mdHdhcmUuXHJcbiAqIFBlcm1pc3Npb24gaXMgZ3JhbnRlZCB0byBhbnlvbmUgdG8gdXNlIHRoaXMgc29mdHdhcmUgZm9yIGFueSBwdXJwb3NlLFxyXG4gKiBpbmNsdWRpbmcgY29tbWVyY2lhbCBhcHBsaWNhdGlvbnMsIGFuZCB0byBhbHRlciBpdCBhbmQgcmVkaXN0cmlidXRlIGl0XHJcbiAqIGZyZWVseSwgc3ViamVjdCB0byB0aGUgZm9sbG93aW5nIHJlc3RyaWN0aW9uczpcclxuICogMS4gVGhlIG9yaWdpbiBvZiB0aGlzIHNvZnR3YXJlIG11c3Qgbm90IGJlIG1pc3JlcHJlc2VudGVkOyB5b3UgbXVzdCBub3RcclxuICogY2xhaW0gdGhhdCB5b3Ugd3JvdGUgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLiBJZiB5b3UgdXNlIHRoaXMgc29mdHdhcmVcclxuICogaW4gYSBwcm9kdWN0LCBhbiBhY2tub3dsZWRnbWVudCBpbiB0aGUgcHJvZHVjdCBkb2N1bWVudGF0aW9uIHdvdWxkIGJlXHJcbiAqIGFwcHJlY2lhdGVkIGJ1dCBpcyBub3QgcmVxdWlyZWQuXHJcbiAqIDIuIEFsdGVyZWQgc291cmNlIHZlcnNpb25zIG11c3QgYmUgcGxhaW5seSBtYXJrZWQgYXMgc3VjaCwgYW5kIG11c3Qgbm90IGJlXHJcbiAqIG1pc3JlcHJlc2VudGVkIGFzIGJlaW5nIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS5cclxuICogMy4gVGhpcyBub3RpY2UgbWF5IG5vdCBiZSByZW1vdmVkIG9yIGFsdGVyZWQgZnJvbSBhbnkgc291cmNlIGRpc3RyaWJ1dGlvbi5cclxuICovXHJcblxyXG5pbXBvcnQgeyBiMk1heWJlIH0gZnJvbSAnLi4vLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQgeyBiMkNsYW1wLCBiMk1hdDIyLCBiMlJvdCwgYjJWZWMyLCBYWSB9IGZyb20gJy4uLy4uL2NvbW1vbi9iMk1hdGgnO1xyXG5pbXBvcnQgeyBiMklKb2ludERlZiwgYjJKb2ludCwgYjJKb2ludERlZiwgYjJKb2ludFR5cGUgfSBmcm9tICcuL2IySm9pbnQnO1xyXG5pbXBvcnQgeyBiMlNvbHZlckRhdGEgfSBmcm9tICcuLi9iMlRpbWVTdGVwJztcclxuaW1wb3J0IHsgYjJCb2R5IH0gZnJvbSAnLi4vYjJCb2R5JztcclxuXHJcbmV4cG9ydCBpbnRlcmZhY2UgYjJJRnJpY3Rpb25Kb2ludERlZiBleHRlbmRzIGIySUpvaW50RGVmIHtcclxuICBsb2NhbEFuY2hvckE6IFhZO1xyXG5cclxuICBsb2NhbEFuY2hvckI6IFhZO1xyXG5cclxuICBtYXhGb3JjZT86IG51bWJlcjtcclxuXHJcbiAgbWF4VG9ycXVlPzogbnVtYmVyO1xyXG59XHJcblxyXG4vLy8gRnJpY3Rpb24gam9pbnQgZGVmaW5pdGlvbi5cclxuZXhwb3J0IGNsYXNzIGIyRnJpY3Rpb25Kb2ludERlZiBleHRlbmRzIGIySm9pbnREZWYgaW1wbGVtZW50cyBiMklGcmljdGlvbkpvaW50RGVmIHtcclxuICByZWFkb25seSBsb2NhbEFuY2hvckE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgcmVhZG9ubHkgbG9jYWxBbmNob3JCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIG1heEZvcmNlID0gMDtcclxuXHJcbiAgbWF4VG9ycXVlID0gMDtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICBzdXBlcihiMkpvaW50VHlwZS5lX2ZyaWN0aW9uSm9pbnQpO1xyXG4gIH1cclxuXHJcbiAgSW5pdGlhbGl6ZShiQTogYjJCb2R5LCBiQjogYjJCb2R5LCBhbmNob3I6IGIyVmVjMik6IHZvaWQge1xyXG4gICAgdGhpcy5ib2R5QSA9IGJBO1xyXG4gICAgdGhpcy5ib2R5QiA9IGJCO1xyXG4gICAgdGhpcy5ib2R5QS5HZXRMb2NhbFBvaW50KGFuY2hvciwgdGhpcy5sb2NhbEFuY2hvckEpO1xyXG4gICAgdGhpcy5ib2R5Qi5HZXRMb2NhbFBvaW50KGFuY2hvciwgdGhpcy5sb2NhbEFuY2hvckIpO1xyXG4gIH1cclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIyRnJpY3Rpb25Kb2ludCBleHRlbmRzIGIySm9pbnQge1xyXG4gIHJlYWRvbmx5IG1fbG9jYWxBbmNob3JBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbEFuY2hvckI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgLy8gU29sdmVyIHNoYXJlZFxyXG4gIHJlYWRvbmx5IG1fbGluZWFySW1wdWxzZTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIG1fYW5ndWxhckltcHVsc2UgPSAwO1xyXG4gIG1fbWF4Rm9yY2UgPSAwO1xyXG4gIG1fbWF4VG9ycXVlID0gMDtcclxuXHJcbiAgLy8gU29sdmVyIHRlbXBcclxuICBtX2luZGV4QSA9IDA7XHJcbiAgbV9pbmRleEIgPSAwO1xyXG4gIHJlYWRvbmx5IG1fckE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX3JCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbENlbnRlckE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xvY2FsQ2VudGVyQjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIG1faW52TWFzc0EgPSAwO1xyXG4gIG1faW52TWFzc0IgPSAwO1xyXG4gIG1faW52SUEgPSAwO1xyXG4gIG1faW52SUIgPSAwO1xyXG4gIHJlYWRvbmx5IG1fbGluZWFyTWFzczogYjJNYXQyMiA9IG5ldyBiMk1hdDIyKCk7XHJcbiAgbV9hbmd1bGFyTWFzcyA9IDA7XHJcblxyXG4gIHJlYWRvbmx5IG1fcUE6IGIyUm90ID0gbmV3IGIyUm90KCk7XHJcbiAgcmVhZG9ubHkgbV9xQjogYjJSb3QgPSBuZXcgYjJSb3QoKTtcclxuICByZWFkb25seSBtX2xhbGNBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sYWxjQjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fSzogYjJNYXQyMiA9IG5ldyBiMk1hdDIyKCk7XHJcblxyXG4gIGNvbnN0cnVjdG9yKGRlZjogYjJJRnJpY3Rpb25Kb2ludERlZikge1xyXG4gICAgc3VwZXIoZGVmKTtcclxuXHJcbiAgICB0aGlzLm1fbG9jYWxBbmNob3JBLkNvcHkoZGVmLmxvY2FsQW5jaG9yQSk7XHJcbiAgICB0aGlzLm1fbG9jYWxBbmNob3JCLkNvcHkoZGVmLmxvY2FsQW5jaG9yQik7XHJcblxyXG4gICAgdGhpcy5tX2xpbmVhckltcHVsc2UuU2V0WmVybygpO1xyXG4gICAgdGhpcy5tX21heEZvcmNlID0gYjJNYXliZShkZWYubWF4Rm9yY2UsIDApO1xyXG4gICAgdGhpcy5tX21heFRvcnF1ZSA9IGIyTWF5YmUoZGVmLm1heFRvcnF1ZSwgMCk7XHJcblxyXG4gICAgdGhpcy5tX2xpbmVhck1hc3MuU2V0WmVybygpO1xyXG4gIH1cclxuXHJcbiAgSW5pdFZlbG9jaXR5Q29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogdm9pZCB7XHJcbiAgICB0aGlzLm1faW5kZXhBID0gdGhpcy5tX2JvZHlBLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1faW5kZXhCID0gdGhpcy5tX2JvZHlCLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1fbG9jYWxDZW50ZXJBLkNvcHkodGhpcy5tX2JvZHlBLm1fc3dlZXAubG9jYWxDZW50ZXIpO1xyXG4gICAgdGhpcy5tX2xvY2FsQ2VudGVyQi5Db3B5KHRoaXMubV9ib2R5Qi5tX3N3ZWVwLmxvY2FsQ2VudGVyKTtcclxuICAgIHRoaXMubV9pbnZNYXNzQSA9IHRoaXMubV9ib2R5QS5tX2ludk1hc3M7XHJcbiAgICB0aGlzLm1faW52TWFzc0IgPSB0aGlzLm1fYm9keUIubV9pbnZNYXNzO1xyXG4gICAgdGhpcy5tX2ludklBID0gdGhpcy5tX2JvZHlBLm1faW52STtcclxuICAgIHRoaXMubV9pbnZJQiA9IHRoaXMubV9ib2R5Qi5tX2ludkk7XHJcblxyXG4gICAgLy8gY29uc3QgY0E6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmM7XHJcbiAgICBjb25zdCBhQTogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYTtcclxuICAgIGNvbnN0IHZBOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udjtcclxuICAgIGxldCB3QTogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnc7XHJcblxyXG4gICAgLy8gY29uc3QgY0I6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmM7XHJcbiAgICBjb25zdCBhQjogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Ql0uYTtcclxuICAgIGNvbnN0IHZCOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udjtcclxuICAgIGxldCB3QjogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnc7XHJcblxyXG4gICAgLy8gY29uc3QgcUE6IGIyUm90ID0gbmV3IGIyUm90KGFBKSwgcUI6IGIyUm90ID0gbmV3IGIyUm90KGFCKTtcclxuICAgIGNvbnN0IHFBOiBiMlJvdCA9IHRoaXMubV9xQS5TZXRBbmdsZShhQSksXHJcbiAgICAgIHFCOiBiMlJvdCA9IHRoaXMubV9xQi5TZXRBbmdsZShhQik7XHJcblxyXG4gICAgLy8gQ29tcHV0ZSB0aGUgZWZmZWN0aXZlIG1hc3MgbWF0cml4LlxyXG4gICAgLy8gbV9yQSA9IGIyTXVsKHFBLCBtX2xvY2FsQW5jaG9yQSAtIG1fbG9jYWxDZW50ZXJBKTtcclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JBLCB0aGlzLm1fbG9jYWxDZW50ZXJBLCB0aGlzLm1fbGFsY0EpO1xyXG4gICAgY29uc3QgckE6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFBLCB0aGlzLm1fbGFsY0EsIHRoaXMubV9yQSk7XHJcbiAgICAvLyBtX3JCID0gYjJNdWwocUIsIG1fbG9jYWxBbmNob3JCIC0gbV9sb2NhbENlbnRlckIpO1xyXG4gICAgYjJWZWMyLlN1YlZWKHRoaXMubV9sb2NhbEFuY2hvckIsIHRoaXMubV9sb2NhbENlbnRlckIsIHRoaXMubV9sYWxjQik7XHJcbiAgICBjb25zdCByQjogYjJWZWMyID0gYjJSb3QuTXVsUlYocUIsIHRoaXMubV9sYWxjQiwgdGhpcy5tX3JCKTtcclxuXHJcbiAgICAvLyBKID0gWy1JIC1yMV9za2V3IEkgcjJfc2tld11cclxuICAgIC8vICAgICBbIDAgICAgICAgLTEgMCAgICAgICAxXVxyXG4gICAgLy8gcl9za2V3ID0gWy1yeTsgcnhdXHJcblxyXG4gICAgLy8gTWF0bGFiXHJcbiAgICAvLyBLID0gWyBtQStyMXleMippQSttQityMnleMippQiwgIC1yMXkqaUEqcjF4LXIyeSppQipyMngsICAgICAgICAgIC1yMXkqaUEtcjJ5KmlCXVxyXG4gICAgLy8gICAgIFsgIC1yMXkqaUEqcjF4LXIyeSppQipyMngsIG1BK3IxeF4yKmlBK21CK3IyeF4yKmlCLCAgICAgICAgICAgcjF4KmlBK3IyeCppQl1cclxuICAgIC8vICAgICBbICAgICAgICAgIC1yMXkqaUEtcjJ5KmlCLCAgICAgICAgICAgcjF4KmlBK3IyeCppQiwgICAgICAgICAgICAgICAgICAgaUEraUJdXHJcblxyXG4gICAgY29uc3QgbUE6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQSxcclxuICAgICAgbUI6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQjtcclxuICAgIGNvbnN0IGlBOiBudW1iZXIgPSB0aGlzLm1faW52SUEsXHJcbiAgICAgIGlCOiBudW1iZXIgPSB0aGlzLm1faW52SUI7XHJcblxyXG4gICAgY29uc3QgSzogYjJNYXQyMiA9IHRoaXMubV9LOyAvLyBuZXcgYjJNYXQyMigpO1xyXG4gICAgSy5leC54ID0gbUEgKyBtQiArIGlBICogckEueSAqIHJBLnkgKyBpQiAqIHJCLnkgKiByQi55O1xyXG4gICAgSy5leC55ID0gLWlBICogckEueCAqIHJBLnkgLSBpQiAqIHJCLnggKiByQi55O1xyXG4gICAgSy5leS54ID0gSy5leC55O1xyXG4gICAgSy5leS55ID0gbUEgKyBtQiArIGlBICogckEueCAqIHJBLnggKyBpQiAqIHJCLnggKiByQi54O1xyXG5cclxuICAgIEsuR2V0SW52ZXJzZSh0aGlzLm1fbGluZWFyTWFzcyk7XHJcblxyXG4gICAgdGhpcy5tX2FuZ3VsYXJNYXNzID0gaUEgKyBpQjtcclxuICAgIGlmICh0aGlzLm1fYW5ndWxhck1hc3MgPiAwKSB7XHJcbiAgICAgIHRoaXMubV9hbmd1bGFyTWFzcyA9IDEgLyB0aGlzLm1fYW5ndWxhck1hc3M7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGRhdGEuc3RlcC53YXJtU3RhcnRpbmcpIHtcclxuICAgICAgLy8gU2NhbGUgaW1wdWxzZXMgdG8gc3VwcG9ydCBhIHZhcmlhYmxlIHRpbWUgc3RlcC5cclxuICAgICAgLy8gbV9saW5lYXJJbXB1bHNlICo9IGRhdGEuc3RlcC5kdFJhdGlvO1xyXG4gICAgICB0aGlzLm1fbGluZWFySW1wdWxzZS5TZWxmTXVsKGRhdGEuc3RlcC5kdFJhdGlvKTtcclxuICAgICAgdGhpcy5tX2FuZ3VsYXJJbXB1bHNlICo9IGRhdGEuc3RlcC5kdFJhdGlvO1xyXG5cclxuICAgICAgLy8gY29uc3QgUDogYjJWZWMyKG1fbGluZWFySW1wdWxzZS54LCBtX2xpbmVhckltcHVsc2UueSk7XHJcbiAgICAgIGNvbnN0IFA6IGIyVmVjMiA9IHRoaXMubV9saW5lYXJJbXB1bHNlO1xyXG5cclxuICAgICAgLy8gdkEgLT0gbUEgKiBQO1xyXG4gICAgICB2QS5TZWxmTXVsU3ViKG1BLCBQKTtcclxuICAgICAgLy8gd0EgLT0gaUEgKiAoYjJDcm9zcyhtX3JBLCBQKSArIG1fYW5ndWxhckltcHVsc2UpO1xyXG4gICAgICB3QSAtPSBpQSAqIChiMlZlYzIuQ3Jvc3NWVih0aGlzLm1fckEsIFApICsgdGhpcy5tX2FuZ3VsYXJJbXB1bHNlKTtcclxuICAgICAgLy8gdkIgKz0gbUIgKiBQO1xyXG4gICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQKTtcclxuICAgICAgLy8gd0IgKz0gaUIgKiAoYjJDcm9zcyhtX3JCLCBQKSArIG1fYW5ndWxhckltcHVsc2UpO1xyXG4gICAgICB3QiArPSBpQiAqIChiMlZlYzIuQ3Jvc3NWVih0aGlzLm1fckIsIFApICsgdGhpcy5tX2FuZ3VsYXJJbXB1bHNlKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV9saW5lYXJJbXB1bHNlLlNldFplcm8oKTtcclxuICAgICAgdGhpcy5tX2FuZ3VsYXJJbXB1bHNlID0gMDtcclxuICAgIH1cclxuXHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udiA9IHZBO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLncgPSB3QTtcclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52ID0gdkI7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udyA9IHdCO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfQ2RvdF92MiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19pbXB1bHNlViA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19vbGRJbXB1bHNlViA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzKGRhdGE6IGIyU29sdmVyRGF0YSk6IHZvaWQge1xyXG4gICAgY29uc3QgdkE6IGIyVmVjMiA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS52O1xyXG4gICAgbGV0IHdBOiBudW1iZXIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udztcclxuICAgIGNvbnN0IHZCOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udjtcclxuICAgIGxldCB3QjogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnc7XHJcblxyXG4gICAgY29uc3QgbUE6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQSxcclxuICAgICAgbUI6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQjtcclxuICAgIGNvbnN0IGlBOiBudW1iZXIgPSB0aGlzLm1faW52SUEsXHJcbiAgICAgIGlCOiBudW1iZXIgPSB0aGlzLm1faW52SUI7XHJcblxyXG4gICAgY29uc3QgaDogbnVtYmVyID0gZGF0YS5zdGVwLmR0O1xyXG5cclxuICAgIC8vIFNvbHZlIGFuZ3VsYXIgZnJpY3Rpb25cclxuICAgIHtcclxuICAgICAgY29uc3QgQ2RvdDogbnVtYmVyID0gd0IgLSB3QTtcclxuICAgICAgbGV0IGltcHVsc2U6IG51bWJlciA9IC10aGlzLm1fYW5ndWxhck1hc3MgKiBDZG90O1xyXG5cclxuICAgICAgY29uc3Qgb2xkSW1wdWxzZTogbnVtYmVyID0gdGhpcy5tX2FuZ3VsYXJJbXB1bHNlO1xyXG4gICAgICBjb25zdCBtYXhJbXB1bHNlOiBudW1iZXIgPSBoICogdGhpcy5tX21heFRvcnF1ZTtcclxuICAgICAgdGhpcy5tX2FuZ3VsYXJJbXB1bHNlID0gYjJDbGFtcCh0aGlzLm1fYW5ndWxhckltcHVsc2UgKyBpbXB1bHNlLCAtbWF4SW1wdWxzZSwgbWF4SW1wdWxzZSk7XHJcbiAgICAgIGltcHVsc2UgPSB0aGlzLm1fYW5ndWxhckltcHVsc2UgLSBvbGRJbXB1bHNlO1xyXG5cclxuICAgICAgd0EgLT0gaUEgKiBpbXB1bHNlO1xyXG4gICAgICB3QiArPSBpQiAqIGltcHVsc2U7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gU29sdmUgbGluZWFyIGZyaWN0aW9uXHJcbiAgICB7XHJcbiAgICAgIC8vIGIyVmVjMiBDZG90ID0gdkIgKyBiMkNyb3NzKHdCLCBtX3JCKSAtIHZBIC0gYjJDcm9zcyh3QSwgbV9yQSk7XHJcbiAgICAgIGNvbnN0IENkb3RfdjI6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVihcclxuICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkIsIHdCLCB0aGlzLm1fckIsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkEsIHdBLCB0aGlzLm1fckEsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICBiMkZyaWN0aW9uSm9pbnQuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfQ2RvdF92MixcclxuICAgICAgKTtcclxuXHJcbiAgICAgIC8vIGIyVmVjMiBpbXB1bHNlID0gLWIyTXVsKG1fbGluZWFyTWFzcywgQ2RvdCk7XHJcbiAgICAgIGNvbnN0IGltcHVsc2VWOiBiMlZlYzIgPSBiMk1hdDIyXHJcbiAgICAgICAgLk11bE1WKHRoaXMubV9saW5lYXJNYXNzLCBDZG90X3YyLCBiMkZyaWN0aW9uSm9pbnQuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfaW1wdWxzZVYpXHJcbiAgICAgICAgLlNlbGZOZWcoKTtcclxuICAgICAgLy8gYjJWZWMyIG9sZEltcHVsc2UgPSBtX2xpbmVhckltcHVsc2U7XHJcbiAgICAgIGNvbnN0IG9sZEltcHVsc2VWID0gYjJGcmljdGlvbkpvaW50LlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX29sZEltcHVsc2VWLkNvcHkoXHJcbiAgICAgICAgdGhpcy5tX2xpbmVhckltcHVsc2UsXHJcbiAgICAgICk7XHJcbiAgICAgIC8vIG1fbGluZWFySW1wdWxzZSArPSBpbXB1bHNlO1xyXG4gICAgICB0aGlzLm1fbGluZWFySW1wdWxzZS5TZWxmQWRkKGltcHVsc2VWKTtcclxuXHJcbiAgICAgIGNvbnN0IG1heEltcHVsc2U6IG51bWJlciA9IGggKiB0aGlzLm1fbWF4Rm9yY2U7XHJcblxyXG4gICAgICBpZiAodGhpcy5tX2xpbmVhckltcHVsc2UuTGVuZ3RoU3F1YXJlZCgpID4gbWF4SW1wdWxzZSAqIG1heEltcHVsc2UpIHtcclxuICAgICAgICB0aGlzLm1fbGluZWFySW1wdWxzZS5Ob3JtYWxpemUoKTtcclxuICAgICAgICB0aGlzLm1fbGluZWFySW1wdWxzZS5TZWxmTXVsKG1heEltcHVsc2UpO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBpbXB1bHNlID0gbV9saW5lYXJJbXB1bHNlIC0gb2xkSW1wdWxzZTtcclxuICAgICAgYjJWZWMyLlN1YlZWKHRoaXMubV9saW5lYXJJbXB1bHNlLCBvbGRJbXB1bHNlViwgaW1wdWxzZVYpO1xyXG5cclxuICAgICAgLy8gdkEgLT0gbUEgKiBpbXB1bHNlO1xyXG4gICAgICB2QS5TZWxmTXVsU3ViKG1BLCBpbXB1bHNlVik7XHJcbiAgICAgIC8vIHdBIC09IGlBICogYjJDcm9zcyhtX3JBLCBpbXB1bHNlKTtcclxuICAgICAgd0EgLT0gaUEgKiBiMlZlYzIuQ3Jvc3NWVih0aGlzLm1fckEsIGltcHVsc2VWKTtcclxuXHJcbiAgICAgIC8vIHZCICs9IG1CICogaW1wdWxzZTtcclxuICAgICAgdkIuU2VsZk11bEFkZChtQiwgaW1wdWxzZVYpO1xyXG4gICAgICAvLyB3QiArPSBpQiAqIGIyQ3Jvc3MobV9yQiwgaW1wdWxzZSk7XHJcbiAgICAgIHdCICs9IGlCICogYjJWZWMyLkNyb3NzVlYodGhpcy5tX3JCLCBpbXB1bHNlVik7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnYgPSB2QTtcclxuICAgIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS53ID0gd0E7XHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udiA9IHZCO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLncgPSB3QjtcclxuICB9XHJcblxyXG4gIFNvbHZlUG9zaXRpb25Db25zdHJhaW50cyhkYXRhOiBiMlNvbHZlckRhdGEpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH1cclxuXHJcbiAgR2V0QW5jaG9yQTxUIGV4dGVuZHMgWFk+KG91dDogVCk6IFQge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ib2R5QS5HZXRXb3JsZFBvaW50KHRoaXMubV9sb2NhbEFuY2hvckEsIG91dCk7XHJcbiAgfVxyXG5cclxuICBHZXRBbmNob3JCPFQgZXh0ZW5kcyBYWT4ob3V0OiBUKTogVCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2JvZHlCLkdldFdvcmxkUG9pbnQodGhpcy5tX2xvY2FsQW5jaG9yQiwgb3V0KTtcclxuICB9XHJcblxyXG4gIEdldFJlYWN0aW9uRm9yY2U8VCBleHRlbmRzIFhZPihpbnZfZHQ6IG51bWJlciwgb3V0OiBUKTogVCB7XHJcbiAgICBvdXQueCA9IGludl9kdCAqIHRoaXMubV9saW5lYXJJbXB1bHNlLng7XHJcbiAgICBvdXQueSA9IGludl9kdCAqIHRoaXMubV9saW5lYXJJbXB1bHNlLnk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgR2V0UmVhY3Rpb25Ub3JxdWUoaW52X2R0OiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIGludl9kdCAqIHRoaXMubV9hbmd1bGFySW1wdWxzZTtcclxuICB9XHJcblxyXG4gIEdldExvY2FsQW5jaG9yQSgpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHJldHVybiB0aGlzLm1fbG9jYWxBbmNob3JBO1xyXG4gIH1cclxuXHJcbiAgR2V0TG9jYWxBbmNob3JCKCk6IFJlYWRvbmx5PGIyVmVjMj4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9sb2NhbEFuY2hvckI7XHJcbiAgfVxyXG5cclxuICBTZXRNYXhGb3JjZShmb3JjZTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fbWF4Rm9yY2UgPSBmb3JjZTtcclxuICB9XHJcblxyXG4gIEdldE1heEZvcmNlKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX21heEZvcmNlO1xyXG4gIH1cclxuXHJcbiAgU2V0TWF4VG9ycXVlKHRvcnF1ZTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fbWF4VG9ycXVlID0gdG9ycXVlO1xyXG4gIH1cclxuXHJcbiAgR2V0TWF4VG9ycXVlKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX21heFRvcnF1ZTtcclxuICB9XHJcbn1cclxuIl19