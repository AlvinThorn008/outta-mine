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
import { b2_linearSlop, b2_maxLinearCorrection, b2Maybe } from '../../common/b2Settings';
import { b2Clamp, b2Min, b2Rot, b2Vec2 } from '../../common/b2Math';
import { b2Joint, b2JointDef } from './b2Joint';
/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in b2JointDef.
export class b2RopeJointDef extends b2JointDef {
    constructor() {
        super(10 /* e_ropeJoint */);
        this.localAnchorA = new b2Vec2(-1, 0);
        this.localAnchorB = new b2Vec2(1, 0);
        this.maxLength = 0;
    }
}
export class b2RopeJoint extends b2Joint {
    constructor(def) {
        super(def);
        // Solver shared
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_maxLength = 0;
        this.m_length = 0;
        this.m_impulse = 0;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_u = new b2Vec2();
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_mass = 0;
        this.m_state = 0 /* e_inactiveLimit */;
        this.m_qA = new b2Rot();
        this.m_qB = new b2Rot();
        this.m_lalcA = new b2Vec2();
        this.m_lalcB = new b2Vec2();
        this.m_localAnchorA.Copy(b2Maybe(def.localAnchorA, new b2Vec2(-1, 0)));
        this.m_localAnchorB.Copy(b2Maybe(def.localAnchorB, new b2Vec2(1, 0)));
        this.m_maxLength = b2Maybe(def.maxLength, 0);
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
        const cA = data.positions[this.m_indexA].c;
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const cB = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // this.m_rA = b2Mul(qA, this.m_localAnchorA - this.m_localCenterA);
        b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // this.m_rB = b2Mul(qB, this.m_localAnchorB - this.m_localCenterB);
        b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // this.m_u = cB + this.m_rB - cA - this.m_rA;
        this.m_u.Copy(cB).SelfAdd(this.m_rB).SelfSub(cA).SelfSub(this.m_rA);
        this.m_length = this.m_u.Length();
        const C = this.m_length - this.m_maxLength;
        if (C > 0) {
            this.m_state = 2 /* e_atUpperLimit */;
        }
        else {
            this.m_state = 0 /* e_inactiveLimit */;
        }
        if (this.m_length > b2_linearSlop) {
            this.m_u.SelfMul(1 / this.m_length);
        }
        else {
            this.m_u.SetZero();
            this.m_mass = 0;
            this.m_impulse = 0;
            return;
        }
        // Compute effective mass.
        const crA = b2Vec2.CrossVV(this.m_rA, this.m_u);
        const crB = b2Vec2.CrossVV(this.m_rB, this.m_u);
        const invMass = this.m_invMassA + this.m_invIA * crA * crA + this.m_invMassB + this.m_invIB * crB * crB;
        this.m_mass = invMass !== 0 ? 1 / invMass : 0;
        if (data.step.warmStarting) {
            // Scale the impulse to support a variable time step.
            this.m_impulse *= data.step.dtRatio;
            // b2Vec2 P = m_impulse * m_u;
            const P = b2Vec2.MulSV(this.m_impulse, this.m_u, b2RopeJoint.InitVelocityConstraints_s_P);
            // vA -= m_invMassA * P;
            vA.SelfMulSub(this.m_invMassA, P);
            wA -= this.m_invIA * b2Vec2.CrossVV(this.m_rA, P);
            // vB += m_invMassB * P;
            vB.SelfMulAdd(this.m_invMassB, P);
            wB += this.m_invIB * b2Vec2.CrossVV(this.m_rB, P);
        }
        else {
            this.m_impulse = 0;
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
        // Cdot = dot(u, v + cross(w, r))
        // b2Vec2 vpA = vA + b2Cross(wA, m_rA);
        const vpA = b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2RopeJoint.SolveVelocityConstraints_s_vpA);
        // b2Vec2 vpB = vB + b2Cross(wB, m_rB);
        const vpB = b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2RopeJoint.SolveVelocityConstraints_s_vpB);
        // float32 C = m_length - m_maxLength;
        const C = this.m_length - this.m_maxLength;
        // float32 Cdot = b2Dot(m_u, vpB - vpA);
        let Cdot = b2Vec2.DotVV(this.m_u, b2Vec2.SubVV(vpB, vpA, b2Vec2.s_t0));
        // Predictive constraint.
        if (C < 0) {
            Cdot += data.step.inv_dt * C;
        }
        let impulse = -this.m_mass * Cdot;
        const oldImpulse = this.m_impulse;
        this.m_impulse = b2Min(0, this.m_impulse + impulse);
        impulse = this.m_impulse - oldImpulse;
        // b2Vec2 P = impulse * m_u;
        const P = b2Vec2.MulSV(impulse, this.m_u, b2RopeJoint.SolveVelocityConstraints_s_P);
        // vA -= m_invMassA * P;
        vA.SelfMulSub(this.m_invMassA, P);
        wA -= this.m_invIA * b2Vec2.CrossVV(this.m_rA, P);
        // vB += m_invMassB * P;
        vB.SelfMulAdd(this.m_invMassB, P);
        wB += this.m_invIB * b2Vec2.CrossVV(this.m_rB, P);
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolvePositionConstraints(data) {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // b2Vec2 rA = b2Mul(qA, this.m_localAnchorA - this.m_localCenterA);
        b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, this.m_localAnchorB - this.m_localCenterB);
        b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 u = cB + rB - cA - rA;
        const u = this.m_u.Copy(cB).SelfAdd(rB).SelfSub(cA).SelfSub(rA);
        const length = u.Normalize();
        let C = length - this.m_maxLength;
        C = b2Clamp(C, 0, b2_maxLinearCorrection);
        const impulse = -this.m_mass * C;
        // b2Vec2 P = impulse * u;
        const P = b2Vec2.MulSV(impulse, u, b2RopeJoint.SolvePositionConstraints_s_P);
        // cA -= m_invMassA * P;
        cA.SelfMulSub(this.m_invMassA, P);
        aA -= this.m_invIA * b2Vec2.CrossVV(rA, P);
        // cB += m_invMassB * P;
        cB.SelfMulAdd(this.m_invMassB, P);
        aB += this.m_invIB * b2Vec2.CrossVV(rB, P);
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return length - this.m_maxLength < b2_linearSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // return out.Set(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
        return b2Vec2.MulSV(inv_dt * this.m_impulse, this.m_u, out);
    }
    GetReactionTorque(inv_dt) {
        return 0;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    SetMaxLength(length) {
        this.m_maxLength = length;
    }
    GetMaxLength() {
        return this.m_maxLength;
    }
    GetLimitState() {
        return this.m_state;
    }
}
b2RopeJoint.InitVelocityConstraints_s_P = new b2Vec2();
b2RopeJoint.SolveVelocityConstraints_s_vpA = new b2Vec2();
b2RopeJoint.SolveVelocityConstraints_s_vpB = new b2Vec2();
b2RopeJoint.SolveVelocityConstraints_s_P = new b2Vec2();
b2RopeJoint.SolvePositionConstraints_s_P = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJSb3BlSm9pbnQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9zcmMvZHluYW1pY3Mvam9pbnRzL2IyUm9wZUpvaW50LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsT0FBTyxFQUFFLGFBQWEsRUFBRSxzQkFBc0IsRUFBRSxPQUFPLEVBQUUsTUFBTSx5QkFBeUIsQ0FBQztBQUN6RixPQUFPLEVBQUUsT0FBTyxFQUFFLEtBQUssRUFBRSxLQUFLLEVBQUUsTUFBTSxFQUFNLE1BQU0scUJBQXFCLENBQUM7QUFDeEUsT0FBTyxFQUFlLE9BQU8sRUFBRSxVQUFVLEVBQTZCLE1BQU0sV0FBVyxDQUFDO0FBV3hGLG1FQUFtRTtBQUNuRSxzQkFBc0I7QUFDdEIsNERBQTREO0FBQzVELHVDQUF1QztBQUN2QyxNQUFNLE9BQU8sY0FBZSxTQUFRLFVBQVU7SUFPNUM7UUFDRSxLQUFLLHNCQUF5QixDQUFDO1FBUHhCLGlCQUFZLEdBQVcsSUFBSSxNQUFNLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFFekMsaUJBQVksR0FBVyxJQUFJLE1BQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFFakQsY0FBUyxHQUFHLENBQUMsQ0FBQztJQUlkLENBQUM7Q0FDRjtBQUVELE1BQU0sT0FBTyxXQUFZLFNBQVEsT0FBTztJQTRCdEMsWUFBWSxHQUFvQjtRQUM5QixLQUFLLENBQUMsR0FBRyxDQUFDLENBQUM7UUE1QmIsZ0JBQWdCO1FBQ1AsbUJBQWMsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQ3RDLG1CQUFjLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUMvQyxnQkFBVyxHQUFHLENBQUMsQ0FBQztRQUNoQixhQUFRLEdBQUcsQ0FBQyxDQUFDO1FBQ2IsY0FBUyxHQUFHLENBQUMsQ0FBQztRQUVkLGNBQWM7UUFDZCxhQUFRLEdBQUcsQ0FBQyxDQUFD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QUM7SUFDOUQsQ0FBQztJQUVELGdCQUFnQixDQUFlLE1BQWMsRUFBRSxHQUFNO1FBQ25ELG9GQUFvRjtRQUNwRixPQUFPLE1BQU0sQ0FBQyxLQUFLLENBQUMsTUFBTSxHQUFHLElBQUksQ0FBQyxTQUFTLEVBQUUsSUFBSSxDQUFDLEdBQUcsRUFBRSxHQUFHLENBQUMsQ0FBQztJQUM5RCxDQUFDO0lBRUQsaUJBQWlCLENBQUMsTUFBYztRQUM5QixPQUFPLENBQUMsQ0FBQztJQUNYLENBQUM7SUFFRCxlQUFlO1FBQ2IsT0FBTyxJQUFJLENBQUMsY0FBYyxDQUFDO0lBQzdCLENBQUM7SUFFRCxlQUFlO1FBQ2IsT0FBTyxJQUFJLENBQUMsY0FBYyxDQUFDO0lBQzdCLENBQUM7SUFFRCxZQUFZLENBQUMsTUFBYztRQUN6QixJQUFJLENBQUMsV0FBVyxHQUFHLE1BQU0sQ0FBQztJQUM1QixDQUFDO0lBRUQsWUFBWTtRQUNWLE9BQU8sSUFBSSxDQUFDLFdBQVcsQ0FBQztJQUMxQixDQUFDO0lBRUQsYUFBYTtRQUNYLE9BQU8sSUFBSSxDQUFDLE9BQU8sQ0FBQztJQUN0QixDQUFDOztBQTVOYyx1Q0FBMkIsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBc0YzQywwQ0FBOEIsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQzlDLDBDQUE4QixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDOUMsd0NBQTRCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQXFENUMsd0NBQTRCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDExIEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHsgYjJfbGluZWFyU2xvcCwgYjJfbWF4TGluZWFyQ29ycmVjdGlvbiwgYjJNYXliZSB9IGZyb20gJy4uLy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJDbGFtcCwgYjJNaW4sIGIyUm90LCBiMlZlYzIsIFhZIH0gZnJvbSAnLi4vLi4vY29tbW9uL2IyTWF0aCc7XHJcbmltcG9ydCB7IGIySUpvaW50RGVmLCBiMkpvaW50LCBiMkpvaW50RGVmLCBiMkpvaW50VHlwZSwgYjJMaW1pdFN0YXRlIH0gZnJvbSAnLi9iMkpvaW50JztcclxuaW1wb3J0IHsgYjJTb2x2ZXJEYXRhIH0gZnJvbSAnLi4vYjJUaW1lU3RlcCc7XHJcblxyXG5leHBvcnQgaW50ZXJmYWNlIGIySVJvcGVKb2ludERlZiBleHRlbmRzIGIySUpvaW50RGVmIHtcclxuICBsb2NhbEFuY2hvckE/OiBYWTtcclxuXHJcbiAgbG9jYWxBbmNob3JCPzogWFk7XHJcblxyXG4gIG1heExlbmd0aD86IG51bWJlcjtcclxufVxyXG5cclxuLy8vIFJvcGUgam9pbnQgZGVmaW5pdGlvbi4gVGhpcyByZXF1aXJlcyB0d28gYm9keSBhbmNob3IgcG9pbnRzIGFuZFxyXG4vLy8gYSBtYXhpbXVtIGxlbmd0aHMuXHJcbi8vLyBOb3RlOiBieSBkZWZhdWx0IHRoZSBjb25uZWN0ZWQgb2JqZWN0cyB3aWxsIG5vdCBjb2xsaWRlLlxyXG4vLy8gc2VlIGNvbGxpZGVDb25uZWN0ZWQgaW4gYjJKb2ludERlZi5cclxuZXhwb3J0IGNsYXNzIGIyUm9wZUpvaW50RGVmIGV4dGVuZHMgYjJKb2ludERlZiBpbXBsZW1lbnRzIGIySVJvcGVKb2ludERlZiB7XHJcbiAgcmVhZG9ubHkgbG9jYWxBbmNob3JBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKC0xLCAwKTtcclxuXHJcbiAgcmVhZG9ubHkgbG9jYWxBbmNob3JCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKDEsIDApO1xyXG5cclxuICBtYXhMZW5ndGggPSAwO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHN1cGVyKGIySm9pbnRUeXBlLmVfcm9wZUpvaW50KTtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlJvcGVKb2ludCBleHRlbmRzIGIySm9pbnQge1xyXG4gIC8vIFNvbHZlciBzaGFyZWRcclxuICByZWFkb25seSBtX2xvY2FsQW5jaG9yQTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fbG9jYWxBbmNob3JCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgbV9tYXhMZW5ndGggPSAwO1xyXG4gIG1fbGVuZ3RoID0gMDtcclxuICBtX2ltcHVsc2UgPSAwO1xyXG5cclxuICAvLyBTb2x2ZXIgdGVtcFxyXG4gIG1faW5kZXhBID0gMDtcclxuICBtX2luZGV4QiA9IDA7XHJcbiAgcmVhZG9ubHkgbV91OiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9yQTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fckI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xvY2FsQ2VudGVyQTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fbG9jYWxDZW50ZXJCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgbV9pbnZNYXNzQSA9IDA7XHJcbiAgbV9pbnZNYXNzQiA9IDA7XHJcbiAgbV9pbnZJQSA9IDA7XHJcbiAgbV9pbnZJQiA9IDA7XHJcbiAgbV9tYXNzID0gMDtcclxuICBtX3N0YXRlID0gYjJMaW1pdFN0YXRlLmVfaW5hY3RpdmVMaW1pdDtcclxuXHJcbiAgcmVhZG9ubHkgbV9xQTogYjJSb3QgPSBuZXcgYjJSb3QoKTtcclxuICByZWFkb25seSBtX3FCOiBiMlJvdCA9IG5ldyBiMlJvdCgpO1xyXG4gIHJlYWRvbmx5IG1fbGFsY0E6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xhbGNCOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIGNvbnN0cnVjdG9yKGRlZjogYjJJUm9wZUpvaW50RGVmKSB7XHJcbiAgICBzdXBlcihkZWYpO1xyXG5cclxuICAgIHRoaXMubV9sb2NhbEFuY2hvckEuQ29weShiMk1heWJlKGRlZi5sb2NhbEFuY2hvckEsIG5ldyBiMlZlYzIoLTEsIDApKSk7XHJcbiAgICB0aGlzLm1fbG9jYWxBbmNob3JCLkNvcHkoYjJNYXliZShkZWYubG9jYWxBbmNob3JCLCBuZXcgYjJWZWMyKDEsIDApKSk7XHJcbiAgICB0aGlzLm1fbWF4TGVuZ3RoID0gYjJNYXliZShkZWYubWF4TGVuZ3RoLCAwKTtcclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIEluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgSW5pdFZlbG9jaXR5Q29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogdm9pZCB7XHJcbiAgICB0aGlzLm1faW5kZXhBID0gdGhpcy5tX2JvZHlBLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1faW5kZXhCID0gdGhpcy5tX2JvZHlCLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1fbG9jYWxDZW50ZXJBLkNvcHkodGhpcy5tX2JvZHlBLm1fc3dlZXAubG9jYWxDZW50ZXIpO1xyXG4gICAgdGhpcy5tX2xvY2FsQ2VudGVyQi5Db3B5KHRoaXMubV9ib2R5Qi5tX3N3ZWVwLmxvY2FsQ2VudGVyKTtcclxuICAgIHRoaXMubV9pbnZNYXNzQSA9IHRoaXMubV9ib2R5QS5tX2ludk1hc3M7XHJcbiAgICB0aGlzLm1faW52TWFzc0IgPSB0aGlzLm1fYm9keUIubV9pbnZNYXNzO1xyXG4gICAgdGhpcy5tX2ludklBID0gdGhpcy5tX2JvZHlBLm1faW52STtcclxuICAgIHRoaXMubV9pbnZJQiA9IHRoaXMubV9ib2R5Qi5tX2ludkk7XHJcblxyXG4gICAgY29uc3QgY0E6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmM7XHJcbiAgICBjb25zdCBhQTogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYTtcclxuICAgIGNvbnN0IHZBOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udjtcclxuICAgIGxldCB3QTogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnc7XHJcblxyXG4gICAgY29uc3QgY0I6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmM7XHJcbiAgICBjb25zdCBhQjogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Ql0uYTtcclxuICAgIGNvbnN0IHZCOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udjtcclxuICAgIGxldCB3QjogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnc7XHJcblxyXG4gICAgY29uc3QgcUE6IGIyUm90ID0gdGhpcy5tX3FBLlNldEFuZ2xlKGFBKSxcclxuICAgICAgcUI6IGIyUm90ID0gdGhpcy5tX3FCLlNldEFuZ2xlKGFCKTtcclxuXHJcbiAgICAvLyB0aGlzLm1fckEgPSBiMk11bChxQSwgdGhpcy5tX2xvY2FsQW5jaG9yQSAtIHRoaXMubV9sb2NhbENlbnRlckEpO1xyXG4gICAgYjJWZWMyLlN1YlZWKHRoaXMubV9sb2NhbEFuY2hvckEsIHRoaXMubV9sb2NhbENlbnRlckEsIHRoaXMubV9sYWxjQSk7XHJcbiAgICBiMlJvdC5NdWxSVihxQSwgdGhpcy5tX2xhbGNBLCB0aGlzLm1fckEpO1xyXG4gICAgLy8gdGhpcy5tX3JCID0gYjJNdWwocUIsIHRoaXMubV9sb2NhbEFuY2hvckIgLSB0aGlzLm1fbG9jYWxDZW50ZXJCKTtcclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JCLCB0aGlzLm1fbG9jYWxDZW50ZXJCLCB0aGlzLm1fbGFsY0IpO1xyXG4gICAgYjJSb3QuTXVsUlYocUIsIHRoaXMubV9sYWxjQiwgdGhpcy5tX3JCKTtcclxuICAgIC8vIHRoaXMubV91ID0gY0IgKyB0aGlzLm1fckIgLSBjQSAtIHRoaXMubV9yQTtcclxuICAgIHRoaXMubV91LkNvcHkoY0IpLlNlbGZBZGQodGhpcy5tX3JCKS5TZWxmU3ViKGNBKS5TZWxmU3ViKHRoaXMubV9yQSk7XHJcblxyXG4gICAgdGhpcy5tX2xlbmd0aCA9IHRoaXMubV91Lkxlbmd0aCgpO1xyXG5cclxuICAgIGNvbnN0IEM6IG51bWJlciA9IHRoaXMubV9sZW5ndGggLSB0aGlzLm1fbWF4TGVuZ3RoO1xyXG4gICAgaWYgKEMgPiAwKSB7XHJcbiAgICAgIHRoaXMubV9zdGF0ZSA9IGIyTGltaXRTdGF0ZS5lX2F0VXBwZXJMaW1pdDtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV9zdGF0ZSA9IGIyTGltaXRTdGF0ZS5lX2luYWN0aXZlTGltaXQ7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHRoaXMubV9sZW5ndGggPiBiMl9saW5lYXJTbG9wKSB7XHJcbiAgICAgIHRoaXMubV91LlNlbGZNdWwoMSAvIHRoaXMubV9sZW5ndGgpO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgdGhpcy5tX3UuU2V0WmVybygpO1xyXG4gICAgICB0aGlzLm1fbWFzcyA9IDA7XHJcbiAgICAgIHRoaXMubV9pbXB1bHNlID0gMDtcclxuICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIENvbXB1dGUgZWZmZWN0aXZlIG1hc3MuXHJcbiAgICBjb25zdCBjckE6IG51bWJlciA9IGIyVmVjMi5Dcm9zc1ZWKHRoaXMubV9yQSwgdGhpcy5tX3UpO1xyXG4gICAgY29uc3QgY3JCOiBudW1iZXIgPSBiMlZlYzIuQ3Jvc3NWVih0aGlzLm1fckIsIHRoaXMubV91KTtcclxuICAgIGNvbnN0IGludk1hc3M6IG51bWJlciA9XHJcbiAgICAgIHRoaXMubV9pbnZNYXNzQSArIHRoaXMubV9pbnZJQSAqIGNyQSAqIGNyQSArIHRoaXMubV9pbnZNYXNzQiArIHRoaXMubV9pbnZJQiAqIGNyQiAqIGNyQjtcclxuXHJcbiAgICB0aGlzLm1fbWFzcyA9IGludk1hc3MgIT09IDAgPyAxIC8gaW52TWFzcyA6IDA7XHJcblxyXG4gICAgaWYgKGRhdGEuc3RlcC53YXJtU3RhcnRpbmcpIHtcclxuICAgICAgLy8gU2NhbGUgdGhlIGltcHVsc2UgdG8gc3VwcG9ydCBhIHZhcmlhYmxlIHRpbWUgc3RlcC5cclxuICAgICAgdGhpcy5tX2ltcHVsc2UgKj0gZGF0YS5zdGVwLmR0UmF0aW87XHJcblxyXG4gICAgICAvLyBiMlZlYzIgUCA9IG1faW1wdWxzZSAqIG1fdTtcclxuICAgICAgY29uc3QgUDogYjJWZWMyID0gYjJWZWMyLk11bFNWKFxyXG4gICAgICAgIHRoaXMubV9pbXB1bHNlLFxyXG4gICAgICAgIHRoaXMubV91LFxyXG4gICAgICAgIGIyUm9wZUpvaW50LkluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUCxcclxuICAgICAgKTtcclxuICAgICAgLy8gdkEgLT0gbV9pbnZNYXNzQSAqIFA7XHJcbiAgICAgIHZBLlNlbGZNdWxTdWIodGhpcy5tX2ludk1hc3NBLCBQKTtcclxuICAgICAgd0EgLT0gdGhpcy5tX2ludklBICogYjJWZWMyLkNyb3NzVlYodGhpcy5tX3JBLCBQKTtcclxuICAgICAgLy8gdkIgKz0gbV9pbnZNYXNzQiAqIFA7XHJcbiAgICAgIHZCLlNlbGZNdWxBZGQodGhpcy5tX2ludk1hc3NCLCBQKTtcclxuICAgICAgd0IgKz0gdGhpcy5tX2ludklCICogYjJWZWMyLkNyb3NzVlYodGhpcy5tX3JCLCBQKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV9pbXB1bHNlID0gMDtcclxuICAgIH1cclxuXHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udiA9IHZBO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLncgPSB3QTtcclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52ID0gdkI7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udyA9IHdCO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfdnBBID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX3ZwQiA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19QID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogdm9pZCB7XHJcbiAgICBjb25zdCB2QTogYjJWZWMyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnY7XHJcbiAgICBsZXQgd0E6IG51bWJlciA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS53O1xyXG4gICAgY29uc3QgdkI6IGIyVmVjMiA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52O1xyXG4gICAgbGV0IHdCOiBudW1iZXIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udztcclxuXHJcbiAgICAvLyBDZG90ID0gZG90KHUsIHYgKyBjcm9zcyh3LCByKSlcclxuICAgIC8vIGIyVmVjMiB2cEEgPSB2QSArIGIyQ3Jvc3Mod0EsIG1fckEpO1xyXG4gICAgY29uc3QgdnBBOiBiMlZlYzIgPSBiMlZlYzIuQWRkVkNyb3NzU1YoXHJcbiAgICAgIHZBLFxyXG4gICAgICB3QSxcclxuICAgICAgdGhpcy5tX3JBLFxyXG4gICAgICBiMlJvcGVKb2ludC5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc192cEEsXHJcbiAgICApO1xyXG4gICAgLy8gYjJWZWMyIHZwQiA9IHZCICsgYjJDcm9zcyh3QiwgbV9yQik7XHJcbiAgICBjb25zdCB2cEI6IGIyVmVjMiA9IGIyVmVjMi5BZGRWQ3Jvc3NTVihcclxuICAgICAgdkIsXHJcbiAgICAgIHdCLFxyXG4gICAgICB0aGlzLm1fckIsXHJcbiAgICAgIGIyUm9wZUpvaW50LlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX3ZwQixcclxuICAgICk7XHJcbiAgICAvLyBmbG9hdDMyIEMgPSBtX2xlbmd0aCAtIG1fbWF4TGVuZ3RoO1xyXG4gICAgY29uc3QgQzogbnVtYmVyID0gdGhpcy5tX2xlbmd0aCAtIHRoaXMubV9tYXhMZW5ndGg7XHJcbiAgICAvLyBmbG9hdDMyIENkb3QgPSBiMkRvdChtX3UsIHZwQiAtIHZwQSk7XHJcbiAgICBsZXQgQ2RvdDogbnVtYmVyID0gYjJWZWMyLkRvdFZWKHRoaXMubV91LCBiMlZlYzIuU3ViVlYodnBCLCB2cEEsIGIyVmVjMi5zX3QwKSk7XHJcblxyXG4gICAgLy8gUHJlZGljdGl2ZSBjb25zdHJhaW50LlxyXG4gICAgaWYgKEMgPCAwKSB7XHJcbiAgICAgIENkb3QgKz0gZGF0YS5zdGVwLmludl9kdCAqIEM7XHJcbiAgICB9XHJcblxyXG4gICAgbGV0IGltcHVsc2U6IG51bWJlciA9IC10aGlzLm1fbWFzcyAqIENkb3Q7XHJcbiAgICBjb25zdCBvbGRJbXB1bHNlOiBudW1iZXIgPSB0aGlzLm1faW1wdWxzZTtcclxuICAgIHRoaXMubV9pbXB1bHNlID0gYjJNaW4oMCwgdGhpcy5tX2ltcHVsc2UgKyBpbXB1bHNlKTtcclxuICAgIGltcHVsc2UgPSB0aGlzLm1faW1wdWxzZSAtIG9sZEltcHVsc2U7XHJcblxyXG4gICAgLy8gYjJWZWMyIFAgPSBpbXB1bHNlICogbV91O1xyXG4gICAgY29uc3QgUDogYjJWZWMyID0gYjJWZWMyLk11bFNWKGltcHVsc2UsIHRoaXMubV91LCBiMlJvcGVKb2ludC5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19QKTtcclxuICAgIC8vIHZBIC09IG1faW52TWFzc0EgKiBQO1xyXG4gICAgdkEuU2VsZk11bFN1Yih0aGlzLm1faW52TWFzc0EsIFApO1xyXG4gICAgd0EgLT0gdGhpcy5tX2ludklBICogYjJWZWMyLkNyb3NzVlYodGhpcy5tX3JBLCBQKTtcclxuICAgIC8vIHZCICs9IG1faW52TWFzc0IgKiBQO1xyXG4gICAgdkIuU2VsZk11bEFkZCh0aGlzLm1faW52TWFzc0IsIFApO1xyXG4gICAgd0IgKz0gdGhpcy5tX2ludklCICogYjJWZWMyLkNyb3NzVlYodGhpcy5tX3JCLCBQKTtcclxuXHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udiA9IHZBO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLncgPSB3QTtcclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52ID0gdkI7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udyA9IHdCO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfUCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzKGRhdGE6IGIyU29sdmVyRGF0YSk6IGJvb2xlYW4ge1xyXG4gICAgY29uc3QgY0E6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmM7XHJcbiAgICBsZXQgYUE6IG51bWJlciA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmE7XHJcbiAgICBjb25zdCBjQjogYjJWZWMyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Ql0uYztcclxuICAgIGxldCBhQjogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Ql0uYTtcclxuXHJcbiAgICBjb25zdCBxQTogYjJSb3QgPSB0aGlzLm1fcUEuU2V0QW5nbGUoYUEpLFxyXG4gICAgICBxQjogYjJSb3QgPSB0aGlzLm1fcUIuU2V0QW5nbGUoYUIpO1xyXG5cclxuICAgIC8vIGIyVmVjMiByQSA9IGIyTXVsKHFBLCB0aGlzLm1fbG9jYWxBbmNob3JBIC0gdGhpcy5tX2xvY2FsQ2VudGVyQSk7XHJcbiAgICBiMlZlYzIuU3ViVlYodGhpcy5tX2xvY2FsQW5jaG9yQSwgdGhpcy5tX2xvY2FsQ2VudGVyQSwgdGhpcy5tX2xhbGNBKTtcclxuICAgIGNvbnN0IHJBOiBiMlZlYzIgPSBiMlJvdC5NdWxSVihxQSwgdGhpcy5tX2xhbGNBLCB0aGlzLm1fckEpO1xyXG4gICAgLy8gYjJWZWMyIHJCID0gYjJNdWwocUIsIHRoaXMubV9sb2NhbEFuY2hvckIgLSB0aGlzLm1fbG9jYWxDZW50ZXJCKTtcclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JCLCB0aGlzLm1fbG9jYWxDZW50ZXJCLCB0aGlzLm1fbGFsY0IpO1xyXG4gICAgY29uc3QgckI6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFCLCB0aGlzLm1fbGFsY0IsIHRoaXMubV9yQik7XHJcbiAgICAvLyBiMlZlYzIgdSA9IGNCICsgckIgLSBjQSAtIHJBO1xyXG4gICAgY29uc3QgdTogYjJWZWMyID0gdGhpcy5tX3UuQ29weShjQikuU2VsZkFkZChyQikuU2VsZlN1YihjQSkuU2VsZlN1YihyQSk7XHJcblxyXG4gICAgY29uc3QgbGVuZ3RoOiBudW1iZXIgPSB1Lk5vcm1hbGl6ZSgpO1xyXG4gICAgbGV0IEM6IG51bWJlciA9IGxlbmd0aCAtIHRoaXMubV9tYXhMZW5ndGg7XHJcblxyXG4gICAgQyA9IGIyQ2xhbXAoQywgMCwgYjJfbWF4TGluZWFyQ29ycmVjdGlvbik7XHJcblxyXG4gICAgY29uc3QgaW1wdWxzZTogbnVtYmVyID0gLXRoaXMubV9tYXNzICogQztcclxuICAgIC8vIGIyVmVjMiBQID0gaW1wdWxzZSAqIHU7XHJcbiAgICBjb25zdCBQOiBiMlZlYzIgPSBiMlZlYzIuTXVsU1YoaW1wdWxzZSwgdSwgYjJSb3BlSm9pbnQuU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfUCk7XHJcblxyXG4gICAgLy8gY0EgLT0gbV9pbnZNYXNzQSAqIFA7XHJcbiAgICBjQS5TZWxmTXVsU3ViKHRoaXMubV9pbnZNYXNzQSwgUCk7XHJcbiAgICBhQSAtPSB0aGlzLm1faW52SUEgKiBiMlZlYzIuQ3Jvc3NWVihyQSwgUCk7XHJcbiAgICAvLyBjQiArPSBtX2ludk1hc3NCICogUDtcclxuICAgIGNCLlNlbGZNdWxBZGQodGhpcy5tX2ludk1hc3NCLCBQKTtcclxuICAgIGFCICs9IHRoaXMubV9pbnZJQiAqIGIyVmVjMi5Dcm9zc1ZWKHJCLCBQKTtcclxuXHJcbiAgICAvLyBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5jID0gY0E7XHJcbiAgICBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5hID0gYUE7XHJcbiAgICAvLyBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5jID0gY0I7XHJcbiAgICBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5hID0gYUI7XHJcblxyXG4gICAgcmV0dXJuIGxlbmd0aCAtIHRoaXMubV9tYXhMZW5ndGggPCBiMl9saW5lYXJTbG9wO1xyXG4gIH1cclxuXHJcbiAgR2V0QW5jaG9yQTxUIGV4dGVuZHMgWFk+KG91dDogVCk6IFQge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ib2R5QS5HZXRXb3JsZFBvaW50KHRoaXMubV9sb2NhbEFuY2hvckEsIG91dCk7XHJcbiAgfVxyXG5cclxuICBHZXRBbmNob3JCPFQgZXh0ZW5kcyBYWT4ob3V0OiBUKTogVCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2JvZHlCLkdldFdvcmxkUG9pbnQodGhpcy5tX2xvY2FsQW5jaG9yQiwgb3V0KTtcclxuICB9XHJcblxyXG4gIEdldFJlYWN0aW9uRm9yY2U8VCBleHRlbmRzIFhZPihpbnZfZHQ6IG51bWJlciwgb3V0OiBUKTogVCB7XHJcbiAgICAvLyByZXR1cm4gb3V0LlNldChpbnZfZHQgKiB0aGlzLm1fbGluZWFySW1wdWxzZS54LCBpbnZfZHQgKiB0aGlzLm1fbGluZWFySW1wdWxzZS55KTtcclxuICAgIHJldHVybiBiMlZlYzIuTXVsU1YoaW52X2R0ICogdGhpcy5tX2ltcHVsc2UsIHRoaXMubV91LCBvdXQpO1xyXG4gIH1cclxuXHJcbiAgR2V0UmVhY3Rpb25Ub3JxdWUoaW52X2R0OiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIDA7XHJcbiAgfVxyXG5cclxuICBHZXRMb2NhbEFuY2hvckEoKTogUmVhZG9ubHk8YjJWZWMyPiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2xvY2FsQW5jaG9yQTtcclxuICB9XHJcblxyXG4gIEdldExvY2FsQW5jaG9yQigpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHJldHVybiB0aGlzLm1fbG9jYWxBbmNob3JCO1xyXG4gIH1cclxuXHJcbiAgU2V0TWF4TGVuZ3RoKGxlbmd0aDogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fbWF4TGVuZ3RoID0gbGVuZ3RoO1xyXG4gIH1cclxuXHJcbiAgR2V0TWF4TGVuZ3RoKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX21heExlbmd0aDtcclxuICB9XHJcblxyXG4gIEdldExpbWl0U3RhdGUoKTogYjJMaW1pdFN0YXRlIHtcclxuICAgIHJldHVybiB0aGlzLm1fc3RhdGU7XHJcbiAgfVxyXG59XHJcbiJdfQ==