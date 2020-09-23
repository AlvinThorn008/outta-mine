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
import { b2_angularSlop, b2_linearSlop, b2_maxLinearCorrection, b2Maybe, } from '../../common/b2Settings';
import { b2Abs, b2Clamp, b2Mat22, b2Mat33, b2Max, b2Min, b2Rot, b2Vec2, b2Vec3, } from '../../common/b2Math';
import { b2Joint, b2JointDef } from './b2Joint';
/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
export class b2PrismaticJointDef extends b2JointDef {
    constructor() {
        super(2 /* e_prismaticJoint */);
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
        this.localAxisA = new b2Vec2(1, 0);
        this.referenceAngle = 0;
        this.enableLimit = false;
        this.lowerTranslation = 0;
        this.upperTranslation = 0;
        this.enableMotor = false;
        this.maxMotorForce = 0;
        this.motorSpeed = 0;
    }
    Initialize(bA, bB, anchor, axis) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.bodyA.GetLocalVector(axis, this.localAxisA);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
}
export class b2PrismaticJoint extends b2Joint {
    constructor(def) {
        super(def);
        // Solver shared
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_localXAxisA = new b2Vec2();
        this.m_localYAxisA = new b2Vec2();
        this.m_referenceAngle = 0;
        this.m_impulse = new b2Vec3(0, 0, 0);
        this.m_motorImpulse = 0;
        this.m_lowerTranslation = 0;
        this.m_upperTranslation = 0;
        this.m_maxMotorForce = 0;
        this.m_motorSpeed = 0;
        this.m_enableLimit = false;
        this.m_enableMotor = false;
        this.m_limitState = 0 /* e_inactiveLimit */;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_axis = new b2Vec2(0, 0);
        this.m_perp = new b2Vec2(0, 0);
        this.m_s1 = 0;
        this.m_s2 = 0;
        this.m_a1 = 0;
        this.m_a2 = 0;
        this.m_K = new b2Mat33();
        this.m_K3 = new b2Mat33();
        this.m_K2 = new b2Mat22();
        this.m_motorMass = 0;
        this.m_qA = new b2Rot();
        this.m_qB = new b2Rot();
        this.m_lalcA = new b2Vec2();
        this.m_lalcB = new b2Vec2();
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localAnchorA.Copy(b2Maybe(def.localAnchorA, b2Vec2.ZERO));
        this.m_localAnchorB.Copy(b2Maybe(def.localAnchorB, b2Vec2.ZERO));
        this.m_localXAxisA.Copy(b2Maybe(def.localAxisA, new b2Vec2(1, 0))).SelfNormalize();
        b2Vec2.CrossOneV(this.m_localXAxisA, this.m_localYAxisA);
        this.m_referenceAngle = b2Maybe(def.referenceAngle, 0);
        this.m_lowerTranslation = b2Maybe(def.lowerTranslation, 0);
        this.m_upperTranslation = b2Maybe(def.upperTranslation, 0);
        this.m_maxMotorForce = b2Maybe(def.maxMotorForce, 0);
        this.m_motorSpeed = b2Maybe(def.motorSpeed, 0);
        this.m_enableLimit = b2Maybe(def.enableLimit, false);
        this.m_enableMotor = b2Maybe(def.enableMotor, false);
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
        // Compute the effective masses.
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 d = (cB - cA) + rB - rA;
        const d = b2Vec2.AddVV(b2Vec2.SubVV(cB, cA, b2Vec2.s_t0), b2Vec2.SubVV(rB, rA, b2Vec2.s_t1), b2PrismaticJoint.InitVelocityConstraints_s_d);
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        // Compute motor Jacobian and effective mass.
        {
            // m_axis = b2Mul(qA, m_localXAxisA);
            b2Rot.MulRV(qA, this.m_localXAxisA, this.m_axis);
            // m_a1 = b2Cross(d + rA, m_axis);
            this.m_a1 = b2Vec2.CrossVV(b2Vec2.AddVV(d, rA, b2Vec2.s_t0), this.m_axis);
            // m_a2 = b2Cross(rB, m_axis);
            this.m_a2 = b2Vec2.CrossVV(rB, this.m_axis);
            this.m_motorMass = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
            if (this.m_motorMass > 0) {
                this.m_motorMass = 1 / this.m_motorMass;
            }
        }
        // Prismatic constraint.
        {
            // m_perp = b2Mul(qA, m_localYAxisA);
            b2Rot.MulRV(qA, this.m_localYAxisA, this.m_perp);
            // m_s1 = b2Cross(d + rA, m_perp);
            this.m_s1 = b2Vec2.CrossVV(b2Vec2.AddVV(d, rA, b2Vec2.s_t0), this.m_perp);
            // m_s2 = b2Cross(rB, m_perp);
            this.m_s2 = b2Vec2.CrossVV(rB, this.m_perp);
            // float32 k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
            this.m_K.ex.x = mA + mB + iA * this.m_s1 * this.m_s1 + iB * this.m_s2 * this.m_s2;
            // float32 k12 = iA * m_s1 + iB * m_s2;
            this.m_K.ex.y = iA * this.m_s1 + iB * this.m_s2;
            // float32 k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
            this.m_K.ex.z = iA * this.m_s1 * this.m_a1 + iB * this.m_s2 * this.m_a2;
            this.m_K.ey.x = this.m_K.ex.y;
            // float32 k22 = iA + iB;
            this.m_K.ey.y = iA + iB;
            if (this.m_K.ey.y === 0) {
                // For bodies with fixed rotation.
                this.m_K.ey.y = 1;
            }
            // float32 k23 = iA * m_a1 + iB * m_a2;
            this.m_K.ey.z = iA * this.m_a1 + iB * this.m_a2;
            this.m_K.ez.x = this.m_K.ex.z;
            this.m_K.ez.y = this.m_K.ey.z;
            // float32 k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
            this.m_K.ez.z = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
            // m_K.ex.Set(k11, k12, k13);
            // m_K.ey.Set(k12, k22, k23);
            // m_K.ez.Set(k13, k23, k33);
        }
        // Compute motor and limit terms.
        if (this.m_enableLimit) {
            // float32 jointTranslation = b2Dot(m_axis, d);
            const jointTranslation = b2Vec2.DotVV(this.m_axis, d);
            if (b2Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2 * b2_linearSlop) {
                this.m_limitState = 3 /* e_equalLimits */;
            }
            else if (jointTranslation <= this.m_lowerTranslation) {
                if (this.m_limitState !== 1 /* e_atLowerLimit */) {
                    this.m_limitState = 1 /* e_atLowerLimit */;
                    this.m_impulse.z = 0;
                }
            }
            else if (jointTranslation >= this.m_upperTranslation) {
                if (this.m_limitState !== 2 /* e_atUpperLimit */) {
                    this.m_limitState = 2 /* e_atUpperLimit */;
                    this.m_impulse.z = 0;
                }
            }
            else {
                this.m_limitState = 0 /* e_inactiveLimit */;
                this.m_impulse.z = 0;
            }
        }
        else {
            this.m_limitState = 0 /* e_inactiveLimit */;
            this.m_impulse.z = 0;
        }
        if (!this.m_enableMotor) {
            this.m_motorImpulse = 0;
        }
        if (data.step.warmStarting) {
            // Account for variable time step.
            // m_impulse *= data.step.dtRatio;
            this.m_impulse.SelfMul(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;
            // b2Vec2 P = m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
            const P = b2Vec2.AddVV(b2Vec2.MulSV(this.m_impulse.x, this.m_perp, b2Vec2.s_t0), b2Vec2.MulSV(this.m_motorImpulse + this.m_impulse.z, this.m_axis, b2Vec2.s_t1), b2PrismaticJoint.InitVelocityConstraints_s_P);
            // float32 LA = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
            const LA = this.m_impulse.x * this.m_s1 +
                this.m_impulse.y +
                (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
            // float32 LB = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;
            const LB = this.m_impulse.x * this.m_s2 +
                this.m_impulse.y +
                (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0;
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
        // Solve linear motor constraint.
        if (this.m_enableMotor && this.m_limitState !== 3 /* e_equalLimits */) {
            // float32 Cdot = b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
            const Cdot = b2Vec2.DotVV(this.m_axis, b2Vec2.SubVV(vB, vA, b2Vec2.s_t0)) +
                this.m_a2 * wB -
                this.m_a1 * wA;
            let impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorForce;
            this.m_motorImpulse = b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;
            // b2Vec2 P = impulse * m_axis;
            const P = b2Vec2.MulSV(impulse, this.m_axis, b2PrismaticJoint.SolveVelocityConstraints_s_P);
            const LA = impulse * this.m_a1;
            const LB = impulse * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        // b2Vec2 Cdot1;
        // Cdot1.x = b2Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
        const Cdot1_x = b2Vec2.DotVV(this.m_perp, b2Vec2.SubVV(vB, vA, b2Vec2.s_t0)) +
            this.m_s2 * wB -
            this.m_s1 * wA;
        // Cdot1.y = wB - wA;
        const Cdot1_y = wB - wA;
        if (this.m_enableLimit && this.m_limitState !== 0 /* e_inactiveLimit */) {
            // Solve prismatic and limit constraint in block form.
            // float32 Cdot2;
            // Cdot2 = b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
            const Cdot2 = b2Vec2.DotVV(this.m_axis, b2Vec2.SubVV(vB, vA, b2Vec2.s_t0)) +
                this.m_a2 * wB -
                this.m_a1 * wA;
            // b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);
            // b2Vec3 f1 = m_impulse;
            const f1 = b2PrismaticJoint.SolveVelocityConstraints_s_f1.Copy(this.m_impulse);
            // b2Vec3 df =  m_K.Solve33(-Cdot);
            const df3 = this.m_K.Solve33(-Cdot1_x, -Cdot1_y, -Cdot2, b2PrismaticJoint.SolveVelocityConstraints_s_df3);
            // m_impulse += df;
            this.m_impulse.SelfAdd(df3);
            if (this.m_limitState === 1 /* e_atLowerLimit */) {
                this.m_impulse.z = b2Max(this.m_impulse.z, 0);
            }
            else if (this.m_limitState === 2 /* e_atUpperLimit */) {
                this.m_impulse.z = b2Min(this.m_impulse.z, 0);
            }
            // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
            // b2Vec2 b = -Cdot1 - (m_impulse.z - f1.z) * b2Vec2(m_K.ez.x, m_K.ez.y);
            const b_x = -Cdot1_x - (this.m_impulse.z - f1.z) * this.m_K.ez.x;
            const b_y = -Cdot1_y - (this.m_impulse.z - f1.z) * this.m_K.ez.y;
            // b2Vec2 f2r = m_K.Solve22(b) + b2Vec2(f1.x, f1.y);
            const f2r = this.m_K.Solve22(b_x, b_y, b2PrismaticJoint.SolveVelocityConstraints_s_f2r);
            f2r.x += f1.x;
            f2r.y += f1.y;
            // m_impulse.x = f2r.x;
            this.m_impulse.x = f2r.x;
            // m_impulse.y = f2r.y;
            this.m_impulse.y = f2r.y;
            // df = m_impulse - f1;
            df3.x = this.m_impulse.x - f1.x;
            df3.y = this.m_impulse.y - f1.y;
            df3.z = this.m_impulse.z - f1.z;
            // b2Vec2 P = df.x * m_perp + df.z * m_axis;
            const P = b2Vec2.AddVV(b2Vec2.MulSV(df3.x, this.m_perp, b2Vec2.s_t0), b2Vec2.MulSV(df3.z, this.m_axis, b2Vec2.s_t1), b2PrismaticJoint.SolveVelocityConstraints_s_P);
            // float32 LA = df.x * m_s1 + df.y + df.z * m_a1;
            const LA = df3.x * this.m_s1 + df3.y + df3.z * this.m_a1;
            // float32 LB = df.x * m_s2 + df.y + df.z * m_a2;
            const LB = df3.x * this.m_s2 + df3.y + df3.z * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        else {
            // Limit is inactive, just solve the prismatic constraint in block form.
            // b2Vec2 df = m_K.Solve22(-Cdot1);
            const df2 = this.m_K.Solve22(-Cdot1_x, -Cdot1_y, b2PrismaticJoint.SolveVelocityConstraints_s_df2);
            this.m_impulse.x += df2.x;
            this.m_impulse.y += df2.y;
            // b2Vec2 P = df.x * m_perp;
            const P = b2Vec2.MulSV(df2.x, this.m_perp, b2PrismaticJoint.SolveVelocityConstraints_s_P);
            // float32 LA = df.x * m_s1 + df.y;
            const LA = df2.x * this.m_s1 + df2.y;
            // float32 LB = df.x * m_s2 + df.y;
            const LB = df2.x * this.m_s2 + df2.y;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
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
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        const rA = b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        const rB = b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 d = cB + rB - cA - rA;
        const d = b2Vec2.SubVV(b2Vec2.AddVV(cB, rB, b2Vec2.s_t0), b2Vec2.AddVV(cA, rA, b2Vec2.s_t1), b2PrismaticJoint.SolvePositionConstraints_s_d);
        // b2Vec2 axis = b2Mul(qA, m_localXAxisA);
        const axis = b2Rot.MulRV(qA, this.m_localXAxisA, this.m_axis);
        // float32 a1 = b2Cross(d + rA, axis);
        const a1 = b2Vec2.CrossVV(b2Vec2.AddVV(d, rA, b2Vec2.s_t0), axis);
        // float32 a2 = b2Cross(rB, axis);
        const a2 = b2Vec2.CrossVV(rB, axis);
        // b2Vec2 perp = b2Mul(qA, m_localYAxisA);
        const perp = b2Rot.MulRV(qA, this.m_localYAxisA, this.m_perp);
        // float32 s1 = b2Cross(d + rA, perp);
        const s1 = b2Vec2.CrossVV(b2Vec2.AddVV(d, rA, b2Vec2.s_t0), perp);
        // float32 s2 = b2Cross(rB, perp);
        const s2 = b2Vec2.CrossVV(rB, perp);
        // b2Vec3 impulse;
        let impulse = b2PrismaticJoint.SolvePositionConstraints_s_impulse;
        // b2Vec2 C1;
        // C1.x = b2Dot(perp, d);
        const C1_x = b2Vec2.DotVV(perp, d);
        // C1.y = aB - aA - m_referenceAngle;
        const C1_y = aB - aA - this.m_referenceAngle;
        let linearError = b2Abs(C1_x);
        const angularError = b2Abs(C1_y);
        let active = false;
        let C2 = 0;
        if (this.m_enableLimit) {
            // float32 translation = b2Dot(axis, d);
            const translation = b2Vec2.DotVV(axis, d);
            if (b2Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2 * b2_linearSlop) {
                // Prevent large angular corrections
                C2 = b2Clamp(translation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
                linearError = b2Max(linearError, b2Abs(translation));
                active = true;
            }
            else if (translation <= this.m_lowerTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = b2Clamp(translation - this.m_lowerTranslation + b2_linearSlop, -b2_maxLinearCorrection, 0);
                linearError = b2Max(linearError, this.m_lowerTranslation - translation);
                active = true;
            }
            else if (translation >= this.m_upperTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = b2Clamp(translation - this.m_upperTranslation - b2_linearSlop, 0, b2_maxLinearCorrection);
                linearError = b2Max(linearError, translation - this.m_upperTranslation);
                active = true;
            }
        }
        if (active) {
            // float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            const k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            // float32 k12 = iA * s1 + iB * s2;
            const k12 = iA * s1 + iB * s2;
            // float32 k13 = iA * s1 * a1 + iB * s2 * a2;
            const k13 = iA * s1 * a1 + iB * s2 * a2;
            // float32 k22 = iA + iB;
            let k22 = iA + iB;
            if (k22 === 0) {
                // For fixed rotation
                k22 = 1;
            }
            // float32 k23 = iA * a1 + iB * a2;
            const k23 = iA * a1 + iB * a2;
            // float32 k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            const k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            // b2Mat33 K;
            const K = this.m_K3;
            // K.ex.Set(k11, k12, k13);
            K.ex.SetXYZ(k11, k12, k13);
            // K.ey.Set(k12, k22, k23);
            K.ey.SetXYZ(k12, k22, k23);
            // K.ez.Set(k13, k23, k33);
            K.ez.SetXYZ(k13, k23, k33);
            // b2Vec3 C;
            // C.x = C1.x;
            // C.y = C1.y;
            // C.z = C2;
            // impulse = K.Solve33(-C);
            impulse = K.Solve33(-C1_x, -C1_y, -C2, impulse);
        }
        else {
            // float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            const k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            // float32 k12 = iA * s1 + iB * s2;
            const k12 = iA * s1 + iB * s2;
            // float32 k22 = iA + iB;
            let k22 = iA + iB;
            if (k22 === 0) {
                k22 = 1;
            }
            // b2Mat22 K;
            const K2 = this.m_K2;
            // K.ex.Set(k11, k12);
            K2.ex.Set(k11, k12);
            // K.ey.Set(k12, k22);
            K2.ey.Set(k12, k22);
            // b2Vec2 impulse1 = K.Solve(-C1);
            const impulse1 = K2.Solve(-C1_x, -C1_y, b2PrismaticJoint.SolvePositionConstraints_s_impulse1);
            impulse.x = impulse1.x;
            impulse.y = impulse1.y;
            impulse.z = 0;
        }
        // b2Vec2 P = impulse.x * perp + impulse.z * axis;
        const P = b2Vec2.AddVV(b2Vec2.MulSV(impulse.x, perp, b2Vec2.s_t0), b2Vec2.MulSV(impulse.z, axis, b2Vec2.s_t1), b2PrismaticJoint.SolvePositionConstraints_s_P);
        // float32 LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        const LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        // float32 LB = impulse.x * s2 + impulse.y + impulse.z * a2;
        const LB = impulse.x * s2 + impulse.y + impulse.z * a2;
        // cA -= mA * P;
        cA.SelfMulSub(mA, P);
        aA -= iA * LA;
        // cB += mB * P;
        cB.SelfMulAdd(mB, P);
        aB += iB * LB;
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return linearError <= b2_linearSlop && angularError <= b2_angularSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
        out.x =
            inv_dt *
                (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x);
        out.y =
            inv_dt *
                (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y);
        return out;
    }
    GetReactionTorque(inv_dt) {
        return inv_dt * this.m_impulse.y;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    GetLocalAxisA() {
        return this.m_localXAxisA;
    }
    GetReferenceAngle() {
        return this.m_referenceAngle;
    }
    GetJointTranslation() {
        // b2Vec2 pA = m_bodyA.GetWorldPoint(m_localAnchorA);
        const pA = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, b2PrismaticJoint.GetJointTranslation_s_pA);
        // b2Vec2 pB = m_bodyB.GetWorldPoint(m_localAnchorB);
        const pB = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, b2PrismaticJoint.GetJointTranslation_s_pB);
        // b2Vec2 d = pB - pA;
        const d = b2Vec2.SubVV(pB, pA, b2PrismaticJoint.GetJointTranslation_s_d);
        // b2Vec2 axis = m_bodyA.GetWorldVector(m_localXAxisA);
        const axis = this.m_bodyA.GetWorldVector(this.m_localXAxisA, b2PrismaticJoint.GetJointTranslation_s_axis);
        // float32 translation = b2Dot(d, axis);
        const translation = b2Vec2.DotVV(d, axis);
        return translation;
    }
    GetJointSpeed() {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;
        // b2Vec2 rA = b2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
        b2Vec2.SubVV(this.m_localAnchorA, bA.m_sweep.localCenter, this.m_lalcA);
        const rA = b2Rot.MulRV(bA.m_xf.q, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
        b2Vec2.SubVV(this.m_localAnchorB, bB.m_sweep.localCenter, this.m_lalcB);
        const rB = b2Rot.MulRV(bB.m_xf.q, this.m_lalcB, this.m_rB);
        // b2Vec2 pA = bA->m_sweep.c + rA;
        const pA = b2Vec2.AddVV(bA.m_sweep.c, rA, b2Vec2.s_t0); // pA uses s_t0
        // b2Vec2 pB = bB->m_sweep.c + rB;
        const pB = b2Vec2.AddVV(bB.m_sweep.c, rB, b2Vec2.s_t1); // pB uses s_t1
        // b2Vec2 d = pB - pA;
        const d = b2Vec2.SubVV(pB, pA, b2Vec2.s_t2); // d uses s_t2
        // b2Vec2 axis = b2Mul(bA.m_xf.q, m_localXAxisA);
        const axis = bA.GetWorldVector(this.m_localXAxisA, this.m_axis);
        const vA = bA.m_linearVelocity;
        const vB = bB.m_linearVelocity;
        const wA = bA.m_angularVelocity;
        const wB = bB.m_angularVelocity;
        // float32 speed = b2Dot(d, b2Cross(wA, axis)) + b2Dot(axis, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
        const speed = b2Vec2.DotVV(d, b2Vec2.CrossSV(wA, axis, b2Vec2.s_t0)) +
            b2Vec2.DotVV(axis, b2Vec2.SubVV(b2Vec2.AddVCrossSV(vB, wB, rB, b2Vec2.s_t0), b2Vec2.AddVCrossSV(vA, wA, rA, b2Vec2.s_t1), b2Vec2.s_t0));
        return speed;
    }
    IsLimitEnabled() {
        return this.m_enableLimit;
    }
    EnableLimit(flag) {
        if (flag !== this.m_enableLimit) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableLimit = flag;
            this.m_impulse.z = 0;
        }
    }
    GetLowerLimit() {
        return this.m_lowerTranslation;
    }
    GetUpperLimit() {
        return this.m_upperTranslation;
    }
    SetLimits(lower, upper) {
        if (lower !== this.m_lowerTranslation || upper !== this.m_upperTranslation) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_lowerTranslation = lower;
            this.m_upperTranslation = upper;
            this.m_impulse.z = 0;
        }
    }
    IsMotorEnabled() {
        return this.m_enableMotor;
    }
    EnableMotor(flag) {
        if (flag !== this.m_enableMotor) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableMotor = flag;
        }
    }
    SetMotorSpeed(speed) {
        if (speed !== this.m_motorSpeed) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_motorSpeed = speed;
        }
    }
    GetMotorSpeed() {
        return this.m_motorSpeed;
    }
    SetMaxMotorForce(force) {
        if (force !== this.m_maxMotorForce) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_maxMotorForce = force;
        }
    }
    GetMaxMotorForce() {
        return this.m_maxMotorForce;
    }
    GetMotorForce(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
}
b2PrismaticJoint.InitVelocityConstraints_s_d = new b2Vec2();
b2PrismaticJoint.InitVelocityConstraints_s_P = new b2Vec2();
b2PrismaticJoint.SolveVelocityConstraints_s_P = new b2Vec2();
b2PrismaticJoint.SolveVelocityConstraints_s_f2r = new b2Vec2();
b2PrismaticJoint.SolveVelocityConstraints_s_f1 = new b2Vec3();
b2PrismaticJoint.SolveVelocityConstraints_s_df3 = new b2Vec3();
b2PrismaticJoint.SolveVelocityConstraints_s_df2 = new b2Vec2();
// A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
// the position solver is not there to resolve forces.It is only there to cope with integration error.
//
// Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
//
// We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
// solver indicates the limit is inactive.
b2PrismaticJoint.SolvePositionConstraints_s_d = new b2Vec2();
b2PrismaticJoint.SolvePositionConstraints_s_impulse = new b2Vec3();
b2PrismaticJoint.SolvePositionConstraints_s_impulse1 = new b2Vec2();
b2PrismaticJoint.SolvePositionConstraints_s_P = new b2Vec2();
b2PrismaticJoint.GetJointTranslation_s_pA = new b2Vec2();
b2PrismaticJoint.GetJointTranslation_s_pB = new b2Vec2();
b2PrismaticJoint.GetJointTranslation_s_d = new b2Vec2();
b2PrismaticJoint.GetJointTranslation_s_axis = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQcmlzbWF0aWNKb2ludC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL3NyYy9keW5hbWljcy9qb2ludHMvYjJQcmlzbWF0aWNKb2ludC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFDTCxjQUFjLEVBQ2QsYUFBYSxFQUNiLHNCQUFzQixFQUN0QixPQUFPLEdBQ1IsTUFBTSx5QkFBeUIsQ0FBQztBQUNqQyxPQUFPLEVBQ0wsS0FBSyxFQUNMLE9BQU8sRUFDUCxPQUFPLEVBQ1AsT0FBTyxFQUNQLEtBQUssRUFDTCxLQUFLLEVBQ0wsS0FBSyxFQUNMLE1BQU0sRUFDTixNQUFNLEdBRVAsTUFBTSxxQkFBcUIsQ0FBQztBQUU3QixPQUFPLEVBQWUsT0FBTyxFQUFFLFVBQVUsRUFBNkIsTUFBTSxXQUFXLENBQUM7QUF5QnhGLGdFQUFnRTtBQUNoRSx1RUFBdUU7QUFDdkUsb0VBQW9FO0FBQ3BFLHNFQUFzRTtBQUN0RSxxRUFBcUU7QUFDckUsa0VBQWtFO0FBQ2xFLE1BQU0sT0FBTyxtQkFBb0IsU0FBUSxVQUFVO0lBcUJqRDtRQUNFLEtBQUssMEJBQThCLENBQUM7UUFyQjdCLGlCQUFZLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUVwQyxpQkFBWSxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7UUFFcEMsZUFBVSxHQUFXLElBQUksTUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUUvQyxtQkFBYyxHQUFHLENBQUMsQ0FBQztRQUVuQixnQkFBVyxHQUFHLEtBQUssQ0FBQztRQUVwQixxQkFBZ0IsR0FBRyxDQUFDLENBQUM7UUFFckIscUJBQWdCLEdBQUcsQ0FBQyxDQUFDO1FBRXJCLGdCQUFXLEdBQUcsS0FBSyxDQUFDO1FBRXBCLGtCQUFhLEdBQUcsQ0FBQyxDQUFDO1FBRWxCLGVBQVUsR0FBRyxDQUFDLENBQUM7SUFJZixDQUFDO0lBRUQs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M0IsNkNBQTRCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUM1QyxtREFBa0MsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQ2xELG9EQUFtQyxHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDbkQsNkNBQTRCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQStNNUMseUNBQXdCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUN4Qyx5Q0FBd0IsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQ3hDLHdDQUF1QixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDdkMsMkNBQTBCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDExIEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHtcclxuICBiMl9hbmd1bGFyU2xvcCxcclxuICBiMl9saW5lYXJTbG9wLFxyXG4gIGIyX21heExpbmVhckNvcnJlY3Rpb24sXHJcbiAgYjJNYXliZSxcclxufSBmcm9tICcuLi8uLi9jb21tb24vYjJTZXR0aW5ncyc7XHJcbmltcG9ydCB7XHJcbiAgYjJBYnMsXHJcbiAgYjJDbGFtcCxcclxuICBiMk1hdDIyLFxyXG4gIGIyTWF0MzMsXHJcbiAgYjJNYXgsXHJcbiAgYjJNaW4sXHJcbiAgYjJSb3QsXHJcbiAgYjJWZWMyLFxyXG4gIGIyVmVjMyxcclxuICBYWSxcclxufSBmcm9tICcuLi8uLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJCb2R5IH0gZnJvbSAnLi4vYjJCb2R5JztcclxuaW1wb3J0IHsgYjJJSm9pbnREZWYsIGIySm9pbnQsIGIySm9pbnREZWYsIGIySm9pbnRUeXBlLCBiMkxpbWl0U3RhdGUgfSBmcm9tICcuL2IySm9pbnQnO1xyXG5pbXBvcnQgeyBiMlNvbHZlckRhdGEgfSBmcm9tICcuLi9iMlRpbWVTdGVwJztcclxuXHJcbmV4cG9ydCBpbnRlcmZhY2UgYjJJUHJpc21hdGljSm9pbnREZWYgZXh0ZW5kcyBiMklKb2ludERlZiB7XHJcbiAgbG9jYWxBbmNob3JBPzogWFk7XHJcblxyXG4gIGxvY2FsQW5jaG9yQj86IFhZO1xyXG5cclxuICBsb2NhbEF4aXNBPzogWFk7XHJcblxyXG4gIHJlZmVyZW5jZUFuZ2xlPzogbnVtYmVyO1xyXG5cclxuICBlbmFibGVMaW1pdD86IGJvb2xlYW47XHJcblxyXG4gIGxvd2VyVHJhbnNsYXRpb24/OiBudW1iZXI7XHJcblxyXG4gIHVwcGVyVHJhbnNsYXRpb24/OiBudW1iZXI7XHJcblxyXG4gIGVuYWJsZU1vdG9yPzogYm9vbGVhbjtcclxuXHJcbiAgbWF4TW90b3JGb3JjZT86IG51bWJlcjtcclxuXHJcbiAgbW90b3JTcGVlZD86IG51bWJlcjtcclxufVxyXG5cclxuLy8vIFByaXNtYXRpYyBqb2ludCBkZWZpbml0aW9uLiBUaGlzIHJlcXVpcmVzIGRlZmluaW5nIGEgbGluZSBvZlxyXG4vLy8gbW90aW9uIHVzaW5nIGFuIGF4aXMgYW5kIGFuIGFuY2hvciBwb2ludC4gVGhlIGRlZmluaXRpb24gdXNlcyBsb2NhbFxyXG4vLy8gYW5jaG9yIHBvaW50cyBhbmQgYSBsb2NhbCBheGlzIHNvIHRoYXQgdGhlIGluaXRpYWwgY29uZmlndXJhdGlvblxyXG4vLy8gY2FuIHZpb2xhdGUgdGhlIGNvbnN0cmFpbnQgc2xpZ2h0bHkuIFRoZSBqb2ludCB0cmFuc2xhdGlvbiBpcyB6ZXJvXHJcbi8vLyB3aGVuIHRoZSBsb2NhbCBhbmNob3IgcG9pbnRzIGNvaW5jaWRlIGluIHdvcmxkIHNwYWNlLiBVc2luZyBsb2NhbFxyXG4vLy8gYW5jaG9ycyBhbmQgYSBsb2NhbCBheGlzIGhlbHBzIHdoZW4gc2F2aW5nIGFuZCBsb2FkaW5nIGEgZ2FtZS5cclxuZXhwb3J0IGNsYXNzIGIyUHJpc21hdGljSm9pbnREZWYgZXh0ZW5kcyBiMkpvaW50RGVmIGltcGxlbWVudHMgYjJJUHJpc21hdGljSm9pbnREZWYge1xyXG4gIHJlYWRvbmx5IGxvY2FsQW5jaG9yQTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICByZWFkb25seSBsb2NhbEFuY2hvckI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgcmVhZG9ubHkgbG9jYWxBeGlzQTogYjJWZWMyID0gbmV3IGIyVmVjMigxLCAwKTtcclxuXHJcbiAgcmVmZXJlbmNlQW5nbGUgPSAwO1xyXG5cclxuICBlbmFibGVMaW1pdCA9IGZhbHNlO1xyXG5cclxuICBsb3dlclRyYW5zbGF0aW9uID0gMDtcclxuXHJcbiAgdXBwZXJUcmFuc2xhdGlvbiA9IDA7XHJcblxyXG4gIGVuYWJsZU1vdG9yID0gZmFsc2U7XHJcblxyXG4gIG1heE1vdG9yRm9yY2UgPSAwO1xyXG5cclxuICBtb3RvclNwZWVkID0gMDtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICBzdXBlcihiMkpvaW50VHlwZS5lX3ByaXNtYXRpY0pvaW50KTtcclxuICB9XHJcblxyXG4gIEluaXRpYWxpemUoYkE6IGIyQm9keSwgYkI6IGIyQm9keSwgYW5jaG9yOiBiMlZlYzIsIGF4aXM6IGIyVmVjMik6IHZvaWQge1xyXG4gICAgdGhpcy5ib2R5QSA9IGJBO1xyXG4gICAgdGhpcy5ib2R5QiA9IGJCO1xyXG4gICAgdGhpcy5ib2R5QS5HZXRMb2NhbFBvaW50KGFuY2hvciwgdGhpcy5sb2NhbEFuY2hvckEpO1xyXG4gICAgdGhpcy5ib2R5Qi5HZXRMb2NhbFBvaW50KGFuY2hvciwgdGhpcy5sb2NhbEFuY2hvckIpO1xyXG4gICAgdGhpcy5ib2R5QS5HZXRMb2NhbFZlY3RvcihheGlzLCB0aGlzLmxvY2FsQXhpc0EpO1xyXG4gICAgdGhpcy5yZWZlcmVuY2VBbmdsZSA9IHRoaXMuYm9keUIuR2V0QW5nbGUoKSAtIHRoaXMuYm9keUEuR2V0QW5nbGUoKTtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMlByaXNtYXRpY0pvaW50IGV4dGVuZHMgYjJKb2ludCB7XHJcbiAgLy8gU29sdmVyIHNoYXJlZFxyXG4gIHJlYWRvbmx5IG1fbG9jYWxBbmNob3JBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbEFuY2hvckI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xvY2FsWEF4aXNBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbFlBeGlzQTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIG1fcmVmZXJlbmNlQW5nbGUgPSAwO1xyXG4gIHJlYWRvbmx5IG1faW1wdWxzZTogYjJWZWMzID0gbmV3IGIyVmVjMygwLCAwLCAwKTtcclxuICBtX21vdG9ySW1wdWxzZSA9IDA7XHJcbiAgbV9sb3dlclRyYW5zbGF0aW9uID0gMDtcclxuICBtX3VwcGVyVHJhbnNsYXRpb24gPSAwO1xyXG4gIG1fbWF4TW90b3JGb3JjZSA9IDA7XHJcbiAgbV9tb3RvclNwZWVkID0gMDtcclxuICBtX2VuYWJsZUxpbWl0ID0gZmFsc2U7XHJcbiAgbV9lbmFibGVNb3RvciA9IGZhbHNlO1xyXG4gIG1fbGltaXRTdGF0ZTogYjJMaW1pdFN0YXRlID0gYjJMaW1pdFN0YXRlLmVfaW5hY3RpdmVMaW1pdDtcclxuXHJcbiAgLy8gU29sdmVyIHRlbXBcclxuICBtX2luZGV4QSA9IDA7XHJcbiAgbV9pbmRleEIgPSAwO1xyXG4gIHJlYWRvbmx5IG1fbG9jYWxDZW50ZXJBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbENlbnRlckI6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBtX2ludk1hc3NBID0gMDtcclxuICBtX2ludk1hc3NCID0gMDtcclxuICBtX2ludklBID0gMDtcclxuICBtX2ludklCID0gMDtcclxuICByZWFkb25seSBtX2F4aXM6IGIyVmVjMiA9IG5ldyBiMlZlYzIoMCwgMCk7XHJcbiAgcmVhZG9ubHkgbV9wZXJwOiBiMlZlYzIgPSBuZXcgYjJWZWMyKDAsIDApO1xyXG4gIG1fczEgPSAwO1xyXG4gIG1fczIgPSAwO1xyXG4gIG1fYTEgPSAwO1xyXG4gIG1fYTIgPSAwO1xyXG4gIHJlYWRvbmx5IG1fSzogYjJNYXQzMyA9IG5ldyBiMk1hdDMzKCk7XHJcbiAgcmVhZG9ubHkgbV9LMzogYjJNYXQzMyA9IG5ldyBiMk1hdDMzKCk7XHJcbiAgcmVhZG9ubHkgbV9LMjogYjJNYXQyMiA9IG5ldyBiMk1hdDIyKCk7XHJcbiAgbV9tb3Rvck1hc3MgPSAwO1xyXG5cclxuICByZWFkb25seSBtX3FBOiBiMlJvdCA9IG5ldyBiMlJvdCgpO1xyXG4gIHJlYWRvbmx5IG1fcUI6IGIyUm90ID0gbmV3IGIyUm90KCk7XHJcbiAgcmVhZG9ubHkgbV9sYWxjQTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fbGFsY0I6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX3JBOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9yQjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBjb25zdHJ1Y3RvcihkZWY6IGIySVByaXNtYXRpY0pvaW50RGVmKSB7XHJcbiAgICBzdXBlcihkZWYpO1xyXG5cclxuICAgIHRoaXMubV9sb2NhbEFuY2hvckEuQ29weShiMk1heWJlKGRlZi5sb2NhbEFuY2hvckEsIGIyVmVjMi5aRVJPKSk7XHJcbiAgICB0aGlzLm1fbG9jYWxBbmNob3JCLkNvcHkoYjJNYXliZShkZWYubG9jYWxBbmNob3JCLCBiMlZlYzIuWkVSTykpO1xyXG4gICAgdGhpcy5tX2xvY2FsWEF4aXNBLkNvcHkoYjJNYXliZShkZWYubG9jYWxBeGlzQSwgbmV3IGIyVmVjMigxLCAwKSkpLlNlbGZOb3JtYWxpemUoKTtcclxuICAgIGIyVmVjMi5Dcm9zc09uZVYodGhpcy5tX2xvY2FsWEF4aXNBLCB0aGlzLm1fbG9jYWxZQXhpc0EpO1xyXG4gICAgdGhpcy5tX3JlZmVyZW5jZUFuZ2xlID0gYjJNYXliZShkZWYucmVmZXJlbmNlQW5nbGUsIDApO1xyXG4gICAgdGhpcy5tX2xvd2VyVHJhbnNsYXRpb24gPSBiMk1heWJlKGRlZi5sb3dlclRyYW5zbGF0aW9uLCAwKTtcclxuICAgIHRoaXMubV91cHBlclRyYW5zbGF0aW9uID0gYjJNYXliZShkZWYudXBwZXJUcmFuc2xhdGlvbiwgMCk7XHJcbiAgICB0aGlzLm1fbWF4TW90b3JGb3JjZSA9IGIyTWF5YmUoZGVmLm1heE1vdG9yRm9yY2UsIDApO1xyXG4gICAgdGhpcy5tX21vdG9yU3BlZWQgPSBiMk1heWJlKGRlZi5tb3RvclNwZWVkLCAwKTtcclxuICAgIHRoaXMubV9lbmFibGVMaW1pdCA9IGIyTWF5YmUoZGVmLmVuYWJsZUxpbWl0LCBmYWxzZSk7XHJcbiAgICB0aGlzLm1fZW5hYmxlTW90b3IgPSBiMk1heWJlKGRlZi5lbmFibGVNb3RvciwgZmFsc2UpO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgSW5pdFZlbG9jaXR5Q29uc3RyYWludHNfc19kID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIEluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgSW5pdFZlbG9jaXR5Q29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogdm9pZCB7XHJcbiAgICB0aGlzLm1faW5kZXhBID0gdGhpcy5tX2JvZHlBLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1faW5kZXhCID0gdGhpcy5tX2JvZHlCLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1fbG9jYWxDZW50ZXJBLkNvcHkodGhpcy5tX2JvZHlBLm1fc3dlZXAubG9jYWxDZW50ZXIpO1xyXG4gICAgdGhpcy5tX2xvY2FsQ2VudGVyQi5Db3B5KHRoaXMubV9ib2R5Qi5tX3N3ZWVwLmxvY2FsQ2VudGVyKTtcclxuICAgIHRoaXMubV9pbnZNYXNzQSA9IHRoaXMubV9ib2R5QS5tX2ludk1hc3M7XHJcbiAgICB0aGlzLm1faW52TWFzc0IgPSB0aGlzLm1fYm9keUIubV9pbnZNYXNzO1xyXG4gICAgdGhpcy5tX2ludklBID0gdGhpcy5tX2JvZHlBLm1faW52STtcclxuICAgIHRoaXMubV9pbnZJQiA9IHRoaXMubV9ib2R5Qi5tX2ludkk7XHJcblxyXG4gICAgY29uc3QgY0E6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmM7XHJcbiAgICBjb25zdCBhQTogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYTtcclxuICAgIGNvbnN0IHZBOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udjtcclxuICAgIGxldCB3QTogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnc7XHJcblxyXG4gICAgY29uc3QgY0I6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmM7XHJcbiAgICBjb25zdCBhQjogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Ql0uYTtcclxuICAgIGNvbnN0IHZCOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udjtcclxuICAgIGxldCB3QjogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnc7XHJcblxyXG4gICAgY29uc3QgcUE6IGIyUm90ID0gdGhpcy5tX3FBLlNldEFuZ2xlKGFBKSxcclxuICAgICAgcUI6IGIyUm90ID0gdGhpcy5tX3FCLlNldEFuZ2xlKGFCKTtcclxuXHJcbiAgICAvLyBDb21wdXRlIHRoZSBlZmZlY3RpdmUgbWFzc2VzLlxyXG4gICAgLy8gYjJWZWMyIHJBID0gYjJNdWwocUEsIG1fbG9jYWxBbmNob3JBIC0gbV9sb2NhbENlbnRlckEpO1xyXG4gICAgYjJWZWMyLlN1YlZWKHRoaXMubV9sb2NhbEFuY2hvckEsIHRoaXMubV9sb2NhbENlbnRlckEsIHRoaXMubV9sYWxjQSk7XHJcbiAgICBjb25zdCByQTogYjJWZWMyID0gYjJSb3QuTXVsUlYocUEsIHRoaXMubV9sYWxjQSwgdGhpcy5tX3JBKTtcclxuICAgIC8vIGIyVmVjMiByQiA9IGIyTXVsKHFCLCBtX2xvY2FsQW5jaG9yQiAtIG1fbG9jYWxDZW50ZXJCKTtcclxuICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JCLCB0aGlzLm1fbG9jYWxDZW50ZXJCLCB0aGlzLm1fbGFsY0IpO1xyXG4gICAgY29uc3QgckI6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFCLCB0aGlzLm1fbGFsY0IsIHRoaXMubV9yQik7XHJcbiAgICAvLyBiMlZlYzIgZCA9IChjQiAtIGNBKSArIHJCIC0gckE7XHJcbiAgICBjb25zdCBkOiBiMlZlYzIgPSBiMlZlYzIuQWRkVlYoXHJcbiAgICAgIGIyVmVjMi5TdWJWVihjQiwgY0EsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgYjJWZWMyLlN1YlZWKHJCLCByQSwgYjJWZWMyLnNfdDEpLFxyXG4gICAgICBiMlByaXNtYXRpY0pvaW50LkluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfZCxcclxuICAgICk7XHJcblxyXG4gICAgY29uc3QgbUE6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQSxcclxuICAgICAgbUI6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQjtcclxuICAgIGNvbnN0IGlBOiBudW1iZXIgPSB0aGlzLm1faW52SUEsXHJcbiAgICAgIGlCOiBudW1iZXIgPSB0aGlzLm1faW52SUI7XHJcblxyXG4gICAgLy8gQ29tcHV0ZSBtb3RvciBKYWNvYmlhbiBhbmQgZWZmZWN0aXZlIG1hc3MuXHJcbiAgICB7XHJcbiAgICAgIC8vIG1fYXhpcyA9IGIyTXVsKHFBLCBtX2xvY2FsWEF4aXNBKTtcclxuICAgICAgYjJSb3QuTXVsUlYocUEsIHRoaXMubV9sb2NhbFhBeGlzQSwgdGhpcy5tX2F4aXMpO1xyXG4gICAgICAvLyBtX2ExID0gYjJDcm9zcyhkICsgckEsIG1fYXhpcyk7XHJcbiAgICAgIHRoaXMubV9hMSA9IGIyVmVjMi5Dcm9zc1ZWKGIyVmVjMi5BZGRWVihkLCByQSwgYjJWZWMyLnNfdDApLCB0aGlzLm1fYXhpcyk7XHJcbiAgICAgIC8vIG1fYTIgPSBiMkNyb3NzKHJCLCBtX2F4aXMpO1xyXG4gICAgICB0aGlzLm1fYTIgPSBiMlZlYzIuQ3Jvc3NWVihyQiwgdGhpcy5tX2F4aXMpO1xyXG5cclxuICAgICAgdGhpcy5tX21vdG9yTWFzcyA9IG1BICsgbUIgKyBpQSAqIHRoaXMubV9hMSAqIHRoaXMubV9hMSArIGlCICogdGhpcy5tX2EyICogdGhpcy5tX2EyO1xyXG4gICAgICBpZiAodGhpcy5tX21vdG9yTWFzcyA+IDApIHtcclxuICAgICAgICB0aGlzLm1fbW90b3JNYXNzID0gMSAvIHRoaXMubV9tb3Rvck1hc3M7XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICAvLyBQcmlzbWF0aWMgY29uc3RyYWludC5cclxuICAgIHtcclxuICAgICAgLy8gbV9wZXJwID0gYjJNdWwocUEsIG1fbG9jYWxZQXhpc0EpO1xyXG4gICAgICBiMlJvdC5NdWxSVihxQSwgdGhpcy5tX2xvY2FsWUF4aXNBLCB0aGlzLm1fcGVycCk7XHJcblxyXG4gICAgICAvLyBtX3MxID0gYjJDcm9zcyhkICsgckEsIG1fcGVycCk7XHJcbiAgICAgIHRoaXMubV9zMSA9IGIyVmVjMi5Dcm9zc1ZWKGIyVmVjMi5BZGRWVihkLCByQSwgYjJWZWMyLnNfdDApLCB0aGlzLm1fcGVycCk7XHJcbiAgICAgIC8vIG1fczIgPSBiMkNyb3NzKHJCLCBtX3BlcnApO1xyXG4gICAgICB0aGlzLm1fczIgPSBiMlZlYzIuQ3Jvc3NWVihyQiwgdGhpcy5tX3BlcnApO1xyXG5cclxuICAgICAgLy8gZmxvYXQzMiBrMTEgPSBtQSArIG1CICsgaUEgKiBtX3MxICogbV9zMSArIGlCICogbV9zMiAqIG1fczI7XHJcbiAgICAgIHRoaXMubV9LLmV4LnggPSBtQSArIG1CICsgaUEgKiB0aGlzLm1fczEgKiB0aGlzLm1fczEgKyBpQiAqIHRoaXMubV9zMiAqIHRoaXMubV9zMjtcclxuICAgICAgLy8gZmxvYXQzMiBrMTIgPSBpQSAqIG1fczEgKyBpQiAqIG1fczI7XHJcbiAgICAgIHRoaXMubV9LLmV4LnkgPSBpQSAqIHRoaXMubV9zMSArIGlCICogdGhpcy5tX3MyO1xyXG4gICAgICAvLyBmbG9hdDMyIGsxMyA9IGlBICogbV9zMSAqIG1fYTEgKyBpQiAqIG1fczIgKiBtX2EyO1xyXG4gICAgICB0aGlzLm1fSy5leC56ID0gaUEgKiB0aGlzLm1fczEgKiB0aGlzLm1fYTEgKyBpQiAqIHRoaXMubV9zMiAqIHRoaXMubV9hMjtcclxuICAgICAgdGhpcy5tX0suZXkueCA9IHRoaXMubV9LLmV4Lnk7XHJcbiAgICAgIC8vIGZsb2F0MzIgazIyID0gaUEgKyBpQjtcclxuICAgICAgdGhpcy5tX0suZXkueSA9IGlBICsgaUI7XHJcbiAgICAgIGlmICh0aGlzLm1fSy5leS55ID09PSAwKSB7XHJcbiAgICAgICAgLy8gRm9yIGJvZGllcyB3aXRoIGZpeGVkIHJvdGF0aW9uLlxyXG4gICAgICAgIHRoaXMubV9LLmV5LnkgPSAxO1xyXG4gICAgICB9XHJcbiAgICAgIC8vIGZsb2F0MzIgazIzID0gaUEgKiBtX2ExICsgaUIgKiBtX2EyO1xyXG4gICAgICB0aGlzLm1fSy5leS56ID0gaUEgKiB0aGlzLm1fYTEgKyBpQiAqIHRoaXMubV9hMjtcclxuICAgICAgdGhpcy5tX0suZXoueCA9IHRoaXMubV9LLmV4Lno7XHJcbiAgICAgIHRoaXMubV9LLmV6LnkgPSB0aGlzLm1fSy5leS56O1xyXG4gICAgICAvLyBmbG9hdDMyIGszMyA9IG1BICsgbUIgKyBpQSAqIG1fYTEgKiBtX2ExICsgaUIgKiBtX2EyICogbV9hMjtcclxuICAgICAgdGhpcy5tX0suZXoueiA9IG1BICsgbUIgKyBpQSAqIHRoaXMubV9hMSAqIHRoaXMubV9hMSArIGlCICogdGhpcy5tX2EyICogdGhpcy5tX2EyO1xyXG5cclxuICAgICAgLy8gbV9LLmV4LlNldChrMTEsIGsxMiwgazEzKTtcclxuICAgICAgLy8gbV9LLmV5LlNldChrMTIsIGsyMiwgazIzKTtcclxuICAgICAgLy8gbV9LLmV6LlNldChrMTMsIGsyMywgazMzKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBDb21wdXRlIG1vdG9yIGFuZCBsaW1pdCB0ZXJtcy5cclxuICAgIGlmICh0aGlzLm1fZW5hYmxlTGltaXQpIHtcclxuICAgICAgLy8gZmxvYXQzMiBqb2ludFRyYW5zbGF0aW9uID0gYjJEb3QobV9heGlzLCBkKTtcclxuICAgICAgY29uc3Qgam9pbnRUcmFuc2xhdGlvbjogbnVtYmVyID0gYjJWZWMyLkRvdFZWKHRoaXMubV9heGlzLCBkKTtcclxuICAgICAgaWYgKGIyQWJzKHRoaXMubV91cHBlclRyYW5zbGF0aW9uIC0gdGhpcy5tX2xvd2VyVHJhbnNsYXRpb24pIDwgMiAqIGIyX2xpbmVhclNsb3ApIHtcclxuICAgICAgICB0aGlzLm1fbGltaXRTdGF0ZSA9IGIyTGltaXRTdGF0ZS5lX2VxdWFsTGltaXRzO1xyXG4gICAgICB9IGVsc2UgaWYgKGpvaW50VHJhbnNsYXRpb24gPD0gdGhpcy5tX2xvd2VyVHJhbnNsYXRpb24pIHtcclxuICAgICAgICBpZiAodGhpcy5tX2xpbWl0U3RhdGUgIT09IGIyTGltaXRTdGF0ZS5lX2F0TG93ZXJMaW1pdCkge1xyXG4gICAgICAgICAgdGhpcy5tX2xpbWl0U3RhdGUgPSBiMkxpbWl0U3RhdGUuZV9hdExvd2VyTGltaXQ7XHJcbiAgICAgICAgICB0aGlzLm1faW1wdWxzZS56ID0gMDtcclxuICAgICAgICB9XHJcbiAgICAgIH0gZWxzZSBpZiAoam9pbnRUcmFuc2xhdGlvbiA+PSB0aGlzLm1fdXBwZXJUcmFuc2xhdGlvbikge1xyXG4gICAgICAgIGlmICh0aGlzLm1fbGltaXRTdGF0ZSAhPT0gYjJMaW1pdFN0YXRlLmVfYXRVcHBlckxpbWl0KSB7XHJcbiAgICAgICAgICB0aGlzLm1fbGltaXRTdGF0ZSA9IGIyTGltaXRTdGF0ZS5lX2F0VXBwZXJMaW1pdDtcclxuICAgICAgICAgIHRoaXMubV9pbXB1bHNlLnogPSAwO1xyXG4gICAgICAgIH1cclxuICAgICAgfSBlbHNlIHtcclxuICAgICAgICB0aGlzLm1fbGltaXRTdGF0ZSA9IGIyTGltaXRTdGF0ZS5lX2luYWN0aXZlTGltaXQ7XHJcbiAgICAgICAgdGhpcy5tX2ltcHVsc2UueiA9IDA7XHJcbiAgICAgIH1cclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV9saW1pdFN0YXRlID0gYjJMaW1pdFN0YXRlLmVfaW5hY3RpdmVMaW1pdDtcclxuICAgICAgdGhpcy5tX2ltcHVsc2UueiA9IDA7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKCF0aGlzLm1fZW5hYmxlTW90b3IpIHtcclxuICAgICAgdGhpcy5tX21vdG9ySW1wdWxzZSA9IDA7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGRhdGEuc3RlcC53YXJtU3RhcnRpbmcpIHtcclxuICAgICAgLy8gQWNjb3VudCBmb3IgdmFyaWFibGUgdGltZSBzdGVwLlxyXG4gICAgICAvLyBtX2ltcHVsc2UgKj0gZGF0YS5zdGVwLmR0UmF0aW87XHJcbiAgICAgIHRoaXMubV9pbXB1bHNlLlNlbGZNdWwoZGF0YS5zdGVwLmR0UmF0aW8pO1xyXG4gICAgICB0aGlzLm1fbW90b3JJbXB1bHNlICo9IGRhdGEuc3RlcC5kdFJhdGlvO1xyXG5cclxuICAgICAgLy8gYjJWZWMyIFAgPSBtX2ltcHVsc2UueCAqIG1fcGVycCArIChtX21vdG9ySW1wdWxzZSArIG1faW1wdWxzZS56KSAqIG1fYXhpcztcclxuICAgICAgY29uc3QgUDogYjJWZWMyID0gYjJWZWMyLkFkZFZWKFxyXG4gICAgICAgIGIyVmVjMi5NdWxTVih0aGlzLm1faW1wdWxzZS54LCB0aGlzLm1fcGVycCwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgIGIyVmVjMi5NdWxTVih0aGlzLm1fbW90b3JJbXB1bHNlICsgdGhpcy5tX2ltcHVsc2UueiwgdGhpcy5tX2F4aXMsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICBiMlByaXNtYXRpY0pvaW50LkluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUCxcclxuICAgICAgKTtcclxuICAgICAgLy8gZmxvYXQzMiBMQSA9IG1faW1wdWxzZS54ICogbV9zMSArIG1faW1wdWxzZS55ICsgKG1fbW90b3JJbXB1bHNlICsgbV9pbXB1bHNlLnopICogbV9hMTtcclxuICAgICAgY29uc3QgTEEgPVxyXG4gICAgICAgIHRoaXMubV9pbXB1bHNlLnggKiB0aGlzLm1fczEgK1xyXG4gICAgICAgIHRoaXMubV9pbXB1bHNlLnkgK1xyXG4gICAgICAgICh0aGlzLm1fbW90b3JJbXB1bHNlICsgdGhpcy5tX2ltcHVsc2UueikgKiB0aGlzLm1fYTE7XHJcbiAgICAgIC8vIGZsb2F0MzIgTEIgPSBtX2ltcHVsc2UueCAqIG1fczIgKyBtX2ltcHVsc2UueSArIChtX21vdG9ySW1wdWxzZSArIG1faW1wdWxzZS56KSAqIG1fYTI7XHJcbiAgICAgIGNvbnN0IExCID1cclxuICAgICAgICB0aGlzLm1faW1wdWxzZS54ICogdGhpcy5tX3MyICtcclxuICAgICAgICB0aGlzLm1faW1wdWxzZS55ICtcclxuICAgICAgICAodGhpcy5tX21vdG9ySW1wdWxzZSArIHRoaXMubV9pbXB1bHNlLnopICogdGhpcy5tX2EyO1xyXG5cclxuICAgICAgLy8gdkEgLT0gbUEgKiBQO1xyXG4gICAgICB2QS5TZWxmTXVsU3ViKG1BLCBQKTtcclxuICAgICAgd0EgLT0gaUEgKiBMQTtcclxuXHJcbiAgICAgIC8vIHZCICs9IG1CICogUDtcclxuICAgICAgdkIuU2VsZk11bEFkZChtQiwgUCk7XHJcbiAgICAgIHdCICs9IGlCICogTEI7XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICB0aGlzLm1faW1wdWxzZS5TZXRaZXJvKCk7XHJcbiAgICAgIHRoaXMubV9tb3RvckltcHVsc2UgPSAwO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS52ID0gdkE7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udyA9IHdBO1xyXG4gICAgLy8gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnYgPSB2QjtcclxuICAgIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS53ID0gd0I7XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19QID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX2YyciA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19mMSA9IG5ldyBiMlZlYzMoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19kZjMgPSBuZXcgYjJWZWMzKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfZGYyID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZVZlbG9jaXR5Q29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogdm9pZCB7XHJcbiAgICBjb25zdCB2QTogYjJWZWMyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnY7XHJcbiAgICBsZXQgd0E6IG51bWJlciA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS53O1xyXG4gICAgY29uc3QgdkI6IGIyVmVjMiA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52O1xyXG4gICAgbGV0IHdCOiBudW1iZXIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udztcclxuXHJcbiAgICBjb25zdCBtQTogbnVtYmVyID0gdGhpcy5tX2ludk1hc3NBLFxyXG4gICAgICBtQjogbnVtYmVyID0gdGhpcy5tX2ludk1hc3NCO1xyXG4gICAgY29uc3QgaUE6IG51bWJlciA9IHRoaXMubV9pbnZJQSxcclxuICAgICAgaUI6IG51bWJlciA9IHRoaXMubV9pbnZJQjtcclxuXHJcbiAgICAvLyBTb2x2ZSBsaW5lYXIgbW90b3IgY29uc3RyYWludC5cclxuICAgIGlmICh0aGlzLm1fZW5hYmxlTW90b3IgJiYgdGhpcy5tX2xpbWl0U3RhdGUgIT09IGIyTGltaXRTdGF0ZS5lX2VxdWFsTGltaXRzKSB7XHJcbiAgICAgIC8vIGZsb2F0MzIgQ2RvdCA9IGIyRG90KG1fYXhpcywgdkIgLSB2QSkgKyBtX2EyICogd0IgLSBtX2ExICogd0E7XHJcbiAgICAgIGNvbnN0IENkb3Q6IG51bWJlciA9XHJcbiAgICAgICAgYjJWZWMyLkRvdFZWKHRoaXMubV9heGlzLCBiMlZlYzIuU3ViVlYodkIsIHZBLCBiMlZlYzIuc190MCkpICtcclxuICAgICAgICB0aGlzLm1fYTIgKiB3QiAtXHJcbiAgICAgICAgdGhpcy5tX2ExICogd0E7XHJcbiAgICAgIGxldCBpbXB1bHNlID0gdGhpcy5tX21vdG9yTWFzcyAqICh0aGlzLm1fbW90b3JTcGVlZCAtIENkb3QpO1xyXG4gICAgICBjb25zdCBvbGRJbXB1bHNlID0gdGhpcy5tX21vdG9ySW1wdWxzZTtcclxuICAgICAgY29uc3QgbWF4SW1wdWxzZSA9IGRhdGEuc3RlcC5kdCAqIHRoaXMubV9tYXhNb3RvckZvcmNlO1xyXG4gICAgICB0aGlzLm1fbW90b3JJbXB1bHNlID0gYjJDbGFtcCh0aGlzLm1fbW90b3JJbXB1bHNlICsgaW1wdWxzZSwgLW1heEltcHVsc2UsIG1heEltcHVsc2UpO1xyXG4gICAgICBpbXB1bHNlID0gdGhpcy5tX21vdG9ySW1wdWxzZSAtIG9sZEltcHVsc2U7XHJcblxyXG4gICAgICAvLyBiMlZlYzIgUCA9IGltcHVsc2UgKiBtX2F4aXM7XHJcbiAgICAgIGNvbnN0IFA6IGIyVmVjMiA9IGIyVmVjMi5NdWxTVihcclxuICAgICAgICBpbXB1bHNlLFxyXG4gICAgICAgIHRoaXMubV9heGlzLFxyXG4gICAgICAgIGIyUHJpc21hdGljSm9pbnQuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfUCxcclxuICAgICAgKTtcclxuICAgICAgY29uc3QgTEEgPSBpbXB1bHNlICogdGhpcy5tX2ExO1xyXG4gICAgICBjb25zdCBMQiA9IGltcHVsc2UgKiB0aGlzLm1fYTI7XHJcblxyXG4gICAgICAvLyB2QSAtPSBtQSAqIFA7XHJcbiAgICAgIHZBLlNlbGZNdWxTdWIobUEsIFApO1xyXG4gICAgICB3QSAtPSBpQSAqIExBO1xyXG5cclxuICAgICAgLy8gdkIgKz0gbUIgKiBQO1xyXG4gICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQKTtcclxuICAgICAgd0IgKz0gaUIgKiBMQjtcclxuICAgIH1cclxuXHJcbiAgICAvLyBiMlZlYzIgQ2RvdDE7XHJcbiAgICAvLyBDZG90MS54ID0gYjJEb3QobV9wZXJwLCB2QiAtIHZBKSArIG1fczIgKiB3QiAtIG1fczEgKiB3QTtcclxuICAgIGNvbnN0IENkb3QxX3g6IG51bWJlciA9XHJcbiAgICAgIGIyVmVjMi5Eb3RWVih0aGlzLm1fcGVycCwgYjJWZWMyLlN1YlZWKHZCLCB2QSwgYjJWZWMyLnNfdDApKSArXHJcbiAgICAgIHRoaXMubV9zMiAqIHdCIC1cclxuICAgICAgdGhpcy5tX3MxICogd0E7XHJcbiAgICAvLyBDZG90MS55ID0gd0IgLSB3QTtcclxuICAgIGNvbnN0IENkb3QxX3kgPSB3QiAtIHdBO1xyXG5cclxuICAgIGlmICh0aGlzLm1fZW5hYmxlTGltaXQgJiYgdGhpcy5tX2xpbWl0U3RhdGUgIT09IGIyTGltaXRTdGF0ZS5lX2luYWN0aXZlTGltaXQpIHtcclxuICAgICAgLy8gU29sdmUgcHJpc21hdGljIGFuZCBsaW1pdCBjb25zdHJhaW50IGluIGJsb2NrIGZvcm0uXHJcbiAgICAgIC8vIGZsb2F0MzIgQ2RvdDI7XHJcbiAgICAgIC8vIENkb3QyID0gYjJEb3QobV9heGlzLCB2QiAtIHZBKSArIG1fYTIgKiB3QiAtIG1fYTEgKiB3QTtcclxuICAgICAgY29uc3QgQ2RvdDI6IG51bWJlciA9XHJcbiAgICAgICAgYjJWZWMyLkRvdFZWKHRoaXMubV9heGlzLCBiMlZlYzIuU3ViVlYodkIsIHZBLCBiMlZlYzIuc190MCkpICtcclxuICAgICAgICB0aGlzLm1fYTIgKiB3QiAtXHJcbiAgICAgICAgdGhpcy5tX2ExICogd0E7XHJcbiAgICAgIC8vIGIyVmVjMyBDZG90KENkb3QxLngsIENkb3QxLnksIENkb3QyKTtcclxuXHJcbiAgICAgIC8vIGIyVmVjMyBmMSA9IG1faW1wdWxzZTtcclxuICAgICAgY29uc3QgZjEgPSBiMlByaXNtYXRpY0pvaW50LlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX2YxLkNvcHkodGhpcy5tX2ltcHVsc2UpO1xyXG4gICAgICAvLyBiMlZlYzMgZGYgPSAgbV9LLlNvbHZlMzMoLUNkb3QpO1xyXG4gICAgICBjb25zdCBkZjMgPSB0aGlzLm1fSy5Tb2x2ZTMzKFxyXG4gICAgICAgIC1DZG90MV94LFxyXG4gICAgICAgIC1DZG90MV95LFxyXG4gICAgICAgIC1DZG90MixcclxuICAgICAgICBiMlByaXNtYXRpY0pvaW50LlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX2RmMyxcclxuICAgICAgKTtcclxuICAgICAgLy8gbV9pbXB1bHNlICs9IGRmO1xyXG4gICAgICB0aGlzLm1faW1wdWxzZS5TZWxmQWRkKGRmMyk7XHJcblxyXG4gICAgICBpZiAodGhpcy5tX2xpbWl0U3RhdGUgPT09IGIyTGltaXRTdGF0ZS5lX2F0TG93ZXJMaW1pdCkge1xyXG4gICAgICAgIHRoaXMubV9pbXB1bHNlLnogPSBiMk1heCh0aGlzLm1faW1wdWxzZS56LCAwKTtcclxuICAgICAgfSBlbHNlIGlmICh0aGlzLm1fbGltaXRTdGF0ZSA9PT0gYjJMaW1pdFN0YXRlLmVfYXRVcHBlckxpbWl0KSB7XHJcbiAgICAgICAgdGhpcy5tX2ltcHVsc2UueiA9IGIyTWluKHRoaXMubV9pbXB1bHNlLnosIDApO1xyXG4gICAgICB9XHJcblxyXG4gICAgICAvLyBmMigxOjIpID0gaW52SygxOjIsMToyKSAqICgtQ2RvdCgxOjIpIC0gSygxOjIsMykgKiAoZjIoMykgLSBmMSgzKSkpICsgZjEoMToyKVxyXG4gICAgICAvLyBiMlZlYzIgYiA9IC1DZG90MSAtIChtX2ltcHVsc2UueiAtIGYxLnopICogYjJWZWMyKG1fSy5lei54LCBtX0suZXoueSk7XHJcbiAgICAgIGNvbnN0IGJfeCA9IC1DZG90MV94IC0gKHRoaXMubV9pbXB1bHNlLnogLSBmMS56KSAqIHRoaXMubV9LLmV6Lng7XHJcbiAgICAgIGNvbnN0IGJfeSA9IC1DZG90MV95IC0gKHRoaXMubV9pbXB1bHNlLnogLSBmMS56KSAqIHRoaXMubV9LLmV6Lnk7XHJcbiAgICAgIC8vIGIyVmVjMiBmMnIgPSBtX0suU29sdmUyMihiKSArIGIyVmVjMihmMS54LCBmMS55KTtcclxuICAgICAgY29uc3QgZjJyID0gdGhpcy5tX0suU29sdmUyMihiX3gsIGJfeSwgYjJQcmlzbWF0aWNKb2ludC5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19mMnIpO1xyXG4gICAgICBmMnIueCArPSBmMS54O1xyXG4gICAgICBmMnIueSArPSBmMS55O1xyXG4gICAgICAvLyBtX2ltcHVsc2UueCA9IGYyci54O1xyXG4gICAgICB0aGlzLm1faW1wdWxzZS54ID0gZjJyLng7XHJcbiAgICAgIC8vIG1faW1wdWxzZS55ID0gZjJyLnk7XHJcbiAgICAgIHRoaXMubV9pbXB1bHNlLnkgPSBmMnIueTtcclxuXHJcbiAgICAgIC8vIGRmID0gbV9pbXB1bHNlIC0gZjE7XHJcbiAgICAgIGRmMy54ID0gdGhpcy5tX2ltcHVsc2UueCAtIGYxLng7XHJcbiAgICAgIGRmMy55ID0gdGhpcy5tX2ltcHVsc2UueSAtIGYxLnk7XHJcbiAgICAgIGRmMy56ID0gdGhpcy5tX2ltcHVsc2UueiAtIGYxLno7XHJcblxyXG4gICAgICAvLyBiMlZlYzIgUCA9IGRmLnggKiBtX3BlcnAgKyBkZi56ICogbV9heGlzO1xyXG4gICAgICBjb25zdCBQOiBiMlZlYzIgPSBiMlZlYzIuQWRkVlYoXHJcbiAgICAgICAgYjJWZWMyLk11bFNWKGRmMy54LCB0aGlzLm1fcGVycCwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgIGIyVmVjMi5NdWxTVihkZjMueiwgdGhpcy5tX2F4aXMsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICBiMlByaXNtYXRpY0pvaW50LlNvbHZlVmVsb2NpdHlDb25zdHJhaW50c19zX1AsXHJcbiAgICAgICk7XHJcbiAgICAgIC8vIGZsb2F0MzIgTEEgPSBkZi54ICogbV9zMSArIGRmLnkgKyBkZi56ICogbV9hMTtcclxuICAgICAgY29uc3QgTEEgPSBkZjMueCAqIHRoaXMubV9zMSArIGRmMy55ICsgZGYzLnogKiB0aGlzLm1fYTE7XHJcbiAgICAgIC8vIGZsb2F0MzIgTEIgPSBkZi54ICogbV9zMiArIGRmLnkgKyBkZi56ICogbV9hMjtcclxuICAgICAgY29uc3QgTEIgPSBkZjMueCAqIHRoaXMubV9zMiArIGRmMy55ICsgZGYzLnogKiB0aGlzLm1fYTI7XHJcblxyXG4gICAgICAvLyB2QSAtPSBtQSAqIFA7XHJcbiAgICAgIHZBLlNlbGZNdWxTdWIobUEsIFApO1xyXG4gICAgICB3QSAtPSBpQSAqIExBO1xyXG5cclxuICAgICAgLy8gdkIgKz0gbUIgKiBQO1xyXG4gICAgICB2Qi5TZWxmTXVsQWRkKG1CLCBQKTtcclxuICAgICAgd0IgKz0gaUIgKiBMQjtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIC8vIExpbWl0IGlzIGluYWN0aXZlLCBqdXN0IHNvbHZlIHRoZSBwcmlzbWF0aWMgY29uc3RyYWludCBpbiBibG9jayBmb3JtLlxyXG4gICAgICAvLyBiMlZlYzIgZGYgPSBtX0suU29sdmUyMigtQ2RvdDEpO1xyXG4gICAgICBjb25zdCBkZjIgPSB0aGlzLm1fSy5Tb2x2ZTIyKFxyXG4gICAgICAgIC1DZG90MV94LFxyXG4gICAgICAgIC1DZG90MV95LFxyXG4gICAgICAgIGIyUHJpc21hdGljSm9pbnQuU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzX3NfZGYyLFxyXG4gICAgICApO1xyXG4gICAgICB0aGlzLm1faW1wdWxzZS54ICs9IGRmMi54O1xyXG4gICAgICB0aGlzLm1faW1wdWxzZS55ICs9IGRmMi55O1xyXG5cclxuICAgICAgLy8gYjJWZWMyIFAgPSBkZi54ICogbV9wZXJwO1xyXG4gICAgICBjb25zdCBQOiBiMlZlYzIgPSBiMlZlYzIuTXVsU1YoXHJcbiAgICAgICAgZGYyLngsXHJcbiAgICAgICAgdGhpcy5tX3BlcnAsXHJcbiAgICAgICAgYjJQcmlzbWF0aWNKb2ludC5Tb2x2ZVZlbG9jaXR5Q29uc3RyYWludHNfc19QLFxyXG4gICAgICApO1xyXG4gICAgICAvLyBmbG9hdDMyIExBID0gZGYueCAqIG1fczEgKyBkZi55O1xyXG4gICAgICBjb25zdCBMQSA9IGRmMi54ICogdGhpcy5tX3MxICsgZGYyLnk7XHJcbiAgICAgIC8vIGZsb2F0MzIgTEIgPSBkZi54ICogbV9zMiArIGRmLnk7XHJcbiAgICAgIGNvbnN0IExCID0gZGYyLnggKiB0aGlzLm1fczIgKyBkZjIueTtcclxuXHJcbiAgICAgIC8vIHZBIC09IG1BICogUDtcclxuICAgICAgdkEuU2VsZk11bFN1YihtQSwgUCk7XHJcbiAgICAgIHdBIC09IGlBICogTEE7XHJcblxyXG4gICAgICAvLyB2QiArPSBtQiAqIFA7XHJcbiAgICAgIHZCLlNlbGZNdWxBZGQobUIsIFApO1xyXG4gICAgICB3QiArPSBpQiAqIExCO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS52ID0gdkE7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udyA9IHdBO1xyXG4gICAgLy8gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnYgPSB2QjtcclxuICAgIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS53ID0gd0I7XHJcbiAgfVxyXG5cclxuICAvLyBBIHZlbG9jaXR5IGJhc2VkIHNvbHZlciBjb21wdXRlcyByZWFjdGlvbiBmb3JjZXMoaW1wdWxzZXMpIHVzaW5nIHRoZSB2ZWxvY2l0eSBjb25zdHJhaW50IHNvbHZlci5VbmRlciB0aGlzIGNvbnRleHQsXHJcbiAgLy8gdGhlIHBvc2l0aW9uIHNvbHZlciBpcyBub3QgdGhlcmUgdG8gcmVzb2x2ZSBmb3JjZXMuSXQgaXMgb25seSB0aGVyZSB0byBjb3BlIHdpdGggaW50ZWdyYXRpb24gZXJyb3IuXHJcbiAgLy9cclxuICAvLyBUaGVyZWZvcmUsIHRoZSBwc2V1ZG8gaW1wdWxzZXMgaW4gdGhlIHBvc2l0aW9uIHNvbHZlciBkbyBub3QgaGF2ZSBhbnkgcGh5c2ljYWwgbWVhbmluZy5UaHVzIGl0IGlzIG9rYXkgaWYgdGhleSBzdWNrLlxyXG4gIC8vXHJcbiAgLy8gV2UgY291bGQgdGFrZSB0aGUgYWN0aXZlIHN0YXRlIGZyb20gdGhlIHZlbG9jaXR5IHNvbHZlci5Ib3dldmVyLCB0aGUgam9pbnQgbWlnaHQgcHVzaCBwYXN0IHRoZSBsaW1pdCB3aGVuIHRoZSB2ZWxvY2l0eVxyXG4gIC8vIHNvbHZlciBpbmRpY2F0ZXMgdGhlIGxpbWl0IGlzIGluYWN0aXZlLlxyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX2QgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfaW1wdWxzZSA9IG5ldyBiMlZlYzMoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc19pbXB1bHNlMSA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc19QID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBTb2x2ZVBvc2l0aW9uQ29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogYm9vbGVhbiB7XHJcbiAgICBjb25zdCBjQTogYjJWZWMyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYztcclxuICAgIGxldCBhQTogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4QV0uYTtcclxuICAgIGNvbnN0IGNCOiBiMlZlYzIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5jO1xyXG4gICAgbGV0IGFCOiBudW1iZXIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5hO1xyXG5cclxuICAgIGNvbnN0IHFBOiBiMlJvdCA9IHRoaXMubV9xQS5TZXRBbmdsZShhQSksXHJcbiAgICAgIHFCOiBiMlJvdCA9IHRoaXMubV9xQi5TZXRBbmdsZShhQik7XHJcblxyXG4gICAgY29uc3QgbUE6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQSxcclxuICAgICAgbUI6IG51bWJlciA9IHRoaXMubV9pbnZNYXNzQjtcclxuICAgIGNvbnN0IGlBOiBudW1iZXIgPSB0aGlzLm1faW52SUEsXHJcbiAgICAgIGlCOiBudW1iZXIgPSB0aGlzLm1faW52SUI7XHJcblxyXG4gICAgLy8gYjJWZWMyIHJBID0gYjJNdWwocUEsIG1fbG9jYWxBbmNob3JBIC0gbV9sb2NhbENlbnRlckEpO1xyXG4gICAgY29uc3QgckE6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFBLCB0aGlzLm1fbGFsY0EsIHRoaXMubV9yQSk7XHJcbiAgICAvLyBiMlZlYzIgckIgPSBiMk11bChxQiwgbV9sb2NhbEFuY2hvckIgLSBtX2xvY2FsQ2VudGVyQik7XHJcbiAgICBjb25zdCByQjogYjJWZWMyID0gYjJSb3QuTXVsUlYocUIsIHRoaXMubV9sYWxjQiwgdGhpcy5tX3JCKTtcclxuICAgIC8vIGIyVmVjMiBkID0gY0IgKyByQiAtIGNBIC0gckE7XHJcbiAgICBjb25zdCBkOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYoXHJcbiAgICAgIGIyVmVjMi5BZGRWVihjQiwgckIsIGIyVmVjMi5zX3QwKSxcclxuICAgICAgYjJWZWMyLkFkZFZWKGNBLCByQSwgYjJWZWMyLnNfdDEpLFxyXG4gICAgICBiMlByaXNtYXRpY0pvaW50LlNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX2QsXHJcbiAgICApO1xyXG5cclxuICAgIC8vIGIyVmVjMiBheGlzID0gYjJNdWwocUEsIG1fbG9jYWxYQXhpc0EpO1xyXG4gICAgY29uc3QgYXhpczogYjJWZWMyID0gYjJSb3QuTXVsUlYocUEsIHRoaXMubV9sb2NhbFhBeGlzQSwgdGhpcy5tX2F4aXMpO1xyXG4gICAgLy8gZmxvYXQzMiBhMSA9IGIyQ3Jvc3MoZCArIHJBLCBheGlzKTtcclxuICAgIGNvbnN0IGExID0gYjJWZWMyLkNyb3NzVlYoYjJWZWMyLkFkZFZWKGQsIHJBLCBiMlZlYzIuc190MCksIGF4aXMpO1xyXG4gICAgLy8gZmxvYXQzMiBhMiA9IGIyQ3Jvc3MockIsIGF4aXMpO1xyXG4gICAgY29uc3QgYTIgPSBiMlZlYzIuQ3Jvc3NWVihyQiwgYXhpcyk7XHJcbiAgICAvLyBiMlZlYzIgcGVycCA9IGIyTXVsKHFBLCBtX2xvY2FsWUF4aXNBKTtcclxuICAgIGNvbnN0IHBlcnA6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFBLCB0aGlzLm1fbG9jYWxZQXhpc0EsIHRoaXMubV9wZXJwKTtcclxuXHJcbiAgICAvLyBmbG9hdDMyIHMxID0gYjJDcm9zcyhkICsgckEsIHBlcnApO1xyXG4gICAgY29uc3QgczEgPSBiMlZlYzIuQ3Jvc3NWVihiMlZlYzIuQWRkVlYoZCwgckEsIGIyVmVjMi5zX3QwKSwgcGVycCk7XHJcbiAgICAvLyBmbG9hdDMyIHMyID0gYjJDcm9zcyhyQiwgcGVycCk7XHJcbiAgICBjb25zdCBzMiA9IGIyVmVjMi5Dcm9zc1ZWKHJCLCBwZXJwKTtcclxuXHJcbiAgICAvLyBiMlZlYzMgaW1wdWxzZTtcclxuICAgIGxldCBpbXB1bHNlID0gYjJQcmlzbWF0aWNKb2ludC5Tb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc19pbXB1bHNlO1xyXG4gICAgLy8gYjJWZWMyIEMxO1xyXG4gICAgLy8gQzEueCA9IGIyRG90KHBlcnAsIGQpO1xyXG4gICAgY29uc3QgQzFfeDogbnVtYmVyID0gYjJWZWMyLkRvdFZWKHBlcnAsIGQpO1xyXG4gICAgLy8gQzEueSA9IGFCIC0gYUEgLSBtX3JlZmVyZW5jZUFuZ2xlO1xyXG4gICAgY29uc3QgQzFfeSA9IGFCIC0gYUEgLSB0aGlzLm1fcmVmZXJlbmNlQW5nbGU7XHJcblxyXG4gICAgbGV0IGxpbmVhckVycm9yID0gYjJBYnMoQzFfeCk7XHJcbiAgICBjb25zdCBhbmd1bGFyRXJyb3IgPSBiMkFicyhDMV95KTtcclxuXHJcbiAgICBsZXQgYWN0aXZlID0gZmFsc2U7XHJcbiAgICBsZXQgQzIgPSAwO1xyXG4gICAgaWYgKHRoaXMubV9lbmFibGVMaW1pdCkge1xyXG4gICAgICAvLyBmbG9hdDMyIHRyYW5zbGF0aW9uID0gYjJEb3QoYXhpcywgZCk7XHJcbiAgICAgIGNvbnN0IHRyYW5zbGF0aW9uOiBudW1iZXIgPSBiMlZlYzIuRG90VlYoYXhpcywgZCk7XHJcbiAgICAgIGlmIChiMkFicyh0aGlzLm1fdXBwZXJUcmFuc2xhdGlvbiAtIHRoaXMubV9sb3dlclRyYW5zbGF0aW9uKSA8IDIgKiBiMl9saW5lYXJTbG9wKSB7XHJcbiAgICAgICAgLy8gUHJldmVudCBsYXJnZSBhbmd1bGFyIGNvcnJlY3Rpb25zXHJcbiAgICAgICAgQzIgPSBiMkNsYW1wKHRyYW5zbGF0aW9uLCAtYjJfbWF4TGluZWFyQ29ycmVjdGlvbiwgYjJfbWF4TGluZWFyQ29ycmVjdGlvbik7XHJcbiAgICAgICAgbGluZWFyRXJyb3IgPSBiMk1heChsaW5lYXJFcnJvciwgYjJBYnModHJhbnNsYXRpb24pKTtcclxuICAgICAgICBhY3RpdmUgPSB0cnVlO1xyXG4gICAgICB9IGVsc2UgaWYgKHRyYW5zbGF0aW9uIDw9IHRoaXMubV9sb3dlclRyYW5zbGF0aW9uKSB7XHJcbiAgICAgICAgLy8gUHJldmVudCBsYXJnZSBsaW5lYXIgY29ycmVjdGlvbnMgYW5kIGFsbG93IHNvbWUgc2xvcC5cclxuICAgICAgICBDMiA9IGIyQ2xhbXAoXHJcbiAgICAgICAgICB0cmFuc2xhdGlvbiAtIHRoaXMubV9sb3dlclRyYW5zbGF0aW9uICsgYjJfbGluZWFyU2xvcCxcclxuICAgICAgICAgIC1iMl9tYXhMaW5lYXJDb3JyZWN0aW9uLFxyXG4gICAgICAgICAgMCxcclxuICAgICAgICApO1xyXG4gICAgICAgIGxpbmVhckVycm9yID0gYjJNYXgobGluZWFyRXJyb3IsIHRoaXMubV9sb3dlclRyYW5zbGF0aW9uIC0gdHJhbnNsYXRpb24pO1xyXG4gICAgICAgIGFjdGl2ZSA9IHRydWU7XHJcbiAgICAgIH0gZWxzZSBpZiAodHJhbnNsYXRpb24gPj0gdGhpcy5tX3VwcGVyVHJhbnNsYXRpb24pIHtcclxuICAgICAgICAvLyBQcmV2ZW50IGxhcmdlIGxpbmVhciBjb3JyZWN0aW9ucyBhbmQgYWxsb3cgc29tZSBzbG9wLlxyXG4gICAgICAgIEMyID0gYjJDbGFtcChcclxuICAgICAgICAgIHRyYW5zbGF0aW9uIC0gdGhpcy5tX3VwcGVyVHJhbnNsYXRpb24gLSBiMl9saW5lYXJTbG9wLFxyXG4gICAgICAgICAgMCxcclxuICAgICAgICAgIGIyX21heExpbmVhckNvcnJlY3Rpb24sXHJcbiAgICAgICAgKTtcclxuICAgICAgICBsaW5lYXJFcnJvciA9IGIyTWF4KGxpbmVhckVycm9yLCB0cmFuc2xhdGlvbiAtIHRoaXMubV91cHBlclRyYW5zbGF0aW9uKTtcclxuICAgICAgICBhY3RpdmUgPSB0cnVlO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGFjdGl2ZSkge1xyXG4gICAgICAvLyBmbG9hdDMyIGsxMSA9IG1BICsgbUIgKyBpQSAqIHMxICogczEgKyBpQiAqIHMyICogczI7XHJcbiAgICAgIGNvbnN0IGsxMSA9IG1BICsgbUIgKyBpQSAqIHMxICogczEgKyBpQiAqIHMyICogczI7XHJcbiAgICAgIC8vIGZsb2F0MzIgazEyID0gaUEgKiBzMSArIGlCICogczI7XHJcbiAgICAgIGNvbnN0IGsxMiA9IGlBICogczEgKyBpQiAqIHMyO1xyXG4gICAgICAvLyBmbG9hdDMyIGsxMyA9IGlBICogczEgKiBhMSArIGlCICogczIgKiBhMjtcclxuICAgICAgY29uc3QgazEzID0gaUEgKiBzMSAqIGExICsgaUIgKiBzMiAqIGEyO1xyXG4gICAgICAvLyBmbG9hdDMyIGsyMiA9IGlBICsgaUI7XHJcbiAgICAgIGxldCBrMjIgPSBpQSArIGlCO1xyXG4gICAgICBpZiAoazIyID09PSAwKSB7XHJcbiAgICAgICAgLy8gRm9yIGZpeGVkIHJvdGF0aW9uXHJcbiAgICAgICAgazIyID0gMTtcclxuICAgICAgfVxyXG4gICAgICAvLyBmbG9hdDMyIGsyMyA9IGlBICogYTEgKyBpQiAqIGEyO1xyXG4gICAgICBjb25zdCBrMjMgPSBpQSAqIGExICsgaUIgKiBhMjtcclxuICAgICAgLy8gZmxvYXQzMiBrMzMgPSBtQSArIG1CICsgaUEgKiBhMSAqIGExICsgaUIgKiBhMiAqIGEyO1xyXG4gICAgICBjb25zdCBrMzMgPSBtQSArIG1CICsgaUEgKiBhMSAqIGExICsgaUIgKiBhMiAqIGEyO1xyXG5cclxuICAgICAgLy8gYjJNYXQzMyBLO1xyXG4gICAgICBjb25zdCBLID0gdGhpcy5tX0szO1xyXG4gICAgICAvLyBLLmV4LlNldChrMTEsIGsxMiwgazEzKTtcclxuICAgICAgSy5leC5TZXRYWVooazExLCBrMTIsIGsxMyk7XHJcbiAgICAgIC8vIEsuZXkuU2V0KGsxMiwgazIyLCBrMjMpO1xyXG4gICAgICBLLmV5LlNldFhZWihrMTIsIGsyMiwgazIzKTtcclxuICAgICAgLy8gSy5lei5TZXQoazEzLCBrMjMsIGszMyk7XHJcbiAgICAgIEsuZXouU2V0WFlaKGsxMywgazIzLCBrMzMpO1xyXG5cclxuICAgICAgLy8gYjJWZWMzIEM7XHJcbiAgICAgIC8vIEMueCA9IEMxLng7XHJcbiAgICAgIC8vIEMueSA9IEMxLnk7XHJcbiAgICAgIC8vIEMueiA9IEMyO1xyXG5cclxuICAgICAgLy8gaW1wdWxzZSA9IEsuU29sdmUzMygtQyk7XHJcbiAgICAgIGltcHVsc2UgPSBLLlNvbHZlMzMoLUMxX3gsIC1DMV95LCAtQzIsIGltcHVsc2UpO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgLy8gZmxvYXQzMiBrMTEgPSBtQSArIG1CICsgaUEgKiBzMSAqIHMxICsgaUIgKiBzMiAqIHMyO1xyXG4gICAgICBjb25zdCBrMTEgPSBtQSArIG1CICsgaUEgKiBzMSAqIHMxICsgaUIgKiBzMiAqIHMyO1xyXG4gICAgICAvLyBmbG9hdDMyIGsxMiA9IGlBICogczEgKyBpQiAqIHMyO1xyXG4gICAgICBjb25zdCBrMTIgPSBpQSAqIHMxICsgaUIgKiBzMjtcclxuICAgICAgLy8gZmxvYXQzMiBrMjIgPSBpQSArIGlCO1xyXG4gICAgICBsZXQgazIyID0gaUEgKyBpQjtcclxuICAgICAgaWYgKGsyMiA9PT0gMCkge1xyXG4gICAgICAgIGsyMiA9IDE7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIGIyTWF0MjIgSztcclxuICAgICAgY29uc3QgSzIgPSB0aGlzLm1fSzI7XHJcbiAgICAgIC8vIEsuZXguU2V0KGsxMSwgazEyKTtcclxuICAgICAgSzIuZXguU2V0KGsxMSwgazEyKTtcclxuICAgICAgLy8gSy5leS5TZXQoazEyLCBrMjIpO1xyXG4gICAgICBLMi5leS5TZXQoazEyLCBrMjIpO1xyXG5cclxuICAgICAgLy8gYjJWZWMyIGltcHVsc2UxID0gSy5Tb2x2ZSgtQzEpO1xyXG4gICAgICBjb25zdCBpbXB1bHNlMSA9IEsyLlNvbHZlKC1DMV94LCAtQzFfeSwgYjJQcmlzbWF0aWNKb2ludC5Tb2x2ZVBvc2l0aW9uQ29uc3RyYWludHNfc19pbXB1bHNlMSk7XHJcbiAgICAgIGltcHVsc2UueCA9IGltcHVsc2UxLng7XHJcbiAgICAgIGltcHVsc2UueSA9IGltcHVsc2UxLnk7XHJcbiAgICAgIGltcHVsc2UueiA9IDA7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gYjJWZWMyIFAgPSBpbXB1bHNlLnggKiBwZXJwICsgaW1wdWxzZS56ICogYXhpcztcclxuICAgIGNvbnN0IFA6IGIyVmVjMiA9IGIyVmVjMi5BZGRWVihcclxuICAgICAgYjJWZWMyLk11bFNWKGltcHVsc2UueCwgcGVycCwgYjJWZWMyLnNfdDApLFxyXG4gICAgICBiMlZlYzIuTXVsU1YoaW1wdWxzZS56LCBheGlzLCBiMlZlYzIuc190MSksXHJcbiAgICAgIGIyUHJpc21hdGljSm9pbnQuU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfUCxcclxuICAgICk7XHJcbiAgICAvLyBmbG9hdDMyIExBID0gaW1wdWxzZS54ICogczEgKyBpbXB1bHNlLnkgKyBpbXB1bHNlLnogKiBhMTtcclxuICAgIGNvbnN0IExBID0gaW1wdWxzZS54ICogczEgKyBpbXB1bHNlLnkgKyBpbXB1bHNlLnogKiBhMTtcclxuICAgIC8vIGZsb2F0MzIgTEIgPSBpbXB1bHNlLnggKiBzMiArIGltcHVsc2UueSArIGltcHVsc2UueiAqIGEyO1xyXG4gICAgY29uc3QgTEIgPSBpbXB1bHNlLnggKiBzMiArIGltcHVsc2UueSArIGltcHVsc2UueiAqIGEyO1xyXG5cclxuICAgIC8vIGNBIC09IG1BICogUDtcclxuICAgIGNBLlNlbGZNdWxTdWIobUEsIFApO1xyXG4gICAgYUEgLT0gaUEgKiBMQTtcclxuICAgIC8vIGNCICs9IG1CICogUDtcclxuICAgIGNCLlNlbGZNdWxBZGQobUIsIFApO1xyXG4gICAgYUIgKz0gaUIgKiBMQjtcclxuXHJcbiAgICAvLyBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5jID0gY0E7XHJcbiAgICBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5hID0gYUE7XHJcbiAgICAvLyBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5jID0gY0I7XHJcbiAgICBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5hID0gYUI7XHJcblxyXG4gICAgcmV0dXJuIGxpbmVhckVycm9yIDw9IGIyX2xpbmVhclNsb3AgJiYgYW5ndWxhckVycm9yIDw9IGIyX2FuZ3VsYXJTbG9wO1xyXG4gIH1cclxuXHJcbiAgR2V0QW5jaG9yQTxUIGV4dGVuZHMgWFk+KG91dDogVCk6IFQge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ib2R5QS5HZXRXb3JsZFBvaW50KHRoaXMubV9sb2NhbEFuY2hvckEsIG91dCk7XHJcbiAgfVxyXG5cclxuICBHZXRBbmNob3JCPFQgZXh0ZW5kcyBYWT4ob3V0OiBUKTogVCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2JvZHlCLkdldFdvcmxkUG9pbnQodGhpcy5tX2xvY2FsQW5jaG9yQiwgb3V0KTtcclxuICB9XHJcblxyXG4gIEdldFJlYWN0aW9uRm9yY2U8VCBleHRlbmRzIFhZPihpbnZfZHQ6IG51bWJlciwgb3V0OiBUKTogVCB7XHJcbiAgICAvLyByZXR1cm4gaW52X2R0ICogKG1faW1wdWxzZS54ICogbV9wZXJwICsgKG1fbW90b3JJbXB1bHNlICsgbV9pbXB1bHNlLnopICogbV9heGlzKTtcclxuICAgIG91dC54ID1cclxuICAgICAgaW52X2R0ICpcclxuICAgICAgKHRoaXMubV9pbXB1bHNlLnggKiB0aGlzLm1fcGVycC54ICsgKHRoaXMubV9tb3RvckltcHVsc2UgKyB0aGlzLm1faW1wdWxzZS56KSAqIHRoaXMubV9heGlzLngpO1xyXG4gICAgb3V0LnkgPVxyXG4gICAgICBpbnZfZHQgKlxyXG4gICAgICAodGhpcy5tX2ltcHVsc2UueCAqIHRoaXMubV9wZXJwLnkgKyAodGhpcy5tX21vdG9ySW1wdWxzZSArIHRoaXMubV9pbXB1bHNlLnopICogdGhpcy5tX2F4aXMueSk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgR2V0UmVhY3Rpb25Ub3JxdWUoaW52X2R0OiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIGludl9kdCAqIHRoaXMubV9pbXB1bHNlLnk7XHJcbiAgfVxyXG5cclxuICBHZXRMb2NhbEFuY2hvckEoKTogUmVhZG9ubHk8YjJWZWMyPiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2xvY2FsQW5jaG9yQTtcclxuICB9XHJcblxyXG4gIEdldExvY2FsQW5jaG9yQigpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHJldHVybiB0aGlzLm1fbG9jYWxBbmNob3JCO1xyXG4gIH1cclxuXHJcbiAgR2V0TG9jYWxBeGlzQSgpOiBSZWFkb25seTxiMlZlYzI+IHtcclxuICAgIHJldHVybiB0aGlzLm1fbG9jYWxYQXhpc0E7XHJcbiAgfVxyXG5cclxuICBHZXRSZWZlcmVuY2VBbmdsZSgpIHtcclxuICAgIHJldHVybiB0aGlzLm1fcmVmZXJlbmNlQW5nbGU7XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBHZXRKb2ludFRyYW5zbGF0aW9uX3NfcEEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgR2V0Sm9pbnRUcmFuc2xhdGlvbl9zX3BCID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIEdldEpvaW50VHJhbnNsYXRpb25fc19kID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIEdldEpvaW50VHJhbnNsYXRpb25fc19heGlzID0gbmV3IGIyVmVjMigpO1xyXG5cclxuICBHZXRKb2ludFRyYW5zbGF0aW9uKCk6IG51bWJlciB7XHJcbiAgICAvLyBiMlZlYzIgcEEgPSBtX2JvZHlBLkdldFdvcmxkUG9pbnQobV9sb2NhbEFuY2hvckEpO1xyXG4gICAgY29uc3QgcEEgPSB0aGlzLm1fYm9keUEuR2V0V29ybGRQb2ludChcclxuICAgICAgdGhpcy5tX2xvY2FsQW5jaG9yQSxcclxuICAgICAgYjJQcmlzbWF0aWNKb2ludC5HZXRKb2ludFRyYW5zbGF0aW9uX3NfcEEsXHJcbiAgICApO1xyXG4gICAgLy8gYjJWZWMyIHBCID0gbV9ib2R5Qi5HZXRXb3JsZFBvaW50KG1fbG9jYWxBbmNob3JCKTtcclxuICAgIGNvbnN0IHBCID0gdGhpcy5tX2JvZHlCLkdldFdvcmxkUG9pbnQoXHJcbiAgICAgIHRoaXMubV9sb2NhbEFuY2hvckIsXHJcbiAgICAgIGIyUHJpc21hdGljSm9pbnQuR2V0Sm9pbnRUcmFuc2xhdGlvbl9zX3BCLFxyXG4gICAgKTtcclxuICAgIC8vIGIyVmVjMiBkID0gcEIgLSBwQTtcclxuICAgIGNvbnN0IGQ6IGIyVmVjMiA9IGIyVmVjMi5TdWJWVihwQiwgcEEsIGIyUHJpc21hdGljSm9pbnQuR2V0Sm9pbnRUcmFuc2xhdGlvbl9zX2QpO1xyXG4gICAgLy8gYjJWZWMyIGF4aXMgPSBtX2JvZHlBLkdldFdvcmxkVmVjdG9yKG1fbG9jYWxYQXhpc0EpO1xyXG4gICAgY29uc3QgYXhpcyA9IHRoaXMubV9ib2R5QS5HZXRXb3JsZFZlY3RvcihcclxuICAgICAgdGhpcy5tX2xvY2FsWEF4aXNBLFxyXG4gICAgICBiMlByaXNtYXRpY0pvaW50LkdldEpvaW50VHJhbnNsYXRpb25fc19heGlzLFxyXG4gICAgKTtcclxuXHJcbiAgICAvLyBmbG9hdDMyIHRyYW5zbGF0aW9uID0gYjJEb3QoZCwgYXhpcyk7XHJcbiAgICBjb25zdCB0cmFuc2xhdGlvbjogbnVtYmVyID0gYjJWZWMyLkRvdFZWKGQsIGF4aXMpO1xyXG4gICAgcmV0dXJuIHRyYW5zbGF0aW9uO1xyXG4gIH1cclxuXHJcbiAgR2V0Sm9pbnRTcGVlZCgpOiBudW1iZXIge1xyXG4gICAgY29uc3QgYkE6IGIyQm9keSA9IHRoaXMubV9ib2R5QTtcclxuICAgIGNvbnN0IGJCOiBiMkJvZHkgPSB0aGlzLm1fYm9keUI7XHJcblxyXG4gICAgLy8gYjJWZWMyIHJBID0gYjJNdWwoYkEtPm1feGYucSwgbV9sb2NhbEFuY2hvckEgLSBiQS0+bV9zd2VlcC5sb2NhbENlbnRlcik7XHJcbiAgICBiMlZlYzIuU3ViVlYodGhpcy5tX2xvY2FsQW5jaG9yQSwgYkEubV9zd2VlcC5sb2NhbENlbnRlciwgdGhpcy5tX2xhbGNBKTtcclxuICAgIGNvbnN0IHJBOiBiMlZlYzIgPSBiMlJvdC5NdWxSVihiQS5tX3hmLnEsIHRoaXMubV9sYWxjQSwgdGhpcy5tX3JBKTtcclxuICAgIC8vIGIyVmVjMiByQiA9IGIyTXVsKGJCLT5tX3hmLnEsIG1fbG9jYWxBbmNob3JCIC0gYkItPm1fc3dlZXAubG9jYWxDZW50ZXIpO1xyXG4gICAgYjJWZWMyLlN1YlZWKHRoaXMubV9sb2NhbEFuY2hvckIsIGJCLm1fc3dlZXAubG9jYWxDZW50ZXIsIHRoaXMubV9sYWxjQik7XHJcbiAgICBjb25zdCByQjogYjJWZWMyID0gYjJSb3QuTXVsUlYoYkIubV94Zi5xLCB0aGlzLm1fbGFsY0IsIHRoaXMubV9yQik7XHJcbiAgICAvLyBiMlZlYzIgcEEgPSBiQS0+bV9zd2VlcC5jICsgckE7XHJcbiAgICBjb25zdCBwQTogYjJWZWMyID0gYjJWZWMyLkFkZFZWKGJBLm1fc3dlZXAuYywgckEsIGIyVmVjMi5zX3QwKTsgLy8gcEEgdXNlcyBzX3QwXHJcbiAgICAvLyBiMlZlYzIgcEIgPSBiQi0+bV9zd2VlcC5jICsgckI7XHJcbiAgICBjb25zdCBwQjogYjJWZWMyID0gYjJWZWMyLkFkZFZWKGJCLm1fc3dlZXAuYywgckIsIGIyVmVjMi5zX3QxKTsgLy8gcEIgdXNlcyBzX3QxXHJcbiAgICAvLyBiMlZlYzIgZCA9IHBCIC0gcEE7XHJcbiAgICBjb25zdCBkOiBiMlZlYzIgPSBiMlZlYzIuU3ViVlYocEIsIHBBLCBiMlZlYzIuc190Mik7IC8vIGQgdXNlcyBzX3QyXHJcbiAgICAvLyBiMlZlYzIgYXhpcyA9IGIyTXVsKGJBLm1feGYucSwgbV9sb2NhbFhBeGlzQSk7XHJcbiAgICBjb25zdCBheGlzID0gYkEuR2V0V29ybGRWZWN0b3IodGhpcy5tX2xvY2FsWEF4aXNBLCB0aGlzLm1fYXhpcyk7XHJcblxyXG4gICAgY29uc3QgdkEgPSBiQS5tX2xpbmVhclZlbG9jaXR5O1xyXG4gICAgY29uc3QgdkIgPSBiQi5tX2xpbmVhclZlbG9jaXR5O1xyXG4gICAgY29uc3Qgd0EgPSBiQS5tX2FuZ3VsYXJWZWxvY2l0eTtcclxuICAgIGNvbnN0IHdCID0gYkIubV9hbmd1bGFyVmVsb2NpdHk7XHJcblxyXG4gICAgLy8gZmxvYXQzMiBzcGVlZCA9IGIyRG90KGQsIGIyQ3Jvc3Mod0EsIGF4aXMpKSArIGIyRG90KGF4aXMsIHZCICsgYjJDcm9zcyh3QiwgckIpIC0gdkEgLSBiMkNyb3NzKHdBLCByQSkpO1xyXG4gICAgY29uc3Qgc3BlZWQgPVxyXG4gICAgICBiMlZlYzIuRG90VlYoZCwgYjJWZWMyLkNyb3NzU1Yod0EsIGF4aXMsIGIyVmVjMi5zX3QwKSkgK1xyXG4gICAgICBiMlZlYzIuRG90VlYoXHJcbiAgICAgICAgYXhpcyxcclxuICAgICAgICBiMlZlYzIuU3ViVlYoXHJcbiAgICAgICAgICBiMlZlYzIuQWRkVkNyb3NzU1YodkIsIHdCLCByQiwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgICAgYjJWZWMyLkFkZFZDcm9zc1NWKHZBLCB3QSwgckEsIGIyVmVjMi5zX3QxKSxcclxuICAgICAgICAgIGIyVmVjMi5zX3QwLFxyXG4gICAgICAgICksXHJcbiAgICAgICk7XHJcbiAgICByZXR1cm4gc3BlZWQ7XHJcbiAgfVxyXG5cclxuICBJc0xpbWl0RW5hYmxlZCgpIHtcclxuICAgIHJldHVybiB0aGlzLm1fZW5hYmxlTGltaXQ7XHJcbiAgfVxyXG5cclxuICBFbmFibGVMaW1pdChmbGFnOiBib29sZWFuKSB7XHJcbiAgICBpZiAoZmxhZyAhPT0gdGhpcy5tX2VuYWJsZUxpbWl0KSB7XHJcbiAgICAgIHRoaXMubV9ib2R5QS5TZXRBd2FrZSh0cnVlKTtcclxuICAgICAgdGhpcy5tX2JvZHlCLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB0aGlzLm1fZW5hYmxlTGltaXQgPSBmbGFnO1xyXG4gICAgICB0aGlzLm1faW1wdWxzZS56ID0gMDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIEdldExvd2VyTGltaXQoKSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2xvd2VyVHJhbnNsYXRpb247XHJcbiAgfVxyXG5cclxuICBHZXRVcHBlckxpbWl0KCkge1xyXG4gICAgcmV0dXJuIHRoaXMubV91cHBlclRyYW5zbGF0aW9uO1xyXG4gIH1cclxuXHJcbiAgU2V0TGltaXRzKGxvd2VyOiBudW1iZXIsIHVwcGVyOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIGlmIChsb3dlciAhPT0gdGhpcy5tX2xvd2VyVHJhbnNsYXRpb24gfHwgdXBwZXIgIT09IHRoaXMubV91cHBlclRyYW5zbGF0aW9uKSB7XHJcbiAgICAgIHRoaXMubV9ib2R5QS5TZXRBd2FrZSh0cnVlKTtcclxuICAgICAgdGhpcy5tX2JvZHlCLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB0aGlzLm1fbG93ZXJUcmFuc2xhdGlvbiA9IGxvd2VyO1xyXG4gICAgICB0aGlzLm1fdXBwZXJUcmFuc2xhdGlvbiA9IHVwcGVyO1xyXG4gICAgICB0aGlzLm1faW1wdWxzZS56ID0gMDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIElzTW90b3JFbmFibGVkKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9lbmFibGVNb3RvcjtcclxuICB9XHJcblxyXG4gIEVuYWJsZU1vdG9yKGZsYWc6IGJvb2xlYW4pOiB2b2lkIHtcclxuICAgIGlmIChmbGFnICE9PSB0aGlzLm1fZW5hYmxlTW90b3IpIHtcclxuICAgICAgdGhpcy5tX2JvZHlBLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB0aGlzLm1fYm9keUIuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9lbmFibGVNb3RvciA9IGZsYWc7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBTZXRNb3RvclNwZWVkKHNwZWVkOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIGlmIChzcGVlZCAhPT0gdGhpcy5tX21vdG9yU3BlZWQpIHtcclxuICAgICAgdGhpcy5tX2JvZHlBLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB0aGlzLm1fYm9keUIuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9tb3RvclNwZWVkID0gc3BlZWQ7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBHZXRNb3RvclNwZWVkKCkge1xyXG4gICAgcmV0dXJuIHRoaXMubV9tb3RvclNwZWVkO1xyXG4gIH1cclxuXHJcbiAgU2V0TWF4TW90b3JGb3JjZShmb3JjZTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICBpZiAoZm9yY2UgIT09IHRoaXMubV9tYXhNb3RvckZvcmNlKSB7XHJcbiAgICAgIHRoaXMubV9ib2R5QS5TZXRBd2FrZSh0cnVlKTtcclxuICAgICAgdGhpcy5tX2JvZHlCLlNldEF3YWtlKHRydWUpO1xyXG4gICAgICB0aGlzLm1fbWF4TW90b3JGb3JjZSA9IGZvcmNlO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgR2V0TWF4TW90b3JGb3JjZSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9tYXhNb3RvckZvcmNlO1xyXG4gIH1cclxuXHJcbiAgR2V0TW90b3JGb3JjZShpbnZfZHQ6IG51bWJlcik6IG51bWJlciB7XHJcbiAgICByZXR1cm4gaW52X2R0ICogdGhpcy5tX21vdG9ySW1wdWxzZTtcclxuICB9XHJcbn1cclxuIl19