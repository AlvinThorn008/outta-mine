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
import { b2_linearSlop, b2Assert, b2Maybe } from '../../common/b2Settings';
import { b2IsValid, b2Rot, b2Vec2 } from '../../common/b2Math';
import { b2Joint, b2JointDef } from './b2Joint';
/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
export class b2GearJointDef extends b2JointDef {
    constructor() {
        super(6 /* e_gearJoint */);
        this.ratio = 1;
    }
}
export class b2GearJoint extends b2Joint {
    constructor(def) {
        super(def);
        this.m_typeA = 0 /* e_unknownJoint */;
        this.m_typeB = 0 /* e_unknownJoint */;
        // Solver shared
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_localAnchorC = new b2Vec2();
        this.m_localAnchorD = new b2Vec2();
        this.m_localAxisC = new b2Vec2();
        this.m_localAxisD = new b2Vec2();
        this.m_referenceAngleA = 0;
        this.m_referenceAngleB = 0;
        this.m_constant = 0;
        this.m_ratio = 0;
        this.m_impulse = 0;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_indexC = 0;
        this.m_indexD = 0;
        this.m_lcA = new b2Vec2();
        this.m_lcB = new b2Vec2();
        this.m_lcC = new b2Vec2();
        this.m_lcD = new b2Vec2();
        this.m_mA = 0;
        this.m_mB = 0;
        this.m_mC = 0;
        this.m_mD = 0;
        this.m_iA = 0;
        this.m_iB = 0;
        this.m_iC = 0;
        this.m_iD = 0;
        this.m_JvAC = new b2Vec2();
        this.m_JvBD = new b2Vec2();
        this.m_JwA = 0;
        this.m_JwB = 0;
        this.m_JwC = 0;
        this.m_JwD = 0;
        this.m_mass = 0;
        this.m_qA = new b2Rot();
        this.m_qB = new b2Rot();
        this.m_qC = new b2Rot();
        this.m_qD = new b2Rot();
        this.m_lalcA = new b2Vec2();
        this.m_lalcB = new b2Vec2();
        this.m_lalcC = new b2Vec2();
        this.m_lalcD = new b2Vec2();
        this.m_joint1 = def.joint1;
        this.m_joint2 = def.joint2;
        this.m_typeA = this.m_joint1.GetType();
        this.m_typeB = this.m_joint2.GetType();
        !!B2_DEBUG &&
            b2Assert(this.m_typeA === 1 /* e_revoluteJoint */ ||
                this.m_typeA === 2 /* e_prismaticJoint */);
        !!B2_DEBUG &&
            b2Assert(this.m_typeB === 1 /* e_revoluteJoint */ ||
                this.m_typeB === 2 /* e_prismaticJoint */);
        let coordinateA, coordinateB;
        // TODO_ERIN there might be some problem with the joint edges in b2Joint.
        this.m_bodyC = this.m_joint1.GetBodyA();
        this.m_bodyA = this.m_joint1.GetBodyB();
        // Get geometry of joint1
        const xfA = this.m_bodyA.m_xf;
        const aA = this.m_bodyA.m_sweep.a;
        const xfC = this.m_bodyC.m_xf;
        const aC = this.m_bodyC.m_sweep.a;
        if (this.m_typeA === 1 /* e_revoluteJoint */) {
            const revolute = def.joint1;
            this.m_localAnchorC.Copy(revolute.m_localAnchorA);
            this.m_localAnchorA.Copy(revolute.m_localAnchorB);
            this.m_referenceAngleA = revolute.m_referenceAngle;
            this.m_localAxisC.SetZero();
            coordinateA = aA - aC - this.m_referenceAngleA;
        }
        else {
            const prismatic = def.joint1;
            this.m_localAnchorC.Copy(prismatic.m_localAnchorA);
            this.m_localAnchorA.Copy(prismatic.m_localAnchorB);
            this.m_referenceAngleA = prismatic.m_referenceAngle;
            this.m_localAxisC.Copy(prismatic.m_localXAxisA);
            // b2Vec2 pC = m_localAnchorC;
            const pC = this.m_localAnchorC;
            // b2Vec2 pA = b2MulT(xfC.q, b2Mul(xfA.q, m_localAnchorA) + (xfA.p - xfC.p));
            const pA = b2Rot.MulTRV(xfC.q, b2Vec2.AddVV(b2Rot.MulRV(xfA.q, this.m_localAnchorA, b2Vec2.s_t0), b2Vec2.SubVV(xfA.p, xfC.p, b2Vec2.s_t1), b2Vec2.s_t0), b2Vec2.s_t0); // pA uses s_t0
            // coordinateA = b2Dot(pA - pC, m_localAxisC);
            coordinateA = b2Vec2.DotVV(b2Vec2.SubVV(pA, pC, b2Vec2.s_t0), this.m_localAxisC);
        }
        this.m_bodyD = this.m_joint2.GetBodyA();
        this.m_bodyB = this.m_joint2.GetBodyB();
        // Get geometry of joint2
        const xfB = this.m_bodyB.m_xf;
        const aB = this.m_bodyB.m_sweep.a;
        const xfD = this.m_bodyD.m_xf;
        const aD = this.m_bodyD.m_sweep.a;
        if (this.m_typeB === 1 /* e_revoluteJoint */) {
            const revolute = def.joint2;
            this.m_localAnchorD.Copy(revolute.m_localAnchorA);
            this.m_localAnchorB.Copy(revolute.m_localAnchorB);
            this.m_referenceAngleB = revolute.m_referenceAngle;
            this.m_localAxisD.SetZero();
            coordinateB = aB - aD - this.m_referenceAngleB;
        }
        else {
            const prismatic = def.joint2;
            this.m_localAnchorD.Copy(prismatic.m_localAnchorA);
            this.m_localAnchorB.Copy(prismatic.m_localAnchorB);
            this.m_referenceAngleB = prismatic.m_referenceAngle;
            this.m_localAxisD.Copy(prismatic.m_localXAxisA);
            // b2Vec2 pD = m_localAnchorD;
            const pD = this.m_localAnchorD;
            // b2Vec2 pB = b2MulT(xfD.q, b2Mul(xfB.q, m_localAnchorB) + (xfB.p - xfD.p));
            const pB = b2Rot.MulTRV(xfD.q, b2Vec2.AddVV(b2Rot.MulRV(xfB.q, this.m_localAnchorB, b2Vec2.s_t0), b2Vec2.SubVV(xfB.p, xfD.p, b2Vec2.s_t1), b2Vec2.s_t0), b2Vec2.s_t0); // pB uses s_t0
            // coordinateB = b2Dot(pB - pD, m_localAxisD);
            coordinateB = b2Vec2.DotVV(b2Vec2.SubVV(pB, pD, b2Vec2.s_t0), this.m_localAxisD);
        }
        this.m_ratio = b2Maybe(def.ratio, 1);
        this.m_constant = coordinateA + this.m_ratio * coordinateB;
        this.m_impulse = 0;
    }
    InitVelocityConstraints(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_indexC = this.m_bodyC.m_islandIndex;
        this.m_indexD = this.m_bodyD.m_islandIndex;
        this.m_lcA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_lcB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_lcC.Copy(this.m_bodyC.m_sweep.localCenter);
        this.m_lcD.Copy(this.m_bodyD.m_sweep.localCenter);
        this.m_mA = this.m_bodyA.m_invMass;
        this.m_mB = this.m_bodyB.m_invMass;
        this.m_mC = this.m_bodyC.m_invMass;
        this.m_mD = this.m_bodyD.m_invMass;
        this.m_iA = this.m_bodyA.m_invI;
        this.m_iB = this.m_bodyB.m_invI;
        this.m_iC = this.m_bodyC.m_invI;
        this.m_iD = this.m_bodyD.m_invI;
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const aC = data.positions[this.m_indexC].a;
        const vC = data.velocities[this.m_indexC].v;
        let wC = data.velocities[this.m_indexC].w;
        const aD = data.positions[this.m_indexD].a;
        const vD = data.velocities[this.m_indexD].v;
        let wD = data.velocities[this.m_indexD].w;
        // b2Rot qA(aA), qB(aB), qC(aC), qD(aD);
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB), qC = this.m_qC.SetAngle(aC), qD = this.m_qD.SetAngle(aD);
        this.m_mass = 0;
        if (this.m_typeA === 1 /* e_revoluteJoint */) {
            this.m_JvAC.SetZero();
            this.m_JwA = 1;
            this.m_JwC = 1;
            this.m_mass += this.m_iA + this.m_iC;
        }
        else {
            // b2Vec2 u = b2Mul(qC, m_localAxisC);
            const u = b2Rot.MulRV(qC, this.m_localAxisC, b2GearJoint.InitVelocityConstraints_s_u);
            // b2Vec2 rC = b2Mul(qC, m_localAnchorC - m_lcC);
            b2Vec2.SubVV(this.m_localAnchorC, this.m_lcC, this.m_lalcC);
            const rC = b2Rot.MulRV(qC, this.m_lalcC, b2GearJoint.InitVelocityConstraints_s_rC);
            // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_lcA);
            b2Vec2.SubVV(this.m_localAnchorA, this.m_lcA, this.m_lalcA);
            const rA = b2Rot.MulRV(qA, this.m_lalcA, b2GearJoint.InitVelocityConstraints_s_rA);
            // m_JvAC = u;
            this.m_JvAC.Copy(u);
            // m_JwC = b2Cross(rC, u);
            this.m_JwC = b2Vec2.CrossVV(rC, u);
            // m_JwA = b2Cross(rA, u);
            this.m_JwA = b2Vec2.CrossVV(rA, u);
            this.m_mass +=
                this.m_mC +
                    this.m_mA +
                    this.m_iC * this.m_JwC * this.m_JwC +
                    this.m_iA * this.m_JwA * this.m_JwA;
        }
        if (this.m_typeB === 1 /* e_revoluteJoint */) {
            this.m_JvBD.SetZero();
            this.m_JwB = this.m_ratio;
            this.m_JwD = this.m_ratio;
            this.m_mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
        }
        else {
            // b2Vec2 u = b2Mul(qD, m_localAxisD);
            const u = b2Rot.MulRV(qD, this.m_localAxisD, b2GearJoint.InitVelocityConstraints_s_u);
            // b2Vec2 rD = b2Mul(qD, m_localAnchorD - m_lcD);
            b2Vec2.SubVV(this.m_localAnchorD, this.m_lcD, this.m_lalcD);
            const rD = b2Rot.MulRV(qD, this.m_lalcD, b2GearJoint.InitVelocityConstraints_s_rD);
            // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_lcB);
            b2Vec2.SubVV(this.m_localAnchorB, this.m_lcB, this.m_lalcB);
            const rB = b2Rot.MulRV(qB, this.m_lalcB, b2GearJoint.InitVelocityConstraints_s_rB);
            // m_JvBD = m_ratio * u;
            b2Vec2.MulSV(this.m_ratio, u, this.m_JvBD);
            // m_JwD = m_ratio * b2Cross(rD, u);
            this.m_JwD = this.m_ratio * b2Vec2.CrossVV(rD, u);
            // m_JwB = m_ratio * b2Cross(rB, u);
            this.m_JwB = this.m_ratio * b2Vec2.CrossVV(rB, u);
            this.m_mass +=
                this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) +
                    this.m_iD * this.m_JwD * this.m_JwD +
                    this.m_iB * this.m_JwB * this.m_JwB;
        }
        // Compute effective mass.
        this.m_mass = this.m_mass > 0 ? 1 / this.m_mass : 0;
        if (data.step.warmStarting) {
            // vA += (m_mA * m_impulse) * m_JvAC;
            vA.SelfMulAdd(this.m_mA * this.m_impulse, this.m_JvAC);
            wA += this.m_iA * this.m_impulse * this.m_JwA;
            // vB += (m_mB * m_impulse) * m_JvBD;
            vB.SelfMulAdd(this.m_mB * this.m_impulse, this.m_JvBD);
            wB += this.m_iB * this.m_impulse * this.m_JwB;
            // vC -= (m_mC * m_impulse) * m_JvAC;
            vC.SelfMulSub(this.m_mC * this.m_impulse, this.m_JvAC);
            wC -= this.m_iC * this.m_impulse * this.m_JwC;
            // vD -= (m_mD * m_impulse) * m_JvBD;
            vD.SelfMulSub(this.m_mD * this.m_impulse, this.m_JvBD);
            wD -= this.m_iD * this.m_impulse * this.m_JwD;
        }
        else {
            this.m_impulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
        // data.velocities[this.m_indexC].v = vC;
        data.velocities[this.m_indexC].w = wC;
        // data.velocities[this.m_indexD].v = vD;
        data.velocities[this.m_indexD].w = wD;
    }
    SolveVelocityConstraints(data) {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const vC = data.velocities[this.m_indexC].v;
        let wC = data.velocities[this.m_indexC].w;
        const vD = data.velocities[this.m_indexD].v;
        let wD = data.velocities[this.m_indexD].w;
        // float32 Cdot = b2Dot(m_JvAC, vA - vC) + b2Dot(m_JvBD, vB - vD);
        let Cdot = b2Vec2.DotVV(this.m_JvAC, b2Vec2.SubVV(vA, vC, b2Vec2.s_t0)) +
            b2Vec2.DotVV(this.m_JvBD, b2Vec2.SubVV(vB, vD, b2Vec2.s_t0));
        Cdot += this.m_JwA * wA - this.m_JwC * wC + (this.m_JwB * wB - this.m_JwD * wD);
        const impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;
        // vA += (m_mA * impulse) * m_JvAC;
        vA.SelfMulAdd(this.m_mA * impulse, this.m_JvAC);
        wA += this.m_iA * impulse * this.m_JwA;
        // vB += (m_mB * impulse) * m_JvBD;
        vB.SelfMulAdd(this.m_mB * impulse, this.m_JvBD);
        wB += this.m_iB * impulse * this.m_JwB;
        // vC -= (m_mC * impulse) * m_JvAC;
        vC.SelfMulSub(this.m_mC * impulse, this.m_JvAC);
        wC -= this.m_iC * impulse * this.m_JwC;
        // vD -= (m_mD * impulse) * m_JvBD;
        vD.SelfMulSub(this.m_mD * impulse, this.m_JvBD);
        wD -= this.m_iD * impulse * this.m_JwD;
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
        // data.velocities[this.m_indexC].v = vC;
        data.velocities[this.m_indexC].w = wC;
        // data.velocities[this.m_indexD].v = vD;
        data.velocities[this.m_indexD].w = wD;
    }
    SolvePositionConstraints(data) {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;
        const cC = data.positions[this.m_indexC].c;
        let aC = data.positions[this.m_indexC].a;
        const cD = data.positions[this.m_indexD].c;
        let aD = data.positions[this.m_indexD].a;
        // b2Rot qA(aA), qB(aB), qC(aC), qD(aD);
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB), qC = this.m_qC.SetAngle(aC), qD = this.m_qD.SetAngle(aD);
        const linearError = 0;
        let coordinateA, coordinateB;
        const JvAC = this.m_JvAC, JvBD = this.m_JvBD;
        let JwA, JwB, JwC, JwD;
        let mass = 0;
        if (this.m_typeA === 1 /* e_revoluteJoint */) {
            JvAC.SetZero();
            JwA = 1;
            JwC = 1;
            mass += this.m_iA + this.m_iC;
            coordinateA = aA - aC - this.m_referenceAngleA;
        }
        else {
            // b2Vec2 u = b2Mul(qC, m_localAxisC);
            const u = b2Rot.MulRV(qC, this.m_localAxisC, b2GearJoint.SolvePositionConstraints_s_u);
            // b2Vec2 rC = b2Mul(qC, m_localAnchorC - m_lcC);
            const rC = b2Rot.MulRV(qC, this.m_lalcC, b2GearJoint.SolvePositionConstraints_s_rC);
            // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_lcA);
            const rA = b2Rot.MulRV(qA, this.m_lalcA, b2GearJoint.SolvePositionConstraints_s_rA);
            // JvAC = u;
            JvAC.Copy(u);
            // JwC = b2Cross(rC, u);
            JwC = b2Vec2.CrossVV(rC, u);
            // JwA = b2Cross(rA, u);
            JwA = b2Vec2.CrossVV(rA, u);
            mass += this.m_mC + this.m_mA + this.m_iC * JwC * JwC + this.m_iA * JwA * JwA;
            // b2Vec2 pC = m_localAnchorC - m_lcC;
            const pC = this.m_lalcC;
            // b2Vec2 pA = b2MulT(qC, rA + (cA - cC));
            const pA = b2Rot.MulTRV(qC, b2Vec2.AddVV(rA, b2Vec2.SubVV(cA, cC, b2Vec2.s_t0), b2Vec2.s_t0), b2Vec2.s_t0); // pA uses s_t0
            // coordinateA = b2Dot(pA - pC, m_localAxisC);
            coordinateA = b2Vec2.DotVV(b2Vec2.SubVV(pA, pC, b2Vec2.s_t0), this.m_localAxisC);
        }
        if (this.m_typeB === 1 /* e_revoluteJoint */) {
            JvBD.SetZero();
            JwB = this.m_ratio;
            JwD = this.m_ratio;
            mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
            coordinateB = aB - aD - this.m_referenceAngleB;
        }
        else {
            // b2Vec2 u = b2Mul(qD, m_localAxisD);
            const u = b2Rot.MulRV(qD, this.m_localAxisD, b2GearJoint.SolvePositionConstraints_s_u);
            // b2Vec2 rD = b2Mul(qD, m_localAnchorD - m_lcD);
            const rD = b2Rot.MulRV(qD, this.m_lalcD, b2GearJoint.SolvePositionConstraints_s_rD);
            // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_lcB);
            const rB = b2Rot.MulRV(qB, this.m_lalcB, b2GearJoint.SolvePositionConstraints_s_rB);
            // JvBD = m_ratio * u;
            b2Vec2.MulSV(this.m_ratio, u, JvBD);
            // JwD = m_ratio * b2Cross(rD, u);
            JwD = this.m_ratio * b2Vec2.CrossVV(rD, u);
            // JwB = m_ratio * b2Cross(rB, u);
            JwB = this.m_ratio * b2Vec2.CrossVV(rB, u);
            mass +=
                this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) +
                    this.m_iD * JwD * JwD +
                    this.m_iB * JwB * JwB;
            // b2Vec2 pD = m_localAnchorD - m_lcD;
            const pD = this.m_lalcD;
            // b2Vec2 pB = b2MulT(qD, rB + (cB - cD));
            const pB = b2Rot.MulTRV(qD, b2Vec2.AddVV(rB, b2Vec2.SubVV(cB, cD, b2Vec2.s_t0), b2Vec2.s_t0), b2Vec2.s_t0); // pB uses s_t0
            // coordinateB = b2Dot(pB - pD, m_localAxisD);
            coordinateB = b2Vec2.DotVV(b2Vec2.SubVV(pB, pD, b2Vec2.s_t0), this.m_localAxisD);
        }
        const C = coordinateA + this.m_ratio * coordinateB - this.m_constant;
        let impulse = 0;
        if (mass > 0) {
            impulse = -C / mass;
        }
        // cA += m_mA * impulse * JvAC;
        cA.SelfMulAdd(this.m_mA * impulse, JvAC);
        aA += this.m_iA * impulse * JwA;
        // cB += m_mB * impulse * JvBD;
        cB.SelfMulAdd(this.m_mB * impulse, JvBD);
        aB += this.m_iB * impulse * JwB;
        // cC -= m_mC * impulse * JvAC;
        cC.SelfMulSub(this.m_mC * impulse, JvAC);
        aC -= this.m_iC * impulse * JwC;
        // cD -= m_mD * impulse * JvBD;
        cD.SelfMulSub(this.m_mD * impulse, JvBD);
        aD -= this.m_iD * impulse * JwD;
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        // data.positions[this.m_indexC].c = cC;
        data.positions[this.m_indexC].a = aC;
        // data.positions[this.m_indexD].c = cD;
        data.positions[this.m_indexD].a = aD;
        // TODO_ERIN not implemented
        return linearError < b2_linearSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // b2Vec2 P = m_impulse * m_JvAC;
        // return inv_dt * P;
        return b2Vec2.MulSV(inv_dt * this.m_impulse, this.m_JvAC, out);
    }
    GetReactionTorque(inv_dt) {
        // float32 L = m_impulse * m_JwA;
        // return inv_dt * L;
        return inv_dt * this.m_impulse * this.m_JwA;
    }
    GetJoint1() {
        return this.m_joint1;
    }
    GetJoint2() {
        return this.m_joint2;
    }
    GetRatio() {
        return this.m_ratio;
    }
    SetRatio(ratio) {
        !!B2_DEBUG && b2Assert(b2IsValid(ratio));
        this.m_ratio = ratio;
    }
}
b2GearJoint.InitVelocityConstraints_s_u = new b2Vec2();
b2GearJoint.InitVelocityConstraints_s_rA = new b2Vec2();
b2GearJoint.InitVelocityConstraints_s_rB = new b2Vec2();
b2GearJoint.InitVelocityConstraints_s_rC = new b2Vec2();
b2GearJoint.InitVelocityConstraints_s_rD = new b2Vec2();
b2GearJoint.SolvePositionConstraints_s_u = new b2Vec2();
b2GearJoint.SolvePositionConstraints_s_rA = new b2Vec2();
b2GearJoint.SolvePositionConstraints_s_rB = new b2Vec2();
b2GearJoint.SolvePositionConstraints_s_rC = new b2Vec2();
b2GearJoint.SolvePositionConstraints_s_rD = new b2Vec2();
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJHZWFySm9pbnQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9zcmMvZHluYW1pY3Mvam9pbnRzL2IyR2VhckpvaW50LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsT0FBTyxFQUFFLGFBQWEsRUFBRSxRQUFRLEVBQUUsT0FBTyxFQUFFLE1BQU0seUJBQXlCLENBQUM7QUFDM0UsT0FBTyxFQUFFLFNBQVMsRUFBRSxLQUFLLEVBQWUsTUFBTSxFQUFNLE1BQU0scUJBQXFCLENBQUM7QUFDaEYsT0FBTyxFQUFlLE9BQU8sRUFBRSxVQUFVLEVBQWUsTUFBTSxXQUFXLENBQUM7QUFjMUUsZ0VBQWdFO0FBQ2hFLDZEQUE2RDtBQUM3RCxNQUFNLE9BQU8sY0FBZSxTQUFRLFVBQVU7SUFPNUM7UUFDRSxLQUFLLHFCQUF5QixDQUFDO1FBSGpDLFVBQUssR0FBRyxDQUFDLENBQUM7SUFJVixDQUFDO0NBQ0Y7QUFFRCxNQUFNLE9BQU8sV0FBWSxTQUFRLE9BQU87SUErRHRDLFlBQVksR0FBb0I7UUFDOUIsS0FBSyxDQUFDLEdBQUcsQ0FBQyxDQUFDO1FBNURiLFlBQU8sMEJBQTJDO1FBQ2xELFlBQU8sMEJBQTJDO1FBT2xELGdCQUFnQjtRQUNQLG1CQUFjLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUN0QyxtQkFBYyxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7UUFDdEMsbUJBQWMsR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQ3RDLG1CQUFjLEdBQVcsSUFBSSxNQUFNLEVBQUUsQ0FBQztRQUV0QyxpQkFBWSxHQUFXLElBQUksTUFBTSxFQUFFLENBQUM7UUFDcEMsaUJBQVksR0FBVyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBRTdDLHNCQUFpQixHQUFHLENBQUMsQ0FBQztRQUN0QixzQkFBaUIsR0FBRyxDQUFDLENBQUM7UUFFdEIsZUFBVSxHQUFHLENBQUMsQ0FBQztRQUNm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QUMsU0FBUyxHQUFHLElBQUksQ0FBQyxLQUFLLENBQUM7WUFDOUMscUNBQXFDO1lBQ3JDLEVBQUUsQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLElBQUksR0FBRyxJQUFJLENBQUMsU0FBUyxFQUFFLElBQUksQ0FBQyxNQUFNLENBQUMsQ0FBQztZQUN2RCxFQUFFLElBQUksSUFBSSxDQUFDLElBQUksR0FBRyxJQUFJLENBQUMsU0FBUyxHQUFHLElBQUksQ0FBQyxLQUFLLENBQUM7WUFDOUMscUNBQXFDO1lBQ3JDLEVBQUUsQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLElBQUksR0FBRyxJQUFJLENBQUMsU0FBUyxFQUFFLElBQUksQ0FBQyxNQUFNLENBQUMsQ0FBQztZQUN2RCxFQUFFLElBQUksSUFBSSxDQUFDLElBQUksR0FBRyxJQUFJLENBQUMsU0FBUyxHQUFHLElBQUksQ0FBQyxLQUFLLENBQUM7U0FDL0M7YUFBTTtZQUNMLElBQUksQ0FBQyxTQUFTLEdBQUcsQ0FBQyxDQUFDO1NBQ3BCO1FBRUQseUNBQXlDO1FBQ3pDLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7UUFDdEMseUNBQXlDO1FBQ3pDLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7UUFDdEMseUNBQXlDO1FBQ3pDLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7UUFDdEMseUNBQXlDO1FBQ3pDLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7SUFDeEMsQ0FBQztJQUVELHdCQUF3QixDQUFDLElBQWtCO1FBQ3pDLE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNwRCxJQUFJLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDbEQsTUFBTSxFQUFFLEdBQVcsSUFBSSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ3BELElBQUksRUFBRSxHQUFXLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNsRCxNQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDcEQsSUFBSSxFQUFFLEdBQVcsSUFBSSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ2xELE1BQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNwRCxJQUFJLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFFbEQsa0VBQWtFO1FBQ2xFLElBQUksSUFBSSxHQUNOLE1BQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLE1BQU0sRUFBRSxNQUFNLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxFQUFFLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxDQUFDO1lBQzVELE1BQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLE1BQU0sRUFBRSxNQUFNLENBQUMsS0FBSyxDQUFDLEVBQUUsRUFBRSxFQUFFLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxDQUFDLENBQUM7UUFDL0QsSUFBSSxJQUFJLElBQUksQ0FBQyxLQUFLLEdBQUcsRUFBRSxHQUFHLElBQUksQ0FBQyxLQUFLLEdBQUcsRUFBRSxHQUFHLENBQUMsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLEdBQUcsSUFBSSxDQUFDLEtB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TSxFQUFFLENBQUM7QUFDNUMseUNBQTZCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUM3Qyx5Q0FBNkIsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO0FBQzdDLHlDQUE2QixHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDN0MseUNBQTZCLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDExIEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHsgYjJfbGluZWFyU2xvcCwgYjJBc3NlcnQsIGIyTWF5YmUgfSBmcm9tICcuLi8uLi9jb21tb24vYjJTZXR0aW5ncyc7XHJcbmltcG9ydCB7IGIySXNWYWxpZCwgYjJSb3QsIGIyVHJhbnNmb3JtLCBiMlZlYzIsIFhZIH0gZnJvbSAnLi4vLi4vY29tbW9uL2IyTWF0aCc7XHJcbmltcG9ydCB7IGIySUpvaW50RGVmLCBiMkpvaW50LCBiMkpvaW50RGVmLCBiMkpvaW50VHlwZSB9IGZyb20gJy4vYjJKb2ludCc7XHJcbmltcG9ydCB7IGIyUHJpc21hdGljSm9pbnQgfSBmcm9tICcuL2IyUHJpc21hdGljSm9pbnQnO1xyXG5pbXBvcnQgeyBiMlJldm9sdXRlSm9pbnQgfSBmcm9tICcuL2IyUmV2b2x1dGVKb2ludCc7XHJcbmltcG9ydCB7IGIyU29sdmVyRGF0YSB9IGZyb20gJy4uL2IyVGltZVN0ZXAnO1xyXG5pbXBvcnQgeyBiMkJvZHkgfSBmcm9tICcuLi9iMkJvZHknO1xyXG5cclxuZXhwb3J0IGludGVyZmFjZSBiMklHZWFySm9pbnREZWYgZXh0ZW5kcyBiMklKb2ludERlZiB7XHJcbiAgam9pbnQxOiBiMlJldm9sdXRlSm9pbnQgfCBiMlByaXNtYXRpY0pvaW50O1xyXG5cclxuICBqb2ludDI6IGIyUmV2b2x1dGVKb2ludCB8IGIyUHJpc21hdGljSm9pbnQ7XHJcblxyXG4gIHJhdGlvPzogbnVtYmVyO1xyXG59XHJcblxyXG4vLy8gR2VhciBqb2ludCBkZWZpbml0aW9uLiBUaGlzIGRlZmluaXRpb24gcmVxdWlyZXMgdHdvIGV4aXN0aW5nXHJcbi8vLyByZXZvbHV0ZSBvciBwcmlzbWF0aWMgam9pbnRzIChhbnkgY29tYmluYXRpb24gd2lsbCB3b3JrKS5cclxuZXhwb3J0IGNsYXNzIGIyR2VhckpvaW50RGVmIGV4dGVuZHMgYjJKb2ludERlZiBpbXBsZW1lbnRzIGIySUdlYXJKb2ludERlZiB7XHJcbiAgam9pbnQxITogYjJSZXZvbHV0ZUpvaW50IHwgYjJQcmlzbWF0aWNKb2ludDtcclxuXHJcbiAgam9pbnQyITogYjJSZXZvbHV0ZUpvaW50IHwgYjJQcmlzbWF0aWNKb2ludDtcclxuXHJcbiAgcmF0aW8gPSAxO1xyXG5cclxuICBjb25zdHJ1Y3RvcigpIHtcclxuICAgIHN1cGVyKGIySm9pbnRUeXBlLmVfZ2VhckpvaW50KTtcclxuICB9XHJcbn1cclxuXHJcbmV4cG9ydCBjbGFzcyBiMkdlYXJKb2ludCBleHRlbmRzIGIySm9pbnQge1xyXG4gIG1fam9pbnQxOiBiMlJldm9sdXRlSm9pbnQgfCBiMlByaXNtYXRpY0pvaW50O1xyXG4gIG1fam9pbnQyOiBiMlJldm9sdXRlSm9pbnQgfCBiMlByaXNtYXRpY0pvaW50O1xyXG5cclxuICBtX3R5cGVBOiBiMkpvaW50VHlwZSA9IGIySm9pbnRUeXBlLmVfdW5rbm93bkpvaW50O1xyXG4gIG1fdHlwZUI6IGIySm9pbnRUeXBlID0gYjJKb2ludFR5cGUuZV91bmtub3duSm9pbnQ7XHJcblxyXG4gIC8vIEJvZHkgQSBpcyBjb25uZWN0ZWQgdG8gYm9keSBDXHJcbiAgLy8gQm9keSBCIGlzIGNvbm5lY3RlZCB0byBib2R5IERcclxuICBtX2JvZHlDOiBiMkJvZHk7XHJcbiAgbV9ib2R5RDogYjJCb2R5O1xyXG5cclxuICAvLyBTb2x2ZXIgc2hhcmVkXHJcbiAgcmVhZG9ubHkgbV9sb2NhbEFuY2hvckE6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xvY2FsQW5jaG9yQjogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fbG9jYWxBbmNob3JDOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbEFuY2hvckQ6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgcmVhZG9ubHkgbV9sb2NhbEF4aXNDOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sb2NhbEF4aXNEOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIG1fcmVmZXJlbmNlQW5nbGVBID0gMDtcclxuICBtX3JlZmVyZW5jZUFuZ2xlQiA9IDA7XHJcblxyXG4gIG1fY29uc3RhbnQgPSAwO1xyXG4gIG1fcmF0aW8gPSAwO1xyXG5cclxuICBtX2ltcHVsc2UgPSAwO1xyXG5cclxuICAvLyBTb2x2ZXIgdGVtcFxyXG4gIG1faW5kZXhBID0gMDtcclxuICBtX2luZGV4QiA9IDA7XHJcbiAgbV9pbmRleEMgPSAwO1xyXG4gIG1faW5kZXhEID0gMDtcclxuICByZWFkb25seSBtX2xjQSA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xjQiA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xjQyA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xjRCA9IG5ldyBiMlZlYzIoKTtcclxuICBtX21BID0gMDtcclxuICBtX21CID0gMDtcclxuICBtX21DID0gMDtcclxuICBtX21EID0gMDtcclxuICBtX2lBID0gMDtcclxuICBtX2lCID0gMDtcclxuICBtX2lDID0gMDtcclxuICBtX2lEID0gMDtcclxuICByZWFkb25seSBtX0p2QUMgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9KdkJEID0gbmV3IGIyVmVjMigpO1xyXG4gIG1fSndBID0gMDtcclxuICBtX0p3QiA9IDA7XHJcbiAgbV9Kd0MgPSAwO1xyXG4gIG1fSndEID0gMDtcclxuICBtX21hc3MgPSAwO1xyXG5cclxuICByZWFkb25seSBtX3FBID0gbmV3IGIyUm90KCk7XHJcbiAgcmVhZG9ubHkgbV9xQiA9IG5ldyBiMlJvdCgpO1xyXG4gIHJlYWRvbmx5IG1fcUMgPSBuZXcgYjJSb3QoKTtcclxuICByZWFkb25seSBtX3FEID0gbmV3IGIyUm90KCk7XHJcbiAgcmVhZG9ubHkgbV9sYWxjQSA9IG5ldyBiMlZlYzIoKTtcclxuICByZWFkb25seSBtX2xhbGNCID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IG1fbGFsY0MgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcmVhZG9ubHkgbV9sYWxjRCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgY29uc3RydWN0b3IoZGVmOiBiMklHZWFySm9pbnREZWYpIHtcclxuICAgIHN1cGVyKGRlZik7XHJcblxyXG4gICAgdGhpcy5tX2pvaW50MSA9IGRlZi5qb2ludDE7XHJcbiAgICB0aGlzLm1fam9pbnQyID0gZGVmLmpvaW50MjtcclxuXHJcbiAgICB0aGlzLm1fdHlwZUEgPSB0aGlzLm1fam9pbnQxLkdldFR5cGUoKTtcclxuICAgIHRoaXMubV90eXBlQiA9IHRoaXMubV9qb2ludDIuR2V0VHlwZSgpO1xyXG5cclxuICAgICEhQjJfREVCVUcgJiZcclxuICAgICAgYjJBc3NlcnQoXHJcbiAgICAgICAgdGhpcy5tX3R5cGVBID09PSBiMkpvaW50VHlwZS5lX3Jldm9sdXRlSm9pbnQgfHxcclxuICAgICAgICAgIHRoaXMubV90eXBlQSA9PT0gYjJKb2ludFR5cGUuZV9wcmlzbWF0aWNKb2ludCxcclxuICAgICAgKTtcclxuICAgICEhQjJfREVCVUcgJiZcclxuICAgICAgYjJBc3NlcnQoXHJcbiAgICAgICAgdGhpcy5tX3R5cGVCID09PSBiMkpvaW50VHlwZS5lX3Jldm9sdXRlSm9pbnQgfHxcclxuICAgICAgICAgIHRoaXMubV90eXBlQiA9PT0gYjJKb2ludFR5cGUuZV9wcmlzbWF0aWNKb2ludCxcclxuICAgICAgKTtcclxuXHJcbiAgICBsZXQgY29vcmRpbmF0ZUE6IG51bWJlciwgY29vcmRpbmF0ZUI6IG51bWJlcjtcclxuXHJcbiAgICAvLyBUT0RPX0VSSU4gdGhlcmUgbWlnaHQgYmUgc29tZSBwcm9ibGVtIHdpdGggdGhlIGpvaW50IGVkZ2VzIGluIGIySm9pbnQuXHJcblxyXG4gICAgdGhpcy5tX2JvZHlDID0gdGhpcy5tX2pvaW50MS5HZXRCb2R5QSgpO1xyXG4gICAgdGhpcy5tX2JvZHlBID0gdGhpcy5tX2pvaW50MS5HZXRCb2R5QigpO1xyXG5cclxuICAgIC8vIEdldCBnZW9tZXRyeSBvZiBqb2ludDFcclxuICAgIGNvbnN0IHhmQSA9IHRoaXMubV9ib2R5QS5tX3hmO1xyXG4gICAgY29uc3QgYUEgPSB0aGlzLm1fYm9keUEubV9zd2VlcC5hO1xyXG4gICAgY29uc3QgeGZDID0gdGhpcy5tX2JvZHlDLm1feGY7XHJcbiAgICBjb25zdCBhQyA9IHRoaXMubV9ib2R5Qy5tX3N3ZWVwLmE7XHJcblxyXG4gICAgaWYgKHRoaXMubV90eXBlQSA9PT0gYjJKb2ludFR5cGUuZV9yZXZvbHV0ZUpvaW50KSB7XHJcbiAgICAgIGNvbnN0IHJldm9sdXRlID0gZGVmLmpvaW50MSBhcyBiMlJldm9sdXRlSm9pbnQ7XHJcbiAgICAgIHRoaXMubV9sb2NhbEFuY2hvckMuQ29weShyZXZvbHV0ZS5tX2xvY2FsQW5jaG9yQSk7XHJcbiAgICAgIHRoaXMubV9sb2NhbEFuY2hvckEuQ29weShyZXZvbHV0ZS5tX2xvY2FsQW5jaG9yQik7XHJcbiAgICAgIHRoaXMubV9yZWZlcmVuY2VBbmdsZUEgPSByZXZvbHV0ZS5tX3JlZmVyZW5jZUFuZ2xlO1xyXG4gICAgICB0aGlzLm1fbG9jYWxBeGlzQy5TZXRaZXJvKCk7XHJcblxyXG4gICAgICBjb29yZGluYXRlQSA9IGFBIC0gYUMgLSB0aGlzLm1fcmVmZXJlbmNlQW5nbGVBO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgY29uc3QgcHJpc21hdGljID0gZGVmLmpvaW50MSBhcyBiMlByaXNtYXRpY0pvaW50O1xyXG4gICAgICB0aGlzLm1fbG9jYWxBbmNob3JDLkNvcHkocHJpc21hdGljLm1fbG9jYWxBbmNob3JBKTtcclxuICAgICAgdGhpcy5tX2xvY2FsQW5jaG9yQS5Db3B5KHByaXNtYXRpYy5tX2xvY2FsQW5jaG9yQik7XHJcbiAgICAgIHRoaXMubV9yZWZlcmVuY2VBbmdsZUEgPSBwcmlzbWF0aWMubV9yZWZlcmVuY2VBbmdsZTtcclxuICAgICAgdGhpcy5tX2xvY2FsQXhpc0MuQ29weShwcmlzbWF0aWMubV9sb2NhbFhBeGlzQSk7XHJcblxyXG4gICAgICAvLyBiMlZlYzIgcEMgPSBtX2xvY2FsQW5jaG9yQztcclxuICAgICAgY29uc3QgcEMgPSB0aGlzLm1fbG9jYWxBbmNob3JDO1xyXG4gICAgICAvLyBiMlZlYzIgcEEgPSBiMk11bFQoeGZDLnEsIGIyTXVsKHhmQS5xLCBtX2xvY2FsQW5jaG9yQSkgKyAoeGZBLnAgLSB4ZkMucCkpO1xyXG4gICAgICBjb25zdCBwQSA9IGIyUm90Lk11bFRSVihcclxuICAgICAgICB4ZkMucSxcclxuICAgICAgICBiMlZlYzIuQWRkVlYoXHJcbiAgICAgICAgICBiMlJvdC5NdWxSVih4ZkEucSwgdGhpcy5tX2xvY2FsQW5jaG9yQSwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgICAgYjJWZWMyLlN1YlZWKHhmQS5wLCB4ZkMucCwgYjJWZWMyLnNfdDEpLFxyXG4gICAgICAgICAgYjJWZWMyLnNfdDAsXHJcbiAgICAgICAgKSxcclxuICAgICAgICBiMlZlYzIuc190MCxcclxuICAgICAgKTsgLy8gcEEgdXNlcyBzX3QwXHJcbiAgICAgIC8vIGNvb3JkaW5hdGVBID0gYjJEb3QocEEgLSBwQywgbV9sb2NhbEF4aXNDKTtcclxuICAgICAgY29vcmRpbmF0ZUEgPSBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKHBBLCBwQywgYjJWZWMyLnNfdDApLCB0aGlzLm1fbG9jYWxBeGlzQyk7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5tX2JvZHlEID0gdGhpcy5tX2pvaW50Mi5HZXRCb2R5QSgpO1xyXG4gICAgdGhpcy5tX2JvZHlCID0gdGhpcy5tX2pvaW50Mi5HZXRCb2R5QigpO1xyXG5cclxuICAgIC8vIEdldCBnZW9tZXRyeSBvZiBqb2ludDJcclxuICAgIGNvbnN0IHhmQjogYjJUcmFuc2Zvcm0gPSB0aGlzLm1fYm9keUIubV94ZjtcclxuICAgIGNvbnN0IGFCOiBudW1iZXIgPSB0aGlzLm1fYm9keUIubV9zd2VlcC5hO1xyXG4gICAgY29uc3QgeGZEOiBiMlRyYW5zZm9ybSA9IHRoaXMubV9ib2R5RC5tX3hmO1xyXG4gICAgY29uc3QgYUQ6IG51bWJlciA9IHRoaXMubV9ib2R5RC5tX3N3ZWVwLmE7XHJcblxyXG4gICAgaWYgKHRoaXMubV90eXBlQiA9PT0gYjJKb2ludFR5cGUuZV9yZXZvbHV0ZUpvaW50KSB7XHJcbiAgICAgIGNvbnN0IHJldm9sdXRlID0gZGVmLmpvaW50MiBhcyBiMlJldm9sdXRlSm9pbnQ7XHJcbiAgICAgIHRoaXMubV9sb2NhbEFuY2hvckQuQ29weShyZXZvbHV0ZS5tX2xvY2FsQW5jaG9yQSk7XHJcbiAgICAgIHRoaXMubV9sb2NhbEFuY2hvckIuQ29weShyZXZvbHV0ZS5tX2xvY2FsQW5jaG9yQik7XHJcbiAgICAgIHRoaXMubV9yZWZlcmVuY2VBbmdsZUIgPSByZXZvbHV0ZS5tX3JlZmVyZW5jZUFuZ2xlO1xyXG4gICAgICB0aGlzLm1fbG9jYWxBeGlzRC5TZXRaZXJvKCk7XHJcblxyXG4gICAgICBjb29yZGluYXRlQiA9IGFCIC0gYUQgLSB0aGlzLm1fcmVmZXJlbmNlQW5nbGVCO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgY29uc3QgcHJpc21hdGljID0gZGVmLmpvaW50MiBhcyBiMlByaXNtYXRpY0pvaW50O1xyXG4gICAgICB0aGlzLm1fbG9jYWxBbmNob3JELkNvcHkocHJpc21hdGljLm1fbG9jYWxBbmNob3JBKTtcclxuICAgICAgdGhpcy5tX2xvY2FsQW5jaG9yQi5Db3B5KHByaXNtYXRpYy5tX2xvY2FsQW5jaG9yQik7XHJcbiAgICAgIHRoaXMubV9yZWZlcmVuY2VBbmdsZUIgPSBwcmlzbWF0aWMubV9yZWZlcmVuY2VBbmdsZTtcclxuICAgICAgdGhpcy5tX2xvY2FsQXhpc0QuQ29weShwcmlzbWF0aWMubV9sb2NhbFhBeGlzQSk7XHJcblxyXG4gICAgICAvLyBiMlZlYzIgcEQgPSBtX2xvY2FsQW5jaG9yRDtcclxuICAgICAgY29uc3QgcEQgPSB0aGlzLm1fbG9jYWxBbmNob3JEO1xyXG4gICAgICAvLyBiMlZlYzIgcEIgPSBiMk11bFQoeGZELnEsIGIyTXVsKHhmQi5xLCBtX2xvY2FsQW5jaG9yQikgKyAoeGZCLnAgLSB4ZkQucCkpO1xyXG4gICAgICBjb25zdCBwQjogYjJWZWMyID0gYjJSb3QuTXVsVFJWKFxyXG4gICAgICAgIHhmRC5xLFxyXG4gICAgICAgIGIyVmVjMi5BZGRWVihcclxuICAgICAgICAgIGIyUm90Lk11bFJWKHhmQi5xLCB0aGlzLm1fbG9jYWxBbmNob3JCLCBiMlZlYzIuc190MCksXHJcbiAgICAgICAgICBiMlZlYzIuU3ViVlYoeGZCLnAsIHhmRC5wLCBiMlZlYzIuc190MSksXHJcbiAgICAgICAgICBiMlZlYzIuc190MCxcclxuICAgICAgICApLFxyXG4gICAgICAgIGIyVmVjMi5zX3QwLFxyXG4gICAgICApOyAvLyBwQiB1c2VzIHNfdDBcclxuICAgICAgLy8gY29vcmRpbmF0ZUIgPSBiMkRvdChwQiAtIHBELCBtX2xvY2FsQXhpc0QpO1xyXG4gICAgICBjb29yZGluYXRlQiA9IGIyVmVjMi5Eb3RWVihiMlZlYzIuU3ViVlYocEIsIHBELCBiMlZlYzIuc190MCksIHRoaXMubV9sb2NhbEF4aXNEKTtcclxuICAgIH1cclxuXHJcbiAgICB0aGlzLm1fcmF0aW8gPSBiMk1heWJlKGRlZi5yYXRpbywgMSk7XHJcblxyXG4gICAgdGhpcy5tX2NvbnN0YW50ID0gY29vcmRpbmF0ZUEgKyB0aGlzLm1fcmF0aW8gKiBjb29yZGluYXRlQjtcclxuXHJcbiAgICB0aGlzLm1faW1wdWxzZSA9IDA7XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBJbml0VmVsb2NpdHlDb25zdHJhaW50c19zX3UgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgSW5pdFZlbG9jaXR5Q29uc3RyYWludHNfc19yQSA9IG5ldyBiMlZlYzIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBJbml0VmVsb2NpdHlDb25zdHJhaW50c19zX3JCID0gbmV3IGIyVmVjMigpO1xyXG4gIHByaXZhdGUgc3RhdGljIEluaXRWZWxvY2l0eUNvbnN0cmFpbnRzX3NfckMgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgSW5pdFZlbG9jaXR5Q29uc3RyYWludHNfc19yRCA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgSW5pdFZlbG9jaXR5Q29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogdm9pZCB7XHJcbiAgICB0aGlzLm1faW5kZXhBID0gdGhpcy5tX2JvZHlBLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1faW5kZXhCID0gdGhpcy5tX2JvZHlCLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1faW5kZXhDID0gdGhpcy5tX2JvZHlDLm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1faW5kZXhEID0gdGhpcy5tX2JvZHlELm1faXNsYW5kSW5kZXg7XHJcbiAgICB0aGlzLm1fbGNBLkNvcHkodGhpcy5tX2JvZHlBLm1fc3dlZXAubG9jYWxDZW50ZXIpO1xyXG4gICAgdGhpcy5tX2xjQi5Db3B5KHRoaXMubV9ib2R5Qi5tX3N3ZWVwLmxvY2FsQ2VudGVyKTtcclxuICAgIHRoaXMubV9sY0MuQ29weSh0aGlzLm1fYm9keUMubV9zd2VlcC5sb2NhbENlbnRlcik7XHJcbiAgICB0aGlzLm1fbGNELkNvcHkodGhpcy5tX2JvZHlELm1fc3dlZXAubG9jYWxDZW50ZXIpO1xyXG4gICAgdGhpcy5tX21BID0gdGhpcy5tX2JvZHlBLm1faW52TWFzcztcclxuICAgIHRoaXMubV9tQiA9IHRoaXMubV9ib2R5Qi5tX2ludk1hc3M7XHJcbiAgICB0aGlzLm1fbUMgPSB0aGlzLm1fYm9keUMubV9pbnZNYXNzO1xyXG4gICAgdGhpcy5tX21EID0gdGhpcy5tX2JvZHlELm1faW52TWFzcztcclxuICAgIHRoaXMubV9pQSA9IHRoaXMubV9ib2R5QS5tX2ludkk7XHJcbiAgICB0aGlzLm1faUIgPSB0aGlzLm1fYm9keUIubV9pbnZJO1xyXG4gICAgdGhpcy5tX2lDID0gdGhpcy5tX2JvZHlDLm1faW52STtcclxuICAgIHRoaXMubV9pRCA9IHRoaXMubV9ib2R5RC5tX2ludkk7XHJcblxyXG4gICAgY29uc3QgYUE6IG51bWJlciA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEFdLmE7XHJcbiAgICBjb25zdCB2QTogYjJWZWMyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLnY7XHJcbiAgICBsZXQgd0E6IG51bWJlciA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS53O1xyXG5cclxuICAgIGNvbnN0IGFCOiBudW1iZXIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5hO1xyXG4gICAgY29uc3QgdkI6IGIyVmVjMiA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52O1xyXG4gICAgbGV0IHdCOiBudW1iZXIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udztcclxuXHJcbiAgICBjb25zdCBhQzogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Q10uYTtcclxuICAgIGNvbnN0IHZDOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Q10udjtcclxuICAgIGxldCB3QzogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleENdLnc7XHJcblxyXG4gICAgY29uc3QgYUQ6IG51bWJlciA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleERdLmE7XHJcbiAgICBjb25zdCB2RDogYjJWZWMyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleERdLnY7XHJcbiAgICBsZXQgd0Q6IG51bWJlciA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhEXS53O1xyXG5cclxuICAgIC8vIGIyUm90IHFBKGFBKSwgcUIoYUIpLCBxQyhhQyksIHFEKGFEKTtcclxuICAgIGNvbnN0IHFBOiBiMlJvdCA9IHRoaXMubV9xQS5TZXRBbmdsZShhQSksXHJcbiAgICAgIHFCOiBiMlJvdCA9IHRoaXMubV9xQi5TZXRBbmdsZShhQiksXHJcbiAgICAgIHFDOiBiMlJvdCA9IHRoaXMubV9xQy5TZXRBbmdsZShhQyksXHJcbiAgICAgIHFEOiBiMlJvdCA9IHRoaXMubV9xRC5TZXRBbmdsZShhRCk7XHJcblxyXG4gICAgdGhpcy5tX21hc3MgPSAwO1xyXG5cclxuICAgIGlmICh0aGlzLm1fdHlwZUEgPT09IGIySm9pbnRUeXBlLmVfcmV2b2x1dGVKb2ludCkge1xyXG4gICAgICB0aGlzLm1fSnZBQy5TZXRaZXJvKCk7XHJcbiAgICAgIHRoaXMubV9Kd0EgPSAxO1xyXG4gICAgICB0aGlzLm1fSndDID0gMTtcclxuICAgICAgdGhpcy5tX21hc3MgKz0gdGhpcy5tX2lBICsgdGhpcy5tX2lDO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgLy8gYjJWZWMyIHUgPSBiMk11bChxQywgbV9sb2NhbEF4aXNDKTtcclxuICAgICAgY29uc3QgdTogYjJWZWMyID0gYjJSb3QuTXVsUlYocUMsIHRoaXMubV9sb2NhbEF4aXNDLCBiMkdlYXJKb2ludC5Jbml0VmVsb2NpdHlDb25zdHJhaW50c19zX3UpO1xyXG4gICAgICAvLyBiMlZlYzIgckMgPSBiMk11bChxQywgbV9sb2NhbEFuY2hvckMgLSBtX2xjQyk7XHJcbiAgICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JDLCB0aGlzLm1fbGNDLCB0aGlzLm1fbGFsY0MpO1xyXG4gICAgICBjb25zdCByQzogYjJWZWMyID0gYjJSb3QuTXVsUlYocUMsIHRoaXMubV9sYWxjQywgYjJHZWFySm9pbnQuSW5pdFZlbG9jaXR5Q29uc3RyYWludHNfc19yQyk7XHJcbiAgICAgIC8vIGIyVmVjMiByQSA9IGIyTXVsKHFBLCBtX2xvY2FsQW5jaG9yQSAtIG1fbGNBKTtcclxuICAgICAgYjJWZWMyLlN1YlZWKHRoaXMubV9sb2NhbEFuY2hvckEsIHRoaXMubV9sY0EsIHRoaXMubV9sYWxjQSk7XHJcbiAgICAgIGNvbnN0IHJBOiBiMlZlYzIgPSBiMlJvdC5NdWxSVihxQSwgdGhpcy5tX2xhbGNBLCBiMkdlYXJKb2ludC5Jbml0VmVsb2NpdHlDb25zdHJhaW50c19zX3JBKTtcclxuICAgICAgLy8gbV9KdkFDID0gdTtcclxuICAgICAgdGhpcy5tX0p2QUMuQ29weSh1KTtcclxuICAgICAgLy8gbV9Kd0MgPSBiMkNyb3NzKHJDLCB1KTtcclxuICAgICAgdGhpcy5tX0p3QyA9IGIyVmVjMi5Dcm9zc1ZWKHJDLCB1KTtcclxuICAgICAgLy8gbV9Kd0EgPSBiMkNyb3NzKHJBLCB1KTtcclxuICAgICAgdGhpcy5tX0p3QSA9IGIyVmVjMi5Dcm9zc1ZWKHJBLCB1KTtcclxuICAgICAgdGhpcy5tX21hc3MgKz1cclxuICAgICAgICB0aGlzLm1fbUMgK1xyXG4gICAgICAgIHRoaXMubV9tQSArXHJcbiAgICAgICAgdGhpcy5tX2lDICogdGhpcy5tX0p3QyAqIHRoaXMubV9Kd0MgK1xyXG4gICAgICAgIHRoaXMubV9pQSAqIHRoaXMubV9Kd0EgKiB0aGlzLm1fSndBO1xyXG4gICAgfVxyXG5cclxuICAgIGlmICh0aGlzLm1fdHlwZUIgPT09IGIySm9pbnRUeXBlLmVfcmV2b2x1dGVKb2ludCkge1xyXG4gICAgICB0aGlzLm1fSnZCRC5TZXRaZXJvKCk7XHJcbiAgICAgIHRoaXMubV9Kd0IgPSB0aGlzLm1fcmF0aW87XHJcbiAgICAgIHRoaXMubV9Kd0QgPSB0aGlzLm1fcmF0aW87XHJcbiAgICAgIHRoaXMubV9tYXNzICs9IHRoaXMubV9yYXRpbyAqIHRoaXMubV9yYXRpbyAqICh0aGlzLm1faUIgKyB0aGlzLm1faUQpO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgLy8gYjJWZWMyIHUgPSBiMk11bChxRCwgbV9sb2NhbEF4aXNEKTtcclxuICAgICAgY29uc3QgdTogYjJWZWMyID0gYjJSb3QuTXVsUlYocUQsIHRoaXMubV9sb2NhbEF4aXNELCBiMkdlYXJKb2ludC5Jbml0VmVsb2NpdHlDb25zdHJhaW50c19zX3UpO1xyXG4gICAgICAvLyBiMlZlYzIgckQgPSBiMk11bChxRCwgbV9sb2NhbEFuY2hvckQgLSBtX2xjRCk7XHJcbiAgICAgIGIyVmVjMi5TdWJWVih0aGlzLm1fbG9jYWxBbmNob3JELCB0aGlzLm1fbGNELCB0aGlzLm1fbGFsY0QpO1xyXG4gICAgICBjb25zdCByRDogYjJWZWMyID0gYjJSb3QuTXVsUlYocUQsIHRoaXMubV9sYWxjRCwgYjJHZWFySm9pbnQuSW5pdFZlbG9jaXR5Q29uc3RyYWludHNfc19yRCk7XHJcbiAgICAgIC8vIGIyVmVjMiByQiA9IGIyTXVsKHFCLCBtX2xvY2FsQW5jaG9yQiAtIG1fbGNCKTtcclxuICAgICAgYjJWZWMyLlN1YlZWKHRoaXMubV9sb2NhbEFuY2hvckIsIHRoaXMubV9sY0IsIHRoaXMubV9sYWxjQik7XHJcbiAgICAgIGNvbnN0IHJCOiBiMlZlYzIgPSBiMlJvdC5NdWxSVihxQiwgdGhpcy5tX2xhbGNCLCBiMkdlYXJKb2ludC5Jbml0VmVsb2NpdHlDb25zdHJhaW50c19zX3JCKTtcclxuICAgICAgLy8gbV9KdkJEID0gbV9yYXRpbyAqIHU7XHJcbiAgICAgIGIyVmVjMi5NdWxTVih0aGlzLm1fcmF0aW8sIHUsIHRoaXMubV9KdkJEKTtcclxuICAgICAgLy8gbV9Kd0QgPSBtX3JhdGlvICogYjJDcm9zcyhyRCwgdSk7XHJcbiAgICAgIHRoaXMubV9Kd0QgPSB0aGlzLm1fcmF0aW8gKiBiMlZlYzIuQ3Jvc3NWVihyRCwgdSk7XHJcbiAgICAgIC8vIG1fSndCID0gbV9yYXRpbyAqIGIyQ3Jvc3MockIsIHUpO1xyXG4gICAgICB0aGlzLm1fSndCID0gdGhpcy5tX3JhdGlvICogYjJWZWMyLkNyb3NzVlYockIsIHUpO1xyXG4gICAgICB0aGlzLm1fbWFzcyArPVxyXG4gICAgICAgIHRoaXMubV9yYXRpbyAqIHRoaXMubV9yYXRpbyAqICh0aGlzLm1fbUQgKyB0aGlzLm1fbUIpICtcclxuICAgICAgICB0aGlzLm1faUQgKiB0aGlzLm1fSndEICogdGhpcy5tX0p3RCArXHJcbiAgICAgICAgdGhpcy5tX2lCICogdGhpcy5tX0p3QiAqIHRoaXMubV9Kd0I7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gQ29tcHV0ZSBlZmZlY3RpdmUgbWFzcy5cclxuICAgIHRoaXMubV9tYXNzID0gdGhpcy5tX21hc3MgPiAwID8gMSAvIHRoaXMubV9tYXNzIDogMDtcclxuXHJcbiAgICBpZiAoZGF0YS5zdGVwLndhcm1TdGFydGluZykge1xyXG4gICAgICAvLyB2QSArPSAobV9tQSAqIG1faW1wdWxzZSkgKiBtX0p2QUM7XHJcbiAgICAgIHZBLlNlbGZNdWxBZGQodGhpcy5tX21BICogdGhpcy5tX2ltcHVsc2UsIHRoaXMubV9KdkFDKTtcclxuICAgICAgd0EgKz0gdGhpcy5tX2lBICogdGhpcy5tX2ltcHVsc2UgKiB0aGlzLm1fSndBO1xyXG4gICAgICAvLyB2QiArPSAobV9tQiAqIG1faW1wdWxzZSkgKiBtX0p2QkQ7XHJcbiAgICAgIHZCLlNlbGZNdWxBZGQodGhpcy5tX21CICogdGhpcy5tX2ltcHVsc2UsIHRoaXMubV9KdkJEKTtcclxuICAgICAgd0IgKz0gdGhpcy5tX2lCICogdGhpcy5tX2ltcHVsc2UgKiB0aGlzLm1fSndCO1xyXG4gICAgICAvLyB2QyAtPSAobV9tQyAqIG1faW1wdWxzZSkgKiBtX0p2QUM7XHJcbiAgICAgIHZDLlNlbGZNdWxTdWIodGhpcy5tX21DICogdGhpcy5tX2ltcHVsc2UsIHRoaXMubV9KdkFDKTtcclxuICAgICAgd0MgLT0gdGhpcy5tX2lDICogdGhpcy5tX2ltcHVsc2UgKiB0aGlzLm1fSndDO1xyXG4gICAgICAvLyB2RCAtPSAobV9tRCAqIG1faW1wdWxzZSkgKiBtX0p2QkQ7XHJcbiAgICAgIHZELlNlbGZNdWxTdWIodGhpcy5tX21EICogdGhpcy5tX2ltcHVsc2UsIHRoaXMubV9KdkJEKTtcclxuICAgICAgd0QgLT0gdGhpcy5tX2lEICogdGhpcy5tX2ltcHVsc2UgKiB0aGlzLm1fSndEO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgdGhpcy5tX2ltcHVsc2UgPSAwO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS52ID0gdkE7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udyA9IHdBO1xyXG4gICAgLy8gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnYgPSB2QjtcclxuICAgIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS53ID0gd0I7XHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Q10udiA9IHZDO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleENdLncgPSB3QztcclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhEXS52ID0gdkQ7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4RF0udyA9IHdEO1xyXG4gIH1cclxuXHJcbiAgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzKGRhdGE6IGIyU29sdmVyRGF0YSk6IHZvaWQge1xyXG4gICAgY29uc3QgdkE6IGIyVmVjMiA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhBXS52O1xyXG4gICAgbGV0IHdBOiBudW1iZXIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udztcclxuICAgIGNvbnN0IHZCOiBiMlZlYzIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udjtcclxuICAgIGxldCB3QjogbnVtYmVyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEJdLnc7XHJcbiAgICBjb25zdCB2QzogYjJWZWMyID0gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleENdLnY7XHJcbiAgICBsZXQgd0M6IG51bWJlciA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhDXS53O1xyXG4gICAgY29uc3QgdkQ6IGIyVmVjMiA9IGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhEXS52O1xyXG4gICAgbGV0IHdEOiBudW1iZXIgPSBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4RF0udztcclxuXHJcbiAgICAvLyBmbG9hdDMyIENkb3QgPSBiMkRvdChtX0p2QUMsIHZBIC0gdkMpICsgYjJEb3QobV9KdkJELCB2QiAtIHZEKTtcclxuICAgIGxldCBDZG90ID1cclxuICAgICAgYjJWZWMyLkRvdFZWKHRoaXMubV9KdkFDLCBiMlZlYzIuU3ViVlYodkEsIHZDLCBiMlZlYzIuc190MCkpICtcclxuICAgICAgYjJWZWMyLkRvdFZWKHRoaXMubV9KdkJELCBiMlZlYzIuU3ViVlYodkIsIHZELCBiMlZlYzIuc190MCkpO1xyXG4gICAgQ2RvdCArPSB0aGlzLm1fSndBICogd0EgLSB0aGlzLm1fSndDICogd0MgKyAodGhpcy5tX0p3QiAqIHdCIC0gdGhpcy5tX0p3RCAqIHdEKTtcclxuXHJcbiAgICBjb25zdCBpbXB1bHNlOiBudW1iZXIgPSAtdGhpcy5tX21hc3MgKiBDZG90O1xyXG4gICAgdGhpcy5tX2ltcHVsc2UgKz0gaW1wdWxzZTtcclxuXHJcbiAgICAvLyB2QSArPSAobV9tQSAqIGltcHVsc2UpICogbV9KdkFDO1xyXG4gICAgdkEuU2VsZk11bEFkZCh0aGlzLm1fbUEgKiBpbXB1bHNlLCB0aGlzLm1fSnZBQyk7XHJcbiAgICB3QSArPSB0aGlzLm1faUEgKiBpbXB1bHNlICogdGhpcy5tX0p3QTtcclxuICAgIC8vIHZCICs9IChtX21CICogaW1wdWxzZSkgKiBtX0p2QkQ7XHJcbiAgICB2Qi5TZWxmTXVsQWRkKHRoaXMubV9tQiAqIGltcHVsc2UsIHRoaXMubV9KdkJEKTtcclxuICAgIHdCICs9IHRoaXMubV9pQiAqIGltcHVsc2UgKiB0aGlzLm1fSndCO1xyXG4gICAgLy8gdkMgLT0gKG1fbUMgKiBpbXB1bHNlKSAqIG1fSnZBQztcclxuICAgIHZDLlNlbGZNdWxTdWIodGhpcy5tX21DICogaW1wdWxzZSwgdGhpcy5tX0p2QUMpO1xyXG4gICAgd0MgLT0gdGhpcy5tX2lDICogaW1wdWxzZSAqIHRoaXMubV9Kd0M7XHJcbiAgICAvLyB2RCAtPSAobV9tRCAqIGltcHVsc2UpICogbV9KdkJEO1xyXG4gICAgdkQuU2VsZk11bFN1Yih0aGlzLm1fbUQgKiBpbXB1bHNlLCB0aGlzLm1fSnZCRCk7XHJcbiAgICB3RCAtPSB0aGlzLm1faUQgKiBpbXB1bHNlICogdGhpcy5tX0p3RDtcclxuXHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4QV0udiA9IHZBO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleEFdLncgPSB3QTtcclxuICAgIC8vIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhCXS52ID0gdkI7XHJcbiAgICBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4Ql0udyA9IHdCO1xyXG4gICAgLy8gZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleENdLnYgPSB2QztcclxuICAgIGRhdGEudmVsb2NpdGllc1t0aGlzLm1faW5kZXhDXS53ID0gd0M7XHJcbiAgICAvLyBkYXRhLnZlbG9jaXRpZXNbdGhpcy5tX2luZGV4RF0udiA9IHZEO1xyXG4gICAgZGF0YS52ZWxvY2l0aWVzW3RoaXMubV9pbmRleERdLncgPSB3RDtcclxuICB9XHJcblxyXG4gIHByaXZhdGUgc3RhdGljIFNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX3UgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckEgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckMgPSBuZXcgYjJWZWMyKCk7XHJcbiAgcHJpdmF0ZSBzdGF0aWMgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckQgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIFNvbHZlUG9zaXRpb25Db25zdHJhaW50cyhkYXRhOiBiMlNvbHZlckRhdGEpOiBib29sZWFuIHtcclxuICAgIGNvbnN0IGNBOiBiMlZlYzIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5jO1xyXG4gICAgbGV0IGFBOiBudW1iZXIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5hO1xyXG4gICAgY29uc3QgY0I6IGIyVmVjMiA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmM7XHJcbiAgICBsZXQgYUI6IG51bWJlciA9IGRhdGEucG9zaXRpb25zW3RoaXMubV9pbmRleEJdLmE7XHJcbiAgICBjb25zdCBjQzogYjJWZWMyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Q10uYztcclxuICAgIGxldCBhQzogbnVtYmVyID0gZGF0YS5wb3NpdGlvbnNbdGhpcy5tX2luZGV4Q10uYTtcclxuICAgIGNvbnN0IGNEOiBiMlZlYzIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhEXS5jO1xyXG4gICAgbGV0IGFEOiBudW1iZXIgPSBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhEXS5hO1xyXG5cclxuICAgIC8vIGIyUm90IHFBKGFBKSwgcUIoYUIpLCBxQyhhQyksIHFEKGFEKTtcclxuICAgIGNvbnN0IHFBOiBiMlJvdCA9IHRoaXMubV9xQS5TZXRBbmdsZShhQSksXHJcbiAgICAgIHFCOiBiMlJvdCA9IHRoaXMubV9xQi5TZXRBbmdsZShhQiksXHJcbiAgICAgIHFDOiBiMlJvdCA9IHRoaXMubV9xQy5TZXRBbmdsZShhQyksXHJcbiAgICAgIHFEOiBiMlJvdCA9IHRoaXMubV9xRC5TZXRBbmdsZShhRCk7XHJcblxyXG4gICAgY29uc3QgbGluZWFyRXJyb3IgPSAwO1xyXG5cclxuICAgIGxldCBjb29yZGluYXRlQTogbnVtYmVyLCBjb29yZGluYXRlQjogbnVtYmVyO1xyXG5cclxuICAgIGNvbnN0IEp2QUM6IGIyVmVjMiA9IHRoaXMubV9KdkFDLFxyXG4gICAgICBKdkJEOiBiMlZlYzIgPSB0aGlzLm1fSnZCRDtcclxuICAgIGxldCBKd0E6IG51bWJlciwgSndCOiBudW1iZXIsIEp3QzogbnVtYmVyLCBKd0Q6IG51bWJlcjtcclxuICAgIGxldCBtYXNzID0gMDtcclxuXHJcbiAgICBpZiAodGhpcy5tX3R5cGVBID09PSBiMkpvaW50VHlwZS5lX3Jldm9sdXRlSm9pbnQpIHtcclxuICAgICAgSnZBQy5TZXRaZXJvKCk7XHJcbiAgICAgIEp3QSA9IDE7XHJcbiAgICAgIEp3QyA9IDE7XHJcbiAgICAgIG1hc3MgKz0gdGhpcy5tX2lBICsgdGhpcy5tX2lDO1xyXG5cclxuICAgICAgY29vcmRpbmF0ZUEgPSBhQSAtIGFDIC0gdGhpcy5tX3JlZmVyZW5jZUFuZ2xlQTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIC8vIGIyVmVjMiB1ID0gYjJNdWwocUMsIG1fbG9jYWxBeGlzQyk7XHJcbiAgICAgIGNvbnN0IHU6IGIyVmVjMiA9IGIyUm90Lk11bFJWKFxyXG4gICAgICAgIHFDLFxyXG4gICAgICAgIHRoaXMubV9sb2NhbEF4aXNDLFxyXG4gICAgICAgIGIyR2VhckpvaW50LlNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX3UsXHJcbiAgICAgICk7XHJcbiAgICAgIC8vIGIyVmVjMiByQyA9IGIyTXVsKHFDLCBtX2xvY2FsQW5jaG9yQyAtIG1fbGNDKTtcclxuICAgICAgY29uc3QgckM6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFDLCB0aGlzLm1fbGFsY0MsIGIyR2VhckpvaW50LlNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX3JDKTtcclxuICAgICAgLy8gYjJWZWMyIHJBID0gYjJNdWwocUEsIG1fbG9jYWxBbmNob3JBIC0gbV9sY0EpO1xyXG4gICAgICBjb25zdCByQTogYjJWZWMyID0gYjJSb3QuTXVsUlYocUEsIHRoaXMubV9sYWxjQSwgYjJHZWFySm9pbnQuU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckEpO1xyXG4gICAgICAvLyBKdkFDID0gdTtcclxuICAgICAgSnZBQy5Db3B5KHUpO1xyXG4gICAgICAvLyBKd0MgPSBiMkNyb3NzKHJDLCB1KTtcclxuICAgICAgSndDID0gYjJWZWMyLkNyb3NzVlYockMsIHUpO1xyXG4gICAgICAvLyBKd0EgPSBiMkNyb3NzKHJBLCB1KTtcclxuICAgICAgSndBID0gYjJWZWMyLkNyb3NzVlYockEsIHUpO1xyXG4gICAgICBtYXNzICs9IHRoaXMubV9tQyArIHRoaXMubV9tQSArIHRoaXMubV9pQyAqIEp3QyAqIEp3QyArIHRoaXMubV9pQSAqIEp3QSAqIEp3QTtcclxuXHJcbiAgICAgIC8vIGIyVmVjMiBwQyA9IG1fbG9jYWxBbmNob3JDIC0gbV9sY0M7XHJcbiAgICAgIGNvbnN0IHBDID0gdGhpcy5tX2xhbGNDO1xyXG4gICAgICAvLyBiMlZlYzIgcEEgPSBiMk11bFQocUMsIHJBICsgKGNBIC0gY0MpKTtcclxuICAgICAgY29uc3QgcEE6IGIyVmVjMiA9IGIyUm90Lk11bFRSVihcclxuICAgICAgICBxQyxcclxuICAgICAgICBiMlZlYzIuQWRkVlYockEsIGIyVmVjMi5TdWJWVihjQSwgY0MsIGIyVmVjMi5zX3QwKSwgYjJWZWMyLnNfdDApLFxyXG4gICAgICAgIGIyVmVjMi5zX3QwLFxyXG4gICAgICApOyAvLyBwQSB1c2VzIHNfdDBcclxuICAgICAgLy8gY29vcmRpbmF0ZUEgPSBiMkRvdChwQSAtIHBDLCBtX2xvY2FsQXhpc0MpO1xyXG4gICAgICBjb29yZGluYXRlQSA9IGIyVmVjMi5Eb3RWVihiMlZlYzIuU3ViVlYocEEsIHBDLCBiMlZlYzIuc190MCksIHRoaXMubV9sb2NhbEF4aXNDKTtcclxuICAgIH1cclxuXHJcbiAgICBpZiAodGhpcy5tX3R5cGVCID09PSBiMkpvaW50VHlwZS5lX3Jldm9sdXRlSm9pbnQpIHtcclxuICAgICAgSnZCRC5TZXRaZXJvKCk7XHJcbiAgICAgIEp3QiA9IHRoaXMubV9yYXRpbztcclxuICAgICAgSndEID0gdGhpcy5tX3JhdGlvO1xyXG4gICAgICBtYXNzICs9IHRoaXMubV9yYXRpbyAqIHRoaXMubV9yYXRpbyAqICh0aGlzLm1faUIgKyB0aGlzLm1faUQpO1xyXG5cclxuICAgICAgY29vcmRpbmF0ZUIgPSBhQiAtIGFEIC0gdGhpcy5tX3JlZmVyZW5jZUFuZ2xlQjtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIC8vIGIyVmVjMiB1ID0gYjJNdWwocUQsIG1fbG9jYWxBeGlzRCk7XHJcbiAgICAgIGNvbnN0IHU6IGIyVmVjMiA9IGIyUm90Lk11bFJWKFxyXG4gICAgICAgIHFELFxyXG4gICAgICAgIHRoaXMubV9sb2NhbEF4aXNELFxyXG4gICAgICAgIGIyR2VhckpvaW50LlNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX3UsXHJcbiAgICAgICk7XHJcbiAgICAgIC8vIGIyVmVjMiByRCA9IGIyTXVsKHFELCBtX2xvY2FsQW5jaG9yRCAtIG1fbGNEKTtcclxuICAgICAgY29uc3QgckQ6IGIyVmVjMiA9IGIyUm90Lk11bFJWKHFELCB0aGlzLm1fbGFsY0QsIGIyR2VhckpvaW50LlNvbHZlUG9zaXRpb25Db25zdHJhaW50c19zX3JEKTtcclxuICAgICAgLy8gYjJWZWMyIHJCID0gYjJNdWwocUIsIG1fbG9jYWxBbmNob3JCIC0gbV9sY0IpO1xyXG4gICAgICBjb25zdCByQjogYjJWZWMyID0gYjJSb3QuTXVsUlYocUIsIHRoaXMubV9sYWxjQiwgYjJHZWFySm9pbnQuU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzX3NfckIpO1xyXG4gICAgICAvLyBKdkJEID0gbV9yYXRpbyAqIHU7XHJcbiAgICAgIGIyVmVjMi5NdWxTVih0aGlzLm1fcmF0aW8sIHUsIEp2QkQpO1xyXG4gICAgICAvLyBKd0QgPSBtX3JhdGlvICogYjJDcm9zcyhyRCwgdSk7XHJcbiAgICAgIEp3RCA9IHRoaXMubV9yYXRpbyAqIGIyVmVjMi5Dcm9zc1ZWKHJELCB1KTtcclxuICAgICAgLy8gSndCID0gbV9yYXRpbyAqIGIyQ3Jvc3MockIsIHUpO1xyXG4gICAgICBKd0IgPSB0aGlzLm1fcmF0aW8gKiBiMlZlYzIuQ3Jvc3NWVihyQiwgdSk7XHJcbiAgICAgIG1hc3MgKz1cclxuICAgICAgICB0aGlzLm1fcmF0aW8gKiB0aGlzLm1fcmF0aW8gKiAodGhpcy5tX21EICsgdGhpcy5tX21CKSArXHJcbiAgICAgICAgdGhpcy5tX2lEICogSndEICogSndEICtcclxuICAgICAgICB0aGlzLm1faUIgKiBKd0IgKiBKd0I7XHJcblxyXG4gICAgICAvLyBiMlZlYzIgcEQgPSBtX2xvY2FsQW5jaG9yRCAtIG1fbGNEO1xyXG4gICAgICBjb25zdCBwRCA9IHRoaXMubV9sYWxjRDtcclxuICAgICAgLy8gYjJWZWMyIHBCID0gYjJNdWxUKHFELCByQiArIChjQiAtIGNEKSk7XHJcbiAgICAgIGNvbnN0IHBCOiBiMlZlYzIgPSBiMlJvdC5NdWxUUlYoXHJcbiAgICAgICAgcUQsXHJcbiAgICAgICAgYjJWZWMyLkFkZFZWKHJCLCBiMlZlYzIuU3ViVlYoY0IsIGNELCBiMlZlYzIuc190MCksIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICBiMlZlYzIuc190MCxcclxuICAgICAgKTsgLy8gcEIgdXNlcyBzX3QwXHJcbiAgICAgIC8vIGNvb3JkaW5hdGVCID0gYjJEb3QocEIgLSBwRCwgbV9sb2NhbEF4aXNEKTtcclxuICAgICAgY29vcmRpbmF0ZUIgPSBiMlZlYzIuRG90VlYoYjJWZWMyLlN1YlZWKHBCLCBwRCwgYjJWZWMyLnNfdDApLCB0aGlzLm1fbG9jYWxBeGlzRCk7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgQzogbnVtYmVyID0gY29vcmRpbmF0ZUEgKyB0aGlzLm1fcmF0aW8gKiBjb29yZGluYXRlQiAtIHRoaXMubV9jb25zdGFudDtcclxuXHJcbiAgICBsZXQgaW1wdWxzZSA9IDA7XHJcbiAgICBpZiAobWFzcyA+IDApIHtcclxuICAgICAgaW1wdWxzZSA9IC1DIC8gbWFzcztcclxuICAgIH1cclxuXHJcbiAgICAvLyBjQSArPSBtX21BICogaW1wdWxzZSAqIEp2QUM7XHJcbiAgICBjQS5TZWxmTXVsQWRkKHRoaXMubV9tQSAqIGltcHVsc2UsIEp2QUMpO1xyXG4gICAgYUEgKz0gdGhpcy5tX2lBICogaW1wdWxzZSAqIEp3QTtcclxuICAgIC8vIGNCICs9IG1fbUIgKiBpbXB1bHNlICogSnZCRDtcclxuICAgIGNCLlNlbGZNdWxBZGQodGhpcy5tX21CICogaW1wdWxzZSwgSnZCRCk7XHJcbiAgICBhQiArPSB0aGlzLm1faUIgKiBpbXB1bHNlICogSndCO1xyXG4gICAgLy8gY0MgLT0gbV9tQyAqIGltcHVsc2UgKiBKdkFDO1xyXG4gICAgY0MuU2VsZk11bFN1Yih0aGlzLm1fbUMgKiBpbXB1bHNlLCBKdkFDKTtcclxuICAgIGFDIC09IHRoaXMubV9pQyAqIGltcHVsc2UgKiBKd0M7XHJcbiAgICAvLyBjRCAtPSBtX21EICogaW1wdWxzZSAqIEp2QkQ7XHJcbiAgICBjRC5TZWxmTXVsU3ViKHRoaXMubV9tRCAqIGltcHVsc2UsIEp2QkQpO1xyXG4gICAgYUQgLT0gdGhpcy5tX2lEICogaW1wdWxzZSAqIEp3RDtcclxuXHJcbiAgICAvLyBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5jID0gY0E7XHJcbiAgICBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhBXS5hID0gYUE7XHJcbiAgICAvLyBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5jID0gY0I7XHJcbiAgICBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhCXS5hID0gYUI7XHJcbiAgICAvLyBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhDXS5jID0gY0M7XHJcbiAgICBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhDXS5hID0gYUM7XHJcbiAgICAvLyBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhEXS5jID0gY0Q7XHJcbiAgICBkYXRhLnBvc2l0aW9uc1t0aGlzLm1faW5kZXhEXS5hID0gYUQ7XHJcblxyXG4gICAgLy8gVE9ET19FUklOIG5vdCBpbXBsZW1lbnRlZFxyXG4gICAgcmV0dXJuIGxpbmVhckVycm9yIDwgYjJfbGluZWFyU2xvcDtcclxuICB9XHJcblxyXG4gIEdldEFuY2hvckE8VCBleHRlbmRzIFhZPihvdXQ6IFQpOiBUIHtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUEuR2V0V29ybGRQb2ludCh0aGlzLm1fbG9jYWxBbmNob3JBLCBvdXQpO1xyXG4gIH1cclxuXHJcbiAgR2V0QW5jaG9yQjxUIGV4dGVuZHMgWFk+KG91dDogVCk6IFQge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ib2R5Qi5HZXRXb3JsZFBvaW50KHRoaXMubV9sb2NhbEFuY2hvckIsIG91dCk7XHJcbiAgfVxyXG5cclxuICBHZXRSZWFjdGlvbkZvcmNlPFQgZXh0ZW5kcyBYWT4oaW52X2R0OiBudW1iZXIsIG91dDogVCk6IFQge1xyXG4gICAgLy8gYjJWZWMyIFAgPSBtX2ltcHVsc2UgKiBtX0p2QUM7XHJcbiAgICAvLyByZXR1cm4gaW52X2R0ICogUDtcclxuICAgIHJldHVybiBiMlZlYzIuTXVsU1YoaW52X2R0ICogdGhpcy5tX2ltcHVsc2UsIHRoaXMubV9KdkFDLCBvdXQpO1xyXG4gIH1cclxuXHJcbiAgR2V0UmVhY3Rpb25Ub3JxdWUoaW52X2R0OiBudW1iZXIpOiBudW1iZXIge1xyXG4gICAgLy8gZmxvYXQzMiBMID0gbV9pbXB1bHNlICogbV9Kd0E7XHJcbiAgICAvLyByZXR1cm4gaW52X2R0ICogTDtcclxuICAgIHJldHVybiBpbnZfZHQgKiB0aGlzLm1faW1wdWxzZSAqIHRoaXMubV9Kd0E7XHJcbiAgfVxyXG5cclxuICBHZXRKb2ludDEoKSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2pvaW50MTtcclxuICB9XHJcblxyXG4gIEdldEpvaW50MigpIHtcclxuICAgIHJldHVybiB0aGlzLm1fam9pbnQyO1xyXG4gIH1cclxuXHJcbiAgR2V0UmF0aW8oKSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3JhdGlvO1xyXG4gIH1cclxuXHJcbiAgU2V0UmF0aW8ocmF0aW86IG51bWJlcik6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChiMklzVmFsaWQocmF0aW8pKTtcclxuICAgIHRoaXMubV9yYXRpbyA9IHJhdGlvO1xyXG4gIH1cclxufVxyXG4iXX0=