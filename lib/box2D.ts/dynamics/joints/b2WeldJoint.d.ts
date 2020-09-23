import { b2Mat33, b2Rot, b2Vec2, b2Vec3, XY } from '../../common/b2Math';
import { b2Body } from '../b2Body';
import { b2IJointDef, b2Joint, b2JointDef } from './b2Joint';
import { b2SolverData } from '../b2TimeStep';
export interface b2IWeldJointDef extends b2IJointDef {
    localAnchorA?: XY;
    localAnchorB?: XY;
    referenceAngle?: number;
    frequencyHz?: number;
    dampingRatio?: number;
}
export declare class b2WeldJointDef extends b2JointDef implements b2IWeldJointDef {
    readonly localAnchorA: b2Vec2;
    readonly localAnchorB: b2Vec2;
    referenceAngle: number;
    frequencyHz: number;
    dampingRatio: number;
    constructor();
    Initialize(bA: b2Body, bB: b2Body, anchor: b2Vec2): void;
}
export declare class b2WeldJoint extends b2Joint {
    m_frequencyHz: number;
    m_dampingRatio: number;
    m_bias: number;
    readonly m_localAnchorA: b2Vec2;
    readonly m_localAnchorB: b2Vec2;
    m_referenceAngle: number;
    m_gamma: number;
    readonly m_impulse: b2Vec3;
    m_indexA: number;
    m_indexB: number;
    readonly m_rA: b2Vec2;
    readonly m_rB: b2Vec2;
    readonly m_localCenterA: b2Vec2;
    readonly m_localCenterB: b2Vec2;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    readonly m_mass: b2Mat33;
    readonly m_qA: b2Rot;
    readonly m_qB: b2Rot;
    readonly m_lalcA: b2Vec2;
    readonly m_lalcB: b2Vec2;
    readonly m_K: b2Mat33;
    constructor(def: b2IWeldJointDef);
    private static InitVelocityConstraints_s_P;
    InitVelocityConstraints(data: b2SolverData): void;
    private static SolveVelocityConstraints_s_Cdot1;
    private static SolveVelocityConstraints_s_impulse1;
    private static SolveVelocityConstraints_s_impulse;
    private static SolveVelocityConstraints_s_P;
    SolveVelocityConstraints(data: b2SolverData): void;
    private static SolvePositionConstraints_s_C1;
    private static SolvePositionConstraints_s_P;
    private static SolvePositionConstraints_s_impulse;
    SolvePositionConstraints(data: b2SolverData): boolean;
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    GetLocalAnchorA(): Readonly<b2Vec2>;
    GetLocalAnchorB(): Readonly<b2Vec2>;
    GetReferenceAngle(): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
}