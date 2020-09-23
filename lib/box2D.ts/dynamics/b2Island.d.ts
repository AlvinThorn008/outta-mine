import { b2Vec2 } from '../common/b2Math';
import { b2Contact } from './contacts/b2Contact';
import { b2ContactVelocityConstraint } from './contacts/b2ContactSolver';
import { b2Joint } from './joints/b2Joint';
import { b2Body } from './b2Body';
import { b2Position, b2Profile, b2TimeStep, b2Velocity } from './b2TimeStep';
import { b2ContactListener } from './b2WorldCallbacks';
export declare class b2Island {
    m_listener: b2ContactListener;
    readonly m_bodies: b2Body[];
    readonly m_contacts: b2Contact[];
    readonly m_joints: b2Joint[];
    readonly m_positions: b2Position[];
    readonly m_velocities: b2Velocity[];
    m_bodyCount: number;
    m_jointCount: number;
    m_contactCount: number;
    m_bodyCapacity: number;
    m_contactCapacity: number;
    m_jointCapacity: number;
    Initialize(bodyCapacity: number, contactCapacity: number, jointCapacity: number, listener: b2ContactListener): void;
    Clear(): void;
    AddBody(body: b2Body): void;
    AddContact(contact: b2Contact): void;
    AddJoint(joint: b2Joint): void;
    Solve(profile: b2Profile, step: b2TimeStep, gravity: b2Vec2, allowSleep: boolean): number;
    private _SolvePositionsConstraits;
    private _SolveInitJoints;
    private _UpdateSleepTime;
    SleepAll(): void;
    SolveTOI(subStep: b2TimeStep, toiIndexA: number, toiIndexB: number): void;
    private static s_impulse;
    Report(constraints: b2ContactVelocityConstraint[]): void;
}
