import { b2Sweep } from '../../common/b2Math';
import { b2Manifold, b2WorldManifold } from '../../collision/b2Collision';
import { b2Body } from '../b2Body';
import { b2Fixture } from '../b2Fixture';
import { b2Shape } from '../../collision/shapes/b2Shape';
import { b2ContactListener } from '../b2WorldCallbacks';
export declare class b2ContactEdge {
    readonly contact: b2Contact;
    private _other;
    prev: b2ContactEdge | null;
    next: b2ContactEdge | null;
    constructor(contact: b2Contact);
    Reset(): void;
    get other(): b2Body;
    set other(value: b2Body);
}
export declare class b2Contact<A extends b2Shape = b2Shape, B extends b2Shape = b2Shape> {
    m_prev: b2Contact | null;
    m_next: b2Contact | null;
    readonly m_nodeA: b2ContactEdge;
    readonly m_nodeB: b2ContactEdge;
    m_fixtureA: b2Fixture;
    m_fixtureB: b2Fixture;
    m_indexA: number;
    m_indexB: number;
    m_manifold: b2Manifold;
    m_toiCount: number;
    m_toi: number;
    m_friction: number;
    m_restitution: number;
    m_tangentSpeed: number;
    m_oldManifold: b2Manifold;
    m_islandFlag: boolean;
    m_touchingFlag: boolean;
    m_enabledFlag: boolean;
    m_filterFlag: boolean;
    m_bulletHitFlag: boolean;
    m_toiFlag: boolean;
    constructor();
    GetManifold(): b2Manifold;
    GetWorldManifold(worldManifold: b2WorldManifold): void;
    IsTouching(): boolean;
    SetEnabled(flag: boolean): void;
    IsEnabled(): boolean;
    GetNext(): b2Contact | null;
    GetFixtureA(): b2Fixture;
    GetChildIndexA(): number;
    GetShapeA(): A;
    GetFixtureB(): b2Fixture;
    GetChildIndexB(): number;
    GetShapeB(): B;
    FlagForFiltering(): void;
    SetFriction(friction: number): void;
    GetFriction(): number;
    ResetFriction(): void;
    SetRestitution(restitution: number): void;
    GetRestitution(): number;
    ResetRestitution(): void;
    SetTangentSpeed(speed: number): void;
    GetTangentSpeed(): number;
    Reset(fixtureA: b2Fixture, indexA: number, fixtureB: b2Fixture, indexB: number): void;
    Update(listener: b2ContactListener): void;
    ComputeTOI(sweepA: b2Sweep, sweepB: b2Sweep): number;
}
