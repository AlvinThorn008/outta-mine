import { b2BroadPhase } from '../collision/b2BroadPhase';
import { b2Contact } from './contacts/b2Contact';
import { b2ContactFactory } from './contacts/b2ContactFactory';
import { b2FixtureProxy } from './b2Fixture';
import { b2ContactFilter, b2ContactListener } from './b2WorldCallbacks';
export declare class b2ContactManager {
    readonly m_broadPhase: b2BroadPhase<b2FixtureProxy>;
    m_contactList: b2Contact | null;
    m_contactCount: number;
    m_contactFilter: b2ContactFilter;
    m_contactListener: b2ContactListener;
    readonly m_contactFactory: b2ContactFactory;
    AddPair(proxyA: b2FixtureProxy, proxyB: b2FixtureProxy): void;
    static s_updatePairs_proxyA: b2FixtureProxy[];
    static s_updatePairs_proxyB: b2FixtureProxy[];
    FindNewContacts(): void;
    Destroy(c: b2Contact): void;
    Collide(): void;
}
