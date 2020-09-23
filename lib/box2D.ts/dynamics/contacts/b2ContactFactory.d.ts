import { b2Contact } from './b2Contact';
import { b2Fixture } from '../b2Fixture';
export declare class b2ContactFactory {
    readonly m_registers: number[];
    pool: b2Contact[];
    constructor();
    private createFromPool;
    private destroyToPool;
    private AddType;
    private InitializeRegisters;
    Create(fixtureA: b2Fixture, indexA: number, fixtureB: b2Fixture, indexB: number): b2Contact | null;
    Destroy(contact: b2Contact): void;
}
