import { b2Controller } from './b2Controller';
import { b2Vec2 } from '../common/b2Math';
import { b2TimeStep } from '../dynamics/b2TimeStep';
import { b2Draw } from '../common/b2Draw';
/**
 * Applies a force every frame
 */
export declare class b2ConstantAccelController extends b2Controller {
    /**
     * The acceleration to apply
     */
    readonly A: b2Vec2;
    Step(step: b2TimeStep): void;
    private static Step_s_dtA;
    Draw(draw: b2Draw): void;
}
