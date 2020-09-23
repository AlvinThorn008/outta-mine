/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
// #if B2_ENABLE_CONTROLLER
import { b2Controller } from './b2Controller';
import { b2Vec2 } from '../common/b2Math';
/**
 * Applies a force every frame
 */
export class b2ConstantForceController extends b2Controller {
    constructor() {
        super(...arguments);
        /**
         * The force to apply
         */
        this.F = new b2Vec2(0, 0);
    }
    Step(step) {
        for (let i = this.m_bodyList; i; i = i.nextBody) {
            const body = i.body;
            if (!body.IsAwake()) {
                continue;
            }
            body.ApplyForce(this.F, body.GetWorldCenter());
        }
    }
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    Draw(draw) { }
}
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb25zdGFudEZvcmNlQ29udHJvbGxlci5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9jb250cm9sbGVycy9iMkNvbnN0YW50Rm9yY2VDb250cm9sbGVyLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsMkJBQTJCO0FBRTNCLE9BQU8sRUFBRSxZQUFZLEVBQUUsTUFBTSxnQkFBZ0IsQ0FBQztBQUM5QyxPQUFPLEVBQUUsTUFBTSxFQUFFLE1BQU0sa0JBQWtCLENBQUM7QUFJMUM7O0dBRUc7QUFDSCxNQUFNLE9BQU8seUJBQTBCLFNBQVEsWUFBWTtJQUEzRDs7UUFDRTs7V0FFRztRQUNNLE1BQUMsR0FBRyxJQUFJLE1BQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7SUFjaEMsQ0FBQztJQVpDLElBQUksQ0FBQyxJQUFnQjtRQUNuQixLQUFLLElBQUksQ0FBQyxHQUFHLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxFQUFFLENBQUMsR0FBRyxDQUFDLENBQUMsUUFBUSxFQUFFO1lBQy9DLE1BQU0sSUFBSSxHQUFHLENBQUMsQ0FBQyxJQUFJLENBQUM7WUFDcEIsSUFBSSxDQUFDLElBQUksQ0FBQyxPQUFPLEVBQUUsRUFBRTtnQkFDbkIsU0FBUzthQUNWO1lBQ0QsSUFBSSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsQ0FBQyxFQUFFLElBQUksQ0FBQyxjQUFjLEVBQUUsQ0FBQyxDQUFDO1NBQ2hEO0lBQ0gsQ0FBQztJQUVELGdFQUFnRTtJQUNoRSxJQUFJLENBQUMsSUFBWSxJQUFHLENBQUM7Q0FDdEI7QUFFRCxTQUFTIiwic291cmNlc0NvbnRlbnQiOlsiLypcclxuICogQ29weXJpZ2h0IChjKSAyMDA2LTIwMDkgRXJpbiBDYXR0byBodHRwOi8vd3d3LmJveDJkLm9yZ1xyXG4gKlxyXG4gKiBUaGlzIHNvZnR3YXJlIGlzIHByb3ZpZGVkICdhcy1pcycsIHdpdGhvdXQgYW55IGV4cHJlc3Mgb3IgaW1wbGllZFxyXG4gKiB3YXJyYW50eS4gIEluIG5vIGV2ZW50IHdpbGwgdGhlIGF1dGhvcnMgYmUgaGVsZCBsaWFibGUgZm9yIGFueSBkYW1hZ2VzXHJcbiAqIGFyaXNpbmcgZnJvbSB0aGUgdXNlIG9mIHRoaXMgc29mdHdhcmUuXHJcbiAqIFBlcm1pc3Npb24gaXMgZ3JhbnRlZCB0byBhbnlvbmUgdG8gdXNlIHRoaXMgc29mdHdhcmUgZm9yIGFueSBwdXJwb3NlLFxyXG4gKiBpbmNsdWRpbmcgY29tbWVyY2lhbCBhcHBsaWNhdGlvbnMsIGFuZCB0byBhbHRlciBpdCBhbmQgcmVkaXN0cmlidXRlIGl0XHJcbiAqIGZyZWVseSwgc3ViamVjdCB0byB0aGUgZm9sbG93aW5nIHJlc3RyaWN0aW9uczpcclxuICogMS4gVGhlIG9yaWdpbiBvZiB0aGlzIHNvZnR3YXJlIG11c3Qgbm90IGJlIG1pc3JlcHJlc2VudGVkOyB5b3UgbXVzdCBub3RcclxuICogY2xhaW0gdGhhdCB5b3Ugd3JvdGUgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLiBJZiB5b3UgdXNlIHRoaXMgc29mdHdhcmVcclxuICogaW4gYSBwcm9kdWN0LCBhbiBhY2tub3dsZWRnbWVudCBpbiB0aGUgcHJvZHVjdCBkb2N1bWVudGF0aW9uIHdvdWxkIGJlXHJcbiAqIGFwcHJlY2lhdGVkIGJ1dCBpcyBub3QgcmVxdWlyZWQuXHJcbiAqIDIuIEFsdGVyZWQgc291cmNlIHZlcnNpb25zIG11c3QgYmUgcGxhaW5seSBtYXJrZWQgYXMgc3VjaCwgYW5kIG11c3Qgbm90IGJlXHJcbiAqIG1pc3JlcHJlc2VudGVkIGFzIGJlaW5nIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS5cclxuICogMy4gVGhpcyBub3RpY2UgbWF5IG5vdCBiZSByZW1vdmVkIG9yIGFsdGVyZWQgZnJvbSBhbnkgc291cmNlIGRpc3RyaWJ1dGlvbi5cclxuICovXHJcblxyXG4vLyAjaWYgQjJfRU5BQkxFX0NPTlRST0xMRVJcclxuXHJcbmltcG9ydCB7IGIyQ29udHJvbGxlciB9IGZyb20gJy4vYjJDb250cm9sbGVyJztcclxuaW1wb3J0IHsgYjJWZWMyIH0gZnJvbSAnLi4vY29tbW9uL2IyTWF0aCc7XHJcbmltcG9ydCB7IGIyVGltZVN0ZXAgfSBmcm9tICcuLi9keW5hbWljcy9iMlRpbWVTdGVwJztcclxuaW1wb3J0IHsgYjJEcmF3IH0gZnJvbSAnLi4vY29tbW9uL2IyRHJhdyc7XHJcblxyXG4vKipcclxuICogQXBwbGllcyBhIGZvcmNlIGV2ZXJ5IGZyYW1lXHJcbiAqL1xyXG5leHBvcnQgY2xhc3MgYjJDb25zdGFudEZvcmNlQ29udHJvbGxlciBleHRlbmRzIGIyQ29udHJvbGxlciB7XHJcbiAgLyoqXHJcbiAgICogVGhlIGZvcmNlIHRvIGFwcGx5XHJcbiAgICovXHJcbiAgcmVhZG9ubHkgRiA9IG5ldyBiMlZlYzIoMCwgMCk7XHJcblxyXG4gIFN0ZXAoc3RlcDogYjJUaW1lU3RlcCkge1xyXG4gICAgZm9yIChsZXQgaSA9IHRoaXMubV9ib2R5TGlzdDsgaTsgaSA9IGkubmV4dEJvZHkpIHtcclxuICAgICAgY29uc3QgYm9keSA9IGkuYm9keTtcclxuICAgICAgaWYgKCFib2R5LklzQXdha2UoKSkge1xyXG4gICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICB9XHJcbiAgICAgIGJvZHkuQXBwbHlGb3JjZSh0aGlzLkYsIGJvZHkuR2V0V29ybGRDZW50ZXIoKSk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICAvLyBlc2xpbnQtZGlzYWJsZS1uZXh0LWxpbmUgQHR5cGVzY3JpcHQtZXNsaW50L25vLWVtcHR5LWZ1bmN0aW9uXHJcbiAgRHJhdyhkcmF3OiBiMkRyYXcpIHt9XHJcbn1cclxuXHJcbi8vICNlbmRpZlxyXG4iXX0=