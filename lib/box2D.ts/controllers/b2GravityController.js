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
import { b2_epsilon } from '../common/b2Settings';
import { b2Sqrt, b2Vec2 } from '../common/b2Math';
/**
 * Applies simplified gravity between every pair of bodies
 */
export class b2GravityController extends b2Controller {
    constructor() {
        super(...arguments);
        /**
         * Specifies the strength of the gravitiation force
         */
        this.G = 1;
        /**
         * If true, gravity is proportional to r^-2, otherwise r^-1
         */
        this.invSqr = true;
    }
    /**
     * @see b2Controller::Step
     */
    Step(step) {
        if (this.invSqr) {
            for (let i = this.m_bodyList; i; i = i.nextBody) {
                const body1 = i.body;
                const p1 = body1.GetWorldCenter();
                const mass1 = body1.GetMass();
                for (let j = this.m_bodyList; j && j !== i; j = j.nextBody) {
                    const body2 = j.body;
                    const p2 = body2.GetWorldCenter();
                    const mass2 = body2.GetMass();
                    const dx = p2.x - p1.x;
                    const dy = p2.y - p1.y;
                    const r2 = dx * dx + dy * dy;
                    if (r2 < b2_epsilon) {
                        continue;
                    }
                    const f = b2GravityController.Step_s_f.Set(dx, dy);
                    f.SelfMul((this.G / r2 / b2Sqrt(r2)) * mass1 * mass2);
                    if (body1.IsAwake()) {
                        body1.ApplyForce(f, p1);
                    }
                    if (body2.IsAwake()) {
                        body2.ApplyForce(f.SelfMul(-1), p2);
                    }
                }
            }
        }
        else {
            for (let i = this.m_bodyList; i; i = i.nextBody) {
                const body1 = i.body;
                const p1 = body1.GetWorldCenter();
                const mass1 = body1.GetMass();
                for (let j = this.m_bodyList; j && j !== i; j = j.nextBody) {
                    const body2 = j.body;
                    const p2 = body2.GetWorldCenter();
                    const mass2 = body2.GetMass();
                    const dx = p2.x - p1.x;
                    const dy = p2.y - p1.y;
                    const r2 = dx * dx + dy * dy;
                    if (r2 < b2_epsilon) {
                        continue;
                    }
                    const f = b2GravityController.Step_s_f.Set(dx, dy);
                    f.SelfMul((this.G / r2) * mass1 * mass2);
                    if (body1.IsAwake()) {
                        body1.ApplyForce(f, p1);
                    }
                    if (body2.IsAwake()) {
                        body2.ApplyForce(f.SelfMul(-1), p2);
                    }
                }
            }
        }
    }
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    Draw(draw) { }
}
b2GravityController.Step_s_f = new b2Vec2();
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJHcmF2aXR5Q29udHJvbGxlci5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9jb250cm9sbGVycy9iMkdyYXZpdHlDb250cm9sbGVyLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsMkJBQTJCO0FBRTNCLE9BQU8sRUFBRSxZQUFZLEVBQUUsTUFBTSxnQkFBZ0IsQ0FBQztBQUU5QyxPQUFPLEVBQUUsVUFBVSxFQUFFLE1BQU0sc0JBQXNCLENBQUM7QUFDbEQsT0FBTyxFQUFFLE1BQU0sRUFBRSxNQUFNLEVBQUUsTUFBTSxrQkFBa0IsQ0FBQztBQUdsRDs7R0FFRztBQUNILE1BQU0sT0FBTyxtQkFBb0IsU0FBUSxZQUFZO0lBQXJEOztRQUNFOztXQUVHO1FBQ0gsTUFBQyxHQUFHLENBQUMsQ0FBQztRQUNOOztXQUVHO1FBQ0gsV0FBTSxHQUFHLElBQUksQ0FBQztJQStEaEIsQ0FBQztJQTdEQzs7T0FFRztJQUNILElBQUksQ0FBQyxJQUFnQjtRQUNuQixJQUFJLElBQUksQ0FBQyxNQUFNLEVBQUU7WUFDZixLQUFLLElBQUksQ0FBQyxHQUFHLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxFQUFFLENBQUMsR0FBRyxDQUFDLENBQUMsUUFBUSxFQUFFO2dCQUMvQyxNQUFNLEtBQUssR0FBRyxDQUFDLENBQUMsSUFBSSxDQUFDO2dCQUNyQixNQUFNLEVBQUUsR0FBRyxLQUFLLENBQUMsY0FBYyxFQUFFLENBQUM7Z0JBQ2xDLE1BQU0sS0FBSyxHQUFHLEtBQUssQ0FBQyxPQUFPLEVBQUUsQ0FBQztnQkFDOUIsS0FBSyxJQUFJLENBQUMsR0FBRyxJQUFJLENBQUMsVUFBVSxFQUFFLENBQUMsSUFBSSxDQUFDLEtBQUssQ0FBQyxFQUFFLENBQUMsR0FBRyxDQUFDLENBQUMsUUFBUSxFQUFFO29CQUMxRCxNQUFNLEtBQUssR0FBRyxDQUFDLENBQUMsSUFBSSxDQUFDO29CQUNyQixN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QU8sQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDO3FCQUNyQztpQkFDRjthQUNGO1NBQ0Y7SUFDSCxDQUFDO0lBSUQsZ0VBQWdFO0lBQ2hFLElBQUksQ0FBQyxJQUFZLElBQUcsQ0FBQzs7QUFITiw0QkFBUSxHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFNekMsU0FBUyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDA5IEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuLy8gI2lmIEIyX0VOQUJMRV9DT05UUk9MTEVSXHJcblxyXG5pbXBvcnQgeyBiMkNvbnRyb2xsZXIgfSBmcm9tICcuL2IyQ29udHJvbGxlcic7XHJcbmltcG9ydCB7IGIyVGltZVN0ZXAgfSBmcm9tICcuLi9keW5hbWljcy9iMlRpbWVTdGVwJztcclxuaW1wb3J0IHsgYjJfZXBzaWxvbiB9IGZyb20gJy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJTcXJ0LCBiMlZlYzIgfSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJEcmF3IH0gZnJvbSAnLi4vY29tbW9uL2IyRHJhdyc7XHJcblxyXG4vKipcclxuICogQXBwbGllcyBzaW1wbGlmaWVkIGdyYXZpdHkgYmV0d2VlbiBldmVyeSBwYWlyIG9mIGJvZGllc1xyXG4gKi9cclxuZXhwb3J0IGNsYXNzIGIyR3Jhdml0eUNvbnRyb2xsZXIgZXh0ZW5kcyBiMkNvbnRyb2xsZXIge1xyXG4gIC8qKlxyXG4gICAqIFNwZWNpZmllcyB0aGUgc3RyZW5ndGggb2YgdGhlIGdyYXZpdGlhdGlvbiBmb3JjZVxyXG4gICAqL1xyXG4gIEcgPSAxO1xyXG4gIC8qKlxyXG4gICAqIElmIHRydWUsIGdyYXZpdHkgaXMgcHJvcG9ydGlvbmFsIHRvIHJeLTIsIG90aGVyd2lzZSByXi0xXHJcbiAgICovXHJcbiAgaW52U3FyID0gdHJ1ZTtcclxuXHJcbiAgLyoqXHJcbiAgICogQHNlZSBiMkNvbnRyb2xsZXI6OlN0ZXBcclxuICAgKi9cclxuICBTdGVwKHN0ZXA6IGIyVGltZVN0ZXApIHtcclxuICAgIGlmICh0aGlzLmludlNxcikge1xyXG4gICAgICBmb3IgKGxldCBpID0gdGhpcy5tX2JvZHlMaXN0OyBpOyBpID0gaS5uZXh0Qm9keSkge1xyXG4gICAgICAgIGNvbnN0IGJvZHkxID0gaS5ib2R5O1xyXG4gICAgICAgIGNvbnN0IHAxID0gYm9keTEuR2V0V29ybGRDZW50ZXIoKTtcclxuICAgICAgICBjb25zdCBtYXNzMSA9IGJvZHkxLkdldE1hc3MoKTtcclxuICAgICAgICBmb3IgKGxldCBqID0gdGhpcy5tX2JvZHlMaXN0OyBqICYmIGogIT09IGk7IGogPSBqLm5leHRCb2R5KSB7XHJcbiAgICAgICAgICBjb25zdCBib2R5MiA9IGouYm9keTtcclxuICAgICAgICAgIGNvbnN0IHAyID0gYm9keTIuR2V0V29ybGRDZW50ZXIoKTtcclxuICAgICAgICAgIGNvbnN0IG1hc3MyID0gYm9keTIuR2V0TWFzcygpO1xyXG4gICAgICAgICAgY29uc3QgZHggPSBwMi54IC0gcDEueDtcclxuICAgICAgICAgIGNvbnN0IGR5ID0gcDIueSAtIHAxLnk7XHJcbiAgICAgICAgICBjb25zdCByMiA9IGR4ICogZHggKyBkeSAqIGR5O1xyXG4gICAgICAgICAgaWYgKHIyIDwgYjJfZXBzaWxvbikge1xyXG4gICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgIH1cclxuICAgICAgICAgIGNvbnN0IGYgPSBiMkdyYXZpdHlDb250cm9sbGVyLlN0ZXBfc19mLlNldChkeCwgZHkpO1xyXG4gICAgICAgICAgZi5TZWxmTXVsKCh0aGlzLkcgLyByMiAvIGIyU3FydChyMikpICogbWFzczEgKiBtYXNzMik7XHJcbiAgICAgICAgICBpZiAoYm9keTEuSXNBd2FrZSgpKSB7XHJcbiAgICAgICAgICAgIGJvZHkxLkFwcGx5Rm9yY2UoZiwgcDEpO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgaWYgKGJvZHkyLklzQXdha2UoKSkge1xyXG4gICAgICAgICAgICBib2R5Mi5BcHBseUZvcmNlKGYuU2VsZk11bCgtMSksIHAyKTtcclxuICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIGZvciAobGV0IGkgPSB0aGlzLm1fYm9keUxpc3Q7IGk7IGkgPSBpLm5leHRCb2R5KSB7XHJcbiAgICAgICAgY29uc3QgYm9keTEgPSBpLmJvZHk7XHJcbiAgICAgICAgY29uc3QgcDEgPSBib2R5MS5HZXRXb3JsZENlbnRlcigpO1xyXG4gICAgICAgIGNvbnN0IG1hc3MxID0gYm9keTEuR2V0TWFzcygpO1xyXG4gICAgICAgIGZvciAobGV0IGogPSB0aGlzLm1fYm9keUxpc3Q7IGogJiYgaiAhPT0gaTsgaiA9IGoubmV4dEJvZHkpIHtcclxuICAgICAgICAgIGNvbnN0IGJvZHkyID0gai5ib2R5O1xyXG4gICAgICAgICAgY29uc3QgcDIgPSBib2R5Mi5HZXRXb3JsZENlbnRlcigpO1xyXG4gICAgICAgICAgY29uc3QgbWFzczIgPSBib2R5Mi5HZXRNYXNzKCk7XHJcbiAgICAgICAgICBjb25zdCBkeCA9IHAyLnggLSBwMS54O1xyXG4gICAgICAgICAgY29uc3QgZHkgPSBwMi55IC0gcDEueTtcclxuICAgICAgICAgIGNvbnN0IHIyID0gZHggKiBkeCArIGR5ICogZHk7XHJcbiAgICAgICAgICBpZiAocjIgPCBiMl9lcHNpbG9uKSB7XHJcbiAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgY29uc3QgZiA9IGIyR3Jhdml0eUNvbnRyb2xsZXIuU3RlcF9zX2YuU2V0KGR4LCBkeSk7XHJcbiAgICAgICAgICBmLlNlbGZNdWwoKHRoaXMuRyAvIHIyKSAqIG1hc3MxICogbWFzczIpO1xyXG4gICAgICAgICAgaWYgKGJvZHkxLklzQXdha2UoKSkge1xyXG4gICAgICAgICAgICBib2R5MS5BcHBseUZvcmNlKGYsIHAxKTtcclxuICAgICAgICAgIH1cclxuICAgICAgICAgIGlmIChib2R5Mi5Jc0F3YWtlKCkpIHtcclxuICAgICAgICAgICAgYm9keTIuQXBwbHlGb3JjZShmLlNlbGZNdWwoLTEpLCBwMik7XHJcbiAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBwcml2YXRlIHN0YXRpYyBTdGVwX3NfZiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgLy8gZXNsaW50LWRpc2FibGUtbmV4dC1saW5lIEB0eXBlc2NyaXB0LWVzbGludC9uby1lbXB0eS1mdW5jdGlvblxyXG4gIERyYXcoZHJhdzogYjJEcmF3KSB7fVxyXG59XHJcblxyXG4vLyAjZW5kaWZcclxuIl19