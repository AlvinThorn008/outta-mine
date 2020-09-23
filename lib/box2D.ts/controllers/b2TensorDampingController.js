/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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
import { b2Mat22, b2Max, b2Vec2 } from '../common/b2Math';
import { b2_epsilon } from '../common/b2Settings';
/**
 * Applies top down linear damping to the controlled bodies
 * The damping is calculated by multiplying velocity by a matrix
 * in local co-ordinates.
 */
export class b2TensorDampingController extends b2Controller {
    constructor() {
        super(...arguments);
        /// Tensor to use in damping model
        this.T = new b2Mat22();
        /*Some examples (matrixes in format (row1; row2))
          (-a 0; 0 -a)    Standard isotropic damping with strength a
          ( 0 a; -a 0)    Electron in fixed field - a force at right angles to velocity with proportional magnitude
          (-a 0; 0 -b)    Differing x and y damping. Useful e.g. for top-down wheels.
          */
        //By the way, tensor in this case just means matrix, don't let the terminology get you down.
        /// Set this to a positive number to clamp the maximum amount of damping done.
        this.maxTimestep = 0;
    }
    // Typically one wants maxTimestep to be 1/(max eigenvalue of T), so that damping will never cause something to reverse direction
    /**
     * @see b2Controller::Step
     */
    Step(step) {
        let timestep = step.dt;
        if (timestep <= b2_epsilon) {
            return;
        }
        if (timestep > this.maxTimestep && this.maxTimestep > 0) {
            timestep = this.maxTimestep;
        }
        for (let i = this.m_bodyList; i; i = i.nextBody) {
            const body = i.body;
            if (!body.IsAwake()) {
                continue;
            }
            const damping = body.GetWorldVector(b2Mat22.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity(), b2Vec2.s_t0), b2Vec2.s_t1), b2TensorDampingController.Step_s_damping);
            //    body->SetLinearVelocity(body->GetLinearVelocity() + timestep * damping);
            body.SetLinearVelocity(b2Vec2.AddVV(body.GetLinearVelocity(), b2Vec2.MulSV(timestep, damping, b2Vec2.s_t0), b2Vec2.s_t1));
        }
    }
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    Draw(draw) { }
    /**
     * Sets damping independantly along the x and y axes
     */
    SetAxisAligned(xDamping, yDamping) {
        this.T.ex.x = -xDamping;
        this.T.ex.y = 0;
        this.T.ey.x = 0;
        this.T.ey.y = -yDamping;
        if (xDamping > 0 || yDamping > 0) {
            this.maxTimestep = 1.0 / b2Max(xDamping, yDamping);
        }
        else {
            this.maxTimestep = 0.0;
        }
    }
}
b2TensorDampingController.Step_s_damping = new b2Vec2();
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJUZW5zb3JEYW1waW5nQ29udHJvbGxlci5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL3NyYy9jb250cm9sbGVycy9iMlRlbnNvckRhbXBpbmdDb250cm9sbGVyLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHO0FBRUgsMkJBQTJCO0FBRTNCLE9BQU8sRUFBRSxZQUFZLEVBQUUsTUFBTSxnQkFBZ0IsQ0FBQztBQUM5QyxPQUFPLEVBQUUsT0FBTyxFQUFFLEtBQUssRUFBRSxNQUFNLEVBQUUsTUFBTSxrQkFBa0IsQ0FBQztBQUUxRCxPQUFPLEVBQUUsVUFBVSxFQUFFLE1BQU0sc0JBQXNCLENBQUM7QUFHbEQ7Ozs7R0FJRztBQUNILE1BQU0sT0FBTyx5QkFBMEIsU0FBUSxZQUFZO0lBQTNEOztRQUNFLGtDQUFrQztRQUN6QixNQUFDLEdBQUcsSUFBSSxPQUFPLEVBQUUsQ0FBQztRQUMzQjs7OztZQUlJO1FBQ0osNEZBQTRGO1FBRTVGLDhFQUE4RTtRQUM5RSxnQkFBVyxHQUFHLENBQUMsQ0FBQztJQTBEbEIsQ0FBQztJQXhEQyxpSUFBaUk7SUFFakk7O09BRUc7SUFDSCxJQUFJLENBQUMsSUFBZ0I7UUFDbkIsSUFBSSxRQUFRLEdBQUcsSUFBSSxDQUFDLEVBQUUsQ0FBQztRQUN2QixJQUFJLFFBQVEsSUFBSSxVQUFVLEVBQUU7WUFDMUIsT0FBTztTQUNSO1FBQ0QsSUFBSSxRQUFRLEdBQUcsSUFBSSxDQUFDLFdBQVcsSUFBSSxJQUFJLENBQUMsV0FBVyxHQUFHLENBQUMsRUFBRTtZQUN2RCxRQUFRLEdBQUcsSUFBSSxDQUFDLFdBQVcsQ0FBQztTQUM3QjtRQUNELEtBQUssSUFBSSxDQUFDLEdBQUcsSUFBSSxDQUFDLFVBQVUsRUFBRSxDQUFDLEVBQUUsQ0FBQyxHQUFHLENBQUMsQ0FBQyxRQUFRLEVBQUU7WUFDL0MsTUFBTSxJQUFJLEdBQUcsQ0FBQyxDQUFDLElBQUksQ0FBQztZQUNwQixJQUFJLENBQUMsSUFBSSxDQUFDLE9BQU8sRUFBRSxFQUFFO2dCQUNuQixTQUFTO2FBQ1Y7WUFDRCxNQUFNLE9BQU8sR0FBRyxJQUFJLENBQUMsY0FBYyxDQUNqQyxPQUFPLENBQUMsS0FBSyxDQUNYLElBQUksQ0FBQyxDQUFDLEVBQ04sSUFBSSxDQUFDLGNBQWMsQ0FBQyxJQUFJLENBQUMsaUJBQWlCLEVBQUUsRUFBRSxNQUFNLENBQUMsSUFBSSxDQUFDLEVBQzFELE1BQU0sQ0FBQyxJQUFJLENBQ1osRUFDRCx5QkFBeUIsQ0FBQyxjQUFjLENBQ3pDLENBQUM7WUFDRiw4RUFBOEU7WUFDOUUsSUFBSSxDQUFDLGlCQUFpQixDQUNwQixNQUFNLENBQUMsS0FBSyxDQUNWLElBQUksQ0FBQyxpQkFBaUIsRUFBRSxFQUN4QixNQUFNLENBQUMsS0FBSyxDQUFDLFFBQVEsRUFBRSxPQUFPLEVBQUUsTUFBTSxDQUFDLElBQUksQ0FBQyxFQUM1QyxNQUFNLENBQUMsSUFBSSxDQUNaLENBQ0YsQ0FBQztTQUNIO0lBQ0gsQ0FBQztJQUlELGdFQUFnRTtJQUNoRSxJQUFJLENBQUMsSUFBWSxJQUFHLENBQUM7SUFFckI7O09BRUc7SUFDSCxjQUFjLENBQUMsUUFBZ0IsRUFBRSxRQUFnQjtRQUMvQyxJQUFJLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxRQUFRLENBQUM7UUFDeEIsSUFBSSxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQztRQUNoQixJQUFJLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDO1FBQ2hCLElBQUksQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLFFBQVEsQ0FBQztRQUN4QixJQUFJLFFBQVEsR0FBRyxDQUFDLElBQUksUUFBUSxHQUFHLENBQUMsRUFBRTtZQUNoQyxJQUFJLENBQUMsV0FBVyxHQUFHLEdBQUcsR0FBRyxLQUFLLENBQUMsUUFBUSxFQUFFLFFBQVEsQ0FBQyxDQUFDO1NBQ3BEO2FBQU07WUFDTCxJQUFJLENBQUMsV0FBVyxHQUFHLEdBQUcsQ0FBQztTQUN4QjtJQUNILENBQUM7O0FBbEJjLHdDQUFjLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQXFCL0MsU0FBUyIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDA3IEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuLy8gI2lmIEIyX0VOQUJMRV9DT05UUk9MTEVSXHJcblxyXG5pbXBvcnQgeyBiMkNvbnRyb2xsZXIgfSBmcm9tICcuL2IyQ29udHJvbGxlcic7XHJcbmltcG9ydCB7IGIyTWF0MjIsIGIyTWF4LCBiMlZlYzIgfSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJUaW1lU3RlcCB9IGZyb20gJy4uL2R5bmFtaWNzL2IyVGltZVN0ZXAnO1xyXG5pbXBvcnQgeyBiMl9lcHNpbG9uIH0gZnJvbSAnLi4vY29tbW9uL2IyU2V0dGluZ3MnO1xyXG5pbXBvcnQgeyBiMkRyYXcgfSBmcm9tICcuLi9jb21tb24vYjJEcmF3JztcclxuXHJcbi8qKlxyXG4gKiBBcHBsaWVzIHRvcCBkb3duIGxpbmVhciBkYW1waW5nIHRvIHRoZSBjb250cm9sbGVkIGJvZGllc1xyXG4gKiBUaGUgZGFtcGluZyBpcyBjYWxjdWxhdGVkIGJ5IG11bHRpcGx5aW5nIHZlbG9jaXR5IGJ5IGEgbWF0cml4XHJcbiAqIGluIGxvY2FsIGNvLW9yZGluYXRlcy5cclxuICovXHJcbmV4cG9ydCBjbGFzcyBiMlRlbnNvckRhbXBpbmdDb250cm9sbGVyIGV4dGVuZHMgYjJDb250cm9sbGVyIHtcclxuICAvLy8gVGVuc29yIHRvIHVzZSBpbiBkYW1waW5nIG1vZGVsXHJcbiAgcmVhZG9ubHkgVCA9IG5ldyBiMk1hdDIyKCk7XHJcbiAgLypTb21lIGV4YW1wbGVzIChtYXRyaXhlcyBpbiBmb3JtYXQgKHJvdzE7IHJvdzIpKVxyXG4gICAgKC1hIDA7IDAgLWEpICAgIFN0YW5kYXJkIGlzb3Ryb3BpYyBkYW1waW5nIHdpdGggc3RyZW5ndGggYVxyXG4gICAgKCAwIGE7IC1hIDApICAgIEVsZWN0cm9uIGluIGZpeGVkIGZpZWxkIC0gYSBmb3JjZSBhdCByaWdodCBhbmdsZXMgdG8gdmVsb2NpdHkgd2l0aCBwcm9wb3J0aW9uYWwgbWFnbml0dWRlXHJcbiAgICAoLWEgMDsgMCAtYikgICAgRGlmZmVyaW5nIHggYW5kIHkgZGFtcGluZy4gVXNlZnVsIGUuZy4gZm9yIHRvcC1kb3duIHdoZWVscy5cclxuICAgICovXHJcbiAgLy9CeSB0aGUgd2F5LCB0ZW5zb3IgaW4gdGhpcyBjYXNlIGp1c3QgbWVhbnMgbWF0cml4LCBkb24ndCBsZXQgdGhlIHRlcm1pbm9sb2d5IGdldCB5b3UgZG93bi5cclxuXHJcbiAgLy8vIFNldCB0aGlzIHRvIGEgcG9zaXRpdmUgbnVtYmVyIHRvIGNsYW1wIHRoZSBtYXhpbXVtIGFtb3VudCBvZiBkYW1waW5nIGRvbmUuXHJcbiAgbWF4VGltZXN0ZXAgPSAwO1xyXG5cclxuICAvLyBUeXBpY2FsbHkgb25lIHdhbnRzIG1heFRpbWVzdGVwIHRvIGJlIDEvKG1heCBlaWdlbnZhbHVlIG9mIFQpLCBzbyB0aGF0IGRhbXBpbmcgd2lsbCBuZXZlciBjYXVzZSBzb21ldGhpbmcgdG8gcmV2ZXJzZSBkaXJlY3Rpb25cclxuXHJcbiAgLyoqXHJcbiAgICogQHNlZSBiMkNvbnRyb2xsZXI6OlN0ZXBcclxuICAgKi9cclxuICBTdGVwKHN0ZXA6IGIyVGltZVN0ZXApIHtcclxuICAgIGxldCB0aW1lc3RlcCA9IHN0ZXAuZHQ7XHJcbiAgICBpZiAodGltZXN0ZXAgPD0gYjJfZXBzaWxvbikge1xyXG4gICAgICByZXR1cm47XHJcbiAgICB9XHJcbiAgICBpZiAodGltZXN0ZXAgPiB0aGlzLm1heFRpbWVzdGVwICYmIHRoaXMubWF4VGltZXN0ZXAgPiAwKSB7XHJcbiAgICAgIHRpbWVzdGVwID0gdGhpcy5tYXhUaW1lc3RlcDtcclxuICAgIH1cclxuICAgIGZvciAobGV0IGkgPSB0aGlzLm1fYm9keUxpc3Q7IGk7IGkgPSBpLm5leHRCb2R5KSB7XHJcbiAgICAgIGNvbnN0IGJvZHkgPSBpLmJvZHk7XHJcbiAgICAgIGlmICghYm9keS5Jc0F3YWtlKCkpIHtcclxuICAgICAgICBjb250aW51ZTtcclxuICAgICAgfVxyXG4gICAgICBjb25zdCBkYW1waW5nID0gYm9keS5HZXRXb3JsZFZlY3RvcihcclxuICAgICAgICBiMk1hdDIyLk11bE1WKFxyXG4gICAgICAgICAgdGhpcy5ULFxyXG4gICAgICAgICAgYm9keS5HZXRMb2NhbFZlY3Rvcihib2R5LkdldExpbmVhclZlbG9jaXR5KCksIGIyVmVjMi5zX3QwKSxcclxuICAgICAgICAgIGIyVmVjMi5zX3QxLFxyXG4gICAgICAgICksXHJcbiAgICAgICAgYjJUZW5zb3JEYW1waW5nQ29udHJvbGxlci5TdGVwX3NfZGFtcGluZyxcclxuICAgICAgKTtcclxuICAgICAgLy8gICAgYm9keS0+U2V0TGluZWFyVmVsb2NpdHkoYm9keS0+R2V0TGluZWFyVmVsb2NpdHkoKSArIHRpbWVzdGVwICogZGFtcGluZyk7XHJcbiAgICAgIGJvZHkuU2V0TGluZWFyVmVsb2NpdHkoXHJcbiAgICAgICAgYjJWZWMyLkFkZFZWKFxyXG4gICAgICAgICAgYm9keS5HZXRMaW5lYXJWZWxvY2l0eSgpLFxyXG4gICAgICAgICAgYjJWZWMyLk11bFNWKHRpbWVzdGVwLCBkYW1waW5nLCBiMlZlYzIuc190MCksXHJcbiAgICAgICAgICBiMlZlYzIuc190MSxcclxuICAgICAgICApLFxyXG4gICAgICApO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgU3RlcF9zX2RhbXBpbmcgPSBuZXcgYjJWZWMyKCk7XHJcblxyXG4gIC8vIGVzbGludC1kaXNhYmxlLW5leHQtbGluZSBAdHlwZXNjcmlwdC1lc2xpbnQvbm8tZW1wdHktZnVuY3Rpb25cclxuICBEcmF3KGRyYXc6IGIyRHJhdykge31cclxuXHJcbiAgLyoqXHJcbiAgICogU2V0cyBkYW1waW5nIGluZGVwZW5kYW50bHkgYWxvbmcgdGhlIHggYW5kIHkgYXhlc1xyXG4gICAqL1xyXG4gIFNldEF4aXNBbGlnbmVkKHhEYW1waW5nOiBudW1iZXIsIHlEYW1waW5nOiBudW1iZXIpIHtcclxuICAgIHRoaXMuVC5leC54ID0gLXhEYW1waW5nO1xyXG4gICAgdGhpcy5ULmV4LnkgPSAwO1xyXG4gICAgdGhpcy5ULmV5LnggPSAwO1xyXG4gICAgdGhpcy5ULmV5LnkgPSAteURhbXBpbmc7XHJcbiAgICBpZiAoeERhbXBpbmcgPiAwIHx8IHlEYW1waW5nID4gMCkge1xyXG4gICAgICB0aGlzLm1heFRpbWVzdGVwID0gMS4wIC8gYjJNYXgoeERhbXBpbmcsIHlEYW1waW5nKTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubWF4VGltZXN0ZXAgPSAwLjA7XHJcbiAgICB9XHJcbiAgfVxyXG59XHJcblxyXG4vLyAjZW5kaWZcclxuIl19