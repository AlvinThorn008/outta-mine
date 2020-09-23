/*
 * Copyright (c) 2013 Google, Inc.
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
import { b2Assert } from '../common/b2Settings';
export class b2StackQueue {
    constructor(capacity) {
        this.m_buffer = [];
        this.m_front = 0;
        this.m_back = 0;
        this.m_buffer.fill(null, 0, capacity);
    }
    get m_capacity() {
        return this.m_buffer.length;
    }
    Push(item) {
        if (this.m_back >= this.m_capacity) {
            for (let i = this.m_front; i < this.m_back; i++) {
                this.m_buffer[i - this.m_front] = this.m_buffer[i];
            }
            this.m_back -= this.m_front;
            this.m_front = 0;
        }
        this.m_buffer[this.m_back] = item;
        this.m_back++;
    }
    Pop() {
        !!B2_DEBUG && b2Assert(this.m_front < this.m_back);
        this.m_buffer[this.m_front] = null;
        this.m_front++;
    }
    Empty() {
        !!B2_DEBUG && b2Assert(this.m_front <= this.m_back);
        return this.m_front === this.m_back;
    }
    Front() {
        const item = this.m_buffer[this.m_front];
        if (!item) {
            throw new Error();
        }
        return item;
    }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJTdGFja1F1ZXVlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL3BhcnRpY2xlL2IyU3RhY2tRdWV1ZS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFBRSxRQUFRLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUVoRCxNQUFNLE9BQU8sWUFBWTtJQVN2QixZQUFZLFFBQWdCO1FBUm5CLGFBQVEsR0FBb0IsRUFBRSxDQUFDO1FBQ3hDLFlBQU8sR0FBRyxDQUFDLENBQUM7UUFDWixXQUFNLEdBQUcsQ0FBQyxDQUFDO1FBT1QsSUFBSSxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsSUFBSSxFQUFFLENBQUMsRUFBRSxRQUFRLENBQUMsQ0FBQztJQUN4QyxDQUFDO0lBTkQsSUFBSSxVQUFVO1FBQ1osT0FBTyxJQUFJLENBQUMsUUFBUSxDQUFDLE1BQU0sQ0FBQztJQUM5QixDQUFDO0lBTUQsSUFBSSxDQUFDLElBQU87UUFDVixJQUFJLElBQUksQ0FBQyxNQUFNLElBQUksSUFBSSxDQUFDLFVBQVUsRUFBRTtZQUNsQyxLQUFLLElBQUksQ0FBQyxHQUFHLElBQUksQ0FBQyxPQUFPLEVBQUUsQ0FBQyxHQUFHLElBQUksQ0FBQyxNQUFNLEVBQUUsQ0FBQyxFQUFFLEVBQUU7Z0JBQy9DLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUMsR0FBRyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO2FBQ3BEO1lBQ0QsSUFBSSxDQUFDLE1BQU0sSUFBSSxJQUFJLENBQUMsT0FBTyxDQUFDO1lBQzVCLElBQUksQ0FBQyxPQUFPLEdBQUcsQ0FBQyxDQUFDO1NBQ2xCO1FBQ0QsSUFBSSxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsTUFBTSxDQUFDLEdBQUcsSUFBSSxDQUFDO1FBQ2xDLElBQUksQ0FBQyxNQUFNLEVBQUUsQ0FBQztJQUNoQixDQUFDO0lBRUQsR0FBRztRQUNELENBQUMsQ0FBQyxRQUFRLElBQUksUUFBUSxDQUFDLElBQUksQ0FBQyxPQUFPLEdBQUcsSUFBSSxDQUFDLE1BQU0sQ0FBQyxDQUFDO1FBQ25ELElBQUksQ0FBQyxRQUFRLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxHQUFHLElBQUksQ0FBQztRQUNuQyxJQUFJLENBQUMsT0FBTyxFQUFFLENBQUM7SUFDakIsQ0FBQztJQUVELEtBQUs7UUFDSCxDQUFDLENBQUMsUUFBUSxJQUFJLFFBQVEsQ0FBQyxJQUFJLENBQUMsT0FBTyxJQUFJLElBQUksQ0FBQyxNQUFNLENBQUMsQ0FBQztRQUNwRCxPQUFPLElBQUksQ0FBQyxPQUFPLEtBQUssSUFBSSxDQUFDLE1BQU0sQ0FBQztJQUN0QyxDQUFDO0lBRUQsS0FBSztRQUNILE1BQU0sSUFBSSxHQUFHLElBQUksQ0FBQyxRQUFRLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxDQUFDO1FBQ3pDLElBQUksQ0FBQyxJQUFJLEVBQUU7WUFDVCxNQUFNLElBQUksS0FBSyxFQUFFLENBQUM7U0FDbkI7UUFDRCxPQUFPLElBQUksQ0FBQztJQUNkLENBQUM7Q0FDRiIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAxMyBHb29nbGUsIEluYy5cclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHsgYjJBc3NlcnQgfSBmcm9tICcuLi9jb21tb24vYjJTZXR0aW5ncyc7XHJcblxyXG5leHBvcnQgY2xhc3MgYjJTdGFja1F1ZXVlPFQ+IHtcclxuICByZWFkb25seSBtX2J1ZmZlcjogQXJyYXk8VCB8IG51bGw+ID0gW107XHJcbiAgbV9mcm9udCA9IDA7XHJcbiAgbV9iYWNrID0gMDtcclxuXHJcbiAgZ2V0IG1fY2FwYWNpdHkoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fYnVmZmVyLmxlbmd0aDtcclxuICB9XHJcblxyXG4gIGNvbnN0cnVjdG9yKGNhcGFjaXR5OiBudW1iZXIpIHtcclxuICAgIHRoaXMubV9idWZmZXIuZmlsbChudWxsLCAwLCBjYXBhY2l0eSk7XHJcbiAgfVxyXG5cclxuICBQdXNoKGl0ZW06IFQpOiB2b2lkIHtcclxuICAgIGlmICh0aGlzLm1fYmFjayA+PSB0aGlzLm1fY2FwYWNpdHkpIHtcclxuICAgICAgZm9yIChsZXQgaSA9IHRoaXMubV9mcm9udDsgaSA8IHRoaXMubV9iYWNrOyBpKyspIHtcclxuICAgICAgICB0aGlzLm1fYnVmZmVyW2kgLSB0aGlzLm1fZnJvbnRdID0gdGhpcy5tX2J1ZmZlcltpXTtcclxuICAgICAgfVxyXG4gICAgICB0aGlzLm1fYmFjayAtPSB0aGlzLm1fZnJvbnQ7XHJcbiAgICAgIHRoaXMubV9mcm9udCA9IDA7XHJcbiAgICB9XHJcbiAgICB0aGlzLm1fYnVmZmVyW3RoaXMubV9iYWNrXSA9IGl0ZW07XHJcbiAgICB0aGlzLm1fYmFjaysrO1xyXG4gIH1cclxuXHJcbiAgUG9wKCk6IHZvaWQge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCh0aGlzLm1fZnJvbnQgPCB0aGlzLm1fYmFjayk7XHJcbiAgICB0aGlzLm1fYnVmZmVyW3RoaXMubV9mcm9udF0gPSBudWxsO1xyXG4gICAgdGhpcy5tX2Zyb250Kys7XHJcbiAgfVxyXG5cclxuICBFbXB0eSgpOiBib29sZWFuIHtcclxuICAgICEhQjJfREVCVUcgJiYgYjJBc3NlcnQodGhpcy5tX2Zyb250IDw9IHRoaXMubV9iYWNrKTtcclxuICAgIHJldHVybiB0aGlzLm1fZnJvbnQgPT09IHRoaXMubV9iYWNrO1xyXG4gIH1cclxuXHJcbiAgRnJvbnQoKTogVCB7XHJcbiAgICBjb25zdCBpdGVtID0gdGhpcy5tX2J1ZmZlclt0aGlzLm1fZnJvbnRdO1xyXG4gICAgaWYgKCFpdGVtKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIGl0ZW07XHJcbiAgfVxyXG59XHJcbiJdfQ==