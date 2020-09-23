/*
 * Copyright (c) 2010 Erin Catto http://www.box2d.org
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
import { b2Assert, b2MakeNullArray } from './b2Settings';
/// This is a growable LIFO stack with an initial capacity of N.
/// If the stack size exceeds the initial capacity, the heap is used
/// to increase the size of the stack.
export class b2GrowableStack {
    constructor(N) {
        this.m_count = 0;
        this.m_stack = b2MakeNullArray(N);
    }
    Reset() {
        this.m_count = 0;
        return this;
    }
    Push(element) {
        if (this.m_count < this.m_stack.length) {
            this.m_stack[this.m_count] = element;
        }
        else {
            this.m_stack.push(element);
        }
        ++this.m_count;
    }
    Pop() {
        !!B2_DEBUG && b2Assert(this.m_count > 0);
        if (this.m_count === 0) {
            throw new Error();
        }
        --this.m_count;
        const element = this.m_stack[this.m_count];
        this.m_stack[this.m_count] = null;
        return element;
    }
    GetCount() {
        return this.m_count;
    }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJHcm93YWJsZVN0YWNrLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL2NvbW1vbi9iMkdyb3dhYmxlU3RhY2sudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7QUFFSCxPQUFPLEVBQUUsUUFBUSxFQUFFLGVBQWUsRUFBRSxNQUFNLGNBQWMsQ0FBQztBQUV6RCxnRUFBZ0U7QUFDaEUsb0VBQW9FO0FBQ3BFLHNDQUFzQztBQUV0QyxNQUFNLE9BQU8sZUFBZTtJQUkxQixZQUFZLENBQVM7UUFGckIsWUFBTyxHQUFHLENBQUMsQ0FBQztRQUdWLElBQUksQ0FBQyxPQUFPLEdBQUcsZUFBZSxDQUFDLENBQUMsQ0FBQyxDQUFDO0lBQ3BDLENBQUM7SUFFRCxLQUFLO1FBQ0gsSUFBSSxDQUFDLE9BQU8sR0FBRyxDQUFDLENBQUM7UUFDakIsT0FBTyxJQUFJLENBQUM7SUFDZCxDQUFDO0lBRUQsSUFBSSxDQUFDLE9BQVU7UUFDYixJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQyxNQUFNLEVBQUU7WUFDdEMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLEdBQUcsT0FBTyxDQUFDO1NBQ3RDO2FBQU07WUFDTCxJQUFJLENBQUMsT0FBTyxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsQ0FBQztTQUM1QjtRQUNELEVBQUUsSUFBSSxDQUFDLE9BQU8sQ0FBQztJQUNqQixDQUFDO0lBRUQsR0FBRztRQUNELENBQUMsQ0FBQyxRQUFRLElBQUksUUFBUSxDQUFDLElBQUksQ0FBQyxPQUFPLEdBQUcsQ0FBQyxDQUFDLENBQUM7UUFDekMsSUFBSSxJQUFJLENBQUMsT0FBTyxLQUFLLENBQUMsRUFBRTtZQUN0QixNQUFNLElBQUksS0FBSyxFQUFFLENBQUM7U0FDbkI7UUFDRCxFQUFFLElBQUksQ0FBQyxPQUFPLENBQUM7UUFDZixNQUFNLE9BQU8sR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUUsQ0FBQztRQUM1QyxJQUFJLENBQUMsT0FBTyxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsR0FBRyxJQUFJLENBQUM7UUFDbEMsT0FBTyxPQUFPLENBQUM7SUFDakIsQ0FBQztJQUVELFFBQVE7UUFDTixPQUFPLElBQUksQ0FBQyxPQUFPLENBQUM7SUFDdEIsQ0FBQztDQUNGIiwic291cmNlc0NvbnRlbnQiOlsiLypcclxuICogQ29weXJpZ2h0IChjKSAyMDEwIEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHsgYjJBc3NlcnQsIGIyTWFrZU51bGxBcnJheSB9IGZyb20gJy4vYjJTZXR0aW5ncyc7XHJcblxyXG4vLy8gVGhpcyBpcyBhIGdyb3dhYmxlIExJRk8gc3RhY2sgd2l0aCBhbiBpbml0aWFsIGNhcGFjaXR5IG9mIE4uXHJcbi8vLyBJZiB0aGUgc3RhY2sgc2l6ZSBleGNlZWRzIHRoZSBpbml0aWFsIGNhcGFjaXR5LCB0aGUgaGVhcCBpcyB1c2VkXHJcbi8vLyB0byBpbmNyZWFzZSB0aGUgc2l6ZSBvZiB0aGUgc3RhY2suXHJcblxyXG5leHBvcnQgY2xhc3MgYjJHcm93YWJsZVN0YWNrPFQ+IHtcclxuICBtX3N0YWNrOiBBcnJheTxUIHwgbnVsbD47XHJcbiAgbV9jb3VudCA9IDA7XHJcblxyXG4gIGNvbnN0cnVjdG9yKE46IG51bWJlcikge1xyXG4gICAgdGhpcy5tX3N0YWNrID0gYjJNYWtlTnVsbEFycmF5KE4pO1xyXG4gIH1cclxuXHJcbiAgUmVzZXQoKTogdGhpcyB7XHJcbiAgICB0aGlzLm1fY291bnQgPSAwO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBQdXNoKGVsZW1lbnQ6IFQpOiB2b2lkIHtcclxuICAgIGlmICh0aGlzLm1fY291bnQgPCB0aGlzLm1fc3RhY2subGVuZ3RoKSB7XHJcbiAgICAgIHRoaXMubV9zdGFja1t0aGlzLm1fY291bnRdID0gZWxlbWVudDtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHRoaXMubV9zdGFjay5wdXNoKGVsZW1lbnQpO1xyXG4gICAgfVxyXG4gICAgKyt0aGlzLm1fY291bnQ7XHJcbiAgfVxyXG5cclxuICBQb3AoKTogVCB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRoaXMubV9jb3VudCA+IDApO1xyXG4gICAgaWYgKHRoaXMubV9jb3VudCA9PT0gMCkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuICAgIC0tdGhpcy5tX2NvdW50O1xyXG4gICAgY29uc3QgZWxlbWVudCA9IHRoaXMubV9zdGFja1t0aGlzLm1fY291bnRdITtcclxuICAgIHRoaXMubV9zdGFja1t0aGlzLm1fY291bnRdID0gbnVsbDtcclxuICAgIHJldHVybiBlbGVtZW50O1xyXG4gIH1cclxuXHJcbiAgR2V0Q291bnQoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLm1fY291bnQ7XHJcbiAgfVxyXG59XHJcbiJdfQ==