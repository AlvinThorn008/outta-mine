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
import { b2Assert, b2Maybe } from '../../common/b2Settings';
import { b2Vec2 } from '../../common/b2Math';
export class b2Jacobian {
    constructor() {
        this.linear = new b2Vec2();
        this.angularA = 0;
        this.angularB = 0;
    }
    SetZero() {
        this.linear.SetZero();
        this.angularA = 0;
        this.angularB = 0;
        return this;
    }
    Set(x, a1, a2) {
        this.linear.Copy(x);
        this.angularA = a1;
        this.angularB = a2;
        return this;
    }
}
/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
export class b2JointEdge {
    constructor(joint) {
        this._other = null; ///< provides quick access to the other body attached.
        this.prev = null; ///< the previous joint edge in the body's joint list
        this.next = null; ///< the next joint edge in the body's joint list
        this.joint = joint;
    }
    get other() {
        !!B2_DEBUG && b2Assert(this._other !== null);
        return this._other;
    }
    set other(value) {
        !!B2_DEBUG && b2Assert(this._other === null);
        this._other = value;
    }
    Reset() {
        this._other = null;
        this.prev = null;
        this.next = null;
    }
}
/// Joint definitions are used to construct joints.
export class b2JointDef {
    constructor(type) {
        /// The joint type is set automatically for concrete joint types.
        this.type = 0 /* e_unknownJoint */;
        /// Use this to attach application specific data to your joints.
        this.userData = null;
        /// Set this flag to true if the attached bodies should collide.
        this.collideConnected = false;
        this.type = type;
    }
}
/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
export class b2Joint {
    constructor(def) {
        this.m_type = 0 /* e_unknownJoint */;
        this.m_prev = null;
        this.m_next = null;
        this.m_edgeA = new b2JointEdge(this);
        this.m_edgeB = new b2JointEdge(this);
        this.m_islandFlag = false;
        this.m_collideConnected = false;
        this.m_userData = null;
        this._logIndex = 0;
        !!B2_DEBUG && b2Assert(def.bodyA !== def.bodyB);
        this.m_type = def.type;
        this.m_edgeA.other = def.bodyB;
        this.m_edgeB.other = def.bodyA;
        this.m_bodyA = def.bodyA;
        this.m_bodyB = def.bodyB;
        this.m_collideConnected = b2Maybe(def.collideConnected, false);
        this.m_userData = b2Maybe(def.userData, null);
    }
    /// Get the type of the concrete joint.
    GetType() {
        return this.m_type;
    }
    /// Get the first body attached to this joint.
    GetBodyA() {
        return this.m_bodyA;
    }
    /// Get the second body attached to this joint.
    GetBodyB() {
        return this.m_bodyB;
    }
    /// Get the next joint the world joint list.
    GetNext() {
        return this.m_next;
    }
    /// Get the user data pointer.
    GetUserData() {
        return this.m_userData;
    }
    /// Set the user data pointer.
    SetUserData(data) {
        this.m_userData = data;
    }
    /// Short-cut function to determine if either body is inactive.
    IsActive() {
        return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
    }
    /// Get collide connected.
    /// Note: modifying the collide connect flag won't work correctly because
    /// the flag is only checked when fixture AABBs begin to overlap.
    GetCollideConnected() {
        return this.m_collideConnected;
    }
    /// Shift the origin for any points stored in world coordinates.
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    ShiftOrigin(newOrigin) { }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJKb2ludC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL3NyYy9keW5hbWljcy9qb2ludHMvYjJKb2ludC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFBRSxRQUFRLEVBQUUsT0FBTyxFQUFFLE1BQU0seUJBQXlCLENBQUM7QUFDNUQsT0FBTyxFQUFFLE1BQU0sRUFBTSxNQUFNLHFCQUFxQixDQUFDO0FBMkJqRCxNQUFNLE9BQU8sVUFBVTtJQUF2QjtRQUNXLFdBQU0sR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDO1FBQy9CLGFBQVEsR0FBRyxDQUFDLENBQUM7UUFDYixhQUFRLEdBQUcsQ0FBQyxDQUFDO0lBZWYsQ0FBQztJQWJDLE9BQU87UUFDTCxJQUFJLENBQUMsTUFBTSxDQUFDLE9BQU8sRUFBRSxDQUFDO1FBQ3RCLElBQUksQ0FBQyxRQUFRLEdBQUcsQ0FBQyxDQUFDO1FBQ2xCLElBQUksQ0FBQyxRQUFRLEdBQUcsQ0FBQyxDQUFDO1FBQ2xCLE9BQU8sSUFBSSxDQUFDO0lBQ2QsQ0FBQztJQUVELEdBQUcsQ0FBQyxDQUFLLEVBQUUsRUFBVSxFQUFFLEVBQVU7UUFDL0IsSUFBSSxDQUFDLE1BQU0sQ0FBQyxJQUFJLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDcEIsSUFBSSxDQUFDLFFBQVEsR0FBRyxFQUFFLENBQUM7UUFDbkIsSUFBSSxDQUFDLFFBQVEsR0FBRyxFQUFFLENBQUM7UUFDbkIsT0FBTyxJQUFJLENBQUM7SUFDZCxDQUFDO0NBQ0Y7QUFFRCw4REFBOEQ7QUFDOUQsNkRBQTZEO0FBQzdELDREQUE0RDtBQUM1RCw4REFBOEQ7QUFDOUQsc0NBQXNDO0FBQ3RDLE1BQU0sT0FBTyxXQUFXO0lBZXRCLFlBQVksS0FBYztRQWRsQixXQUFNLEdBQWtCLElBQUksQ0FBQyxDQUFDLHNEQUFzRDtRQVk1RixTQUFJLEdBQXVCLElBQUksQ0FBQyxDQUFDLHFE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OEI7SUFDOUIsV0FBVztRQUNULE9BQU8sSUFBSSxDQUFDLFVBQVUsQ0FBQztJQUN6QixDQUFDO0lBRUQsOEJBQThCO0lBQzlCLFdBQVcsQ0FBQyxJQUFTO1FBQ25CLElBQUksQ0FBQyxVQUFVLEdBQUcsSUFBSSxDQUFDO0lBQ3pCLENBQUM7SUFFRCwrREFBK0Q7SUFDL0QsUUFBUTtRQUNOLE9BQU8sSUFBSSxDQUFDLE9BQU8sQ0FBQyxRQUFRLEVBQUUsSUFBSSxJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsRUFBRSxDQUFDO0lBQzVELENBQUM7SUFFRCwwQkFBMEI7SUFDMUIseUVBQXlFO0lBQ3pFLGlFQUFpRTtJQUNqRSxtQkFBbUI7UUFDakIsT0FBTyxJQUFJLENBQUMsa0JBQWtCLENBQUM7SUFDakMsQ0FBQztJQUVELGdFQUFnRTtJQUNoRSxnRUFBZ0U7SUFDaEUsV0FBVyxDQUFDLFNBQWEsSUFBUyxDQUFDO0NBUXBDIiwic291cmNlc0NvbnRlbnQiOlsiLypcclxuICogQ29weXJpZ2h0IChjKSAyMDA2LTIwMDcgRXJpbiBDYXR0byBodHRwOi8vd3d3LmJveDJkLm9yZ1xyXG4gKlxyXG4gKiBUaGlzIHNvZnR3YXJlIGlzIHByb3ZpZGVkICdhcy1pcycsIHdpdGhvdXQgYW55IGV4cHJlc3Mgb3IgaW1wbGllZFxyXG4gKiB3YXJyYW50eS4gIEluIG5vIGV2ZW50IHdpbGwgdGhlIGF1dGhvcnMgYmUgaGVsZCBsaWFibGUgZm9yIGFueSBkYW1hZ2VzXHJcbiAqIGFyaXNpbmcgZnJvbSB0aGUgdXNlIG9mIHRoaXMgc29mdHdhcmUuXHJcbiAqIFBlcm1pc3Npb24gaXMgZ3JhbnRlZCB0byBhbnlvbmUgdG8gdXNlIHRoaXMgc29mdHdhcmUgZm9yIGFueSBwdXJwb3NlLFxyXG4gKiBpbmNsdWRpbmcgY29tbWVyY2lhbCBhcHBsaWNhdGlvbnMsIGFuZCB0byBhbHRlciBpdCBhbmQgcmVkaXN0cmlidXRlIGl0XHJcbiAqIGZyZWVseSwgc3ViamVjdCB0byB0aGUgZm9sbG93aW5nIHJlc3RyaWN0aW9uczpcclxuICogMS4gVGhlIG9yaWdpbiBvZiB0aGlzIHNvZnR3YXJlIG11c3Qgbm90IGJlIG1pc3JlcHJlc2VudGVkOyB5b3UgbXVzdCBub3RcclxuICogY2xhaW0gdGhhdCB5b3Ugd3JvdGUgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLiBJZiB5b3UgdXNlIHRoaXMgc29mdHdhcmVcclxuICogaW4gYSBwcm9kdWN0LCBhbiBhY2tub3dsZWRnbWVudCBpbiB0aGUgcHJvZHVjdCBkb2N1bWVudGF0aW9uIHdvdWxkIGJlXHJcbiAqIGFwcHJlY2lhdGVkIGJ1dCBpcyBub3QgcmVxdWlyZWQuXHJcbiAqIDIuIEFsdGVyZWQgc291cmNlIHZlcnNpb25zIG11c3QgYmUgcGxhaW5seSBtYXJrZWQgYXMgc3VjaCwgYW5kIG11c3Qgbm90IGJlXHJcbiAqIG1pc3JlcHJlc2VudGVkIGFzIGJlaW5nIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS5cclxuICogMy4gVGhpcyBub3RpY2UgbWF5IG5vdCBiZSByZW1vdmVkIG9yIGFsdGVyZWQgZnJvbSBhbnkgc291cmNlIGRpc3RyaWJ1dGlvbi5cclxuICovXHJcblxyXG5pbXBvcnQgeyBiMkFzc2VydCwgYjJNYXliZSB9IGZyb20gJy4uLy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJWZWMyLCBYWSB9IGZyb20gJy4uLy4uL2NvbW1vbi9iMk1hdGgnO1xyXG5pbXBvcnQgeyBiMkJvZHkgfSBmcm9tICcuLi9iMkJvZHknO1xyXG5pbXBvcnQgeyBiMlNvbHZlckRhdGEgfSBmcm9tICcuLi9iMlRpbWVTdGVwJztcclxuXHJcbmV4cG9ydCBjb25zdCBlbnVtIGIySm9pbnRUeXBlIHtcclxuICBlX3Vua25vd25Kb2ludCA9IDAsXHJcbiAgZV9yZXZvbHV0ZUpvaW50ID0gMSxcclxuICBlX3ByaXNtYXRpY0pvaW50ID0gMixcclxuICBlX2Rpc3RhbmNlSm9pbnQgPSAzLFxyXG4gIGVfcHVsbGV5Sm9pbnQgPSA0LFxyXG4gIGVfbW91c2VKb2ludCA9IDUsXHJcbiAgZV9nZWFySm9pbnQgPSA2LFxyXG4gIGVfd2hlZWxKb2ludCA9IDcsXHJcbiAgZV93ZWxkSm9pbnQgPSA4LFxyXG4gIGVfZnJpY3Rpb25Kb2ludCA9IDksXHJcbiAgZV9yb3BlSm9pbnQgPSAxMCxcclxuICBlX21vdG9ySm9pbnQgPSAxMSxcclxuICBlX2FyZWFKb2ludCA9IDEyLFxyXG59XHJcblxyXG5leHBvcnQgY29uc3QgZW51bSBiMkxpbWl0U3RhdGUge1xyXG4gIGVfaW5hY3RpdmVMaW1pdCA9IDAsXHJcbiAgZV9hdExvd2VyTGltaXQgPSAxLFxyXG4gIGVfYXRVcHBlckxpbWl0ID0gMixcclxuICBlX2VxdWFsTGltaXRzID0gMyxcclxufVxyXG5cclxuZXhwb3J0IGNsYXNzIGIySmFjb2JpYW4ge1xyXG4gIHJlYWRvbmx5IGxpbmVhciA9IG5ldyBiMlZlYzIoKTtcclxuICBhbmd1bGFyQSA9IDA7XHJcbiAgYW5ndWxhckIgPSAwO1xyXG5cclxuICBTZXRaZXJvKCk6IGIySmFjb2JpYW4ge1xyXG4gICAgdGhpcy5saW5lYXIuU2V0WmVybygpO1xyXG4gICAgdGhpcy5hbmd1bGFyQSA9IDA7XHJcbiAgICB0aGlzLmFuZ3VsYXJCID0gMDtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2V0KHg6IFhZLCBhMTogbnVtYmVyLCBhMjogbnVtYmVyKTogYjJKYWNvYmlhbiB7XHJcbiAgICB0aGlzLmxpbmVhci5Db3B5KHgpO1xyXG4gICAgdGhpcy5hbmd1bGFyQSA9IGExO1xyXG4gICAgdGhpcy5hbmd1bGFyQiA9IGEyO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gQSBqb2ludCBlZGdlIGlzIHVzZWQgdG8gY29ubmVjdCBib2RpZXMgYW5kIGpvaW50cyB0b2dldGhlclxyXG4vLy8gaW4gYSBqb2ludCBncmFwaCB3aGVyZSBlYWNoIGJvZHkgaXMgYSBub2RlIGFuZCBlYWNoIGpvaW50XHJcbi8vLyBpcyBhbiBlZGdlLiBBIGpvaW50IGVkZ2UgYmVsb25ncyB0byBhIGRvdWJseSBsaW5rZWQgbGlzdFxyXG4vLy8gbWFpbnRhaW5lZCBpbiBlYWNoIGF0dGFjaGVkIGJvZHkuIEVhY2ggam9pbnQgaGFzIHR3byBqb2ludFxyXG4vLy8gbm9kZXMsIG9uZSBmb3IgZWFjaCBhdHRhY2hlZCBib2R5LlxyXG5leHBvcnQgY2xhc3MgYjJKb2ludEVkZ2Uge1xyXG4gIHByaXZhdGUgX290aGVyOiBiMkJvZHkgfCBudWxsID0gbnVsbDsgLy8vPCBwcm92aWRlcyBxdWljayBhY2Nlc3MgdG8gdGhlIG90aGVyIGJvZHkgYXR0YWNoZWQuXHJcbiAgZ2V0IG90aGVyKCk6IGIyQm9keSB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRoaXMuX290aGVyICE9PSBudWxsKTtcclxuICAgIHJldHVybiB0aGlzLl9vdGhlciE7XHJcbiAgfVxyXG5cclxuICBzZXQgb3RoZXIodmFsdWU6IGIyQm9keSkge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCh0aGlzLl9vdGhlciA9PT0gbnVsbCk7XHJcbiAgICB0aGlzLl9vdGhlciA9IHZhbHVlO1xyXG4gIH1cclxuXHJcbiAgcmVhZG9ubHkgam9pbnQ6IGIySm9pbnQ7IC8vLzwgdGhlIGpvaW50XHJcbiAgcHJldjogYjJKb2ludEVkZ2UgfCBudWxsID0gbnVsbDsgLy8vPCB0aGUgcHJldmlvdXMgam9pbnQgZWRnZSBpbiB0aGUgYm9keSdzIGpvaW50IGxpc3RcclxuICBuZXh0OiBiMkpvaW50RWRnZSB8IG51bGwgPSBudWxsOyAvLy88IHRoZSBuZXh0IGpvaW50IGVkZ2UgaW4gdGhlIGJvZHkncyBqb2ludCBsaXN0XHJcbiAgY29uc3RydWN0b3Ioam9pbnQ6IGIySm9pbnQpIHtcclxuICAgIHRoaXMuam9pbnQgPSBqb2ludDtcclxuICB9XHJcblxyXG4gIFJlc2V0KCk6IHZvaWQge1xyXG4gICAgdGhpcy5fb3RoZXIgPSBudWxsO1xyXG4gICAgdGhpcy5wcmV2ID0gbnVsbDtcclxuICAgIHRoaXMubmV4dCA9IG51bGw7XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gSm9pbnQgZGVmaW5pdGlvbnMgYXJlIHVzZWQgdG8gY29uc3RydWN0IGpvaW50cy5cclxuZXhwb3J0IGludGVyZmFjZSBiMklKb2ludERlZiB7XHJcbiAgLy8vIFRoZSBqb2ludCB0eXBlIGlzIHNldCBhdXRvbWF0aWNhbGx5IGZvciBjb25jcmV0ZSBqb2ludCB0eXBlcy5cclxuICB0eXBlOiBiMkpvaW50VHlwZTtcclxuXHJcbiAgLy8vIFVzZSB0aGlzIHRvIGF0dGFjaCBhcHBsaWNhdGlvbiBzcGVjaWZpYyBkYXRhIHRvIHlvdXIgam9pbnRzLlxyXG4gIHVzZXJEYXRhPzogYW55O1xyXG5cclxuICAvLy8gVGhlIGZpcnN0IGF0dGFjaGVkIGJvZHkuXHJcbiAgYm9keUE6IGIyQm9keTtcclxuXHJcbiAgLy8vIFRoZSBzZWNvbmQgYXR0YWNoZWQgYm9keS5cclxuICBib2R5QjogYjJCb2R5O1xyXG5cclxuICAvLy8gU2V0IHRoaXMgZmxhZyB0byB0cnVlIGlmIHRoZSBhdHRhY2hlZCBib2RpZXMgc2hvdWxkIGNvbGxpZGUuXHJcbiAgY29sbGlkZUNvbm5lY3RlZD86IGJvb2xlYW47XHJcbn1cclxuXHJcbi8vLyBKb2ludCBkZWZpbml0aW9ucyBhcmUgdXNlZCB0byBjb25zdHJ1Y3Qgam9pbnRzLlxyXG5leHBvcnQgYWJzdHJhY3QgY2xhc3MgYjJKb2ludERlZiBpbXBsZW1lbnRzIGIySUpvaW50RGVmIHtcclxuICAvLy8gVGhlIGpvaW50IHR5cGUgaXMgc2V0IGF1dG9tYXRpY2FsbHkgZm9yIGNvbmNyZXRlIGpvaW50IHR5cGVzLlxyXG4gIHJlYWRvbmx5IHR5cGU6IGIySm9pbnRUeXBlID0gYjJKb2ludFR5cGUuZV91bmtub3duSm9pbnQ7XHJcblxyXG4gIC8vLyBVc2UgdGhpcyB0byBhdHRhY2ggYXBwbGljYXRpb24gc3BlY2lmaWMgZGF0YSB0byB5b3VyIGpvaW50cy5cclxuICB1c2VyRGF0YTogYW55ID0gbnVsbDtcclxuXHJcbiAgLy8vIFRoZSBmaXJzdCBhdHRhY2hlZCBib2R5LlxyXG4gIGJvZHlBITogYjJCb2R5O1xyXG5cclxuICAvLy8gVGhlIHNlY29uZCBhdHRhY2hlZCBib2R5LlxyXG4gIGJvZHlCITogYjJCb2R5O1xyXG5cclxuICAvLy8gU2V0IHRoaXMgZmxhZyB0byB0cnVlIGlmIHRoZSBhdHRhY2hlZCBib2RpZXMgc2hvdWxkIGNvbGxpZGUuXHJcbiAgY29sbGlkZUNvbm5lY3RlZCA9IGZhbHNlO1xyXG5cclxuICBjb25zdHJ1Y3Rvcih0eXBlOiBiMkpvaW50VHlwZSkge1xyXG4gICAgdGhpcy50eXBlID0gdHlwZTtcclxuICB9XHJcbn1cclxuXHJcbi8vLyBUaGUgYmFzZSBqb2ludCBjbGFzcy4gSm9pbnRzIGFyZSB1c2VkIHRvIGNvbnN0cmFpbnQgdHdvIGJvZGllcyB0b2dldGhlciBpblxyXG4vLy8gdmFyaW91cyBmYXNoaW9ucy4gU29tZSBqb2ludHMgYWxzbyBmZWF0dXJlIGxpbWl0cyBhbmQgbW90b3JzLlxyXG5leHBvcnQgYWJzdHJhY3QgY2xhc3MgYjJKb2ludCB7XHJcbiAgcmVhZG9ubHkgbV90eXBlOiBiMkpvaW50VHlwZSA9IGIySm9pbnRUeXBlLmVfdW5rbm93bkpvaW50O1xyXG4gIG1fcHJldjogYjJKb2ludCB8IG51bGwgPSBudWxsO1xyXG4gIG1fbmV4dDogYjJKb2ludCB8IG51bGwgPSBudWxsO1xyXG4gIHJlYWRvbmx5IG1fZWRnZUE6IGIySm9pbnRFZGdlID0gbmV3IGIySm9pbnRFZGdlKHRoaXMpO1xyXG4gIHJlYWRvbmx5IG1fZWRnZUI6IGIySm9pbnRFZGdlID0gbmV3IGIySm9pbnRFZGdlKHRoaXMpO1xyXG4gIG1fYm9keUE6IGIyQm9keTtcclxuICBtX2JvZHlCOiBiMkJvZHk7XHJcblxyXG4gIG1faXNsYW5kRmxhZyA9IGZhbHNlO1xyXG4gIG1fY29sbGlkZUNvbm5lY3RlZCA9IGZhbHNlO1xyXG5cclxuICBtX3VzZXJEYXRhOiBhbnkgPSBudWxsO1xyXG5cclxuICBfbG9nSW5kZXggPSAwO1xyXG5cclxuICBjb25zdHJ1Y3RvcihkZWY6IGIySUpvaW50RGVmKSB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KGRlZi5ib2R5QSAhPT0gZGVmLmJvZHlCKTtcclxuXHJcbiAgICB0aGlzLm1fdHlwZSA9IGRlZi50eXBlO1xyXG4gICAgdGhpcy5tX2VkZ2VBLm90aGVyID0gZGVmLmJvZHlCO1xyXG4gICAgdGhpcy5tX2VkZ2VCLm90aGVyID0gZGVmLmJvZHlBO1xyXG4gICAgdGhpcy5tX2JvZHlBID0gZGVmLmJvZHlBO1xyXG4gICAgdGhpcy5tX2JvZHlCID0gZGVmLmJvZHlCO1xyXG5cclxuICAgIHRoaXMubV9jb2xsaWRlQ29ubmVjdGVkID0gYjJNYXliZShkZWYuY29sbGlkZUNvbm5lY3RlZCwgZmFsc2UpO1xyXG5cclxuICAgIHRoaXMubV91c2VyRGF0YSA9IGIyTWF5YmUoZGVmLnVzZXJEYXRhLCBudWxsKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIHR5cGUgb2YgdGhlIGNvbmNyZXRlIGpvaW50LlxyXG4gIEdldFR5cGUoKTogYjJKb2ludFR5cGUge1xyXG4gICAgcmV0dXJuIHRoaXMubV90eXBlO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgZmlyc3QgYm9keSBhdHRhY2hlZCB0byB0aGlzIGpvaW50LlxyXG4gIEdldEJvZHlBKCk6IGIyQm9keSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2JvZHlBO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgc2Vjb25kIGJvZHkgYXR0YWNoZWQgdG8gdGhpcyBqb2ludC5cclxuICBHZXRCb2R5QigpOiBiMkJvZHkge1xyXG4gICAgcmV0dXJuIHRoaXMubV9ib2R5QjtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGFuY2hvciBwb2ludCBvbiBib2R5QSBpbiB3b3JsZCBjb29yZGluYXRlcy5cclxuICBhYnN0cmFjdCBHZXRBbmNob3JBPFQgZXh0ZW5kcyBYWT4ob3V0OiBUKTogVDtcclxuXHJcbiAgLy8vIEdldCB0aGUgYW5jaG9yIHBvaW50IG9uIGJvZHlCIGluIHdvcmxkIGNvb3JkaW5hdGVzLlxyXG4gIGFic3RyYWN0IEdldEFuY2hvckI8VCBleHRlbmRzIFhZPihvdXQ6IFQpOiBUO1xyXG5cclxuICAvLy8gR2V0IHRoZSByZWFjdGlvbiBmb3JjZSBvbiBib2R5QiBhdCB0aGUgam9pbnQgYW5jaG9yIGluIE5ld3RvbnMuXHJcbiAgYWJzdHJhY3QgR2V0UmVhY3Rpb25Gb3JjZTxUIGV4dGVuZHMgWFk+KGludl9kdDogbnVtYmVyLCBvdXQ6IFQpOiBUO1xyXG5cclxuICAvLy8gR2V0IHRoZSByZWFjdGlvbiB0b3JxdWUgb24gYm9keUIgaW4gTiptLlxyXG4gIGFic3RyYWN0IEdldFJlYWN0aW9uVG9ycXVlKGludl9kdDogbnVtYmVyKTogbnVtYmVyO1xyXG5cclxuICAvLy8gR2V0IHRoZSBuZXh0IGpvaW50IHRoZSB3b3JsZCBqb2ludCBsaXN0LlxyXG4gIEdldE5leHQoKTogYjJKb2ludCB8IG51bGwge1xyXG4gICAgcmV0dXJuIHRoaXMubV9uZXh0O1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgdXNlciBkYXRhIHBvaW50ZXIuXHJcbiAgR2V0VXNlckRhdGEoKTogYW55IHtcclxuICAgIHJldHVybiB0aGlzLm1fdXNlckRhdGE7XHJcbiAgfVxyXG5cclxuICAvLy8gU2V0IHRoZSB1c2VyIGRhdGEgcG9pbnRlci5cclxuICBTZXRVc2VyRGF0YShkYXRhOiBhbnkpOiB2b2lkIHtcclxuICAgIHRoaXMubV91c2VyRGF0YSA9IGRhdGE7XHJcbiAgfVxyXG5cclxuICAvLy8gU2hvcnQtY3V0IGZ1bmN0aW9uIHRvIGRldGVybWluZSBpZiBlaXRoZXIgYm9keSBpcyBpbmFjdGl2ZS5cclxuICBJc0FjdGl2ZSgpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUEuSXNBY3RpdmUoKSAmJiB0aGlzLm1fYm9keUIuSXNBY3RpdmUoKTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgY29sbGlkZSBjb25uZWN0ZWQuXHJcbiAgLy8vIE5vdGU6IG1vZGlmeWluZyB0aGUgY29sbGlkZSBjb25uZWN0IGZsYWcgd29uJ3Qgd29yayBjb3JyZWN0bHkgYmVjYXVzZVxyXG4gIC8vLyB0aGUgZmxhZyBpcyBvbmx5IGNoZWNrZWQgd2hlbiBmaXh0dXJlIEFBQkJzIGJlZ2luIHRvIG92ZXJsYXAuXHJcbiAgR2V0Q29sbGlkZUNvbm5lY3RlZCgpOiBib29sZWFuIHtcclxuICAgIHJldHVybiB0aGlzLm1fY29sbGlkZUNvbm5lY3RlZDtcclxuICB9XHJcblxyXG4gIC8vLyBTaGlmdCB0aGUgb3JpZ2luIGZvciBhbnkgcG9pbnRzIHN0b3JlZCBpbiB3b3JsZCBjb29yZGluYXRlcy5cclxuICAvLyBlc2xpbnQtZGlzYWJsZS1uZXh0LWxpbmUgQHR5cGVzY3JpcHQtZXNsaW50L25vLWVtcHR5LWZ1bmN0aW9uXHJcbiAgU2hpZnRPcmlnaW4obmV3T3JpZ2luOiBYWSk6IHZvaWQge31cclxuXHJcbiAgYWJzdHJhY3QgSW5pdFZlbG9jaXR5Q29uc3RyYWludHMoZGF0YTogYjJTb2x2ZXJEYXRhKTogdm9pZDtcclxuXHJcbiAgYWJzdHJhY3QgU29sdmVWZWxvY2l0eUNvbnN0cmFpbnRzKGRhdGE6IGIyU29sdmVyRGF0YSk6IHZvaWQ7XHJcblxyXG4gIC8vIFRoaXMgcmV0dXJucyB0cnVlIGlmIHRoZSBwb3NpdGlvbiBlcnJvcnMgYXJlIHdpdGhpbiB0b2xlcmFuY2UuXHJcbiAgYWJzdHJhY3QgU29sdmVQb3NpdGlvbkNvbnN0cmFpbnRzKGRhdGE6IGIyU29sdmVyRGF0YSk6IGJvb2xlYW47XHJcbn1cclxuIl19