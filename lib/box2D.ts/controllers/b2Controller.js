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
/**
 * A controller edge is used to connect bodies and controllers
 * together in a bipartite graph.
 */
export class b2ControllerEdge {
    constructor(controller, body) {
        this.prevBody = null; ///< the previous controller edge in the controllers's joint list
        this.nextBody = null; ///< the next controller edge in the controllers's joint list
        this.prevController = null; ///< the previous controller edge in the body's joint list
        this.nextController = null; ///< the next controller edge in the body's joint list
        this.controller = controller;
        this.body = body;
    }
}
/**
 * Base class for controllers. Controllers are a convience for
 * encapsulating common per-step functionality.
 */
export class b2Controller {
    constructor() {
        // m_world: b2World;
        this.m_bodyList = null;
        this.m_bodyCount = 0;
        this.m_prev = null;
        this.m_next = null;
    }
    /**
     * Get the next controller in the world's body list.
     */
    GetNext() {
        return this.m_next;
    }
    /**
     * Get the previous controller in the world's body list.
     */
    GetPrev() {
        return this.m_prev;
    }
    /**
     * Get the parent world of this body.
     */
    // GetWorld() {
    //   return this.m_world;
    // }
    /**
     * Get the attached body list
     */
    GetBodyList() {
        return this.m_bodyList;
    }
    /**
     * Adds a body to the controller list.
     */
    AddBody(body) {
        const edge = new b2ControllerEdge(this, body);
        //Add edge to controller list
        edge.nextBody = this.m_bodyList;
        edge.prevBody = null;
        if (this.m_bodyList) {
            this.m_bodyList.prevBody = edge;
        }
        this.m_bodyList = edge;
        ++this.m_bodyCount;
        //Add edge to body list
        edge.nextController = body.m_controllerList;
        edge.prevController = null;
        if (body.m_controllerList) {
            body.m_controllerList.prevController = edge;
        }
        body.m_controllerList = edge;
        ++body.m_controllerCount;
    }
    /**
     * Removes a body from the controller list.
     */
    RemoveBody(body) {
        //Assert that the controller is not empty
        if (this.m_bodyCount <= 0) {
            throw new Error();
        }
        //Find the corresponding edge
        /*b2ControllerEdge*/
        let edge = this.m_bodyList;
        while (edge && edge.body !== body) {
            edge = edge.nextBody;
        }
        //Assert that we are removing a body that is currently attached to the controller
        if (edge === null) {
            throw new Error();
        }
        //Remove edge from controller list
        if (edge.prevBody) {
            edge.prevBody.nextBody = edge.nextBody;
        }
        if (edge.nextBody) {
            edge.nextBody.prevBody = edge.prevBody;
        }
        if (this.m_bodyList === edge) {
            this.m_bodyList = edge.nextBody;
        }
        --this.m_bodyCount;
        //Remove edge from body list
        if (edge.nextController) {
            edge.nextController.prevController = edge.prevController;
        }
        if (edge.prevController) {
            edge.prevController.nextController = edge.nextController;
        }
        if (body.m_controllerList === edge) {
            body.m_controllerList = edge.nextController;
        }
        --body.m_controllerCount;
    }
    /**
     * Removes all bodies from the controller list.
     */
    Clear() {
        while (this.m_bodyList) {
            this.RemoveBody(this.m_bodyList.body);
        }
        this.m_bodyCount = 0;
    }
}
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb250cm9sbGVyLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL2NvbnRyb2xsZXJzL2IyQ29udHJvbGxlci50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQVFIOzs7R0FHRztBQUNILE1BQU0sT0FBTyxnQkFBZ0I7SUFPM0IsWUFBWSxVQUF3QixFQUFFLElBQVk7UUFKbEQsYUFBUSxHQUE0QixJQUFJLENBQUMsQ0FBQyxpRUFBaUU7UUFDM0csYUFBUSxHQUE0QixJQUFJLENBQUMsQ0FBQyw2REFBNkQ7UUFDdkcsbUJBQWMsR0FBNEIsSUFBSSxDQUFDLENBQUMsMERBQTBEO1FBQzFHLG1CQUFjLEdBQTRCLElBQUksQ0FBQyxDQUFDLHNEQUFzRDtRQUVwRyxJQUFJLENBQUMsVUFBVSxHQUFHLFVBQVUsQ0FBQztRQUM3QixJQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQztJQUNuQixDQUFDO0NBQ0Y7QUFFRDs7O0dBR0c7QUFDSCxNQUFNLE9BQWdCLFlBQVk7SUFBbEM7UUFDRSxvQkFBb0I7UUFDcEIsZUFBVSxHQUE0QixJQUFJLENBQUM7UUFDM0MsZ0JBQVcsR0FBRyxDQUFDLENBQUM7UUFDaEIsV0FBTSxHQUF3QixJQUFJLENBQUM7UUFDbkMsV0FBTSxHQUF3QixJQUFJLENBQUM7SUF5SHJDLENBQUM7SUE3R0M7O09BRUc7SUFDSCxPQUFPO1FBQ0wsT0FBTyxJQUFJLENBQUMsTUFBTSxDQUFDO0lBQ3JCLENBQUM7SUFFRDs7T0FFRztJQUNILE9BQU87UUFDTCxPQUFPLElBQUksQ0FBQyxNQUFNLENBQUM7SUFDckIsQ0FBQztJQUVEOztPQUVHO0lBQ0gsZUFBZTtJQUNmLHlCQUF5QjtJQUN6QixJQUFJO0lBRUo7O09BRUc7SUFDSCxXQUFXO1FBQ1QsT0FBTyxJQUFJLENBQUMsVUFBVSxDQUFDO0lBQ3pCLENBQUM7SUFFRDs7T0FFRztJQUNILE9BQU8sQ0FBQyxJQUFZO1FB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aWdodCAoYykgMjAwNi0yMDA5IEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuLy8gI2lmIEIyX0VOQUJMRV9DT05UUk9MTEVSXHJcblxyXG5pbXBvcnQgeyBiMkJvZHkgfSBmcm9tICcuLi9keW5hbWljcy9iMkJvZHknO1xyXG5pbXBvcnQgeyBiMlRpbWVTdGVwIH0gZnJvbSAnLi4vZHluYW1pY3MvYjJUaW1lU3RlcCc7XHJcbmltcG9ydCB7IGIyRHJhdyB9IGZyb20gJy4uL2NvbW1vbi9iMkRyYXcnO1xyXG5cclxuLyoqXHJcbiAqIEEgY29udHJvbGxlciBlZGdlIGlzIHVzZWQgdG8gY29ubmVjdCBib2RpZXMgYW5kIGNvbnRyb2xsZXJzXHJcbiAqIHRvZ2V0aGVyIGluIGEgYmlwYXJ0aXRlIGdyYXBoLlxyXG4gKi9cclxuZXhwb3J0IGNsYXNzIGIyQ29udHJvbGxlckVkZ2Uge1xyXG4gIHJlYWRvbmx5IGNvbnRyb2xsZXI6IGIyQ29udHJvbGxlcjsgLy8vPCBwcm92aWRlcyBxdWljayBhY2Nlc3MgdG8gb3RoZXIgZW5kIG9mIHRoaXMgZWRnZS5cclxuICByZWFkb25seSBib2R5OiBiMkJvZHk7IC8vLzwgdGhlIGJvZHlcclxuICBwcmV2Qm9keTogYjJDb250cm9sbGVyRWRnZSB8IG51bGwgPSBudWxsOyAvLy88IHRoZSBwcmV2aW91cyBjb250cm9sbGVyIGVkZ2UgaW4gdGhlIGNvbnRyb2xsZXJzJ3Mgam9pbnQgbGlzdFxyXG4gIG5leHRCb2R5OiBiMkNvbnRyb2xsZXJFZGdlIHwgbnVsbCA9IG51bGw7IC8vLzwgdGhlIG5leHQgY29udHJvbGxlciBlZGdlIGluIHRoZSBjb250cm9sbGVycydzIGpvaW50IGxpc3RcclxuICBwcmV2Q29udHJvbGxlcjogYjJDb250cm9sbGVyRWRnZSB8IG51bGwgPSBudWxsOyAvLy88IHRoZSBwcmV2aW91cyBjb250cm9sbGVyIGVkZ2UgaW4gdGhlIGJvZHkncyBqb2ludCBsaXN0XHJcbiAgbmV4dENvbnRyb2xsZXI6IGIyQ29udHJvbGxlckVkZ2UgfCBudWxsID0gbnVsbDsgLy8vPCB0aGUgbmV4dCBjb250cm9sbGVyIGVkZ2UgaW4gdGhlIGJvZHkncyBqb2ludCBsaXN0XHJcbiAgY29uc3RydWN0b3IoY29udHJvbGxlcjogYjJDb250cm9sbGVyLCBib2R5OiBiMkJvZHkpIHtcclxuICAgIHRoaXMuY29udHJvbGxlciA9IGNvbnRyb2xsZXI7XHJcbiAgICB0aGlzLmJvZHkgPSBib2R5O1xyXG4gIH1cclxufVxyXG5cclxuLyoqXHJcbiAqIEJhc2UgY2xhc3MgZm9yIGNvbnRyb2xsZXJzLiBDb250cm9sbGVycyBhcmUgYSBjb252aWVuY2UgZm9yXHJcbiAqIGVuY2Fwc3VsYXRpbmcgY29tbW9uIHBlci1zdGVwIGZ1bmN0aW9uYWxpdHkuXHJcbiAqL1xyXG5leHBvcnQgYWJzdHJhY3QgY2xhc3MgYjJDb250cm9sbGVyIHtcclxuICAvLyBtX3dvcmxkOiBiMldvcmxkO1xyXG4gIG1fYm9keUxpc3Q6IGIyQ29udHJvbGxlckVkZ2UgfCBudWxsID0gbnVsbDtcclxuICBtX2JvZHlDb3VudCA9IDA7XHJcbiAgbV9wcmV2OiBiMkNvbnRyb2xsZXIgfCBudWxsID0gbnVsbDtcclxuICBtX25leHQ6IGIyQ29udHJvbGxlciB8IG51bGwgPSBudWxsO1xyXG5cclxuICAvKipcclxuICAgKiBDb250cm9sbGVycyBvdmVycmlkZSB0aGlzIHRvIGltcGxlbWVudCBwZXItc3RlcCBmdW5jdGlvbmFsaXR5LlxyXG4gICAqL1xyXG4gIGFic3RyYWN0IFN0ZXAoc3RlcDogYjJUaW1lU3RlcCk6IHZvaWQ7XHJcblxyXG4gIC8qKlxyXG4gICAqIENvbnRyb2xsZXJzIG92ZXJyaWRlIHRoaXMgdG8gcHJvdmlkZSBkZWJ1ZyBkcmF3aW5nLlxyXG4gICAqL1xyXG4gIGFic3RyYWN0IERyYXcoZGVidWdEcmF3OiBiMkRyYXcpOiB2b2lkO1xyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIG5leHQgY29udHJvbGxlciBpbiB0aGUgd29ybGQncyBib2R5IGxpc3QuXHJcbiAgICovXHJcbiAgR2V0TmV4dCgpOiBiMkNvbnRyb2xsZXIgfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fbmV4dDtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgcHJldmlvdXMgY29udHJvbGxlciBpbiB0aGUgd29ybGQncyBib2R5IGxpc3QuXHJcbiAgICovXHJcbiAgR2V0UHJldigpOiBiMkNvbnRyb2xsZXIgfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fcHJldjtcclxuICB9XHJcblxyXG4gIC8qKlxyXG4gICAqIEdldCB0aGUgcGFyZW50IHdvcmxkIG9mIHRoaXMgYm9keS5cclxuICAgKi9cclxuICAvLyBHZXRXb3JsZCgpIHtcclxuICAvLyAgIHJldHVybiB0aGlzLm1fd29ybGQ7XHJcbiAgLy8gfVxyXG5cclxuICAvKipcclxuICAgKiBHZXQgdGhlIGF0dGFjaGVkIGJvZHkgbGlzdFxyXG4gICAqL1xyXG4gIEdldEJvZHlMaXN0KCk6IGIyQ29udHJvbGxlckVkZ2UgfCBudWxsIHtcclxuICAgIHJldHVybiB0aGlzLm1fYm9keUxpc3Q7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBBZGRzIGEgYm9keSB0byB0aGUgY29udHJvbGxlciBsaXN0LlxyXG4gICAqL1xyXG4gIEFkZEJvZHkoYm9keTogYjJCb2R5KTogdm9pZCB7XHJcbiAgICBjb25zdCBlZGdlID0gbmV3IGIyQ29udHJvbGxlckVkZ2UodGhpcywgYm9keSk7XHJcblxyXG4gICAgLy9BZGQgZWRnZSB0byBjb250cm9sbGVyIGxpc3RcclxuICAgIGVkZ2UubmV4dEJvZHkgPSB0aGlzLm1fYm9keUxpc3Q7XHJcbiAgICBlZGdlLnByZXZCb2R5ID0gbnVsbDtcclxuICAgIGlmICh0aGlzLm1fYm9keUxpc3QpIHtcclxuICAgICAgdGhpcy5tX2JvZHlMaXN0LnByZXZCb2R5ID0gZWRnZTtcclxuICAgIH1cclxuICAgIHRoaXMubV9ib2R5TGlzdCA9IGVkZ2U7XHJcbiAgICArK3RoaXMubV9ib2R5Q291bnQ7XHJcblxyXG4gICAgLy9BZGQgZWRnZSB0byBib2R5IGxpc3RcclxuICAgIGVkZ2UubmV4dENvbnRyb2xsZXIgPSBib2R5Lm1fY29udHJvbGxlckxpc3Q7XHJcbiAgICBlZGdlLnByZXZDb250cm9sbGVyID0gbnVsbDtcclxuICAgIGlmIChib2R5Lm1fY29udHJvbGxlckxpc3QpIHtcclxuICAgICAgYm9keS5tX2NvbnRyb2xsZXJMaXN0LnByZXZDb250cm9sbGVyID0gZWRnZTtcclxuICAgIH1cclxuICAgIGJvZHkubV9jb250cm9sbGVyTGlzdCA9IGVkZ2U7XHJcbiAgICArK2JvZHkubV9jb250cm9sbGVyQ291bnQ7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBSZW1vdmVzIGEgYm9keSBmcm9tIHRoZSBjb250cm9sbGVyIGxpc3QuXHJcbiAgICovXHJcbiAgUmVtb3ZlQm9keShib2R5OiBiMkJvZHkpOiB2b2lkIHtcclxuICAgIC8vQXNzZXJ0IHRoYXQgdGhlIGNvbnRyb2xsZXIgaXMgbm90IGVtcHR5XHJcbiAgICBpZiAodGhpcy5tX2JvZHlDb3VudCA8PSAwKSB7XHJcbiAgICAgIHRocm93IG5ldyBFcnJvcigpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vRmluZCB0aGUgY29ycmVzcG9uZGluZyBlZGdlXHJcbiAgICAvKmIyQ29udHJvbGxlckVkZ2UqL1xyXG4gICAgbGV0IGVkZ2UgPSB0aGlzLm1fYm9keUxpc3Q7XHJcbiAgICB3aGlsZSAoZWRnZSAmJiBlZGdlLmJvZHkgIT09IGJvZHkpIHtcclxuICAgICAgZWRnZSA9IGVkZ2UubmV4dEJvZHk7XHJcbiAgICB9XHJcblxyXG4gICAgLy9Bc3NlcnQgdGhhdCB3ZSBhcmUgcmVtb3ZpbmcgYSBib2R5IHRoYXQgaXMgY3VycmVudGx5IGF0dGFjaGVkIHRvIHRoZSBjb250cm9sbGVyXHJcbiAgICBpZiAoZWRnZSA9PT0gbnVsbCkge1xyXG4gICAgICB0aHJvdyBuZXcgRXJyb3IoKTtcclxuICAgIH1cclxuXHJcbiAgICAvL1JlbW92ZSBlZGdlIGZyb20gY29udHJvbGxlciBsaXN0XHJcbiAgICBpZiAoZWRnZS5wcmV2Qm9keSkge1xyXG4gICAgICBlZGdlLnByZXZCb2R5Lm5leHRCb2R5ID0gZWRnZS5uZXh0Qm9keTtcclxuICAgIH1cclxuICAgIGlmIChlZGdlLm5leHRCb2R5KSB7XHJcbiAgICAgIGVkZ2UubmV4dEJvZHkucHJldkJvZHkgPSBlZGdlLnByZXZCb2R5O1xyXG4gICAgfVxyXG4gICAgaWYgKHRoaXMubV9ib2R5TGlzdCA9PT0gZWRnZSkge1xyXG4gICAgICB0aGlzLm1fYm9keUxpc3QgPSBlZGdlLm5leHRCb2R5O1xyXG4gICAgfVxyXG4gICAgLS10aGlzLm1fYm9keUNvdW50O1xyXG5cclxuICAgIC8vUmVtb3ZlIGVkZ2UgZnJvbSBib2R5IGxpc3RcclxuICAgIGlmIChlZGdlLm5leHRDb250cm9sbGVyKSB7XHJcbiAgICAgIGVkZ2UubmV4dENvbnRyb2xsZXIucHJldkNvbnRyb2xsZXIgPSBlZGdlLnByZXZDb250cm9sbGVyO1xyXG4gICAgfVxyXG4gICAgaWYgKGVkZ2UucHJldkNvbnRyb2xsZXIpIHtcclxuICAgICAgZWRnZS5wcmV2Q29udHJvbGxlci5uZXh0Q29udHJvbGxlciA9IGVkZ2UubmV4dENvbnRyb2xsZXI7XHJcbiAgICB9XHJcbiAgICBpZiAoYm9keS5tX2NvbnRyb2xsZXJMaXN0ID09PSBlZGdlKSB7XHJcbiAgICAgIGJvZHkubV9jb250cm9sbGVyTGlzdCA9IGVkZ2UubmV4dENvbnRyb2xsZXI7XHJcbiAgICB9XHJcbiAgICAtLWJvZHkubV9jb250cm9sbGVyQ291bnQ7XHJcbiAgfVxyXG5cclxuICAvKipcclxuICAgKiBSZW1vdmVzIGFsbCBib2RpZXMgZnJvbSB0aGUgY29udHJvbGxlciBsaXN0LlxyXG4gICAqL1xyXG4gIENsZWFyKCk6IHZvaWQge1xyXG4gICAgd2hpbGUgKHRoaXMubV9ib2R5TGlzdCkge1xyXG4gICAgICB0aGlzLlJlbW92ZUJvZHkodGhpcy5tX2JvZHlMaXN0LmJvZHkpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubV9ib2R5Q291bnQgPSAwO1xyXG4gIH1cclxufVxyXG5cclxuLy8gI2VuZGlmXHJcbiJdfQ==