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
import { b2Assert } from '../common/b2Settings';
import { b2AABB } from '../collision/b2Collision';
import { b2MassData } from '../collision/shapes/b2Shape';
/// This holds contact filtering data.
export class b2Filter {
    constructor() {
        /// The collision category bits. Normally you would just set one bit.
        this.categoryBits = 0x0001;
        /// The collision mask bits. This states the categories that this
        /// shape would accept for collision.
        this.maskBits = 0xffff;
        /// Collision groups allow a certain group of objects to never collide (negative)
        /// or always collide (positive). Zero means no collision group. Non-zero group
        /// filtering always wins against the mask bits.
        this.groupIndex = 0;
    }
    Clone() {
        return new b2Filter().Copy(this);
    }
    Copy(other) {
        !!B2_DEBUG && b2Assert(this !== other);
        this.categoryBits = other.categoryBits;
        this.maskBits = other.maskBits;
        this.groupIndex = other.groupIndex ?? 0;
        return this;
    }
}
b2Filter.DEFAULT = new b2Filter();
/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
export class b2FixtureDef {
    constructor() {
        /// The shape, this must be set. The shape will be cloned, so you
        /// can create the shape on the stack.
        this.shape = null;
        /// Use this to store application specific fixture data.
        this.userData = null;
        /// The friction coefficient, usually in the range [0,1].
        this.friction = NaN;
        /// The restitution (elasticity) usually in the range [0,1].
        this.restitution = NaN;
        /// The density, usually in kg/m^2.
        this.density = NaN;
        /// A sensor shape collects contact information but never generates a collision
        /// response.
        this.isSensor = false;
        /// Contact filtering data.
        this.filter = new b2Filter();
        this.friction = 0.2;
        this.restitution = 0.0;
        this.density = 0.0;
    }
}
/// This proxy is used internally to connect fixtures to the broad-phase.
export class b2FixtureProxy {
    constructor(fixture, childIndex) {
        this.aabb = new b2AABB();
        this.childIndex = 0;
        this.fixture = fixture;
        this.childIndex = childIndex;
        this.fixture.m_shape.ComputeAABB(this.aabb, this.fixture.m_body.GetTransform(), childIndex);
        this.treeNode = this.fixture.m_body.m_world.m_contactManager.m_broadPhase.CreateProxy(this.aabb, this);
    }
    Reset() {
        this.fixture.m_body.m_world.m_contactManager.m_broadPhase.DestroyProxy(this.treeNode);
    }
    Touch() {
        this.fixture.m_body.m_world.m_contactManager.m_broadPhase.TouchProxy(this.treeNode);
    }
    Synchronize(transform1, transform2, displacement) {
        if (transform1 === transform2) {
            this.fixture.m_shape.ComputeAABB(this.aabb, transform1, this.childIndex);
            this.fixture.m_body.m_world.m_contactManager.m_broadPhase.MoveProxy(this.treeNode, this.aabb, displacement);
        }
        else {
            // Compute an AABB that covers the swept shape (may miss some rotation effect).
            const aabb1 = b2FixtureProxy.Synchronize_s_aabb1;
            const aabb2 = b2FixtureProxy.Synchronize_s_aabb2;
            this.fixture.m_shape.ComputeAABB(aabb1, transform1, this.childIndex);
            this.fixture.m_shape.ComputeAABB(aabb2, transform2, this.childIndex);
            this.aabb.Combine2(aabb1, aabb2);
            this.fixture.m_body.m_world.m_contactManager.m_broadPhase.MoveProxy(this.treeNode, this.aabb, displacement);
        }
    }
}
b2FixtureProxy.Synchronize_s_aabb1 = new b2AABB();
b2FixtureProxy.Synchronize_s_aabb2 = new b2AABB();
/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
export class b2Fixture {
    constructor(body, def) {
        this.m_density = NaN;
        this.m_friction = NaN;
        this.m_restitution = NaN;
        this.m_next = null;
        this._shapeRadius = NaN;
        this.m_proxies = [];
        this.m_filter = new b2Filter();
        this.m_isSensor = false;
        this.m_userData = null;
        this.m_density = def.density ?? 0.0;
        this.m_friction = def.friction ?? 0.2;
        this.m_restitution = def.restitution ?? 0.0;
        this.m_body = body;
        this.m_shape = def.shape.Clone();
        this._shapeType = def.shape.m_type;
        // TODO: need to  sync radius if shape is changed by user!
        this._shapeRadius = def.shape.m_radius;
        this.m_userData = def.userData ?? null;
        this.m_filter.Copy(def.filter ?? b2Filter.DEFAULT);
        this.m_isSensor = def.isSensor ?? false;
    }
    get m_proxyCount() {
        return this.m_proxies.length;
    }
    Reset() {
        // The proxies must be destroyed before calling this.
        !!B2_DEBUG && b2Assert(this.m_proxyCount === 0);
    }
    /// Get the type of the child shape. You can use this to down cast to the concrete shape.
    /// @return the shape type.
    GetType() {
        return this._shapeType;
    }
    /// Get the child shape. You can modify the child shape, however you should not change the
    /// number of vertices because this will crash some collision caching mechanisms.
    /// Manipulating the shape may lead to non-physical behavior.
    GetShape() {
        return this.m_shape;
    }
    /// Set if this fixture is a sensor.
    SetSensor(sensor) {
        if (sensor !== this.m_isSensor) {
            this.m_body.SetAwake(true);
            this.m_isSensor = sensor;
        }
    }
    /// Is this fixture a sensor (non-solid)?
    /// @return the true if the shape is a sensor.
    IsSensor() {
        return this.m_isSensor;
    }
    /// Set the contact filtering data. This will not update contacts until the next time
    /// step when either parent body is active and awake.
    /// This automatically calls Refilter.
    SetFilterData(filter) {
        this.m_filter.Copy(filter);
        this.Refilter();
    }
    /// Get the contact filtering data.
    GetFilterData() {
        return this.m_filter;
    }
    /// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
    Refilter() {
        // Flag associated contacts for filtering.
        let edge = this.m_body.GetContactList();
        while (edge) {
            const contact = edge.contact;
            const fixtureA = contact.GetFixtureA();
            const fixtureB = contact.GetFixtureB();
            if (fixtureA === this || fixtureB === this) {
                contact.FlagForFiltering();
            }
            edge = edge.next;
        }
        // Touch each proxy so that new pairs may be created
        this.TouchProxies();
    }
    /// Get the parent body of this fixture. This is NULL if the fixture is not attached.
    /// @return the parent body.
    GetBody() {
        return this.m_body;
    }
    /// Get the next fixture in the parent body's fixture list.
    /// @return the next shape.
    GetNext() {
        return this.m_next;
    }
    /// Get the user data that was assigned in the fixture definition. Use this to
    /// store your application specific data.
    GetUserData() {
        return this.m_userData;
    }
    /// Set the user data. Use this to store your application specific data.
    SetUserData(data) {
        this.m_userData = data;
    }
    /// Test a point for containment in this fixture.
    /// @param p a point in world coordinates.
    TestPoint(p) {
        return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
    }
    ComputeDistance(p, normal, childIndex) {
        if (B2_ENABLE_PARTICLE) {
            return this.m_shape.ComputeDistance(this.m_body.GetTransform(), p, normal, childIndex);
        }
        else {
            return 0.0;
        }
    }
    /// Cast a ray against this shape.
    /// @param output the ray-cast results.
    /// @param input the ray-cast input parameters.
    RayCast(output, input, childIndex) {
        return this.m_shape.RayCast(output, input, this.m_body.GetTransform(), childIndex);
    }
    /// Get the mass data for this fixture. The mass data is based on the density and
    /// the shape. The rotational inertia is about the shape's origin. This operation
    /// may be expensive.
    GetMassData(massData = new b2MassData()) {
        this.m_shape.ComputeMass(massData, this.m_density);
        return massData;
    }
    /// Set the density of this fixture. This will _not_ automatically adjust the mass
    /// of the body. You must call b2Body::ResetMassData to update the body's mass.
    SetDensity(density) {
        this.m_density = density;
    }
    /// Get the density of this fixture.
    GetDensity() {
        return this.m_density;
    }
    /// Get the coefficient of friction.
    GetFriction() {
        return this.m_friction;
    }
    /// Set the coefficient of friction. This will _not_ change the friction of
    /// existing contacts.
    SetFriction(friction) {
        this.m_friction = friction;
    }
    /// Get the coefficient of restitution.
    GetRestitution() {
        return this.m_restitution;
    }
    /// Set the coefficient of restitution. This will _not_ change the restitution of
    /// existing contacts.
    SetRestitution(restitution) {
        this.m_restitution = restitution;
    }
    /// Get the fixture's AABB. This AABB may be enlarge and/or stale.
    /// If you need a more accurate AABB, compute it using the shape and
    /// the body transform.
    GetAABB(childIndex) {
        !!B2_DEBUG && b2Assert(0 <= childIndex && childIndex < this.m_proxyCount);
        return this.m_proxies[childIndex].aabb;
    }
    // These support body activation/deactivation.
    CreateProxies() {
        if (this.m_proxies.length !== 0) {
            throw new Error();
        }
        // Create proxies in the broad-phase.
        for (let i = 0; i < this.m_shape.GetChildCount(); ++i) {
            this.m_proxies[i] = new b2FixtureProxy(this, i);
        }
    }
    DestroyProxies() {
        // Destroy proxies in the broad-phase.
        for (let i = 0; i < this.m_proxies.length; ++i) {
            this.m_proxies[i].Reset();
        }
        this.m_proxies.length = 0;
    }
    TouchProxies() {
        for (let i = 0; i < this.m_proxies.length; ++i) {
            this.m_proxies[i].Touch();
        }
    }
    SynchronizeProxies(transform1, transform2, displacement) {
        for (let i = 0; i < this.m_proxies.length; ++i) {
            this.m_proxies[i].Synchronize(transform1, transform2, displacement);
        }
    }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJGaXh0dXJlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL2R5bmFtaWNzL2IyRml4dHVyZS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRztBQUVILE9BQU8sRUFBRSxRQUFRLEVBQUUsTUFBTSxzQkFBc0IsQ0FBQztBQUVoRCxPQUFPLEVBQUUsTUFBTSxFQUFtQyxNQUFNLDBCQUEwQixDQUFDO0FBRW5GLE9BQU8sRUFBRSxVQUFVLEVBQXdCLE1BQU0sNkJBQTZCLENBQUM7QUFrQi9FLHNDQUFzQztBQUN0QyxNQUFNLE9BQU8sUUFBUTtJQUFyQjtRQUdFLHFFQUFxRTtRQUNyRSxpQkFBWSxHQUFHLE1BQU0sQ0FBQztRQUV0QixpRUFBaUU7UUFDakUscUNBQXFDO1FBQ3JDLGFBQVEsR0FBRyxNQUFNLENBQUM7UUFFbEIsaUZBQWlGO1FBQ2pGLCtFQUErRTtRQUMvRSxnREFBZ0Q7UUFDaEQsZUFBVSxHQUFHLENBQUMsQ0FBQztJQWFqQixDQUFDO0lBWEMsS0FBSztRQUNILE9BQU8sSUFBSSxRQUFRLEVBQUUsQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7SUFDbkMsQ0FBQztJQUVELElBQUksQ0FBQyxLQUFnQjtRQUNuQixDQUFDLENBQUMsUUFBUSxJQUFJLFFBQVEsQ0FBQyxJQUFJLEtBQUssS0FBSyxDQUFDLENBQUM7UUFDdkMsSUFBSSxDQUFDLFlBQVksR0FBRyxLQUFLLENBQUMsWUFBWSxDQUFDO1FBQ3ZDLElBQUksQ0FBQyxRQUFRLEdBQUcsS0FBSyxDQUFDLFFBQVEsQ0FBQztRQUMvQixJQUFJLENBQUMsVUFBVSxHQUFHLEtBQUssQ0FBQyxVQUFVLElBQUksQ0FBQyxDQUFDO1FBQ3hDLE9BQU8sSUFBSSxDQUFDO0lBQ2QsQ0FBQzs7QUF4QmUsZ0JBQU8sR0FBdUIsSUFBSSxRQUFRLEVBQUUsQ0FBQztBQXNEL0QsMkVBQTJFO0FBQzNFLDBFQUEwRTtBQUMxRSxNQUFN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QUNILENBQUM7SUFFRCxjQUFjO1FBQ1osc0NBQXNDO1FBQ3RDLEtBQUssSUFBSSxDQUFDLEdBQUcsQ0FBQyxFQUFFLENBQUMsR0FBRyxJQUFJLENBQUMsU0FBUyxDQUFDLE1BQU0sRUFBRSxFQUFFLENBQUMsRUFBRTtZQUM5QyxJQUFJLENBQUMsU0FBUyxDQUFDLENBQUMsQ0FBQyxDQUFDLEtBQUssRUFBRSxDQUFDO1NBQzNCO1FBQ0QsSUFBSSxDQUFDLFNBQVMsQ0FBQyxNQUFNLEdBQUcsQ0FBQyxDQUFDO0lBQzVCLENBQUM7SUFFRCxZQUFZO1FBQ1YsS0FBSyxJQUFJLENBQUMsR0FBRyxDQUFDLEVBQUUsQ0FBQyxHQUFHLElBQUksQ0FBQyxTQUFTLENBQUMsTUFBTSxFQUFFLEVBQUUsQ0FBQyxFQUFFO1lBQzlDLElBQUksQ0FBQyxTQUFTLENBQUMsQ0FBQyxDQUFDLENBQUMsS0FBSyxFQUFFLENBQUM7U0FDM0I7SUFDSCxDQUFDO0lBRUQsa0JBQWtCLENBQUMsVUFBdUIsRUFBRSxVQUF1QixFQUFFLFlBQW9CO1FBQ3ZGLEtBQUssSUFBSSxDQUFDLEdBQUcsQ0FBQyxFQUFFLENBQUMsR0FBRyxJQUFJLENBQUMsU0FBUyxDQUFDLE1BQU0sRUFBRSxFQUFFLENBQUMsRUFBRTtZQUM5QyxJQUFJLENBQUMsU0FBUyxDQUFDLENBQUMsQ0FBQyxDQUFDLFdBQVcsQ0FBQyxVQUFVLEVBQUUsVUFBVSxFQUFFLFlBQVksQ0FBQyxDQUFDO1NBQ3JFO0lBQ0gsQ0FBQztDQUNGIiwic291cmNlc0NvbnRlbnQiOlsiLypcclxuICogQ29weXJpZ2h0IChjKSAyMDA2LTIwMDkgRXJpbiBDYXR0byBodHRwOi8vd3d3LmJveDJkLm9yZ1xyXG4gKlxyXG4gKiBUaGlzIHNvZnR3YXJlIGlzIHByb3ZpZGVkICdhcy1pcycsIHdpdGhvdXQgYW55IGV4cHJlc3Mgb3IgaW1wbGllZFxyXG4gKiB3YXJyYW50eS4gIEluIG5vIGV2ZW50IHdpbGwgdGhlIGF1dGhvcnMgYmUgaGVsZCBsaWFibGUgZm9yIGFueSBkYW1hZ2VzXHJcbiAqIGFyaXNpbmcgZnJvbSB0aGUgdXNlIG9mIHRoaXMgc29mdHdhcmUuXHJcbiAqIFBlcm1pc3Npb24gaXMgZ3JhbnRlZCB0byBhbnlvbmUgdG8gdXNlIHRoaXMgc29mdHdhcmUgZm9yIGFueSBwdXJwb3NlLFxyXG4gKiBpbmNsdWRpbmcgY29tbWVyY2lhbCBhcHBsaWNhdGlvbnMsIGFuZCB0byBhbHRlciBpdCBhbmQgcmVkaXN0cmlidXRlIGl0XHJcbiAqIGZyZWVseSwgc3ViamVjdCB0byB0aGUgZm9sbG93aW5nIHJlc3RyaWN0aW9uczpcclxuICogMS4gVGhlIG9yaWdpbiBvZiB0aGlzIHNvZnR3YXJlIG11c3Qgbm90IGJlIG1pc3JlcHJlc2VudGVkOyB5b3UgbXVzdCBub3RcclxuICogY2xhaW0gdGhhdCB5b3Ugd3JvdGUgdGhlIG9yaWdpbmFsIHNvZnR3YXJlLiBJZiB5b3UgdXNlIHRoaXMgc29mdHdhcmVcclxuICogaW4gYSBwcm9kdWN0LCBhbiBhY2tub3dsZWRnbWVudCBpbiB0aGUgcHJvZHVjdCBkb2N1bWVudGF0aW9uIHdvdWxkIGJlXHJcbiAqIGFwcHJlY2lhdGVkIGJ1dCBpcyBub3QgcmVxdWlyZWQuXHJcbiAqIDIuIEFsdGVyZWQgc291cmNlIHZlcnNpb25zIG11c3QgYmUgcGxhaW5seSBtYXJrZWQgYXMgc3VjaCwgYW5kIG11c3Qgbm90IGJlXHJcbiAqIG1pc3JlcHJlc2VudGVkIGFzIGJlaW5nIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS5cclxuICogMy4gVGhpcyBub3RpY2UgbWF5IG5vdCBiZSByZW1vdmVkIG9yIGFsdGVyZWQgZnJvbSBhbnkgc291cmNlIGRpc3RyaWJ1dGlvbi5cclxuICovXHJcblxyXG5pbXBvcnQgeyBiMkFzc2VydCB9IGZyb20gJy4uL2NvbW1vbi9iMlNldHRpbmdzJztcclxuaW1wb3J0IHsgYjJUcmFuc2Zvcm0sIGIyVmVjMiwgWFkgfSBmcm9tICcuLi9jb21tb24vYjJNYXRoJztcclxuaW1wb3J0IHsgYjJBQUJCLCBiMlJheUNhc3RJbnB1dCwgYjJSYXlDYXN0T3V0cHV0IH0gZnJvbSAnLi4vY29sbGlzaW9uL2IyQ29sbGlzaW9uJztcclxuaW1wb3J0IHsgYjJUcmVlTm9kZSB9IGZyb20gJy4uL2NvbGxpc2lvbi9iMkR5bmFtaWNUcmVlJztcclxuaW1wb3J0IHsgYjJNYXNzRGF0YSwgYjJTaGFwZSwgYjJTaGFwZVR5cGUgfSBmcm9tICcuLi9jb2xsaXNpb24vc2hhcGVzL2IyU2hhcGUnO1xyXG5pbXBvcnQgeyBiMkJvZHkgfSBmcm9tICcuL2IyQm9keSc7XHJcblxyXG4vLy8gVGhpcyBob2xkcyBjb250YWN0IGZpbHRlcmluZyBkYXRhLlxyXG5leHBvcnQgaW50ZXJmYWNlIGIySUZpbHRlciB7XHJcbiAgLy8vIFRoZSBjb2xsaXNpb24gY2F0ZWdvcnkgYml0cy4gTm9ybWFsbHkgeW91IHdvdWxkIGp1c3Qgc2V0IG9uZSBiaXQuXHJcbiAgY2F0ZWdvcnlCaXRzOiBudW1iZXI7XHJcblxyXG4gIC8vLyBUaGUgY29sbGlzaW9uIG1hc2sgYml0cy4gVGhpcyBzdGF0ZXMgdGhlIGNhdGVnb3JpZXMgdGhhdCB0aGlzXHJcbiAgLy8vIHNoYXBlIHdvdWxkIGFjY2VwdCBmb3IgY29sbGlzaW9uLlxyXG4gIG1hc2tCaXRzOiBudW1iZXI7XHJcblxyXG4gIC8vLyBDb2xsaXNpb24gZ3JvdXBzIGFsbG93IGEgY2VydGFpbiBncm91cCBvZiBvYmplY3RzIHRvIG5ldmVyIGNvbGxpZGUgKG5lZ2F0aXZlKVxyXG4gIC8vLyBvciBhbHdheXMgY29sbGlkZSAocG9zaXRpdmUpLiBaZXJvIG1lYW5zIG5vIGNvbGxpc2lvbiBncm91cC4gTm9uLXplcm8gZ3JvdXBcclxuICAvLy8gZmlsdGVyaW5nIGFsd2F5cyB3aW5zIGFnYWluc3QgdGhlIG1hc2sgYml0cy5cclxuICBncm91cEluZGV4PzogbnVtYmVyO1xyXG59XHJcblxyXG4vLy8gVGhpcyBob2xkcyBjb250YWN0IGZpbHRlcmluZyBkYXRhLlxyXG5leHBvcnQgY2xhc3MgYjJGaWx0ZXIgaW1wbGVtZW50cyBiMklGaWx0ZXIge1xyXG4gIHN0YXRpYyByZWFkb25seSBERUZBVUxUOiBSZWFkb25seTxiMkZpbHRlcj4gPSBuZXcgYjJGaWx0ZXIoKTtcclxuXHJcbiAgLy8vIFRoZSBjb2xsaXNpb24gY2F0ZWdvcnkgYml0cy4gTm9ybWFsbHkgeW91IHdvdWxkIGp1c3Qgc2V0IG9uZSBiaXQuXHJcbiAgY2F0ZWdvcnlCaXRzID0gMHgwMDAxO1xyXG5cclxuICAvLy8gVGhlIGNvbGxpc2lvbiBtYXNrIGJpdHMuIFRoaXMgc3RhdGVzIHRoZSBjYXRlZ29yaWVzIHRoYXQgdGhpc1xyXG4gIC8vLyBzaGFwZSB3b3VsZCBhY2NlcHQgZm9yIGNvbGxpc2lvbi5cclxuICBtYXNrQml0cyA9IDB4ZmZmZjtcclxuXHJcbiAgLy8vIENvbGxpc2lvbiBncm91cHMgYWxsb3cgYSBjZXJ0YWluIGdyb3VwIG9mIG9iamVjdHMgdG8gbmV2ZXIgY29sbGlkZSAobmVnYXRpdmUpXHJcbiAgLy8vIG9yIGFsd2F5cyBjb2xsaWRlIChwb3NpdGl2ZSkuIFplcm8gbWVhbnMgbm8gY29sbGlzaW9uIGdyb3VwLiBOb24temVybyBncm91cFxyXG4gIC8vLyBmaWx0ZXJpbmcgYWx3YXlzIHdpbnMgYWdhaW5zdCB0aGUgbWFzayBiaXRzLlxyXG4gIGdyb3VwSW5kZXggPSAwO1xyXG5cclxuICBDbG9uZSgpOiBiMkZpbHRlciB7XHJcbiAgICByZXR1cm4gbmV3IGIyRmlsdGVyKCkuQ29weSh0aGlzKTtcclxuICB9XHJcblxyXG4gIENvcHkob3RoZXI6IGIySUZpbHRlcik6IHRoaXMge1xyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCh0aGlzICE9PSBvdGhlcik7XHJcbiAgICB0aGlzLmNhdGVnb3J5Qml0cyA9IG90aGVyLmNhdGVnb3J5Qml0cztcclxuICAgIHRoaXMubWFza0JpdHMgPSBvdGhlci5tYXNrQml0cztcclxuICAgIHRoaXMuZ3JvdXBJbmRleCA9IG90aGVyLmdyb3VwSW5kZXggPz8gMDtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxufVxyXG5cclxuLy8vIEEgZml4dHVyZSBkZWZpbml0aW9uIGlzIHVzZWQgdG8gY3JlYXRlIGEgZml4dHVyZS4gVGhpcyBjbGFzcyBkZWZpbmVzIGFuXHJcbi8vLyBhYnN0cmFjdCBmaXh0dXJlIGRlZmluaXRpb24uIFlvdSBjYW4gcmV1c2UgZml4dHVyZSBkZWZpbml0aW9ucyBzYWZlbHkuXHJcbmV4cG9ydCBpbnRlcmZhY2UgYjJJRml4dHVyZURlZiB7XHJcbiAgLy8vIFRoZSBzaGFwZSwgdGhpcyBtdXN0IGJlIHNldC4gVGhlIHNoYXBlIHdpbGwgYmUgY2xvbmVkLCBzbyB5b3VcclxuICAvLy8gY2FuIGNyZWF0ZSB0aGUgc2hhcGUgb24gdGhlIHN0YWNrLlxyXG4gIHNoYXBlOiBiMlNoYXBlO1xyXG5cclxuICAvLy8gVXNlIHRoaXMgdG8gc3RvcmUgYXBwbGljYXRpb24gc3BlY2lmaWMgZml4dHVyZSBkYXRhLlxyXG4gIHVzZXJEYXRhPzogYW55O1xyXG5cclxuICAvLy8gVGhlIGZyaWN0aW9uIGNvZWZmaWNpZW50LCB1c3VhbGx5IGluIHRoZSByYW5nZSBbMCwxXS5cclxuICBmcmljdGlvbj86IG51bWJlcjtcclxuXHJcbiAgLy8vIFRoZSByZXN0aXR1dGlvbiAoZWxhc3RpY2l0eSkgdXN1YWxseSBpbiB0aGUgcmFuZ2UgWzAsMV0uXHJcbiAgcmVzdGl0dXRpb24/OiBudW1iZXI7XHJcblxyXG4gIC8vLyBUaGUgZGVuc2l0eSwgdXN1YWxseSBpbiBrZy9tXjIuXHJcbiAgZGVuc2l0eT86IG51bWJlcjtcclxuXHJcbiAgLy8vIEEgc2Vuc29yIHNoYXBlIGNvbGxlY3RzIGNvbnRhY3QgaW5mb3JtYXRpb24gYnV0IG5ldmVyIGdlbmVyYXRlcyBhIGNvbGxpc2lvblxyXG4gIC8vLyByZXNwb25zZS5cclxuICBpc1NlbnNvcj86IGJvb2xlYW47XHJcblxyXG4gIC8vLyBDb250YWN0IGZpbHRlcmluZyBkYXRhLlxyXG4gIGZpbHRlcj86IGIySUZpbHRlcjtcclxufVxyXG5cclxuLy8vIEEgZml4dHVyZSBkZWZpbml0aW9uIGlzIHVzZWQgdG8gY3JlYXRlIGEgZml4dHVyZS4gVGhpcyBjbGFzcyBkZWZpbmVzIGFuXHJcbi8vLyBhYnN0cmFjdCBmaXh0dXJlIGRlZmluaXRpb24uIFlvdSBjYW4gcmV1c2UgZml4dHVyZSBkZWZpbml0aW9ucyBzYWZlbHkuXHJcbmV4cG9ydCBjbGFzcyBiMkZpeHR1cmVEZWYgaW1wbGVtZW50cyBiMklGaXh0dXJlRGVmIHtcclxuICAvLy8gVGhlIHNoYXBlLCB0aGlzIG11c3QgYmUgc2V0LiBUaGUgc2hhcGUgd2lsbCBiZSBjbG9uZWQsIHNvIHlvdVxyXG4gIC8vLyBjYW4gY3JlYXRlIHRoZSBzaGFwZSBvbiB0aGUgc3RhY2suXHJcbiAgc2hhcGU6IGIyU2hhcGUgPSAobnVsbCBhcyB1bmtub3duKSBhcyBiMlNoYXBlO1xyXG5cclxuICAvLy8gVXNlIHRoaXMgdG8gc3RvcmUgYXBwbGljYXRpb24gc3BlY2lmaWMgZml4dHVyZSBkYXRhLlxyXG4gIHVzZXJEYXRhOiBhbnkgPSBudWxsO1xyXG5cclxuICAvLy8gVGhlIGZyaWN0aW9uIGNvZWZmaWNpZW50LCB1c3VhbGx5IGluIHRoZSByYW5nZSBbMCwxXS5cclxuICBmcmljdGlvbiA9IE5hTjtcclxuXHJcbiAgLy8vIFRoZSByZXN0aXR1dGlvbiAoZWxhc3RpY2l0eSkgdXN1YWxseSBpbiB0aGUgcmFuZ2UgWzAsMV0uXHJcbiAgcmVzdGl0dXRpb24gPSBOYU47XHJcblxyXG4gIC8vLyBUaGUgZGVuc2l0eSwgdXN1YWxseSBpbiBrZy9tXjIuXHJcbiAgZGVuc2l0eSA9IE5hTjtcclxuXHJcbiAgLy8vIEEgc2Vuc29yIHNoYXBlIGNvbGxlY3RzIGNvbnRhY3QgaW5mb3JtYXRpb24gYnV0IG5ldmVyIGdlbmVyYXRlcyBhIGNvbGxpc2lvblxyXG4gIC8vLyByZXNwb25zZS5cclxuICBpc1NlbnNvciA9IGZhbHNlO1xyXG5cclxuICAvLy8gQ29udGFjdCBmaWx0ZXJpbmcgZGF0YS5cclxuICByZWFkb25seSBmaWx0ZXIgPSBuZXcgYjJGaWx0ZXIoKTtcclxuXHJcbiAgY29uc3RydWN0b3IoKSB7XHJcbiAgICB0aGlzLmZyaWN0aW9uID0gMC4yO1xyXG4gICAgdGhpcy5yZXN0aXR1dGlvbiA9IDAuMDtcclxuICAgIHRoaXMuZGVuc2l0eSA9IDAuMDtcclxuICB9XHJcbn1cclxuXHJcbi8vLyBUaGlzIHByb3h5IGlzIHVzZWQgaW50ZXJuYWxseSB0byBjb25uZWN0IGZpeHR1cmVzIHRvIHRoZSBicm9hZC1waGFzZS5cclxuZXhwb3J0IGNsYXNzIGIyRml4dHVyZVByb3h5IHtcclxuICByZWFkb25seSBhYWJiID0gbmV3IGIyQUFCQigpO1xyXG4gIHJlYWRvbmx5IGZpeHR1cmU6IGIyRml4dHVyZTtcclxuICByZWFkb25seSBjaGlsZEluZGV4OiBudW1iZXIgPSAwO1xyXG4gIHRyZWVOb2RlOiBiMlRyZWVOb2RlPGIyRml4dHVyZVByb3h5PjtcclxuXHJcbiAgY29uc3RydWN0b3IoZml4dHVyZTogYjJGaXh0dXJlLCBjaGlsZEluZGV4OiBudW1iZXIpIHtcclxuICAgIHRoaXMuZml4dHVyZSA9IGZpeHR1cmU7XHJcbiAgICB0aGlzLmNoaWxkSW5kZXggPSBjaGlsZEluZGV4O1xyXG4gICAgdGhpcy5maXh0dXJlLm1fc2hhcGUuQ29tcHV0ZUFBQkIodGhpcy5hYWJiLCB0aGlzLmZpeHR1cmUubV9ib2R5LkdldFRyYW5zZm9ybSgpLCBjaGlsZEluZGV4KTtcclxuICAgIHRoaXMudHJlZU5vZGUgPSB0aGlzLmZpeHR1cmUubV9ib2R5Lm1fd29ybGQubV9jb250YWN0TWFuYWdlci5tX2Jyb2FkUGhhc2UuQ3JlYXRlUHJveHkoXHJcbiAgICAgIHRoaXMuYWFiYixcclxuICAgICAgdGhpcyxcclxuICAgICk7XHJcbiAgfVxyXG5cclxuICBSZXNldCgpOiB2b2lkIHtcclxuICAgIHRoaXMuZml4dHVyZS5tX2JvZHkubV93b3JsZC5tX2NvbnRhY3RNYW5hZ2VyLm1fYnJvYWRQaGFzZS5EZXN0cm95UHJveHkodGhpcy50cmVlTm9kZSk7XHJcbiAgfVxyXG5cclxuICBUb3VjaCgpOiB2b2lkIHtcclxuICAgIHRoaXMuZml4dHVyZS5tX2JvZHkubV93b3JsZC5tX2NvbnRhY3RNYW5hZ2VyLm1fYnJvYWRQaGFzZS5Ub3VjaFByb3h5KHRoaXMudHJlZU5vZGUpO1xyXG4gIH1cclxuXHJcbiAgcHJpdmF0ZSBzdGF0aWMgU3luY2hyb25pemVfc19hYWJiMSA9IG5ldyBiMkFBQkIoKTtcclxuICBwcml2YXRlIHN0YXRpYyBTeW5jaHJvbml6ZV9zX2FhYmIyID0gbmV3IGIyQUFCQigpO1xyXG5cclxuICBTeW5jaHJvbml6ZSh0cmFuc2Zvcm0xOiBiMlRyYW5zZm9ybSwgdHJhbnNmb3JtMjogYjJUcmFuc2Zvcm0sIGRpc3BsYWNlbWVudDogYjJWZWMyKTogdm9pZCB7XHJcbiAgICBpZiAodHJhbnNmb3JtMSA9PT0gdHJhbnNmb3JtMikge1xyXG4gICAgICB0aGlzLmZpeHR1cmUubV9zaGFwZS5Db21wdXRlQUFCQih0aGlzLmFhYmIsIHRyYW5zZm9ybTEsIHRoaXMuY2hpbGRJbmRleCk7XHJcbiAgICAgIHRoaXMuZml4dHVyZS5tX2JvZHkubV93b3JsZC5tX2NvbnRhY3RNYW5hZ2VyLm1fYnJvYWRQaGFzZS5Nb3ZlUHJveHkoXHJcbiAgICAgICAgdGhpcy50cmVlTm9kZSxcclxuICAgICAgICB0aGlzLmFhYmIsXHJcbiAgICAgICAgZGlzcGxhY2VtZW50LFxyXG4gICAgICApO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgLy8gQ29tcHV0ZSBhbiBBQUJCIHRoYXQgY292ZXJzIHRoZSBzd2VwdCBzaGFwZSAobWF5IG1pc3Mgc29tZSByb3RhdGlvbiBlZmZlY3QpLlxyXG4gICAgICBjb25zdCBhYWJiMTogYjJBQUJCID0gYjJGaXh0dXJlUHJveHkuU3luY2hyb25pemVfc19hYWJiMTtcclxuICAgICAgY29uc3QgYWFiYjI6IGIyQUFCQiA9IGIyRml4dHVyZVByb3h5LlN5bmNocm9uaXplX3NfYWFiYjI7XHJcbiAgICAgIHRoaXMuZml4dHVyZS5tX3NoYXBlLkNvbXB1dGVBQUJCKGFhYmIxLCB0cmFuc2Zvcm0xLCB0aGlzLmNoaWxkSW5kZXgpO1xyXG4gICAgICB0aGlzLmZpeHR1cmUubV9zaGFwZS5Db21wdXRlQUFCQihhYWJiMiwgdHJhbnNmb3JtMiwgdGhpcy5jaGlsZEluZGV4KTtcclxuICAgICAgdGhpcy5hYWJiLkNvbWJpbmUyKGFhYmIxLCBhYWJiMik7XHJcbiAgICAgIHRoaXMuZml4dHVyZS5tX2JvZHkubV93b3JsZC5tX2NvbnRhY3RNYW5hZ2VyLm1fYnJvYWRQaGFzZS5Nb3ZlUHJveHkoXHJcbiAgICAgICAgdGhpcy50cmVlTm9kZSxcclxuICAgICAgICB0aGlzLmFhYmIsXHJcbiAgICAgICAgZGlzcGxhY2VtZW50LFxyXG4gICAgICApO1xyXG4gICAgfVxyXG4gIH1cclxufVxyXG5cclxuLy8vIEEgZml4dHVyZSBpcyB1c2VkIHRvIGF0dGFjaCBhIHNoYXBlIHRvIGEgYm9keSBmb3IgY29sbGlzaW9uIGRldGVjdGlvbi4gQSBmaXh0dXJlXHJcbi8vLyBpbmhlcml0cyBpdHMgdHJhbnNmb3JtIGZyb20gaXRzIHBhcmVudC4gRml4dHVyZXMgaG9sZCBhZGRpdGlvbmFsIG5vbi1nZW9tZXRyaWMgZGF0YVxyXG4vLy8gc3VjaCBhcyBmcmljdGlvbiwgY29sbGlzaW9uIGZpbHRlcnMsIGV0Yy5cclxuLy8vIEZpeHR1cmVzIGFyZSBjcmVhdGVkIHZpYSBiMkJvZHk6OkNyZWF0ZUZpeHR1cmUuXHJcbi8vLyBAd2FybmluZyB5b3UgY2Fubm90IHJldXNlIGZpeHR1cmVzLlxyXG5leHBvcnQgY2xhc3MgYjJGaXh0dXJlIHtcclxuICBtX2RlbnNpdHkgPSBOYU47XHJcbiAgbV9mcmljdGlvbiA9IE5hTjtcclxuICBtX3Jlc3RpdHV0aW9uID0gTmFOO1xyXG5cclxuICBtX25leHQ6IGIyRml4dHVyZSB8IG51bGwgPSBudWxsO1xyXG4gIHJlYWRvbmx5IG1fYm9keTogYjJCb2R5O1xyXG4gIHJlYWRvbmx5IG1fc2hhcGU6IGIyU2hhcGU7XHJcbiAgX3NoYXBlVHlwZTogYjJTaGFwZVR5cGU7XHJcbiAgX3NoYXBlUmFkaXVzID0gTmFOO1xyXG5cclxuICByZWFkb25seSBtX3Byb3hpZXM6IGIyRml4dHVyZVByb3h5W10gPSBbXTtcclxuXHJcbiAgZ2V0IG1fcHJveHlDb3VudCgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9wcm94aWVzLmxlbmd0aDtcclxuICB9XHJcblxyXG4gIHJlYWRvbmx5IG1fZmlsdGVyOiBiMkZpbHRlciA9IG5ldyBiMkZpbHRlcigpO1xyXG5cclxuICBtX2lzU2Vuc29yID0gZmFsc2U7XHJcblxyXG4gIG1fdXNlckRhdGE6IGFueSA9IG51bGw7XHJcblxyXG4gIGNvbnN0cnVjdG9yKGJvZHk6IGIyQm9keSwgZGVmOiBiMklGaXh0dXJlRGVmKSB7XHJcbiAgICB0aGlzLm1fZGVuc2l0eSA9IGRlZi5kZW5zaXR5ID8/IDAuMDtcclxuICAgIHRoaXMubV9mcmljdGlvbiA9IGRlZi5mcmljdGlvbiA/PyAwLjI7XHJcbiAgICB0aGlzLm1fcmVzdGl0dXRpb24gPSBkZWYucmVzdGl0dXRpb24gPz8gMC4wO1xyXG4gICAgdGhpcy5tX2JvZHkgPSBib2R5O1xyXG4gICAgdGhpcy5tX3NoYXBlID0gZGVmLnNoYXBlLkNsb25lKCk7XHJcbiAgICB0aGlzLl9zaGFwZVR5cGUgPSBkZWYuc2hhcGUubV90eXBlO1xyXG4gICAgLy8gVE9ETzogbmVlZCB0byAgc3luYyByYWRpdXMgaWYgc2hhcGUgaXMgY2hhbmdlZCBieSB1c2VyIVxyXG4gICAgdGhpcy5fc2hhcGVSYWRpdXMgPSBkZWYuc2hhcGUubV9yYWRpdXM7XHJcbiAgICB0aGlzLm1fdXNlckRhdGEgPSBkZWYudXNlckRhdGEgPz8gbnVsbDtcclxuICAgIHRoaXMubV9maWx0ZXIuQ29weShkZWYuZmlsdGVyID8/IGIyRmlsdGVyLkRFRkFVTFQpO1xyXG4gICAgdGhpcy5tX2lzU2Vuc29yID0gZGVmLmlzU2Vuc29yID8/IGZhbHNlO1xyXG4gIH1cclxuXHJcbiAgUmVzZXQoKTogdm9pZCB7XHJcbiAgICAvLyBUaGUgcHJveGllcyBtdXN0IGJlIGRlc3Ryb3llZCBiZWZvcmUgY2FsbGluZyB0aGlzLlxyXG4gICAgISFCMl9ERUJVRyAmJiBiMkFzc2VydCh0aGlzLm1fcHJveHlDb3VudCA9PT0gMCk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSB0eXBlIG9mIHRoZSBjaGlsZCBzaGFwZS4gWW91IGNhbiB1c2UgdGhpcyB0byBkb3duIGNhc3QgdG8gdGhlIGNvbmNyZXRlIHNoYXBlLlxyXG4gIC8vLyBAcmV0dXJuIHRoZSBzaGFwZSB0eXBlLlxyXG4gIEdldFR5cGUoKTogYjJTaGFwZVR5cGUge1xyXG4gICAgcmV0dXJuIHRoaXMuX3NoYXBlVHlwZTtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGNoaWxkIHNoYXBlLiBZb3UgY2FuIG1vZGlmeSB0aGUgY2hpbGQgc2hhcGUsIGhvd2V2ZXIgeW91IHNob3VsZCBub3QgY2hhbmdlIHRoZVxyXG4gIC8vLyBudW1iZXIgb2YgdmVydGljZXMgYmVjYXVzZSB0aGlzIHdpbGwgY3Jhc2ggc29tZSBjb2xsaXNpb24gY2FjaGluZyBtZWNoYW5pc21zLlxyXG4gIC8vLyBNYW5pcHVsYXRpbmcgdGhlIHNoYXBlIG1heSBsZWFkIHRvIG5vbi1waHlzaWNhbCBiZWhhdmlvci5cclxuICBHZXRTaGFwZSgpOiBiMlNoYXBlIHtcclxuICAgIHJldHVybiB0aGlzLm1fc2hhcGU7XHJcbiAgfVxyXG5cclxuICAvLy8gU2V0IGlmIHRoaXMgZml4dHVyZSBpcyBhIHNlbnNvci5cclxuICBTZXRTZW5zb3Ioc2Vuc29yOiBib29sZWFuKTogdm9pZCB7XHJcbiAgICBpZiAoc2Vuc29yICE9PSB0aGlzLm1faXNTZW5zb3IpIHtcclxuICAgICAgdGhpcy5tX2JvZHkuU2V0QXdha2UodHJ1ZSk7XHJcbiAgICAgIHRoaXMubV9pc1NlbnNvciA9IHNlbnNvcjtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vLyBJcyB0aGlzIGZpeHR1cmUgYSBzZW5zb3IgKG5vbi1zb2xpZCk/XHJcbiAgLy8vIEByZXR1cm4gdGhlIHRydWUgaWYgdGhlIHNoYXBlIGlzIGEgc2Vuc29yLlxyXG4gIElzU2Vuc29yKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9pc1NlbnNvcjtcclxuICB9XHJcblxyXG4gIC8vLyBTZXQgdGhlIGNvbnRhY3QgZmlsdGVyaW5nIGRhdGEuIFRoaXMgd2lsbCBub3QgdXBkYXRlIGNvbnRhY3RzIHVudGlsIHRoZSBuZXh0IHRpbWVcclxuICAvLy8gc3RlcCB3aGVuIGVpdGhlciBwYXJlbnQgYm9keSBpcyBhY3RpdmUgYW5kIGF3YWtlLlxyXG4gIC8vLyBUaGlzIGF1dG9tYXRpY2FsbHkgY2FsbHMgUmVmaWx0ZXIuXHJcbiAgU2V0RmlsdGVyRGF0YShmaWx0ZXI6IGIyRmlsdGVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fZmlsdGVyLkNvcHkoZmlsdGVyKTtcclxuXHJcbiAgICB0aGlzLlJlZmlsdGVyKCk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBjb250YWN0IGZpbHRlcmluZyBkYXRhLlxyXG4gIEdldEZpbHRlckRhdGEoKTogUmVhZG9ubHk8YjJGaWx0ZXI+IHtcclxuICAgIHJldHVybiB0aGlzLm1fZmlsdGVyO1xyXG4gIH1cclxuXHJcbiAgLy8vIENhbGwgdGhpcyBpZiB5b3Ugd2FudCB0byBlc3RhYmxpc2ggY29sbGlzaW9uIHRoYXQgd2FzIHByZXZpb3VzbHkgZGlzYWJsZWQgYnkgYjJDb250YWN0RmlsdGVyOjpTaG91bGRDb2xsaWRlLlxyXG4gIFJlZmlsdGVyKCk6IHZvaWQge1xyXG4gICAgLy8gRmxhZyBhc3NvY2lhdGVkIGNvbnRhY3RzIGZvciBmaWx0ZXJpbmcuXHJcbiAgICBsZXQgZWRnZSA9IHRoaXMubV9ib2R5LkdldENvbnRhY3RMaXN0KCk7XHJcblxyXG4gICAgd2hpbGUgKGVkZ2UpIHtcclxuICAgICAgY29uc3QgY29udGFjdCA9IGVkZ2UuY29udGFjdDtcclxuICAgICAgY29uc3QgZml4dHVyZUEgPSBjb250YWN0LkdldEZpeHR1cmVBKCk7XHJcbiAgICAgIGNvbnN0IGZpeHR1cmVCID0gY29udGFjdC5HZXRGaXh0dXJlQigpO1xyXG4gICAgICBpZiAoZml4dHVyZUEgPT09IHRoaXMgfHwgZml4dHVyZUIgPT09IHRoaXMpIHtcclxuICAgICAgICBjb250YWN0LkZsYWdGb3JGaWx0ZXJpbmcoKTtcclxuICAgICAgfVxyXG5cclxuICAgICAgZWRnZSA9IGVkZ2UubmV4dDtcclxuICAgIH1cclxuXHJcbiAgICAvLyBUb3VjaCBlYWNoIHByb3h5IHNvIHRoYXQgbmV3IHBhaXJzIG1heSBiZSBjcmVhdGVkXHJcbiAgICB0aGlzLlRvdWNoUHJveGllcygpO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgcGFyZW50IGJvZHkgb2YgdGhpcyBmaXh0dXJlLiBUaGlzIGlzIE5VTEwgaWYgdGhlIGZpeHR1cmUgaXMgbm90IGF0dGFjaGVkLlxyXG4gIC8vLyBAcmV0dXJuIHRoZSBwYXJlbnQgYm9keS5cclxuICBHZXRCb2R5KCk6IGIyQm9keSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2JvZHk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBuZXh0IGZpeHR1cmUgaW4gdGhlIHBhcmVudCBib2R5J3MgZml4dHVyZSBsaXN0LlxyXG4gIC8vLyBAcmV0dXJuIHRoZSBuZXh0IHNoYXBlLlxyXG4gIEdldE5leHQoKTogYjJGaXh0dXJlIHwgbnVsbCB7XHJcbiAgICByZXR1cm4gdGhpcy5tX25leHQ7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSB1c2VyIGRhdGEgdGhhdCB3YXMgYXNzaWduZWQgaW4gdGhlIGZpeHR1cmUgZGVmaW5pdGlvbi4gVXNlIHRoaXMgdG9cclxuICAvLy8gc3RvcmUgeW91ciBhcHBsaWNhdGlvbiBzcGVjaWZpYyBkYXRhLlxyXG4gIEdldFVzZXJEYXRhKCk6IGFueSB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3VzZXJEYXRhO1xyXG4gIH1cclxuXHJcbiAgLy8vIFNldCB0aGUgdXNlciBkYXRhLiBVc2UgdGhpcyB0byBzdG9yZSB5b3VyIGFwcGxpY2F0aW9uIHNwZWNpZmljIGRhdGEuXHJcbiAgU2V0VXNlckRhdGEoZGF0YTogYW55KTogdm9pZCB7XHJcbiAgICB0aGlzLm1fdXNlckRhdGEgPSBkYXRhO1xyXG4gIH1cclxuXHJcbiAgLy8vIFRlc3QgYSBwb2ludCBmb3IgY29udGFpbm1lbnQgaW4gdGhpcyBmaXh0dXJlLlxyXG4gIC8vLyBAcGFyYW0gcCBhIHBvaW50IGluIHdvcmxkIGNvb3JkaW5hdGVzLlxyXG4gIFRlc3RQb2ludChwOiBYWSk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIHRoaXMubV9zaGFwZS5UZXN0UG9pbnQodGhpcy5tX2JvZHkuR2V0VHJhbnNmb3JtKCksIHApO1xyXG4gIH1cclxuXHJcbiAgQ29tcHV0ZURpc3RhbmNlKHA6IGIyVmVjMiwgbm9ybWFsOiBiMlZlYzIsIGNoaWxkSW5kZXg6IG51bWJlcik6IG51bWJlciB7XHJcbiAgICBpZiAoQjJfRU5BQkxFX1BBUlRJQ0xFKSB7XHJcbiAgICAgIHJldHVybiB0aGlzLm1fc2hhcGUuQ29tcHV0ZURpc3RhbmNlKHRoaXMubV9ib2R5LkdldFRyYW5zZm9ybSgpLCBwLCBub3JtYWwsIGNoaWxkSW5kZXgpO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgcmV0dXJuIDAuMDtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vLyBDYXN0IGEgcmF5IGFnYWluc3QgdGhpcyBzaGFwZS5cclxuICAvLy8gQHBhcmFtIG91dHB1dCB0aGUgcmF5LWNhc3QgcmVzdWx0cy5cclxuICAvLy8gQHBhcmFtIGlucHV0IHRoZSByYXktY2FzdCBpbnB1dCBwYXJhbWV0ZXJzLlxyXG4gIFJheUNhc3Qob3V0cHV0OiBiMlJheUNhc3RPdXRwdXQsIGlucHV0OiBiMlJheUNhc3RJbnB1dCwgY2hpbGRJbmRleDogbnVtYmVyKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3NoYXBlLlJheUNhc3Qob3V0cHV0LCBpbnB1dCwgdGhpcy5tX2JvZHkuR2V0VHJhbnNmb3JtKCksIGNoaWxkSW5kZXgpO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgbWFzcyBkYXRhIGZvciB0aGlzIGZpeHR1cmUuIFRoZSBtYXNzIGRhdGEgaXMgYmFzZWQgb24gdGhlIGRlbnNpdHkgYW5kXHJcbiAgLy8vIHRoZSBzaGFwZS4gVGhlIHJvdGF0aW9uYWwgaW5lcnRpYSBpcyBhYm91dCB0aGUgc2hhcGUncyBvcmlnaW4uIFRoaXMgb3BlcmF0aW9uXHJcbiAgLy8vIG1heSBiZSBleHBlbnNpdmUuXHJcbiAgR2V0TWFzc0RhdGEobWFzc0RhdGE6IGIyTWFzc0RhdGEgPSBuZXcgYjJNYXNzRGF0YSgpKTogYjJNYXNzRGF0YSB7XHJcbiAgICB0aGlzLm1fc2hhcGUuQ29tcHV0ZU1hc3MobWFzc0RhdGEsIHRoaXMubV9kZW5zaXR5KTtcclxuXHJcbiAgICByZXR1cm4gbWFzc0RhdGE7XHJcbiAgfVxyXG5cclxuICAvLy8gU2V0IHRoZSBkZW5zaXR5IG9mIHRoaXMgZml4dHVyZS4gVGhpcyB3aWxsIF9ub3RfIGF1dG9tYXRpY2FsbHkgYWRqdXN0IHRoZSBtYXNzXHJcbiAgLy8vIG9mIHRoZSBib2R5LiBZb3UgbXVzdCBjYWxsIGIyQm9keTo6UmVzZXRNYXNzRGF0YSB0byB1cGRhdGUgdGhlIGJvZHkncyBtYXNzLlxyXG4gIFNldERlbnNpdHkoZGVuc2l0eTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fZGVuc2l0eSA9IGRlbnNpdHk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBkZW5zaXR5IG9mIHRoaXMgZml4dHVyZS5cclxuICBHZXREZW5zaXR5KCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX2RlbnNpdHk7XHJcbiAgfVxyXG5cclxuICAvLy8gR2V0IHRoZSBjb2VmZmljaWVudCBvZiBmcmljdGlvbi5cclxuICBHZXRGcmljdGlvbigpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMubV9mcmljdGlvbjtcclxuICB9XHJcblxyXG4gIC8vLyBTZXQgdGhlIGNvZWZmaWNpZW50IG9mIGZyaWN0aW9uLiBUaGlzIHdpbGwgX25vdF8gY2hhbmdlIHRoZSBmcmljdGlvbiBvZlxyXG4gIC8vLyBleGlzdGluZyBjb250YWN0cy5cclxuICBTZXRGcmljdGlvbihmcmljdGlvbjogbnVtYmVyKTogdm9pZCB7XHJcbiAgICB0aGlzLm1fZnJpY3Rpb24gPSBmcmljdGlvbjtcclxuICB9XHJcblxyXG4gIC8vLyBHZXQgdGhlIGNvZWZmaWNpZW50IG9mIHJlc3RpdHV0aW9uLlxyXG4gIEdldFJlc3RpdHV0aW9uKCk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy5tX3Jlc3RpdHV0aW9uO1xyXG4gIH1cclxuXHJcbiAgLy8vIFNldCB0aGUgY29lZmZpY2llbnQgb2YgcmVzdGl0dXRpb24uIFRoaXMgd2lsbCBfbm90XyBjaGFuZ2UgdGhlIHJlc3RpdHV0aW9uIG9mXHJcbiAgLy8vIGV4aXN0aW5nIGNvbnRhY3RzLlxyXG4gIFNldFJlc3RpdHV0aW9uKHJlc3RpdHV0aW9uOiBudW1iZXIpOiB2b2lkIHtcclxuICAgIHRoaXMubV9yZXN0aXR1dGlvbiA9IHJlc3RpdHV0aW9uO1xyXG4gIH1cclxuXHJcbiAgLy8vIEdldCB0aGUgZml4dHVyZSdzIEFBQkIuIFRoaXMgQUFCQiBtYXkgYmUgZW5sYXJnZSBhbmQvb3Igc3RhbGUuXHJcbiAgLy8vIElmIHlvdSBuZWVkIGEgbW9yZSBhY2N1cmF0ZSBBQUJCLCBjb21wdXRlIGl0IHVzaW5nIHRoZSBzaGFwZSBhbmRcclxuICAvLy8gdGhlIGJvZHkgdHJhbnNmb3JtLlxyXG4gIEdldEFBQkIoY2hpbGRJbmRleDogbnVtYmVyKTogUmVhZG9ubHk8YjJBQUJCPiB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KDAgPD0gY2hpbGRJbmRleCAmJiBjaGlsZEluZGV4IDwgdGhpcy5tX3Byb3h5Q291bnQpO1xyXG4gICAgcmV0dXJuIHRoaXMubV9wcm94aWVzW2NoaWxkSW5kZXhdLmFhYmI7XHJcbiAgfVxyXG5cclxuICAvLyBUaGVzZSBzdXBwb3J0IGJvZHkgYWN0aXZhdGlvbi9kZWFjdGl2YXRpb24uXHJcbiAgQ3JlYXRlUHJveGllcygpOiB2b2lkIHtcclxuICAgIGlmICh0aGlzLm1fcHJveGllcy5sZW5ndGggIT09IDApIHtcclxuICAgICAgdGhyb3cgbmV3IEVycm9yKCk7XHJcbiAgICB9XHJcbiAgICAvLyBDcmVhdGUgcHJveGllcyBpbiB0aGUgYnJvYWQtcGhhc2UuXHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9zaGFwZS5HZXRDaGlsZENvdW50KCk7ICsraSkge1xyXG4gICAgICB0aGlzLm1fcHJveGllc1tpXSA9IG5ldyBiMkZpeHR1cmVQcm94eSh0aGlzLCBpKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIERlc3Ryb3lQcm94aWVzKCk6IHZvaWQge1xyXG4gICAgLy8gRGVzdHJveSBwcm94aWVzIGluIHRoZSBicm9hZC1waGFzZS5cclxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5tX3Byb3hpZXMubGVuZ3RoOyArK2kpIHtcclxuICAgICAgdGhpcy5tX3Byb3hpZXNbaV0uUmVzZXQoKTtcclxuICAgIH1cclxuICAgIHRoaXMubV9wcm94aWVzLmxlbmd0aCA9IDA7XHJcbiAgfVxyXG5cclxuICBUb3VjaFByb3hpZXMoKTogdm9pZCB7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMubV9wcm94aWVzLmxlbmd0aDsgKytpKSB7XHJcbiAgICAgIHRoaXMubV9wcm94aWVzW2ldLlRvdWNoKCk7XHJcbiAgICB9XHJcbiAgfVxyXG5cclxuICBTeW5jaHJvbml6ZVByb3hpZXModHJhbnNmb3JtMTogYjJUcmFuc2Zvcm0sIHRyYW5zZm9ybTI6IGIyVHJhbnNmb3JtLCBkaXNwbGFjZW1lbnQ6IGIyVmVjMik6IHZvaWQge1xyXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLm1fcHJveGllcy5sZW5ndGg7ICsraSkge1xyXG4gICAgICB0aGlzLm1fcHJveGllc1tpXS5TeW5jaHJvbml6ZSh0cmFuc2Zvcm0xLCB0cmFuc2Zvcm0yLCBkaXNwbGFjZW1lbnQpO1xyXG4gICAgfVxyXG4gIH1cclxufVxyXG4iXX0=