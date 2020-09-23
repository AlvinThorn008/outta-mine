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
import { b2_epsilon, b2_pi, b2Assert } from './b2Settings';
export const b2_pi_over_180 = b2_pi / 180;
export const b2_180_over_pi = 180 / b2_pi;
export const b2_two_pi = 2 * b2_pi;
export function b2Abs(x) {
    return x < 0 ? -x : x;
}
export function b2AbsInt(x) {
    return x >= 0 ? x : -x;
}
export function b2Min(a, b) {
    return a < b ? a : b;
}
export function b2Max(a, b) {
    return a > b ? a : b;
}
// separate for Smi type
export function b2MinInt(a, b) {
    return a < b ? a : b;
}
export function b2MaxInt(a, b) {
    return a > b ? a : b;
}
export function b2Clamp(a, lo, hi) {
    return a < lo ? lo : a > hi ? hi : a;
}
export function b2ClampInt(a, lo, hi) {
    return a < lo ? lo : a > hi ? hi : a;
}
export function b2Swap(a, b) {
    !!B2_DEBUG && b2Assert(false);
    const tmp = a[0];
    a[0] = b[0];
    b[0] = tmp;
}
/// This function is used to ensure that a floating point number is
/// not a NaN or infinity.
export const b2IsValid = isFinite;
export function b2Sq(n) {
    return n * n;
}
/// This is a approximate yet fast inverse square-root.
export function b2InvSqrt(n) {
    return 1.0 / Math.sqrt(n);
}
export const b2Sqrt = Math.sqrt;
export const b2Pow = Math.pow;
export function b2DegToRad(degrees) {
    return degrees * b2_pi_over_180;
}
export function b2RadToDeg(radians) {
    return radians * b2_180_over_pi;
}
export const b2Sin = Math.sin;
export const b2Cos = Math.cos;
export const b2Acos = Math.acos;
export const b2Asin = Math.asin;
export const b2Atan2 = Math.atan2;
export function b2NextPowerOfTwo(x) {
    x |= (x >> 1) & 0x7fffffff;
    x |= (x >> 2) & 0x3fffffff;
    x |= (x >> 4) & 0x0fffffff;
    x |= (x >> 8) & 0x00ffffff;
    x |= (x >> 16) & 0x0000ffff;
    return x + 1;
}
export function b2IsPowerOfTwo(x) {
    return x > 0 && (x & (x - 1)) === 0;
}
export function b2Random() {
    return Math.random() * 2.0 - 1.0;
}
export function b2RandomRange(lo, hi) {
    return (hi - lo) * Math.random() + lo;
}
/// A 2D column vector.
export class b2Vec2 {
    constructor(x = 0.0, y = 0.0) {
        this.x = NaN;
        this.y = NaN;
        this.x = x;
        this.y = y;
    }
    Clone() {
        return new b2Vec2(this.x, this.y);
    }
    SetZero() {
        this.x = 0.0;
        this.y = 0.0;
        return this;
    }
    Set(x, y) {
        this.x = x;
        this.y = y;
        return this;
    }
    Copy(other) {
        this.x = other.x;
        this.y = other.y;
        return this;
    }
    SelfAdd(v) {
        this.x += v.x;
        this.y += v.y;
        return this;
    }
    SelfAddXY(x, y) {
        this.x += x;
        this.y += y;
        return this;
    }
    SelfSub(v) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }
    SelfSubXY(x, y) {
        this.x -= x;
        this.y -= y;
        return this;
    }
    SelfMul(s) {
        this.x *= s;
        this.y *= s;
        return this;
    }
    SelfMulAdd(s, v) {
        this.x += s * v.x;
        this.y += s * v.y;
        return this;
    }
    SelfMulSub(s, v) {
        this.x -= s * v.x;
        this.y -= s * v.y;
        return this;
    }
    Dot(v) {
        return this.x * v.x + this.y * v.y;
    }
    Cross(v) {
        return this.x * v.y - this.y * v.x;
    }
    Length() {
        const x = this.x, y = this.y;
        return Math.sqrt(x * x + y * y);
    }
    LengthSquared() {
        const x = this.x, y = this.y;
        return x * x + y * y;
    }
    Normalize() {
        const length = this.Length();
        if (length >= b2_epsilon) {
            const inv_length = 1 / length;
            this.x *= inv_length;
            this.y *= inv_length;
        }
        return length;
    }
    SelfNormalize() {
        const length = this.Length();
        if (length >= b2_epsilon) {
            const inv_length = 1 / length;
            this.x *= inv_length;
            this.y *= inv_length;
        }
        return this;
    }
    SelfRotate(radians) {
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        const x = this.x;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    }
    SelfRotateCosSin(c, s) {
        const x = this.x;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    }
    IsValid() {
        return isFinite(this.x) && isFinite(this.y);
    }
    SelfCrossVS(s) {
        const x = this.x;
        this.x = s * this.y;
        this.y = -s * x;
        return this;
    }
    SelfCrossSV(s) {
        const x = this.x;
        this.x = -s * this.y;
        this.y = s * x;
        return this;
    }
    SelfMinV(v) {
        this.x = b2Min(this.x, v.x);
        this.y = b2Min(this.y, v.y);
        return this;
    }
    SelfMaxV(v) {
        this.x = b2Max(this.x, v.x);
        this.y = b2Max(this.y, v.y);
        return this;
    }
    SelfAbs() {
        this.x = b2Abs(this.x);
        this.y = b2Abs(this.y);
        return this;
    }
    SelfNeg() {
        this.x = -this.x;
        this.y = -this.y;
        return this;
    }
    SelfSkew() {
        const x = this.x;
        this.x = -this.y;
        this.y = x;
        return this;
    }
    static MakeArray(length) {
        const arr = new Array(length);
        for (let i = 0; i < length; ++i) {
            arr[i] = new b2Vec2();
        }
        return arr;
    }
    static AbsV(v, out) {
        out.x = b2Abs(v.x);
        out.y = b2Abs(v.y);
        return out;
    }
    static MinV(a, b, out) {
        out.x = b2Min(a.x, b.x);
        out.y = b2Min(a.y, b.y);
        return out;
    }
    static MaxV(a, b, out) {
        out.x = b2Max(a.x, b.x);
        out.y = b2Max(a.y, b.y);
        return out;
    }
    static ClampV(v, lo, hi, out) {
        out.x = b2Clamp(v.x, lo.x, hi.x);
        out.y = b2Clamp(v.y, lo.y, hi.y);
        return out;
    }
    static RotateV(v, radians, out) {
        const v_x = v.x;
        const v_y = v.y;
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        out.x = c * v_x - s * v_y;
        out.y = s * v_x + c * v_y;
        return out;
    }
    static DotVV(a, b) {
        return a.x * b.x + a.y * b.y;
    }
    static CrossVV(a, b) {
        return a.x * b.y - a.y * b.x;
    }
    static CrossVS(v, s, out) {
        const v_x = v.x;
        out.x = s * v.y;
        out.y = -s * v_x;
        return out;
    }
    static CrossVOne(v, out) {
        const v_x = v.x;
        out.x = v.y;
        out.y = -v_x;
        return out;
    }
    static CrossSV(s, v, out) {
        const v_x = v.x;
        out.x = -s * v.y;
        out.y = s * v_x;
        return out;
    }
    static CrossOneV(v, out) {
        const v_x = v.x;
        out.x = -v.y;
        out.y = v_x;
        return out;
    }
    static AddVV(a, b, out) {
        out.x = a.x + b.x;
        out.y = a.y + b.y;
        return out;
    }
    static SubVV(a, b, out) {
        out.x = a.x - b.x;
        out.y = a.y - b.y;
        return out;
    }
    static MulSV(s, v, out) {
        out.x = v.x * s;
        out.y = v.y * s;
        return out;
    }
    static MulVS(v, s, out) {
        out.x = v.x * s;
        out.y = v.y * s;
        return out;
    }
    static AddVMulSV(a, s, b, out) {
        out.x = a.x + s * b.x;
        out.y = a.y + s * b.y;
        return out;
    }
    static SubVMulSV(a, s, b, out) {
        out.x = a.x - s * b.x;
        out.y = a.y - s * b.y;
        return out;
    }
    static AddVCrossSV(a, s, v, out) {
        const v_x = v.x;
        out.x = a.x - s * v.y;
        out.y = a.y + s * v_x;
        return out;
    }
    static MidVV(a, b, out) {
        out.x = (a.x + b.x) * 0.5;
        out.y = (a.y + b.y) * 0.5;
        return out;
    }
    static ExtVV(a, b, out) {
        out.x = (b.x - a.x) * 0.5;
        out.y = (b.y - a.y) * 0.5;
        return out;
    }
    static IsEqualToV(a, b) {
        return a.x === b.x && a.y === b.y;
    }
    static DistanceVV(a, b) {
        const c_x = a.x - b.x;
        const c_y = a.y - b.y;
        return Math.sqrt(c_x * c_x + c_y * c_y);
    }
    static DistanceSquaredVV(a, b) {
        const c_x = a.x - b.x;
        const c_y = a.y - b.y;
        return c_x * c_x + c_y * c_y;
    }
    static NegV(v, out) {
        out.x = -v.x;
        out.y = -v.y;
        return out;
    }
}
b2Vec2.ZERO = new b2Vec2(0, 0);
b2Vec2.UNITX = new b2Vec2(1, 0);
b2Vec2.UNITY = new b2Vec2(0, 1);
b2Vec2.s_t0 = new b2Vec2();
b2Vec2.s_t1 = new b2Vec2();
b2Vec2.s_t2 = new b2Vec2();
b2Vec2.s_t3 = new b2Vec2();
export const b2Vec2_zero = new b2Vec2(0, 0);
/// A 2D column vector with 3 elements.
export class b2Vec3 {
    constructor(x = 0.0, y = 0.0, z = 0.0) {
        this.x = NaN;
        this.y = NaN;
        this.z = NaN;
        this.x = x;
        this.y = y;
        this.z = z;
    }
    Clone() {
        return new b2Vec3(this.x, this.y, this.z);
    }
    SetZero() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        return this;
    }
    SetXYZ(x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }
    Copy(other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        return this;
    }
    SelfNeg() {
        this.x = -this.x;
        this.y = -this.y;
        this.z = -this.z;
        return this;
    }
    SelfAdd(v) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
        return this;
    }
    SelfAddXYZ(x, y, z) {
        this.x += x;
        this.y += y;
        this.z += z;
        return this;
    }
    SelfSub(v) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
        return this;
    }
    SelfSubXYZ(x, y, z) {
        this.x -= x;
        this.y -= y;
        this.z -= z;
        return this;
    }
    SelfMul(s) {
        this.x *= s;
        this.y *= s;
        this.z *= s;
        return this;
    }
    static DotV3V3(a, b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    static CrossV3V3(a, b, out) {
        const a_x = a.x, a_y = a.y, a_z = a.z;
        const b_x = b.x, b_y = b.y, b_z = b.z;
        out.x = a_y * b_z - a_z * b_y;
        out.y = a_z * b_x - a_x * b_z;
        out.z = a_x * b_y - a_y * b_x;
        return out;
    }
}
b2Vec3.ZERO = new b2Vec3(0, 0, 0);
b2Vec3.s_t0 = new b2Vec3();
/// A 2-by-2 matrix. Stored in column-major order.
export class b2Mat22 {
    constructor() {
        this.ex = new b2Vec2(1, 0);
        this.ey = new b2Vec2(0, 1);
    }
    Clone() {
        return new b2Mat22().Copy(this);
    }
    static FromVV(c1, c2) {
        return new b2Mat22().SetVV(c1, c2);
    }
    static FromSSSS(r1c1, r1c2, r2c1, r2c2) {
        return new b2Mat22().SetSSSS(r1c1, r1c2, r2c1, r2c2);
    }
    static FromAngle(radians) {
        return new b2Mat22().SetAngle(radians);
    }
    SetSSSS(r1c1, r1c2, r2c1, r2c2) {
        this.ex.Set(r1c1, r2c1);
        this.ey.Set(r1c2, r2c2);
        return this;
    }
    SetVV(c1, c2) {
        this.ex.Copy(c1);
        this.ey.Copy(c2);
        return this;
    }
    SetAngle(radians) {
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        this.ex.Set(c, s);
        this.ey.Set(-s, c);
        return this;
    }
    Copy(other) {
        this.ex.Copy(other.ex);
        this.ey.Copy(other.ey);
        return this;
    }
    SetIdentity() {
        this.ex.Set(1, 0);
        this.ey.Set(0, 1);
        return this;
    }
    SetZero() {
        this.ex.SetZero();
        this.ey.SetZero();
        return this;
    }
    GetAngle() {
        return Math.atan2(this.ex.y, this.ex.x);
    }
    GetInverse(out) {
        const a = this.ex.x;
        const b = this.ey.x;
        const c = this.ex.y;
        const d = this.ey.y;
        let det = a * d - b * c;
        if (det !== 0) {
            det = 1 / det;
        }
        out.ex.x = det * d;
        out.ey.x = -det * b;
        out.ex.y = -det * c;
        out.ey.y = det * a;
        return out;
    }
    Solve(b_x, b_y, out) {
        const a11 = this.ex.x, a12 = this.ey.x;
        const a21 = this.ex.y, a22 = this.ey.y;
        let det = a11 * a22 - a12 * a21;
        if (det !== 0) {
            det = 1.0 / det;
        }
        out.x = det * (a22 * b_x - a12 * b_y);
        out.y = det * (a11 * b_y - a21 * b_x);
        return out;
    }
    SelfAbs() {
        this.ex.SelfAbs();
        this.ey.SelfAbs();
        return this;
    }
    SelfInv() {
        this.GetInverse(this);
        return this;
    }
    SelfAddM(M) {
        this.ex.SelfAdd(M.ex);
        this.ey.SelfAdd(M.ey);
        return this;
    }
    SelfSubM(M) {
        this.ex.SelfSub(M.ex);
        this.ey.SelfSub(M.ey);
        return this;
    }
    static AbsM(M, out) {
        const M_ex = M.ex, M_ey = M.ey;
        out.ex.x = b2Abs(M_ex.x);
        out.ex.y = b2Abs(M_ex.y);
        out.ey.x = b2Abs(M_ey.x);
        out.ey.y = b2Abs(M_ey.y);
        return out;
    }
    static MulMV(M, v, out) {
        const M_ex = M.ex;
        const M_ey = M.ey;
        const v_x = v.x;
        const v_y = v.y;
        out.x = M_ex.x * v_x + M_ey.x * v_y;
        out.y = M_ex.y * v_x + M_ey.y * v_y;
        return out;
    }
    static MulTMV(M, v, out) {
        const M_ex = M.ex;
        const M_ey = M.ey;
        const v_x = v.x;
        const v_y = v.y;
        out.x = M_ex.x * v_x + M_ex.y * v_y;
        out.y = M_ey.x * v_x + M_ey.y * v_y;
        return out;
    }
    static AddMM(A, B, out) {
        const A_ex = A.ex;
        const A_ey = A.ey;
        const B_ex = B.ex;
        const B_ey = B.ey;
        out.ex.x = A_ex.x + B_ex.x;
        out.ex.y = A_ex.y + B_ex.y;
        out.ey.x = A_ey.x + B_ey.x;
        out.ey.y = A_ey.y + B_ey.y;
        return out;
    }
    static MulMM(A, B, out) {
        const A_ex_x = A.ex.x;
        const A_ex_y = A.ex.y;
        const A_ey_x = A.ey.x;
        const A_ey_y = A.ey.y;
        const B_ex_x = B.ex.x;
        const B_ex_y = B.ex.y;
        const B_ey_x = B.ey.x;
        const B_ey_y = B.ey.y;
        out.ex.x = A_ex_x * B_ex_x + A_ey_x * B_ex_y;
        out.ex.y = A_ex_y * B_ex_x + A_ey_y * B_ex_y;
        out.ey.x = A_ex_x * B_ey_x + A_ey_x * B_ey_y;
        out.ey.y = A_ex_y * B_ey_x + A_ey_y * B_ey_y;
        return out;
    }
    static MulTMM(A, B, out) {
        const A_ex_x = A.ex.x;
        const A_ex_y = A.ex.y;
        const A_ey_x = A.ey.x;
        const A_ey_y = A.ey.y;
        const B_ex_x = B.ex.x;
        const B_ex_y = B.ex.y;
        const B_ey_x = B.ey.x;
        const B_ey_y = B.ey.y;
        out.ex.x = A_ex_x * B_ex_x + A_ex_y * B_ex_y;
        out.ex.y = A_ey_x * B_ex_x + A_ey_y * B_ex_y;
        out.ey.x = A_ex_x * B_ey_x + A_ex_y * B_ey_y;
        out.ey.y = A_ey_x * B_ey_x + A_ey_y * B_ey_y;
        return out;
    }
}
b2Mat22.IDENTITY = new b2Mat22();
/// A 3-by-3 matrix. Stored in column-major order.
export class b2Mat33 {
    constructor() {
        this.ex = new b2Vec3(1, 0, 0);
        this.ey = new b2Vec3(0, 1, 0);
        this.ez = new b2Vec3(0, 0, 1);
    }
    Clone() {
        return new b2Mat33().Copy(this);
    }
    SetVVV(c1, c2, c3) {
        this.ex.Copy(c1);
        this.ey.Copy(c2);
        this.ez.Copy(c3);
        return this;
    }
    Copy(other) {
        this.ex.Copy(other.ex);
        this.ey.Copy(other.ey);
        this.ez.Copy(other.ez);
        return this;
    }
    SetIdentity() {
        this.ex.SetXYZ(1, 0, 0);
        this.ey.SetXYZ(0, 1, 0);
        this.ez.SetXYZ(0, 0, 1);
        return this;
    }
    SetZero() {
        this.ex.SetZero();
        this.ey.SetZero();
        this.ez.SetZero();
        return this;
    }
    SelfAddM(M) {
        this.ex.SelfAdd(M.ex);
        this.ey.SelfAdd(M.ey);
        this.ez.SelfAdd(M.ez);
        return this;
    }
    Solve33(b_x, b_y, b_z, out) {
        const a11 = this.ex.x;
        const a21 = this.ex.y;
        const a31 = this.ex.z;
        const a12 = this.ey.x;
        const a22 = this.ey.y;
        const a32 = this.ey.z;
        const a13 = this.ez.x;
        const a23 = this.ez.y;
        const a33 = this.ez.z;
        let det = a11 * (a22 * a33 - a32 * a23) + a21 * (a32 * a13 - a12 * a33) + a31 * (a12 * a23 - a22 * a13);
        if (det !== 0) {
            det = 1.0 / det;
        }
        out.x =
            det *
                (b_x * (a22 * a33 - a32 * a23) +
                    b_y * (a32 * a13 - a12 * a33) +
                    b_z * (a12 * a23 - a22 * a13));
        out.y =
            det *
                (a11 * (b_y * a33 - b_z * a23) +
                    a21 * (b_z * a13 - b_x * a33) +
                    a31 * (b_x * a23 - b_y * a13));
        out.z =
            det *
                (a11 * (a22 * b_z - a32 * b_y) +
                    a21 * (a32 * b_x - a12 * b_z) +
                    a31 * (a12 * b_y - a22 * b_x));
        return out;
    }
    Solve22(b_x, b_y, out) {
        const a11 = this.ex.x;
        const a12 = this.ey.x;
        const a21 = this.ex.y;
        const a22 = this.ey.y;
        let det = a11 * a22 - a12 * a21;
        if (det !== 0) {
            det = 1.0 / det;
        }
        out.x = det * (a22 * b_x - a12 * b_y);
        out.y = det * (a11 * b_y - a21 * b_x);
        return out;
    }
    GetInverse22(M) {
        const a = this.ex.x;
        const b = this.ey.x;
        const c = this.ex.y;
        const d = this.ey.y;
        let det = a * d - b * c;
        if (det !== 0) {
            det = 1.0 / det;
        }
        M.ex.x = det * d;
        M.ey.x = -det * b;
        M.ex.z = 0;
        M.ex.y = -det * c;
        M.ey.y = det * a;
        M.ey.z = 0;
        M.ez.x = 0;
        M.ez.y = 0;
        M.ez.z = 0;
    }
    GetSymInverse33(M) {
        let det = b2Vec3.DotV3V3(this.ex, b2Vec3.CrossV3V3(this.ey, this.ez, b2Vec3.s_t0));
        if (det !== 0) {
            det = 1 / det;
        }
        const a11 = this.ex.x, a12 = this.ey.x, a13 = this.ez.x;
        const a22 = this.ey.y, a23 = this.ez.y;
        const a33 = this.ez.z;
        M.ex.x = det * (a22 * a33 - a23 * a23);
        M.ex.y = det * (a13 * a23 - a12 * a33);
        M.ex.z = det * (a12 * a23 - a13 * a22);
        M.ey.x = M.ex.y;
        M.ey.y = det * (a11 * a33 - a13 * a13);
        M.ey.z = det * (a13 * a12 - a11 * a23);
        M.ez.x = M.ex.z;
        M.ez.y = M.ey.z;
        M.ez.z = det * (a11 * a22 - a12 * a12);
    }
    static MulM33V3(A, v, out) {
        const v_x = v.x, v_y = v.y, v_z = v.z;
        out.x = A.ex.x * v_x + A.ey.x * v_y + A.ez.x * v_z;
        out.y = A.ex.y * v_x + A.ey.y * v_y + A.ez.y * v_z;
        out.z = A.ex.z * v_x + A.ey.z * v_y + A.ez.z * v_z;
        return out;
    }
    static MulM33XYZ(A, x, y, z, out) {
        out.x = A.ex.x * x + A.ey.x * y + A.ez.x * z;
        out.y = A.ex.y * x + A.ey.y * y + A.ez.y * z;
        out.z = A.ex.z * x + A.ey.z * y + A.ez.z * z;
        return out;
    }
    static MulM33V2(A, v, out) {
        const v_x = v.x, v_y = v.y;
        out.x = A.ex.x * v_x + A.ey.x * v_y;
        out.y = A.ex.y * v_x + A.ey.y * v_y;
        return out;
    }
    static MulM33XY(A, x, y, out) {
        out.x = A.ex.x * x + A.ey.x * y;
        out.y = A.ex.y * x + A.ey.y * y;
        return out;
    }
}
b2Mat33.IDENTITY = new b2Mat33();
/// Rotation
export class b2Rot {
    constructor(angle = 0.0) {
        this.s = NaN;
        this.c = NaN;
        this.s = Math.sin(angle);
        this.c = Math.cos(angle);
    }
    Clone() {
        return new b2Rot().Copy(this);
    }
    Copy(other) {
        this.s = other.s;
        this.c = other.c;
        return this;
    }
    SetAngle(angle) {
        this.s = Math.sin(angle);
        this.c = Math.cos(angle);
        return this;
    }
    SetIdentity() {
        this.s = 0;
        this.c = 1;
        return this;
    }
    GetAngle() {
        return Math.atan2(this.s, this.c);
    }
    GetXAxis(out) {
        out.x = this.c;
        out.y = this.s;
        return out;
    }
    GetYAxis(out) {
        out.x = -this.s;
        out.y = this.c;
        return out;
    }
    static MulRR(q, r, out) {
        // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
        // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
        // s = qs * rc + qc * rs
        // c = qc * rc - qs * rs
        const q_c = q.c, q_s = q.s;
        const r_c = r.c, r_s = r.s;
        out.s = q_s * r_c + q_c * r_s;
        out.c = q_c * r_c - q_s * r_s;
        return out;
    }
    static MulTRR(q, r, out) {
        // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
        // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
        // s = qc * rs - qs * rc
        // c = qc * rc + qs * rs
        const q_c = q.c, q_s = q.s;
        const r_c = r.c, r_s = r.s;
        out.s = q_c * r_s - q_s * r_c;
        out.c = q_c * r_c + q_s * r_s;
        return out;
    }
    static MulRV(q, v, out) {
        const q_c = q.c, q_s = q.s;
        const v_x = v.x, v_y = v.y;
        out.x = q_c * v_x - q_s * v_y;
        out.y = q_s * v_x + q_c * v_y;
        return out;
    }
    static MulTRV(q, v, out) {
        const q_c = q.c, q_s = q.s;
        const v_x = v.x, v_y = v.y;
        out.x = q_c * v_x + q_s * v_y;
        out.y = -q_s * v_x + q_c * v_y;
        return out;
    }
}
b2Rot.IDENTITY = new b2Rot();
/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
export class b2Transform {
    constructor() {
        this.p = new b2Vec2();
        this.q = new b2Rot();
    }
    Clone() {
        return new b2Transform().Copy(this);
    }
    Copy(other) {
        this.p.Copy(other.p);
        this.q.Copy(other.q);
        return this;
    }
    SetIdentity() {
        this.p.SetZero();
        this.q.SetIdentity();
        return this;
    }
    SetPositionRotation(position, q) {
        this.p.Copy(position);
        this.q.Copy(q);
        return this;
    }
    SetPositionAngle(pos, a) {
        this.p.Copy(pos);
        this.q.SetAngle(a);
        return this;
    }
    SetPosition(position) {
        this.p.Copy(position);
        return this;
    }
    SetPositionXY(x, y) {
        this.p.Set(x, y);
        return this;
    }
    SetRotation(rotation) {
        this.q.Copy(rotation);
        return this;
    }
    SetRotationAngle(radians) {
        this.q.SetAngle(radians);
        return this;
    }
    GetPosition() {
        return this.p;
    }
    GetRotation() {
        return this.q;
    }
    GetRotationAngle() {
        return this.q.GetAngle();
    }
    GetAngle() {
        return this.q.GetAngle();
    }
    static MulXV(T, v, out) {
        // float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
        // float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
        // return b2Vec2(x, y);
        const T_q_c = T.q.c, T_q_s = T.q.s;
        const v_x = v.x, v_y = v.y;
        out.x = T_q_c * v_x - T_q_s * v_y + T.p.x;
        out.y = T_q_s * v_x + T_q_c * v_y + T.p.y;
        return out;
    }
    static MulTXV(T, v, out) {
        // float32 px = v.x - T.p.x;
        // float32 py = v.y - T.p.y;
        // float32 x = (T.q.c * px + T.q.s * py);
        // float32 y = (-T.q.s * px + T.q.c * py);
        // return b2Vec2(x, y);
        const T_q_c = T.q.c, T_q_s = T.q.s;
        const p_x = v.x - T.p.x;
        const p_y = v.y - T.p.y;
        out.x = T_q_c * p_x + T_q_s * p_y;
        out.y = -T_q_s * p_x + T_q_c * p_y;
        return out;
    }
    static MulXX(A, B, out) {
        b2Rot.MulRR(A.q, B.q, out.q);
        b2Vec2.AddVV(b2Rot.MulRV(A.q, B.p, out.p), A.p, out.p);
        return out;
    }
    static MulTXX(A, B, out) {
        b2Rot.MulTRR(A.q, B.q, out.q);
        b2Rot.MulTRV(A.q, b2Vec2.SubVV(B.p, A.p, out.p), out.p);
        return out;
    }
}
b2Transform.IDENTITY = new b2Transform();
/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
export class b2Sweep {
    constructor() {
        this.localCenter = new b2Vec2();
        this.c0 = new b2Vec2();
        this.c = new b2Vec2();
        this.a0 = NaN;
        this.a = NaN;
        this.alpha0 = NaN;
        this.a0 = 0.0;
        this.a = 0.0;
        this.alpha0 = 0.0;
    }
    Clone() {
        return new b2Sweep().Copy(this);
    }
    Copy(other) {
        this.localCenter.Copy(other.localCenter);
        this.c0.Copy(other.c0);
        this.c.Copy(other.c);
        this.a0 = other.a0;
        this.a = other.a;
        this.alpha0 = other.alpha0;
        return this;
    }
    GetTransform(xf, beta) {
        const one_minus_beta = 1.0 - beta;
        xf.p.x = one_minus_beta * this.c0.x + beta * this.c.x;
        xf.p.y = one_minus_beta * this.c0.y + beta * this.c.y;
        const angle = one_minus_beta * this.a0 + beta * this.a;
        xf.q.SetAngle(angle);
        xf.p.SelfSub(b2Rot.MulRV(xf.q, this.localCenter, b2Vec2.s_t0));
        return xf;
    }
    Advance(alpha) {
        !!B2_DEBUG && b2Assert(this.alpha0 < 1.0);
        const beta = (alpha - this.alpha0) / (1.0 - this.alpha0);
        const one_minus_beta = 1 - beta;
        this.c0.x = one_minus_beta * this.c0.x + beta * this.c.x;
        this.c0.y = one_minus_beta * this.c0.y + beta * this.c.y;
        this.a0 = one_minus_beta * this.a0 + beta * this.a;
        this.alpha0 = alpha;
    }
    Normalize() {
        const d = b2_two_pi * Math.floor(this.a0 / b2_two_pi);
        this.a0 -= d;
        this.a -= d;
    }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJNYXRoLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL2NvbW1vbi9iMk1hdGgudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7QUFFSCxPQUFPLEVBQUUsVUFBVSxFQUFFLEtBQUssRUFBRSxRQUFRLEVBQUUsTUFBTSxjQUFjLENBQUM7QUFFM0QsTUFBTSxDQUFDLE1BQU0sY0FBYyxHQUFXLEtBQUssR0FBRyxHQUFHLENBQUM7QUFDbEQsTUFBTSxDQUFDLE1BQU0sY0FBYyxHQUFXLEdBQUcsR0FBRyxLQUFLLENBQUM7QUFDbEQsTUFBTSxDQUFDLE1BQU0sU0FBUyxHQUFXLENBQUMsR0FBRyxLQUFLLENBQUM7QUFFM0MsTUFBTSxVQUFVLEtBQUssQ0FBQyxDQUFTO0lBQzdCLE9BQU8sQ0FBQyxHQUFHLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztBQUN4QixDQUFDO0FBRUQsTUFBTSxVQUFVLFFBQVEsQ0FBQyxDQUFTO0lBQ2hDLE9BQU8sQ0FBQyxJQUFJLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztBQUN6QixDQUFDO0FBRUQsTUFBTSxVQUFVLEtBQUssQ0FBQyxDQUFTLEVBQUUsQ0FBUztJQUN4QyxPQUFPLENBQUMsR0FBRyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDO0FBQ3ZCLENBQUM7QUFFRCxNQUFNLFVBQVUsS0FBSyxDQUFDLENBQVMsRUFBRSxDQUFTO0lBQ3hDLE9BQU8sQ0FBQyxHQUFHLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUM7QUFDdkIsQ0FBQztBQUVELHdCQUF3QjtBQUN4QixNQUFNLFVBQVUsUUFBUSxDQUFDLENBQVMsRUFBRSxDQUFTO0lBQzNDLE9BQU8sQ0FB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Q0FBQztJQUVELE1BQU0sQ0FBQyxJQUFJLENBQWUsQ0FBSyxFQUFFLEdBQU07UUFDckMsR0FBRyxDQUFDLENBQUMsR0FBRyxLQUFLLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ25CLEdBQUcsQ0FBQyxDQUFDLEdBQUcsS0FBSyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNuQixPQUFPLEdBQUcsQ0FBQztJQUNiLENBQUM7SUFFRCxNQUFNLENBQUMsSUFBSSxDQUFlLENBQUssRUFBRSxDQUFLLEVBQUUsR0FBTTtRQUM1QyxHQUFHLENBQUMsQ0FBQyxHQUFHLEtBQUssQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUN4QixHQUFHLENBQUMsQ0FBQyxHQUFHLEtBQUssQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUN4QixPQUFPLEdBQUcsQ0FBQztJQUNiLENBQUM7SUFFRCxNQUFNLENBQUMsSUFBSSxDQUFlLENBQUssRUFBRSxDQUFLLEVBQUUsR0FBTTtRQUM1QyxHQUFHLENBQUMsQ0FBQyxHQUFHLEtBQUssQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUN4QixHQUFHLENBQUMsQ0FBQyxHQUFHLEtBQUssQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUN4QixPQUFPLEdBQUcsQ0FBQztJQUNiLENBQUM7SUFFRCxNQUFNLENBQUMsTUFBTSxDQUFlLENBQUssRUFBRSxFQUFNLEVBQUUsRUFBTSxFQUFFLEdBQU07UUFDdkQsR0FBRyxDQUFDLENBQUMsR0FBRyxPQUFPLENBQUMsQ0FBQyxDQUFDLENBQUMsRUFBRSxFQUFFLENBQUMsQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNqQyxHQUFHLENBQUMsQ0FBQyxHQUFHLE9BQU8sQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFF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LEVBQUUsQ0FBQztRQUNsQixNQUFNLElBQUksR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDO1FBQ2xCLE1BQU0sR0FBRyxHQUFHLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDaEIsTUFBTSxHQUFHLEdBQUcsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNoQixHQUFHLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLElBQUksQ0FBQyxDQUFDLEdBQUcsR0FBRyxDQUFDO1FBQ3BDLEdBQUcsQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsSUFBSSxDQUFDLENBQUMsR0FBRyxHQUFHLENBQUM7UUFDcEMsT0FBTyxHQUFHLENBQUM7SUFDYixDQUFDO0lBRUQsTUFBTSxDQUFDLE1BQU0sQ0FBZSxDQUFVLEVBQUUsQ0FBSyxFQUFFLEdBQU07UUFDbkQsTUFBTSxJQUFJLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQztRQUNsQixNQUFNLElBQUksR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDO1FBQ2xCLE1BQU0sR0FBRyxHQUFHLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDaEIsTUFBTSxHQUFHLEdBQUcsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNoQixHQUFHLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLElBQUksQ0FBQyxDQUFDLEdBQUcsR0FBRyxDQUFDO1FBQ3BDLEdBQUcsQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsSUFBSSxDQUFDLENBQUMsR0FBRyxHQUFHLENBQUM7UUFDcEMsT0FBTyxHQUFHLENBQUM7SUFDYixDQUFDO0lBRUQsTUFBTSxDQUFDLEtBQUssQ0FBQyxDQUFVLEVBQUUsQ0FBVSxFQUFFLEdBQVk7UUFDL0MsTUFBTSxJQUFJLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQztRQUNsQixNQUFNLElBQUksR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDO1FBQ2xCLE1BQU0sSUFBSSxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUM7UUFDbEIsTUFBTSxJQUFJLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQztRQUNsQixHQUFHLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxDQUFDLENBQUM7UUFDM0IsR0FBRyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsQ0FBQyxDQUFDO1FBQzNCLEdBQUcsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLENBQUMsQ0FBQztRQUMzQixHQUFHLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxDQUFDLENBQUM7UUFDM0IsT0FBTyxHQUFHLENBQUM7SUFDYixDQUFDO0lBRUQsTUFBTSxDQUFDLEtBQUssQ0FBQyxDQUFVLEVBQUUsQ0FBVSxFQUFFLEdBQVk7UUFDL0MsTUFBTSxNQUFNLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDdEIsTUFBTSxNQUFNLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDdEIsTUFBTSxNQUFNLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDdEIsTUFBTSxNQUFNLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDdEIsTUFBTSxNQUFNLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDdEIsTUFBTSxNQUFNLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDdEIsTUFBTSxNQUFNLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDdEIsTUFBTSxNQUFNLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDdEIsR0FBRyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsTUFBTSxHQUFHLE1BQU0sR0FBRyxNQUFNLEdBQUcsTUFBTSxDQUFDO1FBQzdDLEdBQUcsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLE1BQU0sR0FBRyxNQUFNLEdBQUcsTUFBTSxHQUFHLE1BQU0sQ0FBQztRQUM3QyxHQUFHLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxNQUFNLEdBQUcsTUFBTSxHQUFHLE1BQU0sR0FBRyxNQUFNLENBQUM7UUFDN0MsR0FBRyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsTUFBTSxHQUFHLE1BQU0sR0FBRyxNQUFNLEdBQUcsTUFBTSxDQUFDO1FBQzdDLE9BQU8sR0FBRyxDQUFDO0lBQ2IsQ0FBQztJQUVELE1BQU0sQ0FBQyxNQUFNLENBQUMsQ0FBVSxFQUFFLENBQVUsRUFBRSxHQUFZO1FBQ2hELE1BQU0sTUFBTSxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sTUFBTSxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sTUFBTSxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sTUFBTSxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sTUFBTSxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sTUFBTSxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sTUFBTSxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sTUFBTSxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLEdBQUcsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLE1BQU0sR0FBRyxNQUFNLEdBQUcsTUFBTSxHQUFHLE1BQU0sQ0FBQztRQUM3QyxHQUFHLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxNQUFNLEdBQUcsTUFBTSxHQUFHLE1BQU0sR0FBRyxNQUFN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LENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sR0FBRyxHQUFHLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sR0FBRyxHQUFHLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sR0FBRyxHQUFHLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sR0FBRyxHQUFHLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sR0FBRyxHQUFHLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sR0FBRyxHQUFHLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLE1BQU0sR0FBRyxHQUFHLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3RCLElBQUksR0FBRyxHQUNMLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsQ0FBQyxDQUFDO1FBQ2hHLElBQUksR0FBRyxLQUFLLENBQUMsRUFBRTtZQUNiLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDO1NBQ2pCO1FBQ0QsR0FBRyxDQUFDLENBQUM7WUFDSCxHQUFHO2dCQUNILENBQUMsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDO29CQUM1QixHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLENBQUM7b0JBQzdCLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsQ0FBQyxDQUFDLENBQUM7UUFDbkMsR0FBRyxDQUFDLENBQUM7WUFD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Q0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDaEIsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDLENBQUM7UUFDdkMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDLENBQUM7UUFFdkMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDaEIsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDaEIsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDLENBQUM7SUFDekMsQ0FBQztJQUVELE1BQU0sQ0FBQyxRQUFRLENBQWdCLENBQVUsRUFBRSxDQUFNLEVBQUUsR0FBTTtRQUN2RCxNQUFNLEdBQUcsR0FBVyxDQUFDLENBQUMsQ0FBQyxFQUNyQixHQUFHLEdBQVcsQ0FBQyxDQUFDLENBQUMsRUFDakIsR0FBRyxHQUFXLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDcEIsR0FBRyxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLEdBQUcsQ0FBQztRQUNuRCxHQUFHLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLEdBQUcsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxDQUFDO1FBQ25ELEdBQUcsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLEdBQUcsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLENBQUM7UUFDbkQsT0FBTyxHQUFHLENBQUM7SUFDYixDQUFDO0lBRUQsTUFBTSxDQUFDLFNBQVMsQ0FBZ0IsQ0FBVSxFQUFFLENBQVMsRUFBRSxDQUFTLEVBQUUsQ0FBUyxFQUFFLEdBQU07UUFDakYsR0FBRyxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQztRQUM3QyxHQUFHLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDO1FBQzdDLEdBQUcsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUM7UUFDN0MsT0FBTyxHQUFHLENBQUM7SUFDYixDQUFDO0lBRUQsTUFBTSxDQUFDLFFBQVEsQ0FBZSxDQUFVLEVBQUUsQ0FBSyxFQUFFLEdBQU07UUFDckQsTUFBTSxHQUFHLEdBQVcsQ0FBQyxDQUFDLENBQUMsRUFDckIsR0FBRyxHQUFXLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDcEIsR0FBRyxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsRUFB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QUN4QyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxjQUFjLEdBQUcsSUFBSSxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsSUFBSSxHQUFHLElBQUksQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ3pELElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLGNBQWMsR0FBRyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxJQUFJLEdBQUcsSUFBSSxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDekQsSUFBSSxDQUFDLEVBQUUsR0FBRyxjQUFjLEdBQUcsSUFBSSxDQUFDLEVBQUUsR0FBRyxJQUFJLEdBQUcsSUFBSSxDQUFDLENBQUMsQ0FBQztRQUNuRCxJQUFJLENBQUMsTUFBTSxHQUFHLEtBQUssQ0FBQztJQUN0QixDQUFDO0lBRUQsU0FBUztRQUNQLE1BQU0sQ0FBQyxHQUFHLFNBQVMsR0FBRyxJQUFJLENBQUMsS0FBSyxDQUFDLElBQUksQ0FBQyxFQUFFLEdBQUcsU0FBUyxDQUFDLENBQUM7UUFDdEQsSUFBSSxDQUFDLEVBQUUsSUFBSSxDQUFDLENBQUM7UUFDYixJQUFJLENBQUMsQ0FBQyxJQUFJLENBQUMsQ0FBQztJQUNkLENBQUM7Q0FDRiIsInNvdXJjZXNDb250ZW50IjpbIi8qXHJcbiAqIENvcHlyaWdodCAoYykgMjAwNi0yMDA5IEVyaW4gQ2F0dG8gaHR0cDovL3d3dy5ib3gyZC5vcmdcclxuICpcclxuICogVGhpcyBzb2Z0d2FyZSBpcyBwcm92aWRlZCAnYXMtaXMnLCB3aXRob3V0IGFueSBleHByZXNzIG9yIGltcGxpZWRcclxuICogd2FycmFudHkuICBJbiBubyBldmVudCB3aWxsIHRoZSBhdXRob3JzIGJlIGhlbGQgbGlhYmxlIGZvciBhbnkgZGFtYWdlc1xyXG4gKiBhcmlzaW5nIGZyb20gdGhlIHVzZSBvZiB0aGlzIHNvZnR3YXJlLlxyXG4gKiBQZXJtaXNzaW9uIGlzIGdyYW50ZWQgdG8gYW55b25lIHRvIHVzZSB0aGlzIHNvZnR3YXJlIGZvciBhbnkgcHVycG9zZSxcclxuICogaW5jbHVkaW5nIGNvbW1lcmNpYWwgYXBwbGljYXRpb25zLCBhbmQgdG8gYWx0ZXIgaXQgYW5kIHJlZGlzdHJpYnV0ZSBpdFxyXG4gKiBmcmVlbHksIHN1YmplY3QgdG8gdGhlIGZvbGxvd2luZyByZXN0cmljdGlvbnM6XHJcbiAqIDEuIFRoZSBvcmlnaW4gb2YgdGhpcyBzb2Z0d2FyZSBtdXN0IG5vdCBiZSBtaXNyZXByZXNlbnRlZDsgeW91IG11c3Qgbm90XHJcbiAqIGNsYWltIHRoYXQgeW91IHdyb3RlIHRoZSBvcmlnaW5hbCBzb2Z0d2FyZS4gSWYgeW91IHVzZSB0aGlzIHNvZnR3YXJlXHJcbiAqIGluIGEgcHJvZHVjdCwgYW4gYWNrbm93bGVkZ21lbnQgaW4gdGhlIHByb2R1Y3QgZG9jdW1lbnRhdGlvbiB3b3VsZCBiZVxyXG4gKiBhcHByZWNpYXRlZCBidXQgaXMgbm90IHJlcXVpcmVkLlxyXG4gKiAyLiBBbHRlcmVkIHNvdXJjZSB2ZXJzaW9ucyBtdXN0IGJlIHBsYWlubHkgbWFya2VkIGFzIHN1Y2gsIGFuZCBtdXN0IG5vdCBiZVxyXG4gKiBtaXNyZXByZXNlbnRlZCBhcyBiZWluZyB0aGUgb3JpZ2luYWwgc29mdHdhcmUuXHJcbiAqIDMuIFRoaXMgbm90aWNlIG1heSBub3QgYmUgcmVtb3ZlZCBvciBhbHRlcmVkIGZyb20gYW55IHNvdXJjZSBkaXN0cmlidXRpb24uXHJcbiAqL1xyXG5cclxuaW1wb3J0IHsgYjJfZXBzaWxvbiwgYjJfcGksIGIyQXNzZXJ0IH0gZnJvbSAnLi9iMlNldHRpbmdzJztcclxuXHJcbmV4cG9ydCBjb25zdCBiMl9waV9vdmVyXzE4MDogbnVtYmVyID0gYjJfcGkgLyAxODA7XHJcbmV4cG9ydCBjb25zdCBiMl8xODBfb3Zlcl9waTogbnVtYmVyID0gMTgwIC8gYjJfcGk7XHJcbmV4cG9ydCBjb25zdCBiMl90d29fcGk6IG51bWJlciA9IDIgKiBiMl9waTtcclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMkFicyh4OiBudW1iZXIpIHtcclxuICByZXR1cm4geCA8IDAgPyAteCA6IHg7XHJcbn1cclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMkFic0ludCh4OiBudW1iZXIpIHtcclxuICByZXR1cm4geCA+PSAwID8geCA6IC14O1xyXG59XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJNaW4oYTogbnVtYmVyLCBiOiBudW1iZXIpOiBudW1iZXIge1xyXG4gIHJldHVybiBhIDwgYiA/IGEgOiBiO1xyXG59XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJNYXgoYTogbnVtYmVyLCBiOiBudW1iZXIpOiBudW1iZXIge1xyXG4gIHJldHVybiBhID4gYiA/IGEgOiBiO1xyXG59XHJcblxyXG4vLyBzZXBhcmF0ZSBmb3IgU21pIHR5cGVcclxuZXhwb3J0IGZ1bmN0aW9uIGIyTWluSW50KGE6IG51bWJlciwgYjogbnVtYmVyKTogbnVtYmVyIHtcclxuICByZXR1cm4gYSA8IGIgPyBhIDogYjtcclxufVxyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyTWF4SW50KGE6IG51bWJlciwgYjogbnVtYmVyKTogbnVtYmVyIHtcclxuICByZXR1cm4gYSA+IGIgPyBhIDogYjtcclxufVxyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyQ2xhbXAoYTogbnVtYmVyLCBsbzogbnVtYmVyLCBoaTogbnVtYmVyKTogbnVtYmVyIHtcclxuICByZXR1cm4gYSA8IGxvID8gbG8gOiBhID4gaGkgPyBoaSA6IGE7XHJcbn1cclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMkNsYW1wSW50KGE6IG51bWJlciwgbG86IG51bWJlciwgaGk6IG51bWJlcik6IG51bWJlciB7XHJcbiAgcmV0dXJuIGEgPCBsbyA/IGxvIDogYSA+IGhpID8gaGkgOiBhO1xyXG59XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJTd2FwPFQ+KGE6IFRbXSwgYjogVFtdKTogdm9pZCB7XHJcbiAgISFCMl9ERUJVRyAmJiBiMkFzc2VydChmYWxzZSk7XHJcbiAgY29uc3QgdG1wOiBUID0gYVswXTtcclxuICBhWzBdID0gYlswXTtcclxuICBiWzBdID0gdG1wO1xyXG59XHJcblxyXG4vLy8gVGhpcyBmdW5jdGlvbiBpcyB1c2VkIHRvIGVuc3VyZSB0aGF0IGEgZmxvYXRpbmcgcG9pbnQgbnVtYmVyIGlzXHJcbi8vLyBub3QgYSBOYU4gb3IgaW5maW5pdHkuXHJcbmV4cG9ydCBjb25zdCBiMklzVmFsaWQgPSBpc0Zpbml0ZTtcclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMlNxKG46IG51bWJlcik6IG51bWJlciB7XHJcbiAgcmV0dXJuIG4gKiBuO1xyXG59XHJcblxyXG4vLy8gVGhpcyBpcyBhIGFwcHJveGltYXRlIHlldCBmYXN0IGludmVyc2Ugc3F1YXJlLXJvb3QuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMkludlNxcnQobjogbnVtYmVyKTogbnVtYmVyIHtcclxuICByZXR1cm4gMS4wIC8gTWF0aC5zcXJ0KG4pO1xyXG59XHJcblxyXG5leHBvcnQgY29uc3QgYjJTcXJ0ID0gTWF0aC5zcXJ0O1xyXG5cclxuZXhwb3J0IGNvbnN0IGIyUG93ID0gTWF0aC5wb3c7XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJEZWdUb1JhZChkZWdyZWVzOiBudW1iZXIpOiBudW1iZXIge1xyXG4gIHJldHVybiBkZWdyZWVzICogYjJfcGlfb3Zlcl8xODA7XHJcbn1cclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMlJhZFRvRGVnKHJhZGlhbnM6IG51bWJlcik6IG51bWJlciB7XHJcbiAgcmV0dXJuIHJhZGlhbnMgKiBiMl8xODBfb3Zlcl9waTtcclxufVxyXG5cclxuZXhwb3J0IGNvbnN0IGIyU2luID0gTWF0aC5zaW47XHJcbmV4cG9ydCBjb25zdCBiMkNvcyA9IE1hdGguY29zO1xyXG5leHBvcnQgY29uc3QgYjJBY29zID0gTWF0aC5hY29zO1xyXG5leHBvcnQgY29uc3QgYjJBc2luID0gTWF0aC5hc2luO1xyXG5leHBvcnQgY29uc3QgYjJBdGFuMiA9IE1hdGguYXRhbjI7XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gYjJOZXh0UG93ZXJPZlR3byh4OiBudW1iZXIpOiBudW1iZXIge1xyXG4gIHggfD0gKHggPj4gMSkgJiAweDdmZmZmZmZmO1xyXG4gIHggfD0gKHggPj4gMikgJiAweDNmZmZmZmZmO1xyXG4gIHggfD0gKHggPj4gNCkgJiAweDBmZmZmZmZmO1xyXG4gIHggfD0gKHggPj4gOCkgJiAweDAwZmZmZmZmO1xyXG4gIHggfD0gKHggPj4gMTYpICYgMHgwMDAwZmZmZjtcclxuICByZXR1cm4geCArIDE7XHJcbn1cclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMklzUG93ZXJPZlR3byh4OiBudW1iZXIpOiBib29sZWFuIHtcclxuICByZXR1cm4geCA+IDAgJiYgKHggJiAoeCAtIDEpKSA9PT0gMDtcclxufVxyXG5cclxuZXhwb3J0IGZ1bmN0aW9uIGIyUmFuZG9tKCk6IG51bWJlciB7XHJcbiAgcmV0dXJuIE1hdGgucmFuZG9tKCkgKiAyLjAgLSAxLjA7XHJcbn1cclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBiMlJhbmRvbVJhbmdlKGxvOiBudW1iZXIsIGhpOiBudW1iZXIpOiBudW1iZXIge1xyXG4gIHJldHVybiAoaGkgLSBsbykgKiBNYXRoLnJhbmRvbSgpICsgbG87XHJcbn1cclxuXHJcbmV4cG9ydCBpbnRlcmZhY2UgWFkge1xyXG4gIHg6IG51bWJlcjtcclxuICB5OiBudW1iZXI7XHJcbn1cclxuXHJcbi8vLyBBIDJEIGNvbHVtbiB2ZWN0b3IuXHJcbmV4cG9ydCBjbGFzcyBiMlZlYzIgaW1wbGVtZW50cyBYWSB7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFpFUk86IFJlYWRvbmx5PGIyVmVjMj4gPSBuZXcgYjJWZWMyKDAsIDApO1xyXG4gIHN0YXRpYyByZWFkb25seSBVTklUWDogUmVhZG9ubHk8YjJWZWMyPiA9IG5ldyBiMlZlYzIoMSwgMCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFVOSVRZOiBSZWFkb25seTxiMlZlYzI+ID0gbmV3IGIyVmVjMigwLCAxKTtcclxuXHJcbiAgc3RhdGljIHJlYWRvbmx5IHNfdDA6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuICBzdGF0aWMgcmVhZG9ubHkgc190MTogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHN0YXRpYyByZWFkb25seSBzX3QyOiBiMlZlYzIgPSBuZXcgYjJWZWMyKCk7XHJcbiAgc3RhdGljIHJlYWRvbmx5IHNfdDM6IGIyVmVjMiA9IG5ldyBiMlZlYzIoKTtcclxuXHJcbiAgeCA9IE5hTjtcclxuICB5ID0gTmFOO1xyXG5cclxuICBjb25zdHJ1Y3Rvcih4ID0gMC4wLCB5ID0gMC4wKSB7XHJcbiAgICB0aGlzLnggPSB4O1xyXG4gICAgdGhpcy55ID0geTtcclxuICB9XHJcblxyXG4gIENsb25lKCk6IGIyVmVjMiB7XHJcbiAgICByZXR1cm4gbmV3IGIyVmVjMih0aGlzLngsIHRoaXMueSk7XHJcbiAgfVxyXG5cclxuICBTZXRaZXJvKCk6IHRoaXMge1xyXG4gICAgdGhpcy54ID0gMC4wO1xyXG4gICAgdGhpcy55ID0gMC4wO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZXQoeDogbnVtYmVyLCB5OiBudW1iZXIpOiB0aGlzIHtcclxuICAgIHRoaXMueCA9IHg7XHJcbiAgICB0aGlzLnkgPSB5O1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBDb3B5KG90aGVyOiBYWSk6IHRoaXMge1xyXG4gICAgdGhpcy54ID0gb3RoZXIueDtcclxuICAgIHRoaXMueSA9IG90aGVyLnk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZBZGQodjogWFkpOiB0aGlzIHtcclxuICAgIHRoaXMueCArPSB2Lng7XHJcbiAgICB0aGlzLnkgKz0gdi55O1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZWxmQWRkWFkoeDogbnVtYmVyLCB5OiBudW1iZXIpOiB0aGlzIHtcclxuICAgIHRoaXMueCArPSB4O1xyXG4gICAgdGhpcy55ICs9IHk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZTdWIodjogWFkpOiB0aGlzIHtcclxuICAgIHRoaXMueCAtPSB2Lng7XHJcbiAgICB0aGlzLnkgLT0gdi55O1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZWxmU3ViWFkoeDogbnVtYmVyLCB5OiBudW1iZXIpOiB0aGlzIHtcclxuICAgIHRoaXMueCAtPSB4O1xyXG4gICAgdGhpcy55IC09IHk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZNdWwoczogbnVtYmVyKTogdGhpcyB7XHJcbiAgICB0aGlzLnggKj0gcztcclxuICAgIHRoaXMueSAqPSBzO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZWxmTXVsQWRkKHM6IG51bWJlciwgdjogWFkpOiB0aGlzIHtcclxuICAgIHRoaXMueCArPSBzICogdi54O1xyXG4gICAgdGhpcy55ICs9IHMgKiB2Lnk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZNdWxTdWIoczogbnVtYmVyLCB2OiBYWSk6IHRoaXMge1xyXG4gICAgdGhpcy54IC09IHMgKiB2Lng7XHJcbiAgICB0aGlzLnkgLT0gcyAqIHYueTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgRG90KHY6IFhZKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLnggKiB2LnggKyB0aGlzLnkgKiB2Lnk7XHJcbiAgfVxyXG5cclxuICBDcm9zcyh2OiBYWSk6IG51bWJlciB7XHJcbiAgICByZXR1cm4gdGhpcy54ICogdi55IC0gdGhpcy55ICogdi54O1xyXG4gIH1cclxuXHJcbiAgTGVuZ3RoKCk6IG51bWJlciB7XHJcbiAgICBjb25zdCB4OiBudW1iZXIgPSB0aGlzLngsXHJcbiAgICAgIHk6IG51bWJlciA9IHRoaXMueTtcclxuICAgIHJldHVybiBNYXRoLnNxcnQoeCAqIHggKyB5ICogeSk7XHJcbiAgfVxyXG5cclxuICBMZW5ndGhTcXVhcmVkKCk6IG51bWJlciB7XHJcbiAgICBjb25zdCB4OiBudW1iZXIgPSB0aGlzLngsXHJcbiAgICAgIHk6IG51bWJlciA9IHRoaXMueTtcclxuICAgIHJldHVybiB4ICogeCArIHkgKiB5O1xyXG4gIH1cclxuXHJcbiAgTm9ybWFsaXplKCk6IG51bWJlciB7XHJcbiAgICBjb25zdCBsZW5ndGg6IG51bWJlciA9IHRoaXMuTGVuZ3RoKCk7XHJcbiAgICBpZiAobGVuZ3RoID49IGIyX2Vwc2lsb24pIHtcclxuICAgICAgY29uc3QgaW52X2xlbmd0aDogbnVtYmVyID0gMSAvIGxlbmd0aDtcclxuICAgICAgdGhpcy54ICo9IGludl9sZW5ndGg7XHJcbiAgICAgIHRoaXMueSAqPSBpbnZfbGVuZ3RoO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIGxlbmd0aDtcclxuICB9XHJcblxyXG4gIFNlbGZOb3JtYWxpemUoKTogdGhpcyB7XHJcbiAgICBjb25zdCBsZW5ndGg6IG51bWJlciA9IHRoaXMuTGVuZ3RoKCk7XHJcbiAgICBpZiAobGVuZ3RoID49IGIyX2Vwc2lsb24pIHtcclxuICAgICAgY29uc3QgaW52X2xlbmd0aDogbnVtYmVyID0gMSAvIGxlbmd0aDtcclxuICAgICAgdGhpcy54ICo9IGludl9sZW5ndGg7XHJcbiAgICAgIHRoaXMueSAqPSBpbnZfbGVuZ3RoO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZWxmUm90YXRlKHJhZGlhbnM6IG51bWJlcik6IHRoaXMge1xyXG4gICAgY29uc3QgYzogbnVtYmVyID0gTWF0aC5jb3MocmFkaWFucyk7XHJcbiAgICBjb25zdCBzOiBudW1iZXIgPSBNYXRoLnNpbihyYWRpYW5zKTtcclxuICAgIGNvbnN0IHg6IG51bWJlciA9IHRoaXMueDtcclxuICAgIHRoaXMueCA9IGMgKiB4IC0gcyAqIHRoaXMueTtcclxuICAgIHRoaXMueSA9IHMgKiB4ICsgYyAqIHRoaXMueTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2VsZlJvdGF0ZUNvc1NpbihjOiBudW1iZXIsIHM6IG51bWJlcik6IHRoaXMge1xyXG4gICAgY29uc3QgeDogbnVtYmVyID0gdGhpcy54O1xyXG4gICAgdGhpcy54ID0gYyAqIHggLSBzICogdGhpcy55O1xyXG4gICAgdGhpcy55ID0gcyAqIHggKyBjICogdGhpcy55O1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBJc1ZhbGlkKCk6IGJvb2xlYW4ge1xyXG4gICAgcmV0dXJuIGlzRmluaXRlKHRoaXMueCkgJiYgaXNGaW5pdGUodGhpcy55KTtcclxuICB9XHJcblxyXG4gIFNlbGZDcm9zc1ZTKHM6IG51bWJlcik6IHRoaXMge1xyXG4gICAgY29uc3QgeDogbnVtYmVyID0gdGhpcy54O1xyXG4gICAgdGhpcy54ID0gcyAqIHRoaXMueTtcclxuICAgIHRoaXMueSA9IC1zICogeDtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2VsZkNyb3NzU1YoczogbnVtYmVyKTogdGhpcyB7XHJcbiAgICBjb25zdCB4OiBudW1iZXIgPSB0aGlzLng7XHJcbiAgICB0aGlzLnggPSAtcyAqIHRoaXMueTtcclxuICAgIHRoaXMueSA9IHMgKiB4O1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZWxmTWluVih2OiBYWSk6IHRoaXMge1xyXG4gICAgdGhpcy54ID0gYjJNaW4odGhpcy54LCB2LngpO1xyXG4gICAgdGhpcy55ID0gYjJNaW4odGhpcy55LCB2LnkpO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZWxmTWF4Vih2OiBYWSk6IHRoaXMge1xyXG4gICAgdGhpcy54ID0gYjJNYXgodGhpcy54LCB2LngpO1xyXG4gICAgdGhpcy55ID0gYjJNYXgodGhpcy55LCB2LnkpO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZWxmQWJzKCk6IHRoaXMge1xyXG4gICAgdGhpcy54ID0gYjJBYnModGhpcy54KTtcclxuICAgIHRoaXMueSA9IGIyQWJzKHRoaXMueSk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZOZWcoKTogdGhpcyB7XHJcbiAgICB0aGlzLnggPSAtdGhpcy54O1xyXG4gICAgdGhpcy55ID0gLXRoaXMueTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2VsZlNrZXcoKTogdGhpcyB7XHJcbiAgICBjb25zdCB4OiBudW1iZXIgPSB0aGlzLng7XHJcbiAgICB0aGlzLnggPSAtdGhpcy55O1xyXG4gICAgdGhpcy55ID0geDtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE1ha2VBcnJheShsZW5ndGg6IG51bWJlcik6IGIyVmVjMltdIHtcclxuICAgIGNvbnN0IGFyciA9IG5ldyBBcnJheTxiMlZlYzI+KGxlbmd0aCk7XHJcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IGxlbmd0aDsgKytpKSB7XHJcbiAgICAgIGFycltpXSA9IG5ldyBiMlZlYzIoKTtcclxuICAgIH1cclxuICAgIHJldHVybiBhcnI7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgQWJzVjxUIGV4dGVuZHMgWFk+KHY6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIG91dC54ID0gYjJBYnModi54KTtcclxuICAgIG91dC55ID0gYjJBYnModi55KTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTWluVjxUIGV4dGVuZHMgWFk+KGE6IFhZLCBiOiBYWSwgb3V0OiBUKTogVCB7XHJcbiAgICBvdXQueCA9IGIyTWluKGEueCwgYi54KTtcclxuICAgIG91dC55ID0gYjJNaW4oYS55LCBiLnkpO1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBNYXhWPFQgZXh0ZW5kcyBYWT4oYTogWFksIGI6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIG91dC54ID0gYjJNYXgoYS54LCBiLngpO1xyXG4gICAgb3V0LnkgPSBiMk1heChhLnksIGIueSk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIENsYW1wVjxUIGV4dGVuZHMgWFk+KHY6IFhZLCBsbzogWFksIGhpOiBYWSwgb3V0OiBUKTogVCB7XHJcbiAgICBvdXQueCA9IGIyQ2xhbXAodi54LCBsby54LCBoaS54KTtcclxuICAgIG91dC55ID0gYjJDbGFtcCh2LnksIGxvLnksIGhpLnkpO1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBSb3RhdGVWPFQgZXh0ZW5kcyBYWT4odjogWFksIHJhZGlhbnM6IG51bWJlciwgb3V0OiBUKTogVCB7XHJcbiAgICBjb25zdCB2X3ggPSB2Lng7XHJcbiAgICBjb25zdCB2X3kgPSB2Lnk7XHJcbiAgICBjb25zdCBjID0gTWF0aC5jb3MocmFkaWFucyk7XHJcbiAgICBjb25zdCBzID0gTWF0aC5zaW4ocmFkaWFucyk7XHJcbiAgICBvdXQueCA9IGMgKiB2X3ggLSBzICogdl95O1xyXG4gICAgb3V0LnkgPSBzICogdl94ICsgYyAqIHZfeTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgRG90VlYoYTogWFksIGI6IFhZKTogbnVtYmVyIHtcclxuICAgIHJldHVybiBhLnggKiBiLnggKyBhLnkgKiBiLnk7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgQ3Jvc3NWVihhOiBYWSwgYjogWFkpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIGEueCAqIGIueSAtIGEueSAqIGIueDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBDcm9zc1ZTPFQgZXh0ZW5kcyBYWT4odjogWFksIHM6IG51bWJlciwgb3V0OiBUKTogVCB7XHJcbiAgICBjb25zdCB2X3ggPSB2Lng7XHJcbiAgICBvdXQueCA9IHMgKiB2Lnk7XHJcbiAgICBvdXQueSA9IC1zICogdl94O1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBDcm9zc1ZPbmU8VCBleHRlbmRzIFhZPih2OiBYWSwgb3V0OiBUKTogVCB7XHJcbiAgICBjb25zdCB2X3ggPSB2Lng7XHJcbiAgICBvdXQueCA9IHYueTtcclxuICAgIG91dC55ID0gLXZfeDtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgQ3Jvc3NTVjxUIGV4dGVuZHMgWFk+KHM6IG51bWJlciwgdjogWFksIG91dDogVCk6IFQge1xyXG4gICAgY29uc3Qgdl94ID0gdi54O1xyXG4gICAgb3V0LnggPSAtcyAqIHYueTtcclxuICAgIG91dC55ID0gcyAqIHZfeDtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgQ3Jvc3NPbmVWPFQgZXh0ZW5kcyBYWT4odjogWFksIG91dDogVCk6IFQge1xyXG4gICAgY29uc3Qgdl94ID0gdi54O1xyXG4gICAgb3V0LnggPSAtdi55O1xyXG4gICAgb3V0LnkgPSB2X3g7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIEFkZFZWPFQgZXh0ZW5kcyBYWT4oYTogWFksIGI6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIG91dC54ID0gYS54ICsgYi54O1xyXG4gICAgb3V0LnkgPSBhLnkgKyBiLnk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIFN1YlZWPFQgZXh0ZW5kcyBYWT4oYTogWFksIGI6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIG91dC54ID0gYS54IC0gYi54O1xyXG4gICAgb3V0LnkgPSBhLnkgLSBiLnk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bFNWPFQgZXh0ZW5kcyBYWT4oczogbnVtYmVyLCB2OiBYWSwgb3V0OiBUKTogVCB7XHJcbiAgICBvdXQueCA9IHYueCAqIHM7XHJcbiAgICBvdXQueSA9IHYueSAqIHM7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bFZTPFQgZXh0ZW5kcyBYWT4odjogWFksIHM6IG51bWJlciwgb3V0OiBUKTogVCB7XHJcbiAgICBvdXQueCA9IHYueCAqIHM7XHJcbiAgICBvdXQueSA9IHYueSAqIHM7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIEFkZFZNdWxTVjxUIGV4dGVuZHMgWFk+KGE6IFhZLCBzOiBudW1iZXIsIGI6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIG91dC54ID0gYS54ICsgcyAqIGIueDtcclxuICAgIG91dC55ID0gYS55ICsgcyAqIGIueTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgU3ViVk11bFNWPFQgZXh0ZW5kcyBYWT4oYTogWFksIHM6IG51bWJlciwgYjogWFksIG91dDogVCk6IFQge1xyXG4gICAgb3V0LnggPSBhLnggLSBzICogYi54O1xyXG4gICAgb3V0LnkgPSBhLnkgLSBzICogYi55O1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBBZGRWQ3Jvc3NTVjxUIGV4dGVuZHMgWFk+KGE6IFhZLCBzOiBudW1iZXIsIHY6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIGNvbnN0IHZfeDogbnVtYmVyID0gdi54O1xyXG4gICAgb3V0LnggPSBhLnggLSBzICogdi55O1xyXG4gICAgb3V0LnkgPSBhLnkgKyBzICogdl94O1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBNaWRWVjxUIGV4dGVuZHMgWFk+KGE6IFhZLCBiOiBYWSwgb3V0OiBUKTogVCB7XHJcbiAgICBvdXQueCA9IChhLnggKyBiLngpICogMC41O1xyXG4gICAgb3V0LnkgPSAoYS55ICsgYi55KSAqIDAuNTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgRXh0VlY8VCBleHRlbmRzIFhZPihhOiBYWSwgYjogWFksIG91dDogVCk6IFQge1xyXG4gICAgb3V0LnggPSAoYi54IC0gYS54KSAqIDAuNTtcclxuICAgIG91dC55ID0gKGIueSAtIGEueSkgKiAwLjU7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIElzRXF1YWxUb1YoYTogWFksIGI6IFhZKTogYm9vbGVhbiB7XHJcbiAgICByZXR1cm4gYS54ID09PSBiLnggJiYgYS55ID09PSBiLnk7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgRGlzdGFuY2VWVihhOiBYWSwgYjogWFkpOiBudW1iZXIge1xyXG4gICAgY29uc3QgY194OiBudW1iZXIgPSBhLnggLSBiLng7XHJcbiAgICBjb25zdCBjX3k6IG51bWJlciA9IGEueSAtIGIueTtcclxuICAgIHJldHVybiBNYXRoLnNxcnQoY194ICogY194ICsgY195ICogY195KTtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBEaXN0YW5jZVNxdWFyZWRWVihhOiBYWSwgYjogWFkpOiBudW1iZXIge1xyXG4gICAgY29uc3QgY194OiBudW1iZXIgPSBhLnggLSBiLng7XHJcbiAgICBjb25zdCBjX3k6IG51bWJlciA9IGEueSAtIGIueTtcclxuICAgIHJldHVybiBjX3ggKiBjX3ggKyBjX3kgKiBjX3k7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTmVnVjxUIGV4dGVuZHMgWFk+KHY6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIG91dC54ID0gLXYueDtcclxuICAgIG91dC55ID0gLXYueTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG59XHJcblxyXG5leHBvcnQgY29uc3QgYjJWZWMyX3plcm86IFJlYWRvbmx5PGIyVmVjMj4gPSBuZXcgYjJWZWMyKDAsIDApO1xyXG5cclxuZXhwb3J0IGludGVyZmFjZSBYWVogZXh0ZW5kcyBYWSB7XHJcbiAgejogbnVtYmVyO1xyXG59XHJcblxyXG4vLy8gQSAyRCBjb2x1bW4gdmVjdG9yIHdpdGggMyBlbGVtZW50cy5cclxuZXhwb3J0IGNsYXNzIGIyVmVjMyBpbXBsZW1lbnRzIFhZWiB7XHJcbiAgc3RhdGljIHJlYWRvbmx5IFpFUk86IFJlYWRvbmx5PGIyVmVjMz4gPSBuZXcgYjJWZWMzKDAsIDAsIDApO1xyXG5cclxuICBzdGF0aWMgcmVhZG9ubHkgc190MDogYjJWZWMzID0gbmV3IGIyVmVjMygpO1xyXG5cclxuICB4ID0gTmFOO1xyXG4gIHkgPSBOYU47XHJcbiAgeiA9IE5hTjtcclxuXHJcbiAgY29uc3RydWN0b3IoeCA9IDAuMCwgeSA9IDAuMCwgeiA9IDAuMCkge1xyXG4gICAgdGhpcy54ID0geDtcclxuICAgIHRoaXMueSA9IHk7XHJcbiAgICB0aGlzLnogPSB6O1xyXG4gIH1cclxuXHJcbiAgQ2xvbmUoKTogYjJWZWMzIHtcclxuICAgIHJldHVybiBuZXcgYjJWZWMzKHRoaXMueCwgdGhpcy55LCB0aGlzLnopO1xyXG4gIH1cclxuXHJcbiAgU2V0WmVybygpOiB0aGlzIHtcclxuICAgIHRoaXMueCA9IDA7XHJcbiAgICB0aGlzLnkgPSAwO1xyXG4gICAgdGhpcy56ID0gMDtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2V0WFlaKHg6IG51bWJlciwgeTogbnVtYmVyLCB6OiBudW1iZXIpOiB0aGlzIHtcclxuICAgIHRoaXMueCA9IHg7XHJcbiAgICB0aGlzLnkgPSB5O1xyXG4gICAgdGhpcy56ID0gejtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgQ29weShvdGhlcjogWFlaKTogdGhpcyB7XHJcbiAgICB0aGlzLnggPSBvdGhlci54O1xyXG4gICAgdGhpcy55ID0gb3RoZXIueTtcclxuICAgIHRoaXMueiA9IG90aGVyLno7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZOZWcoKTogdGhpcyB7XHJcbiAgICB0aGlzLnggPSAtdGhpcy54O1xyXG4gICAgdGhpcy55ID0gLXRoaXMueTtcclxuICAgIHRoaXMueiA9IC10aGlzLno7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZBZGQodjogWFlaKTogdGhpcyB7XHJcbiAgICB0aGlzLnggKz0gdi54O1xyXG4gICAgdGhpcy55ICs9IHYueTtcclxuICAgIHRoaXMueiArPSB2Lno7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZBZGRYWVooeDogbnVtYmVyLCB5OiBudW1iZXIsIHo6IG51bWJlcik6IHRoaXMge1xyXG4gICAgdGhpcy54ICs9IHg7XHJcbiAgICB0aGlzLnkgKz0geTtcclxuICAgIHRoaXMueiArPSB6O1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZWxmU3ViKHY6IFhZWik6IHRoaXMge1xyXG4gICAgdGhpcy54IC09IHYueDtcclxuICAgIHRoaXMueSAtPSB2Lnk7XHJcbiAgICB0aGlzLnogLT0gdi56O1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZWxmU3ViWFlaKHg6IG51bWJlciwgeTogbnVtYmVyLCB6OiBudW1iZXIpOiB0aGlzIHtcclxuICAgIHRoaXMueCAtPSB4O1xyXG4gICAgdGhpcy55IC09IHk7XHJcbiAgICB0aGlzLnogLT0gejtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2VsZk11bChzOiBudW1iZXIpOiB0aGlzIHtcclxuICAgIHRoaXMueCAqPSBzO1xyXG4gICAgdGhpcy55ICo9IHM7XHJcbiAgICB0aGlzLnogKj0gcztcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIERvdFYzVjMoYTogWFlaLCBiOiBYWVopOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIGEueCAqIGIueCArIGEueSAqIGIueSArIGEueiAqIGIuejtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBDcm9zc1YzVjM8VCBleHRlbmRzIFhZWj4oYTogWFlaLCBiOiBYWVosIG91dDogVCk6IFQge1xyXG4gICAgY29uc3QgYV94OiBudW1iZXIgPSBhLngsXHJcbiAgICAgIGFfeSA9IGEueSxcclxuICAgICAgYV96ID0gYS56O1xyXG4gICAgY29uc3QgYl94OiBudW1iZXIgPSBiLngsXHJcbiAgICAgIGJfeSA9IGIueSxcclxuICAgICAgYl96ID0gYi56O1xyXG4gICAgb3V0LnggPSBhX3kgKiBiX3ogLSBhX3ogKiBiX3k7XHJcbiAgICBvdXQueSA9IGFfeiAqIGJfeCAtIGFfeCAqIGJfejtcclxuICAgIG91dC56ID0gYV94ICogYl95IC0gYV95ICogYl94O1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcbn1cclxuXHJcbi8vLyBBIDItYnktMiBtYXRyaXguIFN0b3JlZCBpbiBjb2x1bW4tbWFqb3Igb3JkZXIuXHJcbmV4cG9ydCBjbGFzcyBiMk1hdDIyIHtcclxuICBzdGF0aWMgcmVhZG9ubHkgSURFTlRJVFk6IFJlYWRvbmx5PGIyTWF0MjI+ID0gbmV3IGIyTWF0MjIoKTtcclxuXHJcbiAgcmVhZG9ubHkgZXg6IGIyVmVjMiA9IG5ldyBiMlZlYzIoMSwgMCk7XHJcbiAgcmVhZG9ubHkgZXk6IGIyVmVjMiA9IG5ldyBiMlZlYzIoMCwgMSk7XHJcblxyXG4gIENsb25lKCk6IGIyTWF0MjIge1xyXG4gICAgcmV0dXJuIG5ldyBiMk1hdDIyKCkuQ29weSh0aGlzKTtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBGcm9tVlYoYzE6IFhZLCBjMjogWFkpOiBiMk1hdDIyIHtcclxuICAgIHJldHVybiBuZXcgYjJNYXQyMigpLlNldFZWKGMxLCBjMik7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgRnJvbVNTU1MocjFjMTogbnVtYmVyLCByMWMyOiBudW1iZXIsIHIyYzE6IG51bWJlciwgcjJjMjogbnVtYmVyKTogYjJNYXQyMiB7XHJcbiAgICByZXR1cm4gbmV3IGIyTWF0MjIoKS5TZXRTU1NTKHIxYzEsIHIxYzIsIHIyYzEsIHIyYzIpO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIEZyb21BbmdsZShyYWRpYW5zOiBudW1iZXIpOiBiMk1hdDIyIHtcclxuICAgIHJldHVybiBuZXcgYjJNYXQyMigpLlNldEFuZ2xlKHJhZGlhbnMpO1xyXG4gIH1cclxuXHJcbiAgU2V0U1NTUyhyMWMxOiBudW1iZXIsIHIxYzI6IG51bWJlciwgcjJjMTogbnVtYmVyLCByMmMyOiBudW1iZXIpOiB0aGlzIHtcclxuICAgIHRoaXMuZXguU2V0KHIxYzEsIHIyYzEpO1xyXG4gICAgdGhpcy5leS5TZXQocjFjMiwgcjJjMik7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNldFZWKGMxOiBYWSwgYzI6IFhZKTogdGhpcyB7XHJcbiAgICB0aGlzLmV4LkNvcHkoYzEpO1xyXG4gICAgdGhpcy5leS5Db3B5KGMyKTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2V0QW5nbGUocmFkaWFuczogbnVtYmVyKTogdGhpcyB7XHJcbiAgICBjb25zdCBjOiBudW1iZXIgPSBNYXRoLmNvcyhyYWRpYW5zKTtcclxuICAgIGNvbnN0IHM6IG51bWJlciA9IE1hdGguc2luKHJhZGlhbnMpO1xyXG4gICAgdGhpcy5leC5TZXQoYywgcyk7XHJcbiAgICB0aGlzLmV5LlNldCgtcywgYyk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIENvcHkob3RoZXI6IGIyTWF0MjIpOiB0aGlzIHtcclxuICAgIHRoaXMuZXguQ29weShvdGhlci5leCk7XHJcbiAgICB0aGlzLmV5LkNvcHkob3RoZXIuZXkpO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZXRJZGVudGl0eSgpOiB0aGlzIHtcclxuICAgIHRoaXMuZXguU2V0KDEsIDApO1xyXG4gICAgdGhpcy5leS5TZXQoMCwgMSk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNldFplcm8oKTogdGhpcyB7XHJcbiAgICB0aGlzLmV4LlNldFplcm8oKTtcclxuICAgIHRoaXMuZXkuU2V0WmVybygpO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBHZXRBbmdsZSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIE1hdGguYXRhbjIodGhpcy5leC55LCB0aGlzLmV4LngpO1xyXG4gIH1cclxuXHJcbiAgR2V0SW52ZXJzZShvdXQ6IGIyTWF0MjIpOiBiMk1hdDIyIHtcclxuICAgIGNvbnN0IGE6IG51bWJlciA9IHRoaXMuZXgueDtcclxuICAgIGNvbnN0IGI6IG51bWJlciA9IHRoaXMuZXkueDtcclxuICAgIGNvbnN0IGM6IG51bWJlciA9IHRoaXMuZXgueTtcclxuICAgIGNvbnN0IGQ6IG51bWJlciA9IHRoaXMuZXkueTtcclxuICAgIGxldCBkZXQ6IG51bWJlciA9IGEgKiBkIC0gYiAqIGM7XHJcbiAgICBpZiAoZGV0ICE9PSAwKSB7XHJcbiAgICAgIGRldCA9IDEgLyBkZXQ7XHJcbiAgICB9XHJcbiAgICBvdXQuZXgueCA9IGRldCAqIGQ7XHJcbiAgICBvdXQuZXkueCA9IC1kZXQgKiBiO1xyXG4gICAgb3V0LmV4LnkgPSAtZGV0ICogYztcclxuICAgIG91dC5leS55ID0gZGV0ICogYTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBTb2x2ZTxUIGV4dGVuZHMgWFk+KGJfeDogbnVtYmVyLCBiX3k6IG51bWJlciwgb3V0OiBUKTogVCB7XHJcbiAgICBjb25zdCBhMTEgPSB0aGlzLmV4LngsXHJcbiAgICAgIGExMiA9IHRoaXMuZXkueDtcclxuICAgIGNvbnN0IGEyMSA9IHRoaXMuZXgueSxcclxuICAgICAgYTIyID0gdGhpcy5leS55O1xyXG4gICAgbGV0IGRldCA9IGExMSAqIGEyMiAtIGExMiAqIGEyMTtcclxuICAgIGlmIChkZXQgIT09IDApIHtcclxuICAgICAgZGV0ID0gMS4wIC8gZGV0O1xyXG4gICAgfVxyXG4gICAgb3V0LnggPSBkZXQgKiAoYTIyICogYl94IC0gYTEyICogYl95KTtcclxuICAgIG91dC55ID0gZGV0ICogKGExMSAqIGJfeSAtIGEyMSAqIGJfeCk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgU2VsZkFicygpOiB0aGlzIHtcclxuICAgIHRoaXMuZXguU2VsZkFicygpO1xyXG4gICAgdGhpcy5leS5TZWxmQWJzKCk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZJbnYoKTogdGhpcyB7XHJcbiAgICB0aGlzLkdldEludmVyc2UodGhpcyk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZBZGRNKE06IGIyTWF0MjIpOiB0aGlzIHtcclxuICAgIHRoaXMuZXguU2VsZkFkZChNLmV4KTtcclxuICAgIHRoaXMuZXkuU2VsZkFkZChNLmV5KTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2VsZlN1Yk0oTTogYjJNYXQyMik6IHRoaXMge1xyXG4gICAgdGhpcy5leC5TZWxmU3ViKE0uZXgpO1xyXG4gICAgdGhpcy5leS5TZWxmU3ViKE0uZXkpO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgQWJzTShNOiBiMk1hdDIyLCBvdXQ6IGIyTWF0MjIpOiBiMk1hdDIyIHtcclxuICAgIGNvbnN0IE1fZXg6IGIyVmVjMiA9IE0uZXgsXHJcbiAgICAgIE1fZXk6IGIyVmVjMiA9IE0uZXk7XHJcbiAgICBvdXQuZXgueCA9IGIyQWJzKE1fZXgueCk7XHJcbiAgICBvdXQuZXgueSA9IGIyQWJzKE1fZXgueSk7XHJcbiAgICBvdXQuZXkueCA9IGIyQWJzKE1fZXkueCk7XHJcbiAgICBvdXQuZXkueSA9IGIyQWJzKE1fZXkueSk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bE1WPFQgZXh0ZW5kcyBYWT4oTTogYjJNYXQyMiwgdjogWFksIG91dDogVCk6IFQge1xyXG4gICAgY29uc3QgTV9leCA9IE0uZXg7XHJcbiAgICBjb25zdCBNX2V5ID0gTS5leTtcclxuICAgIGNvbnN0IHZfeCA9IHYueDtcclxuICAgIGNvbnN0IHZfeSA9IHYueTtcclxuICAgIG91dC54ID0gTV9leC54ICogdl94ICsgTV9leS54ICogdl95O1xyXG4gICAgb3V0LnkgPSBNX2V4LnkgKiB2X3ggKyBNX2V5LnkgKiB2X3k7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bFRNVjxUIGV4dGVuZHMgWFk+KE06IGIyTWF0MjIsIHY6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIGNvbnN0IE1fZXggPSBNLmV4O1xyXG4gICAgY29uc3QgTV9leSA9IE0uZXk7XHJcbiAgICBjb25zdCB2X3ggPSB2Lng7XHJcbiAgICBjb25zdCB2X3kgPSB2Lnk7XHJcbiAgICBvdXQueCA9IE1fZXgueCAqIHZfeCArIE1fZXgueSAqIHZfeTtcclxuICAgIG91dC55ID0gTV9leS54ICogdl94ICsgTV9leS55ICogdl95O1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBBZGRNTShBOiBiMk1hdDIyLCBCOiBiMk1hdDIyLCBvdXQ6IGIyTWF0MjIpOiBiMk1hdDIyIHtcclxuICAgIGNvbnN0IEFfZXggPSBBLmV4O1xyXG4gICAgY29uc3QgQV9leSA9IEEuZXk7XHJcbiAgICBjb25zdCBCX2V4ID0gQi5leDtcclxuICAgIGNvbnN0IEJfZXkgPSBCLmV5O1xyXG4gICAgb3V0LmV4LnggPSBBX2V4LnggKyBCX2V4Lng7XHJcbiAgICBvdXQuZXgueSA9IEFfZXgueSArIEJfZXgueTtcclxuICAgIG91dC5leS54ID0gQV9leS54ICsgQl9leS54O1xyXG4gICAgb3V0LmV5LnkgPSBBX2V5LnkgKyBCX2V5Lnk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bE1NKEE6IGIyTWF0MjIsIEI6IGIyTWF0MjIsIG91dDogYjJNYXQyMik6IGIyTWF0MjIge1xyXG4gICAgY29uc3QgQV9leF94ID0gQS5leC54O1xyXG4gICAgY29uc3QgQV9leF95ID0gQS5leC55O1xyXG4gICAgY29uc3QgQV9leV94ID0gQS5leS54O1xyXG4gICAgY29uc3QgQV9leV95ID0gQS5leS55O1xyXG4gICAgY29uc3QgQl9leF94ID0gQi5leC54O1xyXG4gICAgY29uc3QgQl9leF95ID0gQi5leC55O1xyXG4gICAgY29uc3QgQl9leV94ID0gQi5leS54O1xyXG4gICAgY29uc3QgQl9leV95ID0gQi5leS55O1xyXG4gICAgb3V0LmV4LnggPSBBX2V4X3ggKiBCX2V4X3ggKyBBX2V5X3ggKiBCX2V4X3k7XHJcbiAgICBvdXQuZXgueSA9IEFfZXhfeSAqIEJfZXhfeCArIEFfZXlfeSAqIEJfZXhfeTtcclxuICAgIG91dC5leS54ID0gQV9leF94ICogQl9leV94ICsgQV9leV94ICogQl9leV95O1xyXG4gICAgb3V0LmV5LnkgPSBBX2V4X3kgKiBCX2V5X3ggKyBBX2V5X3kgKiBCX2V5X3k7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bFRNTShBOiBiMk1hdDIyLCBCOiBiMk1hdDIyLCBvdXQ6IGIyTWF0MjIpOiBiMk1hdDIyIHtcclxuICAgIGNvbnN0IEFfZXhfeCA9IEEuZXgueDtcclxuICAgIGNvbnN0IEFfZXhfeSA9IEEuZXgueTtcclxuICAgIGNvbnN0IEFfZXlfeCA9IEEuZXkueDtcclxuICAgIGNvbnN0IEFfZXlfeSA9IEEuZXkueTtcclxuICAgIGNvbnN0IEJfZXhfeCA9IEIuZXgueDtcclxuICAgIGNvbnN0IEJfZXhfeSA9IEIuZXgueTtcclxuICAgIGNvbnN0IEJfZXlfeCA9IEIuZXkueDtcclxuICAgIGNvbnN0IEJfZXlfeSA9IEIuZXkueTtcclxuICAgIG91dC5leC54ID0gQV9leF94ICogQl9leF94ICsgQV9leF95ICogQl9leF95O1xyXG4gICAgb3V0LmV4LnkgPSBBX2V5X3ggKiBCX2V4X3ggKyBBX2V5X3kgKiBCX2V4X3k7XHJcbiAgICBvdXQuZXkueCA9IEFfZXhfeCAqIEJfZXlfeCArIEFfZXhfeSAqIEJfZXlfeTtcclxuICAgIG91dC5leS55ID0gQV9leV94ICogQl9leV94ICsgQV9leV95ICogQl9leV95O1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcbn1cclxuXHJcbi8vLyBBIDMtYnktMyBtYXRyaXguIFN0b3JlZCBpbiBjb2x1bW4tbWFqb3Igb3JkZXIuXHJcbmV4cG9ydCBjbGFzcyBiMk1hdDMzIHtcclxuICBzdGF0aWMgcmVhZG9ubHkgSURFTlRJVFk6IFJlYWRvbmx5PGIyTWF0MzM+ID0gbmV3IGIyTWF0MzMoKTtcclxuXHJcbiAgcmVhZG9ubHkgZXggPSBuZXcgYjJWZWMzKDEsIDAsIDApO1xyXG4gIHJlYWRvbmx5IGV5ID0gbmV3IGIyVmVjMygwLCAxLCAwKTtcclxuICByZWFkb25seSBleiA9IG5ldyBiMlZlYzMoMCwgMCwgMSk7XHJcblxyXG4gIENsb25lKCk6IGIyTWF0MzMge1xyXG4gICAgcmV0dXJuIG5ldyBiMk1hdDMzKCkuQ29weSh0aGlzKTtcclxuICB9XHJcblxyXG4gIFNldFZWVihjMTogWFlaLCBjMjogWFlaLCBjMzogWFlaKTogdGhpcyB7XHJcbiAgICB0aGlzLmV4LkNvcHkoYzEpO1xyXG4gICAgdGhpcy5leS5Db3B5KGMyKTtcclxuICAgIHRoaXMuZXouQ29weShjMyk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIENvcHkob3RoZXI6IGIyTWF0MzMpOiB0aGlzIHtcclxuICAgIHRoaXMuZXguQ29weShvdGhlci5leCk7XHJcbiAgICB0aGlzLmV5LkNvcHkob3RoZXIuZXkpO1xyXG4gICAgdGhpcy5lei5Db3B5KG90aGVyLmV6KTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2V0SWRlbnRpdHkoKTogdGhpcyB7XHJcbiAgICB0aGlzLmV4LlNldFhZWigxLCAwLCAwKTtcclxuICAgIHRoaXMuZXkuU2V0WFlaKDAsIDEsIDApO1xyXG4gICAgdGhpcy5lei5TZXRYWVooMCwgMCwgMSk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNldFplcm8oKTogdGhpcyB7XHJcbiAgICB0aGlzLmV4LlNldFplcm8oKTtcclxuICAgIHRoaXMuZXkuU2V0WmVybygpO1xyXG4gICAgdGhpcy5lei5TZXRaZXJvKCk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNlbGZBZGRNKE06IGIyTWF0MzMpOiB0aGlzIHtcclxuICAgIHRoaXMuZXguU2VsZkFkZChNLmV4KTtcclxuICAgIHRoaXMuZXkuU2VsZkFkZChNLmV5KTtcclxuICAgIHRoaXMuZXouU2VsZkFkZChNLmV6KTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU29sdmUzMzxUIGV4dGVuZHMgWFlaPihiX3g6IG51bWJlciwgYl95OiBudW1iZXIsIGJfejogbnVtYmVyLCBvdXQ6IFQpOiBUIHtcclxuICAgIGNvbnN0IGExMSA9IHRoaXMuZXgueDtcclxuICAgIGNvbnN0IGEyMSA9IHRoaXMuZXgueTtcclxuICAgIGNvbnN0IGEzMSA9IHRoaXMuZXguejtcclxuICAgIGNvbnN0IGExMiA9IHRoaXMuZXkueDtcclxuICAgIGNvbnN0IGEyMiA9IHRoaXMuZXkueTtcclxuICAgIGNvbnN0IGEzMiA9IHRoaXMuZXkuejtcclxuICAgIGNvbnN0IGExMyA9IHRoaXMuZXoueDtcclxuICAgIGNvbnN0IGEyMyA9IHRoaXMuZXoueTtcclxuICAgIGNvbnN0IGEzMyA9IHRoaXMuZXouejtcclxuICAgIGxldCBkZXQgPVxyXG4gICAgICBhMTEgKiAoYTIyICogYTMzIC0gYTMyICogYTIzKSArIGEyMSAqIChhMzIgKiBhMTMgLSBhMTIgKiBhMzMpICsgYTMxICogKGExMiAqIGEyMyAtIGEyMiAqIGExMyk7XHJcbiAgICBpZiAoZGV0ICE9PSAwKSB7XHJcbiAgICAgIGRldCA9IDEuMCAvIGRldDtcclxuICAgIH1cclxuICAgIG91dC54ID1cclxuICAgICAgZGV0ICpcclxuICAgICAgKGJfeCAqIChhMjIgKiBhMzMgLSBhMzIgKiBhMjMpICtcclxuICAgICAgICBiX3kgKiAoYTMyICogYTEzIC0gYTEyICogYTMzKSArXHJcbiAgICAgICAgYl96ICogKGExMiAqIGEyMyAtIGEyMiAqIGExMykpO1xyXG4gICAgb3V0LnkgPVxyXG4gICAgICBkZXQgKlxyXG4gICAgICAoYTExICogKGJfeSAqIGEzMyAtIGJfeiAqIGEyMykgK1xyXG4gICAgICAgIGEyMSAqIChiX3ogKiBhMTMgLSBiX3ggKiBhMzMpICtcclxuICAgICAgICBhMzEgKiAoYl94ICogYTIzIC0gYl95ICogYTEzKSk7XHJcbiAgICBvdXQueiA9XHJcbiAgICAgIGRldCAqXHJcbiAgICAgIChhMTEgKiAoYTIyICogYl96IC0gYTMyICogYl95KSArXHJcbiAgICAgICAgYTIxICogKGEzMiAqIGJfeCAtIGExMiAqIGJfeikgK1xyXG4gICAgICAgIGEzMSAqIChhMTIgKiBiX3kgLSBhMjIgKiBiX3gpKTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBTb2x2ZTIyPFQgZXh0ZW5kcyBYWT4oYl94OiBudW1iZXIsIGJfeTogbnVtYmVyLCBvdXQ6IFQpOiBUIHtcclxuICAgIGNvbnN0IGExMSA9IHRoaXMuZXgueDtcclxuICAgIGNvbnN0IGExMiA9IHRoaXMuZXkueDtcclxuICAgIGNvbnN0IGEyMSA9IHRoaXMuZXgueTtcclxuICAgIGNvbnN0IGEyMiA9IHRoaXMuZXkueTtcclxuICAgIGxldCBkZXQ6IG51bWJlciA9IGExMSAqIGEyMiAtIGExMiAqIGEyMTtcclxuICAgIGlmIChkZXQgIT09IDApIHtcclxuICAgICAgZGV0ID0gMS4wIC8gZGV0O1xyXG4gICAgfVxyXG4gICAgb3V0LnggPSBkZXQgKiAoYTIyICogYl94IC0gYTEyICogYl95KTtcclxuICAgIG91dC55ID0gZGV0ICogKGExMSAqIGJfeSAtIGEyMSAqIGJfeCk7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgR2V0SW52ZXJzZTIyKE06IGIyTWF0MzMpOiB2b2lkIHtcclxuICAgIGNvbnN0IGEgPSB0aGlzLmV4Lng7XHJcbiAgICBjb25zdCBiID0gdGhpcy5leS54O1xyXG4gICAgY29uc3QgYyA9IHRoaXMuZXgueTtcclxuICAgIGNvbnN0IGQgPSB0aGlzLmV5Lnk7XHJcblxyXG4gICAgbGV0IGRldCA9IGEgKiBkIC0gYiAqIGM7XHJcbiAgICBpZiAoZGV0ICE9PSAwKSB7XHJcbiAgICAgIGRldCA9IDEuMCAvIGRldDtcclxuICAgIH1cclxuXHJcbiAgICBNLmV4LnggPSBkZXQgKiBkO1xyXG4gICAgTS5leS54ID0gLWRldCAqIGI7XHJcbiAgICBNLmV4LnogPSAwO1xyXG4gICAgTS5leC55ID0gLWRldCAqIGM7XHJcbiAgICBNLmV5LnkgPSBkZXQgKiBhO1xyXG4gICAgTS5leS56ID0gMDtcclxuICAgIE0uZXoueCA9IDA7XHJcbiAgICBNLmV6LnkgPSAwO1xyXG4gICAgTS5lei56ID0gMDtcclxuICB9XHJcblxyXG4gIEdldFN5bUludmVyc2UzMyhNOiBiMk1hdDMzKTogdm9pZCB7XHJcbiAgICBsZXQgZGV0OiBudW1iZXIgPSBiMlZlYzMuRG90VjNWMyh0aGlzLmV4LCBiMlZlYzMuQ3Jvc3NWM1YzKHRoaXMuZXksIHRoaXMuZXosIGIyVmVjMy5zX3QwKSk7XHJcbiAgICBpZiAoZGV0ICE9PSAwKSB7XHJcbiAgICAgIGRldCA9IDEgLyBkZXQ7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgYTExOiBudW1iZXIgPSB0aGlzLmV4LngsXHJcbiAgICAgIGExMjogbnVtYmVyID0gdGhpcy5leS54LFxyXG4gICAgICBhMTM6IG51bWJlciA9IHRoaXMuZXoueDtcclxuICAgIGNvbnN0IGEyMjogbnVtYmVyID0gdGhpcy5leS55LFxyXG4gICAgICBhMjM6IG51bWJlciA9IHRoaXMuZXoueTtcclxuICAgIGNvbnN0IGEzMzogbnVtYmVyID0gdGhpcy5lei56O1xyXG5cclxuICAgIE0uZXgueCA9IGRldCAqIChhMjIgKiBhMzMgLSBhMjMgKiBhMjMpO1xyXG4gICAgTS5leC55ID0gZGV0ICogKGExMyAqIGEyMyAtIGExMiAqIGEzMyk7XHJcbiAgICBNLmV4LnogPSBkZXQgKiAoYTEyICogYTIzIC0gYTEzICogYTIyKTtcclxuXHJcbiAgICBNLmV5LnggPSBNLmV4Lnk7XHJcbiAgICBNLmV5LnkgPSBkZXQgKiAoYTExICogYTMzIC0gYTEzICogYTEzKTtcclxuICAgIE0uZXkueiA9IGRldCAqIChhMTMgKiBhMTIgLSBhMTEgKiBhMjMpO1xyXG5cclxuICAgIE0uZXoueCA9IE0uZXguejtcclxuICAgIE0uZXoueSA9IE0uZXkuejtcclxuICAgIE0uZXoueiA9IGRldCAqIChhMTEgKiBhMjIgLSBhMTIgKiBhMTIpO1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bE0zM1YzPFQgZXh0ZW5kcyBYWVo+KEE6IGIyTWF0MzMsIHY6IFhZWiwgb3V0OiBUKTogVCB7XHJcbiAgICBjb25zdCB2X3g6IG51bWJlciA9IHYueCxcclxuICAgICAgdl95OiBudW1iZXIgPSB2LnksXHJcbiAgICAgIHZfejogbnVtYmVyID0gdi56O1xyXG4gICAgb3V0LnggPSBBLmV4LnggKiB2X3ggKyBBLmV5LnggKiB2X3kgKyBBLmV6LnggKiB2X3o7XHJcbiAgICBvdXQueSA9IEEuZXgueSAqIHZfeCArIEEuZXkueSAqIHZfeSArIEEuZXoueSAqIHZfejtcclxuICAgIG91dC56ID0gQS5leC56ICogdl94ICsgQS5leS56ICogdl95ICsgQS5lei56ICogdl96O1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBNdWxNMzNYWVo8VCBleHRlbmRzIFhZWj4oQTogYjJNYXQzMywgeDogbnVtYmVyLCB5OiBudW1iZXIsIHo6IG51bWJlciwgb3V0OiBUKTogVCB7XHJcbiAgICBvdXQueCA9IEEuZXgueCAqIHggKyBBLmV5LnggKiB5ICsgQS5lei54ICogejtcclxuICAgIG91dC55ID0gQS5leC55ICogeCArIEEuZXkueSAqIHkgKyBBLmV6LnkgKiB6O1xyXG4gICAgb3V0LnogPSBBLmV4LnogKiB4ICsgQS5leS56ICogeSArIEEuZXoueiAqIHo7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bE0zM1YyPFQgZXh0ZW5kcyBYWT4oQTogYjJNYXQzMywgdjogWFksIG91dDogVCk6IFQge1xyXG4gICAgY29uc3Qgdl94OiBudW1iZXIgPSB2LngsXHJcbiAgICAgIHZfeTogbnVtYmVyID0gdi55O1xyXG4gICAgb3V0LnggPSBBLmV4LnggKiB2X3ggKyBBLmV5LnggKiB2X3k7XHJcbiAgICBvdXQueSA9IEEuZXgueSAqIHZfeCArIEEuZXkueSAqIHZfeTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTXVsTTMzWFk8VCBleHRlbmRzIFhZPihBOiBiMk1hdDMzLCB4OiBudW1iZXIsIHk6IG51bWJlciwgb3V0OiBUKTogVCB7XHJcbiAgICBvdXQueCA9IEEuZXgueCAqIHggKyBBLmV5LnggKiB5O1xyXG4gICAgb3V0LnkgPSBBLmV4LnkgKiB4ICsgQS5leS55ICogeTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gUm90YXRpb25cclxuZXhwb3J0IGNsYXNzIGIyUm90IHtcclxuICBzdGF0aWMgcmVhZG9ubHkgSURFTlRJVFk6IFJlYWRvbmx5PGIyUm90PiA9IG5ldyBiMlJvdCgpO1xyXG5cclxuICBzID0gTmFOO1xyXG4gIGMgPSBOYU47XHJcblxyXG4gIGNvbnN0cnVjdG9yKGFuZ2xlID0gMC4wKSB7XHJcbiAgICB0aGlzLnMgPSBNYXRoLnNpbihhbmdsZSk7XHJcbiAgICB0aGlzLmMgPSBNYXRoLmNvcyhhbmdsZSk7XHJcbiAgfVxyXG5cclxuICBDbG9uZSgpOiBiMlJvdCB7XHJcbiAgICByZXR1cm4gbmV3IGIyUm90KCkuQ29weSh0aGlzKTtcclxuICB9XHJcblxyXG4gIENvcHkob3RoZXI6IGIyUm90KTogdGhpcyB7XHJcbiAgICB0aGlzLnMgPSBvdGhlci5zO1xyXG4gICAgdGhpcy5jID0gb3RoZXIuYztcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2V0QW5nbGUoYW5nbGU6IG51bWJlcik6IHRoaXMge1xyXG4gICAgdGhpcy5zID0gTWF0aC5zaW4oYW5nbGUpO1xyXG4gICAgdGhpcy5jID0gTWF0aC5jb3MoYW5nbGUpO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZXRJZGVudGl0eSgpOiB0aGlzIHtcclxuICAgIHRoaXMucyA9IDA7XHJcbiAgICB0aGlzLmMgPSAxO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBHZXRBbmdsZSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIE1hdGguYXRhbjIodGhpcy5zLCB0aGlzLmMpO1xyXG4gIH1cclxuXHJcbiAgR2V0WEF4aXM8VCBleHRlbmRzIFhZPihvdXQ6IFQpOiBUIHtcclxuICAgIG91dC54ID0gdGhpcy5jO1xyXG4gICAgb3V0LnkgPSB0aGlzLnM7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgR2V0WUF4aXM8VCBleHRlbmRzIFhZPihvdXQ6IFQpOiBUIHtcclxuICAgIG91dC54ID0gLXRoaXMucztcclxuICAgIG91dC55ID0gdGhpcy5jO1xyXG4gICAgcmV0dXJuIG91dDtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBNdWxSUihxOiBiMlJvdCwgcjogYjJSb3QsIG91dDogYjJSb3QpOiBiMlJvdCB7XHJcbiAgICAvLyBbcWMgLXFzXSAqIFtyYyAtcnNdID0gW3FjKnJjLXFzKnJzIC1xYypycy1xcypyY11cclxuICAgIC8vIFtxcyAgcWNdICAgW3JzICByY10gICBbcXMqcmMrcWMqcnMgLXFzKnJzK3FjKnJjXVxyXG4gICAgLy8gcyA9IHFzICogcmMgKyBxYyAqIHJzXHJcbiAgICAvLyBjID0gcWMgKiByYyAtIHFzICogcnNcclxuICAgIGNvbnN0IHFfYzogbnVtYmVyID0gcS5jLFxyXG4gICAgICBxX3M6IG51bWJlciA9IHEucztcclxuICAgIGNvbnN0IHJfYzogbnVtYmVyID0gci5jLFxyXG4gICAgICByX3M6IG51bWJlciA9IHIucztcclxuICAgIG91dC5zID0gcV9zICogcl9jICsgcV9jICogcl9zO1xyXG4gICAgb3V0LmMgPSBxX2MgKiByX2MgLSBxX3MgKiByX3M7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bFRSUihxOiBiMlJvdCwgcjogYjJSb3QsIG91dDogYjJSb3QpOiBiMlJvdCB7XHJcbiAgICAvLyBbIHFjIHFzXSAqIFtyYyAtcnNdID0gW3FjKnJjK3FzKnJzIC1xYypycytxcypyY11cclxuICAgIC8vIFstcXMgcWNdICAgW3JzICByY10gICBbLXFzKnJjK3FjKnJzIHFzKnJzK3FjKnJjXVxyXG4gICAgLy8gcyA9IHFjICogcnMgLSBxcyAqIHJjXHJcbiAgICAvLyBjID0gcWMgKiByYyArIHFzICogcnNcclxuICAgIGNvbnN0IHFfYzogbnVtYmVyID0gcS5jLFxyXG4gICAgICBxX3M6IG51bWJlciA9IHEucztcclxuICAgIGNvbnN0IHJfYzogbnVtYmVyID0gci5jLFxyXG4gICAgICByX3M6IG51bWJlciA9IHIucztcclxuICAgIG91dC5zID0gcV9jICogcl9zIC0gcV9zICogcl9jO1xyXG4gICAgb3V0LmMgPSBxX2MgKiByX2MgKyBxX3MgKiByX3M7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bFJWPFQgZXh0ZW5kcyBYWT4ocTogYjJSb3QsIHY6IFhZLCBvdXQ6IFQpOiBUIHtcclxuICAgIGNvbnN0IHFfYzogbnVtYmVyID0gcS5jLFxyXG4gICAgICBxX3M6IG51bWJlciA9IHEucztcclxuICAgIGNvbnN0IHZfeDogbnVtYmVyID0gdi54LFxyXG4gICAgICB2X3k6IG51bWJlciA9IHYueTtcclxuICAgIG91dC54ID0gcV9jICogdl94IC0gcV9zICogdl95O1xyXG4gICAgb3V0LnkgPSBxX3MgKiB2X3ggKyBxX2MgKiB2X3k7XHJcbiAgICByZXR1cm4gb3V0O1xyXG4gIH1cclxuXHJcbiAgc3RhdGljIE11bFRSVjxUIGV4dGVuZHMgWFk+KHE6IGIyUm90LCB2OiBYWSwgb3V0OiBUKTogVCB7XHJcbiAgICBjb25zdCBxX2M6IG51bWJlciA9IHEuYyxcclxuICAgICAgcV9zOiBudW1iZXIgPSBxLnM7XHJcbiAgICBjb25zdCB2X3g6IG51bWJlciA9IHYueCxcclxuICAgICAgdl95OiBudW1iZXIgPSB2Lnk7XHJcbiAgICBvdXQueCA9IHFfYyAqIHZfeCArIHFfcyAqIHZfeTtcclxuICAgIG91dC55ID0gLXFfcyAqIHZfeCArIHFfYyAqIHZfeTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gQSB0cmFuc2Zvcm0gY29udGFpbnMgdHJhbnNsYXRpb24gYW5kIHJvdGF0aW9uLiBJdCBpcyB1c2VkIHRvIHJlcHJlc2VudFxyXG4vLy8gdGhlIHBvc2l0aW9uIGFuZCBvcmllbnRhdGlvbiBvZiByaWdpZCBmcmFtZXMuXHJcbmV4cG9ydCBjbGFzcyBiMlRyYW5zZm9ybSB7XHJcbiAgc3RhdGljIHJlYWRvbmx5IElERU5USVRZOiBSZWFkb25seTxiMlRyYW5zZm9ybT4gPSBuZXcgYjJUcmFuc2Zvcm0oKTtcclxuXHJcbiAgcmVhZG9ubHkgcDogYjJWZWMyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IHE6IGIyUm90ID0gbmV3IGIyUm90KCk7XHJcblxyXG4gIENsb25lKCk6IGIyVHJhbnNmb3JtIHtcclxuICAgIHJldHVybiBuZXcgYjJUcmFuc2Zvcm0oKS5Db3B5KHRoaXMpO1xyXG4gIH1cclxuXHJcbiAgQ29weShvdGhlcjogYjJUcmFuc2Zvcm0pOiB0aGlzIHtcclxuICAgIHRoaXMucC5Db3B5KG90aGVyLnApO1xyXG4gICAgdGhpcy5xLkNvcHkob3RoZXIucSk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNldElkZW50aXR5KCk6IHRoaXMge1xyXG4gICAgdGhpcy5wLlNldFplcm8oKTtcclxuICAgIHRoaXMucS5TZXRJZGVudGl0eSgpO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZXRQb3NpdGlvblJvdGF0aW9uKHBvc2l0aW9uOiBYWSwgcTogUmVhZG9ubHk8YjJSb3Q+KTogdGhpcyB7XHJcbiAgICB0aGlzLnAuQ29weShwb3NpdGlvbik7XHJcbiAgICB0aGlzLnEuQ29weShxKTtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgU2V0UG9zaXRpb25BbmdsZShwb3M6IFhZLCBhOiBudW1iZXIpOiB0aGlzIHtcclxuICAgIHRoaXMucC5Db3B5KHBvcyk7XHJcbiAgICB0aGlzLnEuU2V0QW5nbGUoYSk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNldFBvc2l0aW9uKHBvc2l0aW9uOiBYWSk6IHRoaXMge1xyXG4gICAgdGhpcy5wLkNvcHkocG9zaXRpb24pO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZXRQb3NpdGlvblhZKHg6IG51bWJlciwgeTogbnVtYmVyKTogdGhpcyB7XHJcbiAgICB0aGlzLnAuU2V0KHgsIHkpO1xyXG4gICAgcmV0dXJuIHRoaXM7XHJcbiAgfVxyXG5cclxuICBTZXRSb3RhdGlvbihyb3RhdGlvbjogUmVhZG9ubHk8YjJSb3Q+KTogdGhpcyB7XHJcbiAgICB0aGlzLnEuQ29weShyb3RhdGlvbik7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIFNldFJvdGF0aW9uQW5nbGUocmFkaWFuczogbnVtYmVyKTogdGhpcyB7XHJcbiAgICB0aGlzLnEuU2V0QW5nbGUocmFkaWFucyk7XHJcbiAgICByZXR1cm4gdGhpcztcclxuICB9XHJcblxyXG4gIEdldFBvc2l0aW9uKCk6IFJlYWRvbmx5PGIyVmVjMj4ge1xyXG4gICAgcmV0dXJuIHRoaXMucDtcclxuICB9XHJcblxyXG4gIEdldFJvdGF0aW9uKCk6IFJlYWRvbmx5PGIyUm90PiB7XHJcbiAgICByZXR1cm4gdGhpcy5xO1xyXG4gIH1cclxuXHJcbiAgR2V0Um90YXRpb25BbmdsZSgpOiBudW1iZXIge1xyXG4gICAgcmV0dXJuIHRoaXMucS5HZXRBbmdsZSgpO1xyXG4gIH1cclxuXHJcbiAgR2V0QW5nbGUoKTogbnVtYmVyIHtcclxuICAgIHJldHVybiB0aGlzLnEuR2V0QW5nbGUoKTtcclxuICB9XHJcblxyXG4gIHN0YXRpYyBNdWxYVjxUT3V0IGV4dGVuZHMgWFk+KFQ6IGIyVHJhbnNmb3JtLCB2OiBYWSwgb3V0OiBUT3V0KTogVE91dCB7XHJcbiAgICAvLyBmbG9hdDMyIHggPSAoVC5xLmMgKiB2LnggLSBULnEucyAqIHYueSkgKyBULnAueDtcclxuICAgIC8vIGZsb2F0MzIgeSA9IChULnEucyAqIHYueCArIFQucS5jICogdi55KSArIFQucC55O1xyXG4gICAgLy8gcmV0dXJuIGIyVmVjMih4LCB5KTtcclxuICAgIGNvbnN0IFRfcV9jOiBudW1iZXIgPSBULnEuYyxcclxuICAgICAgVF9xX3M6IG51bWJlciA9IFQucS5zO1xyXG4gICAgY29uc3Qgdl94OiBudW1iZXIgPSB2LngsXHJcbiAgICAgIHZfeTogbnVtYmVyID0gdi55O1xyXG4gICAgb3V0LnggPSBUX3FfYyAqIHZfeCAtIFRfcV9zICogdl95ICsgVC5wLng7XHJcbiAgICBvdXQueSA9IFRfcV9zICogdl94ICsgVF9xX2MgKiB2X3kgKyBULnAueTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTXVsVFhWPFRPdXQgZXh0ZW5kcyBYWT4oVDogYjJUcmFuc2Zvcm0sIHY6IFhZLCBvdXQ6IFRPdXQpOiBUT3V0IHtcclxuICAgIC8vIGZsb2F0MzIgcHggPSB2LnggLSBULnAueDtcclxuICAgIC8vIGZsb2F0MzIgcHkgPSB2LnkgLSBULnAueTtcclxuICAgIC8vIGZsb2F0MzIgeCA9IChULnEuYyAqIHB4ICsgVC5xLnMgKiBweSk7XHJcbiAgICAvLyBmbG9hdDMyIHkgPSAoLVQucS5zICogcHggKyBULnEuYyAqIHB5KTtcclxuICAgIC8vIHJldHVybiBiMlZlYzIoeCwgeSk7XHJcbiAgICBjb25zdCBUX3FfYzogbnVtYmVyID0gVC5xLmMsXHJcbiAgICAgIFRfcV9zOiBudW1iZXIgPSBULnEucztcclxuICAgIGNvbnN0IHBfeDogbnVtYmVyID0gdi54IC0gVC5wLng7XHJcbiAgICBjb25zdCBwX3k6IG51bWJlciA9IHYueSAtIFQucC55O1xyXG4gICAgb3V0LnggPSBUX3FfYyAqIHBfeCArIFRfcV9zICogcF95O1xyXG4gICAgb3V0LnkgPSAtVF9xX3MgKiBwX3ggKyBUX3FfYyAqIHBfeTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTXVsWFgoQTogYjJUcmFuc2Zvcm0sIEI6IGIyVHJhbnNmb3JtLCBvdXQ6IGIyVHJhbnNmb3JtKTogYjJUcmFuc2Zvcm0ge1xyXG4gICAgYjJSb3QuTXVsUlIoQS5xLCBCLnEsIG91dC5xKTtcclxuICAgIGIyVmVjMi5BZGRWVihiMlJvdC5NdWxSVihBLnEsIEIucCwgb3V0LnApLCBBLnAsIG91dC5wKTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG5cclxuICBzdGF0aWMgTXVsVFhYKEE6IGIyVHJhbnNmb3JtLCBCOiBiMlRyYW5zZm9ybSwgb3V0OiBiMlRyYW5zZm9ybSk6IGIyVHJhbnNmb3JtIHtcclxuICAgIGIyUm90Lk11bFRSUihBLnEsIEIucSwgb3V0LnEpO1xyXG4gICAgYjJSb3QuTXVsVFJWKEEucSwgYjJWZWMyLlN1YlZWKEIucCwgQS5wLCBvdXQucCksIG91dC5wKTtcclxuICAgIHJldHVybiBvdXQ7XHJcbiAgfVxyXG59XHJcblxyXG4vLy8gVGhpcyBkZXNjcmliZXMgdGhlIG1vdGlvbiBvZiBhIGJvZHkvc2hhcGUgZm9yIFRPSSBjb21wdXRhdGlvbi5cclxuLy8vIFNoYXBlcyBhcmUgZGVmaW5lZCB3aXRoIHJlc3BlY3QgdG8gdGhlIGJvZHkgb3JpZ2luLCB3aGljaCBtYXlcclxuLy8vIG5vIGNvaW5jaWRlIHdpdGggdGhlIGNlbnRlciBvZiBtYXNzLiBIb3dldmVyLCB0byBzdXBwb3J0IGR5bmFtaWNzXHJcbi8vLyB3ZSBtdXN0IGludGVycG9sYXRlIHRoZSBjZW50ZXIgb2YgbWFzcyBwb3NpdGlvbi5cclxuZXhwb3J0IGNsYXNzIGIyU3dlZXAge1xyXG4gIHJlYWRvbmx5IGxvY2FsQ2VudGVyID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IGMwID0gbmV3IGIyVmVjMigpO1xyXG4gIHJlYWRvbmx5IGMgPSBuZXcgYjJWZWMyKCk7XHJcbiAgYTAgPSBOYU47XHJcbiAgYSA9IE5hTjtcclxuICBhbHBoYTAgPSBOYU47XHJcblxyXG4gIGNvbnN0cnVjdG9yKCkge1xyXG4gICAgdGhpcy5hMCA9IDAuMDtcclxuICAgIHRoaXMuYSA9IDAuMDtcclxuICAgIHRoaXMuYWxwaGEwID0gMC4wO1xyXG4gIH1cclxuXHJcbiAgQ2xvbmUoKTogYjJTd2VlcCB7XHJcbiAgICByZXR1cm4gbmV3IGIyU3dlZXAoKS5Db3B5KHRoaXMpO1xyXG4gIH1cclxuXHJcbiAgQ29weShvdGhlcjogYjJTd2VlcCk6IHRoaXMge1xyXG4gICAgdGhpcy5sb2NhbENlbnRlci5Db3B5KG90aGVyLmxvY2FsQ2VudGVyKTtcclxuICAgIHRoaXMuYzAuQ29weShvdGhlci5jMCk7XHJcbiAgICB0aGlzLmMuQ29weShvdGhlci5jKTtcclxuICAgIHRoaXMuYTAgPSBvdGhlci5hMDtcclxuICAgIHRoaXMuYSA9IG90aGVyLmE7XHJcbiAgICB0aGlzLmFscGhhMCA9IG90aGVyLmFscGhhMDtcclxuICAgIHJldHVybiB0aGlzO1xyXG4gIH1cclxuXHJcbiAgR2V0VHJhbnNmb3JtKHhmOiBiMlRyYW5zZm9ybSwgYmV0YTogbnVtYmVyKTogYjJUcmFuc2Zvcm0ge1xyXG4gICAgY29uc3Qgb25lX21pbnVzX2JldGEgPSAxLjAgLSBiZXRhO1xyXG4gICAgeGYucC54ID0gb25lX21pbnVzX2JldGEgKiB0aGlzLmMwLnggKyBiZXRhICogdGhpcy5jLng7XHJcbiAgICB4Zi5wLnkgPSBvbmVfbWludXNfYmV0YSAqIHRoaXMuYzAueSArIGJldGEgKiB0aGlzLmMueTtcclxuICAgIGNvbnN0IGFuZ2xlID0gb25lX21pbnVzX2JldGEgKiB0aGlzLmEwICsgYmV0YSAqIHRoaXMuYTtcclxuICAgIHhmLnEuU2V0QW5nbGUoYW5nbGUpO1xyXG5cclxuICAgIHhmLnAuU2VsZlN1YihiMlJvdC5NdWxSVih4Zi5xLCB0aGlzLmxvY2FsQ2VudGVyLCBiMlZlYzIuc190MCkpO1xyXG4gICAgcmV0dXJuIHhmO1xyXG4gIH1cclxuXHJcbiAgQWR2YW5jZShhbHBoYTogbnVtYmVyKTogdm9pZCB7XHJcbiAgICAhIUIyX0RFQlVHICYmIGIyQXNzZXJ0KHRoaXMuYWxwaGEwIDwgMS4wKTtcclxuICAgIGNvbnN0IGJldGE6IG51bWJlciA9IChhbHBoYSAtIHRoaXMuYWxwaGEwKSAvICgxLjAgLSB0aGlzLmFscGhhMCk7XHJcbiAgICBjb25zdCBvbmVfbWludXNfYmV0YTogbnVtYmVyID0gMSAtIGJldGE7XHJcbiAgICB0aGlzLmMwLnggPSBvbmVfbWludXNfYmV0YSAqIHRoaXMuYzAueCArIGJldGEgKiB0aGlzLmMueDtcclxuICAgIHRoaXMuYzAueSA9IG9uZV9taW51c19iZXRhICogdGhpcy5jMC55ICsgYmV0YSAqIHRoaXMuYy55O1xyXG4gICAgdGhpcy5hMCA9IG9uZV9taW51c19iZXRhICogdGhpcy5hMCArIGJldGEgKiB0aGlzLmE7XHJcbiAgICB0aGlzLmFscGhhMCA9IGFscGhhO1xyXG4gIH1cclxuXHJcbiAgTm9ybWFsaXplKCk6IHZvaWQge1xyXG4gICAgY29uc3QgZCA9IGIyX3R3b19waSAqIE1hdGguZmxvb3IodGhpcy5hMCAvIGIyX3R3b19waSk7XHJcbiAgICB0aGlzLmEwIC09IGQ7XHJcbiAgICB0aGlzLmEgLT0gZDtcclxuICB9XHJcbn1cclxuIl19