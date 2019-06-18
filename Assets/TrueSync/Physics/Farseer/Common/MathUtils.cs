/*
* Farseer Physics Engine:
* Copyright (c) 2012 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org 
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

using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace TrueSync.Physics2D
{
    public static class MathUtils
    {
        public static FP Cross(ref TSVector2 a, ref TSVector2 b)
        {
            return a.x * b.y - a.y * b.x;
        }

        public static FP Cross(TSVector2 a, TSVector2 b)
        {
            return Cross(ref a, ref b);
        }

        /// Perform the cross product on two vectors.
        public static TSVector Cross(TSVector a, TSVector b)
        {
            return new TSVector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
        }

        public static TSVector2 Cross(TSVector2 a, FP s)
        {
            return new TSVector2(s * a.y, -s * a.x);
        }

        public static TSVector2 Cross(FP s, TSVector2 a)
        {
            return new TSVector2(-s * a.y, s * a.x);
        }

        public static TSVector2 Abs(TSVector2 v)
        {
            return new TSVector2(FP.Abs(v.x), FP.Abs(v.y));
        }

        public static TSVector2 Mul(ref Mat22 A, TSVector2 v)
        {
            return Mul(ref A, ref v);
        }

        public static TSVector2 Mul(ref Mat22 A, ref TSVector2 v)
        {
            return new TSVector2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
        }

        public static TSVector2 Mul(ref Transform T, TSVector2 v)
        {
            return Mul(ref T, ref v);
        }

        public static TSVector2 Mul(ref Transform T, ref TSVector2 v)
        {
            FP x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
            FP y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;

            return new TSVector2(x, y);
        }

        public static TSVector2 MulT(ref Mat22 A, TSVector2 v)
        {
            return MulT(ref A, ref v);
        }

        public static TSVector2 MulT(ref Mat22 A, ref TSVector2 v)
        {
            return new TSVector2(v.x * A.ex.x + v.y * A.ex.y, v.x * A.ey.x + v.y * A.ey.y);
        }

        public static TSVector2 MulT(ref Transform T, TSVector2 v)
        {
            return MulT(ref T, ref v);
        }

        public static TSVector2 MulT(ref Transform T, ref TSVector2 v)
        {
            FP px = v.x - T.p.x;
            FP py = v.y - T.p.y;
            FP x = (T.q.c * px + T.q.s * py);
            FP y = (-T.q.s * px + T.q.c * py);

            return new TSVector2(x, y);
        }

        // A^T * B
        public static void MulT(ref Mat22 A, ref Mat22 B, out Mat22 C)
        {
            C = new Mat22();
            C.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
            C.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
            C.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
            C.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
        }

        /// Multiply a matrix times a vector.
        public static TSVector Mul(Mat33 A, TSVector v)
        {
            return v.x * A.ex + v.y * A.ey + v.z * A.ez;
        }

        // v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
        //    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
        public static Transform Mul(Transform A, Transform B)
        {
            Transform C = new Transform();
            C.q = Mul(A.q, B.q);
            C.p = Mul(A.q, B.p) + A.p;
            return C;
        }

        // v2 = A.q' * (B.q * v1 + B.p - A.p)
        //    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
        public static void MulT(ref Transform A, ref Transform B, out Transform C)
        {
            C = new Transform();
            C.q = MulT(A.q, B.q);
            C.p = MulT(A.q, B.p - A.p);
        }

        public static void Swap<T>(ref T a, ref T b)
        {
            T tmp = a;
            a = b;
            b = tmp;
        }

        /// Multiply a matrix times a vector.
        public static TSVector2 Mul22(Mat33 A, TSVector2 v)
        {
            return new TSVector2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
        }

        /// Multiply two rotations: q * r
        public static Rot Mul(Rot q, Rot r)
        {
            // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
            // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
            // s = qs * rc + qc * rs
            // c = qc * rc - qs * rs
            Rot qr;
            qr.s = q.s * r.c + q.c * r.s;
            qr.c = q.c * r.c - q.s * r.s;
            return qr;
        }

        public static TSVector2 MulT(Transform T, TSVector2 v)
        {
            FP px = v.x - T.p.x;
            FP py = v.y - T.p.y;
            FP x = (T.q.c * px + T.q.s * py);
            FP y = (-T.q.s * px + T.q.c * py);

            return new TSVector2(x, y);
        }

        /// Transpose multiply two rotations: qT * r
        public static Rot MulT(Rot q, Rot r)
        {
            // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
            // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
            // s = qc * rs - qs * rc
            // c = qc * rc + qs * rs
            Rot qr;
            qr.s = q.c * r.s - q.s * r.c;
            qr.c = q.c * r.c + q.s * r.s;
            return qr;
        }

        // v2 = A.q' * (B.q * v1 + B.p - A.p)
        //    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
        public static Transform MulT(Transform A, Transform B)
        {
            Transform C = new Transform();
            C.q = MulT(A.q, B.q);
            C.p = MulT(A.q, B.p - A.p);
            return C;
        }

        /// Rotate a vector
        public static TSVector2 Mul(Rot q, TSVector2 v)
        {
            return new TSVector2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
        }

        /// Inverse rotate a vector
        public static TSVector2 MulT(Rot q, TSVector2 v)
        {
            return new TSVector2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
        }

        /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
        public static TSVector2 Skew(TSVector2 input)
        {
            return new TSVector2(-input.y, input.x);
        }

        /// <summary>
        /// This function is used to ensure that a FPing point number is
        /// not a NaN or infinity.
        /// </summary>
        /// <param name="x">The x.</param>
        /// <returns>
        /// 	<c>true</c> if the specified x is valid; otherwise, <c>false</c>.
        /// </returns>
        public static bool IsValid(FP x)
        {
            if (FP.IsNaN(x))
            {
                // NaN.
                return false;
            }

            return !FP.IsInfinity(x);
        }

        public static bool IsValid(this TSVector2 x)
        {
            return IsValid(x.x) && IsValid(x.y);
        }

        /// <summary>
        /// This is a approximate yet fast inverse square-root.
        /// </summary>
        /// <param name="x">The x.</param>
        /// <returns></returns>
        public static FP InvSqrt(FP x)
        {
            FPConverter convert = new FPConverter();
            convert.x = x;
            FP xhalf = 0.5f * x;
            convert.i = 0x5f3759df - (convert.i >> 1);
            x = convert.x;
            x = x * (1.5f - xhalf * x * x);
            return x;
        }

        public static int Clamp(int a, int low, int high)
        {
            return Math.Max(low, Math.Min(a, high));
        }

        public static FP Clamp(FP a, FP low, FP high)
        {
            return TrueSync.TSMath.Max(low, TrueSync.TSMath.Min(a, high));
        }

        public static TSVector2 Clamp(TSVector2 a, TSVector2 low, TSVector2 high)
        {
            return TSVector2.Max(low, TSVector2.Min(a, high));
        }

        public static void Cross(ref TSVector2 a, ref TSVector2 b, out FP c)
        {
            c = a.x * b.y - a.y * b.x;
        }

        /// <summary>
        /// Return the angle between two vectors on a plane
        /// The angle is from vector 1 to vector 2, positive anticlockwise
        /// The result is between -pi -> pi
        /// </summary>
        public static FP VectorAngle(ref TSVector2 p1, ref TSVector2 p2)
        {
            FP theta1 = FP.Atan2(p1.y, p1.x);
            FP theta2 = FP.Atan2(p2.y, p2.x);
            FP dtheta = theta2 - theta1;
            while (dtheta > FP.Pi)
                dtheta -= (2 * FP.Pi);
            while (dtheta < -FP.Pi)
                dtheta += (2 * FP.Pi);

            return (dtheta);
        }

        /// Perform the dot product on two vectors.
        public static FP Dot(TSVector a, TSVector b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        public static FP VectorAngle(TSVector2 p1, TSVector2 p2)
        {
            return VectorAngle(ref p1, ref p2);
        }

        /// <summary>
        /// Returns a positive number if c is to the left of the line going from a to b.
        /// </summary>
        /// <returns>Positive number if point is left, negative if point is right, 
        /// and 0 if points are collinear.</returns>
        public static FP Area(TSVector2 a, TSVector2 b, TSVector2 c)
        {
            return Area(ref a, ref b, ref c);
        }

        /// <summary>
        /// Returns a positive number if c is to the left of the line going from a to b.
        /// </summary>
        /// <returns>Positive number if point is left, negative if point is right, 
        /// and 0 if points are collinear.</returns>
        public static FP Area(ref TSVector2 a, ref TSVector2 b, ref TSVector2 c)
        {
            return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y);
        }

        /// <summary>
        /// Determines if three vertices are collinear (ie. on a straight line)
        /// </summary>
        /// <param name="a">First vertex</param>
        /// <param name="b">Second vertex</param>
        /// <param name="c">Third vertex</param>
        /// <param name="tolerance">The tolerance</param>
        /// <returns></returns>
        // TS - public static bool IsCollinear(ref Vector2 a, ref Vector2 b, ref Vector2 c, FP tolerance = 0)
        public static bool IsCollinear(ref TSVector2 a, ref TSVector2 b, ref TSVector2 c, FP tolerance)
        {
            return FPInRange(Area(ref a, ref b, ref c), -tolerance, tolerance);
        }

        public static void Cross(FP s, ref TSVector2 a, out TSVector2 b)
        {
            b = new TSVector2(-s * a.y, s * a.x);
        }

        public static bool FPEquals(FP value1, FP value2)
        {
            return FP.Abs(value1 - value2) <= Settings.Epsilon;
        }

        /// <summary>
        /// Checks if a FPing point Value is equal to another,
        /// within a certain tolerance.
        /// </summary>
        /// <param name="value1">The first FPing point Value.</param>
        /// <param name="value2">The second FPing point Value.</param>
        /// <param name="delta">The FPing point tolerance.</param>
        /// <returns>True if the values are "equal", false otherwise.</returns>
        public static bool FPEquals(FP value1, FP value2, FP delta)
        {
            return FPInRange(value1, value2 - delta, value2 + delta);
        }

        /// <summary>
        /// Checks if a FPing point Value is within a specified
        /// range of values (inclusive).
        /// </summary>
        /// <param name="value">The Value to check.</param>
        /// <param name="min">The minimum Value.</param>
        /// <param name="max">The maximum Value.</param>
        /// <returns>True if the Value is within the range specified,
        /// false otherwise.</returns>
        public static bool FPInRange(FP value, FP min, FP max)
        {
            return (value >= min && value <= max);
        }

        #region Nested type: FPConverter

        [StructLayout(LayoutKind.Explicit)]
        private struct FPConverter
        {
            [FieldOffset(0)]
            public FP x;
            [FieldOffset(0)]
            public int i;
        }

        #endregion

        public static TSVector2 Mul(ref Rot rot, TSVector2 axis)
        {
            return Mul(rot, axis);
        }

        public static TSVector2 MulT(ref Rot rot, TSVector2 axis)
        {
            return MulT(rot, axis);
        }
    }

    /// <summary>
    /// A 2-by-2 matrix. Stored in column-major order.
    /// </summary>
    public struct Mat22
    {
        public TSVector2 ex, ey;

        /// <summary>
        /// Construct this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        public Mat22(TSVector2 c1, TSVector2 c2)
        {
            ex = c1;
            ey = c2;
        }

        /// <summary>
        /// Construct this matrix using scalars.
        /// </summary>
        /// <param name="a11">The a11.</param>
        /// <param name="a12">The a12.</param>
        /// <param name="a21">The a21.</param>
        /// <param name="a22">The a22.</param>
        public Mat22(FP a11, FP a12, FP a21, FP a22)
        {
            ex = new TSVector2(a11, a21);
            ey = new TSVector2(a12, a22);
        }

        public Mat22 Inverse
        {
            get
            {
                FP a = ex.x, b = ey.x, c = ex.y, d = ey.y;
                FP det = a * d - b * c;
                if (det != 0.0f)
                {
                    det = 1.0f / det;
                }

                Mat22 result = new Mat22();
                result.ex.x = det * d;
                result.ex.y = -det * c;

                result.ey.x = -det * b;
                result.ey.y = det * a;

                return result;
            }
        }

        /// <summary>
        /// Initialize this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        public void Set(TSVector2 c1, TSVector2 c2)
        {
            ex = c1;
            ey = c2;
        }

        /// <summary>
        /// Set this to the identity matrix.
        /// </summary>
        public void SetIdentity()
        {
            ex.x = 1.0f;
            ey.x = 0.0f;
            ex.y = 0.0f;
            ey.y = 1.0f;
        }

        /// <summary>
        /// Set this matrix to all zeros.
        /// </summary>
        public void SetZero()
        {
            ex.x = 0.0f;
            ey.x = 0.0f;
            ex.y = 0.0f;
            ey.y = 0.0f;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public TSVector2 Solve(TSVector2 b)
        {
            FP a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
            FP det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            return new TSVector2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
        }

        public static void Add(ref Mat22 A, ref Mat22 B, out Mat22 R)
        {
            R.ex = A.ex + B.ex;
            R.ey = A.ey + B.ey;
        }
    }

    /// <summary>
    /// A 3-by-3 matrix. Stored in column-major order.
    /// </summary>
    public struct Mat33
    {
        public TSVector ex, ey, ez;

        /// <summary>
        /// Construct this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        /// <param name="c3">The c3.</param>
        public Mat33(TSVector c1, TSVector c2, TSVector c3)
        {
            ex = c1;
            ey = c2;
            ez = c3;
        }

        /// <summary>
        /// Set this matrix to all zeros.
        /// </summary>
        public void SetZero()
        {
            ex = TSVector.zero;
            ey = TSVector.zero;
            ez = TSVector.zero;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public TSVector Solve33(TSVector b)
        {
            FP det = TSVector.Dot(ex, TSVector.Cross(ey, ez));
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            return new TSVector(det * TSVector.Dot(b, TSVector.Cross(ey, ez)), det * TSVector.Dot(ex, TSVector.Cross(b, ez)), det * TSVector.Dot(ex, TSVector.Cross(ey, b)));
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases. Solve only the upper
        /// 2-by-2 matrix equation.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public TSVector2 Solve22(TSVector2 b)
        {
            FP a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
            FP det = a11 * a22 - a12 * a21;

            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            return new TSVector2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
        }

        /// Get the inverse of this matrix as a 2-by-2.
        /// Returns the zero matrix if singular.
        public void GetInverse22(ref Mat33 M)
        {
            FP a = ex.x, b = ey.x, c = ex.y, d = ey.y;
            FP det = a * d - b * c;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            M.ex.x = det * d; M.ey.x = -det * b; M.ex.z = 0.0f;
            M.ex.y = -det * c; M.ey.y = det * a; M.ey.z = 0.0f;
            M.ez.x = 0.0f; M.ez.y = 0.0f; M.ez.z = 0.0f;
        }

        /// Get the symmetric inverse of this matrix as a 3-by-3.
        /// Returns the zero matrix if singular.
        public void GetSymInverse33(ref Mat33 M)
        {
            FP det = MathUtils.Dot(ex, MathUtils.Cross(ey, ez));
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            FP a11 = ex.x, a12 = ey.x, a13 = ez.x;
            FP a22 = ey.y, a23 = ez.y;
            FP a33 = ez.z;

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
    }

    /// <summary>
    /// Rotation
    /// </summary>
    public struct Rot
    {
        /// Sine and cosine
        public FP s, c;

        /// <summary>
        /// Initialize from an angle in radians
        /// </summary>
        /// <param name="angle">Angle in radians</param>
        public Rot(FP angle)
        {
            // TODO_ERIN optimize
            s = FP.Sin(angle);
            c = FP.Cos(angle);
        }

        /// <summary>
        /// Set using an angle in radians.
        /// </summary>
        /// <param name="angle"></param>
        public void Set(FP angle)
        {
            // TODO_ERIN optimize
            s = FP.Sin(angle);
            c = FP.Cos(angle);
        }

        /// <summary>
        /// Set to the identity rotation
        /// </summary>
        public void SetIdentity()
        {
            s = 0.0f;
            c = 1.0f;
        }

        /// <summary>
        /// Get the angle in radians
        /// </summary>
        public FP GetAngle()
        {
            return FP.Atan2(s, c);
        }

        /// <summary>
        /// Get the x-axis
        /// </summary>
        public TSVector2 GetXAxis()
        {
            return new TSVector2(c, s);
        }

        /// <summary>
        /// Get the y-axis
        /// </summary>
        public TSVector2 GetYAxis()
        {
            return new TSVector2(-s, c);
        }
    }

    /// <summary>
    /// A transform contains translation and rotation. It is used to represent
    /// the position and orientation of rigid frames.
    /// </summary>
    public struct Transform
    {
        public TSVector2 p;
        public Rot q;

        /// <summary>
        /// Initialize using a position vector and a rotation matrix.
        /// </summary>
        /// <param name="position">The position.</param>
        /// <param name="rotation">The r.</param>
        public Transform(ref TSVector2 position, ref Rot rotation)
        {
            p = position;
            q = rotation;
        }

        /// <summary>
        /// Set this to the identity transform.
        /// </summary>
        public void SetIdentity()
        {
            p = TSVector2.zero;
            q.SetIdentity();
        }

        /// <summary>
        /// Set this based on the position and angle.
        /// </summary>
        /// <param name="position">The position.</param>
        /// <param name="angle">The angle.</param>
        public void Set(TSVector2 position, FP angle)
        {
            p = position;
            q.Set(angle);
        }
    }

    /// <summary>
    /// This describes the motion of a body/shape for TOI computation.
    /// Shapes are defined with respect to the body origin, which may
    /// no coincide with the center of mass. However, to support dynamics
    /// we must interpolate the center of mass position.
    /// </summary>
    public struct Sweep
    {
        /// <summary>
        /// World angles
        /// </summary>
        public FP A;

        public FP A0;

        /// <summary>
        /// Fraction of the current time step in the range [0,1]
        /// c0 and a0 are the positions at alpha0.
        /// </summary>
        public FP Alpha0;

        /// <summary>
        /// Center world positions
        /// </summary>
        public TSVector2 C;

        public TSVector2 C0;

        /// <summary>
        /// Local center of mass position
        /// </summary>
        public TSVector2 LocalCenter;

        /// <summary>
        /// Get the interpolated transform at a specific time.
        /// </summary>
        /// <param name="xfb">The transform.</param>
        /// <param name="beta">beta is a factor in [0,1], where 0 indicates alpha0.</param>
        public void GetTransform(out Transform xfb, FP beta)
        {
            xfb = new Transform();
            xfb.p.x = (1.0f - beta) * C0.x + beta * C.x;
            xfb.p.y = (1.0f - beta) * C0.y + beta * C.y;
            FP angle = (1.0f - beta) * A0 + beta * A;
            xfb.q.Set(angle);

            // Shift to origin
            xfb.p -= MathUtils.Mul(xfb.q, LocalCenter);
        }

        /// <summary>
        /// Advance the sweep forward, yielding a new initial state.
        /// </summary>
        /// <param name="alpha">new initial time..</param>
        public void Advance(FP alpha)
        {
            Debug.Assert(Alpha0 < 1.0f);
            FP beta = (alpha - Alpha0) / (1.0f - Alpha0);
            C0 += beta * (C - C0);
            A0 += beta * (A - A0);
            Alpha0 = alpha;
        }

        /// <summary>
        /// Normalize the angles.
        /// </summary>
        public void Normalize()
        {
            FP d = FP.PiTimes2 * FP.Floor((A0 / FP.PiTimes2));
            A0 -= d;
            A -= d;
        }
    }
}