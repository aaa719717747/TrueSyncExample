/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System;
using System.Collections.Generic;
#endregion

namespace TrueSync.Physics3D {

    /// <summary>
    /// GJK based implementation of Raycasting.
    /// </summary>
    public sealed class GJKCollide {
        private static FP CollideEpsilon = FP.EN3;
        private const int MaxIterations = 15;

        private static ResourcePool<VoronoiSimplexSolver> simplexSolverPool = new ResourcePool<VoronoiSimplexSolver>();

        #region private static void SupportMapTransformed(ISupportMappable support, ref JMatrix orientation, ref JVector position, ref JVector direction, out JVector result)
        private static void SupportMapTransformed(ISupportMappable support, ref TSMatrix orientation, ref TSVector position, ref TSVector direction, out TSVector result) {
            //JVector.Transform(ref direction, ref invOrientation, out result);
            //support.SupportMapping(ref result, out result);
            //JVector.Transform(ref result, ref orientation, out result);
            //JVector.Add(ref result, ref position, out result);

            result.x = ((direction.x * orientation.M11) + (direction.y * orientation.M12)) + (direction.z * orientation.M13);
            result.y = ((direction.x * orientation.M21) + (direction.y * orientation.M22)) + (direction.z * orientation.M23);
            result.z = ((direction.x * orientation.M31) + (direction.y * orientation.M32)) + (direction.z * orientation.M33);

            support.SupportMapping(ref result, out result);

            FP x = ((result.x * orientation.M11) + (result.y * orientation.M21)) + (result.z * orientation.M31);
            FP y = ((result.x * orientation.M12) + (result.y * orientation.M22)) + (result.z * orientation.M32);
            FP z = ((result.x * orientation.M13) + (result.y * orientation.M23)) + (result.z * orientation.M33);

            result.x = position.x + x;
            result.y = position.y + y;
            result.z = position.z + z;
        }
        #endregion

        /// <summary>
        /// Checks if given point is within a shape.
        /// </summary>
        /// <param name="support">The supportmap implementation representing the shape.</param>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="invOrientation">The inverse orientation of the shape.</param>
        /// <param name="position">The position of the shape.</param>
        /// <param name="point">The point to check.</param>
        /// <returns>Returns true if the point is within the shape, otherwise false.</returns>
        public static bool Pointcast(ISupportMappable support, ref TSMatrix orientation, ref TSVector position, ref TSVector point) {
            TSVector arbitraryPoint;

            SupportMapTransformed(support, ref orientation, ref position, ref point, out arbitraryPoint);
            TSVector.Subtract(ref point, ref arbitraryPoint, out arbitraryPoint);

            TSVector r; support.SupportCenter(out r);
            TSVector.Transform(ref r, ref orientation, out r);
            TSVector.Add(ref position, ref r, out r);
            TSVector.Subtract(ref point, ref r, out r);

            TSVector x = point;
            TSVector w, p;
            FP VdotR;

            TSVector v; TSVector.Subtract(ref x, ref arbitraryPoint, out v);
            FP dist = v.sqrMagnitude;
            FP epsilon = CollideEpsilon;

            int maxIter = MaxIterations;

            VoronoiSimplexSolver simplexSolver = simplexSolverPool.GetNew();

            simplexSolver.Reset();

            while ((dist > epsilon) && (maxIter-- != 0)) {
                SupportMapTransformed(support, ref orientation, ref position, ref v, out p);
                TSVector.Subtract(ref x, ref p, out w);

                FP VdotW = TSVector.Dot(ref v, ref w);

                if (VdotW > FP.Zero) {
                    VdotR = TSVector.Dot(ref v, ref r);

                    if (VdotR >= -(TSMath.Epsilon * TSMath.Epsilon)) { simplexSolverPool.GiveBack(simplexSolver); return false; } else simplexSolver.Reset();
                }
                if (!simplexSolver.InSimplex(w)) simplexSolver.AddVertex(w, x, p);

                if (simplexSolver.Closest(out v)) dist = v.sqrMagnitude;
                else dist = FP.Zero;
            }

            simplexSolverPool.GiveBack(simplexSolver);
            return true;

        }


        public static bool ClosestPoints(ISupportMappable support1, ISupportMappable support2, ref TSMatrix orientation1,
            ref TSMatrix orientation2, ref TSVector position1, ref TSVector position2,
            out TSVector p1, out TSVector p2, out TSVector normal) {

            VoronoiSimplexSolver simplexSolver = simplexSolverPool.GetNew();
            simplexSolver.Reset();

            p1 = p2 = TSVector.zero;

            TSVector r = position1 - position2;
            TSVector w, v;

            TSVector supVertexA;
            TSVector rn, vn;

            rn = TSVector.Negate(r);

            SupportMapTransformed(support1, ref orientation1, ref position1, ref rn, out supVertexA);

            TSVector supVertexB;
            SupportMapTransformed(support2, ref orientation2, ref position2, ref r, out supVertexB);

            v = supVertexA - supVertexB;

            normal = TSVector.zero;

            int maxIter = MaxIterations;

            FP distSq = v.sqrMagnitude;
            FP epsilon = CollideEpsilon;

            while ((distSq > epsilon) && (maxIter-- != 0)) {
                vn = TSVector.Negate(v);
                SupportMapTransformed(support1, ref orientation1, ref position1, ref vn, out supVertexA);
                SupportMapTransformed(support2, ref orientation2, ref position2, ref v, out supVertexB);
                w = supVertexA - supVertexB;

                if (!simplexSolver.InSimplex(w)) simplexSolver.AddVertex(w, supVertexA, supVertexB);
                if (simplexSolver.Closest(out v)) {
                    distSq = v.sqrMagnitude;
                    normal = v;
                } else distSq = FP.Zero;
            }


            simplexSolver.ComputePoints(out p1, out p2);

            if (normal.sqrMagnitude > TSMath.Epsilon * TSMath.Epsilon)
                normal.Normalize();

            simplexSolverPool.GiveBack(simplexSolver);

            return true;

        }

        #region TimeOfImpact Conservative Advancement - Depricated
        //    public static bool TimeOfImpact(ISupportMappable support1, ISupportMappable support2, ref JMatrix orientation1,
        //ref JMatrix orientation2, ref JVector position1, ref JVector position2, ref JVector sweptA, ref JVector sweptB,
        //out JVector p1, out JVector p2, out JVector normal)
        //    {

        //        VoronoiSimplexSolver simplexSolver = simplexSolverPool.GetNew();
        //        simplexSolver.Reset();

        //        FP lambda = FP.Zero;

        //        p1 = p2 = JVector.Zero;

        //        JVector x1 = position1;
        //        JVector x2 = position2;

        //        JVector r = sweptA - sweptB;
        //        JVector w, v;

        //        JVector supVertexA;
        //        JVector rn = JVector.Negate(r);
        //        SupportMapTransformed(support1, ref orientation1, ref x1, ref rn, out supVertexA);

        //        JVector supVertexB;
        //        SupportMapTransformed(support2, ref orientation2, ref x2, ref r, out supVertexB);

        //        v = supVertexA - supVertexB;

        //        bool hasResult = false;

        //        normal = JVector.Zero;


        //        FP lastLambda = lambda;

        //        int maxIter = MaxIterations;

        //        FP distSq = v.LengthSquared();
        //        FP epsilon = FP.EN5;

        //        FP VdotR;

        //        while ((distSq > epsilon) && (maxIter-- != 0))
        //        {

        //            JVector vn = JVector.Negate(v);
        //            SupportMapTransformed(support1, ref orientation1, ref x1, ref vn, out supVertexA);
        //            SupportMapTransformed(support2, ref orientation2, ref x2, ref v, out supVertexB);
        //            w = supVertexA - supVertexB;

        //            FP VdotW = JVector.Dot(ref v, ref w);

        //            if (VdotW > FP.Zero)
        //            {
        //                VdotR = JVector.Dot(ref v, ref r);

        //                if (VdotR >= -JMath.Epsilon)
        //                {
        //                    simplexSolverPool.GiveBack(simplexSolver);
        //                    return false;
        //                }
        //                else
        //                {
        //                    lambda = lambda - VdotW / VdotR;


        //                    x1 = position1 + lambda * sweptA;
        //                    x2 = position2 + lambda * sweptB;

        //                    w = supVertexA - supVertexB;

        //                    normal = v;
        //                    hasResult = true;
        //                }
        //            }
        //            if (!simplexSolver.InSimplex(w)) simplexSolver.AddVertex(w, supVertexA, supVertexB);
        //            if (simplexSolver.Closest(out v))
        //            {
        //                distSq = v.LengthSquared();
        //                normal = v;
        //                hasResult = true;
        //            }
        //            else distSq = FP.Zero;
        //        }


        //        simplexSolver.ComputePoints(out p1, out p2);


        //        if (normal.LengthSquared() > JMath.Epsilon * JMath.Epsilon)
        //            normal.Normalize();

        //        p1 = p1 - lambda * sweptA;
        //        p2 = p2 - lambda * sweptB;

        //        simplexSolverPool.GiveBack(simplexSolver);

        //        return true;

        //    }
        #endregion

        // see: btSubSimplexConvexCast.cpp

        /// <summary>
        /// Checks if a ray definied through it's origin and direction collides
        /// with a shape.
        /// </summary>
        /// <param name="support">The supportmap implementation representing the shape.</param>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="invOrientation">The inverse orientation of the shape.</param>
        /// <param name="position">The position of the shape.</param>
        /// <param name="origin">The origin of the ray.</param>
        /// <param name="direction">The direction of the ray.</param>
        /// <param name="fraction">The fraction which gives information where at the 
        /// ray the collision occured. The hitPoint is calculated by: origin+friction*direction.</param>
        /// <param name="normal">The normal from the ray collision.</param>
        /// <returns>Returns true if the ray hit the shape, false otherwise.</returns>
        public static bool Raycast(ISupportMappable support, ref TSMatrix orientation, ref TSMatrix invOrientation,
            ref TSVector position, ref TSVector origin, ref TSVector direction, out FP fraction, out TSVector normal) {
            VoronoiSimplexSolver simplexSolver = simplexSolverPool.GetNew();
            simplexSolver.Reset();

            normal = TSVector.zero;
            fraction = FP.MaxValue;

            FP lambda = FP.Zero;

            TSVector r = direction;
            TSVector x = origin;
            TSVector w, p, v;

            TSVector arbitraryPoint;
            SupportMapTransformed(support, ref orientation, ref position, ref r, out arbitraryPoint);
            TSVector.Subtract(ref x, ref arbitraryPoint, out v);

            int maxIter = MaxIterations;

            FP distSq = v.sqrMagnitude;
            FP epsilon = FP.EN6;

            FP VdotR;

            while ((distSq > epsilon) && (maxIter-- != 0)) {
                SupportMapTransformed(support, ref orientation, ref position, ref v, out p);
                TSVector.Subtract(ref x, ref p, out w);

                FP VdotW = TSVector.Dot(ref v, ref w);

                if (VdotW > FP.Zero) {
                    VdotR = TSVector.Dot(ref v, ref r);

                    if (VdotR >= -TSMath.Epsilon) {
                        simplexSolverPool.GiveBack(simplexSolver);
                        return false;
                    } else {
                        lambda = lambda - VdotW / VdotR;
                        TSVector.Multiply(ref r, lambda, out x);
                        TSVector.Add(ref origin, ref x, out x);
                        TSVector.Subtract(ref x, ref p, out w);
                        normal = v;
                    }
                }
                if (!simplexSolver.InSimplex(w)) simplexSolver.AddVertex(w, x, p);
                if (simplexSolver.Closest(out v)) { distSq = v.sqrMagnitude; } else distSq = FP.Zero;
            }

            #region Retrieving hitPoint

            // Giving back the fraction like this *should* work
            // but is inaccurate against large objects:
            // fraction = lambda;

            TSVector p1, p2;
            simplexSolver.ComputePoints(out p1, out p2);

            p2 = p2 - origin;
            fraction = p2.magnitude / direction.magnitude;

            #endregion

            if (normal.sqrMagnitude > TSMath.Epsilon * TSMath.Epsilon)
                normal.Normalize();

            simplexSolverPool.GiveBack(simplexSolver);

            return true;
        }

        // see: btVoronoiSimplexSolver.cpp
        #region private class VoronoiSimplexSolver - Bullet

        // Bullet has problems with raycasting large objects - so does jitter
        // hope to fix that in the next versions.

        /*
          Bullet for XNA Copyright (c) 2003-2007 Vsevolod Klementjev http://www.codeplex.com/xnadevru
          Bullet original C++ version Copyright (c) 2003-2007 Erwin Coumans http://bulletphysics.com

          This software is provided 'as-is', without any express or implied
          warranty.  In no event will the authors be held liable for any damages
          arising from the use of this software.

          Permission is granted to anyone to use this software for any purpose,
          including commercial applications, and to alter it and redistribute it
          freely, subject to the following restrictions:

          1. The origin of this software must not be misrepresented; you must not
             claim that you wrote the original software. If you use this software
             in a product, an acknowledgment in the product documentation would be
             appreciated but is not required.
          2. Altered source versions must be plainly marked as such, and must not be
             misrepresented as being the original software.
          3. This notice may not be removed or altered from any source distribution.
        */

    }

    /// VoronoiSimplexSolver is an implementation of the closest point distance
    /// algorithm from a 1-4 points simplex to the origin.
    /// Can be used with GJK, as an alternative to Johnson distance algorithm. 
    public class VoronoiSimplexSolver {
        private const int VertexA = 0, VertexB = 1, VertexC = 2, VertexD = 3;

        private const int VoronoiSimplexMaxVerts = 5;
        private const bool CatchDegenerateTetrahedron = true;

        private int _numVertices;

        private TSVector[] _simplexVectorW = new TSVector[VoronoiSimplexMaxVerts];
        private TSVector[] _simplexPointsP = new TSVector[VoronoiSimplexMaxVerts];
        private TSVector[] _simplexPointsQ = new TSVector[VoronoiSimplexMaxVerts];

        private TSVector _cachedPA;
        private TSVector _cachedPB;
        private TSVector _cachedV;
        private TSVector _lastW;
        private bool _cachedValidClosest;

        private SubSimplexClosestResult _cachedBC = new SubSimplexClosestResult();

        // Note that this assumes ray-casts and point-casts will always be called from the
        // same thread which I assume is true from the _cachedBC member
        // If this needs to made multi-threaded a resource pool will be needed
        private SubSimplexClosestResult tempResult = new SubSimplexClosestResult();

        private bool _needsUpdate;

        #region ISimplexSolver Members

        public bool FullSimplex {
            get {
                return _numVertices == 4;
            }
        }

        public int NumVertices {
            get {
                return _numVertices;
            }
        }

        public void Reset() {
            _cachedValidClosest = false;
            _numVertices = 0;
            _needsUpdate = true;
            _lastW = new TSVector(FP.MaxValue, FP.MaxValue, FP.MaxValue);
            _cachedBC.Reset();
        }

        public void AddVertex(TSVector w, TSVector p, TSVector q) {
            _lastW = w;
            _needsUpdate = true;

            _simplexVectorW[_numVertices] = w;
            _simplexPointsP[_numVertices] = p;
            _simplexPointsQ[_numVertices] = q;

            _numVertices++;
        }

        //return/calculate the closest vertex
        public bool Closest(out TSVector v) {
            bool succes = UpdateClosestVectorAndPoints();
            v = _cachedV;
            return succes;
        }

        public FP MaxVertex {
            get {
                int numverts = NumVertices;
                FP maxV = FP.Zero, curLen2;
                for (int i = 0; i < numverts; i++) {
                    curLen2 = _simplexVectorW[i].sqrMagnitude;
                    if (maxV < curLen2) maxV = curLen2;
                }
                return maxV;
            }
        }

        //return the current simplex
        public int GetSimplex(out TSVector[] pBuf, out TSVector[] qBuf, out TSVector[] yBuf) {
            int numverts = NumVertices;
            pBuf = new TSVector[numverts];
            qBuf = new TSVector[numverts];
            yBuf = new TSVector[numverts];
            for (int i = 0; i < numverts; i++) {
                yBuf[i] = _simplexVectorW[i];
                pBuf[i] = _simplexPointsP[i];
                qBuf[i] = _simplexPointsQ[i];
            }
            return numverts;
        }

        public bool InSimplex(TSVector w) {
            //check in case lastW is already removed
            if (w == _lastW) return true;

            //w is in the current (reduced) simplex
            int numverts = NumVertices;
            for (int i = 0; i < numverts; i++)
                if (_simplexVectorW[i] == w) return true;

            return false;
        }

        public void BackupClosest(out TSVector v) {
            v = _cachedV;
        }

        public bool EmptySimplex {
            get {
                return NumVertices == 0;
            }
        }

        public void ComputePoints(out TSVector p1, out TSVector p2) {
            UpdateClosestVectorAndPoints();
            p1 = _cachedPA;
            p2 = _cachedPB;
        }

        #endregion

        public void RemoveVertex(int index) {
            _numVertices--;
            _simplexVectorW[index] = _simplexVectorW[_numVertices];
            _simplexPointsP[index] = _simplexPointsP[_numVertices];
            _simplexPointsQ[index] = _simplexPointsQ[_numVertices];
        }

        public void ReduceVertices(UsageBitfield usedVerts) {
            if ((NumVertices >= 4) && (!usedVerts.UsedVertexD)) RemoveVertex(3);
            if ((NumVertices >= 3) && (!usedVerts.UsedVertexC)) RemoveVertex(2);
            if ((NumVertices >= 2) && (!usedVerts.UsedVertexB)) RemoveVertex(1);
            if ((NumVertices >= 1) && (!usedVerts.UsedVertexA)) RemoveVertex(0);
        }

        public bool UpdateClosestVectorAndPoints() {
            if (_needsUpdate) {
                _cachedBC.Reset();
                _needsUpdate = false;

                TSVector p, a, b, c, d;
                switch (NumVertices) {
                    case 0:
                        _cachedValidClosest = false;
                        break;
                    case 1:
                        _cachedPA = _simplexPointsP[0];
                        _cachedPB = _simplexPointsQ[0];
                        _cachedV = _cachedPA - _cachedPB;
                        _cachedBC.Reset();
                        _cachedBC.SetBarycentricCoordinates(1f, FP.Zero, FP.Zero, FP.Zero);
                        _cachedValidClosest = _cachedBC.IsValid;
                        break;
                    case 2:
                        //closest point origin from line segment
                        TSVector from = _simplexVectorW[0];
                        TSVector to = _simplexVectorW[1];
                        //TSVector nearest;

                        TSVector diff = from * (-1);
                        TSVector v = to - from;
                        FP t = TSVector.Dot(v, diff);

                        if (t > 0) {
                            FP dotVV = v.sqrMagnitude;
                            if (t < dotVV) {
                                t /= dotVV;
                                diff -= t * v;
                                _cachedBC.UsedVertices.UsedVertexA = true;
                                _cachedBC.UsedVertices.UsedVertexB = true;
                            } else {
                                t = 1;
                                diff -= v;
                                //reduce to 1 point
                                _cachedBC.UsedVertices.UsedVertexB = true;
                            }
                        } else {
                            t = 0;
                            //reduce to 1 point
                            _cachedBC.UsedVertices.UsedVertexA = true;
                        }

                        _cachedBC.SetBarycentricCoordinates(1 - t, t, 0, 0);
                        //nearest = from + t * v;

                        _cachedPA = _simplexPointsP[0] + t * (_simplexPointsP[1] - _simplexPointsP[0]);
                        _cachedPB = _simplexPointsQ[0] + t * (_simplexPointsQ[1] - _simplexPointsQ[0]);
                        _cachedV = _cachedPA - _cachedPB;

                        ReduceVertices(_cachedBC.UsedVertices);

                        _cachedValidClosest = _cachedBC.IsValid;
                        break;
                    case 3:
                        //closest point origin from triangle
                        p = new TSVector();
                        a = _simplexVectorW[0];
                        b = _simplexVectorW[1];
                        c = _simplexVectorW[2];

                        ClosestPtPointTriangle(p, a, b, c, ref _cachedBC);
                        _cachedPA = _simplexPointsP[0] * _cachedBC.BarycentricCoords[0] +
                                        _simplexPointsP[1] * _cachedBC.BarycentricCoords[1] +
                                        _simplexPointsP[2] * _cachedBC.BarycentricCoords[2] +
                                        _simplexPointsP[3] * _cachedBC.BarycentricCoords[3];

                        _cachedPB = _simplexPointsQ[0] * _cachedBC.BarycentricCoords[0] +
                                        _simplexPointsQ[1] * _cachedBC.BarycentricCoords[1] +
                                        _simplexPointsQ[2] * _cachedBC.BarycentricCoords[2] +
                                        _simplexPointsQ[3] * _cachedBC.BarycentricCoords[3];

                        _cachedV = _cachedPA - _cachedPB;

                        ReduceVertices(_cachedBC.UsedVertices);
                        _cachedValidClosest = _cachedBC.IsValid;
                        break;
                    case 4:
                        p = new TSVector();
                        a = _simplexVectorW[0];
                        b = _simplexVectorW[1];
                        c = _simplexVectorW[2];
                        d = _simplexVectorW[3];

                        bool hasSeperation = ClosestPtPointTetrahedron(p, a, b, c, d, ref _cachedBC);

                        if (hasSeperation) {
                            _cachedPA = _simplexPointsP[0] * _cachedBC.BarycentricCoords[0] +
                                            _simplexPointsP[1] * _cachedBC.BarycentricCoords[1] +
                                            _simplexPointsP[2] * _cachedBC.BarycentricCoords[2] +
                                            _simplexPointsP[3] * _cachedBC.BarycentricCoords[3];

                            _cachedPB = _simplexPointsQ[0] * _cachedBC.BarycentricCoords[0] +
                                            _simplexPointsQ[1] * _cachedBC.BarycentricCoords[1] +
                                            _simplexPointsQ[2] * _cachedBC.BarycentricCoords[2] +
                                            _simplexPointsQ[3] * _cachedBC.BarycentricCoords[3];

                            _cachedV = _cachedPA - _cachedPB;
                            ReduceVertices(_cachedBC.UsedVertices);
                        } else {
                            if (_cachedBC.Degenerate) {
                                _cachedValidClosest = false;
                            } else {
                                _cachedValidClosest = true;
                                //degenerate case == false, penetration = true + zero
                                _cachedV.x = _cachedV.y = _cachedV.z = FP.Zero;
                            }
                            break; // !!!!!!!!!!!! proverit na vsakiy sluchai
                        }

                        _cachedValidClosest = _cachedBC.IsValid;

                        //closest point origin from tetrahedron
                        break;
                    default:
                        _cachedValidClosest = false;
                        break;
                }
            }

            return _cachedValidClosest;
        }

        public bool ClosestPtPointTriangle(TSVector p, TSVector a, TSVector b, TSVector c,
            ref SubSimplexClosestResult result) {
            result.UsedVertices.Reset();

            FP v, w;

            // Check if P in vertex region outside A
            TSVector ab = b - a;
            TSVector ac = c - a;
            TSVector ap = p - a;
            FP d1 = TSVector.Dot(ab, ap);
            FP d2 = TSVector.Dot(ac, ap);
            if (d1 <= FP.Zero && d2 <= FP.Zero) {
                result.ClosestPointOnSimplex = a;
                result.UsedVertices.UsedVertexA = true;
                result.SetBarycentricCoordinates(1, 0, 0, 0);
                return true; // a; // barycentric coordinates (1,0,0)
            }

            // Check if P in vertex region outside B
            TSVector bp = p - b;
            FP d3 = TSVector.Dot(ab, bp);
            FP d4 = TSVector.Dot(ac, bp);
            if (d3 >= FP.Zero && d4 <= d3) {
                result.ClosestPointOnSimplex = b;
                result.UsedVertices.UsedVertexB = true;
                result.SetBarycentricCoordinates(0, 1, 0, 0);

                return true; // b; // barycentric coordinates (0,1,0)
            }
            // Check if P in edge region of AB, if so return projection of P onto AB
            FP vc = d1 * d4 - d3 * d2;
            if (vc <= FP.Zero && d1 >= FP.Zero && d3 <= FP.Zero) {
                v = d1 / (d1 - d3);
                result.ClosestPointOnSimplex = a + v * ab;
                result.UsedVertices.UsedVertexA = true;
                result.UsedVertices.UsedVertexB = true;
                result.SetBarycentricCoordinates(1 - v, v, 0, 0);
                return true;
                //return a + v * ab; // barycentric coordinates (1-v,v,0)
            }

            // Check if P in vertex region outside C
            TSVector cp = p - c;
            FP d5 = TSVector.Dot(ab, cp);
            FP d6 = TSVector.Dot(ac, cp);
            if (d6 >= FP.Zero && d5 <= d6) {
                result.ClosestPointOnSimplex = c;
                result.UsedVertices.UsedVertexC = true;
                result.SetBarycentricCoordinates(0, 0, 1, 0);
                return true;//c; // barycentric coordinates (0,0,1)
            }

            // Check if P in edge region of AC, if so return projection of P onto AC
            FP vb = d5 * d2 - d1 * d6;
            if (vb <= FP.Zero && d2 >= FP.Zero && d6 <= FP.Zero) {
                w = d2 / (d2 - d6);
                result.ClosestPointOnSimplex = a + w * ac;
                result.UsedVertices.UsedVertexA = true;
                result.UsedVertices.UsedVertexC = true;
                result.SetBarycentricCoordinates(1 - w, 0, w, 0);
                return true;
                //return a + w * ac; // barycentric coordinates (1-w,0,w)
            }

            // Check if P in edge region of BC, if so return projection of P onto BC
            FP va = d3 * d6 - d5 * d4;
            if (va <= FP.Zero && (d4 - d3) >= FP.Zero && (d5 - d6) >= FP.Zero) {
                w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

                result.ClosestPointOnSimplex = b + w * (c - b);
                result.UsedVertices.UsedVertexB = true;
                result.UsedVertices.UsedVertexC = true;
                result.SetBarycentricCoordinates(0, 1 - w, w, 0);
                return true;
                // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
            }

            // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
            FP denom = FP.One / (va + vb + vc);
            v = vb * denom;
            w = vc * denom;

            result.ClosestPointOnSimplex = a + ab * v + ac * w;
            result.UsedVertices.UsedVertexA = true;
            result.UsedVertices.UsedVertexB = true;
            result.UsedVertices.UsedVertexC = true;
            result.SetBarycentricCoordinates(1 - v - w, v, w, 0);

            return true;
        }

        /// Test if point p and d lie on opposite sides of plane through abc
        public int PointOutsideOfPlane(TSVector p, TSVector a, TSVector b, TSVector c, TSVector d) {
            TSVector normal = TSVector.Cross(b - a, c - a);

            FP signp = TSVector.Dot(p - a, normal); // [AP AB AC]
            FP signd = TSVector.Dot(d - a, normal); // [AD AB AC]

            //if (CatchDegenerateTetrahedron)
            if (signd * signd < (FP.EN8)) return -1;

            // Points on opposite sides if expression signs are opposite
            return signp * signd < FP.Zero ? 1 : 0;
        }

        public bool ClosestPtPointTetrahedron(TSVector p, TSVector a, TSVector b, TSVector c, TSVector d,
            ref SubSimplexClosestResult finalResult) {
            tempResult.Reset();

            // Start out assuming point inside all halfspaces, so closest to itself
            finalResult.ClosestPointOnSimplex = p;
            finalResult.UsedVertices.Reset();
            finalResult.UsedVertices.UsedVertexA = true;
            finalResult.UsedVertices.UsedVertexB = true;
            finalResult.UsedVertices.UsedVertexC = true;
            finalResult.UsedVertices.UsedVertexD = true;

            int pointOutsideABC = PointOutsideOfPlane(p, a, b, c, d);
            int pointOutsideACD = PointOutsideOfPlane(p, a, c, d, b);
            int pointOutsideADB = PointOutsideOfPlane(p, a, d, b, c);
            int pointOutsideBDC = PointOutsideOfPlane(p, b, d, c, a);

            if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0) {
                finalResult.Degenerate = true;
                return false;
            }

            if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0)
                return false;

            FP bestSqDist = FP.MaxValue;
            // If point outside face abc then compute closest point on abc
            if (pointOutsideABC != 0) {
                ClosestPtPointTriangle(p, a, b, c, ref tempResult);
                TSVector q = tempResult.ClosestPointOnSimplex;

                FP sqDist = ((TSVector)(q - p)).sqrMagnitude;
                // Update best closest point if (squared) distance is less than current best
                if (sqDist < bestSqDist) {
                    bestSqDist = sqDist;
                    finalResult.ClosestPointOnSimplex = q;
                    //convert result bitmask!
                    finalResult.UsedVertices.Reset();
                    finalResult.UsedVertices.UsedVertexA = tempResult.UsedVertices.UsedVertexA;
                    finalResult.UsedVertices.UsedVertexB = tempResult.UsedVertices.UsedVertexB;
                    finalResult.UsedVertices.UsedVertexC = tempResult.UsedVertices.UsedVertexC;
                    finalResult.SetBarycentricCoordinates(
                            tempResult.BarycentricCoords[VertexA],
                            tempResult.BarycentricCoords[VertexB],
                            tempResult.BarycentricCoords[VertexC],
                            0);
                }
            }

            // Repeat test for face acd
            if (pointOutsideACD != 0) {
                ClosestPtPointTriangle(p, a, c, d, ref tempResult);
                TSVector q = tempResult.ClosestPointOnSimplex;
                //convert result bitmask!

                FP sqDist = ((TSVector)(q - p)).sqrMagnitude;
                if (sqDist < bestSqDist) {
                    bestSqDist = sqDist;
                    finalResult.ClosestPointOnSimplex = q;
                    finalResult.UsedVertices.Reset();
                    finalResult.UsedVertices.UsedVertexA = tempResult.UsedVertices.UsedVertexA;
                    finalResult.UsedVertices.UsedVertexC = tempResult.UsedVertices.UsedVertexB;
                    finalResult.UsedVertices.UsedVertexD = tempResult.UsedVertices.UsedVertexC;
                    finalResult.SetBarycentricCoordinates(
                            tempResult.BarycentricCoords[VertexA],
                            0,
                            tempResult.BarycentricCoords[VertexB],
                            tempResult.BarycentricCoords[VertexC]);
                }
            }
            // Repeat test for face adb

            if (pointOutsideADB != 0) {
                ClosestPtPointTriangle(p, a, d, b, ref tempResult);
                TSVector q = tempResult.ClosestPointOnSimplex;
                //convert result bitmask!

                FP sqDist = ((TSVector)(q - p)).sqrMagnitude;
                if (sqDist < bestSqDist) {
                    bestSqDist = sqDist;
                    finalResult.ClosestPointOnSimplex = q;
                    finalResult.UsedVertices.Reset();
                    finalResult.UsedVertices.UsedVertexA = tempResult.UsedVertices.UsedVertexA;
                    finalResult.UsedVertices.UsedVertexD = tempResult.UsedVertices.UsedVertexB;
                    finalResult.UsedVertices.UsedVertexB = tempResult.UsedVertices.UsedVertexC;
                    finalResult.SetBarycentricCoordinates(
                            tempResult.BarycentricCoords[VertexA],
                            tempResult.BarycentricCoords[VertexC],
                            0,
                            tempResult.BarycentricCoords[VertexB]);

                }
            }
            // Repeat test for face bdc

            if (pointOutsideBDC != 0) {
                ClosestPtPointTriangle(p, b, d, c, ref tempResult);
                TSVector q = tempResult.ClosestPointOnSimplex;
                //convert result bitmask!
                FP sqDist = ((TSVector)(q - p)).sqrMagnitude;
                if (sqDist < bestSqDist) {
                    bestSqDist = sqDist;
                    finalResult.ClosestPointOnSimplex = q;
                    finalResult.UsedVertices.Reset();
                    finalResult.UsedVertices.UsedVertexB = tempResult.UsedVertices.UsedVertexA;
                    finalResult.UsedVertices.UsedVertexD = tempResult.UsedVertices.UsedVertexB;
                    finalResult.UsedVertices.UsedVertexC = tempResult.UsedVertices.UsedVertexC;

                    finalResult.SetBarycentricCoordinates(
                            0,
                            tempResult.BarycentricCoords[VertexA],
                            tempResult.BarycentricCoords[VertexC],
                            tempResult.BarycentricCoords[VertexB]);
                }
            }

            //help! we ended up full !

            if (finalResult.UsedVertices.UsedVertexA &&
                finalResult.UsedVertices.UsedVertexB &&
                finalResult.UsedVertices.UsedVertexC &&
                finalResult.UsedVertices.UsedVertexD) {
                return true;
            }

            return true;
        }
    }

    #endregion

    public class UsageBitfield {
        private bool _usedVertexA, _usedVertexB, _usedVertexC, _usedVertexD;

        public bool UsedVertexA { get { return _usedVertexA; } set { _usedVertexA = value; } }
        public bool UsedVertexB { get { return _usedVertexB; } set { _usedVertexB = value; } }
        public bool UsedVertexC { get { return _usedVertexC; } set { _usedVertexC = value; } }
        public bool UsedVertexD { get { return _usedVertexD; } set { _usedVertexD = value; } }

        public void Reset() {
            _usedVertexA = _usedVertexB = _usedVertexC = _usedVertexD = false;
        }
    }

    public class SubSimplexClosestResult {
        private TSVector _closestPointOnSimplex;

        //MASK for m_usedVertices
        //stores the simplex vertex-usage, using the MASK, 
        // if m_usedVertices & MASK then the related vertex is used
        private UsageBitfield _usedVertices = new UsageBitfield();
        private FP[] _barycentricCoords = new FP[4];
        private bool _degenerate;

        public TSVector ClosestPointOnSimplex { get { return _closestPointOnSimplex; } set { _closestPointOnSimplex = value; } }
        public UsageBitfield UsedVertices { get { return _usedVertices; } set { _usedVertices = value; } }
        public FP[] BarycentricCoords { get { return _barycentricCoords; } set { _barycentricCoords = value; } }
        public bool Degenerate { get { return _degenerate; } set { _degenerate = value; } }

        public void Reset() {
            _degenerate = false;
            SetBarycentricCoordinates();
            _usedVertices.Reset();
        }

        public bool IsValid {
            get {
                return (_barycentricCoords[0] >= FP.Zero) &&
                        (_barycentricCoords[1] >= FP.Zero) &&
                        (_barycentricCoords[2] >= FP.Zero) &&
                        (_barycentricCoords[3] >= FP.Zero);
            }
        }

        public void SetBarycentricCoordinates() {
            SetBarycentricCoordinates(FP.Zero, FP.Zero, FP.Zero, FP.Zero);
        }

        public void SetBarycentricCoordinates(FP a, FP b, FP c, FP d) {
            _barycentricCoords[0] = a;
            _barycentricCoords[1] = b;
            _barycentricCoords[2] = c;
            _barycentricCoords[3] = d;
        }
    }

}