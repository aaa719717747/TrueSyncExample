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
    /// The implementation of the ISupportMappable interface defines the form
    /// of a shape. <seealso cref="GJKCollide"/> <seealso cref="XenoCollide"/>
    /// </summary>
    public interface ISupportMappable
    {
        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        void SupportMapping(ref TSVector direction, out TSVector result);

        /// <summary>
        /// The center of the SupportMap.
        /// </summary>
        /// <param name="center"></param>
        void SupportCenter(out TSVector center);
    }

    /// <summary>
    /// Implementation of the XenoCollide Algorithm by Gary Snethen. 
    /// Narrowphase collision detection highly optimized for C#.
    /// http://xenocollide.snethen.com/
    /// </summary>
    public sealed class XenoCollide
    {

        private static FP CollideEpsilon = FP.EN3;
        private const int MaximumIterations = 5;

        private static void SupportMapTransformed(ISupportMappable support,
            ref TSMatrix orientation, ref TSVector position, ref TSVector direction, out TSVector result)
        {
            // THIS IS *THE* HIGH FREQUENCY CODE OF THE COLLLISION PART OF THE ENGINE

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

        /// <summary>
        /// Checks two shapes for collisions.
        /// </summary>
        /// <param name="support1">The SupportMappable implementation of the first shape to test.</param>
        /// <param name="support2">The SupportMappable implementation of the seconds shape to test.</param>
        /// <param name="orientation1">The orientation of the first shape.</param>
        /// <param name="orientation2">The orientation of the second shape.</param>
        /// <param name="position1">The position of the first shape.</param>
        /// <param name="position2">The position of the second shape</param>
        /// <param name="point">The pointin world coordinates, where collision occur.</param>
        /// <param name="normal">The normal pointing from body2 to body1.</param>
        /// <param name="penetration">Estimated penetration depth of the collision.</param>
        /// <returns>Returns true if there is a collision, false otherwise.</returns>
        public static bool Detect(ISupportMappable support1, ISupportMappable support2, ref TSMatrix orientation1,
             ref TSMatrix orientation2, ref TSVector position1, ref TSVector position2,
             out TSVector point, out TSVector normal, out FP penetration)
        {
            // Used variables
            TSVector temp1, temp2;
            TSVector v01, v02, v0;
            TSVector v11, v12, v1;
            TSVector v21, v22, v2;
            TSVector v31, v32, v3;
			TSVector v41 = TSVector.zero, v42 = TSVector.zero, v4 = TSVector.zero;
            TSVector mn;

            // Initialization of the output
            point = normal = TSVector.zero;
            penetration = FP.Zero;

            //JVector right = JVector.Right;

            // Get the center of shape1 in world coordinates -> v01
            support1.SupportCenter(out v01);
            TSVector.Transform(ref v01, ref orientation1, out v01);
            TSVector.Add(ref position1, ref v01, out v01);

            // Get the center of shape2 in world coordinates -> v02
            support2.SupportCenter(out v02);
            TSVector.Transform(ref v02, ref orientation2, out v02);
            TSVector.Add(ref position2, ref v02, out v02);

            // v0 is the center of the minkowski difference
            TSVector.Subtract(ref v02, ref v01, out v0);

            // Avoid case where centers overlap -- any direction is fine in this case
            if (v0.IsNearlyZero()) v0 = new TSVector(FP.EN4, 0, 0);

            // v1 = support in direction of origin
            mn = v0;
            TSVector.Negate(ref v0, out normal);
			//UnityEngine.Debug.Log("normal: " + normal);

            SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v11);
            SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v12);
            TSVector.Subtract(ref v12, ref v11, out v1);

            if (TSVector.Dot(ref v1, ref normal) <= FP.Zero) return false;

            // v2 = support perpendicular to v1,v0
            TSVector.Cross(ref v1, ref v0, out normal);

            if (normal.IsNearlyZero())
            {
                TSVector.Subtract(ref v1, ref v0, out normal);
				//UnityEngine.Debug.Log("normal: " + normal);

                normal.Normalize();

                point = v11;
                TSVector.Add(ref point, ref v12, out point);
                TSVector.Multiply(ref point, FP.Half, out point);

                TSVector.Subtract(ref v12, ref v11, out temp1);
                penetration = TSVector.Dot(ref temp1, ref normal);

                //point = v11;
                //point2 = v12;
                return true;
            }

            TSVector.Negate(ref normal, out mn);
            SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v21);
            SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v22);
            TSVector.Subtract(ref v22, ref v21, out v2);

            if (TSVector.Dot(ref v2, ref normal) <= FP.Zero) return false;

            // Determine whether origin is on + or - side of plane (v1,v0,v2)
            TSVector.Subtract(ref v1, ref v0, out temp1);
            TSVector.Subtract(ref v2, ref v0, out temp2);
            TSVector.Cross(ref temp1, ref temp2, out normal);

            FP dist = TSVector.Dot(ref normal, ref v0);

            // If the origin is on the - side of the plane, reverse the direction of the plane
            if (dist > FP.Zero)
            {
                TSVector.Swap(ref v1, ref v2);
                TSVector.Swap(ref v11, ref v21);
                TSVector.Swap(ref v12, ref v22);
                TSVector.Negate(ref normal, out normal);
				//UnityEngine.Debug.Log("normal: " + normal);
            }


            int phase2 = 0;
            int phase1 = 0;
            bool hit = false;

            // Phase One: Identify a portal
            while (true)
            {
                if (phase1 > MaximumIterations) return false;

                phase1++;

                // Obtain the support point in a direction perpendicular to the existing plane
                // Note: This point is guaranteed to lie off the plane
                TSVector.Negate(ref normal, out mn);
				//UnityEngine.Debug.Log("mn: " + mn);
                SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v31);
                SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v32);
                TSVector.Subtract(ref v32, ref v31, out v3);


                if (TSVector.Dot(ref v3, ref normal) <= FP.Zero)
                {
                    return false;
                }

                // If origin is outside (v1,v0,v3), then eliminate v2 and loop
                TSVector.Cross(ref v1, ref v3, out temp1);
                if (TSVector.Dot(ref temp1, ref v0) < FP.Zero)
                {
                    v2 = v3;
                    v21 = v31;
                    v22 = v32;
                    TSVector.Subtract(ref v1, ref v0, out temp1);
                    TSVector.Subtract(ref v3, ref v0, out temp2);
                    TSVector.Cross(ref temp1, ref temp2, out normal);
				//	UnityEngine.Debug.Log("normal: " + normal);
                    continue;
                }

                // If origin is outside (v3,v0,v2), then eliminate v1 and loop
                TSVector.Cross(ref v3, ref v2, out temp1);
                if (TSVector.Dot(ref temp1, ref v0) < FP.Zero)
                {
                    v1 = v3;
                    v11 = v31;
                    v12 = v32;
                    TSVector.Subtract(ref v3, ref v0, out temp1);
                    TSVector.Subtract(ref v2, ref v0, out temp2);
                    TSVector.Cross(ref temp1, ref temp2, out normal);
					//UnityEngine.Debug.Log("normal: " + normal);
                    continue;
                }

                // Phase Two: Refine the portal
                // We are now inside of a wedge...
                while (true)
                {
                    phase2++;
					/*
					UnityEngine.Debug.LogError(" ::Start STATE");
					UnityEngine.Debug.Log(temp1 + " " +  temp2);
					UnityEngine.Debug.Log( v01 + " " + v02 + " "+ v0);
					UnityEngine.Debug.Log( v11+" "+ v12 +" "+ v1);
	                UnityEngine.Debug.Log( v21 +" "+ v22 +" "+ v2);
	                UnityEngine.Debug.Log( v31 +" "+ v32 +" "+ v3);
	                UnityEngine.Debug.Log( v41 +" "+ v42 +" "+ v4);
	                UnityEngine.Debug.Log( mn);

					UnityEngine.Debug.LogError(" ::END STATE"); 
*/
					// Compute normal of the wedge face
                    TSVector.Subtract(ref v2, ref v1, out temp1);
                    TSVector.Subtract(ref v3, ref v1, out temp2);
                    TSVector.Cross(ref temp1, ref temp2, out normal);
				// Beginer
				//	UnityEngine.Debug.Log("normal: " + normal);

                    // Can this happen???  Can it be handled more cleanly?
                    if (normal.IsNearlyZero()) return true;

                    normal.Normalize();
					//UnityEngine.Debug.Log("normal: " + normal);
                    // Compute distance from origin to wedge face
                    FP d = TSVector.Dot(ref normal, ref v1);


                    // If the origin is inside the wedge, we have a hit
                    if (d >= 0 && !hit)
                    {
                        // HIT!!!
                        hit = true;
                    }

                    // Find the support point in the direction of the wedge face
                    TSVector.Negate(ref normal, out mn);
                    SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v41);
                    SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v42);
                    TSVector.Subtract(ref v42, ref v41, out v4);

                    TSVector.Subtract(ref v4, ref v3, out temp1);
                    FP delta = TSVector.Dot(ref temp1, ref normal);
                    penetration = TSVector.Dot(ref v4, ref normal);

                    // If the boundary is thin enough or the origin is outside the support plane for the newly discovered vertex, then we can terminate
                    if (delta <= CollideEpsilon || penetration <= FP.Zero || phase2 > MaximumIterations)
                    {

                        if (hit)
                        {
                            TSVector.Cross(ref v1, ref v2, out temp1);
                            FP b0 = TSVector.Dot(ref temp1, ref v3);
                            TSVector.Cross(ref v3, ref v2, out temp1);
                            FP b1 = TSVector.Dot(ref temp1, ref v0);
                            TSVector.Cross(ref v0, ref v1, out temp1);
                            FP b2 = TSVector.Dot(ref temp1, ref v3);
                            TSVector.Cross(ref v2, ref v1, out temp1);
                            FP b3 = TSVector.Dot(ref temp1, ref v0);

                            FP sum = b0 + b1 + b2 + b3;

                            if (sum <= 0)
                            {
                                b0 = 0;
                                TSVector.Cross(ref v2, ref v3, out temp1);
                                b1 = TSVector.Dot(ref temp1, ref normal);
                                TSVector.Cross(ref v3, ref v1, out temp1);
                                b2 = TSVector.Dot(ref temp1, ref normal);
                                TSVector.Cross(ref v1, ref v2, out temp1);
                                b3 = TSVector.Dot(ref temp1, ref normal);

                                sum = b1 + b2 + b3;
                            }

                            FP inv = FP.One / sum;

                            TSVector.Multiply(ref v01, b0, out point);
                            TSVector.Multiply(ref v11, b1, out temp1);
                            TSVector.Add(ref point, ref temp1, out point);
                            TSVector.Multiply(ref v21, b2, out temp1);
                            TSVector.Add(ref point, ref temp1, out point);
                            TSVector.Multiply(ref v31, b3, out temp1);
                            TSVector.Add(ref point, ref temp1, out point);

                            TSVector.Multiply(ref v02, b0, out temp2);
                            TSVector.Add(ref temp2, ref point, out point);
                            TSVector.Multiply(ref v12, b1, out temp1);
                            TSVector.Add(ref point, ref temp1, out point);
                            TSVector.Multiply(ref v22, b2, out temp1);
                            TSVector.Add(ref point, ref temp1, out point);
                            TSVector.Multiply(ref v32, b3, out temp1);
                            TSVector.Add(ref point, ref temp1, out point);

                            TSVector.Multiply(ref point, inv * FP.Half, out point);

                        }

                        // Compute the barycentric coordinates of the origin
                        return hit;
                    }

                    //// Compute the tetrahedron dividing face (v4,v0,v1)
                    //JVector.Cross(ref v4, ref v1, out temp1);
                    //FP d1 = JVector.Dot(ref temp1, ref v0);


                    //// Compute the tetrahedron dividing face (v4,v0,v2)
                    //JVector.Cross(ref v4, ref v2, out temp1);
                    //FP d2 = JVector.Dot(ref temp1, ref v0);


                    // Compute the tetrahedron dividing face (v4,v0,v3)
					//UnityEngine.Debug.LogError("v4:" +  v4 + " v0:" + v0);
                    TSVector.Cross(ref v4, ref v0, out temp1);
					//UnityEngine.Debug.LogError("temp1:"+ temp1);
					
					//Ender
				//	UnityEngine.Debug.Log("normal: " + normal);
                    FP dot = TSVector.Dot(ref temp1, ref v1);

                    if (dot >= FP.Zero)
                    {
					//	UnityEngine.Debug.Log("dot >= 0 temp1:" + temp1 + "  v2:" + v2 );
                        dot = TSVector.Dot(ref temp1, ref v2);

                        if (dot >= FP.Zero)
                        {
					//		UnityEngine.Debug.Log("dot >= 0 v1->v4");
							
                            // Inside d1 & inside d2 ==> eliminate v1
                            v1 = v4;
                            v11 = v41;
                            v12 = v42;
                        }
                        else
                        {
					//		UnityEngine.Debug.Log("dot < v3->v4");
							
                            // Inside d1 & outside d2 ==> eliminate v3
                            v3 = v4;
                            v31 = v41;
                            v32 = v42;
                        }
                    }
                    else
                    {
					//	UnityEngine.Debug.Log("dot < 0 temp1:" + temp1 + "  v3:" + v3 );
                        dot = TSVector.Dot(ref temp1, ref v3);

                        if (dot >= FP.Zero)
                        {
						//	UnityEngine.Debug.Log("dot >= 0 v2 => v4");
                            // Outside d1 & inside d3 ==> eliminate v2
                            v2 = v4;
                            v21 = v41;
                            v22 = v42;
                        }
                        else
                        {
					//		UnityEngine.Debug.Log("dot < 0 v1 => v4");
                            // Outside d1 & outside d3 ==> eliminate v1
                            v1 = v4;
                            v11 = v41;
                            v12 = v42;
                        }
                    }


                }
            }

        }

    }
}