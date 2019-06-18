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
    /// Represents a terrain.
    /// </summary>
    public class TerrainShape : Multishape
    {
        private FP[,] heights;
        private FP scaleX, scaleZ;
        private int heightsLength0, heightsLength1;

        private int minX, maxX;
        private int minZ, maxZ;
        private int numX, numZ;

        private TSBBox boundings;

        private FP sphericalExpansion = 5 * FP.EN2;

        /// <summary>
        /// Expands the triangles by the specified amount.
        /// This stabilizes collision detection for flat shapes.
        /// </summary>
        public FP SphericalExpansion
        {
            get { return sphericalExpansion; }
            set { sphericalExpansion = value; }
        }

        /// <summary>
        /// Initializes a new instance of the TerrainShape class.
        /// </summary>
        /// <param name="heights">An array containing the heights of the terrain surface.</param>
        /// <param name="scaleX">The x-scale factor. (The x-space between neighbour heights)</param>
        /// <param name="scaleZ">The y-scale factor. (The y-space between neighbour heights)</param>
        public TerrainShape(FP[,] heights, FP scaleX, FP scaleZ)
        {
            heightsLength0 = heights.GetLength(0);
            heightsLength1 = heights.GetLength(1);

            #region Bounding Box
            boundings = TSBBox.SmallBox;

            for (int i = 0; i < heightsLength0; i++)
            {
                for (int e = 0; e < heightsLength1; e++)
                {
                    if (heights[i, e] > boundings.max.y)
                        boundings.max.y = heights[i, e];
                    else if (heights[i, e] < boundings.min.y)
                        boundings.min.y = heights[i, e];
                }
            }

            boundings.min.x = FP.Zero;
            boundings.min.z = FP.Zero;

            boundings.max.x = checked(heightsLength0 * scaleX);
            boundings.max.z = checked(heightsLength1 * scaleZ);

            #endregion

            this.heights = heights;
            this.scaleX = scaleX;
            this.scaleZ = scaleZ;

            UpdateShape();
        }

        internal TerrainShape() { }

 
        protected override Multishape CreateWorkingClone()
        {
            TerrainShape clone = new TerrainShape();
            clone.heights = this.heights;
            clone.scaleX = this.scaleX;
            clone.scaleZ = this.scaleZ;
            clone.boundings = this.boundings;
            clone.heightsLength0 = this.heightsLength0;
            clone.heightsLength1 = this.heightsLength1;
            clone.sphericalExpansion = this.sphericalExpansion;
            return clone;
        }


        private TSVector[] points = new TSVector[3];
        private TSVector normal = TSVector.up;

        /// <summary>
        /// Sets the current shape. First <see cref="Prepare"/> has to be called.
        /// After SetCurrentShape the shape immitates another shape.
        /// </summary>
        /// <param name="index"></param>
        public override void SetCurrentShape(int index)
        {
            bool leftTriangle = false;

            if (index >= numX * numZ)
            {
                leftTriangle = true;
                index -= numX * numZ;
            }

            int quadIndexX = index % numX;
            int quadIndexZ = index / numX;

            // each quad has two triangles, called 'leftTriangle' and !'leftTriangle'
            if (leftTriangle)
            {
                points[0].Set((minX + quadIndexX + 0) * scaleX, heights[minX + quadIndexX + 0, minZ + quadIndexZ + 0], (minZ + quadIndexZ + 0) * scaleZ);
                points[1].Set((minX + quadIndexX + 1) * scaleX, heights[minX + quadIndexX + 1, minZ + quadIndexZ + 0], (minZ + quadIndexZ + 0) * scaleZ);
                points[2].Set((minX + quadIndexX + 0) * scaleX, heights[minX + quadIndexX + 0, minZ + quadIndexZ + 1], (minZ + quadIndexZ + 1) * scaleZ);
            }
            else
            {
                points[0].Set((minX + quadIndexX + 1) * scaleX, heights[minX + quadIndexX + 1, minZ + quadIndexZ + 0], (minZ + quadIndexZ + 0) * scaleZ);
                points[1].Set((minX + quadIndexX + 1) * scaleX, heights[minX + quadIndexX + 1, minZ + quadIndexZ + 1], (minZ + quadIndexZ + 1) * scaleZ);
                points[2].Set((minX + quadIndexX + 0) * scaleX, heights[minX + quadIndexX + 0, minZ + quadIndexZ + 1], (minZ + quadIndexZ + 1) * scaleZ);
            }

            TSVector sum = points[0];
            TSVector.Add(ref sum, ref points[1], out sum);
            TSVector.Add(ref sum, ref points[2], out sum);
            TSVector.Multiply(ref sum, FP.One / (3 * FP.One), out sum);
            geomCen = sum;

            TSVector.Subtract(ref points[1], ref points[0], out sum);
            TSVector.Subtract(ref points[2], ref points[0], out normal);
            TSVector.Cross(ref sum, ref normal, out normal);
        }

        public void CollisionNormal(out TSVector normal)
        {
            normal = this.normal;
        }


        /// <summary>
        /// Passes a axis aligned bounding box to the shape where collision
        /// could occour.
        /// </summary>
        /// <param name="box">The bounding box where collision could occur.</param>
        /// <returns>The upper index with which <see cref="SetCurrentShape"/> can be 
        /// called.</returns>
        public override int Prepare(ref TSBBox box)
        {
            // simple idea: the terrain is a grid. x and z is the position in the grid.
            // y the height. we know compute the min and max grid-points. All quads
            // between these points have to be checked.

            // including overflow exception prevention

            if (box.min.x < boundings.min.x) minX = 0;
            else
            {
                minX = (int)FP.Floor(((box.min.x - sphericalExpansion) / scaleX));
                minX = (int)TSMath.Max(minX, 0);
            }

            if (box.max.x > boundings.max.x) maxX = heightsLength0 - 1;
            else
            {
				maxX = (int)FP.Ceiling(((box.max.x + sphericalExpansion) / scaleX));
				maxX = (int)TSMath.Min(maxX, heightsLength0 - 1);
            }

            if (box.min.z < boundings.min.z) minZ = 0;
            else
            {
				minZ = (int)FP.Floor(((box.min.z - sphericalExpansion) / scaleZ));
				minZ = (int)TSMath.Max(minZ, 0);
            }

            if (box.max.z > boundings.max.z) maxZ = heightsLength1 - 1;
            else
            {
				maxZ = (int)FP.Ceiling((FP)((box.max.z + sphericalExpansion) / scaleZ));
				maxZ = (int)TSMath.Min(maxZ, heightsLength1 - 1);
            }

            numX = maxX - minX;
            numZ = maxZ - minZ;

            // since every quad contains two triangles we multiply by 2.
            return numX * numZ * 2;
        }

        /// <summary>
        /// 
        /// </summary>
        public override void CalculateMassInertia()
        {
            this.inertia = TSMatrix.Identity;
            this.Mass = FP.One;
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape. This includes
        /// the whole shape.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override void GetBoundingBox(ref TSMatrix orientation, out TSBBox box)
        {
            box = boundings;

            #region Expand Spherical
            box.min.x -= sphericalExpansion;
            box.min.y -= sphericalExpansion;
            box.min.z -= sphericalExpansion;
            box.max.x += sphericalExpansion;
            box.max.y += sphericalExpansion;
            box.max.z += sphericalExpansion;
            #endregion

            box.Transform(ref orientation);
        }

        public override void MakeHull(ref List<TSVector> triangleList, int generationThreshold)
        {
            for (int index = 0; index < (heightsLength0 - 1) * (heightsLength1 - 1); index++)
            {
                int quadIndexX = index % (heightsLength0 - 1);
                int quadIndexZ = index / (heightsLength0 - 1);

                triangleList.Add(new TSVector((0 + quadIndexX + 0) * scaleX, heights[0 + quadIndexX + 0, 0 + quadIndexZ + 0], (0 + quadIndexZ + 0) * scaleZ));
                triangleList.Add(new TSVector((0 + quadIndexX + 1) * scaleX, heights[0 + quadIndexX + 1, 0 + quadIndexZ + 0], (0 + quadIndexZ + 0) * scaleZ));
                triangleList.Add(new TSVector((0 + quadIndexX + 0) * scaleX, heights[0 + quadIndexX + 0, 0 + quadIndexZ + 1], (0 + quadIndexZ + 1) * scaleZ));

                triangleList.Add(new TSVector((0 + quadIndexX + 1) * scaleX, heights[0 + quadIndexX + 1, 0 + quadIndexZ + 0], (0 + quadIndexZ + 0) * scaleZ));
                triangleList.Add(new TSVector((0 + quadIndexX + 1) * scaleX, heights[0 + quadIndexX + 1, 0 + quadIndexZ + 1], (0 + quadIndexZ + 1) * scaleZ));
                triangleList.Add(new TSVector((0 + quadIndexX + 0) * scaleX, heights[0 + quadIndexX + 0, 0 + quadIndexZ + 1], (0 + quadIndexZ + 1) * scaleZ));
            }
        }

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override void SupportMapping(ref TSVector direction, out TSVector result)
        {
            TSVector expandVector;
            TSVector.Normalize(ref direction, out expandVector);
            TSVector.Multiply(ref expandVector, sphericalExpansion, out expandVector);

            int minIndex = 0;
            FP min = TSVector.Dot(ref points[0], ref direction);
            FP dot = TSVector.Dot(ref points[1], ref direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 1;
            }
            dot = TSVector.Dot(ref points[2], ref direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 2;
            }

            TSVector.Add(ref points[minIndex], ref expandVector, out result);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayDelta"></param>
        /// <returns></returns>
        public override int Prepare(ref TSVector rayOrigin, ref TSVector rayDelta)
        {
            TSBBox box = TSBBox.SmallBox;

            #region RayEnd + Expand Spherical
            TSVector rayEnd;
            TSVector.Normalize(ref rayDelta, out rayEnd);
            rayEnd = rayOrigin + rayDelta + rayEnd * sphericalExpansion;
            #endregion

            box.AddPoint(ref rayOrigin);
            box.AddPoint(ref rayEnd);

            return this.Prepare(ref box);
        }
    }
}
