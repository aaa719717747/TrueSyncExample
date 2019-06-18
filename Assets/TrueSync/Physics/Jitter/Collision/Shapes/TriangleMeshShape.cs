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
    /// A <see cref="Shape"/> representing a triangleMesh.
    /// </summary>
    public class TriangleMeshShape : Multishape
    {
        private List<int> potentialTriangles = new List<int>();
        private Octree octree = null;

        private FP sphericalExpansion = FP.EN2;

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
        /// Creates a new istance if the TriangleMeshShape class.
        /// </summary>
        /// <param name="octree">The octree which holds the triangles
        /// of a mesh.</param>
        public TriangleMeshShape(Octree octree)
        {
            this.octree = octree;
            UpdateShape();
        }

        internal TriangleMeshShape() { }

 
        protected override Multishape CreateWorkingClone()
        {
            TriangleMeshShape clone = new TriangleMeshShape(this.octree);
            clone.sphericalExpansion = this.sphericalExpansion;
            return clone;
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
            potentialTriangles.Clear();

            #region Expand Spherical
            TSBBox exp = box;

            exp.min.x -= sphericalExpansion;
            exp.min.y -= sphericalExpansion;
            exp.min.z -= sphericalExpansion;
            exp.max.x += sphericalExpansion;
            exp.max.y += sphericalExpansion;
            exp.max.z += sphericalExpansion;
            #endregion

            octree.GetTrianglesIntersectingtAABox(potentialTriangles, ref exp);

            return potentialTriangles.Count;
        }

        public override void MakeHull(ref List<TSVector> triangleList, int generationThreshold)
        {
            TSBBox large = TSBBox.LargeBox;

            List<int> indices = new List<int>();
            octree.GetTrianglesIntersectingtAABox(indices, ref large);

            for (int i = 0; i < indices.Count; i++)
            {
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I0));
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I1));
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I2));
            }

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayDelta"></param>
        /// <returns></returns>
        public override int Prepare(ref TSVector rayOrigin, ref TSVector rayDelta)
        {
            potentialTriangles.Clear();

            #region Expand Spherical
            TSVector expDelta;
            TSVector.Normalize(ref rayDelta, out expDelta);
            expDelta = rayDelta + expDelta * sphericalExpansion;
            #endregion

            octree.GetTrianglesIntersectingRay(potentialTriangles, rayOrigin, expDelta);

            return potentialTriangles.Count;
        }

        TSVector[] vecs = new TSVector[3];

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override void SupportMapping(ref TSVector direction, out TSVector result)
        {
            TSVector exp;
            TSVector.Normalize(ref direction, out exp);
            exp *= sphericalExpansion;

            FP min = TSVector.Dot(ref vecs[0], ref direction);
            int minIndex = 0;
            FP dot = TSVector.Dot(ref vecs[1], ref direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 1;
            }
            dot = TSVector.Dot(ref vecs[2], ref direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 2;
            }

            result = vecs[minIndex] + exp;
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape. This includes
        /// the whole shape.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override void GetBoundingBox(ref TSMatrix orientation, out TSBBox box)
        {
            box = octree.rootNodeBox;

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

        private bool flipNormal = false;
        public bool FlipNormals { get { return flipNormal; } set { flipNormal = value; } }

        /// <summary>
        /// Sets the current shape. First <see cref="Prepare"/> has to be called.
        /// After SetCurrentShape the shape immitates another shape.
        /// </summary>
        /// <param name="index"></param>
        public override void SetCurrentShape(int index)
        {
            vecs[0] = octree.GetVertex(octree.tris[potentialTriangles[index]].I0);
            vecs[1] = octree.GetVertex(octree.tris[potentialTriangles[index]].I1);
            vecs[2] = octree.GetVertex(octree.tris[potentialTriangles[index]].I2);

            TSVector sum = vecs[0];
            TSVector.Add(ref sum, ref vecs[1], out sum);
            TSVector.Add(ref sum, ref vecs[2], out sum);
            TSVector.Multiply(ref sum, FP.One / (3 * FP.One), out sum);

      
            geomCen = sum;

            TSVector.Subtract(ref vecs[1], ref vecs[0], out sum);
            TSVector.Subtract(ref vecs[2], ref vecs[0], out normal);
            TSVector.Cross(ref sum, ref normal, out normal);
            normal.Normalize();
            if (flipNormal) normal.Negate();
        }

        private TSVector normal = TSVector.up;

        public void CollisionNormal(out TSVector normal)
        {
            normal = this.normal;
        }
    }

}
