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
    /// A <see cref="Shape"/> representing a compoundShape consisting
    /// of several 'sub' shapes.
    /// </summary>
    public class CompoundShape : Multishape
    {
        #region public struct TransformedShape

        /// <summary>
        /// Holds a 'sub' shape and it's transformation. This TransformedShape can
        /// be added to the <see cref="CompoundShape"/>
        /// </summary>
        public struct TransformedShape
        {
            private Shape shape;
            internal TSVector position;
            internal TSMatrix orientation;
            internal TSMatrix invOrientation;
            internal TSBBox boundingBox;

            /// <summary>
            /// The 'sub' shape.
            /// </summary>
            public Shape Shape { get { return shape; } set { shape = value; } }

            /// <summary>
            /// The position of a 'sub' shape
            /// </summary>
            public TSVector Position { get { return position; } set { position = value; } }

            public TSBBox BoundingBox { get { return boundingBox; } }

            /// <summary>
            /// The inverse orientation of the 'sub' shape.
            /// </summary>
            public TSMatrix InverseOrientation
            {
                get { return invOrientation; }
            }

            /// <summary>
            /// The orienation of the 'sub' shape.
            /// </summary>
            public TSMatrix Orientation
            {
                get { return orientation; }
                set { orientation = value; TSMatrix.Transpose(ref orientation, out invOrientation); }
            }

            public void UpdateBoundingBox()
            {
                Shape.GetBoundingBox(ref orientation, out boundingBox);

                boundingBox.min += position;
                boundingBox.max += position;
            }

            /// <summary>
            /// Creates a new instance of the TransformedShape struct.
            /// </summary>
            /// <param name="shape">The shape.</param>
            /// <param name="orientation">The orientation this shape should have.</param>
            /// <param name="position">The position this shape should have.</param>
            public TransformedShape(Shape shape, TSMatrix orientation, TSVector position)
            {
                this.position = position;
                this.orientation = orientation;
                TSMatrix.Transpose(ref orientation, out invOrientation);
                this.shape = shape;
                this.boundingBox = new TSBBox();
                UpdateBoundingBox();
            }
        }
        #endregion

        private TransformedShape[] shapes;

        /// <summary>
        /// An array conaining all 'sub' shapes and their transforms.
        /// </summary>
        public TransformedShape[] Shapes { get { return this.shapes; } }

        TSVector shifted;
        public TSVector Shift { get { return -FP.One * this.shifted; } }

        private TSBBox mInternalBBox;

        /// <summary>
        /// Created a new instance of the CompountShape class.
        /// </summary>
        /// <param name="shapes">The 'sub' shapes which should be added to this 
        /// class.</param>
        public CompoundShape(List<TransformedShape> shapes)
        {
            this.shapes = new TransformedShape[shapes.Count];
            shapes.CopyTo(this.shapes);

            if (!TestValidity()) 
                throw new ArgumentException("Multishapes are not supported!");

            this.UpdateShape();
        }

        public CompoundShape(TransformedShape[] shapes)
        {
            this.shapes = new TransformedShape[shapes.Length];
            Array.Copy(shapes, this.shapes, shapes.Length);

            if (!TestValidity())
                throw new ArgumentException("Multishapes are not supported!");

            this.UpdateShape();
        }

        private bool TestValidity()
        {
            for (int i = 0; i < shapes.Length; i++)
            {
                if (shapes[i].Shape is Multishape) return false;
            }

            return true;
        }

        public override void MakeHull(ref List<TSVector> triangleList, int generationThreshold)
        {
            List<TSVector> triangles = new List<TSVector>();

            for (int i = 0; i < shapes.Length; i++)
            {
                shapes[i].Shape.MakeHull(ref triangles, 4);
                for (int e = 0; e < triangles.Count; e++)
                {
                    TSVector pos = triangles[e];
                    TSVector.Transform(ref pos,ref shapes[i].orientation,out pos);
                    TSVector.Add(ref pos, ref shapes[i].position,out pos);
                    triangleList.Add(pos);
                }
                triangles.Clear();
            }
        }

        /// <summary>
        /// Translate all subshapes in the way that the center of mass is
        /// in (0,0,0)
        /// </summary>
        private void DoShifting()
        {
            for (int i = 0; i < Shapes.Length; i++) shifted += Shapes[i].position;
            shifted *= (FP.One / shapes.Length);

            for (int i = 0; i < Shapes.Length; i++) Shapes[i].position -= shifted;
        }

        public override void CalculateMassInertia()
        {
            base.inertia = TSMatrix.Zero;
            base.mass = FP.Zero;

            for (int i = 0; i < Shapes.Length; i++)
            {
                TSMatrix currentInertia = Shapes[i].InverseOrientation * Shapes[i].Shape.Inertia * Shapes[i].Orientation;
                TSVector p = Shapes[i].Position * -FP.One;
                FP m = Shapes[i].Shape.Mass;

                currentInertia.M11 += m * (p.y * p.y + p.z * p.z);
                currentInertia.M22 += m * (p.x * p.x + p.z * p.z);
                currentInertia.M33 += m * (p.x * p.x + p.y * p.y);

                currentInertia.M12 += -p.x * p.y * m;
                currentInertia.M21 += -p.x * p.y * m;

                currentInertia.M31 += -p.x * p.z * m;
                currentInertia.M13 += -p.x * p.z * m;

                currentInertia.M32 += -p.y * p.z * m;
                currentInertia.M23 += -p.y * p.z * m;

                base.inertia += currentInertia;
                base.mass += m;
            }
        }


        internal CompoundShape()
        {
        }

        protected override Multishape CreateWorkingClone()
        {
            CompoundShape clone = new CompoundShape();
            clone.shapes = this.shapes;
            return clone;
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
            TSVector.Transform(ref direction, ref shapes[currentShape].invOrientation, out result);
            shapes[currentShape].Shape.SupportMapping(ref direction, out result);
            TSVector.Transform(ref result, ref shapes[currentShape].orientation, out result);
            TSVector.Add(ref result, ref shapes[currentShape].position, out result);
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape. (Inlcuding all
        /// 'sub' shapes)
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override void GetBoundingBox(ref TSMatrix orientation, out TSBBox box)
        {
            box.min = mInternalBBox.min;
            box.max = mInternalBBox.max;

            TSVector localHalfExtents = FP.Half * (box.max - box.min);
            TSVector localCenter = FP.Half * (box.max + box.min);

            TSVector center;
            TSVector.Transform(ref localCenter, ref orientation, out center);

            TSMatrix abs; TSMath.Absolute(ref orientation, out abs);
            TSVector temp;
            TSVector.Transform(ref localHalfExtents, ref abs, out temp);

            box.max = center + temp;
            box.min = center - temp;
        }

        int currentShape = 0;
        List<int> currentSubShapes = new List<int>();

        /// <summary>
        /// Sets the current shape. First <see cref="CompoundShape.Prepare"/> has to be called.
        /// After SetCurrentShape the shape immitates another shape.
        /// </summary>
        /// <param name="index"></param>
        public override void SetCurrentShape(int index)
        {
            currentShape = currentSubShapes[index];
            shapes[currentShape].Shape.SupportCenter(out geomCen);
            geomCen += shapes[currentShape].Position;
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
            currentSubShapes.Clear();

            for (int i = 0; i < shapes.Length; i++)
            {
                if (shapes[i].boundingBox.Contains(ref box) != TSBBox.ContainmentType.Disjoint)
                    currentSubShapes.Add(i);
            }

            return currentSubShapes.Count;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayEnd"></param>
        /// <returns></returns>
        public override int Prepare(ref TSVector rayOrigin, ref TSVector rayEnd)
        {
            TSBBox box = TSBBox.SmallBox;

            box.AddPoint(ref rayOrigin);
            box.AddPoint(ref rayEnd);

            return this.Prepare(ref box);
        }


        public override void UpdateShape()
        {
            DoShifting();
            UpdateInternalBoundingBox();
            base.UpdateShape();
        }

        protected void UpdateInternalBoundingBox()
        {
            mInternalBBox.min = new TSVector(FP.MaxValue);
            mInternalBBox.max = new TSVector(FP.MinValue);

            for (int i = 0; i < shapes.Length; i++)
            {
                shapes[i].UpdateBoundingBox();

                TSBBox.CreateMerged(ref mInternalBBox, ref shapes[i].boundingBox, out mInternalBBox);
            }
        }
    }
}
