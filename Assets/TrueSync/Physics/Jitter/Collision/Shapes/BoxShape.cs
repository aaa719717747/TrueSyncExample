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
    /// A <see cref="Shape"/> representing a box.
    /// </summary>
    public class BoxShape : Shape
    {
        internal TSVector size = TSVector.zero;

        /// <summary>
        /// The sidelength of the box.
        /// </summary>
        public TSVector Size { 
            get { return size; }
            set { size = value; UpdateShape(); }
        }
        
        /// <summary>
        /// Creates a new instance of the BoxShape class.
        /// </summary>
        /// <param name="size">The size of the box.</param>
        public BoxShape(TSVector size)
        {
            this.size = size;
            this.UpdateShape();
        }

        /// <summary>
        /// Creates a new instance of the BoxShape class.
        /// </summary>
        /// <param name="length">The length of the box.</param>
        /// <param name="height">The height of the box.</param>
        /// <param name="width">The width of the box</param>
        public BoxShape(FP length, FP height, FP width)
        {
            this.size.x = length;
            this.size.y = height;
            this.size.z = width;
            this.UpdateShape();
        }

        internal TSVector halfSize = TSVector.zero;

        /// <summary>
        /// This method uses the <see cref="ISupportMappable"/> implementation
        /// to calculate the local bounding box, the mass, geometric center and 
        /// the inertia of the shape. In custom shapes this method should be overidden
        /// to compute this values faster.
        /// </summary>
        public override void UpdateShape()
        {
            this.halfSize = size * FP.Half;
            base.UpdateShape();
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override void GetBoundingBox(ref TSMatrix orientation, out TSBBox box)
        {
            TSMatrix abs; TSMath.Absolute(ref orientation, out abs);
            TSVector temp;
            TSVector.Transform(ref halfSize, ref abs, out temp);

            box.max = temp;
            TSVector.Negate(ref temp, out box.min);
        }

        /// <summary>
        /// This method uses the <see cref="ISupportMappable"/> implementation
        /// to calculate the local bounding box, the mass, geometric center and 
        /// the inertia of the shape. In custom shapes this method should be overidden
        /// to compute this values faster.
        /// </summary>
        public override void CalculateMassInertia()
        {
            mass = size.x * size.y * size.z;

            inertia = TSMatrix.Identity;
            inertia.M11 = (FP.One / (12 * FP.One)) * mass * (size.y * size.y + size.z * size.z);
            inertia.M22 = (FP.One / (12 * FP.One)) * mass * (size.x * size.x + size.z * size.z);
            inertia.M33 = (FP.One / (12 * FP.One)) * mass * (size.x * size.x + size.y * size.y);

            this.geomCen = TSVector.zero;
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
            result.x = FP.Sign(direction.x) * halfSize.x;
            result.y = FP.Sign(direction.y) * halfSize.y;
            result.z = FP.Sign(direction.z) * halfSize.z;
        }
    }
}
