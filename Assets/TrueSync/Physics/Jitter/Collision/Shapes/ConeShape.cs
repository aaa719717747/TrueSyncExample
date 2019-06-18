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
    /// A <see cref="Shape"/> representing a cone.
    /// </summary>
    public class ConeShape : Shape
    {
        internal FP height,radius;

        /// <summary>
        /// The height of the cone.
        /// </summary>
        public FP Height { get { return height; } set { height = value; UpdateShape(); } }

        /// <summary>
        /// The radius of the cone base.
        /// </summary>
        public FP Radius { get { return radius; } set { radius = value; UpdateShape(); } }

        /// <summary>
        /// Initializes a new instance of the ConeShape class.
        /// </summary>
        /// <param name="height">The height of the cone.</param>
        /// <param name="radius">The radius of the cone base.</param>
        public ConeShape(FP height, FP radius)
        {
            this.height = height;
            this.radius = radius;

            this.UpdateShape();
        }

        public override void UpdateShape()
        {
            sina = radius / FP.Sqrt(radius * radius + height * height);
            base.UpdateShape();
        }

        internal FP sina = FP.Zero;

        /// <summary>
        /// 
        /// </summary>
        public override void CalculateMassInertia()
        {
            mass = (FP.One / (3 * FP.One)) * TSMath.Pi * radius * radius * height;

            // inertia through center of mass axis.
            inertia = TSMatrix.Identity;
            inertia.M11 = (3 * FP.EN1 / 8) * mass * (radius * radius + 4 * height * height);
            inertia.M22 = (3 * FP.EN1) * mass * radius * radius;
            inertia.M33 = (3 * FP.EN1 / 8) * mass * (radius * radius + 4 * height * height);

            // J_x=J_y=3/20 M (R^2+4 H^2)

            // the supportmap center is in the half height, the real geomcenter is:
            geomCen = TSVector.zero;
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
            FP sigma = FP.Sqrt(direction.x * direction.x + direction.z * direction.z);

            if (direction.y > direction.magnitude * sina)
            {
                result.x = FP.Zero;
                result.y =  ((2 * FP.One) / (3)) * height;
                result.z = FP.Zero;
            }
            else if (sigma > FP.Zero)
            {
                result.x = radius * direction.x / sigma;
                result.y = -(FP.One / 3 ) * height;
                result.z = radius * direction.z / sigma;
            }
            else
            {
                result.x = FP.Zero;
                result.y = -(FP.One / 3) * height;
                result.z = FP.Zero;
            }

        }
    }
}
