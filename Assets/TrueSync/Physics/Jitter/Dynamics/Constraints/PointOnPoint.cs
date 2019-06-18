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

namespace TrueSync.Physics3D {

    public class PointOnPoint : Constraint
    {
        [AddTracking]
        private TSVector localAnchor1;
        [AddTracking]
        private TSVector localAnchor2;

        [AddTracking]
        private TSVector r1;
        [AddTracking]
        private TSVector r2;

        [AddTracking]
        private FP biasFactor = 5 * FP.EN2;
        [AddTracking]
        private FP softness = FP.EN2;

        /// <summary>
        /// Initializes a new instance of the DistanceConstraint class.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <param name="anchor1">The anchor point of the first body in world space. 
        /// The distance is given by the initial distance between both anchor points.</param>
        /// <param name="anchor2">The anchor point of the second body in world space.
        /// The distance is given by the initial distance between both anchor points.</param>
        public PointOnPoint(RigidBody body1, RigidBody body2, TSVector anchor)
            : base(body1, body2)
        {
            TSVector.Subtract(ref anchor, ref body1.position, out localAnchor1);
            TSVector.Subtract(ref anchor, ref body2.position, out localAnchor2);

            TSVector.Transform(ref localAnchor1, ref body1.invOrientation, out localAnchor1);
            TSVector.Transform(ref localAnchor2, ref body2.invOrientation, out localAnchor2);
        }

        public FP AppliedImpulse { get { return accumulatedImpulse; } }

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public FP Softness { get { return softness; } set { softness = value; } }

        /// <summary>
        /// Defines how big the applied impulses can get which correct errors.
        /// </summary>
        public FP BiasFactor { get { return biasFactor; } set { biasFactor = value; } }

        [AddTracking]
        FP effectiveMass = FP.Zero;
        [AddTracking]
        FP accumulatedImpulse = FP.Zero;
        [AddTracking]
        FP bias;
        [AddTracking]
        FP softnessOverDt;

        [AddTracking]
        TSVector[] jacobian = new TSVector[4];

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The 5simulation timestep</param>
        public override void PrepareForIteration(FP timestep)
        {
            TSVector.Transform(ref localAnchor1, ref body1.orientation, out r1);
            TSVector.Transform(ref localAnchor2, ref body2.orientation, out r2);

            TSVector p1, p2, dp;
            TSVector.Add(ref body1.position, ref r1, out p1);
            TSVector.Add(ref body2.position, ref r2, out p2);

            TSVector.Subtract(ref p2, ref p1, out dp);

            FP deltaLength = dp.magnitude;

            TSVector n = p2 - p1;
            if (n.sqrMagnitude != FP.Zero) n.Normalize();

            jacobian[0] = -FP.One * n;
            jacobian[1] = -FP.One * (r1 % n);
            jacobian[2] = FP.One * n;
            jacobian[3] = (r2 % n);

            effectiveMass = body1.inverseMass + body2.inverseMass
                + TSVector.Transform(jacobian[1], body1.invInertiaWorld) * jacobian[1]
                + TSVector.Transform(jacobian[3], body2.invInertiaWorld) * jacobian[3];

            softnessOverDt = softness / timestep;
            effectiveMass += softnessOverDt;

            effectiveMass = FP.One / effectiveMass;

            bias = deltaLength * biasFactor * (FP.One / timestep);

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * accumulatedImpulse * jacobian[0];
                body1.angularVelocity += TSVector.Transform(accumulatedImpulse * jacobian[1], body1.invInertiaWorld);
            }

            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * accumulatedImpulse * jacobian[2];
                body2.angularVelocity += TSVector.Transform(accumulatedImpulse * jacobian[3], body2.invInertiaWorld);
            }


        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            FP jv =
                body1.linearVelocity * jacobian[0] +
                body1.angularVelocity * jacobian[1] +
                body2.linearVelocity * jacobian[2] +
                body2.angularVelocity * jacobian[3];

            FP softnessScalar = accumulatedImpulse * softnessOverDt;

            FP lambda = -effectiveMass * (jv + bias + softnessScalar);

            accumulatedImpulse += lambda;

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * lambda * jacobian[0];
                body1.angularVelocity += TSVector.Transform(lambda * jacobian[1], body1.invInertiaWorld);
            }

            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * lambda * jacobian[2];
                body2.angularVelocity += TSVector.Transform(lambda * jacobian[3], body2.invInertiaWorld);
            }
        }

        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawLine(body1.position, body1.position + r1);
            drawer.DrawLine(body2.position, body2.position + r2);
        }

    }

}
