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

    #region Constraint Equations
    // Constraint formulation:
    // 
    // C = |(p1-p2) x l|
    //
    #endregion

    /// <summary>
    /// Constraints a point on a body to be fixed on a line
    /// which is fixed on another body.
    /// </summary>
    public class PointOnLine : Constraint
    {
        private TSVector lineNormal;

        private TSVector localAnchor1, localAnchor2;
        private TSVector r1, r2;

        private FP biasFactor = FP.Half;
        private FP softness = FP.Zero;

        /// <summary>
        /// Constraints a point on a body to be fixed on a line
        /// which is fixed on another body.
        /// </summary>
        /// <param name="body1"></param>
        /// <param name="body2"></param>
        /// <param name="lineStartPointBody1"></param>
        /// <param name="lineDirection"></param>
        /// <param name="pointBody2"></param>
        public PointOnLine(RigidBody body1, RigidBody body2,
            TSVector lineStartPointBody1, TSVector pointBody2) : base(body1,body2)
        {

            TSVector.Subtract(ref lineStartPointBody1, ref body1.position, out localAnchor1);
            TSVector.Subtract(ref pointBody2, ref body2.position, out localAnchor2);

            TSVector.Transform(ref localAnchor1, ref body1.invOrientation, out localAnchor1);
            TSVector.Transform(ref localAnchor2, ref body2.invOrientation, out localAnchor2);

            lineNormal = TSVector.Normalize(lineStartPointBody1 - pointBody2);
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

        FP effectiveMass = FP.Zero;
        FP accumulatedImpulse = FP.Zero;
        FP bias;
        FP softnessOverDt;

        TSVector[] jacobian = new TSVector[4];

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The simulation timestep</param>
        public override void PrepareForIteration(FP timestep)
        {
            TSVector.Transform(ref localAnchor1, ref body1.orientation, out r1);
            TSVector.Transform(ref localAnchor2, ref body2.orientation, out r2);

            TSVector p1, p2, dp;
            TSVector.Add(ref body1.position, ref r1, out p1);
            TSVector.Add(ref body2.position, ref r2, out p2);

            TSVector.Subtract(ref p2, ref p1, out dp);

            TSVector l = TSVector.Transform(lineNormal, body1.orientation);
            l.Normalize();

            TSVector t = (p1 - p2) % l;
            if(t.sqrMagnitude != FP.Zero) t.Normalize();
            t = t % l;

            jacobian[0] = t;                      // linearVel Body1
            jacobian[1] = (r1 + p2 - p1) % t;     // angularVel Body1
            jacobian[2] = -FP.One * t;              // linearVel Body2
            jacobian[3] = -FP.One * r2 % t;         // angularVel Body2

            effectiveMass = body1.inverseMass + body2.inverseMass
                + TSVector.Transform(jacobian[1], body1.invInertiaWorld) * jacobian[1]
                + TSVector.Transform(jacobian[3], body2.invInertiaWorld) * jacobian[3];

            softnessOverDt = softness / timestep;
            effectiveMass += softnessOverDt;

            if(effectiveMass != 0) effectiveMass = FP.One / effectiveMass;

            bias = - (l % (p2-p1)).magnitude * biasFactor * (FP.One / timestep);

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
            drawer.DrawLine(body1.position + r1,
                body1.position + r1 + TSVector.Transform(lineNormal, body1.orientation) * 100);
        }

    }
}
