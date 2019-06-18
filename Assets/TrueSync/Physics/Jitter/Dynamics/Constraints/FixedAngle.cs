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
    // C_1 = R1_x - R2_x
    // C_2 = ...
    // C_3 = ...
    //
    // Derivative:
    //
    // dC_1/dt = w1_x - w2_x
    // dC_2/dt = ...
    // dC_3/dt = ...
    //
    // Jacobian:
    // 
    // dC/dt = J*v+b
    //
    // v = (v1x v1y v1z w1x w1y w1z v2x v2y v2z w2x w2y w2z)^(T) 
    //
    //     v1x v1y v1z w1x w1y w1z v2x v2y v2z w2x w2y w2z
    //     -------------------------------------------------
    // J = 0   0   0   1   0    0   0   0   0   -1   0   0   <- dC_1/dt
    //     0   0   0   0   1    0   0   0   0    0  -1   0   <- ...  
    //     0   0   0   0   0    1   0   0   0    0   0  -1   <- ...
    //
    // Effective Mass:
    //
    // 1/m_eff = [J^T * M^-1 * J] = I1^(-1) + I2^(-1)
    #endregion

    /// <summary>
    /// The AngleConstraint constraints two bodies to always have the same relative
    /// orientation to each other. Combine the AngleConstraint with a PointOnLine
    /// Constraint to get a prismatic joint.
    /// </summary>
    public class FixedAngle : Constraint
    {

        private FP biasFactor = 5 * FP.EN2;
        private FP softness = FP.Zero;

        private TSVector accumulatedImpulse;

        private TSMatrix initialOrientation1, initialOrientation2;

        /// <summary>
        /// Constraints two bodies to always have the same relative
        /// orientation to each other. Combine the AngleConstraint with a PointOnLine
        /// Constraint to get a prismatic joint.
        /// </summary>
        public FixedAngle(RigidBody body1, RigidBody body2) : base(body1, body2)
        {
            initialOrientation1 = body1.orientation;
            initialOrientation2 = body2.orientation;

            //orientationDifference = body1.orientation * body2.invOrientation;
            //orientationDifference = JMatrix.Transpose(orientationDifference);
        }

        public TSVector AppliedImpulse { get { return accumulatedImpulse; } }

        public TSMatrix InitialOrientationBody1 { get { return initialOrientation1; } set { initialOrientation1 = value; } }
        public TSMatrix InitialOrientationBody2 { get { return initialOrientation2; } set { initialOrientation2 = value; } }

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public FP Softness { get { return softness; } set { softness = value; } }

        /// <summary>
        /// Defines how big the applied impulses can get which correct errors.
        /// </summary>
        public FP BiasFactor { get { return biasFactor; } set { biasFactor = value; } }

        TSMatrix effectiveMass;
        TSVector bias;
        FP softnessOverDt;
        
        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The 5simulation timestep</param>
        public override void PrepareForIteration(FP timestep)
        {
            effectiveMass = body1.invInertiaWorld + body2.invInertiaWorld;

            softnessOverDt = softness / timestep;

            effectiveMass.M11 += softnessOverDt;
            effectiveMass.M22 += softnessOverDt;
            effectiveMass.M33 += softnessOverDt;

            TSMatrix.Inverse(ref effectiveMass, out effectiveMass);

            TSMatrix orientationDifference;
            TSMatrix.Multiply(ref initialOrientation1, ref initialOrientation2, out orientationDifference);
            TSMatrix.Transpose(ref orientationDifference, out orientationDifference);

            TSMatrix q = orientationDifference * body2.invOrientation * body1.orientation;
            TSVector axis;

            FP x = q.M32 - q.M23;
            FP y = q.M13 - q.M31;
            FP z = q.M21 - q.M12;

            FP r = TSMath.Sqrt(x * x + y * y + z * z);
            FP t = q.M11 + q.M22 + q.M33;

            FP angle = FP.Atan2(r, t - 1);
            axis = new TSVector(x, y, z) * angle;

            if (r != FP.Zero) axis = axis * (FP.One / r);

            bias = axis * biasFactor * (-FP.One / timestep);

            // Apply previous frame solution as initial guess for satisfying the constraint.
            if (!body1.IsStatic) body1.angularVelocity += TSVector.Transform(accumulatedImpulse, body1.invInertiaWorld);
            if (!body2.IsStatic) body2.angularVelocity += TSVector.Transform(-FP.One * accumulatedImpulse, body2.invInertiaWorld);
        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            TSVector jv = body1.angularVelocity - body2.angularVelocity;

            TSVector softnessVector = accumulatedImpulse * softnessOverDt;

            TSVector lambda = -FP.One * TSVector.Transform(jv+bias+softnessVector, effectiveMass);

            accumulatedImpulse += lambda;

            if(!body1.IsStatic) body1.angularVelocity += TSVector.Transform(lambda, body1.invInertiaWorld);
            if(!body2.IsStatic) body2.angularVelocity += TSVector.Transform(-FP.One * lambda, body2.invInertiaWorld);
        }

    }
}
