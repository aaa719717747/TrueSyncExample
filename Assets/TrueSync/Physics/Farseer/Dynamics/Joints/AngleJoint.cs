/*
* Farseer Physics Engine:
* Copyright (c) 2012 Ian Qvist
*/

using System.Diagnostics;

namespace TrueSync.Physics2D
{
    /// <summary>
    /// Maintains a fixed angle between two bodies
    /// </summary>
    public class AngleJoint : Joint2D
    {
        private FP _bias;
        private FP _jointError;
        private FP _massFactor;
        private FP _targetAngle;

        internal AngleJoint()
        {
            JointType = JointType.Angle;
        }

        /// <summary>
        /// Constructor for AngleJoint
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        public AngleJoint(Body bodyA, Body bodyB)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Angle;
            BiasFactor = .2f;
            MaxImpulse = FP.MaxValue;
        }

        public override TSVector2 WorldAnchorA
        {
            get { return BodyA.Position; }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        public override TSVector2 WorldAnchorB
        {
            get { return BodyB.Position; }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        /// <summary>
        /// The desired angle between BodyA and BodyB
        /// </summary>
        public FP TargetAngle
        {
            get { return _targetAngle; }
            set
            {
                if (value != _targetAngle)
                {
                    _targetAngle = value;
                    WakeBodies();
                }
            }
        }

        /// <summary>
        /// Gets or sets the bias factor.
        /// Defaults to 0.2
        /// </summary>
        public FP BiasFactor { get; set; }
        
        /// <summary>
        /// Gets or sets the maximum impulse
        /// Defaults to FP.MaxValue
        /// </summary>
        public FP MaxImpulse { get; set; }
        
        /// <summary>
        /// Gets or sets the softness of the joint
        /// Defaults to 0
        /// </summary>
        public FP Softness { get; set; }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            //TODO
            //return _inv_dt * _impulse;
            return TSVector2.zero;
        }

        public override FP GetReactionTorque(FP invDt)
        {
            return 0;
        }

        internal override void InitVelocityConstraints(ref SolverData data)
        {
            int indexA = BodyA.IslandIndex;
            int indexB = BodyB.IslandIndex;

            FP aW = data.positions[indexA].a;
            FP bW = data.positions[indexB].a;

            _jointError = (bW - aW - TargetAngle);
            _bias = -BiasFactor * data.step.inv_dt * _jointError;
            _massFactor = (1 - Softness) / (BodyA._invI + BodyB._invI);
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            int indexA = BodyA.IslandIndex;
            int indexB = BodyB.IslandIndex;

            FP p = (_bias - data.velocities[indexB].w + data.velocities[indexA].w) * _massFactor;

            data.velocities[indexA].w -= BodyA._invI * FP.Sign(p) * TrueSync.TSMath.Min(FP.Abs(p), MaxImpulse);
            data.velocities[indexB].w += BodyB._invI * FP.Sign(p) * TrueSync.TSMath.Min(FP.Abs(p), MaxImpulse);
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            //no position solving for this joint
            return true;
        }
    }
}