/*
* Farseer Physics Engine:
* Copyright (c) 2012 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org 
* 
* This software is provided 'as-is', without any express or implied 
* warranty.  In no event will the authors be held liable for any damages 
* arising from the use of this software. 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it 
* freely, subject to the following restrictions: 
* 1. The origin of this software must not be misrepresented; you must not 
* claim that you wrote the original software. If you use this software 
* in a product, an acknowledgment in the product documentation would be 
* appreciated but is not required. 
* 2. Altered source versions must be plainly marked as such, and must not be 
* misrepresented as being the original software. 
* 3. This notice may not be removed or altered from any source distribution. 
*/
#pragma warning disable 0162

namespace TrueSync.Physics2D
{
    // Limit:
    // C = norm(pB - pA) - L
    // u = (pB - pA) / norm(pB - pA)
    // Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
    // J = [-u -cross(rA, u) u cross(rB, u)]
    // K = J * invM * JT
    //   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

    /// <summary>
    /// A rope joint enforces a maximum distance between two points on two bodies. It has no other effect.
    /// It can be used on ropes that are made up of several connected bodies, and if there is a need to support a heavy body.
    /// This joint is used for stabiliation of heavy objects on soft constraint joints.
    /// 
    /// Warning: if you attempt to change the maximum length during the simulation you will get some non-physical behavior.
    /// Use the DistanceJoint instead if you want to dynamically control the length.
    /// </summary>
    public class RopeJoint : Joint2D
    {
        // Solver shared
        private FP _impulse;
        private FP _length;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private TSVector2 _localCenterA;
        private TSVector2 _localCenterB;
        private FP _invMassA;
        private FP _invMassB;
        private FP _invIA;
        private FP _invIB;
        private FP _mass;
        private TSVector2 _rA, _rB;
        private TSVector2 _u;

        internal RopeJoint()
        {
            JointType = JointType.Rope;
        }

        /// <summary>
        /// Constructor for RopeJoint.
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="anchorA">The anchor on the first body</param>
        /// <param name="anchorB">The anchor on the second body</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public RopeJoint(Body bodyA, Body bodyB, TSVector2 anchorA, TSVector2 anchorB, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Rope;

            if (useWorldCoordinates)
            {
                LocalAnchorA = bodyA.GetLocalPoint(anchorA);
                LocalAnchorB = bodyB.GetLocalPoint(anchorB);
            }
            else
            {
                LocalAnchorA = anchorA;
                LocalAnchorB = anchorB;
            }

            //FPE feature: Setting default MaxLength
            TSVector2 d = WorldAnchorB - WorldAnchorA;
            MaxLength = d.magnitude;
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public TSVector2 LocalAnchorA { get; set; }

        /// <summary>
        /// The local anchor point on BodyB
        /// </summary>
        public TSVector2 LocalAnchorB { get; set; }

        public override sealed TSVector2 WorldAnchorA
        {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
            set { LocalAnchorA = BodyA.GetLocalPoint(value); }
        }

        public override sealed TSVector2 WorldAnchorB
        {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { LocalAnchorB = BodyB.GetLocalPoint(value); }
        }

        /// <summary>
        /// Get or set the maximum length of the rope.
        /// By default, it is the distance between the two anchor points.
        /// </summary>
        public FP MaxLength { get; set; }

        /// <summary>
        /// Gets the state of the joint.
        /// </summary>
        public LimitState State { get; private set; }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            return (invDt * _impulse) * _u;
        }

        public override FP GetReactionTorque(FP invDt)
        {
            return 0;
        }

        internal override void InitVelocityConstraints(ref SolverData data)
        {
            _indexA = BodyA.IslandIndex;
            _indexB = BodyB.IslandIndex;
            _localCenterA = BodyA._sweep.LocalCenter;
            _localCenterB = BodyB._sweep.LocalCenter;
            _invMassA = BodyA._invMass;
            _invMassB = BodyB._invMass;
            _invIA = BodyA._invI;
            _invIB = BodyB._invI;

            TSVector2 cA = data.positions[_indexA].c;
            FP aA = data.positions[_indexA].a;
            TSVector2 vA = data.velocities[_indexA].v;
            FP wA = data.velocities[_indexA].w;

            TSVector2 cB = data.positions[_indexB].c;
            FP aB = data.positions[_indexB].a;
            TSVector2 vB = data.velocities[_indexB].v;
            FP wB = data.velocities[_indexB].w;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            _rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            _rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            _u = cB + _rB - cA - _rA;

            _length = _u.magnitude;

            FP C = _length - MaxLength;
            if (C > 0.0f)
            {
                State = LimitState.AtUpper;
            }
            else
            {
                State = LimitState.Inactive;
            }

            if (_length > Settings.LinearSlop)
            {
                _u *= 1.0f / _length;
            }
            else
            {
                _u = TSVector2.zero;
                _mass = 0.0f;
                _impulse = 0.0f;
                return;
            }

            // Compute effective mass.
            FP crA = MathUtils.Cross(_rA, _u);
            FP crB = MathUtils.Cross(_rB, _u);
            FP invMass = _invMassA + _invIA * crA * crA + _invMassB + _invIB * crB * crB;

            _mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

            if (Settings.EnableWarmstarting)
            {
                // Scale the impulse to support a variable time step.
                _impulse *= data.step.dtRatio;

                TSVector2 P = _impulse * _u;
                vA -= _invMassA * P;
                wA -= _invIA * MathUtils.Cross(_rA, P);
                vB += _invMassB * P;
                wB += _invIB * MathUtils.Cross(_rB, P);
            }
            else
            {
                _impulse = 0.0f;
            }

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            TSVector2 vA = data.velocities[_indexA].v;
            FP wA = data.velocities[_indexA].w;
            TSVector2 vB = data.velocities[_indexB].v;
            FP wB = data.velocities[_indexB].w;

            // Cdot = dot(u, v + cross(w, r))
            TSVector2 vpA = vA + MathUtils.Cross(wA, _rA);
            TSVector2 vpB = vB + MathUtils.Cross(wB, _rB);
            FP C = _length - MaxLength;
            FP Cdot = TSVector2.Dot(_u, vpB - vpA);

            // Predictive constraint.
            if (C < 0.0f)
            {
                Cdot += data.step.inv_dt * C;
            }

            FP impulse = -_mass * Cdot;
            FP oldImpulse = _impulse;
            _impulse = TrueSync.TSMath.Min(0.0f, _impulse + impulse);
            impulse = _impulse - oldImpulse;

            TSVector2 P = impulse * _u;
            vA -= _invMassA * P;
            wA -= _invIA * MathUtils.Cross(_rA, P);
            vB += _invMassB * P;
            wB += _invIB * MathUtils.Cross(_rB, P);

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            TSVector2 cA = data.positions[_indexA].c;
            FP aA = data.positions[_indexA].a;
            TSVector2 cB = data.positions[_indexB].c;
            FP aB = data.positions[_indexB].a;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            TSVector2 u = cB + rB - cA - rA;

            FP length = u.magnitude; u.Normalize();
            FP C = length - MaxLength;

            C = MathUtils.Clamp(C, 0.0f, Settings.MaxLinearCorrection);

            FP impulse = -_mass * C;
            TSVector2 P = impulse * u;

            cA -= _invMassA * P;
            aA -= _invIA * MathUtils.Cross(rA, P);
            cB += _invMassB * P;
            aB += _invIB * MathUtils.Cross(rB, P);

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return length - MaxLength < Settings.LinearSlop;
        }
    }
}