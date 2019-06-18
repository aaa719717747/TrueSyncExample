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

using System.Diagnostics;

namespace TrueSync.Physics2D
{
    // Pulley:
    // length1 = norm(p1 - s1)
    // length2 = norm(p2 - s2)
    // C0 = (length1 + ratio * length2)_initial
    // C = C0 - (length1 + ratio * length2)
    // u1 = (p1 - s1) / norm(p1 - s1)
    // u2 = (p2 - s2) / norm(p2 - s2)
    // Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
    // J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
    // K = J * invM * JT
    //   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

    /// <summary>
    /// The pulley joint is connected to two bodies and two fixed world points.
    /// The pulley supports a ratio such that:
    /// <![CDATA[length1 + ratio * length2 <= constant]]>
    /// Yes, the force transmitted is scaled by the ratio.
    /// 
    /// Warning: the pulley joint can get a bit squirrelly by itself. They often
    /// work better when combined with prismatic joints. You should also cover the
    /// the anchor points with static shapes to prevent one side from going to zero length.
    /// </summary>
    public class PulleyJoint : Joint2D
    {
        // Solver shared
        private FP _impulse;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private TSVector2 _uA;
        private TSVector2 _uB;
        private TSVector2 _rA;
        private TSVector2 _rB;
        private TSVector2 _localCenterA;
        private TSVector2 _localCenterB;
        private FP _invMassA;
        private FP _invMassB;
        private FP _invIA;
        private FP _invIB;
        private FP _mass;

        internal PulleyJoint()
        {
            JointType = JointType.Pulley;
        }

        /// <summary>
        /// Constructor for PulleyJoint.
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="anchorA">The anchor on the first body.</param>
        /// <param name="anchorB">The anchor on the second body.</param>
        /// <param name="worldAnchorA">The world anchor for the first body.</param>
        /// <param name="worldAnchorB">The world anchor for the second body.</param>
        /// <param name="ratio">The ratio.</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public PulleyJoint(Body bodyA, Body bodyB, TSVector2 anchorA, TSVector2 anchorB, TSVector2 worldAnchorA, TSVector2 worldAnchorB, FP ratio, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Pulley;

            WorldAnchorA = worldAnchorA;
            WorldAnchorB = worldAnchorB;

            if (useWorldCoordinates)
            {
                LocalAnchorA = BodyA.GetLocalPoint(anchorA);
                LocalAnchorB = BodyB.GetLocalPoint(anchorB);

                TSVector2 dA = anchorA - worldAnchorA;
                LengthA = dA.magnitude;
                TSVector2 dB = anchorB - worldAnchorB;
                LengthB = dB.magnitude;
            }
            else
            {
                LocalAnchorA = anchorA;
                LocalAnchorB = anchorB;

                TSVector2 dA = anchorA - BodyA.GetLocalPoint(worldAnchorA);
                LengthA = dA.magnitude;
                TSVector2 dB = anchorB - BodyB.GetLocalPoint(worldAnchorB);
                LengthB = dB.magnitude;
            }

            Debug.Assert(ratio != 0.0f);
            Debug.Assert(ratio > Settings.Epsilon);

            Ratio = ratio;
            Constant = LengthA + ratio * LengthB;
            _impulse = 0.0f;
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public TSVector2 LocalAnchorA { get; set; }

        /// <summary>
        /// The local anchor point on BodyB
        /// </summary>
        public TSVector2 LocalAnchorB { get; set; }

        /// <summary>
        /// Get the first world anchor.
        /// </summary>
        /// <value></value>
        public override sealed TSVector2 WorldAnchorA { get; set; }

        /// <summary>
        /// Get the second world anchor.
        /// </summary>
        /// <value></value>
        public override sealed TSVector2 WorldAnchorB { get; set; }

        /// <summary>
        /// Get the current length of the segment attached to body1.
        /// </summary>
        /// <value></value>
        public FP LengthA { get; set; }

        /// <summary>
        /// Get the current length of the segment attached to body2.
        /// </summary>
        /// <value></value>
        public FP LengthB { get; set; }

        /// <summary>
        /// The current length between the anchor point on BodyA and WorldAnchorA
        /// </summary>
        public FP CurrentLengthA
        {
            get
            {
                TSVector2 p = BodyA.GetWorldPoint(LocalAnchorA);
                TSVector2 s = WorldAnchorA;
                TSVector2 d = p - s;
                return d.magnitude;
            }
        }

        /// <summary>
        /// The current length between the anchor point on BodyB and WorldAnchorB
        /// </summary>
        public FP CurrentLengthB
        {
            get
            {
                TSVector2 p = BodyB.GetWorldPoint(LocalAnchorB);
                TSVector2 s = WorldAnchorB;
                TSVector2 d = p - s;
                return d.magnitude;
            }
        }

        /// <summary>
        /// Get the pulley ratio.
        /// </summary>
        /// <value></value>
        public FP Ratio { get; set; }

        //FPE note: Only used for serialization.
        internal FP Constant { get; set; }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            TSVector2 P = _impulse * _uB;
            return invDt * P;
        }

        public override FP GetReactionTorque(FP invDt)
        {
            return 0.0f;
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

            // Get the pulley axes.
            _uA = cA + _rA - WorldAnchorA;
            _uB = cB + _rB - WorldAnchorB;

            FP lengthA = _uA.magnitude;
            FP lengthB = _uB.magnitude;

            if (lengthA > 10.0f * Settings.LinearSlop)
            {
                _uA *= 1.0f / lengthA;
            }
            else
            {
                _uA = TSVector2.zero;
            }

            if (lengthB > 10.0f * Settings.LinearSlop)
            {
                _uB *= 1.0f / lengthB;
            }
            else
            {
                _uB = TSVector2.zero;
            }

            // Compute effective mass.
            FP ruA = MathUtils.Cross(_rA, _uA);
            FP ruB = MathUtils.Cross(_rB, _uB);

            FP mA = _invMassA + _invIA * ruA * ruA;
            FP mB = _invMassB + _invIB * ruB * ruB;

            _mass = mA + Ratio * Ratio * mB;

            if (_mass > 0.0f)
            {
                _mass = 1.0f / _mass;
            }

            if (Settings.EnableWarmstarting)
            {
                // Scale impulses to support variable time steps.
                _impulse *= data.step.dtRatio;

                // Warm starting.
                TSVector2 PA = -(_impulse) * _uA;
                TSVector2 PB = (-Ratio * _impulse) * _uB;

                vA += _invMassA * PA;
                wA += _invIA * MathUtils.Cross(_rA, PA);
                vB += _invMassB * PB;
                wB += _invIB * MathUtils.Cross(_rB, PB);
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

            TSVector2 vpA = vA + MathUtils.Cross(wA, _rA);
            TSVector2 vpB = vB + MathUtils.Cross(wB, _rB);

            FP Cdot = -TSVector2.Dot(_uA, vpA) - Ratio * TSVector2.Dot(_uB, vpB);
            FP impulse = -_mass * Cdot;
            _impulse += impulse;

            TSVector2 PA = -impulse * _uA;
            TSVector2 PB = -Ratio * impulse * _uB;
            vA += _invMassA * PA;
            wA += _invIA * MathUtils.Cross(_rA, PA);
            vB += _invMassB * PB;
            wB += _invIB * MathUtils.Cross(_rB, PB);

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

            // Get the pulley axes.
            TSVector2 uA = cA + rA - WorldAnchorA;
            TSVector2 uB = cB + rB - WorldAnchorB;

            FP lengthA = uA.magnitude;
            FP lengthB = uB.magnitude;

            if (lengthA > 10.0f * Settings.LinearSlop)
            {
                uA *= 1.0f / lengthA;
            }
            else
            {
                uA = TSVector2.zero;
            }

            if (lengthB > 10.0f * Settings.LinearSlop)
            {
                uB *= 1.0f / lengthB;
            }
            else
            {
                uB = TSVector2.zero;
            }

            // Compute effective mass.
            FP ruA = MathUtils.Cross(rA, uA);
            FP ruB = MathUtils.Cross(rB, uB);

            FP mA = _invMassA + _invIA * ruA * ruA;
            FP mB = _invMassB + _invIB * ruB * ruB;

            FP mass = mA + Ratio * Ratio * mB;

            if (mass > 0.0f)
            {
                mass = 1.0f / mass;
            }

            FP C = Constant - lengthA - Ratio * lengthB;
            FP linearError = FP.Abs(C);

            FP impulse = -mass * C;

            TSVector2 PA = -impulse * uA;
            TSVector2 PB = -Ratio * impulse * uB;

            cA += _invMassA * PA;
            aA += _invIA * MathUtils.Cross(rA, PA);
            cB += _invMassB * PB;
            aB += _invIB * MathUtils.Cross(rB, PB);

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return linearError < Settings.LinearSlop;
        }
    }
}