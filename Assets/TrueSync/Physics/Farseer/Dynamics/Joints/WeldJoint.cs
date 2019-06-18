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
    // Point-to-point constraint
    // C = p2 - p1
    // Cdot = v2 - v1
    //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    // J = [-I -r1_skew I r2_skew ]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    // Angle constraint
    // C = angle2 - angle1 - referenceAngle
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    // K = invI1 + invI2

    /// <summary>
    /// A weld joint essentially glues two bodies together. A weld joint may
    /// distort somewhat because the island constraint solver is approximate.
    /// 
    /// The joint is soft constraint based, which means the two bodies will move
    /// relative to each other, when a force is applied. To combine two bodies
    /// in a rigid fashion, combine the fixtures to a single body instead.
    /// </summary>
    public class WeldJoint : Joint2D
    {
        // Solver shared
        private TSVector _impulse;
        private FP _gamma;
        private FP _bias;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private TSVector2 _rA;
        private TSVector2 _rB;
        private TSVector2 _localCenterA;
        private TSVector2 _localCenterB;
        private FP _invMassA;
        private FP _invMassB;
        private FP _invIA;
        private FP _invIB;
        private Mat33 _mass;

        internal WeldJoint()
        {
            JointType = JointType.Weld;
        }

        /// <summary>
        /// You need to specify an anchor point where they are attached.
        /// The position of the anchor point is important for computing the reaction torque.
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="anchorA">The first body anchor.</param>
        /// <param name="anchorB">The second body anchor.</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public WeldJoint(Body bodyA, Body bodyB, TSVector2 anchorA, TSVector2 anchorB, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Weld;

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

            ReferenceAngle = BodyB.Rotation - BodyA.Rotation;
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public TSVector2 LocalAnchorA { get; set; }

        /// <summary>
        /// The local anchor point on BodyB
        /// </summary>
        public TSVector2 LocalAnchorB { get; set; }

        public override TSVector2 WorldAnchorA
        {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
            set { LocalAnchorA = BodyA.GetLocalPoint(value); }
        }

        public override TSVector2 WorldAnchorB
        {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { LocalAnchorB = BodyB.GetLocalPoint(value); }
        }

        /// <summary>
        /// The bodyB angle minus bodyA angle in the reference state (radians).
        /// </summary>
        public FP ReferenceAngle { get; set; }

        /// <summary>
        /// The frequency of the joint. A higher frequency means a stiffer joint, but
        /// a too high value can cause the joint to oscillate.
        /// Default is 0, which means the joint does no spring calculations.
        /// </summary>
        public FP FrequencyHz { get; set; }

        /// <summary>
        /// The damping on the joint. The damping is only used when
        /// the joint has a frequency (> 0). A higher value means more damping.
        /// </summary>
        public FP DampingRatio { get; set; }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            return invDt * new TSVector2(_impulse.x, _impulse.y);
        }

        public override FP GetReactionTorque(FP invDt)
        {
            return invDt * _impulse.z;
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

            FP aA = data.positions[_indexA].a;
            TSVector2 vA = data.velocities[_indexA].v;
            FP wA = data.velocities[_indexA].w;

            FP aB = data.positions[_indexB].a;
            TSVector2 vB = data.velocities[_indexB].v;
            FP wB = data.velocities[_indexB].w;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            _rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            _rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);

            // J = [-I -r1_skew I r2_skew]
            //     [ 0       -1 0       1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
            //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
            //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

            FP mA = _invMassA, mB = _invMassB;
            FP iA = _invIA, iB = _invIB;

            Mat33 K = new Mat33();
            K.ex.x = mA + mB + _rA.y * _rA.y * iA + _rB.y * _rB.y * iB;
            K.ey.x = -_rA.y * _rA.x * iA - _rB.y * _rB.x * iB;
            K.ez.x = -_rA.y * iA - _rB.y * iB;
            K.ex.y = K.ey.x;
            K.ey.y = mA + mB + _rA.x * _rA.x * iA + _rB.x * _rB.x * iB;
            K.ez.y = _rA.x * iA + _rB.x * iB;
            K.ex.z = K.ez.x;
            K.ey.z = K.ez.y;
            K.ez.z = iA + iB;

            if (FrequencyHz > 0.0f)
            {
                K.GetInverse22(ref _mass);

                FP invM = iA + iB;
                FP m = invM > 0.0f ? 1.0f / invM : 0.0f;

                FP C = aB - aA - ReferenceAngle;

                // Frequency
                FP omega = 2.0f * Settings.Pi * FrequencyHz;

                // Damping coefficient
                FP d = 2.0f * m * DampingRatio * omega;

                // Spring stiffness
                FP k = m * omega * omega;

                // magic formulas
                FP h = data.step.dt;
                _gamma = h * (d + h * k);
                _gamma = _gamma != 0.0f ? 1.0f / _gamma : 0.0f;
                _bias = C * h * k * _gamma;

                invM += _gamma;
                _mass.ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
            }
            else
            {
                K.GetSymInverse33(ref _mass);
                _gamma = 0.0f;
                _bias = 0.0f;
            }

            if (Settings.EnableWarmstarting)
            {
                // Scale impulses to support a variable time step.
                _impulse *= data.step.dtRatio;

                TSVector2 P = new TSVector2(_impulse.x, _impulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(_rA, P) + _impulse.z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(_rB, P) + _impulse.z);
            }
            else
            {
                _impulse = TSVector.zero;
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

            FP mA = _invMassA, mB = _invMassB;
            FP iA = _invIA, iB = _invIB;

            if (FrequencyHz > 0.0f)
            {
                FP Cdot2 = wB - wA;

                FP impulse2 = -_mass.ez.z * (Cdot2 + _bias + _gamma * _impulse.z);
                _impulse.z += impulse2;

                wA -= iA * impulse2;
                wB += iB * impulse2;

                TSVector2 Cdot1 = vB + MathUtils.Cross(wB, _rB) - vA - MathUtils.Cross(wA, _rA);

                TSVector2 impulse1 = -MathUtils.Mul22(_mass, Cdot1);
                _impulse.x += impulse1.x;
                _impulse.y += impulse1.y;

                TSVector2 P = impulse1;

                vA -= mA * P;
                wA -= iA * MathUtils.Cross(_rA, P);

                vB += mB * P;
                wB += iB * MathUtils.Cross(_rB, P);
            }
            else
            {
                TSVector2 Cdot1 = vB + MathUtils.Cross(wB, _rB) - vA - MathUtils.Cross(wA, _rA);
                FP Cdot2 = wB - wA;
                TSVector Cdot = new TSVector(Cdot1.x, Cdot1.y, Cdot2);

                TSVector impulse = MathUtils.Mul(_mass, Cdot) * -1;
                _impulse += impulse;

                TSVector2 P = new TSVector2(impulse.x, impulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(_rA, P) + impulse.z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(_rB, P) + impulse.z);
            }

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

            FP mA = _invMassA, mB = _invMassB;
            FP iA = _invIA, iB = _invIB;

            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);

            FP positionError, angularError;

            Mat33 K = new Mat33();
            K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
            K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
            K.ez.x = -rA.y * iA - rB.y * iB;
            K.ex.y = K.ey.x;
            K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
            K.ez.y = rA.x * iA + rB.x * iB;
            K.ex.z = K.ez.x;
            K.ey.z = K.ez.y;
            K.ez.z = iA + iB;

            if (FrequencyHz > 0.0f)
            {
                TSVector2 C1 = cB + rB - cA - rA;

                positionError = C1.magnitude;
                angularError = 0.0f;

                TSVector2 P = -K.Solve22(C1);

                cA -= mA * P;
                aA -= iA * MathUtils.Cross(rA, P);

                cB += mB * P;
                aB += iB * MathUtils.Cross(rB, P);
            }
            else
            {
                TSVector2 C1 = cB + rB - cA - rA;
                FP C2 = aB - aA - ReferenceAngle;

                positionError = C1.magnitude;
                angularError = FP.Abs(C2);

                TSVector C = new TSVector(C1.x, C1.y, C2);

                TSVector impulse = K.Solve33(C) * -1;
                TSVector2 P = new TSVector2(impulse.x, impulse.y);

                cA -= mA * P;
                aA -= iA * (MathUtils.Cross(rA, P) + impulse.z);

                cB += mB * P;
                aB += iB * (MathUtils.Cross(rB, P) + impulse.z);
            }

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}