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
    /// <summary>
    /// A motor joint is used to control the relative motion
    /// between two bodies. A typical usage is to control the movement
    /// of a dynamic body with respect to the ground.
    /// </summary>
    public class MotorJoint : Joint2D
    {
        // Solver shared
        private TSVector2 _linearOffset;
        private FP _angularOffset;
        private TSVector2 _linearImpulse;
        private FP _angularImpulse;
        private FP _maxForce;
        private FP _maxTorque;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private TSVector2 _rA;
        private TSVector2 _rB;
        private TSVector2 _localCenterA;
        private TSVector2 _localCenterB;
        private TSVector2 _linearError;
        private FP _angularError;
        private FP _invMassA;
        private FP _invMassB;
        private FP _invIA;
        private FP _invIB;
        private Mat22 _linearMass;
        private FP _angularMass;

        internal MotorJoint()
        {
            JointType = JointType.Motor;
        }

        /// <summary>
        /// Constructor for MotorJoint.
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public MotorJoint(Body bodyA, Body bodyB, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Motor;

            TSVector2 xB = BodyB.Position;

            if (useWorldCoordinates)
                _linearOffset = BodyA.GetLocalPoint(xB);
            else
                _linearOffset = xB;

            //Defaults
            _angularOffset = 0.0f;
            _maxForce = 1.0f;
            _maxTorque = 1.0f;
            CorrectionFactor = 0.3f;

            _angularOffset = BodyB.Rotation - BodyA.Rotation;
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
        /// The maximum amount of force that can be applied to BodyA
        /// </summary>
        public FP MaxForce
        {
            set
            {
                Debug.Assert(MathUtils.IsValid(value) && value >= 0.0f);
                _maxForce = value;
            }
            get { return _maxForce; }
        }

        /// <summary>
        /// The maximum amount of torque that can be applied to BodyA
        /// </summary>
        public FP MaxTorque
        {
            set
            {
                Debug.Assert(MathUtils.IsValid(value) && value >= 0.0f);
                _maxTorque = value;
            }
            get { return _maxTorque; }
        }

        /// <summary>
        /// The linear (translation) offset.
        /// </summary>
        public TSVector2 LinearOffset
        {
            set
            {
                if (_linearOffset.x != value.x || _linearOffset.y != value.y)
                {
                    WakeBodies();
                    _linearOffset = value;
                }
            }
            get { return _linearOffset; }
        }

        /// <summary>
        /// Get or set the angular offset.
        /// </summary>
        public FP AngularOffset
        {
            set
            {
                if (_angularOffset != value)
                {
                    WakeBodies();
                    _angularOffset = value;
                }
            }
            get { return _angularOffset; }
        }

        //FPE note: Used for serialization.
        internal FP CorrectionFactor { get; set; }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            return invDt * _linearImpulse;
        }

        public override FP GetReactionTorque(FP invDt)
        {
            return invDt * _angularImpulse;
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

            Rot qA = new Rot(aA);
            Rot qB = new Rot(aB);

            // Compute the effective mass matrix.
            _rA = MathUtils.Mul(qA, -_localCenterA);
            _rB = MathUtils.Mul(qB, -_localCenterB);

            // J = [-I -r1_skew I r2_skew]
            //     [ 0       -1 0       1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
            //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
            //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

            FP mA = _invMassA, mB = _invMassB;
            FP iA = _invIA, iB = _invIB;

            Mat22 K = new Mat22();
            K.ex.x = mA + mB + iA * _rA.y * _rA.y + iB * _rB.y * _rB.y;
            K.ex.y = -iA * _rA.x * _rA.y - iB * _rB.x * _rB.y;
            K.ey.x = K.ex.y;
            K.ey.y = mA + mB + iA * _rA.x * _rA.x + iB * _rB.x * _rB.x;

            _linearMass = K.Inverse;

            _angularMass = iA + iB;
            if (_angularMass > 0.0f)
            {
                _angularMass = 1.0f / _angularMass;
            }

            _linearError = cB + _rB - cA - _rA - MathUtils.Mul(qA, _linearOffset);
            _angularError = aB - aA - _angularOffset;

            if (Settings.EnableWarmstarting)
            {
                // Scale impulses to support a variable time step.
                _linearImpulse *= data.step.dtRatio;
                _angularImpulse *= data.step.dtRatio;

                TSVector2 P = new TSVector2(_linearImpulse.x, _linearImpulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(_rA, P) + _angularImpulse);
                vB += mB * P;
                wB += iB * (MathUtils.Cross(_rB, P) + _angularImpulse);
            }
            else
            {
                _linearImpulse = TSVector2.zero;
                _angularImpulse = 0.0f;
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

            FP h = data.step.dt;
            FP inv_h = data.step.inv_dt;

            // Solve angular friction
            {
                FP Cdot = wB - wA + inv_h * CorrectionFactor * _angularError;
                FP impulse = -_angularMass * Cdot;

                FP oldImpulse = _angularImpulse;
                FP maxImpulse = h * _maxTorque;
                _angularImpulse = MathUtils.Clamp(_angularImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _angularImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve linear friction
            {
                TSVector2 Cdot = vB + MathUtils.Cross(wB, _rB) - vA - MathUtils.Cross(wA, _rA) + inv_h * CorrectionFactor * _linearError;

                TSVector2 impulse = -MathUtils.Mul(ref _linearMass, ref Cdot);
                TSVector2 oldImpulse = _linearImpulse;
                _linearImpulse += impulse;

                FP maxImpulse = h * _maxForce;

                if (_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
                {
                    _linearImpulse.Normalize();
                    _linearImpulse *= maxImpulse;
                }

                impulse = _linearImpulse - oldImpulse;

                vA -= mA * impulse;
                wA -= iA * MathUtils.Cross(_rA, impulse);

                vB += mB * impulse;
                wB += iB * MathUtils.Cross(_rB, impulse);
            }

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            return true;
        }
    }
}