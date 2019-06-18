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
    /// <summary>
    /// A revolute joint constrains to bodies to share a common point while they
    /// are free to rotate about the point. The relative rotation about the shared
    /// point is the joint angle. You can limit the relative rotation with
    /// a joint limit that specifies a lower and upper angle. You can use a motor
    /// to drive the relative rotation about the shared point. A maximum motor torque
    /// is provided so that infinite forces are not generated.
    /// </summary>
    public class RevoluteJoint : Joint2D
    {
        // Solver shared
        private TSVector _impulse;
        private FP _motorImpulse;

        private bool _enableMotor;
        private FP _maxMotorTorque;
        private FP _motorSpeed;

        private bool _enableLimit;
        private FP _referenceAngle;
        private FP _lowerAngle;
        private FP _upperAngle;

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
        private Mat33 _mass;			// effective mass for point-to-point constraint.
        private FP _motorMass;	    // effective mass for motor/limit angular constraint.
        private LimitState _limitState;

        internal RevoluteJoint()
        {
            JointType = JointType.Revolute;
        }

        /// <summary>
        /// Constructor of RevoluteJoint. 
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="anchorA">The first body anchor.</param>
        /// <param name="anchorB">The second anchor.</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public RevoluteJoint(Body bodyA, Body bodyB, TSVector2 anchorA, TSVector2 anchorB, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Revolute;

            if (useWorldCoordinates)
            {
                LocalAnchorA = BodyA.GetLocalPoint(anchorA);
                LocalAnchorB = BodyB.GetLocalPoint(anchorB);
            }
            else
            {
                LocalAnchorA = anchorA;
                LocalAnchorB = anchorB;
            }

            ReferenceAngle = BodyB.Rotation - BodyA.Rotation;

            _impulse = TSVector.zero;
            _limitState = LimitState.Inactive;
        }

        /// <summary>
        /// Constructor of RevoluteJoint. 
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="anchor">The shared anchor.</param>
        /// <param name="useWorldCoordinates"></param>
        public RevoluteJoint(Body bodyA, Body bodyB, TSVector2 anchor, bool useWorldCoordinates = false)
            : this(bodyA, bodyB, anchor, anchor, useWorldCoordinates)
        {
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
        /// The referance angle computed as BodyB angle minus BodyA angle.
        /// </summary>
        public FP ReferenceAngle
        {
            get { return _referenceAngle; }
            set
            {
                WakeBodies();
                _referenceAngle = value;
            }
        }

        /// <summary>
        /// Get the current joint angle in radians.
        /// </summary>
        public FP JointAngle
        {
            get { return BodyB._sweep.A - BodyA._sweep.A - ReferenceAngle; }
        }

        /// <summary>
        /// Get the current joint angle speed in radians per second.
        /// </summary>
        public FP JointSpeed
        {
            get { return BodyB._angularVelocity - BodyA._angularVelocity; }
        }

        /// <summary>
        /// Is the joint limit enabled?
        /// </summary>
        /// <value><c>true</c> if [limit enabled]; otherwise, <c>false</c>.</value>
        public bool LimitEnabled
        {
            get { return _enableLimit; }
            set
            {
                if (_enableLimit != value)
                {
                    WakeBodies();
                    _enableLimit = value;
                    _impulse.z = 0.0f;
                }
            }
        }

        /// <summary>
        /// Get the lower joint limit in radians.
        /// </summary>
        public FP LowerLimit
        {
            get { return _lowerAngle; }
            set
            {
                if (_lowerAngle != value)
                {
                    WakeBodies();
                    _lowerAngle = value;
                    _impulse.z = 0.0f;
                }
            }
        }

        /// <summary>
        /// Get the upper joint limit in radians.
        /// </summary>
        public FP UpperLimit
        {
            get { return _upperAngle; }
            set
            {
                if (_upperAngle != value)
                {
                    WakeBodies();
                    _upperAngle = value;
                    _impulse.z = 0.0f;
                }
            }
        }

        /// <summary>
        /// Set the joint limits, usually in meters.
        /// </summary>
        /// <param name="lower">The lower limit</param>
        /// <param name="upper">The upper limit</param>
        public void SetLimits(FP lower, FP upper)
        {
            if (lower != _lowerAngle || upper != _upperAngle)
            {
                WakeBodies();
                _upperAngle = upper;
                _lowerAngle = lower;
                _impulse.z = 0.0f;
            }
        }

        /// <summary>
        /// Is the joint motor enabled?
        /// </summary>
        /// <value><c>true</c> if [motor enabled]; otherwise, <c>false</c>.</value>
        public bool MotorEnabled
        {
            get { return _enableMotor; }
            set
            {
                WakeBodies();
                _enableMotor = value;
            }
        }

        /// <summary>
        /// Get or set the motor speed in radians per second.
        /// </summary>
        public FP MotorSpeed
        {
            set
            {
                WakeBodies();
                _motorSpeed = value;
            }
            get { return _motorSpeed; }
        }

        /// <summary>
        /// Get or set the maximum motor torque, usually in N-m.
        /// </summary>
        public FP MaxMotorTorque
        {
            set
            {
                WakeBodies();
                _maxMotorTorque = value;
            }
            get { return _maxMotorTorque; }
        }

        /// <summary>
        /// Get or set the current motor impulse, usually in N-m.
        /// </summary>
        public FP MotorImpulse
        {
            get { return _motorImpulse; }
            set
            {
                WakeBodies();
                _motorImpulse = value;
            }
        }

        /// <summary>
        /// Gets the motor torque in N-m.
        /// </summary>
        /// <param name="invDt">The inverse delta time</param>
        public FP GetMotorTorque(FP invDt)
        {
            return invDt * _motorImpulse;
        }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            TSVector2 p = new TSVector2(_impulse.x, _impulse.y);
            return invDt * p;
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

            bool fixedRotation = (iA + iB == 0.0f);

            _mass.ex.x = mA + mB + _rA.y * _rA.y * iA + _rB.y * _rB.y * iB;
            _mass.ey.x = -_rA.y * _rA.x * iA - _rB.y * _rB.x * iB;
            _mass.ez.x = -_rA.y * iA - _rB.y * iB;
            _mass.ex.y = _mass.ey.x;
            _mass.ey.y = mA + mB + _rA.x * _rA.x * iA + _rB.x * _rB.x * iB;
            _mass.ez.y = _rA.x * iA + _rB.x * iB;
            _mass.ex.z = _mass.ez.x;
            _mass.ey.z = _mass.ez.y;
            _mass.ez.z = iA + iB;

            _motorMass = iA + iB;
            if (_motorMass > 0.0f)
            {
                _motorMass = 1.0f / _motorMass;
            }

            if (_enableMotor == false || fixedRotation)
            {
                _motorImpulse = 0.0f;
            }

            if (_enableLimit && fixedRotation == false)
            {
                FP jointAngle = aB - aA - ReferenceAngle;
                if (FP.Abs(_upperAngle - _lowerAngle) < 2.0f * Settings.AngularSlop)
                {
                    _limitState = LimitState.Equal;
                }
                else if (jointAngle <= _lowerAngle)
                {
                    if (_limitState != LimitState.AtLower)
                    {
                        _impulse.z = 0.0f;
                    }
                    _limitState = LimitState.AtLower;
                }
                else if (jointAngle >= _upperAngle)
                {
                    if (_limitState != LimitState.AtUpper)
                    {
                        _impulse.z = 0.0f;
                    }
                    _limitState = LimitState.AtUpper;
                }
                else
                {
                    _limitState = LimitState.Inactive;
                    _impulse.z = 0.0f;
                }
            }
            else
            {
                _limitState = LimitState.Inactive;
            }

            if (Settings.EnableWarmstarting)
            {
                // Scale impulses to support a variable time step.
                _impulse *= data.step.dtRatio;
                _motorImpulse *= data.step.dtRatio;

                TSVector2 P = new TSVector2(_impulse.x, _impulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(_rA, P) + MotorImpulse + _impulse.z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(_rB, P) + MotorImpulse + _impulse.z);
            }
            else
            {
                _impulse = TSVector.zero;
                _motorImpulse = 0.0f;
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

            bool fixedRotation = (iA + iB == 0.0f);

            // Solve motor constraint.
            if (_enableMotor && _limitState != LimitState.Equal && fixedRotation == false)
            {
                FP Cdot = wB - wA - _motorSpeed;
                FP impulse = _motorMass * (-Cdot);
                FP oldImpulse = _motorImpulse;
                FP maxImpulse = data.step.dt * _maxMotorTorque;
                _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _motorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve limit constraint.
            if (_enableLimit && _limitState != LimitState.Inactive && fixedRotation == false)
            {
                TSVector2 Cdot1 = vB + MathUtils.Cross(wB, _rB) - vA - MathUtils.Cross(wA, _rA);
                FP Cdot2 = wB - wA;
                TSVector Cdot = new TSVector(Cdot1.x, Cdot1.y, Cdot2);

                TSVector impulse = _mass.Solve33(Cdot) * -1;

                if (_limitState == LimitState.Equal)
                {
                    _impulse += impulse;
                }
                else if (_limitState == LimitState.AtLower)
                {
                    FP newImpulse = _impulse.z + impulse.z;
                    if (newImpulse < 0.0f)
                    {
                        TSVector2 rhs = -Cdot1 + _impulse.z * new TSVector2(_mass.ez.x, _mass.ez.y);
                        TSVector2 reduced = _mass.Solve22(rhs);
                        impulse.x = reduced.x;
                        impulse.y = reduced.y;
                        impulse.z = -_impulse.z;
                        _impulse.x += reduced.x;
                        _impulse.y += reduced.y;
                        _impulse.z = 0.0f;
                    }
                    else
                    {
                        _impulse += impulse;
                    }
                }
                else if (_limitState == LimitState.AtUpper)
                {
                    FP newImpulse = _impulse.z + impulse.z;
                    if (newImpulse > 0.0f)
                    {
                        TSVector2 rhs = -Cdot1 + _impulse.z * new TSVector2(_mass.ez.x, _mass.ez.y);
                        TSVector2 reduced = _mass.Solve22(rhs);
                        impulse.x = reduced.x;
                        impulse.y = reduced.y;
                        impulse.z = -_impulse.z;
                        _impulse.x += reduced.x;
                        _impulse.y += reduced.y;
                        _impulse.z = 0.0f;
                    }
                    else
                    {
                        _impulse += impulse;
                    }
                }

                TSVector2 P = new TSVector2(impulse.x, impulse.y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(_rA, P) + impulse.z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(_rB, P) + impulse.z);
            }
            else
            {
                // Solve point-to-point constraint
                TSVector2 Cdot = vB + MathUtils.Cross(wB, _rB) - vA - MathUtils.Cross(wA, _rA);
                TSVector2 impulse = _mass.Solve22(-Cdot);

                _impulse.x += impulse.x;
                _impulse.y += impulse.y;

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
            TSVector2 cA = data.positions[_indexA].c;
            FP aA = data.positions[_indexA].a;
            TSVector2 cB = data.positions[_indexB].c;
            FP aB = data.positions[_indexB].a;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            FP angularError = 0.0f;
            FP positionError;

            bool fixedRotation = (_invIA + _invIB == 0.0f);

            // Solve angular limit constraint.
            if (_enableLimit && _limitState != LimitState.Inactive && fixedRotation == false)
            {
                FP angle = aB - aA - ReferenceAngle;
                FP limitImpulse = 0.0f;

                if (_limitState == LimitState.Equal)
                {
                    // Prevent large angular corrections
                    FP C = MathUtils.Clamp(angle - _lowerAngle, -Settings.MaxAngularCorrection, Settings.MaxAngularCorrection);
                    limitImpulse = -_motorMass * C;
                    angularError = FP.Abs(C);
                }
                else if (_limitState == LimitState.AtLower)
                {
                    FP C = angle - _lowerAngle;
                    angularError = -C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(C + Settings.AngularSlop, -Settings.MaxAngularCorrection, 0.0f);
                    limitImpulse = -_motorMass * C;
                }
                else if (_limitState == LimitState.AtUpper)
                {
                    FP C = angle - _upperAngle;
                    angularError = C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(C - Settings.AngularSlop, 0.0f, Settings.MaxAngularCorrection);
                    limitImpulse = -_motorMass * C;
                }

                aA -= _invIA * limitImpulse;
                aB += _invIB * limitImpulse;
            }

            // Solve point-to-point constraint.
            {
                qA.Set(aA);
                qB.Set(aB);
                TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
                TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);

                TSVector2 C = cB + rB - cA - rA;
                positionError = C.magnitude;

                FP mA = _invMassA, mB = _invMassB;
                FP iA = _invIA, iB = _invIB;

                Mat22 K = new Mat22();
                K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
                K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
                K.ey.x = K.ex.y;
                K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

                TSVector2 impulse = -K.Solve(C);

                cA -= mA * impulse;
                aA -= iA * MathUtils.Cross(rA, impulse);

                cB += mB * impulse;
                aB += iB * MathUtils.Cross(rB, impulse);
            }

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}