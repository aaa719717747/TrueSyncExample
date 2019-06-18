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
    // Linear constraint (point-to-line)
    // d = p2 - p1 = x2 + r2 - x1 - r1
    // C = dot(perp, d)
    // Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    //      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
    // J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
    //
    // Angular constraint
    // C = a2 - a1 + a_initial
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    //
    // K = J * invM * JT
    //
    // J = [-a -s1 a s2]
    //     [0  -1  0  1]
    // a = perp
    // s1 = cross(d + r1, a) = cross(p2 - x1, a)
    // s2 = cross(r2, a) = cross(p2 - x2, a)
    // Motor/Limit linear constraint
    // C = dot(ax1, d)
    // Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
    // J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]
    // Block Solver
    // We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
    // when the mass has poor distribution (leading to large torques about the joint anchor points).
    //
    // The Jacobian has 3 rows:
    // J = [-uT -s1 uT s2] // linear
    //     [0   -1   0  1] // angular
    //     [-vT -a1 vT a2] // limit
    //
    // u = perp
    // v = axis
    // s1 = cross(d + r1, u), s2 = cross(r2, u)
    // a1 = cross(d + r1, v), a2 = cross(r2, v)
    // M * (v2 - v1) = JT * df
    // J * v2 = bias
    //
    // v2 = v1 + invM * JT * df
    // J * (v1 + invM * JT * df) = bias
    // K * df = bias - J * v1 = -Cdot
    // K = J * invM * JT
    // Cdot = J * v1 - bias
    //
    // Now solve for f2.
    // df = f2 - f1
    // K * (f2 - f1) = -Cdot
    // f2 = invK * (-Cdot) + f1
    //
    // Clamp accumulated limit impulse.
    // lower: f2(3) = max(f2(3), 0)
    // upper: f2(3) = min(f2(3), 0)
    //
    // Solve for correct f2(1:2)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
    //                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
    // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
    //
    // Now compute impulse to be applied:
    // df = f2 - f1

    /// <summary>
    /// A prismatic joint. This joint provides one degree of freedom: translation
    /// along an axis fixed in bodyA. Relative rotation is prevented. You can
    /// use a joint limit to restrict the range of motion and a joint motor to
    /// drive the motion or to model joint friction.
    /// </summary>
    public class PrismaticJoint : Joint2D
    {
        private TSVector2 _localYAxisA;
        private TSVector _impulse;
        private FP _lowerTranslation;
        private FP _upperTranslation;
        private FP _maxMotorForce;
        private FP _motorSpeed;
        private bool _enableLimit;
        private bool _enableMotor;
        private LimitState _limitState;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private TSVector2 _localCenterA;
        private TSVector2 _localCenterB;
        private FP _invMassA;
        private FP _invMassB;
        private FP _invIA;
        private FP _invIB;
        private TSVector2 _axis, _perp;
        private FP _s1, _s2;
        private FP _a1, _a2;
        private Mat33 _K;
        private FP _motorMass;
        private TSVector2 _axis1;

        internal PrismaticJoint()
        {
            JointType = JointType.Prismatic;
        }

        /// <summary>
        /// This requires defining a line of
        /// motion using an axis and an anchor point. The definition uses local
        /// anchor points and a local axis so that the initial configuration
        /// can violate the constraint slightly. The joint translation is zero
        /// when the local anchor points coincide in world space. Using local
        /// anchors and a local axis helps when saving and loading a game.
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="anchorA">The first body anchor.</param>
        /// <param name="anchorB">The second body anchor.</param>
        /// <param name="axis">The axis.</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public PrismaticJoint(Body bodyA, Body bodyB, TSVector2 anchorA, TSVector2 anchorB, TSVector2 axis, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            Initialize(anchorA, anchorB, axis, useWorldCoordinates);
        }

        public PrismaticJoint(Body bodyA, Body bodyB, TSVector2 anchor, TSVector2 axis, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            Initialize(anchor, anchor, axis, useWorldCoordinates);
        }

        private void Initialize(TSVector2 localAnchorA, TSVector2 localAnchorB, TSVector2 axis, bool useWorldCoordinates)
        {
            JointType = JointType.Prismatic;

            if (useWorldCoordinates)
            {
                LocalAnchorA = BodyA.GetLocalPoint(localAnchorA);
                LocalAnchorB = BodyB.GetLocalPoint(localAnchorB);
            }
            else
            {
                LocalAnchorA = localAnchorA;
                LocalAnchorB = localAnchorB;
            }

            Axis = axis; //FPE only: store the orignal value for use in Serialization
            ReferenceAngle = BodyB.Rotation - BodyA.Rotation;

            _limitState = LimitState.Inactive;
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
        /// Get the current joint translation, usually in meters.
        /// </summary>
        /// <value></value>
        public FP JointTranslation
        {
            get
            {
                TSVector2 d = BodyB.GetWorldPoint(LocalAnchorB) - BodyA.GetWorldPoint(LocalAnchorA);
                TSVector2 axis = BodyA.GetWorldVector(LocalXAxis);

                return TSVector2.Dot(d, axis);
            }
        }

        /// <summary>
        /// Get the current joint translation speed, usually in meters per second.
        /// </summary>
        /// <value></value>
        public FP JointSpeed
        {
            get
            {
                Transform xf1, xf2;
                BodyA.GetTransform(out xf1);
                BodyB.GetTransform(out xf2);

                TSVector2 r1 = MathUtils.Mul(ref xf1.q, LocalAnchorA - BodyA.LocalCenter);
                TSVector2 r2 = MathUtils.Mul(ref xf2.q, LocalAnchorB - BodyB.LocalCenter);
                TSVector2 p1 = BodyA._sweep.C + r1;
                TSVector2 p2 = BodyB._sweep.C + r2;
                TSVector2 d = p2 - p1;
                TSVector2 axis = BodyA.GetWorldVector(LocalXAxis);

                TSVector2 v1 = BodyA._linearVelocity;
                TSVector2 v2 = BodyB._linearVelocity;
                FP w1 = BodyA._angularVelocity;
                FP w2 = BodyB._angularVelocity;

                FP speed = TSVector2.Dot(d, MathUtils.Cross(w1, axis)) + TSVector2.Dot(axis, v2 + MathUtils.Cross(w2, r2) - v1 - MathUtils.Cross(w1, r1));
                return speed;
            }
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
                Debug.Assert(BodyA.FixedRotation == false || BodyB.FixedRotation == false, "Warning: limits does currently not work with fixed rotation");

                if (value != _enableLimit)
                {
                    WakeBodies();
                    _enableLimit = value;
                    _impulse.z = 0;
                }
            }
        }

        /// <summary>
        /// Get the lower joint limit, usually in meters.
        /// </summary>
        /// <value></value>
        public FP LowerLimit
        {
            get { return _lowerTranslation; }
            set
            {
                if (value != _lowerTranslation)
                {
                    WakeBodies();
                    _lowerTranslation = value;
                    _impulse.z = 0.0f;
                }
            }
        }

        /// <summary>
        /// Get the upper joint limit, usually in meters.
        /// </summary>
        /// <value></value>
        public FP UpperLimit
        {
            get { return _upperTranslation; }
            set
            {
                if (value != _upperTranslation)
                {
                    WakeBodies();
                    _upperTranslation = value;
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
            if (upper != _upperTranslation || lower != _lowerTranslation)
            {
                WakeBodies();
                _upperTranslation = upper;
                _lowerTranslation = lower;
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
        /// Set the motor speed, usually in meters per second.
        /// </summary>
        /// <value>The speed.</value>
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
        /// Set the maximum motor force, usually in N.
        /// </summary>
        /// <value>The force.</value>
        public FP MaxMotorForce
        {
            get { return _maxMotorForce; }
            set
            {
                WakeBodies();
                _maxMotorForce = value;
            }
        }

        /// <summary>
        /// Get the current motor impulse, usually in N.
        /// </summary>
        /// <value></value>
        public FP MotorImpulse { get; set; }

        /// <summary>
        /// Gets the motor force.
        /// </summary>
        /// <param name="invDt">The inverse delta time</param>
        public FP GetMotorForce(FP invDt)
        {
            return invDt * MotorImpulse;
        }

        /// <summary>
        /// The axis at which the joint moves.
        /// </summary>
        public TSVector2 Axis
        {
            get { return _axis1; }
            set
            {
                _axis1 = value;
                LocalXAxis = BodyA.GetLocalVector(_axis1);
                LocalXAxis.Normalize();
                _localYAxisA = MathUtils.Cross(1.0f, LocalXAxis);
            }
        }

        /// <summary>
        /// The axis in local coordinates relative to BodyA
        /// </summary>
        public TSVector2 LocalXAxis { get; private set; }

        /// <summary>
        /// The reference angle.
        /// </summary>
        public FP ReferenceAngle { get; set; }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            return invDt * (_impulse.x * _perp + (MotorImpulse + _impulse.z) * _axis);
        }

        public override FP GetReactionTorque(FP invDt)
        {
            return invDt * _impulse.y;
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

            // Compute the effective masses.
            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            TSVector2 d = (cB - cA) + rB - rA;

            FP mA = _invMassA, mB = _invMassB;
            FP iA = _invIA, iB = _invIB;

            // Compute motor Jacobian and effective mass.
            {
                _axis = MathUtils.Mul(qA, LocalXAxis);
                _a1 = MathUtils.Cross(d + rA, _axis);
                _a2 = MathUtils.Cross(rB, _axis);

                _motorMass = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
                if (_motorMass > 0.0f)
                {
                    _motorMass = 1.0f / _motorMass;
                }
            }

            // Prismatic constraint.
            {
                _perp = MathUtils.Mul(qA, _localYAxisA);

                _s1 = MathUtils.Cross(d + rA, _perp);
                _s2 = MathUtils.Cross(rB, _perp);

                FP k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
                FP k12 = iA * _s1 + iB * _s2;
                FP k13 = iA * _s1 * _a1 + iB * _s2 * _a2;
                FP k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    // For bodies with fixed rotation.
                    k22 = 1.0f;
                }
                FP k23 = iA * _a1 + iB * _a2;
                FP k33 = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;

                _K.ex = new TSVector(k11, k12, k13);
                _K.ey = new TSVector(k12, k22, k23);
                _K.ez = new TSVector(k13, k23, k33);
            }

            // Compute motor and limit terms.
            if (_enableLimit)
            {
                FP jointTranslation = TSVector2.Dot(_axis, d);
                if (FP.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.LinearSlop)
                {
                    _limitState = LimitState.Equal;
                }
                else if (jointTranslation <= _lowerTranslation)
                {
                    if (_limitState != LimitState.AtLower)
                    {
                        _limitState = LimitState.AtLower;
                        _impulse.z = 0.0f;
                    }
                }
                else if (jointTranslation >= _upperTranslation)
                {
                    if (_limitState != LimitState.AtUpper)
                    {
                        _limitState = LimitState.AtUpper;
                        _impulse.z = 0.0f;
                    }
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
                _impulse.z = 0.0f;
            }

            if (_enableMotor == false)
            {
                MotorImpulse = 0.0f;
            }

            if (Settings.EnableWarmstarting)
            {
                // Account for variable time step.
                _impulse *= data.step.dtRatio;
                MotorImpulse *= data.step.dtRatio;

                TSVector2 P = _impulse.x * _perp + (MotorImpulse + _impulse.z) * _axis;
                FP LA = _impulse.x * _s1 + _impulse.y + (MotorImpulse + _impulse.z) * _a1;
                FP LB = _impulse.x * _s2 + _impulse.y + (MotorImpulse + _impulse.z) * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else
            {
                _impulse = TSVector.zero;
                MotorImpulse = 0.0f;
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

            // Solve linear motor constraint.
            if (_enableMotor && _limitState != LimitState.Equal)
            {
                FP Cdot = TSVector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
                FP impulse = _motorMass * (_motorSpeed - Cdot);
                FP oldImpulse = MotorImpulse;
                FP maxImpulse = data.step.dt * _maxMotorForce;
                MotorImpulse = MathUtils.Clamp(MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = MotorImpulse - oldImpulse;

                TSVector2 P = impulse * _axis;
                FP LA = impulse * _a1;
                FP LB = impulse * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            TSVector2 Cdot1 = new TSVector2();
            Cdot1.x = TSVector2.Dot(_perp, vB - vA) + _s2 * wB - _s1 * wA;
            Cdot1.y = wB - wA;

            if (_enableLimit && _limitState != LimitState.Inactive)
            {
                // Solve prismatic and limit constraint in block form.
                FP Cdot2;
                Cdot2 = TSVector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
                TSVector Cdot = new TSVector(Cdot1.x, Cdot1.y, Cdot2);

                TSVector f1 = _impulse;
                TSVector df = _K.Solve33(Cdot * -1);
                _impulse += df;

                if (_limitState == LimitState.AtLower)
                {
                    _impulse.z = TrueSync.TSMath.Max(_impulse.z, 0.0f);
                }
                else if (_limitState == LimitState.AtUpper)
                {
                    _impulse.z = TrueSync.TSMath.Min(_impulse.z, 0.0f);
                }

                // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
                TSVector2 b = -Cdot1 - (_impulse.z - f1.z) * new TSVector2(_K.ez.x, _K.ez.y);
                TSVector2 f2r = _K.Solve22(b) + new TSVector2(f1.x, f1.y);
                _impulse.x = f2r.x;
                _impulse.y = f2r.y;

                df = _impulse - f1;

                TSVector2 P = df.x * _perp + df.z * _axis;
                FP LA = df.x * _s1 + df.y + df.z * _a1;
                FP LB = df.x * _s2 + df.y + df.z * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else
            {
                // Limit is inactive, just solve the prismatic constraint in block form.
                TSVector2 df = _K.Solve22(-Cdot1);
                _impulse.x += df.x;
                _impulse.y += df.y;

                TSVector2 P = df.x * _perp;
                FP LA = df.x * _s1 + df.y;
                FP LB = df.x * _s2 + df.y;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
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

            // Compute fresh Jacobians
            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            TSVector2 d = cB + rB - cA - rA;

            TSVector2 axis = MathUtils.Mul(qA, LocalXAxis);
            FP a1 = MathUtils.Cross(d + rA, axis);
            FP a2 = MathUtils.Cross(rB, axis);
            TSVector2 perp = MathUtils.Mul(qA, _localYAxisA);

            FP s1 = MathUtils.Cross(d + rA, perp);
            FP s2 = MathUtils.Cross(rB, perp);

            TSVector impulse;
            TSVector2 C1 = new TSVector2();
            C1.x = TSVector2.Dot(perp, d);
            C1.y = aB - aA - ReferenceAngle;

            FP linearError = FP.Abs(C1.x);
            FP angularError = FP.Abs(C1.y);

            bool active = false;
            FP C2 = 0.0f;
            if (_enableLimit)
            {
                FP translation = TSVector2.Dot(axis, d);
                if (FP.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.LinearSlop)
                {
                    // Prevent large angular corrections
                    C2 = MathUtils.Clamp(translation, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);
                    linearError = TrueSync.TSMath.Max(linearError, FP.Abs(translation));
                    active = true;
                }
                else if (translation <= _lowerTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - _lowerTranslation + Settings.LinearSlop, -Settings.MaxLinearCorrection, 0.0f);
                    linearError = TrueSync.TSMath.Max(linearError, _lowerTranslation - translation);
                    active = true;
                }
                else if (translation >= _upperTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - _upperTranslation - Settings.LinearSlop, 0.0f, Settings.MaxLinearCorrection);
                    linearError = TrueSync.TSMath.Max(linearError, translation - _upperTranslation);
                    active = true;
                }
            }

            if (active)
            {
                FP k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                FP k12 = iA * s1 + iB * s2;
                FP k13 = iA * s1 * a1 + iB * s2 * a2;
                FP k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    // For fixed rotation
                    k22 = 1.0f;
                }
                FP k23 = iA * a1 + iB * a2;
                FP k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

                Mat33 K = new Mat33();
                K.ex = new TSVector(k11, k12, k13);
                K.ey = new TSVector(k12, k22, k23);
                K.ez = new TSVector(k13, k23, k33);

                TSVector C = new TSVector();
                C.x = C1.x;
                C.y = C1.y;
                C.z = C2;

                impulse = K.Solve33(C * -1);
            }
            else
            {
                FP k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                FP k12 = iA * s1 + iB * s2;
                FP k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    k22 = 1.0f;
                }

                Mat22 K = new Mat22();
                K.ex = new TSVector2(k11, k12);
                K.ey = new TSVector2(k12, k22);

                TSVector2 impulse1 = K.Solve(-C1);
                impulse = new TSVector();
                impulse.x = impulse1.x;
                impulse.y = impulse1.y;
                impulse.z = 0.0f;
            }

            TSVector2 P = impulse.x * perp + impulse.z * axis;
            FP LA = impulse.x * s1 + impulse.y + impulse.z * a1;
            FP LB = impulse.x * s2 + impulse.y + impulse.z * a2;

            cA -= mA * P;
            aA -= iA * LA;
            cB += mB * P;
            aB += iB * LB;

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}