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
    // Linear constraint (point-to-line)
    // d = pB - pA = xB + rB - xA - rA
    // C = dot(ay, d)
    // Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
    //      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
    // J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

    // Spring linear constraint
    // C = dot(ax, d)
    // Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
    // J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

    // Motor rotational constraint
    // Cdot = wB - wA
    // J = [0 0 -1 0 0 1]

    /// <summary>
    /// A wheel joint. This joint provides two degrees of freedom: translation
    /// along an axis fixed in bodyA and rotation in the plane. You can use a
    /// joint limit to restrict the range of motion and a joint motor to drive
    /// the rotation or to model rotational friction.
    /// This joint is designed for vehicle suspensions.
    /// </summary>
    public class WheelJoint : Joint2D
    {
        // Solver shared
        private TSVector2 _localYAxis;

        private FP _impulse;
        private FP _motorImpulse;
        private FP _springImpulse;

        private FP _maxMotorTorque;
        private FP _motorSpeed;
        private bool _enableMotor;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private TSVector2 _localCenterA;
        private TSVector2 _localCenterB;
        private FP _invMassA;
        private FP _invMassB;
        private FP _invIA;
        private FP _invIB;

        private TSVector2 _ax, _ay;
        private FP _sAx, _sBx;
        private FP _sAy, _sBy;

        private FP _mass;
        private FP _motorMass;
        private FP _springMass;

        private FP _bias;
        private FP _gamma;
        private TSVector2 _axis;

        internal WheelJoint()
        {
            JointType = JointType.Wheel;
        }

        /// <summary>
        /// Constructor for WheelJoint
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="anchor">The anchor point</param>
        /// <param name="axis">The axis</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public WheelJoint(Body bodyA, Body bodyB, TSVector2 anchor, TSVector2 axis, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Wheel;

            if (useWorldCoordinates)
            {
                LocalAnchorA = bodyA.GetLocalPoint(anchor);
                LocalAnchorB = bodyB.GetLocalPoint(anchor);
            }
            else
            {
                LocalAnchorA = bodyA.GetLocalPoint(bodyB.GetWorldPoint(anchor));
                LocalAnchorB = anchor;
            }

            Axis = axis; //FPE only: We maintain the original value as it is supposed to.
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
        /// The axis at which the suspension moves.
        /// </summary>
        public TSVector2 Axis
        {
            get { return _axis; }
            set
            {
                _axis = value;
                LocalXAxis = BodyA.GetLocalVector(_axis);
                _localYAxis = MathUtils.Cross(1.0f, LocalXAxis);
            }
        }

        /// <summary>
        /// The axis in local coordinates relative to BodyA
        /// </summary>
        public TSVector2 LocalXAxis { get; private set; }

        /// <summary>
        /// The desired motor speed in radians per second.
        /// </summary>
        public FP MotorSpeed
        {
            get { return _motorSpeed; }
            set
            {
                WakeBodies();
                _motorSpeed = value;
            }
        }

        /// <summary>
        /// The maximum motor torque, usually in N-m.
        /// </summary>
        public FP MaxMotorTorque
        {
            get { return _maxMotorTorque; }
            set
            {
                WakeBodies();
                _maxMotorTorque = value;
            }
        }

        /// <summary>
        /// Suspension frequency, zero indicates no suspension
        /// </summary>
        public FP Frequency { get; set; }

        /// <summary>
        /// Suspension damping ratio, one indicates critical damping
        /// </summary>
        public FP DampingRatio { get; set; }

        /// <summary>
        /// Gets the translation along the axis
        /// </summary>
        public FP JointTranslation
        {
            get
            {
                Body bA = BodyA;
                Body bB = BodyB;

                TSVector2 pA = bA.GetWorldPoint(LocalAnchorA);
                TSVector2 pB = bB.GetWorldPoint(LocalAnchorB);
                TSVector2 d = pB - pA;
                TSVector2 axis = bA.GetWorldVector(LocalXAxis);

                FP translation = TSVector2.Dot(d, axis);
                return translation;
            }
        }

        /// <summary>
        /// Gets the angular velocity of the joint
        /// </summary>
        public FP JointSpeed
        {
            get
            {
                FP wA = BodyA.AngularVelocity;
                FP wB = BodyB.AngularVelocity;
                return wB - wA;
            }
        }

        /// <summary>
        /// Enable/disable the joint motor.
        /// </summary>
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
        /// Gets the torque of the motor
        /// </summary>
        /// <param name="invDt">inverse delta time</param>
        public FP GetMotorTorque(FP invDt)
        {
            return invDt * _motorImpulse;
        }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            return invDt * (_impulse * _ay + _springImpulse * _ax);
        }

        public override FP GetReactionTorque(FP invDt)
        {
            return invDt * _motorImpulse;
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

            FP mA = _invMassA, mB = _invMassB;
            FP iA = _invIA, iB = _invIB;

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
            TSVector2 d1 = cB + rB - cA - rA;

            // Point to line constraint
            {
                _ay = MathUtils.Mul(qA, _localYAxis);
                _sAy = MathUtils.Cross(d1 + rA, _ay);
                _sBy = MathUtils.Cross(rB, _ay);

                _mass = mA + mB + iA * _sAy * _sAy + iB * _sBy * _sBy;

                if (_mass > 0.0f)
                {
                    _mass = 1.0f / _mass;
                }
            }

            // Spring constraint
            _springMass = 0.0f;
            _bias = 0.0f;
            _gamma = 0.0f;
            if (Frequency > 0.0f)
            {
                _ax = MathUtils.Mul(qA, LocalXAxis);
                _sAx = MathUtils.Cross(d1 + rA, _ax);
                _sBx = MathUtils.Cross(rB, _ax);

                FP invMass = mA + mB + iA * _sAx * _sAx + iB * _sBx * _sBx;

                if (invMass > 0.0f)
                {
                    _springMass = 1.0f / invMass;

                    FP C = TSVector2.Dot(d1, _ax);

                    // Frequency
                    FP omega = 2.0f * Settings.Pi * Frequency;

                    // Damping coefficient
                    FP d = 2.0f * _springMass * DampingRatio * omega;

                    // Spring stiffness
                    FP k = _springMass * omega * omega;

                    // magic formulas
                    FP h = data.step.dt;
                    _gamma = h * (d + h * k);
                    if (_gamma > 0.0f)
                    {
                        _gamma = 1.0f / _gamma;
                    }

                    _bias = C * h * k * _gamma;

                    _springMass = invMass + _gamma;
                    if (_springMass > 0.0f)
                    {
                        _springMass = 1.0f / _springMass;
                    }
                }
            }
            else
            {
                _springImpulse = 0.0f;
            }

            // Rotational motor
            if (_enableMotor)
            {
                _motorMass = iA + iB;
                if (_motorMass > 0.0f)
                {
                    _motorMass = 1.0f / _motorMass;
                }
            }
            else
            {
                _motorMass = 0.0f;
                _motorImpulse = 0.0f;
            }

            if (Settings.EnableWarmstarting)
            {
                // Account for variable time step.
                _impulse *= data.step.dtRatio;
                _springImpulse *= data.step.dtRatio;
                _motorImpulse *= data.step.dtRatio;

                TSVector2 P = _impulse * _ay + _springImpulse * _ax;
                FP LA = _impulse * _sAy + _springImpulse * _sAx + _motorImpulse;
                FP LB = _impulse * _sBy + _springImpulse * _sBx + _motorImpulse;

                vA -= _invMassA * P;
                wA -= _invIA * LA;

                vB += _invMassB * P;
                wB += _invIB * LB;
            }
            else
            {
                _impulse = 0.0f;
                _springImpulse = 0.0f;
                _motorImpulse = 0.0f;
            }

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            FP mA = _invMassA, mB = _invMassB;
            FP iA = _invIA, iB = _invIB;

            TSVector2 vA = data.velocities[_indexA].v;
            FP wA = data.velocities[_indexA].w;
            TSVector2 vB = data.velocities[_indexB].v;
            FP wB = data.velocities[_indexB].w;

            // Solve spring constraint
            {
                FP Cdot = TSVector2.Dot(_ax, vB - vA) + _sBx * wB - _sAx * wA;
                FP impulse = -_springMass * (Cdot + _bias + _gamma * _springImpulse);
                _springImpulse += impulse;

                TSVector2 P = impulse * _ax;
                FP LA = impulse * _sAx;
                FP LB = impulse * _sBx;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            // Solve rotational motor constraint
            {
                FP Cdot = wB - wA - _motorSpeed;
                FP impulse = -_motorMass * Cdot;

                FP oldImpulse = _motorImpulse;
                FP maxImpulse = data.step.dt * _maxMotorTorque;
                _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _motorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve point to line constraint
            {
                FP Cdot = TSVector2.Dot(_ay, vB - vA) + _sBy * wB - _sAy * wA;
                FP impulse = -_mass * Cdot;
                _impulse += impulse;

                TSVector2 P = impulse * _ay;
                FP LA = impulse * _sAy;
                FP LB = impulse * _sBy;

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

            TSVector2 rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            TSVector2 rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            TSVector2 d = (cB - cA) + rB - rA;

            TSVector2 ay = MathUtils.Mul(qA, _localYAxis);

            FP sAy = MathUtils.Cross(d + rA, ay);
            FP sBy = MathUtils.Cross(rB, ay);

            FP C = TSVector2.Dot(d, ay);

            FP k = _invMassA + _invMassB + _invIA * _sAy * _sAy + _invIB * _sBy * _sBy;

            FP impulse;
            if (k != 0.0f)
            {
                impulse = -C / k;
            }
            else
            {
                impulse = 0.0f;
            }

            TSVector2 P = impulse * ay;
            FP LA = impulse * sAy;
            FP LB = impulse * sBy;

            cA -= _invMassA * P;
            aA -= _invIA * LA;
            cB += _invMassB * P;
            aB += _invIB * LB;

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return FP.Abs(C) <= Settings.LinearSlop;
        }
    }
}