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
    // Gear Joint:
    // C0 = (coordinate1 + ratio * coordinate2)_initial
    // C = (coordinate1 + ratio * coordinate2) - C0 = 0
    // J = [J1 ratio * J2]
    // K = J * invM * JT
    //   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
    //
    // Revolute:
    // coordinate = rotation
    // Cdot = angularVelocity
    // J = [0 0 1]
    // K = J * invM * JT = invI
    //
    // Prismatic:
    // coordinate = dot(p - pg, ug)
    // Cdot = dot(v + cross(w, r), ug)
    // J = [ug cross(r, ug)]
    // K = J * invM * JT = invMass + invI * cross(r, ug)^2

    /// <summary>
    /// A gear joint is used to connect two joints together.
    /// Either joint can be a revolute or prismatic joint.
    /// You specify a gear ratio to bind the motions together:
    /// <![CDATA[coordinate1 + ratio * coordinate2 = ant]]>
    /// The ratio can be negative or positive. If one joint is a revolute joint
    /// and the other joint is a prismatic joint, then the ratio will have units
    /// of length or units of 1/length.
    ///
    /// Warning: You have to manually destroy the gear joint if jointA or jointB is destroyed.
    /// </summary>
    public class GearJoint : Joint2D
    {
        private JointType _typeA;
        private JointType _typeB;

        private Body _bodyA;
        private Body _bodyB;
        private Body _bodyC;
        private Body _bodyD;

        // Solver shared
        private TSVector2 _localAnchorA;
        private TSVector2 _localAnchorB;
        private TSVector2 _localAnchorC;
        private TSVector2 _localAnchorD;

        private TSVector2 _localAxisC;
        private TSVector2 _localAxisD;

        private FP _referenceAngleA;
        private FP _referenceAngleB;

        private FP _constant;
        private FP _ratio;

        private FP _impulse;

        // Solver temp
        private int _indexA, _indexB, _indexC, _indexD;
        private TSVector2 _lcA, _lcB, _lcC, _lcD;
        private FP _mA, _mB, _mC, _mD;
        private FP _iA, _iB, _iC, _iD;
        private TSVector2 _JvAC, _JvBD;
        private FP _JwA, _JwB, _JwC, _JwD;
        private FP _mass;

        /// <summary>
        /// Requires two existing revolute or prismatic joints (any combination will work).
        /// The provided joints must attach a dynamic body to a static body.
        /// </summary>
        /// <param name="jointA">The first joint.</param>
        /// <param name="jointB">The second joint.</param>
        /// <param name="ratio">The ratio.</param>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        // TS - public GearJoint(Body bodyA, Body bodyB, Joint jointA, Joint jointB, FP ratio = 1f)
        public GearJoint(Body bodyA, Body bodyB, Joint2D jointA, Joint2D jointB, FP ratio)
        {
            JointType = JointType.Gear;
            BodyA = bodyA;
            BodyB = bodyB;
            JointA = jointA;
            JointB = jointB;
            Ratio = ratio;

            _typeA = jointA.JointType;
            _typeB = jointB.JointType;

            Debug.Assert(_typeA == JointType.Revolute || _typeA == JointType.Prismatic || _typeA == JointType.FixedRevolute || _typeA == JointType.FixedPrismatic);
            Debug.Assert(_typeB == JointType.Revolute || _typeB == JointType.Prismatic || _typeB == JointType.FixedRevolute || _typeB == JointType.FixedPrismatic);

            FP coordinateA, coordinateB;

            // TODO_ERIN there might be some problem with the joint edges in b2Joint.

            _bodyC = JointA.BodyA;
            _bodyA = JointA.BodyB;

            // Get geometry of joint1
            Transform xfA = _bodyA._xf;
            FP aA = _bodyA._sweep.A;
            Transform xfC = _bodyC._xf;
            FP aC = _bodyC._sweep.A;

            if (_typeA == JointType.Revolute)
            {
                RevoluteJoint revolute = (RevoluteJoint)jointA;
                _localAnchorC = revolute.LocalAnchorA;
                _localAnchorA = revolute.LocalAnchorB;
                _referenceAngleA = revolute.ReferenceAngle;
                _localAxisC = TSVector2.zero;

                coordinateA = aA - aC - _referenceAngleA;
            }
            else
            {
                PrismaticJoint prismatic = (PrismaticJoint)jointA;
                _localAnchorC = prismatic.LocalAnchorA;
                _localAnchorA = prismatic.LocalAnchorB;
                _referenceAngleA = prismatic.ReferenceAngle;
                _localAxisC = prismatic.LocalXAxis;

                TSVector2 pC = _localAnchorC;
                TSVector2 pA = MathUtils.MulT(xfC.q, MathUtils.Mul(xfA.q, _localAnchorA) + (xfA.p - xfC.p));
                coordinateA = TSVector2.Dot(pA - pC, _localAxisC);
            }

            _bodyD = JointB.BodyA;
            _bodyB = JointB.BodyB;

            // Get geometry of joint2
            Transform xfB = _bodyB._xf;
            FP aB = _bodyB._sweep.A;
            Transform xfD = _bodyD._xf;
            FP aD = _bodyD._sweep.A;

            if (_typeB == JointType.Revolute)
            {
                RevoluteJoint revolute = (RevoluteJoint)jointB;
                _localAnchorD = revolute.LocalAnchorA;
                _localAnchorB = revolute.LocalAnchorB;
                _referenceAngleB = revolute.ReferenceAngle;
                _localAxisD = TSVector2.zero;

                coordinateB = aB - aD - _referenceAngleB;
            }
            else
            {
                PrismaticJoint prismatic = (PrismaticJoint)jointB;
                _localAnchorD = prismatic.LocalAnchorA;
                _localAnchorB = prismatic.LocalAnchorB;
                _referenceAngleB = prismatic.ReferenceAngle;
                _localAxisD = prismatic.LocalXAxis;

                TSVector2 pD = _localAnchorD;
                TSVector2 pB = MathUtils.MulT(xfD.q, MathUtils.Mul(xfB.q, _localAnchorB) + (xfB.p - xfD.p));
                coordinateB = TSVector2.Dot(pB - pD, _localAxisD);
            }

            _ratio = ratio;
            _constant = coordinateA + _ratio * coordinateB;
            _impulse = 0.0f;
        }

        public override TSVector2 WorldAnchorA
        {
            get { return _bodyA.GetWorldPoint(_localAnchorA); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        public override TSVector2 WorldAnchorB
        {
            get { return _bodyB.GetWorldPoint(_localAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        /// <summary>
        /// The gear ratio.
        /// </summary>
        public FP Ratio
        {
            get { return _ratio; }
            set
            {
                Debug.Assert(MathUtils.IsValid(value));
                _ratio = value;
            }
        }

        /// <summary>
        /// The first revolute/prismatic joint attached to the gear joint.
        /// </summary>
        public Joint2D JointA { get; private set; }

        /// <summary>
        /// The second revolute/prismatic joint attached to the gear joint.
        /// </summary>
        public Joint2D JointB { get; private set; }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            TSVector2 P = _impulse * _JvAC;
            return invDt * P;
        }

        public override FP GetReactionTorque(FP invDt)
        {
            FP L = _impulse * _JwA;
            return invDt * L;
        }

        internal override void InitVelocityConstraints(ref SolverData data)
        {
            _indexA = _bodyA.IslandIndex;
            _indexB = _bodyB.IslandIndex;
            _indexC = _bodyC.IslandIndex;
            _indexD = _bodyD.IslandIndex;
            _lcA = _bodyA._sweep.LocalCenter;
            _lcB = _bodyB._sweep.LocalCenter;
            _lcC = _bodyC._sweep.LocalCenter;
            _lcD = _bodyD._sweep.LocalCenter;
            _mA = _bodyA._invMass;
            _mB = _bodyB._invMass;
            _mC = _bodyC._invMass;
            _mD = _bodyD._invMass;
            _iA = _bodyA._invI;
            _iB = _bodyB._invI;
            _iC = _bodyC._invI;
            _iD = _bodyD._invI;

            FP aA = data.positions[_indexA].a;
            TSVector2 vA = data.velocities[_indexA].v;
            FP wA = data.velocities[_indexA].w;

            FP aB = data.positions[_indexB].a;
            TSVector2 vB = data.velocities[_indexB].v;
            FP wB = data.velocities[_indexB].w;

            FP aC = data.positions[_indexC].a;
            TSVector2 vC = data.velocities[_indexC].v;
            FP wC = data.velocities[_indexC].w;

            FP aD = data.positions[_indexD].a;
            TSVector2 vD = data.velocities[_indexD].v;
            FP wD = data.velocities[_indexD].w;

            Rot qA = new Rot(aA), qB = new Rot(aB), qC = new Rot(aC), qD = new Rot(aD);

            _mass = 0.0f;

            if (_typeA == JointType.Revolute)
            {
                _JvAC = TSVector2.zero;
                _JwA = 1.0f;
                _JwC = 1.0f;
                _mass += _iA + _iC;
            }
            else
            {
                TSVector2 u = MathUtils.Mul(qC, _localAxisC);
                TSVector2 rC = MathUtils.Mul(qC, _localAnchorC - _lcC);
                TSVector2 rA = MathUtils.Mul(qA, _localAnchorA - _lcA);
                _JvAC = u;
                _JwC = MathUtils.Cross(rC, u);
                _JwA = MathUtils.Cross(rA, u);
                _mass += _mC + _mA + _iC * _JwC * _JwC + _iA * _JwA * _JwA;
            }

            if (_typeB == JointType.Revolute)
            {
                _JvBD = TSVector2.zero;
                _JwB = _ratio;
                _JwD = _ratio;
                _mass += _ratio * _ratio * (_iB + _iD);
            }
            else
            {
                TSVector2 u = MathUtils.Mul(qD, _localAxisD);
                TSVector2 rD = MathUtils.Mul(qD, _localAnchorD - _lcD);
                TSVector2 rB = MathUtils.Mul(qB, _localAnchorB - _lcB);
                _JvBD = _ratio * u;
                _JwD = _ratio * MathUtils.Cross(rD, u);
                _JwB = _ratio * MathUtils.Cross(rB, u);
                _mass += _ratio * _ratio * (_mD + _mB) + _iD * _JwD * _JwD + _iB * _JwB * _JwB;
            }

            // Compute effective mass.
            _mass = _mass > 0.0f ? 1.0f / _mass : 0.0f;

            if (Settings.EnableWarmstarting)
            {
                vA += (_mA * _impulse) * _JvAC;
                wA += _iA * _impulse * _JwA;
                vB += (_mB * _impulse) * _JvBD;
                wB += _iB * _impulse * _JwB;
                vC -= (_mC * _impulse) * _JvAC;
                wC -= _iC * _impulse * _JwC;
                vD -= (_mD * _impulse) * _JvBD;
                wD -= _iD * _impulse * _JwD;
            }
            else
            {
                _impulse = 0.0f;
            }

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
            data.velocities[_indexC].v = vC;
            data.velocities[_indexC].w = wC;
            data.velocities[_indexD].v = vD;
            data.velocities[_indexD].w = wD;
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            TSVector2 vA = data.velocities[_indexA].v;
            FP wA = data.velocities[_indexA].w;
            TSVector2 vB = data.velocities[_indexB].v;
            FP wB = data.velocities[_indexB].w;
            TSVector2 vC = data.velocities[_indexC].v;
            FP wC = data.velocities[_indexC].w;
            TSVector2 vD = data.velocities[_indexD].v;
            FP wD = data.velocities[_indexD].w;

            FP Cdot = TSVector2.Dot(_JvAC, vA - vC) + TSVector2.Dot(_JvBD, vB - vD);
            Cdot += (_JwA * wA - _JwC * wC) + (_JwB * wB - _JwD * wD);

            FP impulse = -_mass * Cdot;
            _impulse += impulse;

            vA += (_mA * impulse) * _JvAC;
            wA += _iA * impulse * _JwA;
            vB += (_mB * impulse) * _JvBD;
            wB += _iB * impulse * _JwB;
            vC -= (_mC * impulse) * _JvAC;
            wC -= _iC * impulse * _JwC;
            vD -= (_mD * impulse) * _JvBD;
            wD -= _iD * impulse * _JwD;

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
            data.velocities[_indexC].v = vC;
            data.velocities[_indexC].w = wC;
            data.velocities[_indexD].v = vD;
            data.velocities[_indexD].w = wD;
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            TSVector2 cA = data.positions[_indexA].c;
            FP aA = data.positions[_indexA].a;
            TSVector2 cB = data.positions[_indexB].c;
            FP aB = data.positions[_indexB].a;
            TSVector2 cC = data.positions[_indexC].c;
            FP aC = data.positions[_indexC].a;
            TSVector2 cD = data.positions[_indexD].c;
            FP aD = data.positions[_indexD].a;

            Rot qA = new Rot(aA), qB = new Rot(aB), qC = new Rot(aC), qD = new Rot(aD);

            FP linearError = 0.0f;

            FP coordinateA, coordinateB;

            TSVector2 JvAC, JvBD;
            FP JwA, JwB, JwC, JwD;
            FP mass = 0.0f;

            if (_typeA == JointType.Revolute)
            {
                JvAC = TSVector2.zero;
                JwA = 1.0f;
                JwC = 1.0f;
                mass += _iA + _iC;

                coordinateA = aA - aC - _referenceAngleA;
            }
            else
            {
                TSVector2 u = MathUtils.Mul(qC, _localAxisC);
                TSVector2 rC = MathUtils.Mul(qC, _localAnchorC - _lcC);
                TSVector2 rA = MathUtils.Mul(qA, _localAnchorA - _lcA);
                JvAC = u;
                JwC = MathUtils.Cross(rC, u);
                JwA = MathUtils.Cross(rA, u);
                mass += _mC + _mA + _iC * JwC * JwC + _iA * JwA * JwA;

                TSVector2 pC = _localAnchorC - _lcC;
                TSVector2 pA = MathUtils.MulT(qC, rA + (cA - cC));
                coordinateA = TSVector2.Dot(pA - pC, _localAxisC);
            }

            if (_typeB == JointType.Revolute)
            {
                JvBD = TSVector2.zero;
                JwB = _ratio;
                JwD = _ratio;
                mass += _ratio * _ratio * (_iB + _iD);

                coordinateB = aB - aD - _referenceAngleB;
            }
            else
            {
                TSVector2 u = MathUtils.Mul(qD, _localAxisD);
                TSVector2 rD = MathUtils.Mul(qD, _localAnchorD - _lcD);
                TSVector2 rB = MathUtils.Mul(qB, _localAnchorB - _lcB);
                JvBD = _ratio * u;
                JwD = _ratio * MathUtils.Cross(rD, u);
                JwB = _ratio * MathUtils.Cross(rB, u);
                mass += _ratio * _ratio * (_mD + _mB) + _iD * JwD * JwD + _iB * JwB * JwB;

                TSVector2 pD = _localAnchorD - _lcD;
                TSVector2 pB = MathUtils.MulT(qD, rB + (cB - cD));
                coordinateB = TSVector2.Dot(pB - pD, _localAxisD);
            }

            FP C = (coordinateA + _ratio * coordinateB) - _constant;

            FP impulse = 0.0f;
            if (mass > 0.0f)
            {
                impulse = -C / mass;
            }

            cA += _mA * impulse * JvAC;
            aA += _iA * impulse * JwA;
            cB += _mB * impulse * JvBD;
            aB += _iB * impulse * JwB;
            cC -= _mC * impulse * JvAC;
            aC -= _iC * impulse * JwC;
            cD -= _mD * impulse * JvBD;
            aD -= _iD * impulse * JwD;

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;
            data.positions[_indexC].c = cC;
            data.positions[_indexC].a = aC;
            data.positions[_indexD].c = cD;
            data.positions[_indexD].a = aD;

            // TODO_ERIN not implemented
            return linearError < Settings.LinearSlop;
        }
    }
}