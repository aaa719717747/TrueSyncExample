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
    // p = attached point, m = mouse point
    // C = p - m
    // Cdot = v
    //      = v + cross(w, r)
    // J = [I r_skew]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    /// <summary>
    /// A mouse joint is used to make a point on a body track a
    /// specified world point. This a soft constraint with a maximum
    /// force. This allows the constraint to stretch and without
    /// applying huge forces.
    /// NOTE: this joint is not documented in the manual because it was
    /// developed to be used in the testbed. If you want to learn how to
    /// use the mouse joint, look at the testbed.
    /// </summary>
    public class FixedMouseJoint : Joint2D
    {
        private TSVector2 _worldAnchor;
        private FP _frequency;
        private FP _dampingRatio;
        private FP _beta;

        // Solver shared
        private TSVector2 _impulse;
        private FP _maxForce;
        private FP _gamma;

        // Solver temp
        private int _indexA;
        private TSVector2 _rA;
        private TSVector2 _localCenterA;
        private FP _invMassA;
        private FP _invIA;
        private Mat22 _mass;
        private TSVector2 _C;

        /// <summary>
        /// This requires a world target point,
        /// tuning parameters, and the time step.
        /// </summary>
        /// <param name="body">The body.</param>
        /// <param name="worldAnchor">The target.</param>
        public FixedMouseJoint(Body body, TSVector2 worldAnchor)
            : base(body)
        {
            JointType = JointType.FixedMouse;
            Frequency = 5.0f;
            DampingRatio = 0.7f;
            MaxForce = 1000 * body.Mass;

            Debug.Assert(worldAnchor.IsValid());

            _worldAnchor = worldAnchor;
            LocalAnchorA = MathUtils.MulT(BodyA._xf, worldAnchor);
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public TSVector2 LocalAnchorA { get; set; }

        public override TSVector2 WorldAnchorA
        {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
            set { LocalAnchorA = BodyA.GetLocalPoint(value); }
        }

        public override TSVector2 WorldAnchorB
        {
            get { return _worldAnchor; }
            set
            {
                WakeBodies();
                _worldAnchor = value;
            }
        }

        /// <summary>
        /// The maximum constraint force that can be exerted
        /// to move the candidate body. Usually you will express
        /// as some multiple of the weight (multiplier * mass * gravity).
        /// </summary>
        public FP MaxForce
        {
            get { return _maxForce; }
            set
            {
                Debug.Assert(MathUtils.IsValid(value) && value >= 0.0f);
                _maxForce = value;
            }
        }

        /// <summary>
        /// The response speed.
        /// </summary>
        public FP Frequency
        {
            get { return _frequency; }
            set
            {
                Debug.Assert(MathUtils.IsValid(value) && value >= 0.0f);
                _frequency = value;
            }
        }

        /// <summary>
        /// The damping ratio. 0 = no damping, 1 = critical damping.
        /// </summary>
        public FP DampingRatio
        {
            get { return _dampingRatio; }
            set
            {
                Debug.Assert(MathUtils.IsValid(value) && value >= 0.0f);
                _dampingRatio = value;
            }
        }

        public override TSVector2 GetReactionForce(FP invDt)
        {
            return invDt * _impulse;
        }

        public override FP GetReactionTorque(FP invDt)
        {
            return invDt * 0.0f;
        }

        internal override void InitVelocityConstraints(ref SolverData data)
        {
            _indexA = BodyA.IslandIndex;
            _localCenterA = BodyA._sweep.LocalCenter;
            _invMassA = BodyA._invMass;
            _invIA = BodyA._invI;

            TSVector2 cA = data.positions[_indexA].c;
            FP aA = data.positions[_indexA].a;
            TSVector2 vA = data.velocities[_indexA].v;
            FP wA = data.velocities[_indexA].w;

            Rot qA = new Rot(aA);

            FP mass = BodyA.Mass;

            // Frequency
            FP omega = 2.0f * Settings.Pi * Frequency;

            // Damping coefficient
            FP d = 2.0f * mass * DampingRatio * omega;

            // Spring stiffness
            FP k = mass * (omega * omega);

            // magic formulas
            // gamma has units of inverse mass.
            // beta has units of inverse time.
            FP h = data.step.dt;
            //Debug.Assert(d + h * k > Settings.Epsilon);
            _gamma = h * (d + h * k);
            if (_gamma != 0.0f)
            {
                _gamma = 1.0f / _gamma;
            }

            _beta = h * k * _gamma;

            // Compute the effective mass matrix.
            _rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
            //      = [1/m1+1/m2     0    ] + invI1 * [r1.Y*r1.Y -r1.X*r1.Y] + invI2 * [r1.Y*r1.Y -r1.X*r1.Y]
            //        [    0     1/m1+1/m2]           [-r1.X*r1.Y r1.X*r1.X]           [-r1.X*r1.Y r1.X*r1.X]
            Mat22 K = new Mat22();
            K.ex.x = _invMassA + _invIA * _rA.y * _rA.y + _gamma;
            K.ex.y = -_invIA * _rA.x * _rA.y;
            K.ey.x = K.ex.y;
            K.ey.y = _invMassA + _invIA * _rA.x * _rA.x + _gamma;

            _mass = K.Inverse;

            _C = cA + _rA - _worldAnchor;
            _C *= _beta;

            // Cheat with some damping
            wA *= 0.98f;

            if (Settings.EnableWarmstarting)
            {
                _impulse *= data.step.dtRatio;
                vA += _invMassA * _impulse;
                wA += _invIA * MathUtils.Cross(_rA, _impulse);
            }
            else
            {
                _impulse = TSVector2.zero;
            }

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            TSVector2 vA = data.velocities[_indexA].v;
            FP wA = data.velocities[_indexA].w;

            // Cdot = v + cross(w, r)
            TSVector2 Cdot = vA + MathUtils.Cross(wA, _rA);
            TSVector2 impulse = MathUtils.Mul(ref _mass, -(Cdot + _C + _gamma * _impulse));

            TSVector2 oldImpulse = _impulse;
            _impulse += impulse;
            FP maxImpulse = data.step.dt * MaxForce;
            if (_impulse.LengthSquared() > maxImpulse * maxImpulse)
            {
                _impulse *= maxImpulse / _impulse.magnitude;
            }
            impulse = _impulse - oldImpulse;

            vA += _invMassA * impulse;
            wA += _invIA * MathUtils.Cross(_rA, impulse);

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            return true;
        }
    }
}