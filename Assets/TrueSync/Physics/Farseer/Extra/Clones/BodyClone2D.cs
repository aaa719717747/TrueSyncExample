using System.Collections.Generic;

namespace TrueSync.Physics2D {

    internal class BodyClone2D {

        private static ResourcePoolShapeClone2D poolClone2D = new ResourcePoolShapeClone2D();

        public FP _angularDamping;
        public BodyType _bodyType;
        public FP _inertia;
        public FP _linearDamping;
        public FP _mass;
        public bool _sleepingAllowed = true;
        public bool _awake = true;
        public bool _fixedRotation;

        public bool _enabled = true;
        public FP _angularVelocity;
        public TSVector2 _linearVelocity;
        public TSVector2 _force;
        public FP _invI;
        public FP _invMass;
        public FP _sleepTime;
        public Sweep _sweep = new Sweep();
        public FP _torque;
        public Transform _xf = new Transform();
        public bool _island;
        public bool disabled;

        public FP GravityScale;
        public bool IsBullet;
        public bool IgnoreCCD;
        public bool IgnoreGravity;

        public FP prevKinematicMass;
        public FP prevKinematicInvMass;
        public FP prevKinematicInertia;
        public FP prevKinematicInvI;
        public Sweep prevKinematicSweep;

        public ContactEdgeClone2D contactEdgeClone;

        public List<GenericShapeClone2D> shapesClone = new List<GenericShapeClone2D>();

        public void Reset() {
            if (contactEdgeClone != null) {
                WorldClone2D.poolContactEdgeClone.GiveBack(contactEdgeClone);
            }
        }

        public void Clone(Body body) {
            this._angularDamping = body._angularDamping;
            this._bodyType = body._bodyType;
            this._inertia = body._inertia;
            this._linearDamping = body._linearDamping;
            this._mass = body._mass;
            this._sleepingAllowed = body._sleepingAllowed;
            this._awake = body._awake;
            this._fixedRotation = body._fixedRotation;
            this._enabled = body._enabled;
            this._angularVelocity = body._angularVelocity;
            this._linearVelocity = body._linearVelocity;
            this._force = body._force;
            this._invI = body._invI;
            this._invMass = body._invMass;
            this._sleepTime = body._sleepTime;

            this._sweep.A = body._sweep.A;
            this._sweep.A0 = body._sweep.A0;
            this._sweep.Alpha0 = body._sweep.Alpha0;
            this._sweep.C = body._sweep.C;
            this._sweep.C0 = body._sweep.C0;
            this._sweep.LocalCenter = body._sweep.LocalCenter;

            this._torque = body._torque;

            this._xf.p = body._xf.p;
            this._xf.q = body._xf.q;

            this._island = body._island;
            this.disabled = body.disabled;

            this.GravityScale = body.GravityScale;
            this.IsBullet = body.IsBullet;
            this.IgnoreCCD = body.IgnoreCCD;
            this.IgnoreGravity = body.IgnoreGravity;

            this.prevKinematicMass = body.prevKinematicMass;
            this.prevKinematicInvMass = body.prevKinematicInvMass;
            this.prevKinematicInertia = body.prevKinematicInertia;
            this.prevKinematicInvI = body.prevKinematicInvI;
            this.prevKinematicSweep = body.prevKinematicSweep;

            for (int index = 0, length = shapesClone.Count; index < length; index++) {
                poolClone2D.GiveBack(shapesClone[index]);
            }

            this.shapesClone.Clear();

            List<Physics2D.Fixture> fixtureList = body.FixtureList;
            for (int index = 0, length = fixtureList.Count; index < length; index++) {
                GenericShapeClone2D shapeClone = poolClone2D.GetNew();
                shapeClone.Clone(body.FixtureList[index].Shape);

                this.shapesClone.Add(shapeClone);
            }

            if (body.ContactList == null) {
                this.contactEdgeClone = null;
            } else {
                this.contactEdgeClone = WorldClone2D.poolContactEdgeClone.GetNew();
                this.contactEdgeClone.Clone(body.ContactList);
            }
        }

		public void Restore(Physics2D.Body body) {
            body._angularDamping = this._angularDamping;
            body._bodyType = this._bodyType;
            body._inertia = this._inertia;
            body._linearDamping = this._linearDamping;
            body._mass = this._mass;
            body._sleepingAllowed = this._sleepingAllowed;
            body._awake = this._awake;
            body._fixedRotation = this._fixedRotation;
            body._enabled = this._enabled;
            body._angularVelocity = this._angularVelocity;
            body._linearVelocity = this._linearVelocity;
            body._force = this._force;
            body._invI = this._invI;
            body._invMass = this._invMass;
            body._sleepTime = this._sleepTime;
            body._sweep = this._sweep;
            body._torque = this._torque;

            body._xf.p = this._xf.p;
            body._xf.q = this._xf.q;

            body._island = this._island;

            bool lastDisabled = body.disabled;
            body.disabled = this.disabled;

            if (lastDisabled && !body.disabled) {
                Physics2D.ContactManager.physicsManager.GetGameObject(body).SetActive(true);
            }

            body.GravityScale = this.GravityScale;
            body.IsBullet = this.IsBullet;
            body.IgnoreCCD = this.IgnoreCCD;
            body.IgnoreGravity = this.IgnoreGravity;

            body.prevKinematicMass = this.prevKinematicMass;
            body.prevKinematicInvMass = this.prevKinematicInvMass;
            body.prevKinematicInertia = this.prevKinematicInertia;
            body.prevKinematicInvI = this.prevKinematicInvI;
            body.prevKinematicSweep = this.prevKinematicSweep;

            List<Physics2D.Fixture> fixtureList = body.FixtureList;
            for (int index = 0, length = this.shapesClone.Count; index < length; index++) {
                this.shapesClone[index].Restore(fixtureList[index].Shape);
            }
        }

    }

}