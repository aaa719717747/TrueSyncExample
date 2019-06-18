using System.Collections.Generic;

namespace TrueSync.Physics3D {

    public class RigidBodyClone {

        public static ResourcePoolGenericShapeClone poolGenericShapeClone = new ResourcePoolGenericShapeClone();

        public TSVector position;
        public TSMatrix orientation;	
		public TSVector linearVelocity;
		public TSVector angularVelocity;

		public TSMatrix inertia;
		public TSMatrix invInertia;

		public TSMatrix invInertiaWorld;
		public TSMatrix invOrientation;

		public TSVector force;
		public TSVector torque;

		public GenericShapeClone shapeClone;

		public List<RigidBody> connections = new List<RigidBody>();
		public List<Constraint> constraints = new List<Constraint>();

		public bool isActive;

		public FP inactiveTime;

		public int marker;

		public bool affectedByGravity;

		public TSBBox boundingBox;

		public int internalIndex;

		public FP inverseMass;

		public bool isColliderOnly;

		public bool isStatic;

        public bool isKinematic;

        public TSVector sweptDirection;

        public bool disabled;

        public TSRigidBodyConstraints freezeConstraint;

        public TSVector _freezePosition;

        public TSMatrix _freezeRotation;

        public TSQuaternion _freezeRotationQuat;

        public bool prevKinematicGravity;

        public FP staticFriction;

        public FP restitution;

        public FP linearDrag;

        public FP angularDrag;

        private int index, length;

        public void Reset() {
            if (this.shapeClone != null) {
                poolGenericShapeClone.GiveBack(this.shapeClone);
            }
        }

		public void Clone (RigidBody rb) {
			this.marker = rb.marker;
			this.affectedByGravity = rb.affectedByGravity;
			this.boundingBox = rb.boundingBox;
			this.internalIndex = rb.internalIndex;
			this.inverseMass = rb.inverseMass;
			this.isColliderOnly = rb.isColliderOnly;
			this.isStatic = rb.isStatic;
            this.isKinematic = rb.isKinematic;
			this.sweptDirection = rb.sweptDirection;

			this.position = rb.Position;
			this.orientation = rb.Orientation;
			this.linearVelocity = rb.LinearVelocity;
			this.angularVelocity = rb.AngularVelocity;
			this.inertia = rb.Inertia;
			this.invInertia = rb.InverseInertia;
			this.invInertiaWorld = rb.InverseInertiaWorld;
			this.invOrientation = rb.invOrientation;
			this.force = rb.Force;
			this.torque = rb.Torque;

            this.shapeClone = poolGenericShapeClone.GetNew();
            this.shapeClone.Clone(rb.Shape);

			this.connections.Clear ();
            for (index = 0, length = rb.connections.Count; index < length; index++) {
                this.connections.Add (rb.connections[index]);
            }
            this.constraints.Clear();
            for (index = 0, length = rb.constraints.Count; index < length; index++) {
                this.constraints.Add(rb.constraints[index]);
            }

			this.isActive = rb.IsActive;

			this.inactiveTime = rb.inactiveTime;

			this.marker = rb.marker;

            this.disabled = rb.disabled;

            this.freezeConstraint = rb._freezeConstraints;
            this._freezePosition = rb._freezePosition;
            this._freezeRotation = rb._freezeRotation;
            this._freezeRotationQuat = rb._freezeRotationQuat;
            this.prevKinematicGravity = rb.prevKinematicGravity;
            this.linearDrag = rb.linearDrag;
            this.angularDrag = rb.angularDrag;
            this.staticFriction = rb.staticFriction;
            this.restitution = rb.restitution;
        }

		public void Restore(World world, RigidBody rb) {
			rb.marker = marker;
			rb.affectedByGravity = affectedByGravity;
			rb.boundingBox = boundingBox;
			rb.internalIndex = internalIndex;
			rb.inverseMass = inverseMass;
			rb.isColliderOnly = isColliderOnly;
			rb.isStatic = isStatic;
            rb.isKinematic = isKinematic;
            rb.sweptDirection = sweptDirection;

			rb.position = position;
			rb.orientation = orientation;
			rb.inertia = inertia;
			rb.invInertia = invInertia;
			rb.invInertiaWorld = invInertiaWorld;
			rb.invOrientation = invOrientation;
			rb.force = force;
			rb.torque = torque;

            this.shapeClone.Restore(rb.Shape);

            rb.connections.Clear ();
			rb.connections.AddRange (connections);

			rb.arbiters.Clear();
			rb.constraints.Clear();
            rb.constraints.AddRange(this.constraints);

            rb.isActive = isActive;
			rb.inactiveTime = inactiveTime;

			rb.marker = marker;

            bool lastDisabled = rb.disabled;
            rb.disabled = disabled;

            rb._freezeConstraints = freezeConstraint;
            rb._freezePosition = _freezePosition;
            rb._freezeRotation = _freezeRotation;
            rb._freezeRotationQuat = _freezeRotationQuat;
            rb.prevKinematicGravity = this.prevKinematicGravity;

            rb.linearDrag = this.linearDrag;
            rb.angularDrag = this.angularDrag;

            rb.linearVelocity = linearVelocity;
            rb.angularVelocity = angularVelocity;

            rb.staticFriction = this.staticFriction;
            rb.restitution = this.restitution;

            if (lastDisabled && !rb.disabled) {
                world.physicsManager.GetGameObject(rb).SetActive(true);
            }
        }
			
	}

}