/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System;
using System.Collections.Generic;
using System.Threading;
#endregion

namespace TrueSync.Physics3D {


    public enum RigidBodyIndex
    {
        RigidBody1, RigidBody2
    }

    /**
    *  @brief Represents a TrueSync version of Unity's Rigidbody. 
    **/
    public class RigidBody : IBroadphaseEntity, IDebugDrawable, IEquatable<RigidBody>, IComparable, IBody3D
    {
        [Flags]
        public enum DampingType { None = 0x00, Angular = 0x01, Linear = 0x02 }

        internal TSMatrix inertia;
        internal TSMatrix invInertia;

        internal TSMatrix invInertiaWorld;
        internal TSMatrix orientation;
        internal TSMatrix invOrientation;
        internal TSVector position;
        internal TSVector linearVelocity;
        internal TSVector angularVelocity;

        internal FP staticFriction;
        internal FP restitution;

        internal TSBBox boundingBox;

        internal FP inactiveTime = FP.Zero;

        internal bool isActive = true;
        internal bool isStatic = false;
        internal bool isKinematic = false;
        internal bool affectedByGravity = true;
		internal bool isColliderOnly = false;

        internal CollisionIsland island;
        internal FP inverseMass;

        internal TSVector force, torque;

        private int hashCode;

        internal int internalIndex = 0;

        private ShapeUpdatedHandler updatedHandler;

        internal List<RigidBody> connections = new List<RigidBody>();

		internal HashList<Arbiter> arbiters = new HashList<Arbiter>();
        internal HashList<Arbiter> arbitersTrigger = new HashList<Arbiter>();

        internal HashList<Constraint> constraints = new HashList<Constraint>();

		private ReadOnlyHashset<Arbiter> readOnlyArbiters;
		private ReadOnlyHashset<Constraint> readOnlyConstraints;

        internal int marker = 0;

        internal bool disabled = false;

        internal TSRigidBodyConstraints _freezeConstraints = TSRigidBodyConstraints.None;

        internal TSVector _freezePosition = TSVector.zero;

        internal TSMatrix _freezeRotation = TSMatrix.Identity;
        internal TSQuaternion _freezeRotationQuat = TSQuaternion.identity;

        // Previous state of gravity before switch Kinematic to true
        internal bool prevKinematicGravity;

        internal FP linearDrag;

        internal FP angularDrag;

        public TSRigidBodyConstraints FreezeConstraints {
            get {
                return _freezeConstraints;
            }

            set {
                if (_freezeConstraints != value) {
                    _freezeConstraints = value;

                    _freezePosition = position;

                    if (_freezeRotation != orientation) {
                        _freezeRotation = orientation;
                        _freezeRotationQuat = TSQuaternion.CreateFromMatrix(_freezeRotation);
                    }
                }
            }
        }

        public bool Disabled {
            get {
                return disabled;
            }
        }

        public RigidBody(Shape shape)
            : this(shape, new BodyMaterial(), false)
        {
        }

        internal bool isParticle = false;

        /// <summary>
        /// If true, the body as no angular movement.
        /// </summary>
        public bool IsParticle { 
            get { return isParticle; }
            set
            {
                if (isParticle && !value)
                {
                    updatedHandler = new ShapeUpdatedHandler(ShapeUpdated);
                    this.Shape.ShapeUpdated += updatedHandler;
                    SetMassProperties();
                    isParticle = false;
                }
                else if (!isParticle && value)
                {
                    this.inertia = TSMatrix.Zero;
                    this.invInertia = this.invInertiaWorld = TSMatrix.Zero;
                    this.invOrientation = this.orientation = TSMatrix.Identity;
                    inverseMass = FP.One;

                    this.Shape.ShapeUpdated -= updatedHandler;

                    isParticle = true;
                }

                Update();
            }
        }

        /// <summary>
        /// Initializes a new instance of the RigidBody class.
        /// </summary>
        /// <param name="shape">The shape of the body.</param>
        public RigidBody(Shape shape, BodyMaterial material)
            :this(shape,material,false)
        {
        }

        /// <summary>
        /// Initializes a new instance of the RigidBody class.
        /// </summary>
        /// <param name="shape">The shape of the body.</param>
        /// <param name="isParticle">If set to true the body doesn't rotate. 
        /// Also contacts are only solved for the linear motion part.</param>
        public RigidBody(Shape shape, BodyMaterial material, bool isParticle)
        {
			readOnlyArbiters = new ReadOnlyHashset<Arbiter>(arbiters);
			readOnlyConstraints = new ReadOnlyHashset<Constraint>(constraints);

            instanceCount++;
            instance = instanceCount;
            hashCode = CalculateHash(instance);

            this.Shape = shape;
            orientation = TSMatrix.Identity;

            if (!isParticle)
            {
                updatedHandler = new ShapeUpdatedHandler(ShapeUpdated);
                this.Shape.ShapeUpdated += updatedHandler;
                SetMassProperties();
            }
            else
            {
                this.inertia = TSMatrix.Zero;
                this.invInertia = this.invInertiaWorld = TSMatrix.Zero;
                this.invOrientation = this.orientation = TSMatrix.Identity;
                inverseMass = FP.One;
            }

            AllowDeactivation = true;
            EnableSpeculativeContacts = false;

            this.isParticle = isParticle;

            Update();
        }

        /// <summary>
        /// Calculates a hashcode for this RigidBody.
        /// The hashcode should be unique as possible
        /// for every body.
        /// </summary>
        /// <returns>The hashcode.</returns>
        public override int GetHashCode()
        {
            return hashCode;
        }

		internal ReadOnlyHashset<Arbiter> Arbiters { get { return readOnlyArbiters; } }
        internal ReadOnlyHashset<Constraint> Constraints { get { return readOnlyConstraints; } }

        /// <summary>
        /// If set to false the body will never be deactived by the
        /// world.
        /// </summary>
        internal bool AllowDeactivation { get; set; }

        internal bool EnableSpeculativeContacts { get; set; }

        /// <summary>
        /// The axis aligned bounding box of the body.
        /// </summary>
        public TSBBox BoundingBox { get { return boundingBox; } }


        internal static int instanceCount = 0;
        private int instance;

        private int CalculateHash(int a)
        {
            a = (a ^ 61) ^ (a >> 16);
            a = a + (a << 3);
            a = a ^ (a >> 4);
            a = a * 0x27d4eb2d;
            a = a ^ (a >> 15);
            return a;
        }

        /// <summary>
        /// Gets the current collision island the body is in.
        /// </summary>
        internal CollisionIsland CollisionIsland { get { return this.island; } }

        /// <summary>
        /// If set to false the velocity is set to zero,
        /// the body gets immediately freezed.
        /// </summary>
        public bool IsActive
        {
            get 
            {
                return isActive;
            }
            set
            {
                if (!isActive && value)
                {
                    // if inactive and should be active
                    inactiveTime = FP.Zero;
                }
                else if (isActive && !value)
                {
                    // if active and should be inactive
                    inactiveTime = FP.PositiveInfinity;
                    this.angularVelocity.MakeZero();
                    this.linearVelocity.MakeZero();
                }

                isActive = value;
            }
        }

		public bool IsColliderOnly
		{
			get 
			{
				return isColliderOnly;
			}
			set
			{
				isColliderOnly = value;
			}
		}

        /// <summary>
        /// Applies an impulse on the center of the body. Changing
        /// linear velocity.
        /// </summary>
        /// <param name="impulse">Impulse direction and magnitude.</param>
        public void ApplyImpulse(TSVector impulse)
        {
            if (this.isStatic) {
                return;
            }

            TSVector temp;
            TSVector.Multiply(ref impulse, inverseMass, out temp);
            TSVector.Add(ref linearVelocity, ref temp, out linearVelocity);
        }

        /// <summary>
        /// Applies an impulse on the specific position. Changing linear
        /// and angular velocity.
        /// </summary>
        /// <param name="impulse">Impulse direction and magnitude.</param>
        /// <param name="relativePosition">The position where the impulse gets applied
        /// in Body coordinate frame.</param>
        public void ApplyImpulse(TSVector impulse, TSVector relativePosition)
        {
            if (this.isStatic) {
                return;
            }

            TSVector temp;
            TSVector.Multiply(ref impulse, inverseMass, out temp);
            TSVector.Add(ref linearVelocity, ref temp, out linearVelocity);

            TSVector.Cross(ref relativePosition, ref impulse, out temp);
            TSVector.Transform(ref temp, ref invInertiaWorld, out temp);
            TSVector.Add(ref angularVelocity, ref temp, out angularVelocity);
        }

        /// <summary>
        /// Adds a force to the center of the body. The force gets applied
        /// the next time <see cref="World.Step"/> is called. The 'impact'
        /// of the force depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="force">The force to add next <see cref="World.Step"/>.</param>
        public void AddForce(TSVector force)
        {
            TSVector.Add(ref force, ref this.force, out this.force);
        }

        /// <summary>
        /// Adds a force to the center of the body. The force gets applied
        /// the next time <see cref="World.Step"/> is called. The 'impact'
        /// of the force depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="force">The force to add next <see cref="World.Step"/>.</param>
        /// <param name="pos">The position where the force is applied.</param>
        public void AddForce(TSVector force, TSVector pos)
        {
            TSVector.Add(ref this.force, ref force, out this.force);
            TSVector.Subtract(ref pos, ref this.position, out pos);
            TSVector.Cross(ref pos, ref force, out pos);
            TSVector.Add(ref pos, ref this.torque, out this.torque);
        }

        /// <summary>
        /// Returns the torque which acts this timestep on the body.
        /// </summary>
        public TSVector Torque { get { return torque; } }

        /// <summary>
        /// Returns the force which acts this timestep on the body.
        /// </summary>
        public TSVector Force { get { return force; } }

        /// <summary>
        /// Adds torque to the body. The torque gets applied
        /// the next time <see cref="World.Step"/> is called. The 'impact'
        /// of the torque depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="torque">The torque to add next <see cref="World.Step"/>.</param>
        public void AddTorque(TSVector torque)
        {
            TSVector.Add(ref torque, ref this.torque, out this.torque);
        }

        /// <summary>
        /// Adds torque to the body. The torque gets applied
        /// the next time <see cref="World.Step"/> is called. The 'impact'
        /// of the torque depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="torque">The torque to add next <see cref="World.Step"/>.</param>
        public void AddRelativeTorque(TSVector torque)
        {
            torque = TSVector.Transform(torque, this.TSOrientation);
            TSVector.Add(ref torque, ref this.torque, out this.torque);
        }

        protected bool useShapeMassProperties = true;

        /// <summary>
        /// By calling this method the shape inertia and mass is used.
        /// </summary>
        public void SetMassProperties()
        {
            this.inertia = Shape.inertia;
            TSMatrix.Inverse(ref inertia, out invInertia);
            this.inverseMass = FP.One / Shape.mass;
            useShapeMassProperties = true;
        }

        /// <summary>
        /// The engine used the given values for inertia and mass and ignores
        /// the shape mass properties.
        /// </summary>
        /// <param name="inertia">The inertia/inverse inertia of the untransformed object.</param>
        /// <param name="mass">The mass/inverse mass of the object.</param>
        /// <param name="setAsInverseValues">Sets the InverseInertia and the InverseMass
        /// to this values.</param>
        public void SetMassProperties(TSMatrix inertia, FP mass, bool setAsInverseValues)
        {
            if (setAsInverseValues)
            {
                if (!isParticle)
                {
                    this.invInertia = inertia;
                    TSMatrix.Inverse(ref inertia, out this.inertia);
                }
                this.inverseMass = mass;
            }
            else
            {
                if (!isParticle)
                {
                    this.inertia = inertia;
                    TSMatrix.Inverse(ref inertia, out this.invInertia);
                }
                this.inverseMass = FP.One / mass;
            }

            useShapeMassProperties = false;
            Update();
        }

        private void ShapeUpdated()
        {
            if (useShapeMassProperties) SetMassProperties();
            Update();
            UpdateHullData();
        }

        /// <summary>
        /// The shape the body is using.
        /// </summary>
        public Shape Shape 
        {
            get { return shape; } 
            set 
            {
                // deregister update event
                if(shape != null) shape.ShapeUpdated -= updatedHandler;

                // register new event
                shape = value; 
                shape.ShapeUpdated += new ShapeUpdatedHandler(ShapeUpdated); 
            } 
        }

        internal Shape shape;

        #region Properties

        private DampingType damping = DampingType.Angular | DampingType.Linear;

        public DampingType Damping { get { return damping; } set { damping = value; } }

        /// <summary>
        /// The inertia currently used for this body.
        /// </summary>
        public TSMatrix Inertia { get { return inertia; } }

        /// <summary>
        /// The inverse inertia currently used for this body.
        /// </summary>
        public TSMatrix InverseInertia { get { return invInertia; } }

        /// <summary>
        /// The velocity of the body.
        /// </summary>
        public TSVector LinearVelocity
        {
            get { return linearVelocity; }
            set 
            { 
                if (this.isStatic) {
                    return;
                }

                linearVelocity = value;
            }
        }

        // TODO: check here is static!
        /// <summary>
        /// The angular velocity of the body.
        /// </summary>
        public TSVector AngularVelocity
        {
            get { return angularVelocity; }
            set
            {
                if (this.isStatic) {
                    return;
                }

                angularVelocity = value;
            }
        }

        /// <summary>
        /// The current position of the body.
        /// </summary>
        public TSVector Position
        {
            get { return position; }
            set {
                if ((_freezeConstraints & TSRigidBodyConstraints.FreezePosition) > 0) {
                    _freezePosition = value;
                }

                position = value ; Update();
            }
        }

        /// <summary>
        /// The current oriention of the body.
        /// </summary>
        public TSMatrix Orientation
        {
            get { return orientation; }
            set {
                if ((_freezeConstraints & TSRigidBodyConstraints.FreezeRotation) > 0) {
                    _freezeRotation = value;
                    _freezeRotationQuat = TSQuaternion.CreateFromMatrix(_freezeRotation);
                }

                orientation = value; Update();
            }
        }

        /// <summary>
        /// If set to true the body can't be moved.
        /// </summary>
        public bool IsStatic
        {
            get
            {
                return isStatic;
            }
            set
            {
                if (value && !isStatic)
                {
                    if(island != null)
                        island.islandManager.MakeBodyStatic(this);

                    this.angularVelocity.MakeZero();
                    this.linearVelocity.MakeZero();
                }
                isStatic = value;
            }
        }

        public bool TSIsStatic {
            get {
                return IsStatic;
            }
            set {
                this.IsStatic = value;
            }
        }

        public bool IsKinematic {
            get {
                return isKinematic;
            }
            set {
                if (isKinematic != value) {
                    isKinematic = value;

                    if (isKinematic) {
                        prevKinematicGravity = affectedByGravity;
                        affectedByGravity = false;
                        IsStatic = true;
                    } else {
                        affectedByGravity = prevKinematicGravity;
                        IsStatic = false;
                    }
                }
            }
        }

        public bool AffectedByGravity { get { return affectedByGravity; } set { affectedByGravity = value; prevKinematicGravity = value; } }

        /// <summary>
        /// The inverse inertia tensor in world space.
        /// </summary>
        public TSMatrix InverseInertiaWorld
        {
            get
            {
                return invInertiaWorld;
            }
        }

        /// <summary>
        /// Setting the mass automatically scales the inertia.
        /// To set the mass indepedently from the mass use SetMassProperties.
        /// </summary>
        public FP Mass
        {
            get { return FP.One / inverseMass; }
            set 
            {
                if (value <= FP.Zero) throw new ArgumentException("Mass can't be less or equal zero.");

                // scale inertia
                if (!isParticle)
                {
                    TSMatrix.Multiply(ref Shape.inertia, value / Shape.mass, out inertia);
                    TSMatrix.Inverse(ref inertia, out invInertia);
                }

                inverseMass = FP.One / value;
            }
        }

        #endregion


        internal TSVector sweptDirection = TSVector.zero;

        internal void SweptExpandBoundingBox(FP timestep)
        {
            sweptDirection = linearVelocity * timestep;

            if (sweptDirection.x < FP.Zero)
            {
                boundingBox.min.x += sweptDirection.x;
            }
            else
            {
                boundingBox.max.x += sweptDirection.x;
            }

            if (sweptDirection.y < FP.Zero)
            {
                boundingBox.min.y += sweptDirection.y;
            }
            else
            {
                boundingBox.max.y += sweptDirection.y;
            }

            if (sweptDirection.z < FP.Zero)
            {
                boundingBox.min.z += sweptDirection.z;
            }
            else
            {
                boundingBox.max.z += sweptDirection.z;
            }
        }

        /// <summary>
        /// Recalculates the axis aligned bounding box and the inertia
        /// values in world space.
        /// </summary>
        public virtual void Update()
        {
            if (isParticle)
            {
                this.inertia = TSMatrix.Zero;
                this.invInertia = this.invInertiaWorld = TSMatrix.Zero;
                this.invOrientation = this.orientation = TSMatrix.Identity;
                this.boundingBox = shape.boundingBox;
                TSVector.Add(ref boundingBox.min, ref this.position, out boundingBox.min);
                TSVector.Add(ref boundingBox.max, ref this.position, out boundingBox.max);

                angularVelocity.MakeZero();
            }
            else
            {
                // Given: Orientation, Inertia
                TSMatrix.Transpose(ref orientation, out invOrientation);
                this.Shape.GetBoundingBox(ref orientation, out boundingBox);
                TSVector.Add(ref boundingBox.min, ref this.position, out boundingBox.min);
                TSVector.Add(ref boundingBox.max, ref this.position, out boundingBox.max);


                if (!isStatic)
                {
                    TSMatrix.Multiply(ref invOrientation, ref invInertia, out invInertiaWorld);
                    TSMatrix.Multiply(ref invInertiaWorld, ref orientation, out invInertiaWorld);
                }
            }
        }

        public bool Equals(RigidBody other)
        {
            return (other.instance == this.instance);
        }

        public int CompareTo(object otherObj)
        {
            RigidBody other = (RigidBody) otherObj;

            if (other.instance < this.instance) return -1;
            else if (other.instance > this.instance) return 1;
            else return 0;
        }

        internal int BroadphaseTag { get; set; }

        public bool IsStaticOrInactive
        {
            //get { return (!this.isActive || (this.isStatic && !isKinematic)); }
            get { return (!this.isActive || (this.isStatic)); }
        }

        public bool IsStaticNonKinematic {
            get { return (!this.isActive || (this.isStatic && !isKinematic)); }
        }

        private bool enableDebugDraw = false;
        internal bool EnableDebugDraw
        {
            get { return enableDebugDraw; }
            set
            {
                enableDebugDraw = value;
                UpdateHullData();
            }
        }

        public bool TSAffectedByGravity {
            get {
                return AffectedByGravity;
            }

            set {
                AffectedByGravity = value;
            }
        }

        public bool TSIsKinematic {
            get {
                return IsKinematic;
            }

            set {
                IsKinematic = value;
            }
        }

        public TSVector TSLinearVelocity {
            get {
                return LinearVelocity;
            }

            set {
                LinearVelocity = value;
            }
        }

        public TSVector TSAngularVelocity {
            get {
                return AngularVelocity;
            }

            set {
                AngularVelocity = value;
            }
        }

        public bool TSDisabled {
            get {
                return Disabled;
            }

            set {
                disabled = true;
            }
        }

        public TSVector TSPosition {
            get {
                return Position;
            }

            set {
                Position = value;
            }
        }

        public TSMatrix TSOrientation {
            get {
                return Orientation;
            }

            set {
                Orientation = value;
            }
        }

        public FP TSLinearDrag {
            get {
                return linearDrag;
            }

            set {
                linearDrag = value;
            }
        }

        public FP TSAngularDrag {
            get {
                return angularDrag;
            }

            set {
                angularDrag = value;
            }
        }

        public FP TSFriction {
            get {
                return staticFriction;
            }

            set {
                staticFriction = value;
            }
        }

        public FP TSRestitution {
            get {
                return restitution;
            }

            set {
                restitution = value;
            }
        }

        private List<TSVector> hullPoints = new List<TSVector>();

        private void UpdateHullData()
        {
            hullPoints.Clear();

            if(enableDebugDraw) shape.MakeHull(ref hullPoints, 3);
        }


        public void DebugDraw(IDebugDrawer drawer)
        {
            TSVector pos1,pos2,pos3;

            for(int i = 0;i<hullPoints.Count;i+=3)
            {
                pos1 = hullPoints[i + 0];
                pos2 = hullPoints[i + 1];
                pos3 = hullPoints[i + 2];

                TSVector.Transform(ref pos1, ref orientation, out pos1);
                TSVector.Add(ref pos1, ref position, out pos1);

                TSVector.Transform(ref pos2, ref orientation, out pos2);
                TSVector.Add(ref pos2, ref position, out pos2);

                TSVector.Transform(ref pos3, ref orientation, out pos3);
                TSVector.Add(ref pos3, ref position, out pos3);

                drawer.DrawTriangle(pos1, pos2, pos3);
            }
        }

        internal int GetInstance() {
			return instance;
		}

        internal void PostStep() {
            if (_freezeConstraints > 0) {
                bool freezePosX = (_freezeConstraints & TSRigidBodyConstraints.FreezePositionX) == TSRigidBodyConstraints.FreezePositionX;
                bool freezePosY = (_freezeConstraints & TSRigidBodyConstraints.FreezePositionY) == TSRigidBodyConstraints.FreezePositionY;
                bool freezePosZ = (_freezeConstraints & TSRigidBodyConstraints.FreezePositionZ) == TSRigidBodyConstraints.FreezePositionZ;

                if (freezePosX) {
                    position.x = _freezePosition.x;
                }

                if (freezePosY) {
                    position.y = _freezePosition.y;
                }

                if (freezePosZ) {
                    position.z = _freezePosition.z;
                }

                bool freezeRotX = (_freezeConstraints & TSRigidBodyConstraints.FreezeRotationX) == TSRigidBodyConstraints.FreezeRotationX;
                bool freezeRotY = (_freezeConstraints & TSRigidBodyConstraints.FreezeRotationY) == TSRigidBodyConstraints.FreezeRotationY;
                bool freezeRotZ = (_freezeConstraints & TSRigidBodyConstraints.FreezeRotationZ) == TSRigidBodyConstraints.FreezeRotationZ;

                if (freezeRotX || freezeRotY || freezeRotZ) {
                    TSQuaternion q = TSQuaternion.CreateFromMatrix(Orientation);

                    if (freezeRotX) {
                        q.x = _freezeRotationQuat.x;
                    }

                    if (freezeRotY) {
                        q.y = _freezeRotationQuat.y;
                    }

                    if (freezeRotZ) {
                        q.z = _freezeRotationQuat.z;
                    }

                    q.Normalize();

                    Orientation = TSMatrix.CreateFromQuaternion(q);
                }
            }
        }

        public string Checkum() {
            return string.Format("{0}|{1}", position, orientation);
        }

        public void TSApplyForce(TSVector force) {
            AddForce(force);
        }

        public void TSApplyForce(TSVector force, TSVector position) {
            AddForce(force, position);
        }

        public void TSApplyImpulse(TSVector force) {
            ApplyImpulse(force);
        }

        public void TSApplyImpulse(TSVector force, TSVector position) {
            ApplyImpulse(force, position);
        }

        public void TSApplyTorque(TSVector force) {
            AddTorque(force);
        }

        public void TSApplyRelativeTorque(TSVector force) {
            AddRelativeTorque(force);
        }

    }

}