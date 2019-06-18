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
using System.Collections.ObjectModel;
#endregion

namespace TrueSync.Physics3D
{

    /// <summary>
    /// This class brings 'dynamics' and 'collisions' together. It handles
    /// all bodies and constraints.
    /// </summary>
    public class World : IWorld
    {
        public delegate void WorldStep(FP timestep);

        public class WorldEvents
        {
            // Post&Prestep
            public event WorldStep PreStep;
            public event WorldStep PostStep;

            // Add&Remove
            public event Action<RigidBody> AddedRigidBody;
            public event Action<RigidBody> RemovedRigidBody;
            public event Action<Constraint> AddedConstraint;
            public event Action<Constraint> RemovedConstraint;
            public event Action<SoftBody> AddedSoftBody;
            public event Action<SoftBody> RemovedSoftBody;

            // Collision
            public event Action<Contact> BodiesBeginCollide;
			public event Action<Contact> BodiesStayCollide;
            public event Action<RigidBody, RigidBody> BodiesEndCollide;

            public event Action<Contact> TriggerBeginCollide;
            public event Action<Contact> TriggerStayCollide;
            public event Action<RigidBody, RigidBody> TriggerEndCollide;

            public event Action<Contact> ContactCreated;

            // Deactivation
            public event Action<RigidBody> DeactivatedBody;
            public event Action<RigidBody> ActivatedBody;

            internal WorldEvents() { }

            #region Raise Events

            internal void RaiseWorldPreStep(FP timestep)
            {
                if (PreStep != null) PreStep(timestep);
            }

            internal void RaiseWorldPostStep(FP timestep)
            {
                if (PostStep != null) PostStep(timestep);
            }

            internal void RaiseAddedRigidBody(RigidBody body)
            {
                if (AddedRigidBody != null) AddedRigidBody(body);
            }

            internal void RaiseRemovedRigidBody(RigidBody body)
            {
                if (RemovedRigidBody != null) RemovedRigidBody(body);
            }

            internal void RaiseAddedConstraint(Constraint constraint)
            {
                if (AddedConstraint != null) AddedConstraint(constraint);
            }

            internal void RaiseRemovedConstraint(Constraint constraint)
            {
                if (RemovedConstraint != null) RemovedConstraint(constraint);
            }

            internal void RaiseAddedSoftBody(SoftBody body)
            {
                if (AddedSoftBody != null) AddedSoftBody(body);
            }

            internal void RaiseRemovedSoftBody(SoftBody body)
            {
                if (RemovedSoftBody != null) RemovedSoftBody(body);
            }

            internal void RaiseBodiesBeginCollide(Contact contact)
            {
                if (BodiesBeginCollide != null) BodiesBeginCollide(contact);
            }

			internal void RaiseBodiesStayCollide(Contact contact)
			{
				if (BodiesStayCollide != null) BodiesStayCollide(contact);
			}

            internal void RaiseBodiesEndCollide(RigidBody body1, RigidBody body2)
            {
                if (BodiesEndCollide != null) BodiesEndCollide(body1, body2);
            }

            internal void RaiseTriggerBeginCollide(Contact contact) {
                if (TriggerBeginCollide != null) TriggerBeginCollide(contact);
            }

            internal void RaiseTriggerStayCollide(Contact contact) {
                if (TriggerStayCollide != null) TriggerStayCollide(contact);
            }

            internal void RaiseTriggerEndCollide(RigidBody body1, RigidBody body2) {
                if (TriggerEndCollide != null) TriggerEndCollide(body1, body2);
            }

            internal void RaiseActivatedBody(RigidBody body)
            {
                if (ActivatedBody != null) ActivatedBody(body);
            }

            internal void RaiseDeactivatedBody(RigidBody body)
            {
                if (DeactivatedBody != null) DeactivatedBody(body);
            }

            internal void RaiseContactCreated(Contact contact)
            {
                if (ContactCreated != null) ContactCreated(contact);
            }

            #endregion
        }

        private ContactSettings contactSettings = new ContactSettings();

        private FP inactiveAngularThresholdSq = FP.EN1;
        private FP inactiveLinearThresholdSq = FP.EN1;
        private FP deactivationTime = 2;

        private int contactIterations = 6;
        private int smallIterations = 3;
        private FP timestep = FP.Zero;

        public IslandManager islands = new IslandManager();

		public HashList<OverlapPairContact> initialCollisions = new HashList<OverlapPairContact>();
        public HashList<OverlapPairContact> initialTriggers = new HashList<OverlapPairContact>();
        private OverlapPairContact cacheOverPairContact = new OverlapPairContact(null, null);

        internal HashList<RigidBody> rigidBodies = new HashList<RigidBody>();
        internal HashList<Constraint> constraints = new HashList<Constraint>();
        internal HashList<SoftBody> softbodies = new HashList<SoftBody>();

		public ReadOnlyHashset<RigidBody> RigidBodies { get; private set; }
		public ReadOnlyHashset<Constraint> Constraints { get; private set; }
		public ReadOnlyHashset<SoftBody> SoftBodies { get; private set; }

        private WorldEvents events = new WorldEvents();
        public WorldEvents Events { get { return events; } }

        /// <summary>
        /// Holds a list of <see cref="Arbiter"/>. All currently
        /// active arbiter in the <see cref="World"/> are stored in this map.
        /// </summary>
        public ArbiterMap ArbiterMap { get { return arbiterMap; } }
        private ArbiterMap arbiterMap;

        public ArbiterMap ArbiterTriggerMap { get { return arbiterTriggerMap; } }
        private ArbiterMap arbiterTriggerMap;

        public Queue<Arbiter> removedArbiterQueue = new Queue<Arbiter>();
        public Queue<Arbiter> addedArbiterQueue = new Queue<Arbiter>();

        private TSVector gravity = new TSVector(0, -981 * FP.EN2, 0);

        public ContactSettings ContactSettings { get { return contactSettings; } }

        /// <summary>
        /// Gets a read only collection of the <see cref="TrueSync.CollisionIsland"/> objects managed by
        /// this class.
        /// </summary>
        public List<CollisionIsland> Islands { get { return islands; } }

        private Action<object> arbiterCallback;
        private Action<object> integrateCallback;

        private CollisionDetectedHandler collisionDetectionHandler;

        public IPhysicsManager physicsManager;

        /// <summary>
        /// Create a new instance of the <see cref="World"/> class.
        /// </summary>
        /// <param name="collision">The collisionSystem which is used to detect
        /// collisions. See for example: <see cref="CollisionSystemSAP"/>
        /// or <see cref="CollisionSystemBrute"/>.
        /// </param>
        public World(CollisionSystem collision)
        {
            if (collision == null)
                throw new ArgumentNullException("The CollisionSystem can't be null.", "collision");

            RigidBody.instanceCount = 0;
            Constraint.instanceCount = 0;

            arbiterCallback = new Action<object>(ArbiterCallback);
            integrateCallback = new Action<object>(IntegrateCallback);

            // Create the readonly wrappers
			this.RigidBodies = new ReadOnlyHashset<RigidBody>(rigidBodies);
			this.Constraints = new ReadOnlyHashset<Constraint>(constraints);
			this.SoftBodies = new ReadOnlyHashset<SoftBody>(softbodies);

            this.CollisionSystem = collision;

            collisionDetectionHandler = new CollisionDetectedHandler(CollisionDetected);

            this.CollisionSystem.CollisionDetected += collisionDetectionHandler;

            this.arbiterMap = new ArbiterMap();
            this.arbiterTriggerMap = new ArbiterMap();

            AllowDeactivation = false;
        }

        public void AddBody(SoftBody body)
        {
            if (body == null) throw new ArgumentNullException("body", "body can't be null.");
            if (softbodies.Contains(body)) throw new ArgumentException("The body was already added to the world.", "body");

            this.softbodies.Add(body);
            this.CollisionSystem.AddEntity(body);

            events.RaiseAddedSoftBody(body);

            foreach (Constraint constraint in body.EdgeSprings)
                AddConstraint(constraint);

            foreach (SoftBody.MassPoint massPoint in body.VertexBodies)
            {
                events.RaiseAddedRigidBody(massPoint);
                rigidBodies.Add(massPoint);
            }
        }

        public bool RemoveBody(SoftBody body)
        {
            if (!this.softbodies.Remove(body)) return false;

            this.CollisionSystem.RemoveEntity(body);

            events.RaiseRemovedSoftBody(body);

            foreach (Constraint constraint in body.EdgeSprings)
                RemoveConstraint(constraint);

            foreach (SoftBody.MassPoint massPoint in body.VertexBodies)
                RemoveBody(massPoint, true);

            return true;
        }

        /// <summary>
        /// Gets the <see cref="CollisionSystem"/> used
        /// to detect collisions.
        /// </summary>
        public CollisionSystem CollisionSystem { set; get; }

        /// <summary>
        /// In Jitter many objects get added to stacks after they were used.
        /// If a new object is needed the old object gets removed from the stack
        /// and is reused. This saves some time and also garbage collections.
        /// Calling this method removes all cached objects from all
        /// stacks.
        /// </summary>
        public void ResetResourcePools()
        {
            IslandManager.Pool.ResetResourcePool();
            Arbiter.Pool.ResetResourcePool();
            Contact.Pool.ResetResourcePool();
        }

        /// <summary>
        /// Removes all objects from the world and removes all memory cached objects.
        /// </summary>
        public void Clear()
        {
            // remove bodies from collision system
            for (int index = 0, length = rigidBodies.Count; index < length; index++) {
                RigidBody body = rigidBodies[index];

                CollisionSystem.RemoveEntity(body);

                if (body.island != null)
                {
                    body.island.ClearLists();
                    body.island = null;
                }

                body.connections.Clear();
                body.arbiters.Clear();
                body.constraints.Clear();

                events.RaiseRemovedRigidBody(body);
            }

            for (int index = 0, length = softbodies.Count; index < length; index++) {
                SoftBody body = softbodies[index];
                CollisionSystem.RemoveEntity(body);
            }

            // remove bodies from the world
            rigidBodies.Clear();

            // remove constraints
            for (int index = 0, length = constraints.Count; index < length; index++) {
                Constraint constraint = constraints[index];
                events.RaiseRemovedConstraint(constraint);
            }
            constraints.Clear();

            softbodies.Clear();

            // remove all islands
            islands.RemoveAll();

            // delete the arbiters
            arbiterMap.Clear();
            arbiterTriggerMap.Clear();

            ResetResourcePools();
        }

        /// <summary>
        /// Gets or sets the gravity in this <see cref="World"/>. The default gravity
        /// is (0,-9.81,0)
        /// </summary>
        public TSVector Gravity { get { return gravity; } set { gravity = value; } }

        /// <summary>
        /// Global sets or gets if a body is able to be temporarily deactivated by the engine to
        /// safe computation time. Use <see cref="SetInactivityThreshold"/> to set parameters
        /// of the deactivation process.
        /// </summary>
        public bool AllowDeactivation { get; set; }

        /// <summary>
        /// Sets parameters for the <see cref="RigidBody"/> deactivation process.
        /// If the bodies angular velocity is less than the angular velocity threshold
        /// and its linear velocity is lower then the linear velocity threshold for a 
        /// specific time the body gets deactivated. A body can be reactivated by setting
        /// <see cref="RigidBody.IsActive"/> to true. A body gets also automatically
        /// reactivated if another moving object hits it or the <see cref="CollisionIsland"/>
        /// the object is in gets activated.
        /// </summary>
        /// <param name="angularVelocity">The threshold value for the angular velocity. The default value
        /// is 0.1.</param>
        /// <param name="linearVelocity">The threshold value for the linear velocity. The default value
        /// is 0.1</param>
        /// <param name="time">The threshold value for the time in seconds. The default value is 2.</param>
        public void SetInactivityThreshold(FP angularVelocity, FP linearVelocity, FP time)
        {
            if (angularVelocity < FP.Zero) throw new ArgumentException("Angular velocity threshold has to " +
                 "be larger than zero", "angularVelocity");

            if (linearVelocity < FP.Zero) throw new ArgumentException("Linear velocity threshold has to " +
                "be larger than zero", "linearVelocity");

            if (time < FP.Zero) throw new ArgumentException("Deactivation time threshold has to " +
                "be larger than zero", "time");

            this.inactiveAngularThresholdSq = angularVelocity * angularVelocity;
            this.inactiveLinearThresholdSq = linearVelocity * linearVelocity;
            this.deactivationTime = time;
        }

        /// <summary>
        /// Jitter uses an iterativ approach to solve collisions and contacts. You can set the number of
        /// iterations Jitter should do. In general the more iterations the more stable a simulation gets
        /// but also costs computation time.
        /// </summary>
        /// <param name="iterations">The number of contact iterations. Default value 10.</param>
        /// <param name="smallIterations">The number of contact iteration used for smaller (two and three constraint) systems. Default value 4.</param>
        /// <remarks>The number of iterations for collision and contact should be between 3 - 30.
        /// More iterations means more stability and also a longer calculation time.</remarks>
        public void SetIterations(int iterations, int smallIterations)
        {
            if (iterations < 1) throw new ArgumentException("The number of collision " +
                 "iterations has to be larger than zero", "iterations");

            if (smallIterations < 1) throw new ArgumentException("The number of collision " +
                "iterations has to be larger than zero", "smallIterations");

            this.contactIterations = iterations;
            this.smallIterations = smallIterations;
        }

        /// <summary>
        /// Removes a <see cref="RigidBody"/> from the world.
        /// </summary>
        /// <param name="body">The body which should be removed.</param>
        /// <returns>Returns false if the body could not be removed from the world.</returns>
        public bool RemoveBody(RigidBody body)
        {
            return RemoveBody(body, false);
        }

        private bool RemoveBody(RigidBody body, bool removeMassPoints)
        {
            // Its very important to clean up, after removing a body
            if (!removeMassPoints && body.IsParticle) return false;

            // remove the body from the world list
            if (!rigidBodies.Remove(body)) return false;

            // Remove all connected constraints and arbiters
            for (int index = 0, length = body.arbiters.Count; index < length; index++) {
                Arbiter arbiter = body.arbiters[index];

                arbiterMap.Remove(arbiter);

                events.RaiseBodiesEndCollide(arbiter.body1, arbiter.body2);

                cacheOverPairContact.SetBodies(arbiter.body1, arbiter.body2);
				initialCollisions.Remove (cacheOverPairContact);
            }

            for (int index = 0, length = body.arbitersTrigger.Count; index < length; index++) {
                Arbiter arbiter = body.arbitersTrigger[index];
                arbiterTriggerMap.Remove(arbiter);

                if (arbiter.body1.isColliderOnly) {
                    events.RaiseTriggerEndCollide(arbiter.body1, arbiter.body2);
                } else {
                    events.RaiseTriggerEndCollide(arbiter.body2, arbiter.body1);
                }

                cacheOverPairContact.SetBodies(arbiter.body1, arbiter.body2);
                initialTriggers.Remove(cacheOverPairContact);
            }

            for (int index = 0, length = body.constraints.Count; index < length; index++) {
                Constraint constraint = body.constraints[index];

                constraints.Remove(constraint);
                events.RaiseRemovedConstraint(constraint);
            }

            // remove the body from the collision system
            CollisionSystem.RemoveEntity(body);

            // remove the body from the island manager
            islands.RemoveBody(body);

            events.RaiseRemovedRigidBody(body);

            return true;
        }


        /// <summary>
        /// Adds a <see cref="RigidBody"/> to the world.
        /// </summary>
        /// <param name="body">The body which should be added.</param>
        public void AddBody(RigidBody body)
        {
            if (body == null) throw new ArgumentNullException("body", "body can't be null.");
            if(rigidBodies.Contains(body)) throw new ArgumentException("The body was already added to the world.", "body");

            events.RaiseAddedRigidBody(body);

            this.CollisionSystem.AddEntity(body);

            rigidBodies.Add(body);
        }

        /// <summary>
        /// Add a <see cref="Constraint"/> to the world. Fast, O(1).
        /// </summary>
        /// <param name="constraint">The constraint which should be added.</param>
        /// <returns>True if the constraint was successfully removed.</returns>
        public bool RemoveConstraint(Constraint constraint)
        {
            if (!constraints.Remove(constraint)) return false;
            events.RaiseRemovedConstraint(constraint);

            islands.ConstraintRemoved(constraint);

            return true;
        }

        /// <summary>
        /// Add a <see cref="Constraint"/> to the world.
        /// </summary>
        /// <param name="constraint">The constraint which should be removed.</param>
        public void AddConstraint(Constraint constraint)
        {
            if(constraints.Contains(constraint)) 
                throw new ArgumentException("The constraint was already added to the world.", "constraint");

            constraints.Add(constraint);

            islands.ConstraintCreated(constraint);

            events.RaiseAddedConstraint(constraint);
        }

        /// <summary>
        /// Integrates the whole world a timestep further in time.
        /// </summary>
        /// <param name="timestep">The timestep in seconds. 
        /// It should be small as possible to keep the simulation stable.
        /// The physics simulation shouldn't run slower than 60fps.
        /// (timestep=1/60).</param>
        public void Step(FP timestep)
        {
            this.timestep = timestep;

            // yeah! nothing to do!
            if (timestep == FP.Zero) return;

            // throw exception if the timestep is smaller zero.
            if (timestep < FP.Zero) throw new ArgumentException("The timestep can't be negative.", "timestep");

            // Calculate this
			//currentAngularDampFactor = (FP)Math.Pow((double)(float)angularDamping, (double)(float)timestep);
			//currentLinearDampFactor = (FP)Math.Pow((double)(float)linearDamping, (double)(float)timestep);

#if(WINDOWS_PHONE)
            events.RaiseWorldPreStep(timestep);
            foreach (RigidBody body in rigidBodies) body.PreStep(timestep);
            UpdateContacts();

            while (removedArbiterQueue.Count > 0) islands.ArbiterRemoved(removedArbiterQueue.Dequeue());

            foreach (SoftBody body in softbodies)
            {
                body.Update(timestep);
                body.DoSelfCollision(collisionDetectionHandler);
            }

            CollisionSystem.Detect();
           
            while (addedArbiterQueue.Count > 0) islands.ArbiterCreated(addedArbiterQueue.Dequeue());

            CheckDeactivation();

            IntegrateForces();
            HandleArbiter(contactIterations);
            Integrate();

            foreach (RigidBody body in rigidBodies) body.PostStep(timestep);
            events.RaiseWorldPostStep(timestep);
#else
            events.RaiseWorldPreStep(timestep);

            UpdateContacts();

            for (int index = 0, length = initialCollisions.Count; index < length; index++) {
                OverlapPairContact op = initialCollisions[index];
                events.RaiseBodiesStayCollide(op.contact);
			}

            for (int index = 0, length = initialTriggers.Count; index < length; index++) {
                OverlapPairContact op = initialTriggers[index];
                events.RaiseTriggerStayCollide(op.contact);
            }

            while (removedArbiterQueue.Count > 0) islands.ArbiterRemoved(removedArbiterQueue.Dequeue());

            for (int index = 0, length = softbodies.Count; index < length; index++) {
                SoftBody body = softbodies[index];
                body.Update(timestep);
                body.DoSelfCollision(collisionDetectionHandler);
            }

            CollisionSystem.Detect();

            while (addedArbiterQueue.Count > 0) islands.ArbiterCreated(addedArbiterQueue.Dequeue());
            
            CheckDeactivation();

            IntegrateForces();

            HandleArbiter(contactIterations);

            Integrate();

            for (int index = 0, length = rigidBodies.Count; index < length; index++) {
                RigidBody body = rigidBodies[index];
                body.PostStep();

                for (int index2 = 0, length2 = body.constraints.Count; index2 < length2; index2++) {
                    body.constraints[index2].PostStep();
                }
            }

            events.RaiseWorldPostStep(timestep);
#endif
        }

		public FP accumulatedTime = FP.Zero;

        /// <summary>
        /// Integrates the whole world several fixed timestep further in time.
        /// </summary>
        /// <param name="totalTime">The time to integrate.</param>
        /// <param name="timestep">The timestep in seconds. 
        /// It should be small as possible to keep the simulation stable.
        /// The physics simulation shouldn't run slower than 60fps.
        /// (timestep=1/60).</param>
        /// <param name="maxSteps">The maximum number of substeps. After that Jitter gives up
        /// to keep up with the given totalTime.</param>
        public void Step(FP totalTime, FP timestep, int maxSteps)
        {
            int counter = 0;
            accumulatedTime += totalTime;

            while (accumulatedTime > timestep)
            {
                Step(timestep);

                accumulatedTime -= timestep;
                counter++;

                if (counter > maxSteps)
                {
                    // okay, okay... we can't keep up
                    accumulatedTime = FP.Zero;
                    break;
                }
            }

        }

        private void UpdateArbiterContacts(Arbiter arbiter)
        {
            if (arbiter.contactList.Count == 0)
            {
                lock (removedArbiterStack) { removedArbiterStack.Push(arbiter); }
                return;
            }

            for (int i = arbiter.contactList.Count - 1; i >= 0; i--)
            {
                Contact c = arbiter.contactList[i];
                c.UpdatePosition();

                if (c.penetration < -contactSettings.breakThreshold)
                {
                    Contact.Pool.GiveBack(c);
                    arbiter.contactList.RemoveAt(i);
                    continue;
                }
                else
                {
                    TSVector diff; TSVector.Subtract(ref c.p1, ref c.p2, out diff);
                    FP distance = TSVector.Dot(ref diff, ref c.normal);

                    diff = diff - distance * c.normal;
                    distance = diff.sqrMagnitude;

                    // hack (multiplication by factor 100) in the
                    // following line.
                    if (distance > contactSettings.breakThreshold * contactSettings.breakThreshold * 100)
                    {
                        Contact.Pool.GiveBack(c);
                        arbiter.contactList.RemoveAt(i);
                        continue;
                    }
                }

            }
        }

        public Stack<Arbiter> removedArbiterStack = new Stack<Arbiter>();

        private void UpdateContacts()
        {
            UpdateContacts(arbiterMap);
            UpdateContacts(arbiterTriggerMap);
        }

        private void UpdateContacts(ArbiterMap selectedArbiterMap) {
            foreach (Arbiter arbiter in selectedArbiterMap.Arbiters) {
                UpdateArbiterContacts(arbiter);
            }

            while (removedArbiterStack.Count > 0) {
                Arbiter arbiter = removedArbiterStack.Pop();
                Arbiter.Pool.GiveBack(arbiter);
                selectedArbiterMap.Remove(arbiter);
                               
                if (selectedArbiterMap == arbiterMap) {
                    removedArbiterQueue.Enqueue(arbiter);
                    events.RaiseBodiesEndCollide(arbiter.body1, arbiter.body2);

                    cacheOverPairContact.SetBodies(arbiter.body1, arbiter.body2);
                    initialCollisions.Remove(cacheOverPairContact);
                } else {
                    if (arbiter.body1.isColliderOnly) {
                        events.RaiseTriggerEndCollide(arbiter.body1, arbiter.body2);
                    } else {
                        events.RaiseTriggerEndCollide(arbiter.body2, arbiter.body1);
                    }

                    cacheOverPairContact.SetBodies(arbiter.body1, arbiter.body2);
                    initialTriggers.Remove(cacheOverPairContact);
                }
            }
        }

        #region private void ArbiterCallback(object obj)
        private void ArbiterCallback(object obj)
        {
            CollisionIsland island = obj as CollisionIsland;

            int thisIterations;
            if (island.Bodies.Count + island.Constraints.Count > 3) thisIterations = contactIterations;
            else thisIterations = smallIterations;

            for (int i = -1; i < thisIterations; i++)
            {
                // Contact and Collision
                for (int index = 0, length = island.arbiter.Count; index < length; index++) {
                    Arbiter arbiter = island.arbiter[index];

                    int contactCount = arbiter.contactList.Count;
                    for (int e = 0; e < contactCount; e++)
                    {
                        if (i == -1) arbiter.contactList[e].PrepareForIteration(timestep);
                        else arbiter.contactList[e].Iterate();
                    }
                }

                //  Constraints
                for (int index = 0, length = island.constraints.Count; index < length; index++) {
                    Constraint c = island.constraints[index];

                    if (c.body1 != null && !c.body1.IsActive && c.body2 != null && !c.body2.IsActive)
                        continue;

                    if (i == -1) c.PrepareForIteration(timestep);
                    else c.Iterate();
                }

            }
        }
        #endregion

        private void HandleArbiter(int iterations)
        {
            for (int i = 0; i < islands.Count; i++)
            {
                if (islands[i].IsActive()) arbiterCallback(islands[i]);
            }
        }

        private void IntegrateForces()
        {
            for (int index = 0, length = rigidBodies.Count; index < length; index++) {
                RigidBody body = rigidBodies[index];
                if (!body.isStatic && body.IsActive)
                {
                    TSVector temp;
                    TSVector.Multiply(ref body.force, body.inverseMass * timestep, out temp);
                    TSVector.Add(ref temp, ref body.linearVelocity, out body.linearVelocity);

                    if (!(body.isParticle))
                    {
                        TSVector.Multiply(ref body.torque, timestep, out temp);
                        TSVector.Transform(ref temp, ref body.invInertiaWorld, out temp);
                        TSVector.Add(ref temp, ref body.angularVelocity, out body.angularVelocity);
                    }

                    if (body.affectedByGravity)
                    {
                        TSVector.Multiply(ref gravity, timestep, out temp);
                        TSVector.Add(ref body.linearVelocity, ref temp, out body.linearVelocity);
                    }
                }

                body.force.MakeZero();
                body.torque.MakeZero();

            }
        }

        #region private void IntegrateCallback(object obj)
        private void IntegrateCallback(object obj)
        {
            RigidBody body = obj as RigidBody;

            TSVector temp;
            TSVector.Multiply(ref body.linearVelocity, timestep, out temp);
            TSVector.Add(ref temp, ref body.position, out body.position);

            if (!(body.isParticle))
            {

                //exponential map
                TSVector axis;
                FP angle = body.angularVelocity.magnitude;

                if (angle < FP.EN3)
                {
                    // use Taylor's expansions of sync function
                    // axis = body.angularVelocity * (FP.Half * timestep - (timestep * timestep * timestep) * (0.020833333333f) * angle * angle);
					TSVector.Multiply(ref body.angularVelocity, (FP.Half * timestep - (timestep * timestep * timestep) * (2082 * FP.EN6) * angle * angle), out axis);
                }
                else
                {
                    // sync(fAngle) = sin(c*fAngle)/t
                    TSVector.Multiply(ref body.angularVelocity, (FP.Sin(FP.Half * angle * timestep) / angle), out axis);
                }

                TSQuaternion dorn = new TSQuaternion(axis.x, axis.y, axis.z, FP.Cos(angle * timestep * FP.Half));
                TSQuaternion ornA; TSQuaternion.CreateFromMatrix(ref body.orientation, out ornA);

                TSQuaternion.Multiply(ref dorn, ref ornA, out dorn);

                dorn.Normalize(); TSMatrix.CreateFromQuaternion(ref dorn, out body.orientation);
            }

            body.linearVelocity *= 1 / (1 + timestep * body.linearDrag);
            body.angularVelocity *= 1 / (1 + timestep * body.angularDrag);

            /*if ((body.Damping & RigidBody.DampingType.Linear) != 0)
                TSVector.Multiply(ref body.linearVelocity, currentLinearDampFactor, out body.linearVelocity);

            if ((body.Damping & RigidBody.DampingType.Angular) != 0)
                TSVector.Multiply(ref body.angularVelocity, currentAngularDampFactor, out body.angularVelocity);*/

            body.Update();

            
            if (CollisionSystem.EnableSpeculativeContacts || body.EnableSpeculativeContacts)
                body.SweptExpandBoundingBox(timestep);
        }
        #endregion


        private void Integrate()
        {
            for (int index = 0, length = rigidBodies.Count; index < length; index++) {
                RigidBody body = rigidBodies[index];
                if (body.isStatic || !body.IsActive) continue;
                integrateCallback(body);
            }
        }

        internal bool CanBodiesCollide(RigidBody body1, RigidBody body2) {
            if (body1.disabled || body2.disabled || !physicsManager.IsCollisionEnabled(body1, body2)) {
                return false;
            }

            if (body1.IsStaticNonKinematic && body2.IsStaticNonKinematic) {
                return false;
            }

            bool anyBodyColliderOnly = body1.IsColliderOnly || body2.IsColliderOnly;

            if (anyBodyColliderOnly) {
                if ((body1.IsColliderOnly && body1.IsStaticNonKinematic && body2.IsStaticNonKinematic) || (body2.IsColliderOnly && body2.IsStaticNonKinematic && body1.IsStaticNonKinematic)) {
                    return false;
                }
            } else {
                if ((body1.isKinematic && body2.isStatic) || (body2.isKinematic && body1.isStatic)) {
                    return false;
                }
            }

            return true;
        }

        private void CollisionDetected(RigidBody body1, RigidBody body2, TSVector point1, TSVector point2, TSVector normal, FP penetration) {
            bool anyBodyColliderOnly = body1.IsColliderOnly || body2.IsColliderOnly;

            Arbiter arbiter = null;
            ArbiterMap selectedArbiterMap = null;
            if (anyBodyColliderOnly) {
                selectedArbiterMap = arbiterTriggerMap;
            } else {
                selectedArbiterMap = arbiterMap;
            }

            bool arbiterCreated = false;

            lock (selectedArbiterMap) {
                selectedArbiterMap.LookUpArbiter(body1, body2, out arbiter);
                if (arbiter == null) {
                    arbiter = Arbiter.Pool.GetNew();
                    arbiter.body1 = body1; arbiter.body2 = body2;
                    selectedArbiterMap.Add(new ArbiterKey(body1, body2), arbiter);

                    arbiterCreated = true;
                }
            }

            Contact contact = null;

            if (arbiter.body1 == body1) {
                TSVector.Negate(ref normal, out normal);
                contact = arbiter.AddContact(point1, point2, normal, penetration, contactSettings);
            } else {
                contact = arbiter.AddContact(point2, point1, normal, penetration, contactSettings);
            }

            if (arbiterCreated) { 
                if (anyBodyColliderOnly) {
                    /*if (body1.isColliderOnly) {
                        events.RaiseTriggerBeginCollide(body1, body2);
                    } else {
                        events.RaiseTriggerBeginCollide(body2, body1);
                    }*/

                    events.RaiseTriggerBeginCollide(contact);

                    body1.arbitersTrigger.Add(arbiter);
                    body2.arbitersTrigger.Add(arbiter);

                    OverlapPairContact overlapContact = new OverlapPairContact(body1, body2);
                    overlapContact.contact = contact;

                    initialTriggers.Add(overlapContact);
                } else {
                    events.RaiseBodiesBeginCollide(contact);
                    addedArbiterQueue.Enqueue(arbiter);

                    OverlapPairContact overlapContact = new OverlapPairContact(body1, body2);
                    overlapContact.contact = contact;

                    initialCollisions.Add(overlapContact);
                }
            }

            if (!anyBodyColliderOnly && contact != null) events.RaiseContactCreated(contact);

        }

        private void CheckDeactivation()
        {
            if (!AllowDeactivation) {
                return;
            }

            // A body deactivation DOESN'T kill the contacts - they are stored in
            // the arbitermap within the arbiters. So, waking up ist STABLE - old
            // contacts are reused. Also the collisionislands build every frame (based 
            // on the contacts) keep the same.

            foreach (CollisionIsland island in islands)
            {
                bool deactivateIsland = true;

                // global allowdeactivation
                if (!this.AllowDeactivation) deactivateIsland = false;
                else
                {
                    for (int index = 0, length = island.bodies.Count; index < length; index++) {
                        RigidBody body = island.bodies[index];
                        // body allowdeactivation
                        if (body.AllowDeactivation && (body.angularVelocity.sqrMagnitude < inactiveAngularThresholdSq &&
                        (body.linearVelocity.sqrMagnitude < inactiveLinearThresholdSq))) {
                            body.inactiveTime += timestep;
                            if (body.inactiveTime < deactivationTime) {
                                deactivateIsland = false;
                            }
                        } else {
                            body.inactiveTime = FP.Zero;
                            deactivateIsland = false;
                        }
                    }
                }

                for (int index = 0, length = island.bodies.Count; index < length; index++) {
                    RigidBody body = island.bodies[index];

                    if (body.isActive == deactivateIsland)
                    {
                        if (body.isActive)
                        {
                            body.IsActive = false;
                            events.RaiseDeactivatedBody(body);
                        }
                        else
                        {
                            body.IsActive = true;
                            events.RaiseActivatedBody(body);
                        }
                    }
                    
                }
            }
        }

        public List<IBody> Bodies() {
            List<IBody> bodies = new List<IBody>();
            for (int index = 0; index < rigidBodies.Count; index++) {
                bodies.Add(rigidBodies[index]);
            }

            return bodies;
        }

    }
}
