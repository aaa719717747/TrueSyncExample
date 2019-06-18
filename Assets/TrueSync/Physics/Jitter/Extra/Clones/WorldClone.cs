using System.Collections.Generic;

namespace TrueSync.Physics3D {
 
    public class WorldClone : IWorldClone {

		public static ResourcePoolRigidBodyClone poolRigidBodyClone = new ResourcePoolRigidBodyClone();
		public static ResourcePoolArbiterClone poolArbiterClone = new ResourcePoolArbiterClone();
		public static ResourcePoolCollisionIslandClone poolCollisionIslandClone = new ResourcePoolCollisionIslandClone();
	
		public Dictionary<int, RigidBodyClone> clonedPhysics = new Dictionary<int, RigidBodyClone>();
		public List<CollisionIslandClone> collisionIslands = new List<CollisionIslandClone>();
		public CollisionSystemPersistentSAPClone cloneCollision = new CollisionSystemPersistentSAPClone();

		public List<ArbiterClone> clonedArbiters = new List<ArbiterClone>();
		public List<OverlapPairContact> clonedInitialCollisions = new List<OverlapPairContact>();

        public List<ArbiterClone> clonedArbitersTrigger = new List<ArbiterClone>();
        public List<OverlapPairContact> clonedInitialTriggers = new List<OverlapPairContact>();

        public int rigidBodyInstanceCount;
        public int constraintInstanceCount;

        public List<Contact> contactsToGiveBack = new List<Contact>();
        public List<Arbiter> arbiterToGiveBack = new List<Arbiter>();
        public List<CollisionIsland> collisionIslandToGiveBack = new List<CollisionIsland>();

        private int index, length;

        public string checksum {
            get; private set;
        }

        public void Reset() {
            if (clonedPhysics != null) {
                foreach (RigidBodyClone cc in clonedPhysics.Values) {
                    cc.Reset();
                    poolRigidBodyClone.GiveBack(cc);
                }
            }

            if (collisionIslands != null) {
                for (index = 0, length = collisionIslands.Count; index < length; index++) {
                    CollisionIslandClone cc = collisionIslands[index];

                    cc.Reset();
                    poolCollisionIslandClone.GiveBack(cc);
                }
            }

            if (cloneCollision != null) {
                cloneCollision.Reset();
            }

            if (clonedArbiters != null) {
                for (index = 0, length = clonedArbiters.Count; index < length; index++) {
                    ArbiterClone cc = clonedArbiters[index];

                    cc.Reset();
                    poolArbiterClone.GiveBack(cc);
                }
            }

            if (clonedArbitersTrigger != null) {
                for (index = 0, length = clonedArbitersTrigger.Count; index < length; index++) {
                    ArbiterClone cc = clonedArbitersTrigger[index];

                    cc.Reset();
                    poolArbiterClone.GiveBack(cc);
                }
            }
        }

        public void Clone(IWorld iWorld) {
            Clone(iWorld, false);
        }

        public void Clone(IWorld iWorld, bool doChecksum) {
            World world = (World) iWorld;

            Reset();

            if (doChecksum) { 
                checksum = ChecksumExtractor.GetEncodedChecksum();
            }

			clonedPhysics.Clear();

			foreach (RigidBody rb in world.RigidBodies) {
				RigidBodyClone rigidBodyClone = poolRigidBodyClone.GetNew();
				rigidBodyClone.Clone (rb);

				clonedPhysics.Add (rb.GetInstance(), rigidBodyClone);
			}

			clonedArbiters.Clear();
			foreach (Arbiter arb in world.ArbiterMap.Arbiters) {
				ArbiterClone arbiterClone = poolArbiterClone.GetNew ();
				arbiterClone.Clone (arb);

				clonedArbiters.Add (arbiterClone);
			}

            clonedArbitersTrigger.Clear();
            foreach (Arbiter arb in world.ArbiterTriggerMap.Arbiters) {
                ArbiterClone arbiterClone = poolArbiterClone.GetNew();
                arbiterClone.Clone(arb);

                clonedArbitersTrigger.Add(arbiterClone);
            }

            collisionIslands.Clear();

            for (index = 0, length = world.islands.Count; index < length; index++) {
                CollisionIsland ci = world.islands[index];

				CollisionIslandClone collisionIslandClone = poolCollisionIslandClone.GetNew ();
				collisionIslandClone.Clone (ci);

				collisionIslands.Add (collisionIslandClone);
			}

			cloneCollision.Clone ((CollisionSystemPersistentSAP) world.CollisionSystem);

			clonedInitialCollisions.Clear();
			clonedInitialCollisions.AddRange (world.initialCollisions);

            clonedInitialTriggers.Clear();
            clonedInitialTriggers.AddRange(world.initialTriggers);

            rigidBodyInstanceCount = RigidBody.instanceCount;
            constraintInstanceCount = Constraint.instanceCount;
        }

		public void Restore(IWorld iWorld) {
            World world = (World) iWorld;

            List<RigidBody> bodiesToRemove = new List<RigidBody>();

            foreach (RigidBody rb in world.RigidBodies) {
                if (!clonedPhysics.ContainsKey(rb.GetInstance())) {
                    bodiesToRemove.Add(rb);
                }
            }

            for (index = 0, length = bodiesToRemove.Count; index < length; index++) {
                RigidBody rb = bodiesToRemove[index];

                world.RemoveBody(rb);
            }

            foreach (RigidBody rb in world.RigidBodies) {
                if (clonedPhysics.ContainsKey(rb.GetInstance())) {
                    RigidBodyClone rbClone = clonedPhysics[rb.GetInstance()];
                    rbClone.Restore(world, rb);

                    rb.island = null;
                    rb.arbiters.Clear();
                    rb.arbitersTrigger.Clear();
                }
			}

            foreach (Arbiter arb in world.ArbiterMap.Arbiters) {
                for (index = 0, length = arb.contactList.Count; index < length; index++) {
                    Contact c = arb.contactList[index];

                    contactsToGiveBack.Add(c);
                }
                arbiterToGiveBack.Add(arb);
            }
			world.ArbiterMap.Clear ();

            foreach (Arbiter arb in world.ArbiterTriggerMap.Arbiters) {
                foreach (Contact c in arb.contactList) {
                    contactsToGiveBack.Add(c);
                }

                arbiterToGiveBack.Add(arb);
            }
            world.ArbiterTriggerMap.Clear();

            for (index = 0, length = world.islands.islands.Count; index < length; index++) {
                CollisionIsland ci = world.islands.islands[index];

                collisionIslandToGiveBack.Add(ci);                
			}

            for (index = 0, length = clonedArbiters.Count; index < length; index++) {
                ArbiterClone arbC = clonedArbiters[index];

				Arbiter arbiter = Arbiter.Pool.GetNew ();
				arbC.Restore (arbiter);

                arbiter.body1.arbiters.Add (arbiter);
				arbiter.body2.arbiters.Add (arbiter);

				world.ArbiterMap.Add (new ArbiterKey(arbiter.body1, arbiter.body2), arbiter);
            }

            for (index = 0, length = clonedArbitersTrigger.Count; index < length; index++) {
                ArbiterClone arbC = clonedArbitersTrigger[index];

                Arbiter arbiter = Arbiter.Pool.GetNew();
                arbC.Restore(arbiter);

                arbiter.body1.arbitersTrigger.Add(arbiter);
                arbiter.body2.arbitersTrigger.Add(arbiter);

                world.ArbiterTriggerMap.Add(new ArbiterKey(arbiter.body1, arbiter.body2), arbiter);
            }

            world.islands.islands.Clear();

            for (index = 0, length = collisionIslands.Count; index < length; index++) {
                CollisionIslandClone ci = collisionIslands[index];

                CollisionIsland collisionIsland = IslandManager.Pool.GetNew();
                ci.Restore(collisionIsland, world);

                world.islands.islands.Add(collisionIsland);
            }

            cloneCollision.Restore ((CollisionSystemPersistentSAP) world.CollisionSystem);

			world.initialCollisions.Clear ();
			world.initialCollisions.AddRange (clonedInitialCollisions);

            world.initialTriggers.Clear();
            world.initialTriggers.AddRange(clonedInitialTriggers);

            RigidBody.instanceCount = rigidBodyInstanceCount;
            Constraint.instanceCount = constraintInstanceCount;

            for (index = 0, length = contactsToGiveBack.Count; index < length; index++) {
                Contact obj = contactsToGiveBack[index];

                Contact.Pool.GiveBack(obj);
            }
            contactsToGiveBack.Clear();

            for (index = 0, length = arbiterToGiveBack.Count; index < length; index++) {
                Arbiter obj = arbiterToGiveBack[index];

                Arbiter.Pool.GiveBack(obj);
            }
            arbiterToGiveBack.Clear();

            for (index = 0, length = collisionIslandToGiveBack.Count; index < length; index++) {
                CollisionIsland obj = collisionIslandToGiveBack[index];

                IslandManager.Pool.GiveBack(obj);
            }
            collisionIslandToGiveBack.Clear();            
        }

    }

}