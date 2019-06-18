using System.Collections.Generic;

namespace TrueSync.Physics3D {
    
    public class CollisionIslandClone {

		public List<RigidBody> bodies = new List<RigidBody>();
		public List<ArbiterClone> arbiters = new List<ArbiterClone>();
		public List<Constraint> constraints = new List<Constraint>();

        private int index, length;

        public void Reset() {
            for (index = 0, length = arbiters.Count; index < length; index++) {
                ArbiterClone cc = arbiters[index];
                cc.Reset();
                WorldClone.poolArbiterClone.GiveBack(cc);
            }
        }

        public void Clone(CollisionIsland ci) {
			bodies.Clear();

            for (index = 0, length = ci.bodies.Count; index < length; index++) {
                bodies.Add(ci.bodies[index]);
            }

			arbiters.Clear();
            for (index = 0, length = ci.arbiter.Count; index < length; index++) {
				ArbiterClone arbiterClone = WorldClone.poolArbiterClone.GetNew();
				arbiterClone.Clone (ci.arbiter[index]);

				arbiters.Add (arbiterClone);
			}

			constraints.Clear();
            for (index = 0, length = ci.constraints.Count; index < length; index++) {
                constraints.Add (ci.constraints[index]);
            }
        }

		public void Restore(CollisionIsland ci, World world) {
			ci.ClearLists ();

			ci.islandManager = world.islands;

            for (index = 0, length = bodies.Count; index < length; index++) {
                RigidBody rb = bodies[index];

				rb.island = ci;
				ci.bodies.Add (rb);
            }

            for (index = 0, length = arbiters.Count; index < length; index++) {
                ArbiterClone arbC = arbiters[index];

				Arbiter arbiter = null;
				world.ArbiterMap.LookUpArbiter (arbC.body1, arbC.body2, out arbiter);

				ci.arbiter.Add (arbiter);
            }

            for (index = 0, length = constraints.Count; index < length; index++) {
                Constraint cons = constraints[index];

				ci.constraints.Add (cons);
            }
		}

    }

}