using System.Collections.Generic;

namespace TrueSync.Physics3D {
    
    public class CollisionSystemPersistentSAPClone {

		public static ResourcePoolSweetPointClone poolSweetPointClone = new ResourcePoolSweetPointClone();

        public List<IBroadphaseEntity> bodyList = new List<IBroadphaseEntity>();

		public List<SweetPointClone> axis1 = new List<SweetPointClone>();
		public List<SweetPointClone> axis2 = new List<SweetPointClone>();
		public List<SweetPointClone> axis3 = new List<SweetPointClone>();

		public List<OverlapPair> fullOverlaps = new List<OverlapPair>();

		public List<IBroadphaseEntity> activeList = new List<IBroadphaseEntity>();

		public bool swapOrder;

        private int index, length;

        public void Reset() {
            for (index = 0, length = axis1.Count; index < length; index++) {
                poolSweetPointClone.GiveBack(axis1[index]);
            }
            for (index = 0, length = axis2.Count; index < length; index++) {
                poolSweetPointClone.GiveBack(axis2[index]);
            }
            for (index = 0, length = axis3.Count; index < length; index++) {
                poolSweetPointClone.GiveBack(axis3[index]);
            }
        }

        public void Clone(CollisionSystemPersistentSAP cs) {
            bodyList.Clear();
            for (index = 0, length = cs.bodyList.Count; index < length; index++) {
                bodyList.Add(cs.bodyList[index]);
            }

            axis1.Clear();
            for (index = 0, length = cs.axis1.Count; index < length; index++) {
                SweetPointClone sPointClone = poolSweetPointClone.GetNew ();
				sPointClone.Clone (cs.axis1[index]);

				axis1.Add (sPointClone);	
			}

            axis2.Clear();
            for (index = 0, length = cs.axis2.Count; index < length; index++) {
                SweetPointClone sPointClone = poolSweetPointClone.GetNew ();
				sPointClone.Clone (cs.axis2[index]);

				axis2.Add (sPointClone);	
			}

            axis3.Clear();
            for (index = 0, length = cs.axis3.Count; index < length; index++) {
                SweetPointClone sPointClone = poolSweetPointClone.GetNew ();
				sPointClone.Clone (cs.axis3[index]);

				axis3.Add (sPointClone);	
			}

			fullOverlaps.Clear ();
            for (index = 0, length = cs.fullOverlaps.Count; index < length; index++) {
                fullOverlaps.Add (cs.fullOverlaps[index]);
            }

            activeList.Clear();
            for (index = 0, length = cs.activeList.Count; index < length; index++) {
                activeList.Add (cs.activeList[index]);
            }

			swapOrder = cs.swapOrder;
		}

		public void Restore(CollisionSystemPersistentSAP cs) {
			cs.bodyList.Clear ();
			cs.bodyList.AddRange (bodyList);

			cs.axis1.Clear ();
            for (index = 0, length = axis1.Count; index < length; index++) {
                SweetPointClone sp = axis1[index];

				SweepPoint spN = new SweepPoint (null, false, 0);
				sp.Restore (spN);
				cs.axis1.Add (spN);
            }

            cs.axis2.Clear ();
            for (index = 0, length = axis2.Count; index < length; index++) {
                SweetPointClone sp = axis2[index];

				SweepPoint spN = new SweepPoint (null, false, 0);
				sp.Restore (spN);
				cs.axis2.Add (spN);
            }

            cs.axis3.Clear ();
            for (index = 0, length = axis3.Count; index < length; index++) {
                SweetPointClone sp = axis3[index];

				SweepPoint spN = new SweepPoint (null, false, 0);
				sp.Restore (spN);
				cs.axis3.Add (spN);
            }

            cs.fullOverlaps.Clear ();
			cs.fullOverlaps.AddRange(fullOverlaps);
				
			cs.activeList.Clear ();
			cs.activeList.AddRange (activeList);

			cs.swapOrder = swapOrder;
		}

	}
}
