using System.Collections.Generic;

namespace TrueSync.Physics3D {

    public class ArbiterClone {

		public static ResourcePoolContactClone poolContactClone = new ResourcePoolContactClone();

		public RigidBody body1;

		public RigidBody body2;

		public List<ContactClone> contactList = new List<ContactClone>();

        private int index, length;

        public void Reset() {
            for (index = 0, length = contactList.Count; index < length; index++) {
                poolContactClone.GiveBack(contactList[index]);
            }
        }

		public void Clone(Arbiter arb) {
			body1 = arb.body1;
			body2 = arb.body2;

			contactList.Clear ();

            for (index = 0, length = arb.contactList.Count; index < length; index++) {
				ContactClone contactClone = poolContactClone.GetNew();
				contactClone.Clone (arb.contactList[index]);

				contactList.Add (contactClone);	
			}
		}

		public void Restore(Arbiter arb) {
			arb.body1 = body1;
			arb.body2 = body2;

			arb.contactList.Clear ();

            for (index = 0, length = contactList.Count; index < length; index++) {
                ContactClone cc = contactList[index];

                Contact contact = Contact.Pool.GetNew();
                cc.Restore(contact);

                arb.contactList.Add(contact);
            }
        }

    }

}