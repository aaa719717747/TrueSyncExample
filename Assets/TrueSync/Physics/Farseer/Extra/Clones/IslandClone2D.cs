using System.Collections.Generic;

namespace TrueSync.Physics2D {

    internal class IslandClone2D {

        private List<ContactCloneKey> _contactsKeys = new List<ContactCloneKey>(10);

        public List<Body> Bodies = new List<Body>(10);
        public int BodyCount;
        public int ContactCount;
        public int JointCount;

        public List<Velocity> _velocities = new List<Velocity>(10);
        public List<Position> _positions = new List<Position>(10);

        public int BodyCapacity;
        public int ContactCapacity;
        public int JointCapacity;

        public void Clone(Island island) {
            this.Bodies.Clear();
            for (int index = 0, length = island.Bodies.Length; index < length; index++) {
                this.Bodies.Add(island.Bodies[index]);
            }

            this._velocities.Clear();
            for (int index = 0, length = island._velocities.Length; index < length; index++) {
                this._velocities.Add(island._velocities[index]);
            }

            this._positions.Clear();
            for (int index = 0, length = island._positions.Length; index < length; index++) {
                this._positions.Add(island._positions[index]);
            }

            _contactsKeys.Clear();
            for (int index = 0; index < island.ContactCount; index++) {
                _contactsKeys.Add(island._contacts[index].Key);
            }

            this.BodyCount = island.BodyCount;
            this.ContactCount = island.ContactCount;
            this.JointCount = island.JointCount;
            this.BodyCapacity = island.BodyCapacity;
            this.ContactCapacity = island.ContactCapacity;
            this.JointCapacity = island.JointCapacity;
        }

        public void Restore(Island island, Dictionary<ContactCloneKey, Physics2D.Contact> contactDic) {
            island.Reset(this.BodyCapacity, this.ContactCapacity, this.JointCapacity, island._contactManager);

            for (int index = 0, length = this.Bodies.Count; index < length; index++) {
                island.Bodies[index] = this.Bodies[index];
            }

            for (int index = 0, length = this._velocities.Count; index < length; index++) {
                island._velocities[index] = this._velocities[index];
            }

            for (int index = 0, length = this._positions.Count; index < length; index++) {
                island._positions[index] = this._positions[index];
            }

            for (int index = 0; index < ContactCount; index++) {
                island._contacts[index] = contactDic[_contactsKeys[index]];
            }

            island.BodyCount = this.BodyCount;
            island.ContactCount = this.ContactCount;
            island.JointCount = this.JointCount;
        }

    }

}