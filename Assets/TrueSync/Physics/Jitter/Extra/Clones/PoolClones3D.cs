namespace TrueSync.Physics3D {

    public class ResourcePoolRigidBodyClone : ResourcePool<RigidBodyClone> {

        protected override RigidBodyClone NewInstance() {
            return new RigidBodyClone();
        }

    }

    public class ResourcePoolArbiterClone : ResourcePool<ArbiterClone> {

        protected override ArbiterClone NewInstance() {
            return new ArbiterClone();
        }

    }

    public class ResourcePoolCollisionIslandClone : ResourcePool<CollisionIslandClone> {

        protected override CollisionIslandClone NewInstance() {
            return new CollisionIslandClone();
        }

    }

    public class ResourcePoolContactClone : ResourcePool<ContactClone> {

        protected override ContactClone NewInstance() {
            return new ContactClone();
        }

    }

    public class ResourcePoolSweetPointClone : ResourcePool<SweetPointClone> {

        protected override SweetPointClone NewInstance() {
            return new SweetPointClone();
        }

    }

    public class ResourcePoolGenericShapeClone : ResourcePool<GenericShapeClone> {

        protected override GenericShapeClone NewInstance() {
            return new GenericShapeClone();
        }

    }

}