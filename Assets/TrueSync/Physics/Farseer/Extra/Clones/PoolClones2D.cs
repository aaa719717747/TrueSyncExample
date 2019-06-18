namespace TrueSync.Physics2D {

    internal class ResourcePoolBodyClone2D : ResourcePool<BodyClone2D> {

        protected override BodyClone2D NewInstance() {
            return new BodyClone2D();
        }

    }

    internal class ResourcePoolContactClone2D : ResourcePool<ContactClone2D> {

        protected override ContactClone2D NewInstance() {
            return new ContactClone2D();
        }

    }

    internal class ResourcePoolContactEdgeClone2D : ResourcePool<ContactEdgeClone2D> {

        protected override ContactEdgeClone2D NewInstance() {
            return new ContactEdgeClone2D();
        }

    }

    internal class ResourcePoolContactEdge2D : ResourcePool<ContactEdge> {

        protected override ContactEdge NewInstance() {
            return new ContactEdge();
        }

    }

    internal class ResourcePoolTreeFixtureProxy2D : ResourcePool<TreeNode<FixtureProxy>> {

        protected override TreeNode<FixtureProxy> NewInstance() {
            return new TreeNode<FixtureProxy>();
        }

    }

    internal class ResourcePoolShapeClone2D : ResourcePool<GenericShapeClone2D> {

        protected override GenericShapeClone2D NewInstance() {
            return new GenericShapeClone2D();
        }

    }

}