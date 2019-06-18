namespace TrueSync.Physics2D {

	public class ConstraintHierarchy2D : IBodyConstraint
	{

		private Body parent;

		private Body child;

		private TSVector2 childOffset;

		public ConstraintHierarchy2D(Body parent, Body child, TSVector2 childOffset) {
			this.parent = parent;
			this.child = child;

			this.childOffset = childOffset;
		}

		public void PostStep() {
            TSVector2 newPos = childOffset + parent.Position;
            child.SetTransformIgnoreContacts(ref newPos, child.Rotation);
		}

	}

}