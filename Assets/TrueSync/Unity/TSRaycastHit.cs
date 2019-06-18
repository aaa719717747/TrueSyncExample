using System;

namespace TrueSync
{

    /**
    *  @brief Represents few information about a raycast hit. 
    **/
    public class TSRaycastHit
	{
		public TSRigidBody rigidbody { get; set; }
		public TSCollider collider { get; set; }
		public TSTransform transform { get; set; }
		public TSVector point { get; set; }
		public TSVector normal { get; set; }
		public FP distance { get; set; }

        public TSRaycastHit() { }

		public TSRaycastHit(TSRigidBody rigidbody, TSCollider collider, TSTransform transform, TSVector normal, TSVector origin, TSVector direction, FP fraction)
		{
			this.rigidbody = rigidbody;
			this.collider = collider;
			this.transform = transform;
			this.normal = normal;
			this.point = origin + direction * fraction;
			this.distance = fraction * direction.magnitude;
		}
	}
}

