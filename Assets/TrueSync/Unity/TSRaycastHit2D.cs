namespace TrueSync
{

    /**
    *  @brief Information about a 2D cast hit.
    **/
    public class TSRaycastHit2D {

        public TSCollider2D collider;

        public TSRigidBody2D rigidbody;

        public TSTransform2D transform;

        public TSRaycastHit2D(TSCollider2D collider) {
            this.collider = collider;
            this.rigidbody = collider.GetComponent<TSRigidBody2D>();
            this.transform = collider.GetComponent<TSTransform2D>();
        }

    }

}