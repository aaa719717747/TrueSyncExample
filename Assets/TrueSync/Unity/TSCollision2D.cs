using UnityEngine;

namespace TrueSync {

    /**
    *  @brief Represents information about a contact point
    **/
    public class TSContactPoint2D {

        /**
        *  @brief Contact point between two bodies
        **/
        public TSVector2 point;

        /**
        *  @brief Normal vector from the contact point
        **/
        public TSVector2 normal;

    }

    /**
    *  @brief Represents information about a contact between two 2D bodies
    **/
    public class TSCollision2D {

        /**
        *  @brief An array of {@link TSContactPoint}
        **/
        public TSContactPoint2D[] contacts = new TSContactPoint2D[1];

        /**
        *  @brief {@link TSCollider} of the body hit
        **/
        public TSCollider2D collider;

        /**
        *  @brief GameObject of the body hit
        **/
        public GameObject gameObject;

        /**
        *  @brief {@link TSRigidBody} of the body hit, if there is one attached
        **/
        public TSRigidBody2D rigidbody;

        /**
        *  @brief {@link TSTransform} of the body hit
        **/
        public TSTransform2D transform;

        /**
        *  @brief The {@link TSTransform} of the body hit
        **/
        public TSVector2 relativeVelocity;

        internal void Update(GameObject otherGO, Physics2D.Contact c) {
            if (this.gameObject == null) {
                this.gameObject = otherGO;
                this.collider = this.gameObject.GetComponent<TSCollider2D>();
                this.rigidbody = this.gameObject.GetComponent<TSRigidBody2D>();
                this.transform = this.collider.tsTransform;
            }

            if (c != null) {
                if (contacts[0] == null) {
                    contacts[0] = new TSContactPoint2D();
                }

                TSVector2 normal;
                Physics2D.FixedArray2<TSVector2> points;

                c.GetWorldManifold(out normal, out points);

                contacts[0].normal = normal;
                contacts[0].point = points[0];

                this.relativeVelocity = c.CalculateRelativeVelocity();
            }
        }

    }

}