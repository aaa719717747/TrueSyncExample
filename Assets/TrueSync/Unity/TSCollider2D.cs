using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace TrueSync {
    /**
     *  @brief Abstract collider for 2D shapes.
     **/
    [RequireComponent(typeof(TSTransform2D))]
    [Serializable]
    [ExecuteInEditMode]
    public abstract class TSCollider2D : MonoBehaviour, ICollider {

        private Physics2D.Shape shape;

        /**
         *  @brief Shape used by a collider.
         **/
        public Physics2D.Shape Shape {
            get {
                if (shape == null)
                    shape = CreateShape();
                return shape;
            }
            protected set { shape = value; }
        }

        [FormerlySerializedAs("isTrigger")]
        [SerializeField]
        private bool _isTrigger;

        /**
         *  @brief If it is only a trigger and doesn't interfere on collisions. 
         **/
        public bool isTrigger {

            get {
                if (_body != null) {
                    return _body.IsSensor;
                }

                return _isTrigger;
            }

            set {
                _isTrigger = value;

                if (_body != null) {
                    _body.IsSensor = value;
                }
            }

        }

        /**
         *  @brief Simulated material. 
         **/
        public TSMaterial tsMaterial;

        [SerializeField]
        private TSVector2 center;

        private Vector3 scaledCenter;

        internal Physics2D.Body _body;

        /**
         *  @brief Returns {@link RigidBody} associated to this TSRigidBody.         
         **/
        public IBody2D Body {
            get {
                if (_body == null) {
                    CheckPhysics();
                }

                return _body;
            }
        }

        /**
         *  @brief Center of the collider shape.
         **/
        public TSVector2 Center {
            get {
                return center;
            }
            set {
                center = value;
            }
        }

        /**
         *  @brief Returns a version of collider's center scaled by parent's transform.
         */
        public TSVector2 ScaledCenter {
			get {
                return TSVector2.Scale(center, new TSVector2(lossyScale.x, lossyScale.y));
			}
		}

        /**
         *  @brief Creates the shape related to a concrete implementation of TSCollider.
         **/
        public virtual Physics2D.Shape CreateShape() {
            return null;
        }

        public virtual Physics2D.Shape[] CreateShapes() {
            return new Physics2D.Shape[0];
        }

        private TSRigidBody2D tsRigidBody;

        [HideInInspector]
        public TSTransform2D tsTransform;

        /**
         *  @brief Returns the {@link TSRigidBody} attached.
         */
        public TSRigidBody2D attachedRigidbody {
            get {
                return tsRigidBody;
            }
        }

        /**
         *  @brief Holds an first value of the GameObject's lossy scale.
         **/
        [SerializeField]
        [HideInInspector]
        protected TSVector lossyScale = TSVector.one;

        /**
         *  @brief Creates a new {@link TSRigidBody} when there is no one attached to this GameObject.
         **/
        public void Awake() {
            tsTransform = this.GetComponent<TSTransform2D>();
            tsRigidBody = this.GetComponent<TSRigidBody2D>();

            if (lossyScale == TSVector.one) {
                lossyScale = TSVector.Abs(transform.localScale.ToTSVector());
            }
        }

        public void Update() {
            if (!Application.isPlaying) {
                lossyScale = TSVector.Abs(transform.lossyScale.ToTSVector());
            }
        }

        private void CreateBodyFixture(Physics2D.Body body, Physics2D.Shape shape) {
            Physics2D.Fixture fixture = body.CreateFixture(shape);

            if (tsMaterial != null) {
                fixture.Friction = tsMaterial.friction;
                fixture.Restitution = tsMaterial.restitution;
            }
        }

        private void CreateBody(Physics2D.World world) {
            Physics2D.Body body = Physics2D.BodyFactory.CreateBody(world);

            if (tsMaterial == null) {
                tsMaterial = GetComponent<TSMaterial>();
            }

            Physics2D.Shape shape = Shape;
            if (shape != null) {
                CreateBodyFixture(body, shape);
            } else {
                Physics2D.Shape[] shapes = CreateShapes();
                for (int index = 0, length = shapes.Length; index < length; index++) {
                    CreateBodyFixture(body, shapes[index]);
                }
            }

            if (tsRigidBody == null) {
                body.BodyType = Physics2D.BodyType.Static;
            } else {
                if (tsRigidBody.isKinematic) {
                    body.BodyType = TrueSync.Physics2D.BodyType.Kinematic;
                } else {
                    body.BodyType = Physics2D.BodyType.Dynamic;
                    body.IgnoreGravity = !tsRigidBody.useGravity;
                }

                if (tsRigidBody.mass <= 0) {
                    tsRigidBody.mass = 1;
                }

                body.FixedRotation = tsRigidBody.freezeZAxis;
                body.Mass = tsRigidBody.mass;
                body.TSLinearDrag = tsRigidBody.drag;
                body.TSAngularDrag = tsRigidBody.angularDrag;
            }

            body.IsSensor = isTrigger;
            body.CollidesWith = Physics2D.Category.All;

            _body = body;
        }

        /**
         *  @brief Initializes Shape and RigidBody and sets initial values to position and orientation based on Unity's transform.
         **/
        public void Initialize(Physics2D.World world) {
            CreateBody(world);
        }

        private void CheckPhysics() {
            if (_body == null && PhysicsManager.instance != null) {
                PhysicsManager.instance.AddBody(this);
            }
        }

        /**
         *  @brief Do a base matrix transformation to draw correctly all collider gizmos.
         **/
        public virtual void OnDrawGizmos() {
            if (!this.enabled) {
                return;
            }

            Vector3 position = _body != null ? _body.TSPosition.ToVector() : (transform.position + ScaledCenter.ToVector());
            Quaternion rotation = _body != null ? Quaternion.Euler(0, 0, (_body.Rotation * FP.Rad2Deg).AsFloat()) : transform.rotation;

            Gizmos.color = Color.yellow;

			Matrix4x4 cubeTransform = Matrix4x4.TRS(position, rotation, GetGizmosSize());
            Matrix4x4 oldGizmosMatrix = Gizmos.matrix;

            Gizmos.matrix *= cubeTransform;

            DrawGizmos();

            Gizmos.matrix = oldGizmosMatrix;
        }

        /**
         *  @brief Returns the gizmos size.
         **/
        protected abstract Vector3 GetGizmosSize();

        /**
         *  @brief Draws the specific gizmos of concrete collider (for example "Gizmos.DrawWireCube" for a {@link TSBoxCollider}).
         **/
        protected abstract void DrawGizmos();

        /**
         *  @brief Returns true if the body was already initialized.
         **/
        public bool IsBodyInitialized {
            get {
                return _body != null;
            }
        }

    }

}