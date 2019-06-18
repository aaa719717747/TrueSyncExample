using System;
using UnityEngine;
using UnityEngine.Serialization;
using TrueSync.Physics3D;

namespace TrueSync {
    /**
     *  @brief Abstract collider for 3D shapes. 
     **/
    [RequireComponent(typeof(TSTransform))]
    [Serializable]
    [ExecuteInEditMode]
    public abstract class TSCollider : MonoBehaviour, ICollider {

        private Shape shape;

        /**
         *  @brief Shape used by a collider.
         **/
        public Shape Shape {
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
                    return _body.IsColliderOnly;
                }

                return _isTrigger;
            }
            set {
                _isTrigger = value;

                if (_body != null) {
                    _body.IsColliderOnly = _isTrigger;
                }
            }
        }

        /**
         *  @brief Simulated material. 
         **/
        public TSMaterial tsMaterial;

        [SerializeField]
        private TSVector center;

        private Vector3 scaledCenter;

        internal RigidBody _body;

        /**
         *  @brief Center of the collider shape.
         **/
        public TSVector Center {
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
        public TSVector ScaledCenter {
			get {
				return TSVector.Scale (Center, lossyScale);
			}
		}

        /**
         *  @brief Creates the shape related to a concrete implementation of TSCollider.
         **/
        public abstract Shape CreateShape();

        private TSRigidBody tsRigidBody;

        /**
         *  @brief Returns the {@link TSRigidBody} attached.
         */
        public TSRigidBody attachedRigidbody {
            get {
                return tsRigidBody;
            }
        }

        /**
         *  @brief Returns body's boundind box.
         */
        public TSBBox bounds {
            get {
                return this._body.BoundingBox;
            }
        }

        /**
         *  @brief Returns the body linked to this collider.
         */
        public IBody3D Body {
            get {
                if (_body == null) {
                    CheckPhysics();
                }

                return _body;
            }
        }

        /**
         *  @brief Holds an first value of the GameObject's lossy scale.
         **/
        [SerializeField]
        [HideInInspector]
        protected TSVector lossyScale = TSVector.one;

        [HideInInspector]
        public TSTransform tsTransform;

        /**
         *  @brief Creates a new {@link TSRigidBody} when there is no one attached to this GameObject.
         **/
        public void Awake() {
            tsTransform = this.GetComponent<TSTransform>();
            tsRigidBody = this.GetComponent<TSRigidBody>();

            if (lossyScale == TSVector.one) {
                lossyScale = TSVector.Abs(transform.localScale.ToTSVector());
            }
        }

        public void Update() {
            if (!Application.isPlaying) {
                lossyScale = TSVector.Abs(transform.lossyScale.ToTSVector());
            }
        }

        private void CreateBody() {
            RigidBody newBody = new RigidBody(Shape);

            if (tsMaterial == null) {
                tsMaterial = GetComponent<TSMaterial>();
            }

            if (tsMaterial != null) {
                newBody.TSFriction = tsMaterial.friction;
                newBody.TSRestitution = tsMaterial.restitution;
            }

            newBody.IsColliderOnly = isTrigger;
            newBody.IsKinematic = tsRigidBody != null && tsRigidBody.isKinematic;

            bool isStatic = tsRigidBody == null || tsRigidBody.isKinematic;

            if (tsRigidBody != null) {
                newBody.AffectedByGravity = tsRigidBody.useGravity;

                if (tsRigidBody.mass <= 0) {
                    tsRigidBody.mass = 1;
                }

                newBody.Mass = tsRigidBody.mass;
                newBody.TSLinearDrag = tsRigidBody.drag;
                newBody.TSAngularDrag = tsRigidBody.angularDrag;
            } else {
                newBody.SetMassProperties();
            }

            if (isStatic) {
                newBody.AffectedByGravity = false;
                newBody.IsStatic = true;
            }

            _body = newBody;
        }

        /**
         *  @brief Initializes Shape and RigidBody and sets initial values to position and orientation based on Unity's transform.
         **/
        public void Initialize() {
            CreateBody();
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

            Vector3 position = _body != null ? _body.Position.ToVector() : (transform.position + ScaledCenter.ToVector());
            Quaternion rotation = _body != null ? _body.Orientation.ToQuaternion() : transform.rotation;

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