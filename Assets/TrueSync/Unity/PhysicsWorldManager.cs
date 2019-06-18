using UnityEngine;
using System.Collections.Generic;
using System;
using TrueSync.Physics3D;

namespace TrueSync {

    /**
     *  @brief Manages the 3D physics simulation.
     **/
    public class PhysicsWorldManager : IPhysicsManager {

        public static PhysicsWorldManager instance;

        private World world;

        Dictionary<IBody, GameObject> gameObjectMap;

        Dictionary<RigidBody, Dictionary<RigidBody, TSCollision>> collisionInfo;

        /**
         *  @brief Property access to simulated gravity.
         **/
        public TSVector Gravity {
            get;
            set;
        }

        /**
         *  @brief Property access to speculative contacts.
         **/
        public bool SpeculativeContacts {
            get;
            set;
        }

        public FP LockedTimeStep {
            get;
            set;
        }

        // Use this for initialization
        public void Init() {
            ChecksumExtractor.Init(this);

            gameObjectMap = new Dictionary<IBody, GameObject>();
            collisionInfo = new Dictionary<RigidBody, Dictionary<RigidBody, TSCollision>>();

            CollisionSystemPersistentSAP collisionSystem = new CollisionSystemPersistentSAP();
            collisionSystem.EnableSpeculativeContacts = SpeculativeContacts;

            world = new World(collisionSystem);
            collisionSystem.world = world;

            world.physicsManager = this;
            world.Gravity = Gravity;

            world.Events.BodiesBeginCollide += CollisionEnter;
            world.Events.BodiesStayCollide += CollisionStay;
            world.Events.BodiesEndCollide += CollisionExit;

            world.Events.TriggerBeginCollide += TriggerEnter;
            world.Events.TriggerStayCollide += TriggerStay;
            world.Events.TriggerEndCollide += TriggerExit;

            world.Events.RemovedRigidBody += OnRemovedRigidBody;

            instance = this;

            AddRigidBodies();
        }

        /**
         *  @brief Goes one step further on the physics simulation.
         **/
        public void UpdateStep() {
            world.Step(LockedTimeStep);
        }

        /**
         *  @brief Instance of the current simulated world.
         **/
        public IWorld GetWorld() {
            return world;
        }

        void AddRigidBodies() {
            TSCollider[] bodies = GameObject.FindObjectsOfType<TSCollider>();
            List<TSCollider> sortedBodies = new List<TSCollider>(bodies);
            sortedBodies.Sort(UnityUtils.bodyComparer);

            for (int i = 0; i < sortedBodies.Count; i++) {
                AddBody(sortedBodies[i]);
            }
        }

        /**
         *  @brief Add a new RigidBody to the world.
         *  
         *  @param jRigidBody Instance of a {@link TSRigidBody}.
         **/
        public void AddBody(ICollider iCollider) {
            if (!(iCollider is TSCollider)) {
                Debug.LogError("You have a 2D object but your Physics 2D is disabled.");
                return;
            }

            TSCollider tsCollider = (TSCollider) iCollider;

            if (tsCollider._body != null) {
                //already added
                return;
            }

            TSRigidBody tsRB = tsCollider.GetComponent<TSRigidBody>();
            TSRigidBodyConstraints constraints = tsRB != null ? tsRB.constraints : TSRigidBodyConstraints.None;

            tsCollider.Initialize();
            world.AddBody(tsCollider._body);
            gameObjectMap[tsCollider._body] = tsCollider.gameObject;

            if (tsCollider.gameObject.transform.parent != null && tsCollider.gameObject.transform.parent.GetComponentInParent<TSCollider>() != null) {
                TSCollider parentCollider = tsCollider.gameObject.transform.parent.GetComponentInParent<TSCollider>();
				world.AddConstraint(new ConstraintHierarchy(parentCollider.Body, tsCollider._body, (tsCollider.GetComponent<TSTransform>().position + tsCollider.ScaledCenter) - (parentCollider.GetComponent<TSTransform>().position + parentCollider.ScaledCenter)));
            }

            tsCollider._body.FreezeConstraints = constraints;
        }

        public void RemoveBody(IBody iBody) {
            world.RemoveBody((RigidBody) iBody);
        }

        public void OnRemoveBody(System.Action<IBody> OnRemoveBody){
            world.Events.RemovedRigidBody += delegate (RigidBody rb) {
                OnRemoveBody(rb);
            };
        }

        public bool Raycast(TSVector rayOrigin, TSVector rayDirection, RaycastCallback raycast, out IBody body, out TSVector normal, out FP fraction) {
            RigidBody rb;
            bool result = world.CollisionSystem.Raycast(rayOrigin, rayDirection, raycast, out rb, out normal, out fraction);
            body = rb;

            return result;
        }

        public bool Raycast(TSVector rayOrigin, TSVector rayDirection, RaycastCallback raycast, int layerMask, out IBody body, out TSVector normal, out FP fraction)
        {
            RigidBody rb;
            bool result = world.CollisionSystem.Raycast(rayOrigin, rayDirection, raycast, layerMask, out rb, out normal, out fraction);
            body = rb;
            return result;
        }

        public TSRaycastHit Raycast(TSRay ray, FP maxDistance, RaycastCallback callback = null) {
            IBody hitBody;
            TSVector hitNormal;
            FP hitFraction;

            TSVector origin = ray.origin;
            TSVector direction = ray.direction;

            if (Raycast(origin, direction, callback, out hitBody, out hitNormal, out hitFraction)) {
                if (hitFraction <= maxDistance) {
                    GameObject other = PhysicsManager.instance.GetGameObject(hitBody);
                    TSRigidBody bodyComponent = other.GetComponent<TSRigidBody>();
                    TSCollider colliderComponent = other.GetComponent<TSCollider>();
                    TSTransform transformComponent = other.GetComponent<TSTransform>();
                    return new TSRaycastHit(bodyComponent, colliderComponent, transformComponent, hitNormal, ray.origin, ray.direction, hitFraction);
                }
            } else {
                direction *= maxDistance;
                if (Raycast(origin, direction, callback, out hitBody, out hitNormal, out hitFraction)) {
                    GameObject other = PhysicsManager.instance.GetGameObject(hitBody);
                    TSRigidBody bodyComponent = other.GetComponent<TSRigidBody>();
                    TSCollider colliderComponent = other.GetComponent<TSCollider>();
                    TSTransform transformComponent = other.GetComponent<TSTransform>();
                    return new TSRaycastHit(bodyComponent, colliderComponent, transformComponent, hitNormal, ray.origin, direction, hitFraction);
                }
            }
            return null;
        }

        public TSRaycastHit Raycast(TSRay ray, FP maxDistance, int layerMask, RaycastCallback callback = null)
        {
            IBody hitBody;
            TSVector hitNormal;
            FP hitFraction;

            TSVector origin = ray.origin;
            TSVector direction = ray.direction;

            direction *= maxDistance;
            if (Raycast(origin, direction, callback, layerMask, out hitBody, out hitNormal, out hitFraction))
            {
                GameObject other = PhysicsManager.instance.GetGameObject(hitBody);
                TSRigidBody bodyComponent = other.GetComponent<TSRigidBody>();
                TSCollider colliderComponent = other.GetComponent<TSCollider>();
                TSTransform transformComponent = other.GetComponent<TSTransform>();
                return new TSRaycastHit(bodyComponent, colliderComponent, transformComponent, hitNormal, ray.origin, direction, hitFraction);
            }

            return null;
        }

        private void OnRemovedRigidBody(RigidBody body) {
            GameObject go = gameObjectMap[body];

            if (go != null) {
                GameObject.Destroy(go);
            }
        }

        private void CollisionEnter(Contact c) {
            CollisionDetected(c.body1, c.body2, c, "OnSyncedCollisionEnter");
        }

        private void CollisionStay(Contact c) {
            CollisionDetected(c.body1, c.body2, c, "OnSyncedCollisionStay");
        }

        private void CollisionExit(RigidBody body1, RigidBody body2) {
            CollisionDetected(body1, body2, null, "OnSyncedCollisionExit");
        }

        private void TriggerEnter(Contact c) {
            CollisionDetected(c.body1, c.body2, c, "OnSyncedTriggerEnter");
        }

        private void TriggerStay(Contact c) {
            CollisionDetected(c.body1, c.body2, c, "OnSyncedTriggerStay");
        }

        private void TriggerExit(RigidBody body1, RigidBody body2) {
            CollisionDetected(body1, body2, null, "OnSyncedTriggerExit");
        }

        private void CollisionDetected(RigidBody body1, RigidBody body2, Contact c, string callbackName) {
            if (!gameObjectMap.ContainsKey(body1) || !gameObjectMap.ContainsKey(body2)) {
                return;
            }

            GameObject b1 = gameObjectMap[body1];
            GameObject b2 = gameObjectMap[body2];

            if (b1 == null || b2 == null) {
                return;
            }

            b1.SendMessage(callbackName, GetCollisionInfo(body1, body2, c), SendMessageOptions.DontRequireReceiver);
            b2.SendMessage(callbackName, GetCollisionInfo(body2, body1, c), SendMessageOptions.DontRequireReceiver);

			TrueSyncManager.UpdateCoroutines ();
        }

        private TSCollision GetCollisionInfo(RigidBody body1, RigidBody body2, Contact c) {
            if (!collisionInfo.ContainsKey(body1)) {
                collisionInfo.Add(body1, new Dictionary<RigidBody, TSCollision>());
            }

            Dictionary<RigidBody, TSCollision> collisionInfoBody1 = collisionInfo[body1];

            TSCollision result = null;

            if (collisionInfoBody1.ContainsKey(body2)) {
                result = collisionInfoBody1[body2];
            } else {
                result = new TSCollision();
                collisionInfoBody1.Add(body2, result);
            }


            result.Update(gameObjectMap[body2], c);

            return result;
        }

        /**
         *  @brief Get the GameObject related to a specific RigidBody.
         *  
         *  @param rigidBody Instance of a {@link RigidBody}
         **/
        public GameObject GetGameObject(IBody rigidBody) {
            if (!gameObjectMap.ContainsKey(rigidBody)) {
                return null;
            }

            return gameObjectMap[rigidBody];
        }

        public int GetBodyLayer(IBody body) {
            GameObject go = GetGameObject(body);
            if (go == null) {
                return -1;
            }

            return go.layer;
        }

        /**
         *  @brief Check if the collision between two RigidBodies is enabled.
         *  
         *  @param rigidBody1 First {@link RigidBody}
         *  @param rigidBody2 Second {@link RigidBody}
         **/
        public bool IsCollisionEnabled(IBody rigidBody1, IBody rigidBody2) {
            return LayerCollisionMatrix.CollisionEnabled(gameObjectMap[rigidBody1], gameObjectMap[rigidBody2]);
        }

        public IWorldClone GetWorldClone() {
            return new WorldClone();
        }

    }

}