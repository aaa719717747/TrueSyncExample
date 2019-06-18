using UnityEngine;
using System.Collections.Generic;
using System;
using TrueSync.Physics2D;

namespace TrueSync {

    /**
     *  @brief Manages the 2D physics simulation.
     **/
    public class Physics2DWorldManager : IPhysicsManager {

        /**
         *  @brief Public access to a manager instance.
         **/
        public static Physics2DWorldManager instance;
        
        private Physics2D.World world;

        Dictionary<IBody, GameObject> gameObjectMap;

        Dictionary<Physics2D.Body, Dictionary<Physics2D.Body, TSCollision2D>> collisionInfo;

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

            Settings.ContinuousPhysics = SpeculativeContacts;

            world = new TrueSync.Physics2D.World(new TSVector2(Gravity.x, Gravity.y));
            ContactManager.physicsManager = this;

            world.BodyRemoved += OnRemovedRigidBody;
            world.ContactManager.BeginContact += CollisionEnter;
            world.ContactManager.StayContact += CollisionStay;
            world.ContactManager.EndContact += CollisionExit;

            gameObjectMap = new Dictionary<IBody, GameObject>();
            collisionInfo = new Dictionary<Physics2D.Body, Dictionary<Physics2D.Body, TSCollision2D>>();

            instance = this;
            AddRigidBodies();
        }

        /**
         *  @brief Goes one step further on the physics simulation.
         **/
        public void UpdateStep() {
            world.Step(LockedTimeStep);
        }

        public IWorld GetWorld() {
            return world;
        }

        void AddRigidBodies() {
            TSCollider2D[] bodies = GameObject.FindObjectsOfType<TSCollider2D>();
            List<TSCollider2D> sortedBodies = new List<TSCollider2D>(bodies);
            sortedBodies.Sort(UnityUtils.body2DComparer);

            for (int i = 0; i < sortedBodies.Count; i++) {
                AddBody(sortedBodies[i]);
            }
        }

        public void AddBody(ICollider iCollider) {
            if (!(iCollider is TSCollider2D)) {
                Debug.LogError("You have a 3D object but your Physics 3D is disabled.");
                return;
            }

            TSCollider2D tsCollider = (TSCollider2D) iCollider;

            if (tsCollider._body != null) {
                //already added
                return;
            }

            tsCollider.Initialize(world);
            gameObjectMap[tsCollider._body] = tsCollider.gameObject;

            if (tsCollider.gameObject.transform.parent != null && tsCollider.gameObject.transform.parent.GetComponentInParent<TSCollider2D>() != null) {
                TSCollider2D parentCollider = tsCollider.gameObject.transform.parent.GetComponentInParent<TSCollider2D>();
                Physics2D.Body childBody = tsCollider._body;

                childBody.bodyConstraints.Add(new ConstraintHierarchy2D(((Physics2D.Body)parentCollider.Body), tsCollider._body, (tsCollider.GetComponent<TSTransform2D>().position + tsCollider.ScaledCenter) - (parentCollider.GetComponent<TSTransform2D>().position + parentCollider.ScaledCenter)));
            }

            world.ProcessAddedBodies();
        }

        internal string GetChecksum(bool plain) {
            return ChecksumExtractor.GetEncodedChecksum();
        }

        public void RemoveBody(IBody iBody) {
            world.RemoveBody((TrueSync.Physics2D.Body) iBody);
            world.ProcessRemovedBodies();
        }

        public void OnRemoveBody(Action<IBody> OnRemoveBody) {
            world.BodyRemoved += delegate (TrueSync.Physics2D.Body body) {
                OnRemoveBody(body);
            };
        }

        private void OnRemovedRigidBody(TrueSync.Physics2D.Body body) {
            GameObject go = gameObjectMap[body];

            if (go != null) {
                GameObject.Destroy(go);
            }
        }

        private bool CollisionEnter(TrueSync.Physics2D.Contact contact) {
            if (contact.FixtureA.IsSensor || contact.FixtureB.IsSensor) {
                TriggerEnter(contact);

                return true;
            }

            CollisionDetected(contact.FixtureA.Body, contact.FixtureB.Body, contact, "OnSyncedCollisionEnter");
            return true;
        }

        private void CollisionStay(TrueSync.Physics2D.Contact contact) {
            if (contact.FixtureA.IsSensor || contact.FixtureB.IsSensor) {
                TriggerStay(contact);
                return;
            }

            CollisionDetected(contact.FixtureA.Body, contact.FixtureB.Body, contact, "OnSyncedCollisionStay");
        }

        private void CollisionExit(TrueSync.Physics2D.Contact contact) {
            if (contact.FixtureA.IsSensor || contact.FixtureB.IsSensor) {
                TriggerExit(contact);
                return;
            }

            CollisionDetected(contact.FixtureA.Body, contact.FixtureB.Body, null, "OnSyncedCollisionExit");
        }

        private void TriggerEnter(TrueSync.Physics2D.Contact contact) {
            CollisionDetected(contact.FixtureA.Body, contact.FixtureB.Body, contact, "OnSyncedTriggerEnter");
        }

        private void TriggerStay(TrueSync.Physics2D.Contact contact) {
            CollisionDetected(contact.FixtureA.Body, contact.FixtureB.Body, contact, "OnSyncedTriggerStay");
        }

        private void TriggerExit(TrueSync.Physics2D.Contact contact) {
            CollisionDetected(contact.FixtureA.Body, contact.FixtureB.Body, null, "OnSyncedTriggerExit");
        }

        private void CollisionDetected(Physics2D.Body body1, Physics2D.Body body2, TrueSync.Physics2D.Contact contact, string callbackName) {
            if (!gameObjectMap.ContainsKey(body1) || !gameObjectMap.ContainsKey(body2)) {
                return;
            }

            GameObject b1 = gameObjectMap[body1];
            GameObject b2 = gameObjectMap[body2];

            if (b1 == null || b2 == null) {
                return;
            }

            b1.SendMessage(callbackName, GetCollisionInfo(body1, body2, contact), SendMessageOptions.DontRequireReceiver);
            b2.SendMessage(callbackName, GetCollisionInfo(body2, body1, contact), SendMessageOptions.DontRequireReceiver);

			TrueSyncManager.UpdateCoroutines ();
        }

        private TSCollision2D GetCollisionInfo(Physics2D.Body body1, Physics2D.Body body2, TrueSync.Physics2D.Contact c) {
            if (!collisionInfo.ContainsKey(body1)) {
                collisionInfo.Add(body1, new Dictionary<Physics2D.Body, TSCollision2D>());
            }

            Dictionary<Physics2D.Body, TSCollision2D> collisionInfoBody1 = collisionInfo[body1];

            TSCollision2D result = null;

            if (collisionInfoBody1.ContainsKey(body2)) {
                result = collisionInfoBody1[body2];
            } else {
                result = new TSCollision2D();
                collisionInfoBody1.Add(body2, result);
            }

            result.Update(gameObjectMap[body2], c);

            return result;
        }

        public GameObject GetGameObject(IBody body) {
            if (!gameObjectMap.ContainsKey(body)) {
                return null;
            }

            return gameObjectMap[body];
        }

        public int GetBodyLayer(IBody body) {
            GameObject go = GetGameObject(body);
            if (go == null) {
                return -1;
            }

            return go.layer;
        }

        public TSRaycastHit2D Raycast(TSVector2 origin, TSVector2 direction, FP distance) {
            TSRaycastHit2D result = null;

            Func<TrueSync.Physics2D.Fixture, TSVector2, TSVector2, FP, FP> callback = delegate (TrueSync.Physics2D.Fixture fixture, TSVector2 point, TSVector2 normal, FP fraction) {
                result = new TSRaycastHit2D(gameObjectMap[fixture.Body].GetComponent<TSCollider2D>());
                return 0;
            };

            world.RayCast(callback, origin, origin + direction * distance);

            return result;
        }

        public TSRaycastHit2D[] RaycastAll(TSVector2 origin, TSVector2 direction, FP distance) {
            List<TSRaycastHit2D> result = new List<TSRaycastHit2D>();

            Func<TrueSync.Physics2D.Fixture, TSVector2, TSVector2, FP, FP> callback = delegate (TrueSync.Physics2D.Fixture fixture, TSVector2 point, TSVector2 normal, FP fraction) {
                result.Add(new TSRaycastHit2D(gameObjectMap[fixture.Body].GetComponent<TSCollider2D>()));
                return -1;
            };

            world.RayCast(callback, origin, origin + direction * distance);

            if (result.Count == 0) {
                return null;
            }

            return result.ToArray();
        }

        public bool IsCollisionEnabled(IBody rigidBody1, IBody rigidBody2) {
            if (gameObjectMap.ContainsKey(rigidBody1) && gameObjectMap.ContainsKey(rigidBody2)) {
                return LayerCollisionMatrix.CollisionEnabled(gameObjectMap[rigidBody1], gameObjectMap[rigidBody2]);
            }

            return true;
        }

        public IWorldClone GetWorldClone() {
            return new WorldClone2D();
        }

    }

}