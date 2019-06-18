using System;
using UnityEngine;

namespace TrueSync {

    /**
     *  @brief Represents each player's behaviour simulated on every machine connected to the game.
     */
    public abstract class TrueSyncBehaviour : MonoBehaviour, ITrueSyncBehaviourGamePlay, ITrueSyncBehaviourCallbacks {

        /**
         * @brief Number of players connected to the game.
         **/
        [HideInInspector]
        public int numberOfPlayers;

        /**
         *  @brief Index of the owner at initial players list.
         */
		public int ownerIndex = -1;

        /**
         *  @brief Basic info about the owner of this behaviour.
         */
        [HideInInspector]
        public TSPlayerInfo owner;

        /**
         *  @brief Basic info about the local player.
         */
        [HideInInspector]
        public TSPlayerInfo localOwner;

        private TSTransform _tsTransform;

        /**
         *  @brief Returns the {@link TSTransform} attached.
         */
        public TSTransform tsTransform {
            get {
                if (_tsTransform == null) {
                    _tsTransform = this.GetComponent<TSTransform>();
                }

                return _tsTransform;
            }
        }

        private TSTransform2D _tsTransform2D;

        /**
         *  @brief Returns the {@link TSTransform2D} attached.
         */
        public TSTransform2D tsTransform2D {
            get {
                if (_tsTransform2D == null) {
                    _tsTransform2D = this.GetComponent<TSTransform2D>();
                }

                return _tsTransform2D;
            }
        }

        private TSRigidBody _tsRigidBody;

        /**
         *  @brief Returns the {@link TSRigidBody} attached.
         */
        public TSRigidBody tsRigidBody {
            get {
                if (_tsRigidBody == null) {
                    _tsRigidBody = this.GetComponent<TSRigidBody>();
                }

                return _tsRigidBody;
            }
        }

        private TSRigidBody2D _tsRigidBody2D;

        /**
         *  @brief Returns the {@link TSRigidBody2D} attached.
         */
        public TSRigidBody2D tsRigidBody2D {
            get {
                if (_tsRigidBody2D == null) {
                    _tsRigidBody2D = this.GetComponent<TSRigidBody2D>();
                }

                return _tsRigidBody2D;
            }
        }

        private TSCollider _tsCollider;

        /**
         *  @brief Returns the {@link TSCollider} attached.
         */
        public TSCollider tsCollider {
            get {
                if (_tsCollider == null) {
                    _tsCollider = this.GetComponent<TSCollider>();
                }

                return _tsCollider;
            }
        }

        private TSCollider2D _tsCollider2D;

        /**
         *  @brief Returns the {@link TSCollider2D} attached.
         */
        public TSCollider2D tsCollider2D {
            get {
                if (_tsCollider2D == null) {
                    _tsCollider2D = this.GetComponent<TSCollider2D>();
                }

                return _tsCollider2D;
            }
        }

        /**
         * @brief It is not called for instances of {@link TrueSyncBehaviour}.
         **/
        public void SetGameInfo(TSPlayerInfo localOwner, int numberOfPlayers) {}

        /**
         * @brief Called once when the object becomes active.
         **/
        public virtual void OnSyncedStart() { }

        /**
         * @brief Called once on instances owned by the local player after the object becomes active.
         **/
        public virtual void OnSyncedStartLocalPlayer() { }

        /**
         * @brief Called when the game has paused.
         **/
        public virtual void OnGamePaused() { }

        /**
         * @brief Called when the game has unpaused.
         **/
        public virtual void OnGameUnPaused() { }

        /**
         * @brief Called when the game has ended.
         **/
        public virtual void OnGameEnded() { }

        /**
         *  @brief Called before {@link #OnSyncedUpdate}.
         *  
         *  Called once every lockstepped frame.
         */
        public virtual void OnPreSyncedUpdate() { }

        /**
         *  @brief Game updates goes here.
         *  
         *  Called once every lockstepped frame.
         */
        public virtual void OnSyncedUpdate() { }

        /**
         *  @brief Get local player data.
         *  
         *  Called once every lockstepped frame.
         */
        public virtual void OnSyncedInput() { }

        /**
         * @brief Callback called when a player get disconnected.
         **/
        public virtual void OnPlayerDisconnection(int playerId) {}

    }

}