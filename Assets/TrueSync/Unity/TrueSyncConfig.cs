using UnityEngine;

namespace TrueSync {

    /**
    * @brief Represents a set of configurations for TrueSync.
    **/
    public class TrueSyncConfig : ScriptableObject {

        private const int COLLISION_LAYERS = 32;
        // 32 layers -> 516 unique intersections
        private const int COLLISION_TOGGLES = 1024;

        /**
         * @brief Synchronization window size.
         **/
        public int syncWindow = 10;

        /**
         * @brief Rollback window size.
         **/
        public int rollbackWindow = 0;

        /**
         * @brief Maximum number of ticks to wait until all other players inputs arrive.
         **/
        public int panicWindow = 100;

        /**
         * @brief Indicates if the 2D physics engine should be enabled.
         **/
        public bool physics2DEnabled = true;

        /**
         * @brief Holds which layers should be ingnored in 2D collisions
         **/
        public bool[] physics2DIgnoreMatrix = new bool[COLLISION_TOGGLES];

        /**
         *  @brief Represents the simulated gravity.
         **/
        public TSVector2 gravity2D = new TSVector2(0, -10);

        /**
         *  @brief If true enables a deeper collision detection system.
         **/
        public bool speculativeContacts2D = false;

        /**
         * @brief Indicates if the 3D physics engine should be enabled.
         **/
        public bool physics3DEnabled = true;

        /**
         * @brief Holds which layers should be ingnored in 3D collisions
         **/
        public bool[] physics3DIgnoreMatrix = new bool[COLLISION_TOGGLES];

        /**
         *  @brief Represents the simulated gravity.
         **/
        public TSVector gravity3D = new TSVector(0, -10, 0);

        /**
         *  @brief If true enables a deeper collision detection system.
         **/
        public bool speculativeContacts3D = false;

        /**
         * @brief When true shows a debug interface with a few information.
         **/
        public bool showStats = false;

        /**
         * @brief Time between each TrueSync's frame.
         **/
        public FP lockedTimeStep = 0.02f;

        public TrueSyncConfig() {
        }

        /**
         * @brief Returns true if the collision between layerA and layerB should be ignored.
         **/
        public bool GetIgnoreLayerCollision(int layerA, int layerB) {
            if (layerB < layerA) {
                int aux = layerA;
                layerA = layerB;
                layerB = aux;
            }

            int matrixIndex = ((COLLISION_LAYERS + COLLISION_LAYERS - layerA + 1) * layerA) / 2 + layerB;

            if (physics2DEnabled) {
                return physics2DIgnoreMatrix[matrixIndex];
            } else if (physics3DEnabled) {
                return physics3DIgnoreMatrix[matrixIndex];
            }

            return false;
        }

    }

}