using UnityEngine;

namespace TrueSync {

    /**
     * @brief Manages the collision matrix of physics simulation.
     **/
    public class LayerCollisionMatrix {

        /**
         * @brief Returns true if the given layers can collide.
         * 
         * @param layerA Layer of the first object
         * @param layerB Layer of the second object
         **/
        public static bool CollisionEnabled(int layerA, int layerB) {
            TrueSyncConfig tsConfig = TrueSyncManager.Config;
            if (tsConfig == null) {
                return true;
            }

            return !tsConfig.GetIgnoreLayerCollision(layerA, layerB);            
        }

        /**
         * @brief Returns true if the given GameObjects can collide (based on its layers).
         * 
         * @param goA First GameObject
         * @param goB Second GameObject
         **/
        public static bool CollisionEnabled(GameObject goA, GameObject goB) {
            return CollisionEnabled(goA.layer, goB.layer);
        }

    }

}