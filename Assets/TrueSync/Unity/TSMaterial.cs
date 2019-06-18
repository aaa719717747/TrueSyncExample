using UnityEngine;

namespace TrueSync {

    /**
     *  @brief Simulates physical properties of a body.
     **/
    [AddComponentMenu("TrueSync/Physics/TSMaterial", 22)]
    public class TSMaterial : MonoBehaviour {

        /**
         *  @brief Static friction when in contact. 
         **/
         [Header("接触时的静摩擦")]
        public FP friction = 0.25f;

        /**
         *  @brief Coeficient of restitution. 
         **/
        [Header("恢复系数")]
        public FP restitution = 0;

    }

}