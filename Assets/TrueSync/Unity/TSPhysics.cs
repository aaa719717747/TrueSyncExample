using System;

namespace TrueSync
{

    /**
    *  @brief Helpers for 3D physics.
    **/
    public class TSPhysics {

        public static bool Raycast(TSVector rayOrigin, TSVector rayDirection, out TSRaycastHit hit, FP maxDistance, int layerMask = UnityEngine.Physics.DefaultRaycastLayers)
        {
            TSRay ray = new TSRay(rayOrigin, direction:rayDirection);
            hit = PhysicsWorldManager.instance.Raycast(ray, maxDistance, layerMask:layerMask);
            if (hit != null)
            {
                if (hit.distance <= maxDistance)
                    return true;
            }
            return false;
        }
    }

}