namespace TrueSync.Physics3D {

    /**
    *  @brief Constraints options that can be applied to a {@link TSRigidBody} .
    **/
    public enum TSRigidBodyConstraints {

        /**
         *  @brief No constraints. 
         **/
        None = 0,

        /**
         *  @brief Freeze position in X axis. 
         **/
        FreezePositionX = 2,

        /**
         *  @brief Freeze position in Y axis. 
         **/
        FreezePositionY = 4,

        /**
         *  @brief Freeze position in Z axis. 
         **/
        FreezePositionZ = 8,

        /**
         *  @brief Freeze position in all axis. 
         **/
        FreezePosition = FreezePositionX | FreezePositionY | FreezePositionZ,

        /**
         *  @brief Freeze rotation in X axis. 
         **/
        FreezeRotationX = 16,

        /**
         *  @brief Freeze rotation in Y axis. 
         **/
        FreezeRotationY = 32,

        /**
         *  @brief Freeze rotation in Z axis. 
         **/
        FreezeRotationZ = 64,

        /**
         *  @brief Freeze rotation in all axis. 
         **/
        FreezeRotation = FreezeRotationX | FreezeRotationY | FreezeRotationZ,

        /**
         *  @brief Freeze position and rotation in all axis. 
         **/
        FreezeAll = FreezePosition | FreezeRotation

    }

}