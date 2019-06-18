namespace TrueSync.Physics3D {

    /**
     *  @brief Represents physical properties of a {@link RigidBody}. 
     **/
    public class BodyMaterial {

        internal FP kineticFriction = FP.One / 4;
        internal FP staticFriction = FP.One / 2;
        internal FP restitution = FP.Zero;

        public BodyMaterial() { }

        /**
         *  @brief Elastic restituion. 
         **/
        public FP Restitution {
            get { return restitution; }
            set { restitution = value; }
        }

        /**
         *  @brief Static friction. 
         **/
        public FP StaticFriction {
            get { return staticFriction; }
            set { staticFriction = value; }
        }

        /**
         *  @brief Kinectic friction. 
         **/
        public FP KineticFriction {
            get { return kineticFriction; }
            set { kineticFriction = value; }
        }

    }

}