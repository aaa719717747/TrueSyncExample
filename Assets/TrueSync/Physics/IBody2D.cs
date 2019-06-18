namespace TrueSync {

    /**
    * @brief Represents an interface to 2D bodies.
    **/
    public interface IBody2D : IBody {

        /**
        * @brief If true the body doesn't move around by collisions.
        **/
        bool TSIsStatic {
            get; set;
        }

        /**
        * @brief Linear drag coeficient.
        **/
        FP TSLinearDrag {
            get; set;
        }

        /**
        * @brief Angular drag coeficient.
        **/
        FP TSAngularDrag {
            get; set;
        }

        /**
         *  @brief Static friction when in contact. 
         **/
        FP TSFriction {
            get; set;
        }

        /**
        * @brief Coeficient of restitution.
        **/
        FP TSRestitution {
            get; set;
        }

        /**
        * @brief Set/get body's position.
        **/
        TSVector2 TSPosition {
            get; set;
        }

        /**
        * @brief Set/get body's orientation.
        **/
        FP TSOrientation {
            get; set;
        }

        /**
        * @brief If true the body is affected by gravity.
        **/
        bool TSAffectedByGravity {
            get; set;
        }

        /**
        * @brief If true the body is managed as kinematic.
        **/
        bool TSIsKinematic {
            get; set;
        }

        /**
        * @brief Set/get body's linear velocity.
        **/
        TSVector2 TSLinearVelocity {
            get; set;
        }

        /**
        * @brief Set/get body's angular velocity. (radians/s).
        **/
        FP TSAngularVelocity {
            get; set;
        }

        /**
        * @brief Applies a force to the body's center.
        **/
        void TSApplyForce(TSVector2 force);

        /**
        * @brief Applies a force to the body at a specific position.
        **/
        void TSApplyForce(TSVector2 force, TSVector2 position);

        /**
        * @brief Applies a impulse to the body's center.
        **/
        void TSApplyImpulse(TSVector2 force);

        /**
        * @brief Applies a impulse to the body at a specific position.
        **/
        void TSApplyImpulse(TSVector2 force, TSVector2 position);

        /**
        * @brief Applies a torque force to the body.
        **/
        void TSApplyTorque(TSVector2 force);

    }

}