using System;
using System.Collections.Generic;
using TrueSync.Physics2D;
using FP = TrueSync.FP;

namespace TrueSync.Physics2D
{
    /// <summary>
    /// Put a limit on the linear (translation - the movespeed) and angular (rotation) velocity
    /// of bodies added to this controller.
    /// </summary>
    public class VelocityLimitController : Controller
    {
        public bool LimitAngularVelocity = true;
        public bool LimitLinearVelocity = true;
        private List<Body> _bodies = new List<Body>();
        private FP _maxAngularSqared;
        private FP _maxAngularVelocity;
        private FP _maxLinearSqared;
        private FP _maxLinearVelocity;

        /// <summary>
        /// Initializes a new instance of the <see cref="VelocityLimitController"/> class.
        /// Sets the max linear velocity to Settings.MaxTranslation
        /// Sets the max angular velocity to Settings.MaxRotation
        /// </summary>
        public VelocityLimitController()
            : base(ControllerType.VelocityLimitController)
        {
            MaxLinearVelocity = Settings.MaxTranslation;
            MaxAngularVelocity = Settings.MaxRotation;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="VelocityLimitController"/> class.
        /// Pass in 0 or FP.MaxValue to disable the limit.
        /// maxAngularVelocity = 0 will disable the angular velocity limit.
        /// </summary>
        /// <param name="maxLinearVelocity">The max linear velocity.</param>
        /// <param name="maxAngularVelocity">The max angular velocity.</param>
        public VelocityLimitController(FP maxLinearVelocity, FP maxAngularVelocity)
            : base(ControllerType.VelocityLimitController)
        {
            if (maxLinearVelocity == 0 || maxLinearVelocity == FP.MaxValue)
                LimitLinearVelocity = false;

            if (maxAngularVelocity == 0 || maxAngularVelocity == FP.MaxValue)
                LimitAngularVelocity = false;

            MaxLinearVelocity = maxLinearVelocity;
            MaxAngularVelocity = maxAngularVelocity;
        }

        /// <summary>
        /// Gets or sets the max angular velocity.
        /// </summary>
        /// <value>The max angular velocity.</value>
        public FP MaxAngularVelocity
        {
            get { return _maxAngularVelocity; }
            set
            {
                _maxAngularVelocity = value;
                _maxAngularSqared = _maxAngularVelocity * _maxAngularVelocity;
            }
        }

        /// <summary>
        /// Gets or sets the max linear velocity.
        /// </summary>
        /// <value>The max linear velocity.</value>
        public FP MaxLinearVelocity
        {
            get { return _maxLinearVelocity; }
            set
            {
                _maxLinearVelocity = value;
                _maxLinearSqared = _maxLinearVelocity * _maxLinearVelocity;
            }
        }

        public override void Update(FP dt)
        {
            foreach (Body body in _bodies)
            {
                if (!IsActiveOn(body))
                    continue;

                if (LimitLinearVelocity)
                {
                    //Translation
                    // Check for large velocities.
                    FP translationX = dt * body._linearVelocity.x;
                    FP translationY = dt * body._linearVelocity.y;
                    FP result = translationX * translationX + translationY * translationY;

                    if (result > dt * _maxLinearSqared)
                    {
                        FP sq = FP.Sqrt(result);

                        FP ratio = _maxLinearVelocity / sq;
                        body._linearVelocity.x *= ratio;
                        body._linearVelocity.y *= ratio;
                    }
                }

                if (LimitAngularVelocity)
                {
                    //Rotation
                    FP rotation = dt * body._angularVelocity;
                    if (rotation * rotation > _maxAngularSqared)
                    {
                        FP ratio = _maxAngularVelocity / FP.Abs(rotation);
                        body._angularVelocity *= ratio;
                    }
                }
            }
        }

        public void AddBody(Body body)
        {
            _bodies.Add(body);
        }

        public void RemoveBody(Body body)
        {
            _bodies.Remove(body);
        }
    }
}