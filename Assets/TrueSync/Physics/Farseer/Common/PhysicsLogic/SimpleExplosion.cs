using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;

namespace TrueSync.Physics2D
{
    /// <summary>
    /// Creates a simple explosion that ignores other bodies hiding behind static bodies.
    /// </summary>
    public sealed class SimpleExplosion : PhysicsLogic
    {
        public SimpleExplosion(World world)
            : base(world, PhysicsLogicType.Explosion)
        {
            Power = 1; //linear
        }

        /// <summary>
        /// This is the power used in the power function. A value of 1 means the force
        /// applied to bodies in the explosion is linear. A value of 2 means it is exponential.
        /// </summary>
        public FP Power { get; set; }

        /// <summary>
        /// Activate the explosion at the specified position.
        /// </summary>
        /// <param name="pos">The position (center) of the explosion.</param>
        /// <param name="radius">The radius of the explosion.</param>
        /// <param name="force">The force applied</param>
        /// <param name="maxForce">A maximum amount of force. When force gets over this value, it will be equal to maxForce</param>
        /// <returns>A list of bodies and the amount of force that was applied to them.</returns>
        // TS - public Dictionary<Body, Vector2> Activate(Vector2 pos, FP radius, FP force, FP maxForce = FP.MaxValue)
        public Dictionary<Body, TSVector2> Activate(TSVector2 pos, FP radius, FP force, FP maxForce)
        {
            HashSet<Body> affectedBodies = new HashSet<Body>();

            AABB aabb;
            aabb.LowerBound = pos - new TSVector2(radius);
            aabb.UpperBound = pos + new TSVector2(radius);

            // Query the world for bodies within the radius.
            World.QueryAABB(fixture =>
            {
                if (TSVector2.Distance(fixture.Body.Position, pos) <= radius)
                {
                    if (!affectedBodies.Contains(fixture.Body))
                        affectedBodies.Add(fixture.Body);
                }

                return true;
            }, ref aabb);

            return ApplyImpulse(pos, radius, force, maxForce, affectedBodies);
        }

        private Dictionary<Body, TSVector2> ApplyImpulse(TSVector2 pos, FP radius, FP force, FP maxForce, HashSet<Body> overlappingBodies)
        {
            Dictionary<Body, TSVector2> forces = new Dictionary<Body, TSVector2>(overlappingBodies.Count);

            foreach (Body overlappingBody in overlappingBodies)
            {
                if (IsActiveOn(overlappingBody))
                {
                    FP distance = TSVector2.Distance(pos, overlappingBody.Position);
                    FP forcePercent = GetPercent(distance, radius);

                    TSVector2 forceVector = pos - overlappingBody.Position;
                    forceVector *= 1f / FP.Sqrt(forceVector.x * forceVector.x + forceVector.y * forceVector.y);
                    forceVector *= TSMath.Min(force * forcePercent, maxForce);
                    forceVector *= -1;

                    overlappingBody.ApplyLinearImpulse(forceVector);
                    forces.Add(overlappingBody, forceVector);
                }
            }

            return forces;
        }

        private FP GetPercent(FP distance, FP radius)
        {
            //(1-(distance/radius))^power-1
            // TODO - PORT
            // FP percent = (FP)Math.Pow(1 - ((distance - radius) / radius), Power) - 1;
            FP percent = (FP)Math.Pow((1 - ((distance - radius) / radius)).AsFloat(), Power.AsFloat()) - 1;

            if (FP.IsNaN(percent))
                return 0f;

            return TSMath.Clamp(percent, 0f, 1f);
        }
    }
}