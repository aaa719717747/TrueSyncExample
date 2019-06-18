/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

namespace TrueSync.Physics3D {

    #region public class ContactSettings
    public class ContactSettings
    {
        public enum MaterialCoefficientMixingType { TakeMaximum, TakeMinimum, UseAverage }

        internal FP maximumBias = 10;
        internal FP bias = 25 * FP.EN2;
        internal FP minVelocity = FP.EN3;
        internal FP allowedPenetration = FP.EN2;
        internal FP breakThreshold = FP.EN2;

        internal MaterialCoefficientMixingType materialMode = MaterialCoefficientMixingType.UseAverage;

        public FP MaximumBias { get { return maximumBias; } set { maximumBias = value; } }

        public FP BiasFactor { get { return bias; } set { bias = value; } }

        public FP MinimumVelocity { get { return minVelocity; } set { minVelocity = value; } }

        public FP AllowedPenetration { get { return allowedPenetration; } set { allowedPenetration = value; } }

        public FP BreakThreshold { get { return breakThreshold; } set { breakThreshold = value; } }

        public MaterialCoefficientMixingType MaterialCoefficientMixing { get { return materialMode; } set { materialMode = value; } }
    }
    #endregion


    /// <summary>
    /// </summary>
    public class Contact : IConstraint
    {
        public ContactSettings settings;

		public RigidBody body1, body2;

		public TSVector normal, tangent;

		public TSVector realRelPos1, realRelPos2;
		public TSVector relativePos1, relativePos2;
		public TSVector p1, p2;

		public FP accumulatedNormalImpulse = FP.Zero;
		public FP accumulatedTangentImpulse = FP.Zero;

		public FP penetration = FP.Zero;
		public FP initialPen = FP.Zero;

		public FP staticFriction, dynamicFriction, restitution;
		public FP friction = FP.Zero;

		public FP massNormal = FP.Zero, massTangent = FP.Zero;
		public FP restitutionBias = FP.Zero;

		public bool newContact = false;

		public bool treatBody1AsStatic = false;
		public bool treatBody2AsStatic = false;


		public bool body1IsMassPoint;
		public bool body2IsMassPoint;

		public FP lostSpeculativeBounce = FP.Zero;
		public FP speculativeVelocity = FP.Zero;

        /// <summary>
        /// A contact resource pool.
        /// </summary>
        public static readonly ResourcePool<Contact> Pool =
            new ResourcePool<Contact>();

		public FP lastTimeStep = FP.PositiveInfinity;

        #region Properties
        public FP Restitution
        {
            get { return restitution; }
            set { restitution = value; }
        }

        public FP StaticFriction
        {
            get { return staticFriction; }
            set { staticFriction = value; }
        }

        public FP DynamicFriction
        {
            get { return dynamicFriction; }
            set { dynamicFriction = value; }
        }

        /// <summary>
        /// The first body involved in the contact.
        /// </summary>
        public RigidBody Body1 { get { return body1; } }

        /// <summary>
        /// The second body involved in the contact.
        /// </summary>
        public RigidBody Body2 { get { return body2; } }

        /// <summary>
        /// The penetration of the contact.
        /// </summary>
        public FP Penetration { get { return penetration; } }

        /// <summary>
        /// The collision position in world space of body1.
        /// </summary>
        public TSVector Position1 { get { return p1; } }

        /// <summary>
        /// The collision position in world space of body2.
        /// </summary>
        public TSVector Position2 { get { return p2; } }

        /// <summary>
        /// The contact tangent.
        /// </summary>
        public TSVector Tangent { get { return tangent; } }

        /// <summary>
        /// The contact normal.
        /// </summary>
        public TSVector Normal { get { return normal; } }
        #endregion

        /// <summary>
        /// Calculates relative velocity of body contact points on the bodies.
        /// </summary>
        /// <param name="relVel">The relative velocity of body contact points on the bodies.</param>
        public TSVector CalculateRelativeVelocity()
        {
            FP x, y, z;

            x = (body2.angularVelocity.y * relativePos2.z) - (body2.angularVelocity.z * relativePos2.y) + body2.linearVelocity.x;
            y = (body2.angularVelocity.z * relativePos2.x) - (body2.angularVelocity.x * relativePos2.z) + body2.linearVelocity.y;
            z = (body2.angularVelocity.x * relativePos2.y) - (body2.angularVelocity.y * relativePos2.x) + body2.linearVelocity.z;

            TSVector relVel;
            relVel.x = x - (body1.angularVelocity.y * relativePos1.z) + (body1.angularVelocity.z * relativePos1.y) - body1.linearVelocity.x;
            relVel.y = y - (body1.angularVelocity.z * relativePos1.x) + (body1.angularVelocity.x * relativePos1.z) - body1.linearVelocity.y;
            relVel.z = z - (body1.angularVelocity.x * relativePos1.y) + (body1.angularVelocity.y * relativePos1.x) - body1.linearVelocity.z;

            return relVel;
        }

        /// <summary>
        /// Solves the contact iteratively.
        /// </summary>
        public void Iterate()
        {
            //body1.linearVelocity = JVector.Zero;
            //body2.linearVelocity = JVector.Zero;
            //return;

            if (treatBody1AsStatic && treatBody2AsStatic) return;

            FP dvx, dvy, dvz;

            dvx = body2.linearVelocity.x - body1.linearVelocity.x;
            dvy = body2.linearVelocity.y - body1.linearVelocity.y;
            dvz = body2.linearVelocity.z - body1.linearVelocity.z;

            if (!body1IsMassPoint)
            {
                dvx = dvx - (body1.angularVelocity.y * relativePos1.z) + (body1.angularVelocity.z * relativePos1.y);
                dvy = dvy - (body1.angularVelocity.z * relativePos1.x) + (body1.angularVelocity.x * relativePos1.z);
                dvz = dvz - (body1.angularVelocity.x * relativePos1.y) + (body1.angularVelocity.y * relativePos1.x);
            }

            if (!body2IsMassPoint)
            {
                dvx = dvx + (body2.angularVelocity.y * relativePos2.z) - (body2.angularVelocity.z * relativePos2.y);
                dvy = dvy + (body2.angularVelocity.z * relativePos2.x) - (body2.angularVelocity.x * relativePos2.z);
                dvz = dvz + (body2.angularVelocity.x * relativePos2.y) - (body2.angularVelocity.y * relativePos2.x);
            }

            // this gets us some performance
            if (dvx * dvx + dvy * dvy + dvz * dvz < settings.minVelocity * settings.minVelocity)
            { return; }

            FP vn = normal.x * dvx + normal.y * dvy + normal.z * dvz;
            FP normalImpulse = massNormal * (-vn + restitutionBias + speculativeVelocity);

            FP oldNormalImpulse = accumulatedNormalImpulse;
            accumulatedNormalImpulse = oldNormalImpulse + normalImpulse;
            if (accumulatedNormalImpulse < FP.Zero) accumulatedNormalImpulse = FP.Zero;
            normalImpulse = accumulatedNormalImpulse - oldNormalImpulse;

            FP vt = dvx * tangent.x + dvy * tangent.y + dvz * tangent.z;
            FP maxTangentImpulse = friction * accumulatedNormalImpulse;
            FP tangentImpulse = massTangent * (-vt);

            FP oldTangentImpulse = accumulatedTangentImpulse;
            accumulatedTangentImpulse = oldTangentImpulse + tangentImpulse;
            if (accumulatedTangentImpulse < -maxTangentImpulse) accumulatedTangentImpulse = -maxTangentImpulse;
            else if (accumulatedTangentImpulse > maxTangentImpulse) accumulatedTangentImpulse = maxTangentImpulse;

            tangentImpulse = accumulatedTangentImpulse - oldTangentImpulse;

            // Apply contact impulse
            TSVector impulse = normal * normalImpulse + tangent * tangentImpulse;
            ApplyImpulse(ref impulse);

        }

        public FP AppliedNormalImpulse { get { return accumulatedNormalImpulse; } }
        public FP AppliedTangentImpulse { get { return accumulatedTangentImpulse; } }

        /// <summary>
        /// The points in wolrd space gets recalculated by transforming the
        /// local coordinates. Also new penetration depth is estimated.
        /// </summary>
        public void UpdatePosition()
        {
            if (body1IsMassPoint)
            {
                TSVector.Add(ref realRelPos1, ref body1.position, out p1);
            }
            else
            {
                TSVector.Transform(ref realRelPos1, ref body1.orientation, out p1);
                TSVector.Add(ref p1, ref body1.position, out p1);
            }

            if (body2IsMassPoint)
            {
                TSVector.Add(ref realRelPos2, ref body2.position, out p2);
            }
            else
            {
                TSVector.Transform(ref realRelPos2, ref body2.orientation, out p2);
                TSVector.Add(ref p2, ref body2.position, out p2);
            }


            TSVector dist; TSVector.Subtract(ref p1, ref p2, out dist);
            penetration = TSVector.Dot(ref dist, ref normal);
        }

        /// <summary>
        /// An impulse is applied an both contact points.
        /// </summary>
        /// <param name="impulse">The impulse to apply.</param>
        public void ApplyImpulse(ref TSVector impulse)
        {
            #region INLINE - HighFrequency
            //JVector temp;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.x -= (impulse.x * body1.inverseMass);
                body1.linearVelocity.y -= (impulse.y * body1.inverseMass);
                body1.linearVelocity.z -= (impulse.z * body1.inverseMass);

                FP num0, num1, num2;
                num0 = relativePos1.y * impulse.z - relativePos1.z * impulse.y;
                num1 = relativePos1.z * impulse.x - relativePos1.x * impulse.z;
                num2 = relativePos1.x * impulse.y - relativePos1.y * impulse.x;

                FP num3 =
                    (((num0 * body1.invInertiaWorld.M11) +
                    (num1 * body1.invInertiaWorld.M21)) +
                    (num2 * body1.invInertiaWorld.M31));
                FP num4 =
                    (((num0 * body1.invInertiaWorld.M12) +
                    (num1 * body1.invInertiaWorld.M22)) +
                    (num2 * body1.invInertiaWorld.M32));
                FP num5 =
                    (((num0 * body1.invInertiaWorld.M13) +
                    (num1 * body1.invInertiaWorld.M23)) +
                    (num2 * body1.invInertiaWorld.M33));

                body1.angularVelocity.x -= num3;
                body1.angularVelocity.y -= num4;
                body1.angularVelocity.z -= num5;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.x += (impulse.x * body2.inverseMass);
                body2.linearVelocity.y += (impulse.y * body2.inverseMass);
                body2.linearVelocity.z += (impulse.z * body2.inverseMass);

                FP num0, num1, num2;
                num0 = relativePos2.y * impulse.z - relativePos2.z * impulse.y;
                num1 = relativePos2.z * impulse.x - relativePos2.x * impulse.z;
                num2 = relativePos2.x * impulse.y - relativePos2.y * impulse.x;

                FP num3 =
                    (((num0 * body2.invInertiaWorld.M11) +
                    (num1 * body2.invInertiaWorld.M21)) +
                    (num2 * body2.invInertiaWorld.M31));
                FP num4 =
                    (((num0 * body2.invInertiaWorld.M12) +
                    (num1 * body2.invInertiaWorld.M22)) +
                    (num2 * body2.invInertiaWorld.M32));
                FP num5 =
                    (((num0 * body2.invInertiaWorld.M13) +
                    (num1 * body2.invInertiaWorld.M23)) +
                    (num2 * body2.invInertiaWorld.M33));

                body2.angularVelocity.x += num3;
                body2.angularVelocity.y += num4;
                body2.angularVelocity.z += num5;
            }


            #endregion
        }

        public void ApplyImpulse(TSVector impulse)
        {
            #region INLINE - HighFrequency
            //JVector temp;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.x -= (impulse.x * body1.inverseMass);
                body1.linearVelocity.y -= (impulse.y * body1.inverseMass);
                body1.linearVelocity.z -= (impulse.z * body1.inverseMass);

                FP num0, num1, num2;
                num0 = relativePos1.y * impulse.z - relativePos1.z * impulse.y;
                num1 = relativePos1.z * impulse.x - relativePos1.x * impulse.z;
                num2 = relativePos1.x * impulse.y - relativePos1.y * impulse.x;

                FP num3 =
                    (((num0 * body1.invInertiaWorld.M11) +
                    (num1 * body1.invInertiaWorld.M21)) +
                    (num2 * body1.invInertiaWorld.M31));
                FP num4 =
                    (((num0 * body1.invInertiaWorld.M12) +
                    (num1 * body1.invInertiaWorld.M22)) +
                    (num2 * body1.invInertiaWorld.M32));
                FP num5 =
                    (((num0 * body1.invInertiaWorld.M13) +
                    (num1 * body1.invInertiaWorld.M23)) +
                    (num2 * body1.invInertiaWorld.M33));

                body1.angularVelocity.x -= num3;
                body1.angularVelocity.y -= num4;
                body1.angularVelocity.z -= num5;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.x += (impulse.x * body2.inverseMass);
                body2.linearVelocity.y += (impulse.y * body2.inverseMass);
                body2.linearVelocity.z += (impulse.z * body2.inverseMass);

                FP num0, num1, num2;
                num0 = relativePos2.y * impulse.z - relativePos2.z * impulse.y;
                num1 = relativePos2.z * impulse.x - relativePos2.x * impulse.z;
                num2 = relativePos2.x * impulse.y - relativePos2.y * impulse.x;

                FP num3 =
                    (((num0 * body2.invInertiaWorld.M11) +
                    (num1 * body2.invInertiaWorld.M21)) +
                    (num2 * body2.invInertiaWorld.M31));
                FP num4 =
                    (((num0 * body2.invInertiaWorld.M12) +
                    (num1 * body2.invInertiaWorld.M22)) +
                    (num2 * body2.invInertiaWorld.M32));
                FP num5 =
                    (((num0 * body2.invInertiaWorld.M13) +
                    (num1 * body2.invInertiaWorld.M23)) +
                    (num2 * body2.invInertiaWorld.M33));

                body2.angularVelocity.x += num3;
                body2.angularVelocity.y += num4;
                body2.angularVelocity.z += num5;
            }


            #endregion
        }

        /// <summary>
        /// PrepareForIteration has to be called before <see cref="Iterate"/>.
        /// </summary>
        /// <param name="timestep">The timestep of the simulation.</param>
        public void PrepareForIteration(FP timestep)
        {
            TSVector dv = CalculateRelativeVelocity();
            FP kNormal = FP.Zero;

            TSVector rantra = TSVector.zero;
            if (!treatBody1AsStatic)
            {
                kNormal += body1.inverseMass;

                if (!body1IsMassPoint)
                {
                    TSVector.Cross(ref relativePos1, ref normal, out rantra);
                    TSVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    TSVector.Cross(ref rantra, ref relativePos1, out rantra);
                }
            }

            TSVector rbntrb = TSVector.zero;
            if (!treatBody2AsStatic)
            {
                kNormal += body2.inverseMass;

                if (!body2IsMassPoint)
                {
                    TSVector.Cross(ref relativePos2, ref normal, out rbntrb);
                    TSVector.Transform(ref rbntrb, ref body2.invInertiaWorld, out rbntrb);
                    TSVector.Cross(ref rbntrb, ref relativePos2, out rbntrb);
                }
            }

            if (!treatBody1AsStatic)
                kNormal += TSVector.Dot(ref rantra, ref normal);

            if (!treatBody2AsStatic)
                kNormal += TSVector.Dot(ref rbntrb, ref normal);

            massNormal = FP.One / kNormal;

            tangent = dv - TSVector.Dot(dv, normal) * normal;
            tangent.Normalize();

            FP kTangent = FP.Zero;

            if (treatBody1AsStatic) rantra.MakeZero();
            else
            {
                kTangent += body1.inverseMass;
  
                if (!body1IsMassPoint)
                {
                    TSVector.Cross(ref relativePos1, ref normal, out rantra);
                    TSVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    TSVector.Cross(ref rantra, ref relativePos1, out rantra);
                }

            }

            if (treatBody2AsStatic)
                rbntrb.MakeZero();
            else
            {
                kTangent += body2.inverseMass;

                if (!body2IsMassPoint)
                {
                    TSVector.Cross(ref relativePos2, ref tangent, out rbntrb);
                    TSVector.Transform(ref rbntrb, ref body2.invInertiaWorld, out rbntrb);
                    TSVector.Cross(ref rbntrb, ref relativePos2, out rbntrb);
                }
            }

            if (!treatBody1AsStatic)
                kTangent += TSVector.Dot(ref rantra, ref tangent);
            if (!treatBody2AsStatic)
                kTangent += TSVector.Dot(ref rbntrb, ref tangent);

            massTangent = FP.One / kTangent;

            restitutionBias = lostSpeculativeBounce;

            speculativeVelocity = FP.Zero;

            FP relNormalVel = TSVector.Dot(ref normal, ref dv);

            if (Penetration > settings.allowedPenetration)
            {
                restitutionBias = settings.bias * (FP.One / timestep) * TSMath.Max(FP.Zero, Penetration - settings.allowedPenetration);
                restitutionBias = TSMath.Clamp(restitutionBias, FP.Zero, settings.maximumBias);
              //  body1IsMassPoint = body2IsMassPoint = false;
            }
      

            FP timeStepRatio = timestep / lastTimeStep;
            accumulatedNormalImpulse *= timeStepRatio;
            accumulatedTangentImpulse *= timeStepRatio;

            {
                // Static/Dynamic friction
                FP relTangentVel = -TSVector.Dot(ref tangent, ref dv);
                FP tangentImpulse = massTangent * relTangentVel;
                FP maxTangentImpulse = -staticFriction * accumulatedNormalImpulse;

                if (tangentImpulse < maxTangentImpulse)
                    friction = dynamicFriction;
                else
                    friction = staticFriction;
            }

            TSVector impulse;

            // Simultaneos solving and restitution is simply not possible
            // so fake it a bit by just applying restitution impulse when there
            // is a new contact.
            if (relNormalVel < -FP.One && newContact)
            {
                restitutionBias = TSMath.Max(-restitution * relNormalVel, restitutionBias);
            }

            // Speculative Contacts!
            // if the penetration is negative (which means the bodies are not already in contact, but they will
            // be in the future) we store the current bounce bias in the variable 'lostSpeculativeBounce'
            // and apply it the next frame, when the speculative contact was already solved.
            if (penetration < -settings.allowedPenetration)
            {
                speculativeVelocity = penetration / timestep;

                lostSpeculativeBounce = restitutionBias;
                restitutionBias = FP.Zero;
            }
            else
            {
                lostSpeculativeBounce = FP.Zero;
            }

            impulse = normal * accumulatedNormalImpulse + tangent * accumulatedTangentImpulse;
            ApplyImpulse(ref impulse);

            lastTimeStep = timestep;

            newContact = false;
        }

        public void TreatBodyAsStatic(RigidBodyIndex index)
        {
            if (index == RigidBodyIndex.RigidBody1) treatBody1AsStatic = true;
            else treatBody2AsStatic = true;
        }


        /// <summary>
        /// Initializes a contact.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <param name="point1">The collision point in worldspace</param>
        /// <param name="point2">The collision point in worldspace</param>
        /// <param name="n">The normal pointing to body2.</param>
        /// <param name="penetration">The estimated penetration depth.</param>
        public void Initialize(RigidBody body1, RigidBody body2, ref TSVector point1, ref TSVector point2, ref TSVector n,
            FP penetration, bool newContact, ContactSettings settings)
        {
            this.body1 = body1;  this.body2 = body2;
            this.normal = n; normal.Normalize();
            this.p1 = point1; this.p2 = point2;

            this.newContact = newContact;

            TSVector.Subtract(ref p1, ref body1.position, out relativePos1);
            TSVector.Subtract(ref p2, ref body2.position, out relativePos2);
            TSVector.Transform(ref relativePos1, ref body1.invOrientation, out realRelPos1);
            TSVector.Transform(ref relativePos2, ref body2.invOrientation, out realRelPos2);

            this.initialPen = penetration;
            this.penetration = penetration;

            body1IsMassPoint = body1.isParticle;
            body2IsMassPoint = body2.isParticle;

            // Material Properties
            if (newContact)
            {
                treatBody1AsStatic = body1.isStatic;
                treatBody2AsStatic = body2.isStatic;

                accumulatedNormalImpulse = FP.Zero;
                accumulatedTangentImpulse = FP.Zero;

                lostSpeculativeBounce = FP.Zero;

                switch (settings.MaterialCoefficientMixing)
                {
                    case ContactSettings.MaterialCoefficientMixingType.TakeMaximum:
                        staticFriction = TSMath.Max(body1.staticFriction, body2.staticFriction);
                        dynamicFriction = TSMath.Max(body1.staticFriction, body2.staticFriction);
                        restitution = TSMath.Max(body1.restitution, body2.restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.TakeMinimum:
                        staticFriction = TSMath.Min(body1.staticFriction, body2.staticFriction);
                        dynamicFriction = TSMath.Min(body1.staticFriction, body2.staticFriction);
                        restitution = TSMath.Min(body1.restitution, body2.restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.UseAverage:
                        staticFriction = (body1.staticFriction + body2.staticFriction) * FP.Half;
                        dynamicFriction = (body1.staticFriction + body2.staticFriction) * FP.Half;
                        restitution = (body1.restitution + body2.restitution) * FP.Half;
                        break;
                }

            }

            this.settings = settings;
        }
    }
}