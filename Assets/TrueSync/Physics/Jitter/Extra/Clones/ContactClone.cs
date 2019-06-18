namespace TrueSync.Physics3D {

    public class ContactClone {
		
        public ContactSettings settings;

        public RigidBody body1, body2;

        public TSVector normal, tangent;

		public TSVector realRelPos1, realRelPos2;
		public TSVector relativePos1, relativePos2;
		public TSVector p1, p2;

		public FP accumulatedNormalImpulse;
		public FP accumulatedTangentImpulse;

		public FP penetration;
		public FP initialPen;

		public FP staticFriction, dynamicFriction, restitution;
		public FP friction;

		public FP massNormal, massTangent;
		public FP restitutionBias;

		public bool newContact;

		public bool treatBody1AsStatic;
		public bool treatBody2AsStatic;

		public bool body1IsMassPoint; 
		public bool body2IsMassPoint;

		public FP lostSpeculativeBounce;
		public FP speculativeVelocity;

		public FP lastTimeStep;

		public void Clone(Contact contact) {
			settings = contact.settings;
			body1 = contact.body1;
			body2 = contact.body2;
			normal = contact.normal;
			tangent = contact.tangent;
			realRelPos1 = contact.realRelPos1;
			realRelPos2 = contact.realRelPos2;
			relativePos1 = contact.relativePos1;
			relativePos2 = contact.relativePos2;
			p1 = contact.p1;
			p2 = contact.p2;
			accumulatedNormalImpulse = contact.accumulatedNormalImpulse;
			accumulatedTangentImpulse = contact.accumulatedTangentImpulse;
			penetration = contact.penetration;
			initialPen = contact.initialPen;
			staticFriction = contact.staticFriction;
			dynamicFriction = contact.dynamicFriction;
			restitution = contact.restitution;
			friction = contact.friction;
			massNormal = contact.massNormal;
			massTangent = contact.massTangent;
			restitutionBias = contact.restitutionBias;
			newContact = contact.newContact;
			treatBody1AsStatic = contact.treatBody1AsStatic;
			treatBody2AsStatic = contact.treatBody2AsStatic;
			body1IsMassPoint = contact.body1IsMassPoint;
			body2IsMassPoint = contact.body2IsMassPoint;
			lostSpeculativeBounce = contact.lostSpeculativeBounce;
			speculativeVelocity = contact.speculativeVelocity;
			lastTimeStep = contact.lastTimeStep;
		}

		public void Restore(Contact contact) {
			contact.settings = settings;
			contact.body1 = body1;
			contact.body2 = body2;
			contact.normal = normal;
			contact.tangent = tangent;
			contact.realRelPos1 = realRelPos1;
			contact.realRelPos2 = realRelPos2;
			contact.relativePos1 = relativePos1;
			contact.relativePos2 = relativePos2;
			contact.p1 = p1;
			contact.p2 = p2;
			contact.accumulatedNormalImpulse = accumulatedNormalImpulse;
			contact.accumulatedTangentImpulse = accumulatedTangentImpulse;
			contact.penetration = penetration;
			contact.initialPen = initialPen;
			contact.staticFriction = staticFriction;
			contact.dynamicFriction = dynamicFriction;
			contact.restitution = restitution;
			contact.friction = friction;
			contact.massNormal = massNormal;
			contact.massTangent = massTangent;
			contact.restitutionBias = restitutionBias;
			contact.newContact = newContact;
			contact.treatBody1AsStatic = treatBody1AsStatic;
			contact.treatBody2AsStatic = treatBody2AsStatic;
			contact.body1IsMassPoint = body1IsMassPoint;
			contact.body2IsMassPoint = body2IsMassPoint;
			contact.lostSpeculativeBounce = lostSpeculativeBounce;
			contact.speculativeVelocity = speculativeVelocity;
			contact.lastTimeStep = lastTimeStep;
		}

    }

}