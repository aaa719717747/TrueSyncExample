namespace TrueSync.Physics3D {

	public class Constraint2D : Constraint {

        private bool freezeZAxis;

		public Constraint2D(RigidBody body, bool freezeZAxis) : base(body, null) {
            this.freezeZAxis = freezeZAxis;

        }			

		public override void PostStep() {
			TSVector pos = Body1.Position;
			pos.z = 0;
			Body1.Position = pos;

			TSQuaternion q = TSQuaternion.CreateFromMatrix(Body1.Orientation);
			q.Normalize();
			q.x = 0;
			q.y = 0;

			if (freezeZAxis) {
				q.z = 0;
			}

			Body1.Orientation = TSMatrix.CreateFromQuaternion(q);

			if (Body1.isStatic) {
				return;
			}

			TSVector vel = Body1.LinearVelocity;
			vel.z = 0;
			Body1.LinearVelocity = vel;

            TSVector av = Body1.AngularVelocity;
			av.x = 0;
			av.y = 0;

            if (freezeZAxis) {
                av.z = 0;
            }

			Body1.AngularVelocity = av;
		}

	}

}