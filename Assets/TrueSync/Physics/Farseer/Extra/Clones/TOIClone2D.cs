namespace TrueSync.Physics2D {

    internal class TOIClone2D {

        public DistanceProxy ProxyA = new DistanceProxy();
        public DistanceProxy ProxyB = new DistanceProxy();
        public Sweep SweepA;
        public Sweep SweepB;
        public FP TMax;

        public void Clone(TOIInput input) {
            this.ProxyA.Radius = input.ProxyA.Radius;
            this.ProxyA.Vertices.Clear();
            this.ProxyA.Vertices.AddRange(input.ProxyA.Vertices);

            this.ProxyB.Radius = input.ProxyB.Radius;
            this.ProxyB.Vertices.Clear();
            this.ProxyB.Vertices.AddRange(input.ProxyB.Vertices);

            this.SweepA = input.SweepA;
            this.SweepB = input.SweepB;
            this.TMax = input.TMax;
        }

		public void Restore(TOIInput input) {
            input.ProxyA.Radius = this.ProxyA.Radius;
            input.ProxyA.Vertices.Clear();
            input.ProxyA.Vertices.AddRange(this.ProxyA.Vertices);

            input.ProxyB.Radius = this.ProxyB.Radius;
            input.ProxyB.Vertices.Clear();
            input.ProxyB.Vertices.AddRange(this.ProxyB.Vertices);

            input.SweepA = this.SweepA;
            input.SweepB = this.SweepB;
            input.TMax = this.TMax;
        }

    }

}