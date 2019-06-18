using FP = TrueSync.FP;

namespace TrueSync.Physics2D
{
    internal class Point
    {
        // Pointers to next and previous points in Monontone Mountain
        public Point Next, Prev;
        public FP X, Y;

        public Point(FP x, FP y)
        {
            X = x;
            Y = y;
            Next = null;
            Prev = null;
        }

        public static Point operator -(Point p1, Point p2)
        {
            return new Point(p1.X - p2.X, p1.Y - p2.Y);
        }

        public static Point operator +(Point p1, Point p2)
        {
            return new Point(p1.X + p2.X, p1.Y + p2.Y);
        }

        public static Point operator -(Point p1, FP f)
        {
            return new Point(p1.X - f, p1.Y - f);
        }

        public static Point operator +(Point p1, FP f)
        {
            return new Point(p1.X + f, p1.Y + f);
        }

        public FP Cross(Point p)
        {
            return X * p.Y - Y * p.X;
        }

        public FP Dot(Point p)
        {
            return X * p.X + Y * p.Y;
        }

        public bool Neq(Point p)
        {
            return p.X != X || p.Y != Y;
        }

        public FP Orient2D(Point pb, Point pc)
        {
            FP acx = X - pc.X;
            FP bcx = pb.X - pc.X;
            FP acy = Y - pc.Y;
            FP bcy = pb.Y - pc.Y;
            return acx * bcy - acy * bcx;
        }
    }
}