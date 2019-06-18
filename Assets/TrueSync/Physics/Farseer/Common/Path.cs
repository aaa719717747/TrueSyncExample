using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;

namespace TrueSync.Physics2D
{
    //Contributed by Matthew Bettcher

    /// <summary>
    /// Path:
    /// Very similar to Vertices, but this
    /// class contains vectors describing
    /// control points on a Catmull-Rom
    /// curve.
    /// </summary>
    public class Path
    {
        /// <summary>
        /// All the points that makes up the curve
        /// </summary>
        public List<TSVector2> ControlPoints;

        private FP _deltaT;

        /// <summary>
        /// Initializes a new instance of the <see cref="Path"/> class.
        /// </summary>
        public Path()
        {
            ControlPoints = new List<TSVector2>();
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Path"/> class.
        /// </summary>
        /// <param name="vertices">The vertices to created the path from.</param>
        public Path(TSVector2[] vertices)
        {
            ControlPoints = new List<TSVector2>(vertices.Length);

            for (int i = 0; i < vertices.Length; i++)
            {
                Add(vertices[i]);
            }
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Path"/> class.
        /// </summary>
        /// <param name="vertices">The vertices to created the path from.</param>
        public Path(IList<TSVector2> vertices)
        {
            ControlPoints = new List<TSVector2>(vertices.Count);
            for (int i = 0; i < vertices.Count; i++)
            {
                Add(vertices[i]);
            }
        }

        /// <summary>
        /// True if the curve is closed.
        /// </summary>
        /// <value><c>true</c> if closed; otherwise, <c>false</c>.</value>
        public bool Closed { get; set; }

        /// <summary>
        /// Gets the next index of a controlpoint
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int NextIndex(int index)
        {
            if (index == ControlPoints.Count - 1)
            {
                return 0;
            }
            return index + 1;
        }

        /// <summary>
        /// Gets the previous index of a controlpoint
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int PreviousIndex(int index)
        {
            if (index == 0)
            {
                return ControlPoints.Count - 1;
            }
            return index - 1;
        }

        /// <summary>
        /// Translates the control points by the specified vector.
        /// </summary>
        /// <param name="vector">The vector.</param>
        public void Translate(ref TSVector2 vector)
        {
            for (int i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = TSVector2.Add(ControlPoints[i], vector);
        }

        /// <summary>
        /// Scales the control points by the specified vector.
        /// </summary>
        /// <param name="value">The Value.</param>
        public void Scale(ref TSVector2 value)
        {
            for (int i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = TSVector2.Multiply(ControlPoints[i], value);
        }

        public override string ToString()
        {
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < ControlPoints.Count; i++)
            {
                builder.Append(ControlPoints[i].ToString());
                if (i < ControlPoints.Count - 1)
                {
                    builder.Append(" ");
                }
            }
            return builder.ToString();
        }

        /// <summary>
        /// Returns a set of points defining the
        /// curve with the specifed number of divisions
        /// between each control point.
        /// </summary>
        /// <param name="divisions">Number of divisions between each control point.</param>
        /// <returns></returns>
        public Vertices GetVertices(int divisions)
        {
            Vertices verts = new Vertices();

            FP timeStep = 1f / divisions;

            for (FP i = 0; i < 1f; i += timeStep)
            {
                verts.Add(GetPosition(i));
            }

            return verts;
        }

        public TSVector2 GetPosition(FP time)
        {
            TSVector2 temp;

            if (ControlPoints.Count < 2)
                throw new Exception("You need at least 2 control points to calculate a position.");

            if (Closed)
            {
                Add(ControlPoints[0]);

                _deltaT = 1f / (ControlPoints.Count - 1);

                int p = (int)(time / _deltaT);

                // use a circular indexing system
                int p0 = p - 1;
                if (p0 < 0) p0 = p0 + (ControlPoints.Count - 1);
                else if (p0 >= ControlPoints.Count - 1) p0 = p0 - (ControlPoints.Count - 1);
                int p1 = p;
                if (p1 < 0) p1 = p1 + (ControlPoints.Count - 1);
                else if (p1 >= ControlPoints.Count - 1) p1 = p1 - (ControlPoints.Count - 1);
                int p2 = p + 1;
                if (p2 < 0) p2 = p2 + (ControlPoints.Count - 1);
                else if (p2 >= ControlPoints.Count - 1) p2 = p2 - (ControlPoints.Count - 1);
                int p3 = p + 2;
                if (p3 < 0) p3 = p3 + (ControlPoints.Count - 1);
                else if (p3 >= ControlPoints.Count - 1) p3 = p3 - (ControlPoints.Count - 1);

                // relative time
                FP lt = (time - _deltaT * p) / _deltaT;

                temp = TSVector2.CatmullRom(ControlPoints[p0], ControlPoints[p1], ControlPoints[p2], ControlPoints[p3], lt);

                RemoveAt(ControlPoints.Count - 1);
            }
            else
            {
                int p = (int)(time / _deltaT);

                // 
                int p0 = p - 1;
                if (p0 < 0) p0 = 0;
                else if (p0 >= ControlPoints.Count - 1) p0 = ControlPoints.Count - 1;
                int p1 = p;
                if (p1 < 0) p1 = 0;
                else if (p1 >= ControlPoints.Count - 1) p1 = ControlPoints.Count - 1;
                int p2 = p + 1;
                if (p2 < 0) p2 = 0;
                else if (p2 >= ControlPoints.Count - 1) p2 = ControlPoints.Count - 1;
                int p3 = p + 2;
                if (p3 < 0) p3 = 0;
                else if (p3 >= ControlPoints.Count - 1) p3 = ControlPoints.Count - 1;

                // relative time
                FP lt = (time - _deltaT * p) / _deltaT;

                temp = TSVector2.CatmullRom(ControlPoints[p0], ControlPoints[p1], ControlPoints[p2], ControlPoints[p3], lt);
            }

            return temp;
        }

        /// <summary>
        /// Gets the normal for the given time.
        /// </summary>
        /// <param name="time">The time</param>
        /// <returns>The normal.</returns>
        public TSVector2 GetPositionNormal(FP time)
        {
            FP offsetTime = time + 0.0001f;

            TSVector2 a = GetPosition(time);
            TSVector2 b = GetPosition(offsetTime);

            TSVector2 output, temp;

            TSVector2.Subtract(ref a, ref b, out temp);

#if (XBOX360 || WINDOWS_PHONE)
output = new Vector2();
#endif
            output.x = -temp.y;
            output.y = temp.x;

            TSVector2.Normalize(ref output, out output);

            return output;
        }

        public void Add(TSVector2 point)
        {
            ControlPoints.Add(point);
            _deltaT = 1f / (ControlPoints.Count - 1);
        }

        public void Remove(TSVector2 point)
        {
            ControlPoints.Remove(point);
            _deltaT = 1f / (ControlPoints.Count - 1);
        }

        public void RemoveAt(int index)
        {
            ControlPoints.RemoveAt(index);
            _deltaT = 1f / (ControlPoints.Count - 1);
        }

        public FP GetLength()
        {
            List<TSVector2> verts = GetVertices(ControlPoints.Count * 25);
            FP length = 0;

            for (int i = 1; i < verts.Count; i++)
            {
                length += TSVector2.Distance(verts[i - 1], verts[i]);
            }

            if (Closed)
                length += TSVector2.Distance(verts[ControlPoints.Count - 1], verts[0]);

            return length;
        }

        public List<TSVector> SubdivideEvenly(int divisions)
        {
            List<TSVector> verts = new List<TSVector>();

            FP length = GetLength();

            FP deltaLength = length / divisions + 0.001f;
            FP t = 0.000f;

            // we always start at the first control point
            TSVector2 start = ControlPoints[0];
            TSVector2 end = GetPosition(t);

            // increment t until we are at half the distance
            while (deltaLength * 0.5f >= TSVector2.Distance(start, end))
            {
                end = GetPosition(t);
                t += 0.0001f;

                if (t >= 1f)
                    break;
            }

            start = end;

            // for each box
            for (int i = 1; i < divisions; i++)
            {
                TSVector2 normal = GetPositionNormal(t);
                FP angle = FP.Atan2(normal.y, normal.x);

                verts.Add(new TSVector(end.x, end.y, angle));

                // until we reach the correct distance down the curve
                while (deltaLength >= TSVector2.Distance(start, end))
                {
                    end = GetPosition(t);
                    t += 0.00001f;

                    if (t >= 1f)
                        break;
                }
                if (t >= 1f)
                    break;

                start = end;
            }
            return verts;
        }
    }
}