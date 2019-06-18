/*
* Farseer Physics Engine:
* Copyright (c) 2012 Ian Qvist
*/

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using Microsoft.Xna.Framework;

namespace TrueSync.Physics2D
{
    public enum PolygonError
    {
        /// <summary>
        /// There were no errors in the polygon
        /// </summary>
        NoError,

        /// <summary>
        /// Polygon must have between 3 and Settings.MaxPolygonVertices vertices.
        /// </summary>
        InvalidAmountOfVertices,

        /// <summary>
        /// Polygon must be simple. This means no overlapping edges.
        /// </summary>
        NotSimple,

        /// <summary>
        /// Polygon must have a counter clockwise winding.
        /// </summary>
        NotCounterClockWise,

        /// <summary>
        /// The polygon is concave, it needs to be convex.
        /// </summary>
        NotConvex,

        /// <summary>
        /// Polygon area is too small.
        /// </summary>
        AreaTooSmall,

        /// <summary>
        /// The polygon has a side that is too short.
        /// </summary>
        SideTooSmall
    }

#if !(XBOX360)
    [DebuggerDisplay("Count = {Count} Vertices = {ToString()}")]
#endif
    public class Vertices : List<TSVector2>
    {
        public Vertices() { }

        public Vertices(int capacity) : base(capacity) { }

        public Vertices(IEnumerable<TSVector2> vertices)
        {
            AddRange(vertices);
        }

        internal bool AttachedToBody { get; set; }

        /// <summary>
        /// You can add holes to this collection.
        /// It will get respected by some of the triangulation algoithms, but otherwise not used.
        /// </summary>
        public List<Vertices> Holes { get; set; }

        /// <summary>
        /// Gets the next index. Used for iterating all the edges with wrap-around.
        /// </summary>
        /// <param name="index">The current index</param>
        public int NextIndex(int index)
        {
            return (index + 1 > Count - 1) ? 0 : index + 1;
        }

        /// <summary>
        /// Gets the next vertex. Used for iterating all the edges with wrap-around.
        /// </summary>
        /// <param name="index">The current index</param>
        public TSVector2 NextVertex(int index)
        {
            return this[NextIndex(index)];
        }

        /// <summary>
        /// Gets the previous index. Used for iterating all the edges with wrap-around.
        /// </summary>
        /// <param name="index">The current index</param>
        public int PreviousIndex(int index)
        {
            return index - 1 < 0 ? Count - 1 : index - 1;
        }

        /// <summary>
        /// Gets the previous vertex. Used for iterating all the edges with wrap-around.
        /// </summary>
        /// <param name="index">The current index</param>
        public TSVector2 PreviousVertex(int index)
        {
            return this[PreviousIndex(index)];
        }

        /// <summary>
        /// Gets the signed area.
        /// If the area is less than 0, it indicates that the polygon is clockwise winded.
        /// </summary>
        /// <returns>The signed area</returns>
        public FP GetSignedArea()
        {
            //The simplest polygon which can exist in the Euclidean plane has 3 sides.
            if (Count < 3)
                return 0;

            int i;
            FP area = 0;

            for (i = 0; i < Count; i++)
            {
                int j = (i + 1) % Count;

                TSVector2 vi = this[i];
                TSVector2 vj = this[j];

                area += vi.x * vj.y;
                area -= vi.y * vj.x;
            }
            area /= 2.0f;
            return area;
        }

        /// <summary>
        /// Gets the area.
        /// </summary>
        /// <returns></returns>
        public FP GetArea()
        {
            FP area = GetSignedArea();
            return (area < 0 ? -area : area);
        }

        /// <summary>
        /// Gets the centroid.
        /// </summary>
        /// <returns></returns>
        public TSVector2 GetCentroid()
        {
            //The simplest polygon which can exist in the Euclidean plane has 3 sides.
            if (Count < 3)
                return new TSVector2(FP.NaN, FP.NaN);

            // Same algorithm is used by Box2D
            TSVector2 c = TSVector2.zero;
            FP area = 0.0f;
            FP inv3 = 1.0f / 3.0f;

            for (int i = 0; i < Count; ++i)
            {
                // Triangle vertices.
                TSVector2 current = this[i];
                TSVector2 next = (i + 1 < Count ? this[i + 1] : this[0]);

                FP triangleArea = 0.5f * (current.x * next.y - current.y * next.x);
                area += triangleArea;

                // Area weighted centroid
                c += triangleArea * inv3 * (current + next);
            }

            // Centroid
            c *= 1.0f / area;
            return c;
        }

        /// <summary>
        /// Returns an AABB that fully contains this polygon.
        /// </summary>
        public AABB GetAABB()
        {
            AABB aabb;
            TSVector2 lowerBound = new TSVector2(FP.MaxValue, FP.MaxValue);
            TSVector2 upperBound = new TSVector2(FP.MinValue, FP.MinValue);

            for (int i = 0; i < Count; ++i)
            {
                if (this[i].x < lowerBound.x)
                {
                    lowerBound.x = this[i].x;
                }
                if (this[i].x > upperBound.x)
                {
                    upperBound.x = this[i].x;
                }

                if (this[i].y < lowerBound.y)
                {
                    lowerBound.y = this[i].y;
                }
                if (this[i].y > upperBound.y)
                {
                    upperBound.y = this[i].y;
                }
            }

            aabb.LowerBound = lowerBound;
            aabb.UpperBound = upperBound;

            return aabb;
        }

        /// <summary>
        /// Translates the vertices with the specified vector.
        /// </summary>
        /// <param name="value">The value.</param>
        public void Translate(TSVector2 value)
        {
            Translate(ref value);
        }

        /// <summary>
        /// Translates the vertices with the specified vector.
        /// </summary>
        /// <param name="value">The vector.</param>
        public void Translate(ref TSVector2 value)
        {
            Debug.Assert(!AttachedToBody, "Translating vertices that are used by a Body can result in unstable behavior. Use Body.Position instead.");

            for (int i = 0; i < Count; i++)
                this[i] = TSVector2.Add(this[i], value);

            if (Holes != null && Holes.Count > 0)
            {
                foreach (Vertices hole in Holes)
                {
                    hole.Translate(ref value);
                }
            }
        }

        /// <summary>
        /// Scales the vertices with the specified vector.
        /// </summary>
        /// <param name="value">The Value.</param>
        public void Scale(TSVector2 value)
        {
            Scale(ref value);
        }

        /// <summary>
        /// Scales the vertices with the specified vector.
        /// </summary>
        /// <param name="value">The Value.</param>
        public void Scale(ref TSVector2 value)
        {
            Debug.Assert(!AttachedToBody, "Scaling vertices that are used by a Body can result in unstable behavior.");

            for (int i = 0; i < Count; i++)
                this[i] = TSVector2.Multiply(this[i], value);

            if (Holes != null && Holes.Count > 0)
            {
                foreach (Vertices hole in Holes)
                {
                    hole.Scale(ref value);
                }
            }
        }

        /// <summary>
        /// Rotate the vertices with the defined value in radians.
        /// 
        /// Warning: Using this method on an active set of vertices of a Body,
        /// will cause problems with collisions. Use Body.Rotation instead.
        /// </summary>
        /// <param name="value">The amount to rotate by in radians.</param>
        public void Rotate(FP value)
        {
            Debug.Assert(!AttachedToBody, "Rotating vertices that are used by a Body can result in unstable behavior.");

            FP num1 = FP.Cos(value);
            FP num2 = FP.Sin(value);

            for (int i = 0; i < Count; i++)
            {
                TSVector2 position = this[i];
                this[i] = new TSVector2((position.x * num1 + position.y * -num2), (position.x * num2 + position.y * num1));
            }

            if (Holes != null && Holes.Count > 0)
            {
                foreach (Vertices hole in Holes)
                {
                    hole.Rotate(value);
                }
            }
        }

        /// <summary>
        /// Determines whether the polygon is convex.
        /// O(n^2) running time.
        /// 
        /// Assumptions:
        /// - The polygon is in counter clockwise order
        /// - The polygon has no overlapping edges
        /// </summary>
        /// <returns>
        /// 	<c>true</c> if it is convex; otherwise, <c>false</c>.
        /// </returns>
        public bool IsConvex()
        {
            //The simplest polygon which can exist in the Euclidean plane has 3 sides.
            if (Count < 3)
                return false;

            //Triangles are always convex
            if (Count == 3)
                return true;

            // Checks the polygon is convex and the interior is to the left of each edge.
            for (int i = 0; i < Count; ++i)
            {
                int next = i + 1 < Count ? i + 1 : 0;
                TSVector2 edge = this[next] - this[i];

                for (int j = 0; j < Count; ++j)
                {
                    // Don't check vertices on the current edge.
                    if (j == i || j == next)
                        continue;

                    TSVector2 r = this[j] - this[i];

                    FP s = edge.x * r.y - edge.y * r.x;

                    if (s <= 0.0f)
                        return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Indicates if the vertices are in counter clockwise order.
        /// Warning: If the area of the polygon is 0, it is unable to determine the winding.
        /// </summary>
        public bool IsCounterClockWise()
        {
            //The simplest polygon which can exist in the Euclidean plane has 3 sides.
            if (Count < 3)
                return false;

            return (GetSignedArea() > 0.0f);
        }

        /// <summary>
        /// Forces the vertices to be counter clock wise order.
        /// </summary>
        public void ForceCounterClockWise()
        {
            //The simplest polygon which can exist in the Euclidean plane has 3 sides.
            if (Count < 3)
                return;

            if (!IsCounterClockWise())
                Reverse();
        }

        /// <summary>
        /// Checks if the vertices forms an simple polygon by checking for edge crossings.
        /// </summary>
        public bool IsSimple()
        {
            //The simplest polygon which can exist in the Euclidean plane has 3 sides.
            if (Count < 3)
                return false;

            for (int i = 0; i < Count; ++i)
            {
                TSVector2 a1 = this[i];
                TSVector2 a2 = NextVertex(i);
                for (int j = i + 1; j < Count; ++j)
                {
                    TSVector2 b1 = this[j];
                    TSVector2 b2 = NextVertex(j);

                    TSVector2 temp;

                    if (LineTools.LineIntersect2(ref a1, ref a2, ref b1, ref b2, out temp))
                        return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Checks if the polygon is valid for use in the engine.
        ///
        /// Performs a full check, for simplicity, convexity,
        /// orientation, minimum angle, and volume.
        /// 
        /// From Eric Jordan's convex decomposition library
        /// </summary>
        /// <returns>PolygonError.NoError if there were no error.</returns>
        public PolygonError CheckPolygon()
        {
            if (Count < 3 || Count > Settings.MaxPolygonVertices)
                return PolygonError.InvalidAmountOfVertices;

            if (!IsSimple())
                return PolygonError.NotSimple;

            if (GetArea() <= Settings.Epsilon)
                return PolygonError.AreaTooSmall;

            if (!IsConvex())
                return PolygonError.NotConvex;

            //Check if the sides are of adequate length.
            for (int i = 0; i < Count; ++i)
            {
                int next = i + 1 < Count ? i + 1 : 0;
                TSVector2 edge = this[next] - this[i];
                if (edge.LengthSquared() <= Settings.EpsilonSqr)
                {
                    return PolygonError.SideTooSmall;
                }
            }

            if (!IsCounterClockWise())
                return PolygonError.NotCounterClockWise;

            return PolygonError.NoError;
        }

        /// <summary>
        /// Projects to axis.
        /// </summary>
        /// <param name="axis">The axis.</param>
        /// <param name="min">The min.</param>
        /// <param name="max">The max.</param>
        public void ProjectToAxis(ref TSVector2 axis, out FP min, out FP max)
        {
            // To project a point on an axis use the dot product
            FP dotProduct = TSVector2.Dot(axis, this[0]);
            min = dotProduct;
            max = dotProduct;

            for (int i = 0; i < Count; i++)
            {
                dotProduct = TSVector2.Dot(this[i], axis);
                if (dotProduct < min)
                {
                    min = dotProduct;
                }
                else
                {
                    if (dotProduct > max)
                    {
                        max = dotProduct;
                    }
                }
            }
        }

        /// <summary>
        /// Winding number test for a point in a polygon.
        /// </summary>
        /// See more info about the algorithm here: http://softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm
        /// <param name="point">The point to be tested.</param>
        /// <returns>-1 if the winding number is zero and the point is outside
        /// the polygon, 1 if the point is inside the polygon, and 0 if the point
        /// is on the polygons edge.</returns>
        public int PointInPolygon(ref TSVector2 point)
        {
            // Winding number
            int wn = 0;

            // Iterate through polygon's edges
            for (int i = 0; i < Count; i++)
            {
                // Get points
                TSVector2 p1 = this[i];
                TSVector2 p2 = this[NextIndex(i)];

                // Test if a point is directly on the edge
                TSVector2 edge = p2 - p1;
                FP area = MathUtils.Area(ref p1, ref p2, ref point);
                if (area == 0f && TSVector2.Dot(point - p1, edge) >= 0f && TSVector2.Dot(point - p2, edge) <= 0f)
                {
                    return 0;
                }
                // Test edge for intersection with ray from point
                if (p1.y <= point.y)
                {
                    if (p2.y > point.y && area > 0f)
                    {
                        ++wn;
                    }
                }
                else
                {
                    if (p2.y <= point.y && area < 0f)
                    {
                        --wn;
                    }
                }
            }
            return (wn == 0 ? -1 : 1);
        }

        /// <summary>
        /// Compute the sum of the angles made between the test point and each pair of points making up the polygon. 
        /// If this sum is 2pi then the point is an interior point, if 0 then the point is an exterior point. 
        /// ref: http://ozviz.wasp.uwa.edu.au/~pbourke/geometry/insidepoly/  - Solution 2 
        /// </summary>
        public bool PointInPolygonAngle(ref TSVector2 point)
        {
            FP angle = 0;

            // Iterate through polygon's edges
            for (int i = 0; i < Count; i++)
            {
                // Get points
                TSVector2 p1 = this[i] - point;
                TSVector2 p2 = this[NextIndex(i)] - point;

                angle += MathUtils.VectorAngle(ref p1, ref p2);
            }

            if (FP.Abs(angle) < FP.Pi)
            {
                return false;
            }

            return true;
        }

        #region Matt Bettcher's Extension

        #region Fields

        static readonly IndexableCyclicalLinkedList<Vertex> polygonVertices = new IndexableCyclicalLinkedList<Vertex>();
        static readonly IndexableCyclicalLinkedList<Vertex> earVertices = new IndexableCyclicalLinkedList<Vertex>();
        static readonly CyclicalList<Vertex> convexVertices = new CyclicalList<Vertex>();
        static readonly CyclicalList<Vertex> reflexVertices = new CyclicalList<Vertex>();

        #endregion

        #region Vertex

        struct Vertex {
            public readonly TSVector2 Position;
            public readonly short Index;

            public Vertex(TSVector2 position, short index) {
                Position = position;
                Index = index;
            }

            public override bool Equals(object obj) {
                if (obj.GetType() != typeof(Vertex))
                    return false;
                return Equals((Vertex)obj);
            }

            public bool Equals(Vertex obj) {
                return obj.Position.Equals(Position) && obj.Index == Index;
            }

            public override int GetHashCode() {
                unchecked {
                    return (Position.GetHashCode() * 397) ^ Index;
                }
            }

            public override string ToString() {
                return string.Format("{0} ({1})", Position, Index);
            }
        }
        #endregion

        #region LineSegment

        struct LineSegment {
            public Vertex A;
            public Vertex B;

            public LineSegment(Vertex a, Vertex b) {
                A = a;
                B = b;
            }

            public FP? IntersectsWithRay(TSVector2 origin, TSVector2 direction) {
                FP largestDistance = TSMath.Max(A.Position.x - origin.x, B.Position.x - origin.x) * 2f;
                LineSegment raySegment = new LineSegment(new Vertex(origin, 0), new Vertex(origin + (direction * largestDistance), 0));

                TSVector2? intersection = FindIntersection(this, raySegment);
                FP? value = null;

                if (intersection != null)
                    value = TSVector2.Distance(origin, intersection.Value);

                return value;
            }

            public static TSVector2? FindIntersection(LineSegment a, LineSegment b) {
                FP x1 = a.A.Position.x;
                FP y1 = a.A.Position.y;
                FP x2 = a.B.Position.x;
                FP y2 = a.B.Position.y;
                FP x3 = b.A.Position.x;
                FP y3 = b.A.Position.y;
                FP x4 = b.B.Position.x;
                FP y4 = b.B.Position.y;

                FP denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

                FP uaNum = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
                FP ubNum = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

                FP ua = uaNum / denom;
                FP ub = ubNum / denom;

                if (TSMath.Clamp(ua, 0f, 1f) != ua || TSMath.Clamp(ub, 0f, 1f) != ub)
                    return null;

                return a.A.Position + (a.B.Position - a.A.Position) * ua;
            }
        }

        #endregion

        #region Triangle

        /// <summary>
        /// A basic triangle structure that holds the three vertices that make up a given triangle.
        /// </summary>
        struct Triangle {
            public readonly Vertex A;
            public readonly Vertex B;
            public readonly Vertex C;

            public Triangle(Vertex a, Vertex b, Vertex c) {
                A = a;
                B = b;
                C = c;
            }

            public bool ContainsPoint(Vertex point) {
                //return true if the point to test is one of the vertices
                if (point.Equals(A) || point.Equals(B) || point.Equals(C))
                    return true;

                bool oddNodes = false;

                if (checkPointToSegment(C, A, point))
                    oddNodes = !oddNodes;
                if (checkPointToSegment(A, B, point))
                    oddNodes = !oddNodes;
                if (checkPointToSegment(B, C, point))
                    oddNodes = !oddNodes;

                return oddNodes;
            }

            public static bool ContainsPoint(Vertex a, Vertex b, Vertex c, Vertex point) {
                return new Triangle(a, b, c).ContainsPoint(point);
            }

            static bool checkPointToSegment(Vertex sA, Vertex sB, Vertex point) {
                if ((sA.Position.y < point.Position.y && sB.Position.y >= point.Position.y) ||
                    (sB.Position.y < point.Position.y && sA.Position.y >= point.Position.y)) {
                    FP x =
                        sA.Position.x +
                        (point.Position.y - sA.Position.y) /
                        (sB.Position.y - sA.Position.y) *
                        (sB.Position.x - sA.Position.x);

                    if (x < point.Position.x)
                        return true;
                }

                return false;
            }

            public override bool Equals(object obj) {
                if (obj.GetType() != typeof(Triangle))
                    return false;
                return Equals((Triangle)obj);
            }

            public bool Equals(Triangle obj) {
                return obj.A.Equals(A) && obj.B.Equals(B) && obj.C.Equals(C);
            }

            public override int GetHashCode() {
                unchecked {
                    int result = A.GetHashCode();
                    result = (result * 397) ^ B.GetHashCode();
                    result = (result * 397) ^ C.GetHashCode();
                    return result;
                }
            }
        }
        #endregion

        #region CyclicalList
        /// <summary>
        /// Implements a List structure as a cyclical list where indices are wrapped.
        /// </summary>
        /// <typeparam name="T">The Type to hold in the list.</typeparam>
        class CyclicalList<T> : List<T> {
            public new T this[int index] {
                get {
                    //perform the index wrapping
                    while (index < 0)
                        index = Count + index;
                    if (index >= Count)
                        index %= Count;

                    return base[index];
                }
            }
        }
        #endregion

        #region IndexableCyclicalLinkedList
        /// <summary>
        /// Implements a LinkedList that is both indexable as well as cyclical. Thus
        /// indexing into the list with an out-of-bounds index will automatically cycle
        /// around the list to find a valid node.
        /// </summary>
        class IndexableCyclicalLinkedList<T> : LinkedList<T> {
            /// <summary>
            /// Gets the LinkedListNode at a particular index.
            /// </summary>
            /// <param name="index">The index of the node to retrieve.</param>
            /// <returns>The LinkedListNode found at the index given.</returns>
            public LinkedListNode<T> this[int index] {
                get {
                    //perform the index wrapping
                    while (index < 0)
                        index = Count + index;
                    if (index >= Count)
                        index %= Count;

                    //find the proper node
                    LinkedListNode<T> node = First;
                    for (int i = 0; i < index; i++)
                        node = node.Next;

                    return node;
                }
            }

            /// <summary>
            /// Removes the node at a given index.
            /// </summary>
            /// <param name="index">The index of the node to remove.</param>
            public void RemoveAt(int index) {
                Remove(this[index]);
            }

            /// <summary>
            /// Finds the index of a given item.
            /// </summary>
            /// <param name="item">The item to find.</param>
            /// <returns>The index of the item if found; -1 if the item is not found.</returns>
            public int IndexOf(T item) {
                for (int i = 0; i < Count; i++)
                    if (this[i].Value.Equals(item))
                        return i;

                return -1;
            }
        }
        #endregion

        #region Public Methods

        #region Triangulate

        /// <summary>
        /// Triangulates a 2D polygon produced the indexes required to render the points as a triangle list.
        /// </summary>
        /// <param name="inputVertices">The polygon vertices in counter-clockwise winding order.</param>
        /// <param name="desiredWindingOrder">The desired output winding order.</param>
        /// <param name="outputVertices">The resulting vertices that include any reversals of winding order and holes.</param>
        /// <param name="indices">The resulting indices for rendering the shape as a triangle list.</param>
        public static void Triangulate(
            TSVector2[] inputVertices,
            WindingOrder desiredWindingOrder,
            out TSVector2[] outputVertices,
            out short[] indices) {
            //Log("\nBeginning triangulation...");

            List<Triangle> triangles = new List<Triangle>();

            //make sure we have our vertices wound properly
            if (DetermineWindingOrder(inputVertices) == WindingOrder.Clockwise)
                outputVertices = ReverseWindingOrder(inputVertices);
            else
                outputVertices = (TSVector2[])inputVertices.Clone();

            //clear all of the lists
            polygonVertices.Clear();
            earVertices.Clear();
            convexVertices.Clear();
            reflexVertices.Clear();

            //generate the cyclical list of vertices in the polygon
            for (int i = 0; i < outputVertices.Length; i++)
                polygonVertices.AddLast(new Vertex(outputVertices[i], (short)i));

            //categorize all of the vertices as convex, reflex, and ear
            FindConvexAndReflexVertices();
            FindEarVertices();

            //clip all the ear vertices
            while (polygonVertices.Count > 3 && earVertices.Count > 0)
                ClipNextEar(triangles);

            //if there are still three points, use that for the last triangle
            if (polygonVertices.Count == 3)
                triangles.Add(new Triangle(
                    polygonVertices[0].Value,
                    polygonVertices[1].Value,
                    polygonVertices[2].Value));

            //add all of the triangle indices to the output array
            indices = new short[triangles.Count * 3];

            //move the if statement out of the loop to prevent all the
            //redundant comparisons
            if (desiredWindingOrder == WindingOrder.CounterClockwise) {
                for (int i = 0; i < triangles.Count; i++) {
                    indices[(i * 3)] = triangles[i].A.Index;
                    indices[(i * 3) + 1] = triangles[i].B.Index;
                    indices[(i * 3) + 2] = triangles[i].C.Index;
                }
            } else {
                for (int i = 0; i < triangles.Count; i++) {
                    indices[(i * 3)] = triangles[i].C.Index;
                    indices[(i * 3) + 1] = triangles[i].B.Index;
                    indices[(i * 3) + 2] = triangles[i].A.Index;
                }
            }
        }

        #endregion

        #region CutHoleInShape

        /// <summary>
        /// Cuts a hole into a shape.
        /// </summary>
        /// <param name="shapeVerts">An array of vertices for the primary shape.</param>
        /// <param name="holeVerts">An array of vertices for the hole to be cut. It is assumed that these vertices lie completely within the shape verts.</param>
        /// <returns>The new array of vertices that can be passed to Triangulate to properly triangulate the shape with the hole.</returns>
        public static TSVector2[] CutHoleInShape(TSVector2[] shapeVerts, TSVector2[] holeVerts) {
            Log("\nCutting hole into shape...");

            //make sure the shape vertices are wound counter clockwise and the hole vertices clockwise
            shapeVerts = EnsureWindingOrder(shapeVerts, WindingOrder.CounterClockwise);
            holeVerts = EnsureWindingOrder(holeVerts, WindingOrder.Clockwise);

            //clear all of the lists
            polygonVertices.Clear();
            earVertices.Clear();
            convexVertices.Clear();
            reflexVertices.Clear();

            //generate the cyclical list of vertices in the polygon
            for (int i = 0; i < shapeVerts.Length; i++)
                polygonVertices.AddLast(new Vertex(shapeVerts[i], (short)i));

            CyclicalList<Vertex> holePolygon = new CyclicalList<Vertex>();
            for (int i = 0; i < holeVerts.Length; i++)
                holePolygon.Add(new Vertex(holeVerts[i], (short)(i + polygonVertices.Count)));

#if DEBUG
            StringBuilder vString = new StringBuilder();
            foreach (Vertex v in polygonVertices)
                vString.Append(string.Format("{0}, ", v));
            Log("Shape Vertices: {0}", vString);

            vString = new StringBuilder();
            foreach (Vertex v in holePolygon)
                vString.Append(string.Format("{0}, ", v));
            Log("Hole Vertices: {0}", vString);
#endif

            FindConvexAndReflexVertices();
            FindEarVertices();

            //find the hole vertex with the largest X value
            Vertex rightMostHoleVertex = holePolygon[0];
            foreach (Vertex v in holePolygon)
                if (v.Position.x > rightMostHoleVertex.Position.x)
                    rightMostHoleVertex = v;

            //construct a list of all line segments where at least one vertex
            //is to the right of the rightmost hole vertex with one vertex
            //above the hole vertex and one below
            List<LineSegment> segmentsToTest = new List<LineSegment>();
            for (int i = 0; i < polygonVertices.Count; i++) {
                Vertex a = polygonVertices[i].Value;
                Vertex b = polygonVertices[i + 1].Value;

                if ((a.Position.x > rightMostHoleVertex.Position.x || b.Position.x > rightMostHoleVertex.Position.x) &&
                    ((a.Position.y >= rightMostHoleVertex.Position.y && b.Position.y <= rightMostHoleVertex.Position.y) ||
                    (a.Position.y <= rightMostHoleVertex.Position.y && b.Position.y >= rightMostHoleVertex.Position.y)))
                    segmentsToTest.Add(new LineSegment(a, b));
            }

            //now we try to find the closest intersection point heading to the right from
            //our hole vertex.
            FP? closestPoint = null;
            LineSegment closestSegment = new LineSegment();
            foreach (LineSegment segment in segmentsToTest) {
                FP? intersection = segment.IntersectsWithRay(rightMostHoleVertex.Position, TSVector2.right);
                if (intersection != null) {
                    if (closestPoint == null || closestPoint.Value > intersection.Value) {
                        closestPoint = intersection;
                        closestSegment = segment;
                    }
                }
            }

            //if closestPoint is null, there were no collisions (likely from improper input data),
            //but we'll just return without doing anything else
            if (closestPoint == null)
                return shapeVerts;

            //otherwise we can find our mutually visible vertex to split the polygon
            TSVector2 I = rightMostHoleVertex.Position + TSVector2.right * closestPoint.Value;
            Vertex P = (closestSegment.A.Position.x > closestSegment.B.Position.x)
                ? closestSegment.A
                : closestSegment.B;

            //construct triangle MIP
            Triangle mip = new Triangle(rightMostHoleVertex, new Vertex(I, 1), P);

            //see if any of the reflex vertices lie inside of the MIP triangle
            List<Vertex> interiorReflexVertices = new List<Vertex>();
            foreach (Vertex v in reflexVertices)
                if (mip.ContainsPoint(v))
                    interiorReflexVertices.Add(v);

            //if there are any interior reflex vertices, find the one that, when connected
            //to our rightMostHoleVertex, forms the line closest to Vector2.UnitX
            if (interiorReflexVertices.Count > 0) {
                FP closestDot = -1f;
                foreach (Vertex v in interiorReflexVertices) {
                    //compute the dot product of the vector against the UnitX
                    TSVector2 d = TSVector2.Normalize(v.Position - rightMostHoleVertex.Position);
                    FP dot = TSVector2.Dot(TSVector2.right, d);

                    //if this line is the closest we've found
                    if (dot > closestDot) {
                        //save the value and save the vertex as P
                        closestDot = dot;
                        P = v;
                    }
                }
            }

            //now we just form our output array by injecting the hole vertices into place
            //we know we have to inject the hole into the main array after point P going from
            //rightMostHoleVertex around and then back to P.
            int mIndex = holePolygon.IndexOf(rightMostHoleVertex);
            int injectPoint = polygonVertices.IndexOf(P);

            Log("Inserting hole at injection point {0} starting at hole vertex {1}.",
                P,
                rightMostHoleVertex);
            for (int i = mIndex; i <= mIndex + holePolygon.Count; i++) {
                Log("Inserting vertex {0} after vertex {1}.", holePolygon[i], polygonVertices[injectPoint].Value);
                polygonVertices.AddAfter(polygonVertices[injectPoint++], holePolygon[i]);
            }
            polygonVertices.AddAfter(polygonVertices[injectPoint], P);

#if DEBUG
            vString = new StringBuilder();
            foreach (Vertex v in polygonVertices)
                vString.Append(string.Format("{0}, ", v));
            Log("New Shape Vertices: {0}\n", vString);
#endif

            //finally we write out the new polygon vertices and return them out
            TSVector2[] newShapeVerts = new TSVector2[polygonVertices.Count];
            for (int i = 0; i < polygonVertices.Count; i++)
                newShapeVerts[i] = polygonVertices[i].Value.Position;

            return newShapeVerts;
        }

        #endregion

        #region EnsureWindingOrder

        /// <summary>
        /// Ensures that a set of vertices are wound in a particular order, reversing them if necessary.
        /// </summary>
        /// <param name="vertices">The vertices of the polygon.</param>
        /// <param name="windingOrder">The desired winding order.</param>
        /// <returns>A new set of vertices if the winding order didn't match; otherwise the original set.</returns>
        public static TSVector2[] EnsureWindingOrder(TSVector2[] vertices, WindingOrder windingOrder) {
            //Log("\nEnsuring winding order of {0}...", windingOrder);
            if (DetermineWindingOrder(vertices) != windingOrder) {
                //Log("Reversing vertices...");
                return ReverseWindingOrder(vertices);
            }

            //Log("No reversal needed.");
            return vertices;
        }

        #endregion

        #region ReverseWindingOrder

        /// <summary>
        /// Reverses the winding order for a set of vertices.
        /// </summary>
        /// <param name="vertices">The vertices of the polygon.</param>
        /// <returns>The new vertices for the polygon with the opposite winding order.</returns>
        public static TSVector2[] ReverseWindingOrder(TSVector2[] vertices) {
            //Log("\nReversing winding order...");
            TSVector2[] newVerts = new TSVector2[vertices.Length];

#if DEBUG
            //StringBuilder vString = new StringBuilder();
            //foreach (Vector2 v in vertices)
            //	vString.Append(string.Format("{0}, ", v));
            //Log("Original Vertices: {0}", vString);
#endif

            newVerts[0] = vertices[0];
            for (int i = 1; i < newVerts.Length; i++)
                newVerts[i] = vertices[vertices.Length - i];

#if DEBUG
            //vString = new StringBuilder();
            //foreach (Vector2 v in newVerts)
            //	vString.Append(string.Format("{0}, ", v));
            //Log("New Vertices After Reversal: {0}\n", vString);
#endif

            return newVerts;
        }

        #endregion

        #region DetermineWindingOrder

        /// <summary>
        /// Determines the winding order of a polygon given a set of vertices.
        /// </summary>
        /// <param name="vertices">The vertices of the polygon.</param>
        /// <returns>The calculated winding order of the polygon.</returns>
        public static WindingOrder DetermineWindingOrder(TSVector2[] vertices) {
            int clockWiseCount = 0;
            int counterClockWiseCount = 0;
            TSVector2 p1 = vertices[0];

            for (int i = 1; i < vertices.Length; i++) {
                TSVector2 p2 = vertices[i];
                TSVector2 p3 = vertices[(i + 1) % vertices.Length];

                TSVector2 e1 = p1 - p2;
                TSVector2 e2 = p3 - p2;

                if (e1.x * e2.y - e1.y * e2.x >= 0)
                    clockWiseCount++;
                else
                    counterClockWiseCount++;

                p1 = p2;
            }

            return (clockWiseCount > counterClockWiseCount)
                ? WindingOrder.Clockwise
                : WindingOrder.CounterClockwise;
        }

        #endregion

        #endregion

        #region Private Methods

        #region ClipNextEar

        private static void ClipNextEar(ICollection<Triangle> triangles) {
            //find the triangle
            Vertex ear = earVertices[0].Value;
            Vertex prev = polygonVertices[polygonVertices.IndexOf(ear) - 1].Value;
            Vertex next = polygonVertices[polygonVertices.IndexOf(ear) + 1].Value;
            triangles.Add(new Triangle(ear, next, prev));

            //remove the ear from the shape
            earVertices.RemoveAt(0);
            polygonVertices.RemoveAt(polygonVertices.IndexOf(ear));
            //Log("\nRemoved Ear: {0}", ear);

            //validate the neighboring vertices
            ValidateAdjacentVertex(prev);
            ValidateAdjacentVertex(next);

            //write out the states of each of the lists
#if DEBUG
            /*StringBuilder rString = new StringBuilder();
            foreach (Vertex v in reflexVertices)
                rString.Append(string.Format("{0}, ", v.Index));
            Log("Reflex Vertices: {0}", rString);
            StringBuilder cString = new StringBuilder();
            foreach (Vertex v in convexVertices)
                cString.Append(string.Format("{0}, ", v.Index));
            Log("Convex Vertices: {0}", cString);
            StringBuilder eString = new StringBuilder();
            foreach (Vertex v in earVertices)
                eString.Append(string.Format("{0}, ", v.Index));
            Log("Ear Vertices: {0}", eString);*/
#endif
        }

        #endregion

        #region ValidateAdjacentVertex

        private static void ValidateAdjacentVertex(Vertex vertex) {
            //Log("Validating: {0}...", vertex);

            if (reflexVertices.Contains(vertex)) {
                if (IsConvex(vertex)) {
                    reflexVertices.Remove(vertex);
                    convexVertices.Add(vertex);
                    //Log("Vertex: {0} now convex", vertex);
                } else {
                    //Log("Vertex: {0} still reflex", vertex);
                }
            }

            if (convexVertices.Contains(vertex)) {
                bool wasEar = earVertices.Contains(vertex);
                bool isEar = IsEar(vertex);

                if (wasEar && !isEar) {
                    earVertices.Remove(vertex);
                    //Log("Vertex: {0} no longer ear", vertex);
                } else if (!wasEar && isEar) {
                    earVertices.AddFirst(vertex);
                    //Log("Vertex: {0} now ear", vertex);
                } else {
                    //Log("Vertex: {0} still ear", vertex);
                }
            }
        }

        #endregion

        #region FindConvexAndReflexVertices

        private static void FindConvexAndReflexVertices() {
            for (int i = 0; i < polygonVertices.Count; i++) {
                Vertex v = polygonVertices[i].Value;

                if (IsConvex(v)) {
                    convexVertices.Add(v);
                    //Log("Convex: {0}", v);
                } else {
                    reflexVertices.Add(v);
                    //Log("Reflex: {0}", v);
                }
            }
        }

        #endregion

        #region FindEarVertices

        private static void FindEarVertices() {
            for (int i = 0; i < convexVertices.Count; i++) {
                Vertex c = convexVertices[i];

                if (IsEar(c)) {
                    earVertices.AddLast(c);
                    //Log("Ear: {0}", c);
                }
            }
        }

        #endregion

        #region IsEar

        private static bool IsEar(Vertex c) {
            Vertex p = polygonVertices[polygonVertices.IndexOf(c) - 1].Value;
            Vertex n = polygonVertices[polygonVertices.IndexOf(c) + 1].Value;

            //Log("Testing vertex {0} as ear with triangle {1}, {0}, {2}...", c, p, n);

            foreach (Vertex t in reflexVertices) {
                if (t.Equals(p) || t.Equals(c) || t.Equals(n))
                    continue;

                if (Triangle.ContainsPoint(p, c, n, t)) {
                    //Log("\tTriangle contains vertex {0}...", t);
                    return false;
                }
            }

            return true;
        }

        #endregion

        #region IsConvex

        private static bool IsConvex(Vertex c) {
            Vertex p = polygonVertices[polygonVertices.IndexOf(c) - 1].Value;
            Vertex n = polygonVertices[polygonVertices.IndexOf(c) + 1].Value;

            TSVector2 d1 = TSVector2.Normalize(c.Position - p.Position);
            TSVector2 d2 = TSVector2.Normalize(n.Position - c.Position);
            TSVector2 n2 = new TSVector2(-d2.y, d2.x);

            return (TSVector2.Dot(d1, n2) <= 0f);
        }

        #endregion

        #region IsReflex

        private static bool IsReflex(Vertex c) {
            return !IsConvex(c);
        }

        #endregion

        #region Log

        //[Conditional("DEBUG")]
        private static void Log(string format, params object[] parameters) {
            System.Console.WriteLine(format, parameters);
        }

        #endregion

        #endregion

        #region WindingOrder

        /// <summary>
        /// Specifies a desired winding order for the shape vertices.
        /// </summary>
        public enum WindingOrder {
            Clockwise,
            CounterClockwise
        }

        #endregion

        #endregion

        public override string ToString()
        {
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < Count; i++)
            {
                builder.Append(this[i].ToString());
                if (i < Count - 1)
                {
                    builder.Append(" ");
                }
            }
            return builder.ToString();
        }
    }
}