using System;
using System.Collections.Generic;

namespace TrueSync.Physics2D
{
    public static class BodyFactory
    {
        public static Body CreateBody(World world, object userData = null)
        {
            Body body = new Body(world, null, 0, userData);
            return body;
        }

        // TS - public static Body CreateBody(World world, Vector2 position, FP rotation = 0, object userData = null)
        public static Body CreateBody(World world, TSVector2 position, FP rotation, object userData = null)
        {
            Body body = new Body(world, position, rotation, userData);
            return body;
        }

        public static Body CreateEdge(World world, TSVector2 start, TSVector2 end, object userData = null)
        {
            Body body = CreateBody(world);
            FixtureFactory.AttachEdge(start, end, body, userData);
            return body;
        }

        public static Body CreateChainShape(World world, Vertices vertices, object userData = null)
        {
            return CreateChainShape(world, vertices, TSVector2.zero, userData);
        }

        public static Body CreateChainShape(World world, Vertices vertices, TSVector2 position, object userData = null)
        {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachChainShape(vertices, body, userData);
            return body;
        }

        public static Body CreateLoopShape(World world, Vertices vertices, object userData = null)
        {
            return CreateLoopShape(world, vertices, TSVector2.zero, userData);
        }

        public static Body CreateLoopShape(World world, Vertices vertices, TSVector2 position, object userData = null)
        {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachLoopShape(vertices, body, userData);
            return body;
        }

        public static Body CreateRectangle(World world, FP width, FP height, FP density, object userData = null)
        {
            return CreateRectangle(world, width, height, density, TSVector2.zero, userData);
        }

        public static Body CreateRectangle(World world, FP width, FP height, FP density, TSVector2 position, object userData = null)
        {
            if (width <= 0)
                throw new ArgumentOutOfRangeException("width", "Width must be more than 0 meters");

            if (height <= 0)
                throw new ArgumentOutOfRangeException("height", "Height must be more than 0 meters");

            Body newBody = CreateBody(world, position);
            newBody.UserData = userData;

            Vertices rectangleVertices = PolygonTools.CreateRectangle(width / 2, height / 2);
            PolygonShape rectangleShape = new PolygonShape(rectangleVertices, density);
            newBody.CreateFixture(rectangleShape);

            return newBody;
        }

        public static Body CreateCircle(World world, FP radius, FP density, object userData = null)
        {
            return CreateCircle(world, radius, density, TSVector2.zero, userData);
        }

        public static Body CreateCircle(World world, FP radius, FP density, TSVector2 position, object userData = null)
        {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachCircle(radius, density, body, userData);
            return body;
        }

        public static Body CreateEllipse(World world, FP xRadius, FP yRadius, int edges, FP density, object userData = null)
        {
            return CreateEllipse(world, xRadius, yRadius, edges, density, TSVector2.zero, userData);
        }

        public static Body CreateEllipse(World world, FP xRadius, FP yRadius, int edges, FP density,
                                         TSVector2 position, object userData = null)
        {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachEllipse(xRadius, yRadius, edges, density, body, userData);
            return body;
        }

        public static Body CreatePolygon(World world, Vertices vertices, FP density, object userData)
        {
            return CreatePolygon(world, vertices, density, TSVector2.zero, userData);
        }

        public static Body CreatePolygon(World world, Vertices vertices, FP density, TSVector2 position, object userData)
        {
            Body body = CreateBody(world, position);
            FixtureFactory.AttachPolygon(vertices, density, body, userData);
            return body;
        }

        public static Body CreateCompoundPolygon(World world, List<Vertices> list, FP density, object userData = null)
        {
            return CreateCompoundPolygon(world, list, density, TSVector2.zero, userData);
        }

        public static Body CreateCompoundPolygon(World world, List<Vertices> list, FP density, TSVector2 position, object userData = null)
        {
            //We create a single body
            Body polygonBody = CreateBody(world, position);
            FixtureFactory.AttachCompoundPolygon(list, density, polygonBody, userData);
            return polygonBody;
        }

        public static Body CreateGear(World world, FP radius, int numberOfTeeth, FP tipPercentage, FP toothHeight, FP density, object userData = null)
        {
            Vertices gearPolygon = PolygonTools.CreateGear(radius, numberOfTeeth, tipPercentage, toothHeight);

            //Gears can in some cases be convex
            if (!gearPolygon.IsConvex())
            {
                //Decompose the gear:
                List<Vertices> list = Triangulate.ConvexPartition(gearPolygon, TriangulationAlgorithm.Earclip, true, FP.EN3);

                return CreateCompoundPolygon(world, list, density, userData);
            }

            return CreatePolygon(world, gearPolygon, density, userData);
        }

        /// <summary>
        /// Creates a capsule.
        /// Note: Automatically decomposes the capsule if it contains too many vertices (controlled by Settings.MaxPolygonVertices)
        /// </summary>
        /// <returns></returns>
        public static Body CreateCapsule(World world, FP height, FP topRadius, int topEdges, FP bottomRadius, int bottomEdges, FP density, TSVector2 position, object userData = null)
        {
            Vertices verts = PolygonTools.CreateCapsule(height, topRadius, topEdges, bottomRadius, bottomEdges);

            Body body;

            //There are too many vertices in the capsule. We decompose it.
            if (verts.Count >= Settings.MaxPolygonVertices)
            {
                List<Vertices> vertList = Triangulate.ConvexPartition(verts, TriangulationAlgorithm.Earclip, true, FP.EN3);
                body = CreateCompoundPolygon(world, vertList, density, userData);
                body.Position = position;

                return body;
            }

            body = CreatePolygon(world, verts, density, userData);
            body.Position = position;

            return body;
        }

        public static Body CreateCapsule(World world, FP height, FP endRadius, FP density,
                                         object userData = null)
        {
            //Create the middle rectangle
            Vertices rectangle = PolygonTools.CreateRectangle(endRadius, height / 2);

            List<Vertices> list = new List<Vertices>();
            list.Add(rectangle);

            Body body = CreateCompoundPolygon(world, list, density, userData);
            body.UserData = userData;

            //Create the two circles
            CircleShape topCircle = new CircleShape(endRadius, density);
            topCircle.Position = new TSVector2(0, height / 2);
            body.CreateFixture(topCircle);

            CircleShape bottomCircle = new CircleShape(endRadius, density);
            bottomCircle.Position = new TSVector2(0, -(height / 2));
            body.CreateFixture(bottomCircle);
            return body;
        }

        /// <summary>
        /// Creates a rounded rectangle.
        /// Note: Automatically decomposes the capsule if it contains too many vertices (controlled by Settings.MaxPolygonVertices)
        /// </summary>
        /// <param name="world">The world.</param>
        /// <param name="width">The width.</param>
        /// <param name="height">The height.</param>
        /// <param name="xRadius">The x radius.</param>
        /// <param name="yRadius">The y radius.</param>
        /// <param name="segments">The segments.</param>
        /// <param name="density">The density.</param>
        /// <param name="position">The position.</param>
        /// <returns></returns>
        public static Body CreateRoundedRectangle(World world, FP width, FP height, FP xRadius, FP yRadius, int segments, FP density, TSVector2 position, object userData = null)
        {
            Vertices verts = PolygonTools.CreateRoundedRectangle(width, height, xRadius, yRadius, segments);

            //There are too many vertices in the capsule. We decompose it.
            if (verts.Count >= Settings.MaxPolygonVertices)
            {
                List<Vertices> vertList = Triangulate.ConvexPartition(verts, TriangulationAlgorithm.Earclip, true, FP.EN3);
                Body body = CreateCompoundPolygon(world, vertList, density, userData);
                body.Position = position;
                return body;
            }

            return CreatePolygon(world, verts, density, null);
        }

        public static Body CreateRoundedRectangle(World world, FP width, FP height, FP xRadius, FP yRadius, int segments, FP density, object userData = null)
        {
            return CreateRoundedRectangle(world, width, height, xRadius, yRadius, segments, density, TSVector2.zero, userData);
        }

        public static BreakableBody CreateBreakableBody(World world, Vertices vertices, FP density)
        {
            return CreateBreakableBody(world, vertices, density, TSVector2.zero);
        }

        public static BreakableBody CreateBreakableBody(World world, IEnumerable<Shape> shapes)
        {
            return CreateBreakableBody(world, shapes, TSVector2.zero);
        }

        /// <summary>
        /// Creates a breakable body. You would want to remove collinear points before using this.
        /// </summary>
        /// <param name="world">The world.</param>
        /// <param name="vertices">The vertices.</param>
        /// <param name="density">The density.</param>
        /// <param name="position">The position.</param>
        /// <returns></returns>
        public static BreakableBody CreateBreakableBody(World world, Vertices vertices, FP density, TSVector2 position)
        {
            List<Vertices> triangles = Triangulate.ConvexPartition(vertices, TriangulationAlgorithm.Earclip, true, FP.EN3);

            BreakableBody breakableBody = new BreakableBody(triangles, world, density);
            breakableBody.MainBody.Position = position;
            world.AddBreakableBody(breakableBody);

            return breakableBody;
        }

        public static BreakableBody CreateBreakableBody(World world, IEnumerable<Shape> shapes, TSVector2 position)
        {
            BreakableBody breakableBody = new BreakableBody(shapes, world);
            breakableBody.MainBody.Position = position;
            world.AddBreakableBody(breakableBody);

            return breakableBody;
        }

        public static Body CreateLineArc(World world, FP radians, int sides, FP radius, TSVector2 position, FP angle, bool closed)
        {
            Body body = CreateBody(world);
            FixtureFactory.AttachLineArc(radians, sides, radius, position, angle, closed, body);
            return body;
        }

        public static Body CreateSolidArc(World world, FP density, FP radians, int sides, FP radius, TSVector2 position, FP angle)
        {
            Body body = CreateBody(world);
            FixtureFactory.AttachSolidArc(density, radians, sides, radius, position, angle, body);
            return body;
        }
    }
}