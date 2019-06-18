using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;

namespace TrueSync.Physics2D
{
    /// <summary>
    /// An easy to use factory for creating bodies
    /// </summary>
    public static class FixtureFactory
    {
        public static Fixture AttachEdge(TSVector2 start, TSVector2 end, Body body, object userData = null)
        {
            EdgeShape edgeShape = new EdgeShape(start, end);
            return body.CreateFixture(edgeShape, userData);
        }

        public static Fixture AttachChainShape(Vertices vertices, Body body, object userData = null)
        {
            ChainShape shape = new ChainShape(vertices);
            return body.CreateFixture(shape, userData);
        }

        public static Fixture AttachLoopShape(Vertices vertices, Body body, object userData = null)
        {
            ChainShape shape = new ChainShape(vertices, true);
            return body.CreateFixture(shape, userData);
        }

        public static Fixture AttachRectangle(FP width, FP height, FP density, TSVector2 offset, Body body, object userData = null)
        {
            Vertices rectangleVertices = PolygonTools.CreateRectangle(width / 2, height / 2);
            rectangleVertices.Translate(ref offset);
            PolygonShape rectangleShape = new PolygonShape(rectangleVertices, density);
            return body.CreateFixture(rectangleShape, userData);
        }

        public static Fixture AttachCircle(FP radius, FP density, Body body, object userData = null)
        {
            if (radius <= 0)
                throw new ArgumentOutOfRangeException("radius", "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius, density);
            return body.CreateFixture(circleShape, userData);
        }

        public static Fixture AttachCircle(FP radius, FP density, Body body, TSVector2 offset, object userData = null)
        {
            if (radius <= 0)
                throw new ArgumentOutOfRangeException("radius", "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius, density);
            circleShape.Position = offset;
            return body.CreateFixture(circleShape, userData);
        }

        public static Fixture AttachPolygon(Vertices vertices, FP density, Body body, object userData = null)
        {
            if (vertices.Count <= 1)
                throw new ArgumentOutOfRangeException("vertices", "Too few points to be a polygon");

            PolygonShape polygon = new PolygonShape(vertices, density);
            return body.CreateFixture(polygon, userData);
        }

        public static Fixture AttachEllipse(FP xRadius, FP yRadius, int edges, FP density, Body body, object userData = null)
        {
            if (xRadius <= 0)
                throw new ArgumentOutOfRangeException("xRadius", "X-radius must be more than 0");

            if (yRadius <= 0)
                throw new ArgumentOutOfRangeException("yRadius", "Y-radius must be more than 0");

            Vertices ellipseVertices = PolygonTools.CreateEllipse(xRadius, yRadius, edges);
            PolygonShape polygonShape = new PolygonShape(ellipseVertices, density);
            return body.CreateFixture(polygonShape, userData);
        }

        public static List<Fixture> AttachCompoundPolygon(List<Vertices> list, FP density, Body body, object userData = null)
        {
            List<Fixture> res = new List<Fixture>(list.Count);

            //Then we create several fixtures using the body
            foreach (Vertices vertices in list)
            {
                if (vertices.Count == 2)
                {
                    EdgeShape shape = new EdgeShape(vertices[0], vertices[1]);
                    res.Add(body.CreateFixture(shape, userData));
                }
                else
                {
                    PolygonShape shape = new PolygonShape(vertices, density);
                    res.Add(body.CreateFixture(shape, userData));
                }
            }

            return res;
        }

        public static Fixture AttachLineArc(FP radians, int sides, FP radius, TSVector2 position, FP angle, bool closed, Body body)
        {
            Vertices arc = PolygonTools.CreateArc(radians, sides, radius);
            arc.Rotate((FP.Pi - radians) / 2 + angle);
            arc.Translate(ref position);

            return closed ? AttachLoopShape(arc, body) : AttachChainShape(arc, body);
        }

        public static List<Fixture> AttachSolidArc(FP density, FP radians, int sides, FP radius, TSVector2 position, FP angle, Body body)
        {
            Vertices arc = PolygonTools.CreateArc(radians, sides, radius);
            arc.Rotate((FP.Pi - radians) / 2 + angle);

            arc.Translate(ref position);

            //Close the arc
            arc.Add(arc[0]);

            List<Vertices> triangles = Triangulate.ConvexPartition(arc, TriangulationAlgorithm.Earclip, true, FP.EN3);

            return AttachCompoundPolygon(triangles, density, body);
        }
    }
}