using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Xna.Framework;

namespace TrueSync.Physics2D
{
    public static class PolygonTools
    {

        public static void TransformVertices(Vertices vertices, TSVector2 center, FP angle) {
            Transform xf = new Transform();
            xf.p = center;
            xf.q.Set(angle);

            // Transform vertices
            for (int i = 0; i < vertices.Count; ++i) {
                vertices[i] = MathUtils.Mul(ref xf, vertices[i]) - center;
            }
        }

        /// <summary>
        /// Build vertices to represent an axis-aligned box.
        /// </summary>
        /// <param name="hx">the half-width.</param>
        /// <param name="hy">the half-height.</param>
        public static Vertices CreateRectangle(FP hx, FP hy)
        {
            Vertices vertices = new Vertices(4);
            vertices.Add(new TSVector2(-hx, -hy));
            vertices.Add(new TSVector2(hx, -hy));
            vertices.Add(new TSVector2(hx, hy));
            vertices.Add(new TSVector2(-hx, hy));

            return vertices;
        }

        /// <summary>
        /// Build vertices to represent an oriented box.
        /// </summary>
        /// <param name="hx">the half-width.</param>
        /// <param name="hy">the half-height.</param>
        /// <param name="center">the center of the box in local coordinates.</param>
        /// <param name="angle">the rotation of the box in local coordinates.</param>
        public static Vertices CreateRectangle(FP hx, FP hy, TSVector2 center, FP angle)
        {
            Vertices vertices = CreateRectangle(hx, hy);

            Transform xf = new Transform();
            xf.p = center;
            xf.q.Set(angle);

            // Transform vertices
            for (int i = 0; i < 4; ++i)
            {
                vertices[i] = MathUtils.Mul(ref xf, vertices[i]) - center;
            }

            return vertices;
        }

        //Rounded rectangle contributed by Jonathan Smars - jsmars@gmail.com

        /// <summary>
        /// Creates a rounded rectangle with the specified width and height.
        /// </summary>
        /// <param name="width">The width.</param>
        /// <param name="height">The height.</param>
        /// <param name="xRadius">The rounding X radius.</param>
        /// <param name="yRadius">The rounding Y radius.</param>
        /// <param name="segments">The number of segments to subdivide the edges.</param>
        /// <returns></returns>
        public static Vertices CreateRoundedRectangle(FP width, FP height, FP xRadius, FP yRadius,
                                                      int segments)
        {
            if (yRadius > height / 2 || xRadius > width / 2)
                throw new Exception("Rounding amount can't be more than half the height and width respectively.");
            if (segments < 0)
                throw new Exception("Segments must be zero or more.");

            //We need at least 8 vertices to create a rounded rectangle
            Debug.Assert(Settings.MaxPolygonVertices >= 8);

            Vertices vertices = new Vertices();
            if (segments == 0)
            {
                vertices.Add(new TSVector2(width * .5f - xRadius, -height * .5f));
                vertices.Add(new TSVector2(width * .5f, -height * .5f + yRadius));

                vertices.Add(new TSVector2(width * .5f, height * .5f - yRadius));
                vertices.Add(new TSVector2(width * .5f - xRadius, height * .5f));

                vertices.Add(new TSVector2(-width * .5f + xRadius, height * .5f));
                vertices.Add(new TSVector2(-width * .5f, height * .5f - yRadius));

                vertices.Add(new TSVector2(-width * .5f, -height * .5f + yRadius));
                vertices.Add(new TSVector2(-width * .5f + xRadius, -height * .5f));
            }
            else
            {
                int numberOfEdges = (segments * 4 + 8);

                FP stepSize = FP.PiTimes2 / (numberOfEdges - 4);
                int perPhase = numberOfEdges / 4;

                TSVector2 posOffset = new TSVector2(width / 2 - xRadius, height / 2 - yRadius);
                vertices.Add(posOffset + new TSVector2(xRadius, -yRadius + yRadius));
                short phase = 0;
                for (int i = 1; i < numberOfEdges; i++)
                {
                    if (i - perPhase == 0 || i - perPhase * 3 == 0)
                    {
                        posOffset.x *= -1;
                        phase--;
                    }
                    else if (i - perPhase * 2 == 0)
                    {
                        posOffset.y *= -1;
                        phase--;
                    }

                    vertices.Add(posOffset + new TSVector2(xRadius * FP.Cos(stepSize * -(i + phase)),
                                                         -yRadius * FP.Sin(stepSize * -(i + phase))));
                }
            }

            return vertices;
        }

        /// <summary>
        /// Set this as a single edge.
        /// </summary>
        /// <param name="start">The first point.</param>
        /// <param name="end">The second point.</param>
        public static Vertices CreateLine(TSVector2 start, TSVector2 end)
        {
            Vertices vertices = new Vertices(2);
            vertices.Add(start);
            vertices.Add(end);

            return vertices;
        }

        /// <summary>
        /// Creates a circle with the specified radius and number of edges.
        /// </summary>
        /// <param name="radius">The radius.</param>
        /// <param name="numberOfEdges">The number of edges. The more edges, the more it resembles a circle</param>
        /// <returns></returns>
        public static Vertices CreateCircle(FP radius, int numberOfEdges)
        {
            return CreateEllipse(radius, radius, numberOfEdges);
        }

        /// <summary>
        /// Creates a ellipse with the specified width, height and number of edges.
        /// </summary>
        /// <param name="xRadius">Width of the ellipse.</param>
        /// <param name="yRadius">Height of the ellipse.</param>
        /// <param name="numberOfEdges">The number of edges. The more edges, the more it resembles an ellipse</param>
        /// <returns></returns>
        public static Vertices CreateEllipse(FP xRadius, FP yRadius, int numberOfEdges)
        {
            Vertices vertices = new Vertices();

            FP stepSize = FP.PiTimes2 / numberOfEdges;

            vertices.Add(new TSVector2(xRadius, 0));
            for (int i = numberOfEdges - 1; i > 0; --i)
                vertices.Add(new TSVector2(xRadius * FP.Cos(stepSize * i),
                                         -yRadius * FP.Sin(stepSize * i)));

            return vertices;
        }

        public static Vertices CreateArc(FP radians, int sides, FP radius)
        {
            Debug.Assert(radians > 0, "The arc needs to be larger than 0");
            Debug.Assert(sides > 1, "The arc needs to have more than 1 sides");
            Debug.Assert(radius > 0, "The arc needs to have a radius larger than 0");

            Vertices vertices = new Vertices();

            FP stepSize = radians / sides;
            for (int i = sides - 1; i > 0; i--)
            {
                vertices.Add(new TSVector2(radius * FP.Cos(stepSize * i),
                                         radius * FP.Sin(stepSize * i)));
            }

            return vertices;
        }

        //Capsule contributed by Yobiv

        /// <summary>
        /// Creates an capsule with the specified height, radius and number of edges.
        /// A capsule has the same form as a pill capsule.
        /// </summary>
        /// <param name="height">Height (inner height + 2 * radius) of the capsule.</param>
        /// <param name="endRadius">Radius of the capsule ends.</param>
        /// <param name="edges">The number of edges of the capsule ends. The more edges, the more it resembles an capsule</param>
        /// <returns></returns>
        public static Vertices CreateCapsule(FP height, FP endRadius, int edges)
        {
            if (endRadius >= height / 2)
                throw new ArgumentException(
                    "The radius must be lower than height / 2. Higher values of radius would create a circle, and not a half circle.",
                    "endRadius");

            return CreateCapsule(height, endRadius, edges, endRadius, edges);
        }

        /// <summary>
        /// Creates an capsule with the specified  height, radius and number of edges.
        /// A capsule has the same form as a pill capsule.
        /// </summary>
        /// <param name="height">Height (inner height + radii) of the capsule.</param>
        /// <param name="topRadius">Radius of the top.</param>
        /// <param name="topEdges">The number of edges of the top. The more edges, the more it resembles an capsule</param>
        /// <param name="bottomRadius">Radius of bottom.</param>
        /// <param name="bottomEdges">The number of edges of the bottom. The more edges, the more it resembles an capsule</param>
        /// <returns></returns>
        public static Vertices CreateCapsule(FP height, FP topRadius, int topEdges, FP bottomRadius,
                                             int bottomEdges)
        {
            if (height <= 0)
                throw new ArgumentException("Height must be longer than 0", "height");

            if (topRadius <= 0)
                throw new ArgumentException("The top radius must be more than 0", "topRadius");

            if (topEdges <= 0)
                throw new ArgumentException("Top edges must be more than 0", "topEdges");

            if (bottomRadius <= 0)
                throw new ArgumentException("The bottom radius must be more than 0", "bottomRadius");

            if (bottomEdges <= 0)
                throw new ArgumentException("Bottom edges must be more than 0", "bottomEdges");

            if (topRadius >= height / 2)
                throw new ArgumentException(
                    "The top radius must be lower than height / 2. Higher values of top radius would create a circle, and not a half circle.",
                    "topRadius");

            if (bottomRadius >= height / 2)
                throw new ArgumentException(
                    "The bottom radius must be lower than height / 2. Higher values of bottom radius would create a circle, and not a half circle.",
                    "bottomRadius");

            Vertices vertices = new Vertices();

            FP newHeight = (height - topRadius - bottomRadius) * 0.5f;

            // top
            vertices.Add(new TSVector2(topRadius, newHeight));

            FP stepSize = FP.Pi / topEdges;
            for (int i = 1; i < topEdges; i++)
            {
                vertices.Add(new TSVector2(topRadius * FP.Cos(stepSize * i),
                                         topRadius * FP.Sin(stepSize * i) + newHeight));
            }

            vertices.Add(new TSVector2(-topRadius, newHeight));

            // bottom
            vertices.Add(new TSVector2(-bottomRadius, -newHeight));

            stepSize = FP.Pi / bottomEdges;
            for (int i = 1; i < bottomEdges; i++)
            {
                vertices.Add(new TSVector2(-bottomRadius * FP.Cos(stepSize * i),
                                         -bottomRadius * FP.Sin(stepSize * i) - newHeight));
            }

            vertices.Add(new TSVector2(bottomRadius, -newHeight));

            return vertices;
        }

        /// <summary>
        /// Creates a gear shape with the specified radius and number of teeth.
        /// </summary>
        /// <param name="radius">The radius.</param>
        /// <param name="numberOfTeeth">The number of teeth.</param>
        /// <param name="tipPercentage">The tip percentage.</param>
        /// <param name="toothHeight">Height of the tooth.</param>
        /// <returns></returns>
        public static Vertices CreateGear(FP radius, int numberOfTeeth, FP tipPercentage, FP toothHeight)
        {
            Vertices vertices = new Vertices();

            FP stepSize = FP.PiTimes2 / numberOfTeeth;
            tipPercentage /= 100f;
            TSMath.Clamp(tipPercentage, 0f, 1f);
            FP toothTipStepSize = (stepSize / 2f) * tipPercentage;

            FP toothAngleStepSize = (stepSize - (toothTipStepSize * 2f)) / 2f;

            for (int i = numberOfTeeth - 1; i >= 0; --i)
            {
                if (toothTipStepSize > 0f)
                {
                    vertices.Add(
                        new TSVector2(radius *
                                    FP.Cos(stepSize * i + toothAngleStepSize * 2f + toothTipStepSize),
                                    -radius *
                                    FP.Sin(stepSize * i + toothAngleStepSize * 2f + toothTipStepSize)));

                    vertices.Add(
                        new TSVector2((radius + toothHeight) *
                                    FP.Cos(stepSize * i + toothAngleStepSize + toothTipStepSize),
                                    -(radius + toothHeight) *
                                    FP.Sin(stepSize * i + toothAngleStepSize + toothTipStepSize)));
                }

                vertices.Add(new TSVector2((radius + toothHeight) *
                                         FP.Cos(stepSize * i + toothAngleStepSize),
                                         -(radius + toothHeight) *
                                         FP.Sin(stepSize * i + toothAngleStepSize)));

                vertices.Add(new TSVector2(radius * FP.Cos(stepSize * i),
                                         -radius * FP.Sin(stepSize * i)));
            }

            return vertices;
        }

    }

}