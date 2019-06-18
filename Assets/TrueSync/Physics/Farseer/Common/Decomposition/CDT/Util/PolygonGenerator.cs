/* Poly2Tri
 * Copyright (c) 2009-2010, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

using System;
using TrueSync.Physics2D;
using FP = TrueSync.FP;
using TSRandom = TrueSync.TSRandom;

namespace TrueSync.Physics2D
{
    internal class PolygonGenerator
    {
        private static readonly TSRandom RNG = TSRandom.New(0);

        private static FP PI_2 = 2.0* FP.Pi;

        public static Polygon RandomCircleSweep(FP scale, int vertexCount)
        {
            PolygonPoint point;
            PolygonPoint[] points;
            FP radius = scale/4;

            points = new PolygonPoint[vertexCount];
            for (int i = 0; i < vertexCount; i++)
            {
                do
                {
                    if (i%250 == 0)
                    {
                        radius += scale/2*(0.5 - RNG.NextFP());
                    }
                    else if (i%50 == 0)
                    {
                        radius += scale/5*(0.5 - RNG.NextFP());
                    }
                    else
                    {
                        radius += 25*scale/vertexCount*(0.5 - RNG.NextFP());
                    }
                    radius = radius > scale/2 ? scale/2 : radius;
                    radius = radius < scale/10 ? scale/10 : radius;
                } while (radius < scale/10 || radius > scale/2);
                point = new PolygonPoint(radius*FP.Cos((PI_2*i)/vertexCount),
                                         radius*FP.Sin((PI_2*i)/vertexCount));
                points[i] = point;
            }
            return new Polygon(points);
        }

        public static Polygon RandomCircleSweep2(FP scale, int vertexCount)
        {
            PolygonPoint point;
            PolygonPoint[] points;
            FP radius = scale/4;

            points = new PolygonPoint[vertexCount];
            for (int i = 0; i < vertexCount; i++)
            {
                do
                {
                    radius += scale/5*(0.5 - RNG.NextFP());
                    radius = radius > scale/2 ? scale/2 : radius;
                    radius = radius < scale/10 ? scale/10 : radius;
                } while (radius < scale/10 || radius > scale/2);
                point = new PolygonPoint(radius* FP.Cos((PI_2*i)/vertexCount),
                                         radius* FP.Sin((PI_2*i)/vertexCount));
                points[i] = point;
            }
            return new Polygon(points);
        }
    }
}