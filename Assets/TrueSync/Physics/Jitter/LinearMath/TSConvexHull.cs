/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System.Collections.Generic;
#endregion

namespace TrueSync.Physics3D {

    /// <summary>
    /// Fast but dirty convex hull creation.
    /// advanced convex hull creation: http://www.qhull.org
    /// </summary>
    public static class TSConvexHull
    {
        #region public enum Approximation
        public enum Approximation
        {
            Level1 = 6,
            Level2 = 7,
            Level3 = 8,
            Level4 = 9,
            Level5 = 10,
            Level6 = 11,
            Level7 = 12,
            Level8 = 13,
            Level9 = 15,
            Level10 = 20,
            Level15 = 25,
            Level20 = 30
        }
        #endregion

        public static int[] Build(List<TSVector> pointCloud, Approximation factor)
        {
            List<int> allIndices = new List<int>();

            int steps = (int)factor;

            for (int thetaIndex = 0; thetaIndex < steps; thetaIndex++)
            {
                // [0,PI]
                FP theta = TSMath.Pi / (steps - 1) * thetaIndex;
                FP sinTheta = FP.Sin(theta);
                FP cosTheta = FP.Cos(theta);

                for (int phiIndex = 0; phiIndex < steps; phiIndex++)
                {
                    // [-PI,PI]
                    FP phi =  ((2 * FP.One) * TSMath.Pi) / (steps - 0) * phiIndex - TSMath.Pi;
                    FP sinPhi = FP.Sin(phi);
                    FP cosPhi = FP.Cos(phi);

                    TSVector dir = new TSVector(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);

                    int index = FindExtremePoint(pointCloud, ref dir);
                    allIndices.Add(index);
                }
            }

            allIndices.Sort();

            for (int i = 1; i < allIndices.Count; i++)
            {
                if (allIndices[i - 1] == allIndices[i])
                { allIndices.RemoveAt(i - 1); i--; }
            }

            return allIndices.ToArray();

            // or using 3.5 extensions
            // return allIndices.Distinct().ToArray();
        }

        private static int FindExtremePoint(List<TSVector> points,ref TSVector dir)
        {
            int index = 0;
            FP current = FP.MinValue;

            TSVector point; FP value;

            for (int i = 1; i < points.Count; i++)
            {
                point = points[i];

                value = TSVector.Dot(ref point, ref dir);
                if (value > current) { current = value; index= i; }
            }

            return index;
        }
    }
}
