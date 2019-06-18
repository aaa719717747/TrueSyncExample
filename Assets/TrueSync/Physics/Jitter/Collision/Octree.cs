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
using System;
using System.Collections.Generic;
#endregion

namespace TrueSync.Physics3D {

    /// <summary>
    /// structure used to set up the mesh
    /// </summary>
    #region public struct TriangleVertexIndices
    public struct TriangleVertexIndices
    {
        /// <summary>
        /// The first index.
        /// </summary>
        public int I0;
        /// <summary>
        /// The second index.
        /// </summary>
        public int I1;
        /// <summary>
        /// The third index.
        /// </summary>
        public int I2;

        /// <summary>
        /// Initializes a new instance of the TriangleVertexIndex structure.
        /// </summary>
        /// <param name="i0">The index of the first vertex.</param>
        /// <param name="i1">The index of the second vertex.</param>
        /// <param name="i2">The index of the third vertex.</param>
        public TriangleVertexIndices(int i0, int i1, int i2)
        {
            this.I0 = i0;
            this.I1 = i1;
            this.I2 = i2;
        }

        /// <summary>
        /// Sets the values for the indices.
        /// </summary>
        /// <param name="i0">The index of the first vertex.</param>
        /// <param name="i1">The index of the second vertex.</param>
        /// <param name="i2">The index of the third vertex.</param>
        public void Set(int i0, int i1, int i2)
        {
            I0 = i0; I1 = i1; I2 = i2;
        }
    }
    #endregion


    /// <summary>
    /// An octree implementation.
    /// </summary>
    public class Octree
    {
        /// <summary>
        /// endices into the children - P means "plus" and M means "minus" and the
        /// letters are xyz. So PPM means +ve x, +ve y, -ve z
        /// </summary>
        [Flags]
        private enum EChild
        {
            XP = 0x1,
            YP = 0x2,
            ZP = 0x4,
            PPP = XP | YP | ZP,
            PPM = XP | YP,
            PMP = XP | ZP,
            PMM = XP,
            MPP = YP | ZP,
            MPM = YP,
            MMP = ZP,
            MMM = 0x0,
        }

        private struct Node
        {
            public UInt16[] nodeIndices;
            public int[] triIndices;
            public TSBBox box;
        }

        private class BuildNode
        {
            public int childType; // will default to MMM (usually ECHild but can also be -1)
            public List<int> nodeIndices = new List<int>();
            public List<int> triIndices = new List<int>();
            public TSBBox box;
        }

        private TSVector[] positions;
        private TSBBox[] triBoxes;
        private Node[] nodes;
        //private UInt16[] nodeStack;
        internal TriangleVertexIndices[] tris;
        internal TSBBox rootNodeBox;

        /// <summary>
        /// Gets the root node box containing the whole octree.
        /// </summary>
        public TSBBox RootNodeBox { get { return rootNodeBox; } }
       
        /// <summary>
        /// Clears the octree.
        /// </summary>
        #region public void Clear()
        public void Clear()
        {
            positions = null;
            triBoxes = null;
            tris = null;
            nodes = null;
            nodeStackPool.ResetResourcePool();
        }
        #endregion

        /// <summary>
        /// Sets new triangles.
        /// </summary>
        /// <param name="positions">Vertices.</param>
        /// <param name="tris">Indices.</param>
        #region public void AddTriangles(List<JVector> positions, List<TriangleVertexIndices> tris)
        public void SetTriangles(List<TSVector> positions, List<TriangleVertexIndices> tris)
        {
            // copy the position data into a array
            this.positions = new TSVector[positions.Count];
            positions.CopyTo(this.positions);

            // copy the triangles
            this.tris = new TriangleVertexIndices[tris.Count];
            tris.CopyTo(this.tris);
        }
        #endregion

        /// <summary>
        /// Builds the octree.
        /// </summary>
        #region public void BuildOctree()
        public void BuildOctree()
        {
            // create tri and tri bounding box arrays
            triBoxes = new TSBBox[tris.Length];

            // create an infinite size root box
            rootNodeBox = new TSBBox(new TSVector(FP.PositiveInfinity, FP.PositiveInfinity, FP.PositiveInfinity),
                                           new TSVector(FP.NegativeInfinity, FP.NegativeInfinity, FP.NegativeInfinity));


            for (int i = 0; i < tris.Length; i++)
            {
                TSVector.Min(ref positions[tris[i].I1], ref positions[tris[i].I2], out triBoxes[i].min);
                TSVector.Min(ref positions[tris[i].I0], ref triBoxes[i].min, out triBoxes[i].min);

                TSVector.Max(ref positions[tris[i].I1], ref positions[tris[i].I2], out triBoxes[i].max);
                TSVector.Max(ref positions[tris[i].I0], ref triBoxes[i].max, out triBoxes[i].max);

                // get size of the root box
                TSVector.Min(ref rootNodeBox.min, ref triBoxes[i].min, out rootNodeBox.min);
                TSVector.Max(ref rootNodeBox.max, ref triBoxes[i].max, out rootNodeBox.max);
            }

            List<BuildNode> buildNodes = new List<BuildNode>();
            buildNodes.Add(new BuildNode());
            buildNodes[0].box = rootNodeBox;

            TSBBox[] children = new TSBBox[8];
            for (int triNum = 0; triNum < tris.Length; triNum++)
            {
                int nodeIndex = 0;
                TSBBox box = rootNodeBox;

                while (box.Contains(ref triBoxes[triNum]) == TSBBox.ContainmentType.Contains)
                {
                    int childCon = -1;
                    for (int i = 0; i < 8; ++i)
                    {
                        CreateAABox(ref box, (EChild)i,out children[i]);
                        if (children[i].Contains(ref triBoxes[triNum]) == TSBBox.ContainmentType.Contains)
                        {
                            // this box contains the tri, it can be the only one that does,
                            // so we can stop our child search now and recurse into it
                            childCon = i;
                            break;
                        }
                    }

                    // no child contains this tri completely, so it belong in this node
                    if (childCon == -1)
                    {
                        buildNodes[nodeIndex].triIndices.Add(triNum);
                        break;
                    }
                    else
                    {
                        // do we already have this child
                        int childIndex = -1;
                        for (int index = 0; index < buildNodes[nodeIndex].nodeIndices.Count; ++index)
                        {
                            if (buildNodes[buildNodes[nodeIndex].nodeIndices[index]].childType == childCon)
                            {
                                childIndex = index;
                                break;
                            }
                        }
                        if (childIndex == -1)
                        {
                            // nope create child
                            BuildNode parentNode = buildNodes[nodeIndex];
                            BuildNode newNode = new BuildNode();
                            newNode.childType = childCon;
                            newNode.box = children[childCon];
                            buildNodes.Add(newNode);

                            nodeIndex = buildNodes.Count - 1;
                            box = children[childCon];
                            parentNode.nodeIndices.Add(nodeIndex);
                        }
                        else
                        {
                            nodeIndex = buildNodes[nodeIndex].nodeIndices[childIndex];
                            box = children[childCon];
                        }
                    }
                }
            }

            // now convert to the tighter Node from BuildNodes
            nodes = new Node[buildNodes.Count];
            nodeStackPool = new ArrayResourcePool<ushort>(buildNodes.Count);
            //nodeStack = new UInt16[buildNodes.Count];
            for (int i = 0; i < nodes.Length; i++)
            {
                nodes[i].nodeIndices = new UInt16[buildNodes[i].nodeIndices.Count];
                for (int index = 0; index < nodes[i].nodeIndices.Length; ++index)
                {
                    nodes[i].nodeIndices[index] = (UInt16)buildNodes[i].nodeIndices[index];
                }

                nodes[i].triIndices = new int[buildNodes[i].triIndices.Count];
                buildNodes[i].triIndices.CopyTo(nodes[i].triIndices);
                nodes[i].box = buildNodes[i].box;
            }
            buildNodes.Clear(); buildNodes = null;
        }
        #endregion

        /// <summary>
        /// Initializes a new instance of the Octree class.
        /// </summary>
        /// <param name="positions">Vertices.</param>
        /// <param name="tris">Indices.</param>
        #region Constructor
        public Octree(List<TSVector> positions, List<TriangleVertexIndices> tris)
        {
            SetTriangles(positions, tris);
            BuildOctree();
        }
        #endregion

        /// <summary>
        /// Create a bounding box appropriate for a child, based on a parents AABox
        /// </summary>
        /// <param name="aabb"></param>
        /// <param name="child"></param>
        /// <param name="result"></param>
        #region  private void CreateAABox(ref JBBox aabb, EChild child,out JBBox result)
        private void CreateAABox(ref TSBBox aabb, EChild child,out TSBBox result)
        {
            TSVector dims;
            TSVector.Subtract(ref aabb.max, ref aabb.min, out dims);
            TSVector.Multiply(ref dims, FP.Half, out dims);

            TSVector offset = TSVector.zero;

            switch (child)
            {
                case EChild.PPP: offset = new TSVector(1, 1, 1); break;
                case EChild.PPM: offset = new TSVector(1, 1, 0); break;
                case EChild.PMP: offset = new TSVector(1, 0, 1); break;
                case EChild.PMM: offset = new TSVector(1, 0, 0); break;
                case EChild.MPP: offset = new TSVector(0, 1, 1); break;
                case EChild.MPM: offset = new TSVector(0, 1, 0); break;
                case EChild.MMP: offset = new TSVector(0, 0, 1); break;
                case EChild.MMM: offset = new TSVector(0, 0, 0); break;

                default:
                    System.Diagnostics.Debug.WriteLine("Octree.CreateAABox  got impossible child");
                    break;
            }

            result = new TSBBox();
            result.min = new TSVector(offset.x * dims.x, offset.y * dims.y, offset.z * dims.z);
            TSVector.Add(ref result.min, ref aabb.min, out result.min);

            TSVector.Add(ref result.min, ref dims, out result.max);

            // expand it just a tiny bit just to be safe!
            FP extra = FP.EN5;

            TSVector temp; TSVector.Multiply(ref dims, extra, out temp);
            TSVector.Subtract(ref result.min, ref temp, out result.min);
            TSVector.Add(ref result.max, ref temp, out result.max);
        }
        #endregion

        #region private void GatherTriangles(int nodeIndex, ref List<int> tris)
        private void GatherTriangles(int nodeIndex, ref List<int> tris)
        {
            // add this nodes triangles
            tris.AddRange(nodes[nodeIndex].triIndices);

            // recurse into this nodes children
            int numChildren = nodes[nodeIndex].nodeIndices.Length;
            for (int i = 0; i < numChildren; ++i)
            {
                int childNodeIndex = nodes[nodeIndex].nodeIndices[i];
                GatherTriangles(childNodeIndex, ref tris);
            }
        }
        #endregion


        /// <summary>
        /// Returns all triangles which intersect the given axis aligned bounding box.
        /// </summary>
        /// <param name="triangles">The list to add the triangles to.</param>
        /// <param name="testBox">The axis alignes bounding box.</param>
        /// <returns></returns>
        #region public int GetTrianglesIntersectingtAABox(List<int> triangles, ref JBBox testBox)
        public int GetTrianglesIntersectingtAABox(List<int> triangles, ref TSBBox testBox)
        {
            if (nodes.Length == 0)
                return 0;
            int curStackIndex = 0;
            int endStackIndex = 1;

            UInt16[] nodeStack = nodeStackPool.GetNew();

            nodeStack[0] = 0;

            int triCount = 0;

            while (curStackIndex < endStackIndex)
            {
                UInt16 nodeIndex = nodeStack[curStackIndex];
                curStackIndex++;
                if (nodes[nodeIndex].box.Contains(ref testBox) != TSBBox.ContainmentType.Disjoint)
                {
                    for (int i = 0; i < nodes[nodeIndex].triIndices.Length; ++i)
                    {
                        if (triBoxes[nodes[nodeIndex].triIndices[i]].Contains(ref testBox) != TSBBox.ContainmentType.Disjoint)
                        {
                            triangles.Add(nodes[nodeIndex].triIndices[i]);
                            triCount++;
                        }
                    }

                    int numChildren = nodes[nodeIndex].nodeIndices.Length;
                    for (int i = 0; i < numChildren; ++i)
                    {
                        nodeStack[endStackIndex++] = nodes[nodeIndex].nodeIndices[i];
                    }
                }
            }

            nodeStackPool.GiveBack(nodeStack);

            return triCount;
        }
        #endregion

        private ArrayResourcePool<UInt16> nodeStackPool;

        /// <summary>
        /// Returns all triangles which intersect the given axis aligned bounding box.
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayDelta"></param>
        /// <param name="triangles"></param>
        /// <returns></returns>
        #region public int GetTrianglesIntersectingtRay(JVector rayOrigin, JVector rayDelta)
        public int GetTrianglesIntersectingRay(List<int> triangles, TSVector rayOrigin, TSVector rayDelta)
        {
            if (nodes.Length == 0)
                return 0;
            int curStackIndex = 0;
            int endStackIndex = 1;

            UInt16[] nodeStack = nodeStackPool.GetNew();
            nodeStack[0] = 0;

            int triCount = 0;

            while (curStackIndex < endStackIndex)
            {
                UInt16 nodeIndex = nodeStack[curStackIndex];
                curStackIndex++;
                if (nodes[nodeIndex].box.SegmentIntersect(ref rayOrigin, ref rayDelta))
                {
                    for (int i = 0; i < nodes[nodeIndex].triIndices.Length; ++i)
                    {
                        if (triBoxes[nodes[nodeIndex].triIndices[i]].SegmentIntersect(ref rayOrigin, ref rayDelta))
                        {
                            triangles.Add(nodes[nodeIndex].triIndices[i]);
                            triCount++;
                        }
                    }

                    int numChildren = nodes[nodeIndex].nodeIndices.Length;
                    for (int i = 0; i < numChildren; ++i)
                    {
                        nodeStack[endStackIndex++] = nodes[nodeIndex].nodeIndices[i];
                    }
                }
            }

            nodeStackPool.GiveBack(nodeStack);
            return triCount;
        }
        #endregion

        /// <summary>
        /// Gets the indices of a triangle by index.
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns>The indices of a triangle.</returns>
        #region public TriangleVertexIndices GetTriangleVertexIndex(int index)
        public TriangleVertexIndices GetTriangleVertexIndex(int index)
        {
            return tris[index];
        }
        #endregion

        /// <summary>
        /// Gets a vertex from the vertex list.
        /// </summary>
        /// <param name="vertex">The index of the vertex</param>
        /// <returns></returns>
        #region public JVector GetVertex(int vertex)
        public TSVector GetVertex(int vertex)
        {
            return positions[vertex];
        }

        /// <summary>
        /// Gets a vertex from the vertex list.
        /// </summary>
        /// <param name="vertex">The index of the vertex</param>
        /// <param name="result"></param>
        public void GetVertex(int vertex, out TSVector result)
        {
            result = positions[vertex];
        }
        #endregion

        /// <summary>
        /// Gets the number of triangles within this octree.
        /// </summary>
        #region public int NumTriangles
        public int NumTriangles
        {
            get { return tris.Length; }
        }
        #endregion


    }
}
