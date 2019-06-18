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
using System.Collections.ObjectModel;
#endregion

namespace TrueSync.Physics3D {
	public partial class SoftBody : IBroadphaseEntity, IComparable
    {
        [Flags]
        public enum SpringType
        {
            EdgeSpring = 0x02, ShearSpring = 0x04, BendSpring = 0x08
        }

		public int CompareTo(object otherObj)
		{
            SoftBody other = (SoftBody) otherObj;

			if (other.volume < this.volume) return -1;
			else if (other.volume > this.volume) return 1;
			else return 0;
		}

        #region public class Spring : Constraint
        public class Spring : Constraint
        {
            public enum DistanceBehavior
            {
                LimitDistance,
                LimitMaximumDistance,
                LimitMinimumDistance,
            }

            public SpringType SpringType { get; set; }

            private FP biasFactor = FP.EN1;
            private FP softness = FP.EN2;
            private FP distance;

            private DistanceBehavior behavior = DistanceBehavior.LimitDistance;

            /// <summary>
            /// Initializes a new instance of the DistanceConstraint class.
            /// </summary>
            /// <param name="body1">The first body.</param>
            /// <param name="body2">The second body.</param>
            /// <param name="anchor1">The anchor point of the first body in world space. 
            /// The distance is given by the initial distance between both anchor points.</param>
            /// <param name="anchor2">The anchor point of the second body in world space.
            /// The distance is given by the initial distance between both anchor points.</param>
            public Spring(RigidBody body1, RigidBody body2)
                : base(body1, body2)
            {
                distance = (body1.position - body2.position).magnitude;
            }

            public FP AppliedImpulse { get { return accumulatedImpulse; } }

            /// <summary>
            /// 
            /// </summary>
            public FP Distance { get { return distance; } set { distance = value; } }

            /// <summary>
            /// 
            /// </summary>
            public DistanceBehavior Behavior { get { return behavior; } set { behavior = value; } }

            /// <summary>
            /// Defines how big the applied impulses can get.
            /// </summary>
            public FP Softness { get { return softness; } set { softness = value; } }

            /// <summary>
            /// Defines how big the applied impulses can get which correct errors.
            /// </summary>
            public FP BiasFactor { get { return biasFactor; } set { biasFactor = value; } }

            FP effectiveMass = FP.Zero;
            FP accumulatedImpulse = FP.Zero;
            FP bias;
            FP softnessOverDt;

            TSVector[] jacobian = new TSVector[2];

            bool skipConstraint = false;

            /// <summary>
            /// Called once before iteration starts.
            /// </summary>
            /// <param name="timestep">The 5simulation timestep</param>
            public override void PrepareForIteration(FP timestep)
            {
                TSVector dp;
                TSVector.Subtract(ref body2.position, ref body1.position, out dp);

                FP deltaLength = dp.magnitude - distance;

                if (behavior == DistanceBehavior.LimitMaximumDistance && deltaLength <= FP.Zero)
                {
                    skipConstraint = true;
                }
                else if (behavior == DistanceBehavior.LimitMinimumDistance && deltaLength >= FP.Zero)
                {
                    skipConstraint = true;
                }
                else
                {
                    skipConstraint = false;

                    TSVector n = dp;
                    if (n.sqrMagnitude != FP.Zero) n.Normalize();

                    jacobian[0] = -FP.One * n;
                    //jacobian[1] = -FP.One * (r1 % n);
                    jacobian[1] = FP.One * n;
                    //jacobian[3] = (r2 % n);

                    effectiveMass = body1.inverseMass + body2.inverseMass;

                    softnessOverDt = softness / timestep;
                    effectiveMass += softnessOverDt;

                    effectiveMass = FP.One / effectiveMass;

                    bias = deltaLength * biasFactor * (FP.One / timestep);

                    if (!body1.isStatic)
                    {
                        body1.linearVelocity += body1.inverseMass * accumulatedImpulse * jacobian[0];
                    }

                    if (!body2.isStatic)
                    {
                        body2.linearVelocity += body2.inverseMass * accumulatedImpulse * jacobian[1];
                    }
                }

            }

            /// <summary>
            /// Iteratively solve this constraint.
            /// </summary>
            public override void Iterate()
            {
                if (skipConstraint) return;

                FP jv = TSVector.Dot(ref body1.linearVelocity, ref jacobian[0]);
                jv += TSVector.Dot(ref body2.linearVelocity, ref jacobian[1]);

                FP softnessScalar = accumulatedImpulse * softnessOverDt;

                FP lambda = -effectiveMass * (jv + bias + softnessScalar);

                if (behavior == DistanceBehavior.LimitMinimumDistance)
                {
                    FP previousAccumulatedImpulse = accumulatedImpulse;
                    accumulatedImpulse = TSMath.Max(accumulatedImpulse + lambda, 0);
                    lambda = accumulatedImpulse - previousAccumulatedImpulse;
                }
                else if (behavior == DistanceBehavior.LimitMaximumDistance)
                {
                    FP previousAccumulatedImpulse = accumulatedImpulse;
                    accumulatedImpulse = TSMath.Min(accumulatedImpulse + lambda, 0);
                    lambda = accumulatedImpulse - previousAccumulatedImpulse;
                }
                else
                {
                    accumulatedImpulse += lambda;
                }

                TSVector temp;

                if (!body1.isStatic)
                {
                    TSVector.Multiply(ref jacobian[0], lambda * body1.inverseMass, out temp);
                    TSVector.Add(ref temp, ref body1.linearVelocity, out body1.linearVelocity);
                }

                if (!body2.isStatic)
                {
                    TSVector.Multiply(ref jacobian[1], lambda * body2.inverseMass, out temp);
                    TSVector.Add(ref temp, ref body2.linearVelocity, out body2.linearVelocity);
                }
            }

            public override void DebugDraw(IDebugDrawer drawer)
            {
                drawer.DrawLine(body1.position, body2.position);
            }

        }
        #endregion

        #region public class MassPoint : RigidBody
        public class MassPoint : RigidBody
        {
            public SoftBody SoftBody { get; private set; }

            public MassPoint(Shape shape, SoftBody owner, BodyMaterial material)
                : base(shape, material, true)
            {
                this.SoftBody = owner;
            }

        }
        #endregion

        #region public class Triangle : ISupportMappable
        public class Triangle : ISupportMappable
        {
            private SoftBody owner;

            public SoftBody Owner { get { return owner; } }

            internal TSBBox boundingBox;
            internal int dynamicTreeID;
            internal TriangleVertexIndices indices;


            public TSBBox BoundingBox { get { return boundingBox; } }
            public int DynamicTreeID { get { return dynamicTreeID; } }

            public TriangleVertexIndices Indices { get { return indices; } }

            public MassPoint VertexBody1 { get { return owner.points[indices.I0]; } }
            public MassPoint VertexBody2 { get { return owner.points[indices.I1]; } }
            public MassPoint VertexBody3 { get { return owner.points[indices.I2]; } }

            public Triangle(SoftBody owner)
            {
                this.owner = owner;
            }

            public void GetNormal(out TSVector normal)
            {
                TSVector sum;
                TSVector.Subtract(ref owner.points[indices.I1].position, ref owner.points[indices.I0].position, out sum);
                TSVector.Subtract(ref owner.points[indices.I2].position, ref owner.points[indices.I0].position, out normal);
                TSVector.Cross(ref sum, ref normal, out normal);
            }

            public void UpdateBoundingBox()
            {
                boundingBox = TSBBox.SmallBox;
                boundingBox.AddPoint(ref owner.points[indices.I0].position);
                boundingBox.AddPoint(ref owner.points[indices.I1].position);
                boundingBox.AddPoint(ref owner.points[indices.I2].position);

                boundingBox.min -= new TSVector(owner.triangleExpansion);
                boundingBox.max += new TSVector(owner.triangleExpansion);
            }

            public FP CalculateArea()
            {
                return ((owner.points[indices.I1].position - owner.points[indices.I0].position) %
                    (owner.points[indices.I2].position - owner.points[indices.I0].position)).magnitude;
            }

            public void SupportMapping(ref TSVector direction, out TSVector result)
            {

                FP min = TSVector.Dot(ref owner.points[indices.I0].position, ref direction);
                FP dot = TSVector.Dot(ref owner.points[indices.I1].position, ref direction);

                TSVector minVertex = owner.points[indices.I0].position;

                if (dot > min)
                {
                    min = dot;
                    minVertex = owner.points[indices.I1].position;
                }
                dot = TSVector.Dot(ref owner.points[indices.I2].position, ref direction);
                if (dot > min)
                {
                    min = dot;
                    minVertex = owner.points[indices.I2].position;
                }


                TSVector exp;
                TSVector.Normalize(ref direction, out exp);
                exp *= owner.triangleExpansion;
                result = minVertex + exp;


            }

            public void SupportCenter(out TSVector center)
            {
                center = owner.points[indices.I0].position;
                TSVector.Add(ref center, ref owner.points[indices.I1].position, out center);
                TSVector.Add(ref center, ref owner.points[indices.I2].position, out center);
                TSVector.Multiply(ref center, FP.One / (3 * FP.One), out center);
            }
        }
        #endregion

        private SphereShape sphere = new SphereShape(FP.EN1);

        protected List<Spring> springs = new List<Spring>();
        protected List<MassPoint> points = new List<MassPoint>();
        protected List<Triangle> triangles = new List<Triangle>();

        public ReadOnlyCollection<Spring> EdgeSprings { get; private set; }
        public ReadOnlyCollection<MassPoint> VertexBodies { get; private set; }
        public ReadOnlyCollection<Triangle> Triangles { private set; get; }

        protected FP triangleExpansion = FP.EN1;

        private bool selfCollision = false;

        public bool SelfCollision { get { return selfCollision; } set { selfCollision = value; } }

        public FP TriangleExpansion { get { return triangleExpansion; } 
            set { triangleExpansion = value; } }

        public FP VertexExpansion { get { return sphere.Radius; } set { sphere.Radius = value; } }

        private FP volume = FP.One;
        private FP mass = FP.One;

        internal DynamicTree<Triangle> dynamicTree = new DynamicTree<Triangle>();
        public DynamicTree<Triangle> DynamicTree { get { return dynamicTree; } }

        private BodyMaterial material = new BodyMaterial();
        public BodyMaterial Material { get { return material; } }

        TSBBox box = new TSBBox();

        bool active = true;


        /// <summary>
        /// Does create an empty body. Derive from SoftBody and fill 
        /// EdgeSprings,VertexBodies and Triangles by yourself.
        /// </summary>
        public SoftBody()
        {
        }

        /// <summary>
        /// Creates a 2D-Cloth. Connects Nearest Neighbours (4x, called EdgeSprings) and adds additional
        /// shear/bend constraints (4xShear+4xBend).
        /// </summary>
        /// <param name="sizeX"></param>
        /// <param name="sizeY"></param>
        /// <param name="scale"></param>
        public SoftBody(int sizeX,int sizeY, FP scale)
        {
            List<TriangleVertexIndices> indices = new List<TriangleVertexIndices>();
            List<TSVector> vertices = new List<TSVector>();

            for (int i = 0; i < sizeY; i++)
            {
                for (int e = 0; e < sizeX; e++)
                {
                    vertices.Add(new TSVector(i, 0, e) *scale);
                }
            }
            
            for (int i = 0; i < sizeX-1; i++)
            {
                for (int e = 0; e < sizeY-1; e++)
                {
                    TriangleVertexIndices index = new TriangleVertexIndices();
                    {

                        index.I0 = (e + 0) * sizeX + i + 0;
                        index.I1 = (e + 0) * sizeX + i + 1;
                        index.I2 = (e + 1) * sizeX + i + 1;

                        indices.Add(index);

                        index.I0 = (e + 0) * sizeX + i + 0;
                        index.I1 = (e + 1) * sizeX + i + 1;
                        index.I2 = (e + 1) * sizeX + i + 0;

                        indices.Add(index);    
                    }
                }
            }

            EdgeSprings = springs.AsReadOnly();
            VertexBodies = points.AsReadOnly();
            Triangles = triangles.AsReadOnly();

            AddPointsAndSprings(indices, vertices);

            for (int i = 0; i < sizeX - 1; i++)
            {
                for (int e = 0; e < sizeY - 1; e++)
                {
                    Spring spring = new Spring(points[(e + 0) * sizeX + i + 1], points[(e + 1) * sizeX + i + 0]);
                    spring.Softness = FP.EN2; spring.BiasFactor = FP.EN1;
                    springs.Add(spring);
                }
            }

            foreach (Spring spring in springs)
            {
                TSVector delta = spring.body1.position - spring.body2.position;

                if (delta.z != FP.Zero && delta.x != FP.Zero) spring.SpringType = SpringType.ShearSpring;
                else spring.SpringType = SpringType.EdgeSpring;
            }


            for (int i = 0; i < sizeX - 2; i++)
            {
                for (int e = 0; e < sizeY - 2; e++)
                {
                    Spring spring1 = new Spring(points[(e + 0) * sizeX + i + 0], points[(e + 0) * sizeX + i + 2]);
                    spring1.Softness = FP.EN2; spring1.BiasFactor = FP.EN1;

                    Spring spring2 = new Spring(points[(e + 0) * sizeX + i + 0], points[(e + 2) * sizeX + i + 0]);
                    spring2.Softness = FP.EN2; spring2.BiasFactor = FP.EN1;

                    spring1.SpringType = SpringType.BendSpring;
                    spring2.SpringType = SpringType.BendSpring;

                    springs.Add(spring1);
                    springs.Add(spring2);
                }
            }
        }

        public SoftBody(List<TriangleVertexIndices> indices, List<TSVector> vertices)
        {
            EdgeSprings = springs.AsReadOnly();
            VertexBodies = points.AsReadOnly();

            AddPointsAndSprings(indices, vertices);
            Triangles = triangles.AsReadOnly();
        }


        private FP pressure = FP.Zero;
        public FP Pressure { get { return pressure; } set { pressure = value; } }

        private struct Edge
        {
            public int Index1;
            public int Index2;

            public Edge(int index1, int index2)
            {
                Index1 = index1;
                Index2 = index2;
            }

            public override int GetHashCode()
            {
                return Index1.GetHashCode() + Index2.GetHashCode();
            }

            public override bool Equals(object obj)
            {
                Edge e = (Edge)obj;
                return (e.Index1 == Index1 && e.Index2 == Index2 || e.Index1 == Index2 && e.Index2 == Index1);
            }
        }

        #region AddPressureForces
        private void AddPressureForces(FP timeStep)
        {
            if (pressure == FP.Zero || volume == FP.Zero) return;

            FP invVolume = FP.One / volume;

            foreach (Triangle t in triangles)
            {
                TSVector v1 = points[t.indices.I0].position;
                TSVector v2 = points[t.indices.I1].position;
                TSVector v3 = points[t.indices.I2].position;

                TSVector cross = (v3 - v1) % (v2 - v1);
                //TSVector center = (v1 + v2 + v3) * (FP.One / (3 * FP.One));

                points[t.indices.I0].AddForce(invVolume * cross * pressure);
                points[t.indices.I1].AddForce(invVolume * cross * pressure);
                points[t.indices.I2].AddForce(invVolume * cross * pressure);
            }
        }
        #endregion

        public void Translate(TSVector position)
        {
            foreach (MassPoint point in points) point.Position += position;

            Update(FP.Epsilon);
        }

        public void AddForce(TSVector force)
        {
            // TODO
            throw new NotImplementedException();
        }

        public void Rotate(TSMatrix orientation, TSVector center)
        {
            for (int i = 0; i < points.Count; i++)
            {
                points[i].position = TSVector.Transform(points[i].position - center, orientation);
            }
        }

        public TSVector CalculateCenter()
        {
            // TODO
            throw new NotImplementedException();
        }

        private HashSet<Edge> GetEdges(List<TriangleVertexIndices> indices)
        {
            HashSet<Edge> edges = new HashSet<Edge>();

            for (int i = 0; i < indices.Count; i++)
            {
                Edge edge;

                edge = new Edge(indices[i].I0, indices[i].I1);
                if (!edges.Contains(edge)) edges.Add(edge);

                edge = new Edge(indices[i].I1, indices[i].I2);
                if (!edges.Contains(edge)) edges.Add(edge);

                edge = new Edge(indices[i].I2, indices[i].I0);
                if (!edges.Contains(edge)) edges.Add(edge);
            }

            return edges;
        }

        List<int> queryList = new List<int>();

        public virtual void DoSelfCollision(CollisionDetectedHandler collision)
        {
            if (!selfCollision) return;

            TSVector point, normal;
            FP penetration;

            for (int i = 0; i < points.Count; i++)
            {
                queryList.Clear();
                this.dynamicTree.Query(queryList, ref points[i].boundingBox);

                for (int e = 0; e < queryList.Count; e++)
                {
                    Triangle t = this.dynamicTree.GetUserData(queryList[e]);

                    if (!(t.VertexBody1 == points[i] || t.VertexBody2 == points[i] || t.VertexBody3 == points[i]))
                    {
                        if (XenoCollide.Detect(points[i].Shape, t, ref points[i].orientation,
                            ref TSMatrix.InternalIdentity, ref points[i].position, ref TSVector.InternalZero,
                            out point, out normal, out penetration))
                        {
                            int nearest = CollisionSystem.FindNearestTrianglePoint(this, queryList[e], ref point);

                            collision(points[i], points[nearest], point, point, normal, penetration);
                     
                        }
                    }
                }
            }
        }
                    
                

        private void AddPointsAndSprings(List<TriangleVertexIndices> indices, List<TSVector> vertices)
        {
            for (int i = 0; i < vertices.Count; i++)
            {
                MassPoint point = new MassPoint(sphere, this,material);
                point.Position = vertices[i];

                point.Mass = FP.EN1;

                points.Add(point);
            }

            for (int i = 0; i < indices.Count; i++)
            {
                TriangleVertexIndices index = indices[i];
                
                Triangle t = new Triangle(this);

                t.indices = index;
                triangles.Add(t);

                t.boundingBox = TSBBox.SmallBox;
                t.boundingBox.AddPoint(points[t.indices.I0].position);
                t.boundingBox.AddPoint(points[t.indices.I1].position);
                t.boundingBox.AddPoint(points[t.indices.I2].position);

                t.dynamicTreeID = dynamicTree.AddProxy(ref t.boundingBox, t);
            }

            HashSet<Edge> edges = GetEdges(indices);

            int count = 0;

            foreach (Edge edge in edges)
            {
                Spring spring = new Spring(points[edge.Index1], points[edge.Index2]);
                spring.Softness = FP.EN2; spring.BiasFactor = FP.EN1;
                spring.SpringType = SpringType.EdgeSpring;

                springs.Add(spring);
                count++;
            }

        }

        public void SetSpringValues(FP bias, FP softness)
        {
            SetSpringValues(SpringType.EdgeSpring | SpringType.ShearSpring | SpringType.BendSpring,
                bias, softness);
        }

        public void SetSpringValues(SpringType type, FP bias, FP softness)
        {
            for (int i = 0; i < springs.Count; i++)
            {
                if ((springs[i].SpringType & type) != 0)
                {
                    springs[i].Softness = softness; springs[i].BiasFactor = bias;
                }
            }
        }

        public virtual void Update(FP timestep)
        {
            active = false;

            foreach (MassPoint point in points)
            {
                if (point.isActive && !point.isStatic) { active = true; break; }
            }

            if(!active) return;

            box = TSBBox.SmallBox;
            volume = FP.Zero;
            mass = FP.Zero;

            foreach (MassPoint point in points)
            {
                mass += point.Mass;
                box.AddPoint(point.position);
            }

            box.min -= new TSVector(TriangleExpansion);
            box.max += new TSVector(TriangleExpansion);

            foreach (Triangle t in triangles)
            {
                // Update bounding box and move proxy in dynamic tree.
                t.UpdateBoundingBox();

                TSVector linVel = t.VertexBody1.linearVelocity + 
                    t.VertexBody2.linearVelocity + 
                    t.VertexBody3.linearVelocity;

                linVel *= FP.One / (3 * FP.One);

                dynamicTree.MoveProxy(t.dynamicTreeID, ref t.boundingBox, linVel * timestep);

                TSVector v1 = points[t.indices.I0].position;
                TSVector v2 = points[t.indices.I1].position;
                TSVector v3 = points[t.indices.I2].position;

                volume -= ((v2.y - v1.y) * (v3.z - v1.z) -
                    (v2.z - v1.z) * (v3.y - v1.y)) * (v1.x + v2.x + v3.x);
            }

            volume /= (FP)6;

            AddPressureForces(timestep);
        }

        public FP Mass
        {
            get
            {
                return mass;
            }
            set
            {
                for (int i = 0; i < points.Count; i++)
                {
                    points[i].Mass = value / points.Count;
                }
            }
        }

        public FP Volume { get { return volume; } }

        public TSBBox BoundingBox
        {
            get { return box; }
        }

        public int BroadphaseTag { get; set; }

        public object Tag { get; set; }

        public bool IsStaticOrInactive
        {
            get { return !active; }
        }

        public bool IsStaticNonKinematic {
            get {
                return !active;
            }
        }
    }


}


    
