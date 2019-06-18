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
using System.Diagnostics;
#endregion

namespace TrueSync.Physics3D {

    /// <summary>
    /// Entity of the Broadphase system. (Either a Softbody or a RigidBody)
    /// </summary>
    public interface IBroadphaseEntity
    {
        TSBBox BoundingBox { get; }
        bool IsStaticOrInactive{ get; }

        bool IsStaticNonKinematic { get; }
    }


    /// <summary>
    /// A delegate for collision detection.
    /// </summary>
    /// <param name="body1">The first body colliding with the second one.</param>
    /// <param name="body2">The second body colliding with the first one.</param>
    /// <param name="point">The point on body in world coordinates, where collision occur.</param>
    /// <param name="normal">The normal pointing from body2 to body1.</param>
    /// <param name="penetration">Estimated penetration depth of the collision.</param>
    /// <seealso cref="CollisionSystem.Detect(bool)"/>
    /// <seealso cref="CollisionSystem.Detect(RigidBody,RigidBody)"/>
    public delegate void CollisionDetectedHandler(RigidBody body1,RigidBody body2, 
                    TSVector point1, TSVector point2, TSVector normal,FP penetration);

    /// <summary>
    /// A delegate to inform the user that a pair of bodies passed the broadsphase
    /// system of the engine.
    /// </summary>
    /// <param name="body1">The first body.</param>
    /// <param name="body2">The second body.</param>
    /// <returns>If false is returned the collision information is dropped. The CollisionDetectedHandler
    /// is never called.</returns>
    public delegate bool PassedBroadphaseHandler(IBroadphaseEntity entity1, IBroadphaseEntity entity2);

    /// <summary>
    /// A delegate to inform the user that a pair of bodies passed the narrowphase
    /// system of the engine.
    /// </summary>
    /// <param name="body1">The first body.</param>
    /// <param name="body2">The second body.</param>
    /// <returns>If false is returned the collision information is dropped. The CollisionDetectedHandler
    /// is never called.</returns>
    public delegate bool PassedNarrowphaseHandler(RigidBody body1,RigidBody body2, 
                    ref TSVector point, ref TSVector normal,FP penetration);

    /// <summary>
    /// A delegate for raycasting.
    /// </summary>
    /// <param name="body">The body for which collision with the ray is detected.</param>
    /// <param name="normal">The normal of the collision.</param>
    /// <param name="fraction">The fraction which gives information where at the 
    /// ray the collision occured. The hitPoint is calculated by: rayStart+friction*direction.</param>
    /// <returns>If false is returned the collision information is dropped.</returns>
    public delegate bool RaycastCallback(RigidBody body,TSVector normal, FP fraction);

    /// <summary>
    /// CollisionSystem. Used by the world class to detect all collisions. 
    /// Can be used seperatly from the physics.
    /// </summary>
    public abstract class CollisionSystem
    {

        /// <summary>
        /// Helper class which holds two bodies. Mostly used
        /// for multithreaded detection. (Passing this as
        /// the object parameter to ThreadManager.Instance.AddTask)
        /// </summary>
        #region protected class BroadphasePair
        protected class BroadphasePair
        {
            /// <summary>
            /// The first body.
            /// </summary>
            public IBroadphaseEntity Entity1;
            /// <summary>
            /// The second body.
            /// </summary>
            public IBroadphaseEntity Entity2;

            /// <summary>
            /// A resource pool of Pairs.
            /// </summary>
            public static ResourcePool<BroadphasePair> Pool = new ResourcePool<BroadphasePair>();
        }
        #endregion

        /// <summary>
        /// Remove a body from the collision system. Removing a body from the world
        /// does automatically remove it from the collision system.
        /// </summary>
        /// <param name="body">The body to remove.</param>
        /// <returns>Returns true if the body was successfully removed, otherwise false.</returns>
        public abstract bool RemoveEntity(IBroadphaseEntity body);

        /// <summary>
        /// Add a body to the collision system. Adding a body to the world
        /// does automatically add it to the collision system.
        /// </summary>
        /// <param name="body">The body to remove.</param>
        public abstract void AddEntity(IBroadphaseEntity body);

        /// <summary>
        /// Gets called when the broadphase system has detected possible collisions.
        /// </summary>
        public event PassedBroadphaseHandler PassedBroadphase;

        public World world;

        private CollisionDetectedHandler collisionDetected;

        /// <summary>
        /// Gets called when broad- and narrow phase collision were positive.
        /// </summary>
        public event CollisionDetectedHandler CollisionDetected {
            add {
                collisionDetected += value;
            }
            remove {
                collisionDetected -= value;
            }
        }

        private bool speculativeContacts = false;
        public bool EnableSpeculativeContacts { get { return speculativeContacts; }
            set { speculativeContacts = value; }
        }

        /// <summary>
        /// Initializes a new instance of the CollisionSystem.
        /// </summary>
        public CollisionSystem()
        {
        }

        internal bool useTerrainNormal = true;
        internal bool useTriangleMeshNormal = true;

        /// <summary>
        /// If set to true the collision system uses the normal of
        /// the current colliding triangle as collision normal. This
        /// fixes unwanted behavior on triangle transitions.
        /// </summary>
        public bool UseTriangleMeshNormal { get { return useTriangleMeshNormal; } set { useTriangleMeshNormal = value; } }
        
                /// <summary>
        /// If set to true the collision system uses the normal of
        /// the current colliding triangle as collision normal. This
        /// fixes unwanted behavior on triangle transitions.
        /// </summary>
        public bool UseTerrainNormal { get { return useTerrainNormal; } set { useTerrainNormal = value; } }

        /// <summary>
        /// Checks two bodies for collisions using narrowphase.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        #region public virtual void Detect(IBroadphaseEntity body1, IBroadphaseEntity body2)
        public virtual void Detect(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
        {
            //Debug.Assert(entity1 != entity2, "CollisionSystem reports selfcollision. Something is wrong.");

            RigidBody rigidBody1 = entity1 as RigidBody;
            RigidBody rigidBody2 = entity2 as RigidBody;

            if (rigidBody1 != null)
            { 
                if(rigidBody2 != null)
                {
                    // most common
                    DetectRigidRigid(rigidBody1, rigidBody2);
                }
                else
                {
                    SoftBody softBody2 = entity2 as SoftBody;
                    if(softBody2 != null) DetectSoftRigid(rigidBody1,softBody2);
                }
            }
            else
            {
                SoftBody softBody1 = entity1 as SoftBody;

                if(rigidBody2 != null)
                {
                    if(softBody1 != null) DetectSoftRigid(rigidBody2,softBody1);
                }
                else
                {
                    // less common
                    SoftBody softBody2 = entity2 as SoftBody;
                    if(softBody1 != null && softBody2 != null) DetectSoftSoft(softBody1,softBody2);
                }
            }
        }

        private ResourcePool<List<int>> potentialTriangleLists = new ResourcePool<List<int>>();

        private void DetectSoftSoft(SoftBody body1, SoftBody body2)
        {
            List<int> my = potentialTriangleLists.GetNew();
            List<int> other = potentialTriangleLists.GetNew();

            body1.dynamicTree.Query(other, my, body2.dynamicTree);

            for (int i = 0; i < other.Count; i++)
            {
                SoftBody.Triangle myTriangle = body1.dynamicTree.GetUserData(my[i]);
                SoftBody.Triangle otherTriangle = body2.dynamicTree.GetUserData(other[i]);

                TSVector point, normal;
                FP penetration;
                bool result;

                result = XenoCollide.Detect(myTriangle, otherTriangle, ref TSMatrix.InternalIdentity, ref TSMatrix.InternalIdentity,
                    ref TSVector.InternalZero, ref TSVector.InternalZero, out point, out normal, out penetration);

                if (result)
                {
                    int minIndexMy = FindNearestTrianglePoint(body1, my[i], ref point);
                    int minIndexOther = FindNearestTrianglePoint(body2, other[i], ref point);


                    RaiseCollisionDetected(body1.VertexBodies[minIndexMy],
                        body2.VertexBodies[minIndexOther], ref point, ref point, ref normal, penetration);
                }
            }

            my.Clear(); other.Clear();
            potentialTriangleLists.GiveBack(my);
            potentialTriangleLists.GiveBack(other);
        }

        private void DetectRigidRigid(RigidBody body1, RigidBody body2)
        {
            bool b1IsMulti = (body1.Shape is Multishape);
            bool b2IsMulti = (body2.Shape is Multishape);

            bool speculative = speculativeContacts ||
                (body1.EnableSpeculativeContacts || body2.EnableSpeculativeContacts);

            TSVector point, normal;
            FP penetration;

            if (!b1IsMulti && !b2IsMulti)
            {
                if (XenoCollide.Detect(body1.Shape, body2.Shape, ref body1.orientation,
                    ref body2.orientation, ref body1.position, ref body2.position,
                    out point, out normal, out penetration))
                {
                    //normal = JVector.Up;
                    //UnityEngine.Debug.Log("FINAL  --- >>> normal: " + normal);
                    TSVector point1, point2;
                    FindSupportPoints(body1, body2, body1.Shape, body2.Shape, ref point, ref normal, out point1, out point2);
                    RaiseCollisionDetected(body1, body2, ref point1, ref point2, ref normal, penetration);
                }
                else if (speculative)
                {
                    TSVector hit1, hit2;

                    if (GJKCollide.ClosestPoints(body1.Shape, body2.Shape, ref body1.orientation, ref body2.orientation,
                        ref body1.position, ref body2.position, out hit1, out hit2, out normal))
                    {
                        TSVector delta = hit2 - hit1;

                        if (delta.sqrMagnitude < (body1.sweptDirection - body2.sweptDirection).sqrMagnitude)
                        {
                            penetration = delta * normal;

                            if (penetration < FP.Zero)
                            {
                                RaiseCollisionDetected(body1, body2, ref hit1, ref hit2, ref normal, penetration);
                            }

                        }
                    }

                }

				//UnityEngine.Debug.Log("-----------------------: " + normal);
            }
            else if (b1IsMulti && b2IsMulti)
            {
                Multishape ms1 = (body1.Shape as Multishape);
                Multishape ms2 = (body2.Shape as Multishape);

                ms1 = ms1.RequestWorkingClone();
                ms2 = ms2.RequestWorkingClone();

                TSBBox transformedBoundingBox = body2.boundingBox;
                transformedBoundingBox.InverseTransform(ref body1.position, ref body1.orientation);

                int ms1Length = ms1.Prepare(ref transformedBoundingBox);

                transformedBoundingBox = body1.boundingBox;
                transformedBoundingBox.InverseTransform(ref body2.position, ref body2.orientation);

                int ms2Length = ms2.Prepare(ref transformedBoundingBox);

                if (ms1Length == 0 || ms2Length == 0)
                {
                    ms1.ReturnWorkingClone();
                    ms2.ReturnWorkingClone();
                    return;
                }

                for (int i = 0; i < ms1Length; i++)
                {
                    ms1.SetCurrentShape(i);

                    for (int e = 0; e < ms2Length; e++)
                    {
                        ms2.SetCurrentShape(e);

                        if (XenoCollide.Detect(ms1, ms2, ref body1.orientation,
                            ref body2.orientation, ref body1.position, ref body2.position,
                            out point, out normal, out penetration))
                        {
                            TSVector point1, point2;
                            FindSupportPoints(body1, body2, ms1, ms2, ref point, ref normal, out point1, out point2);
                            RaiseCollisionDetected(body1, body2, ref point1, ref point2, ref normal, penetration);
                        }
                        else if (speculative)
                        {
                            TSVector hit1, hit2;

                            if (GJKCollide.ClosestPoints(ms1, ms2, ref body1.orientation, ref body2.orientation,
                                ref body1.position, ref body2.position, out hit1, out hit2, out normal))
                            {
                                TSVector delta = hit2 - hit1;

                                if (delta.sqrMagnitude < (body1.sweptDirection - body2.sweptDirection).sqrMagnitude)
                                {
                                    penetration = delta * normal;

                                    if (penetration < FP.Zero)
                                    {
                                        RaiseCollisionDetected(body1, body2, ref hit1, ref hit2, ref normal, penetration);
                                    }
                                }
                            }


                        }
                    }
                }

                ms1.ReturnWorkingClone();
                ms2.ReturnWorkingClone();

            }
            else
            {
                RigidBody b1, b2;

                if (body2.Shape is Multishape) { b1 = body2; b2 = body1; }
                else { b2 = body2; b1 = body1; }

                Multishape ms = (b1.Shape as Multishape);

                ms = ms.RequestWorkingClone();

                TSBBox transformedBoundingBox = b2.boundingBox;
                transformedBoundingBox.InverseTransform(ref b1.position, ref b1.orientation);

                int msLength = ms.Prepare(ref transformedBoundingBox);

                if (msLength == 0)
                {
                    ms.ReturnWorkingClone();
                    return;
                }

                for (int i = 0; i < msLength; i++)
                {
                    ms.SetCurrentShape(i);

                    if (XenoCollide.Detect(ms, b2.Shape, ref b1.orientation,
                        ref b2.orientation, ref b1.position, ref b2.position,
                        out point, out normal, out penetration))
                    {
                        TSVector point1, point2;
                        FindSupportPoints(b1, b2, ms, b2.Shape, ref point, ref normal, out point1, out point2);

                        if (useTerrainNormal && ms is TerrainShape)
                        {
                            (ms as TerrainShape).CollisionNormal(out normal);
                            TSVector.Transform(ref normal, ref b1.orientation, out normal);
                        }
                        else if (useTriangleMeshNormal && ms is TriangleMeshShape)
                        {
                            (ms as TriangleMeshShape).CollisionNormal(out normal);
                            TSVector.Transform(ref normal, ref b1.orientation, out normal);
                        }

                        RaiseCollisionDetected(b1, b2, ref point1, ref point2, ref normal, penetration);
                    }
                    else if (speculative)
                    {
                        TSVector hit1, hit2;

                        if (GJKCollide.ClosestPoints(ms, b2.Shape, ref b1.orientation, ref b2.orientation,
                            ref b1.position, ref b2.position, out hit1, out hit2, out normal))
                        {
                            TSVector delta = hit2 - hit1;

                            if (delta.sqrMagnitude < (body1.sweptDirection - body2.sweptDirection).sqrMagnitude)
                            {
                                penetration = delta * normal;

                                if (penetration < FP.Zero)
                                {
                                    RaiseCollisionDetected(b1, b2, ref hit1, ref hit2, ref normal, penetration);
                                }
                            }
                        }
                    }
                }

                ms.ReturnWorkingClone();
            }
        }

        private void DetectSoftRigid(RigidBody rigidBody, SoftBody softBody)
        {
            if (rigidBody.Shape is Multishape)
            {
                Multishape ms = (rigidBody.Shape as Multishape);
                ms = ms.RequestWorkingClone();

                TSBBox transformedBoundingBox = softBody.BoundingBox;
                transformedBoundingBox.InverseTransform(ref rigidBody.position, ref rigidBody.orientation);

                int msLength = ms.Prepare(ref transformedBoundingBox);

                List<int> detected = potentialTriangleLists.GetNew();
                softBody.dynamicTree.Query(detected, ref rigidBody.boundingBox);

                foreach (int i in detected)
                {
                    SoftBody.Triangle t = softBody.dynamicTree.GetUserData(i);

                    TSVector point, normal;
                    FP penetration;
                    bool result;

                    for (int e = 0; e < msLength; e++)
                    {
                        ms.SetCurrentShape(e);

                        result = XenoCollide.Detect(ms, t, ref rigidBody.orientation, ref TSMatrix.InternalIdentity,
                            ref rigidBody.position, ref TSVector.InternalZero, out point, out normal, out penetration);

                        if (result)
                        {
                            int minIndex = FindNearestTrianglePoint(softBody, i, ref point);

                            RaiseCollisionDetected(rigidBody,
                                softBody.VertexBodies[minIndex], ref point, ref point, ref normal, penetration);
                        }
                    }

                }

                detected.Clear(); potentialTriangleLists.GiveBack(detected);
                ms.ReturnWorkingClone();      
            }
            else
            {
                List<int> detected = potentialTriangleLists.GetNew();
                softBody.dynamicTree.Query(detected, ref rigidBody.boundingBox);

                foreach (int i in detected)
                {
                    SoftBody.Triangle t = softBody.dynamicTree.GetUserData(i);

                    TSVector point, normal;
                    FP penetration;
                    bool result;

                    result = XenoCollide.Detect(rigidBody.Shape, t, ref rigidBody.orientation, ref TSMatrix.InternalIdentity,
                        ref rigidBody.position, ref TSVector.InternalZero, out point, out normal, out penetration);

                    if (result)
                    {
                        int minIndex = FindNearestTrianglePoint(softBody, i, ref point);

                        RaiseCollisionDetected(rigidBody,
                            softBody.VertexBodies[minIndex], ref point, ref point, ref normal, penetration);
                    }
                }

                detected.Clear();
                potentialTriangleLists.GiveBack(detected);
            }
        }

        public static int FindNearestTrianglePoint(SoftBody sb, int id, ref TSVector point)
        {
            SoftBody.Triangle triangle = sb.dynamicTree.GetUserData(id);
            TSVector p;

            p = sb.VertexBodies[triangle.indices.I0].position;
            TSVector.Subtract(ref p, ref point, out p);

            FP length0 = p.sqrMagnitude;

            p = sb.VertexBodies[triangle.indices.I1].position;
            TSVector.Subtract(ref p, ref point, out p);

            FP length1 = p.sqrMagnitude;

            p = sb.VertexBodies[triangle.indices.I2].position;
            TSVector.Subtract(ref p, ref point, out p);

            FP length2 = p.sqrMagnitude;

            if (length0 < length1)
            {
                if (length0 < length2) return triangle.indices.I0;
                else return triangle.indices.I2;
            }
            else
            {
                if (length1 < length2) return triangle.indices.I1;
                else return triangle.indices.I2;
            }
        }


        private void FindSupportPoints(RigidBody body1, RigidBody body2,
            Shape shape1, Shape shape2, ref TSVector point, ref TSVector normal,
            out TSVector point1, out TSVector point2)
        {
            TSVector mn; TSVector.Negate(ref normal, out mn);

            TSVector sA; SupportMapping(body1, shape1, ref mn, out sA);
            TSVector sB; SupportMapping(body2, shape2, ref normal, out sB);

            TSVector.Subtract(ref sA, ref point, out sA);
            TSVector.Subtract(ref sB, ref point, out sB);

            FP dot1 = TSVector.Dot(ref sA, ref normal);
            FP dot2 = TSVector.Dot(ref sB, ref normal);

            TSVector.Multiply(ref normal, dot1, out sA);
            TSVector.Multiply(ref normal, dot2, out sB);

            TSVector.Add(ref point, ref sA, out point1);
            TSVector.Add(ref point, ref sB, out point2);
        }

        private void SupportMapping(RigidBody body, Shape workingShape, ref TSVector direction, out TSVector result)
        {
            TSVector.Transform(ref direction, ref body.invOrientation, out result);
            workingShape.SupportMapping(ref result, out result);
            TSVector.Transform(ref result, ref body.orientation, out result);
            TSVector.Add(ref result, ref body.position, out result);
        }

        #endregion

        /// <summary>
        /// Sends a ray (definied by start and direction) through the scene (all bodies added).
        /// NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        public abstract bool Raycast(TSVector rayOrigin, TSVector rayDirection, RaycastCallback raycast, out RigidBody body, out TSVector normal, out FP fraction);

        public abstract bool Raycast(TSVector rayOrigin, TSVector rayDirection, RaycastCallback raycast, int layerMask, out RigidBody body, out TSVector normal, out FP fraction);

        /// <summary>
        /// Raycasts a single body. NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        public abstract bool Raycast(RigidBody body, TSVector rayOrigin, TSVector rayDirection, out TSVector normal, out FP fraction);


        /// <summary>
        /// Checks the state of two bodies.
        /// </summary>
        /// <param name="entity1">The first body.</param>
        /// <param name="entity2">The second body.</param>
        /// <returns>Returns true if both are static or inactive.</returns>
        public bool CheckBothStaticOrInactive(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
        {
            return (entity1.IsStaticOrInactive && entity2.IsStaticOrInactive);
       }

        public bool CheckBothStaticNonKinematic(IBroadphaseEntity entity1, IBroadphaseEntity entity2) {
            return !world.CanBodiesCollide((RigidBody) entity1, (RigidBody) entity2);
            //return (entity1.IsStaticNonKinematic && entity2.IsStaticNonKinematic);
        }

        /// <summary>
        /// Checks the AABB of the two rigid bodies.
        /// </summary>
        /// <param name="entity1">The first body.</param>
        /// <param name="entity2">The second body.</param>
        /// <returns>Returns true if an intersection occours.</returns>
        public bool CheckBoundingBoxes(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
        {
            TSBBox box1 = entity1.BoundingBox;
            TSBBox box2 = entity2.BoundingBox;

            return ((((box1.max.z >= box2.min.z) && (box1.min.z <= box2.max.z)) &&
                ((box1.max.y >= box2.min.y) && (box1.min.y <= box2.max.y))) &&
                ((box1.max.x >= box2.min.x) && (box1.min.x <= box2.max.x)));
        }

        /// <summary>
        /// Raises the PassedBroadphase event.
        /// </summary>
        /// <param name="entity1">The first body.</param>
        /// <param name="entity2">The second body.</param>
        /// <returns>Returns false if the collision information
        /// should be dropped</returns>
        public bool RaisePassedBroadphase(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
        {
			if (this.PassedBroadphase != null)
                return this.PassedBroadphase(entity1, entity2);

            // allow this detection by default
            return true;
        }


        /// <summary>
        /// Raises the CollisionDetected event.
        /// </summary>
        /// <param name="body1">The first body involved in the collision.</param>
        /// <param name="body2">The second body involved in the collision.</param>
        /// <param name="point">The collision point.</param>
        /// <param name="normal">The normal pointing to body1.</param>
        /// <param name="penetration">The penetration depth.</param>
        protected void RaiseCollisionDetected(RigidBody body1, RigidBody body2,
                                            ref TSVector point1, ref TSVector point2,
                                            ref TSVector normal, FP penetration)
        {
            if (this.collisionDetected != null)
                this.collisionDetected(body1, body2, point1, point2, normal, penetration);
        }

        /// <summary>
        /// Tells the collisionsystem to check all bodies for collisions. Hook into the <see cref="PassedBroadphase"/>
        /// and <see cref="CollisionDetected"/> events to get the results.
        /// </summary>
        public abstract void Detect();
    }
}
