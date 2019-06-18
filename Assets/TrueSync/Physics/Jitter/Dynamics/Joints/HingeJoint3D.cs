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

namespace TrueSync.Physics3D {

    /// <summary>
    /// Connects to bodies with a hinge joint.
    /// </summary>
    public class HingeJoint3D : Joint3D
    {

        private PointOnPoint[] worldPointConstraint;

        /// <summary>
        /// Initializes a new instance of the HingeJoint class.
        /// </summary>
        /// <param name="world">The world class where the constraints get added to.</param>
        /// <param name="body1">The first body connected to the second one.</param>
        /// <param name="body2">The second body connected to the first one.</param>
        /// <param name="position">The position in world space where both bodies get connected.</param>
        /// <param name="hingeAxis">The axis if the hinge.</param>
        /// 
        public static void Create(IWorld world, IBody3D body1, IBody3D body2, TSVector position, TSVector hingeAxis) {
            new HingeJoint3D(world, body1, body2, position, hingeAxis);
        }

        private HingeJoint3D(IWorld world, IBody3D body1, IBody3D body2, TSVector position, TSVector hingeAxis) : base((World) world)
        {
            worldPointConstraint = new PointOnPoint[2];

            hingeAxis *= FP.Half;

            TSVector pos1 = position; TSVector.Add(ref pos1,ref hingeAxis,out pos1);
            TSVector pos2 = position; TSVector.Subtract(ref pos2,ref hingeAxis,out pos2);

            worldPointConstraint[0] = new PointOnPoint((RigidBody)body1, (RigidBody)body2, pos1);
            worldPointConstraint[1] = new PointOnPoint((RigidBody)body1, (RigidBody)body2, pos2);

            StateTracker.AddTracking(worldPointConstraint[0]);
            StateTracker.AddTracking(worldPointConstraint[1]);

            Activate();
        }

        public PointOnPoint PointOnPointConstraint1 { get { return worldPointConstraint[0]; } }

        public PointOnPoint PointOnPointConstraint2 { get { return worldPointConstraint[1]; } }

        public FP AppliedImpulse { get { return worldPointConstraint[0].AppliedImpulse + worldPointConstraint[1].AppliedImpulse; } }

        /// <summary>
        /// Adds the internal constraints of this joint to the world class.
        /// </summary>
        public override void Activate()
        {
            World.AddConstraint(worldPointConstraint[0]);
            World.AddConstraint(worldPointConstraint[1]);
        }

        /// <summary>
        /// Removes the internal constraints of this joint from the world class.
        /// </summary>
        public override void Deactivate()
        {
            World.RemoveConstraint(worldPointConstraint[0]);
            World.RemoveConstraint(worldPointConstraint[1]);
        }
    }
}
