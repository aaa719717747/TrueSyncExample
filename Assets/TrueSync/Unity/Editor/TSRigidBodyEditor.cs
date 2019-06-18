using UnityEngine;
using UnityEditor;
using TrueSync.Physics3D;

namespace TrueSync {

    /**
    *  @brief Custom editor to {@link TSRigidBody}.
    **/
    [CustomEditor(typeof(TSRigidBody))]
    [CanEditMultipleObjects]
    public class TSRigidBodyEditor : Editor {

        private bool constraintsFoldout;

        public override void OnInspectorGUI() {
            TSRigidBody tsRB = (target as TSRigidBody);

            DrawDefaultInspector();

            serializedObject.Update();

            constraintsFoldout = EditorGUILayout.Foldout(constraintsFoldout, "Constraints");

            if (constraintsFoldout) {
                EditorGUI.indentLevel++;

                TSRigidBodyConstraints freezeConstraintPos = tsRB.constraints, freezeConstraintRot = tsRB.constraints;

                DrawFreezePanel(ref freezeConstraintPos, true);
                DrawFreezePanel(ref freezeConstraintRot, false);

                tsRB.constraints = (freezeConstraintPos | freezeConstraintRot);

                EditorGUI.indentLevel--;
            }

            serializedObject.ApplyModifiedProperties();

            if (GUI.changed) {
                EditorUtility.SetDirty(target);
            }
        }

        private static void DrawFreezePanel(ref TSRigidBodyConstraints freezeConstraint, bool position) {
            TSRigidBodyConstraints axisX = position ? TSRigidBodyConstraints.FreezePositionX : TSRigidBodyConstraints.FreezeRotationX;
            TSRigidBodyConstraints axisY = position ? TSRigidBodyConstraints.FreezePositionY : TSRigidBodyConstraints.FreezeRotationY;
            TSRigidBodyConstraints axisZ = position ? TSRigidBodyConstraints.FreezePositionZ : TSRigidBodyConstraints.FreezeRotationZ;

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PrefixLabel(position ? "Freeze Position" : "Freeze Rotation");

            Rect controlRect = GUILayoutUtility.GetLastRect();
            controlRect.width = 30;
            controlRect.x += EditorGUIUtility.labelWidth;

            bool fX = GUI.Toggle(controlRect, CheckAxis(freezeConstraint, axisX), "X");

            controlRect.x += 30;
            bool fY = GUI.Toggle(controlRect, CheckAxis(freezeConstraint, axisY), "Y");

            controlRect.x += 30;
            bool fZ = GUI.Toggle(controlRect, CheckAxis(freezeConstraint, axisZ), "Z");

            freezeConstraint = TSRigidBodyConstraints.None;

            if (fX) {
                freezeConstraint |= axisX;
            }

            if (fY) {
                freezeConstraint |= axisY;
            }

            if (fZ) {
                freezeConstraint |= axisZ;
            }

            EditorGUILayout.EndHorizontal();
        }

        private static bool CheckAxis(TSRigidBodyConstraints toCheck, TSRigidBodyConstraints axis) {
            return (toCheck & axis) == axis;
        }

    }

}