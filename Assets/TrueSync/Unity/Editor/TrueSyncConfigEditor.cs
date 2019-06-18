using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

namespace TrueSync {

    /**
    *  @brief Custom editor to {@link TrueSyncConfig}.
    **/
    [CustomEditor(typeof(TrueSyncConfig))]
    public class TrueSyncConfigEditor : Editor {

        private const int OFFSET_LEFT = 30;
        private const int OFFSET_TOP = 5;
        private const int OFFSET_TOP_COLLUMNS = 20;
        private const int TOGGLE_SIZE = 10;
        private const int PIVOT_ANGLE = -90;
        private const int HEIGHT_LABEL = 16;
        private const int NUMBER_OF_LAYERS = 32;

        private struct LayerInfo {

            public int layerIndex;

            public string layerName;

            public Vector2 labelSize;

        }

        private LayerInfo[] layersInfo = new LayerInfo[NUMBER_OF_LAYERS];
        private int validLayersCount;

        private bool physics2DCollisionFoldout;
        private bool physics3DCollisionFoldout;

        public override void OnInspectorGUI() {
            serializedObject.Update();

            TrueSyncConfig settings = (TrueSyncConfig) target;
            Undo.RecordObject(settings, "Edit TrueSyncConfig");

            EditorGUILayout.LabelField("General", EditorStyles.boldLabel);
            EditorGUI.indentLevel++;

            settings.syncWindow = EditorGUILayout.IntField("Sync Window", settings.syncWindow);
            if (settings.syncWindow < 0) {
                settings.syncWindow = 0;
            }

            settings.rollbackWindow = EditorGUILayout.IntField("Rollback Window", settings.rollbackWindow);
            if (settings.rollbackWindow < 0) {
                settings.rollbackWindow = 0;
            }

            settings.panicWindow = EditorGUILayout.IntField("Panic Window", settings.panicWindow);
            if (settings.panicWindow < 0) {
                settings.panicWindow = 0;
            }

            settings.lockedTimeStep = EditorGUILayout.FloatField("Locked Time Step", settings.lockedTimeStep.AsFloat());
            if (settings.lockedTimeStep < 0) {
                settings.lockedTimeStep = 0;
            }

            settings.showStats = EditorGUILayout.Toggle("Show Stats", settings.showStats);

            EditorGUI.indentLevel--;

            GUILayout.Space(10);

            settings.physics2DEnabled = EditorGUILayout.BeginToggleGroup("2D Physics", settings.physics2DEnabled);

            if (settings.physics2DEnabled) {
                settings.physics3DEnabled = false;

                EditorGUI.indentLevel++;

                Vector2 gField2D = EditorGUILayout.Vector2Field("Gravity", settings.gravity2D.ToVector());
                settings.gravity2D.x = gField2D.x;
                settings.gravity2D.y = gField2D.y;

                settings.speculativeContacts2D = EditorGUILayout.Toggle("Speculative Contacts", settings.speculativeContacts2D);

                physics2DCollisionFoldout = EditorGUILayout.Foldout(physics2DCollisionFoldout, "Layer Collision Matrix");
                if (physics2DCollisionFoldout) {
                    DrawLayerMatrix(settings.physics2DIgnoreMatrix);
                }

                EditorGUI.indentLevel--;
            }

            EditorGUILayout.EndToggleGroup();

            settings.physics3DEnabled = EditorGUILayout.BeginToggleGroup("3D Physics", settings.physics3DEnabled);

            if (settings.physics3DEnabled) {
                settings.physics2DEnabled = false;

                EditorGUI.indentLevel++;

                Vector3 gField3D = EditorGUILayout.Vector3Field("Gravity", settings.gravity3D.ToVector());
                settings.gravity3D.x = gField3D.x;
                settings.gravity3D.y = gField3D.y;
                settings.gravity3D.z = gField3D.z;

                settings.speculativeContacts3D = EditorGUILayout.Toggle("Speculative Contacts", settings.speculativeContacts3D);

                physics3DCollisionFoldout = EditorGUILayout.Foldout(physics3DCollisionFoldout, "Layer Collision Matrix");
                if (physics3DCollisionFoldout) {
                    DrawLayerMatrix(settings.physics3DIgnoreMatrix);
                }

                EditorGUI.indentLevel--;
            }

            EditorGUILayout.EndToggleGroup();

            serializedObject.ApplyModifiedProperties();

            GUILayout.Space(20);

            if (GUILayout.Button("Highlight Settings")) {
                EditorGUIUtility.PingObject(target);
            }

            if (GUI.changed) {
                EditorUtility.SetDirty(target);
            }
        }

        private void DrawLayerMatrix(bool[] ignoreMatrix) {
            float maxLabelWidth = 0;

            validLayersCount = 0;
            for (int i = 0; i < NUMBER_OF_LAYERS; i++) {
                if (LayerMask.LayerToName(i) != "") {
                    layersInfo[validLayersCount].layerIndex = i;
                    layersInfo[validLayersCount].layerName = LayerMask.LayerToName(i);

                    GUIContent labelGUIContent = new GUIContent(layersInfo[validLayersCount].layerName);
                    Vector2 lSize = GUI.skin.label.CalcSize(labelGUIContent);
                    layersInfo[validLayersCount].labelSize = lSize;

                    if (lSize.x > maxLabelWidth) {
                        maxLabelWidth = lSize.x;
                    }

                    validLayersCount++;
                }
            }

            Rect lastAnchorRect = GUILayoutUtility.GetLastRect();

            DrawLayerMatrixColumns(lastAnchorRect, maxLabelWidth);
            DrawLayerMatrixRows(lastAnchorRect, maxLabelWidth);
            DrawLayerMatrixToggles(ignoreMatrix, lastAnchorRect, maxLabelWidth);

            GUILayout.Space(validLayersCount * HEIGHT_LABEL + maxLabelWidth + OFFSET_TOP_COLLUMNS);
        }

        private void DrawLayerMatrixColumns(Rect lastAnchorRect, float maxLabelWidth) {
            Matrix4x4 oldMatrix = GUI.matrix;

            Vector2 rPoint = new Vector2(lastAnchorRect.xMin, lastAnchorRect.yMax + maxLabelWidth + OFFSET_TOP_COLLUMNS);

            float offsetX = rPoint.x + OFFSET_LEFT + maxLabelWidth;

            GUIUtility.RotateAroundPivot(PIVOT_ANGLE, new Vector2(0, rPoint.y));
            GUI.BeginGroup(new Rect(rPoint, new Vector2(maxLabelWidth, lastAnchorRect.width)));

            float offsetY = 0;

            for (int i = validLayersCount - 1; i >= 0; i--) {
                Vector2 lSize = layersInfo[i].labelSize;
                GUI.Label(new Rect(0, offsetX + offsetY, lSize.x, lSize.y), layersInfo[i].layerName);

                offsetY += lSize.y;
            }

            GUI.EndGroup();

            GUI.matrix = oldMatrix;
        }

        private void DrawLayerMatrixRows(Rect lastAnchorRect, float maxLabelWidth) {
            Vector2 rPoint = new Vector2(lastAnchorRect.xMin, lastAnchorRect.yMax + maxLabelWidth + OFFSET_TOP);

            GUI.BeginGroup(new Rect(rPoint, new Vector2(maxLabelWidth + OFFSET_LEFT, lastAnchorRect.width)));

            float offsetY = 0;

            GUI.skin.label.alignment = TextAnchor.MiddleRight;

            for (int i = 0; i < validLayersCount; i++) {
                Vector2 lSize = layersInfo[i].labelSize;
                GUI.Label(new Rect(OFFSET_LEFT, offsetY, maxLabelWidth, lSize.y), layersInfo[i].layerName);

                offsetY += lSize.y;
            }

            GUI.EndGroup();
        }

        private void DrawLayerMatrixToggles(bool[] ignoreMatrix, Rect lastAnchorRect, float maxLabelWidth) {
            Vector2 rPoint = new Vector2(lastAnchorRect.xMin + maxLabelWidth + OFFSET_LEFT, lastAnchorRect.yMax + maxLabelWidth + OFFSET_TOP);

            GUI.BeginGroup(new Rect(rPoint, new Vector2(lastAnchorRect.width, lastAnchorRect.width - (maxLabelWidth + OFFSET_LEFT))));
            int matrixIndex = -1;
            for (int i = 0; i < validLayersCount; i++) {
                for (int j = 0, lengthJ = validLayersCount - i; j < lengthJ; j++) {
                    int layerAIndex = layersInfo[i].layerIndex;
                    int layerBIndex = layersInfo[validLayersCount-j-1].layerIndex;

                    matrixIndex = ((NUMBER_OF_LAYERS + NUMBER_OF_LAYERS - layerAIndex + 1) * layerAIndex) / 2 + layerBIndex;
                    ignoreMatrix[matrixIndex] = GUI.Toggle(new Rect(j * HEIGHT_LABEL + 1, i * HEIGHT_LABEL, TOGGLE_SIZE, TOGGLE_SIZE), ignoreMatrix[matrixIndex], "");
                }
            }

            GUI.EndGroup();
        }

    }

}