using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

namespace TrueSync {

    [CustomPropertyDrawer(typeof(TSVector2))]
    public class TSVector2Drawer : PropertyDrawer {

        private const int INDENT_OFFSET = 15;
        private const int LABEL_WIDTH = 12;
        private const int LABEL_MARGIN = 1;

        private static GUIContent xLabel = new GUIContent("X");
        private static GUIContent yLabel = new GUIContent("Y");

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
            EditorGUI.BeginProperty(position, label, property);

            position = EditorGUI.PrefixLabel(position, label);

            position.width /= 2f;
            float indentOffsetLevel = (INDENT_OFFSET) * EditorGUI.indentLevel;
            position.width += indentOffsetLevel;

            EditorGUIUtility.labelWidth = indentOffsetLevel + LABEL_WIDTH;

            SerializedProperty xSerProperty = property.FindPropertyRelative("x");
            position.x -= indentOffsetLevel;
            EditorGUI.PropertyField(position, xSerProperty, xLabel);

            position.x += position.width;

            SerializedProperty ySerProperty = property.FindPropertyRelative("y");
            position.x -= indentOffsetLevel;
            EditorGUI.PropertyField(position, ySerProperty, yLabel);

            EditorGUI.EndProperty();
        }

    }

}