using System;
using UnityEditor;
using UnityEngine;

namespace TrueSync {

    [CustomPropertyDrawer(typeof(FP))]
    public class TSFPDrawer : PropertyDrawer {

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
            EditorGUI.BeginProperty(position, label, property);

            var rawProp = property.FindPropertyRelative("_serializedValue");

            FP fpValue = FP.FromRaw(rawProp.longValue);
            fpValue = EditorGUI.FloatField(position, label, (float)fpValue);

            rawProp.longValue = fpValue.RawValue;
            EditorUtility.SetDirty(rawProp.serializedObject.targetObject);

            EditorGUI.EndProperty();
        }

    }

}