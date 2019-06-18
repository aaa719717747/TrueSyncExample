using System.IO;
using UnityEditor;
using UnityEngine;

namespace TrueSync {

    /**
    * @brief Represents the TrueSync menu context bar.
    **/
    public class MenuContext {

        private static string ASSETS_PREFABS_PATH = "Assets/TrueSync/Unity/Prefabs/{0}.prefab";

        private static void InstantiatePrefab(string path) {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(string.Format(ASSETS_PREFABS_PATH, path));
            PrefabUtility.InstantiatePrefab(prefab);
        }

        private static void CreateTrueSyncConfigAsset() {
            TrueSyncConfig asset = ScriptableObject.CreateInstance<TrueSyncConfig>();

            string path = AssetDatabase.GetAssetPath(Selection.activeObject);
            if (path == "") {
                path = "Assets";
            } else if (Path.GetExtension(path) != "") {
                path = path.Replace(Path.GetFileName(AssetDatabase.GetAssetPath(Selection.activeObject)), "");
            }

            string assetPathAndName = AssetDatabase.GenerateUniqueAssetPath(path + "/TrueSyncConfig.asset");

            AssetDatabase.CreateAsset(asset, assetPathAndName);

            AssetDatabase.SaveAssets();
            EditorUtility.FocusProjectWindow();
            Selection.activeObject = asset;
        }

        [MenuItem("Assets/Create/TrueSyncConfig", false, 0)]
        static void CreateTrueSyncConfig() {
            CreateTrueSyncConfigAsset();
        }

        [MenuItem("GameObject/TrueSync/Cube", false, 0)]
        static void CreatePrefabCube() {
            InstantiatePrefab("Basic/Cube");
        }

        [MenuItem("GameObject/TrueSync/Sphere", false, 0)]
        static void CreatePrefabSphere() {
            InstantiatePrefab("Basic/Sphere");
        }

        [MenuItem("GameObject/TrueSync/Capsule", false, 0)]
        static void CreatePrefabCapsule() {
            InstantiatePrefab("Basic/Capsule");
        }

        [MenuItem("GameObject/TrueSync/TrueSyncManager", false, 11)]
        static void CreatePrefabTrueSync() {            
            InstantiatePrefab("TrueSyncManager");
        }

    }

}