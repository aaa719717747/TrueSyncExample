using UnityEngine;
using TrueSync.Physics3D;

namespace TrueSync {

    /**
     *  @brief Collider with a terrain shape. 
     **/
    [AddComponentMenu("TrueSync/Physics/TerrainCollider", 0)]
    [RequireComponent(typeof(Terrain))]
    public class TSTerrainCollider : TSCollider {

        /**
         *  @brief Terrain's resolution. 
         **/
        public int Resolution {
            get {
                var terrain = GetComponent<Terrain>();
                var data = terrain.terrainData;
                int resolusion = data.heightmapResolution;
                return resolusion;
            }
        }

        /**
         *  @brief Terrain's size. 
         **/
        public Vector3 Size {
            get {
                var terrain = GetComponent<Terrain>();
                var data = terrain.terrainData;
                return data.size;
            }
        }

        /**
         *  @brief Creates a terrain shape based on a Terrain component attached. 
         **/
        public override Shape CreateShape() {
            var terrain = GetComponent<Terrain>();
            var data = terrain.terrainData;
            int resolusion = data.heightmapResolution;
            var heightsFloat = data.GetHeights(0, 0, resolusion, resolusion);
            FP[,] heights = new FP[heightsFloat.GetLength(0), heightsFloat.GetLength(1)];

            for (int indexI = 0; indexI < heightsFloat.GetLength(0); indexI++) {
                for (int indexJ = 0; indexJ < heightsFloat.GetLength(1); indexJ++) {
                    heights[indexI, indexJ] = heightsFloat[indexI, indexJ];
                }
            }

            FP verticalScale = data.size.y;
            for (int x = 0; x < resolusion; x++) {
                for (int z = 0; z < resolusion; z++)
                    heights[x, z] *= verticalScale;
            }
            for (int x = 0; x < resolusion - 1; x++) {
                for (int z = x; z < resolusion; z++) {
                    FP h1 = heights[x, z];
                    FP h2 = heights[z, x];
                    heights[x, z] = h2;
                    heights[z, x] = h1;
                }
            }

            var result = new TerrainShape(heights, data.size.x / (resolusion - 1), data.size.z / (resolusion - 1));
            return result;
        }

        protected override Vector3 GetGizmosSize() {
            return Vector3.one;
        }

        protected override void DrawGizmos() {
        }

    }

}