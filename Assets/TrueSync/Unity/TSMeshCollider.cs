using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using TrueSync.Physics3D;

namespace TrueSync {

    /**
     *  @brief Collider with a mesh shape. 
     **/
    [AddComponentMenu("TrueSync/Physics/MeshCollider", 0)]
    public class TSMeshCollider : TSCollider {

        [SerializeField]
        private Mesh mesh;

        /**
         *  @brief Mesh attached to the same game object. 
         **/
        public Mesh Mesh {
            get { return mesh; }
            set {
                mesh = value;
                vertices = GetVertices();
                indices = GetIndices();

                if (_body != null) {
                    _body.Shape = CreateShape();
                }
            }
        }

        private List<TSVector> vertices;

        /**
         *  @brief A list of all mesh's vertices. 
         **/
        public List<TSVector> Vertices {
            get {
                if (vertices == null)
                    vertices = GetVertices();
                return vertices;
            }
        }

        private List<TriangleVertexIndices> indices;

        /**
         *  @brief A list of mess related structs. 
         **/
        public List<TriangleVertexIndices> Indices {
            get {
                if (indices == null)
                    indices = GetIndices();
                return indices;
            }
        }

        /**
         *  @brief Gets (if any) the mesh attached to this game object. 
         **/
        public void Reset() {
            if (mesh == null) {
                var meshFilter = GetComponent<MeshFilter>();
                mesh = meshFilter.sharedMesh;
            }
        }

        /**
         *  @brief Creates a shape based on attached mesh. 
         **/
        public override Shape CreateShape() {
            var octree = new Octree(Vertices, Indices);
            return new TriangleMeshShape(octree);
        }

        private List<TriangleVertexIndices> GetIndices() {
            var triangles = mesh.triangles;
            var result = new List<TriangleVertexIndices>();
            for (int i = 0; i < triangles.Length; i += 3)
                result.Add(new TriangleVertexIndices(triangles[i + 2], triangles[i + 1], triangles[i + 0]));
            return result;
        }

        private List<TSVector> GetVertices() {
            var result = mesh.vertices.Select(p => new TSVector(p.x * lossyScale.x, p.y * lossyScale.y, p.z * lossyScale.z)).ToList();
            return result;
        }

        protected override Vector3 GetGizmosSize() {
            return lossyScale.ToVector();
        }

        protected override void DrawGizmos() {
            Gizmos.DrawWireMesh(mesh);
        }

    }

}