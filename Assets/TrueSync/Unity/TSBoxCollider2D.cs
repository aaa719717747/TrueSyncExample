using UnityEngine;
using UnityEngine.Serialization;

namespace TrueSync {
    /**
     *  @brief Collider with a box 2D shape. 
     **/
    [AddComponentMenu("TrueSync/Physics/BoxCollider2D", 0)]
    public class TSBoxCollider2D : TSCollider2D {

        [FormerlySerializedAs("size")]
        [SerializeField]
        private TSVector2 _size = TSVector2.one;

        /**
         *  @brief Size of the box. 
         **/
        public TSVector2 size {
            get {
                if (_body != null) {
                    TSVector2 halfVector = ((Physics2D.PolygonShape)_body.FixtureList[0].Shape).Vertices[0] * 2;
                    halfVector.x /= lossyScale.x;
                    halfVector.y /= -lossyScale.y;

                    return halfVector;
                }

                return _size;
            }
            set {
                _size = value;

                if (_body != null) {
                    TSVector size3 = new TSVector(_size.x, _size.y, 1);
                    TSVector sizeScaled = TSVector.Scale(size3, lossyScale);

                    ((Physics2D.PolygonShape)_body.FixtureList[0].Shape).Vertices = TrueSync.Physics2D.PolygonTools.CreateRectangle(sizeScaled.x * FP.Half, sizeScaled.y * FP.Half);
                }

            }
        }

        /**
         *  @brief Sets initial values to {@link #size} based on a pre-existing BoxCollider or BoxCollider2D.
         **/
        public void Reset() {
            if (GetComponent<BoxCollider2D>() != null) {
                BoxCollider2D boxCollider2D = GetComponent<BoxCollider2D>();

                size = new TSVector2(boxCollider2D.size.x, boxCollider2D.size.y);
                Center = new TSVector2(boxCollider2D.offset.x, boxCollider2D.offset.y);
                isTrigger = boxCollider2D.isTrigger;
            } else if (GetComponent<BoxCollider>() != null) {
                BoxCollider boxCollider = GetComponent<BoxCollider>();

                size = boxCollider.size.ToTSVector2();
                Center = boxCollider.center.ToTSVector2();
                isTrigger = boxCollider.isTrigger;
            }
        }

        /**
         *  @brief Create the internal shape used to represent a TSBoxCollider.
         **/
        public override TrueSync.Physics2D.Shape CreateShape() {
            TSVector size3 = new TSVector(size.x, size.y, 1);
            TSVector sizeScaled = TSVector.Scale(size3, lossyScale);
            return new TrueSync.Physics2D.PolygonShape(TrueSync.Physics2D.PolygonTools.CreateRectangle(sizeScaled.x * FP.Half, sizeScaled.y * FP.Half), 1);
        }

        protected override void DrawGizmos() {
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one);
        }

        protected override Vector3 GetGizmosSize() {
            TSVector size3 = new TSVector(size.x, size.y, 1);
            return TSVector.Scale(size3, lossyScale).ToVector();
        }

    }

}