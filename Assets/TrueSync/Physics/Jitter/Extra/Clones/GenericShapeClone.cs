namespace TrueSync.Physics3D {

    /**
    * @brief A generic shape clone.
    **/
    public class GenericShapeClone {

        public TSMatrix inertia;
        public FP mass;

        public TSBBox boundingBox;
        public TSVector geomCen;

        public TSVector vector1, vector2;

        public FP fp1, fp2, fp3;

        public void Clone(Shape sh) {
            this.inertia = sh.inertia;
            this.mass = sh.mass;
            this.boundingBox = sh.boundingBox;
            this.geomCen = sh.geomCen;

            if (sh is BoxShape) {
                CloneBox((BoxShape) sh);
            } else if (sh is SphereShape) {
                CloneSphere((SphereShape) sh);
            } else if (sh is ConeShape) {
                CloneCone((ConeShape) sh);
            } else if (sh is CylinderShape) {
                CloneCylinder((CylinderShape) sh);
            } else if (sh is CapsuleShape) {
                CloneCapsule((CapsuleShape)sh);
            }
        }

        private void CloneBox(BoxShape sh) {
            this.vector1 = sh.size;
            this.vector2 = sh.halfSize;
        }

        private void CloneSphere(SphereShape sh) {
            this.fp1 = sh.radius;
        }

        private void CloneCone(ConeShape sh) {
            this.fp1 = sh.radius;
            this.fp2 = sh.height;
            this.fp3 = sh.sina;
        }

        private void CloneCylinder(CylinderShape sh) {
            this.fp1 = sh.radius;
            this.fp2 = sh.height;
        }

        private void CloneCapsule(CapsuleShape sh) {
            this.fp1 = sh.radius;
            this.fp2 = sh.length;
        }

        public void Restore(Shape sh) {
            sh.inertia = this.inertia;
            sh.mass = this.mass;
            sh.boundingBox = this.boundingBox;
            sh.geomCen = this.geomCen;

            if (sh is BoxShape) {
                RestoreBox((BoxShape)sh);
            } else if (sh is SphereShape) {
                RestoreSphere((SphereShape)sh);
            } else if (sh is ConeShape) {
                RestoreCone((ConeShape)sh);
            } else if (sh is CylinderShape) {
                RestoreCylinder((CylinderShape)sh);
            } else if (sh is CapsuleShape) {
                RestoreCapsule((CapsuleShape)sh);
            }
        }

        private void RestoreBox(BoxShape sh) {
            sh.size = this.vector1;
            sh.halfSize = this.vector2;
        }

        private void RestoreSphere(SphereShape sh) {
            sh.radius = this.fp1;
        }

        private void RestoreCone(ConeShape sh) {
            sh.radius = this.fp1;
            sh.height = this.fp2;
            sh.sina = this.fp3;
        }

        private void RestoreCylinder(CylinderShape sh) {
            sh.radius = this.fp1;
            sh.height = this.fp2;
        }

        private void RestoreCapsule(CapsuleShape sh) {
            sh.radius = this.fp1;
            sh.length = this.fp2;
        }

    }

}