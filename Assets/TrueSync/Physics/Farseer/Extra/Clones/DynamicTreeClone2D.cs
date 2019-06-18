using System.Collections.Generic;

namespace TrueSync.Physics2D {

    internal class DynamicTreeClone2D {

        public List<int> _raycastStack = new List<int>(256);
        public List<int> _queryStack = new List<int>(256);

        public int _freeList;
        public int _nodeCapacity;
        public int _nodeCount;
        public List<TreeNode<Physics2D.FixtureProxy>> _nodes = new List<Physics2D.TreeNode<FixtureProxy>>();
        public int _root;

        private int index, length;

        public void Clone(DynamicTree<Physics2D.FixtureProxy> dynamicTree) {
            this._raycastStack.Clear();

            var raycastEnum = dynamicTree._raycastStack.GetEnumerator();
            while (raycastEnum.MoveNext()) {
                this._raycastStack.Add(raycastEnum.Current);
            }

            this._queryStack.Clear();

            var queryEnum = dynamicTree._queryStack.GetEnumerator();
            while (queryEnum.MoveNext()) {
                this._queryStack.Add(queryEnum.Current);
            }

            this._freeList = dynamicTree._freeList;
            this._nodeCapacity = dynamicTree._nodeCapacity;
            this._nodeCount = dynamicTree._nodeCount;
            this._root = dynamicTree._root;

            for (index = 0, length = _nodes.Count; index < length; index++) {
                WorldClone2D.poolTreeFixtureProxy.GiveBack(_nodes[index]);
            }

            this._nodes.Clear();

            for (index = 0, length = dynamicTree._nodes.Length; index < length; index++) {
                TreeNode<Physics2D.FixtureProxy> tn = dynamicTree._nodes[index];

                TreeNode<Physics2D.FixtureProxy> cloneTn = WorldClone2D.poolTreeFixtureProxy.GetNew();
                cloneTn.AABB = tn.AABB;
                cloneTn.Child1 = tn.Child1;
                cloneTn.Child2 = tn.Child2;
                cloneTn.Height = tn.Height;
                cloneTn.ParentOrNext = tn.ParentOrNext;
                cloneTn.UserData = tn.UserData;

                this._nodes.Add(cloneTn);
            }
        }

		public void Restore(DynamicTree<Physics2D.FixtureProxy> dynamicTree) {
            dynamicTree._raycastStack.Clear();

            for (index = 0, length = _raycastStack.Count; index < length; index++) {
                dynamicTree._raycastStack.Push(_raycastStack[index]);
            }

            dynamicTree._queryStack.Clear();

            for (index = 0, length = _queryStack.Count; index < length; index++) {
                dynamicTree._queryStack.Push(_queryStack[index]);
            }

            dynamicTree._freeList = this._freeList;
            dynamicTree._nodeCapacity = this._nodeCapacity;
            dynamicTree._nodeCount = this._nodeCount;
            dynamicTree._root = this._root;

            dynamicTree._nodes = new TreeNode<FixtureProxy>[this._nodes.Count];

            for (index = 0, length = this._nodes.Count; index < length; index++) {
                TreeNode<FixtureProxy> cloneTn = this._nodes[index];

                TreeNode<FixtureProxy> newTn = WorldClone2D.poolTreeFixtureProxy.GetNew();
                newTn.AABB = cloneTn.AABB;
                newTn.Child1 = cloneTn.Child1;
                newTn.Child2 = cloneTn.Child2;
                newTn.Height = cloneTn.Height;
                newTn.ParentOrNext = cloneTn.ParentOrNext;
                newTn.UserData = cloneTn.UserData;

                dynamicTree._nodes[index] = newTn;
            }
        }

    }

}