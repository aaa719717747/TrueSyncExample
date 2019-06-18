using System.Collections.Generic;

namespace TrueSync.Physics2D {

    internal class DynamicTreeBroadPhaseClone2D {

        public int[] _moveBuffer;
        public int _moveCapacity;
        public int _moveCount;

        public List<Pair> _pairBuffer = new List<Pair>();
        public int _pairCapacity;
        public int _pairCount;
        public int _proxyCount;
        public int _queryProxyId;

        public DynamicTreeClone2D dynamicTreeClone = new DynamicTreeClone2D();

        public void Clone(DynamicTreeBroadPhase dynamicTreeBroadPhase) {
            this._moveBuffer = dynamicTreeBroadPhase._moveBuffer;
            this._moveCapacity = dynamicTreeBroadPhase._moveCapacity;
            this._moveCount = dynamicTreeBroadPhase._moveCount;

            this._pairBuffer.Clear();
            for (int index = 0, length = dynamicTreeBroadPhase._pairBuffer.Length; index < length; index++) {
                this._pairBuffer.Add(dynamicTreeBroadPhase._pairBuffer[index]);
            }

            this._pairCapacity = dynamicTreeBroadPhase._pairCapacity;
            this._pairCount = dynamicTreeBroadPhase._pairCount;
            this._proxyCount = dynamicTreeBroadPhase._proxyCount;
            this._queryProxyId = dynamicTreeBroadPhase._queryProxyId;

            dynamicTreeClone.Clone(dynamicTreeBroadPhase._tree);
        }

		public void Restore(DynamicTreeBroadPhase dynamicTreeBroadPhase) {
            dynamicTreeBroadPhase._moveBuffer = this._moveBuffer;
            dynamicTreeBroadPhase._moveCapacity = this._moveCapacity;
            dynamicTreeBroadPhase._moveCount = this._moveCount;

            if (dynamicTreeBroadPhase._pairBuffer.Length != this._pairBuffer.Count) {
                dynamicTreeBroadPhase._pairBuffer = new Pair[this._pairBuffer.Count];
            }

            for (int index = 0, length = this._pairBuffer.Count; index < length; index++) {
                dynamicTreeBroadPhase._pairBuffer[index] = this._pairBuffer[index];
            }

            dynamicTreeBroadPhase._pairCapacity = this._pairCapacity;
            dynamicTreeBroadPhase._pairCount = this._pairCount;
            dynamicTreeBroadPhase._proxyCount = this._proxyCount;
            dynamicTreeBroadPhase._queryProxyId = this._queryProxyId;

            dynamicTreeClone.Restore(dynamicTreeBroadPhase._tree);
        }

    }

}