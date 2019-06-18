using System;

namespace TrueSync.Physics2D
{
    public interface IBroadPhase
    {
        int ProxyCount { get; }
        void UpdatePairs(BroadphaseDelegate callback);

        bool TestOverlap(int proxyIdA, int proxyIdB);

        int AddProxy(ref FixtureProxy proxy);

        void RemoveProxy(int proxyId);

        void MoveProxy(int proxyId, ref AABB aabb, TSVector2 displacement);

        FixtureProxy GetProxy(int proxyId);

        void TouchProxy(int proxyId);

        void GetFatAABB(int proxyId, out AABB aabb);

        void Query(Func<int, bool> callback, ref AABB aabb);

        void RayCast(Func<RayCastInput, int, FP> callback, ref RayCastInput input);

        void ShiftOrigin(TSVector2 newOrigin);
    }
}