using System;
using System.Collections.Generic;

namespace TrueSync
{
    // 帧同步托管行为类
	public class TrueSyncManagedBehaviour : ITrueSyncBehaviourGamePlay, ITrueSyncBehaviour, ITrueSyncBehaviourCallbacks
	{
		public ITrueSyncBehaviour trueSyncBehavior;

		[AddTracking]
		public bool disabled;

		public TSPlayerInfo localOwner;

		public TSPlayerInfo owner;

		public TrueSyncManagedBehaviour(ITrueSyncBehaviour trueSyncBehavior)
		{
			StateTracker.AddTracking(this);
			StateTracker.AddTracking(trueSyncBehavior);
			this.trueSyncBehavior = trueSyncBehavior;
		}

        #region ITrueSyncBehaviourGamePlay 接口方法
        public void OnPreSyncedUpdate()
		{
			bool flag = this.trueSyncBehavior is ITrueSyncBehaviourGamePlay;
			if (flag)
			{
				((ITrueSyncBehaviourGamePlay)this.trueSyncBehavior).OnPreSyncedUpdate();
			}
		}

		public void OnSyncedInput()
		{
			bool flag = this.trueSyncBehavior is ITrueSyncBehaviourGamePlay;
			if (flag)
			{
				((ITrueSyncBehaviourGamePlay)this.trueSyncBehavior).OnSyncedInput();
			}
		}

		public void OnSyncedUpdate()
		{
			bool flag = this.trueSyncBehavior is ITrueSyncBehaviourGamePlay;
			if (flag)
			{
				((ITrueSyncBehaviourGamePlay)this.trueSyncBehavior).OnSyncedUpdate();
			}
		}
        #endregion ITrueSyncBehaviourGamePlay 接口方法

        #region ITrueSyncBehaviour 接口方法
        public void SetGameInfo(TSPlayerInfo localOwner, int numberOfPlayers)
		{
			this.trueSyncBehavior.SetGameInfo(localOwner, numberOfPlayers);
		}
        #endregion ITrueSyncBehaviour 接口方法

        #region 生命周期方法
        // 开始同步
        public static void OnGameStarted(List<TrueSyncManagedBehaviour> generalBehaviours, Dictionary<byte, List<TrueSyncManagedBehaviour>> behaviorsByPlayer)
        {
            int i = 0;
            int count = generalBehaviours.Count;
            while (i < count)
            {
                generalBehaviours[i].OnSyncedStart();
                i++;
            }
            Dictionary<byte, List<TrueSyncManagedBehaviour>>.Enumerator enumerator = behaviorsByPlayer.GetEnumerator();
            while (enumerator.MoveNext())
            {
                KeyValuePair<byte, List<TrueSyncManagedBehaviour>> current = enumerator.Current;
                List<TrueSyncManagedBehaviour> value = current.Value;
                int j = 0;
                int count2 = value.Count;
                while (j < count2)
                {
                    value[j].OnSyncedStart();
                    j++;
                }
            }
        }

        // 游戏暂停
        public static void OnGamePaused(List<TrueSyncManagedBehaviour> generalBehaviours, Dictionary<byte, List<TrueSyncManagedBehaviour>> behaviorsByPlayer)
        {
            int i = 0;
            int count = generalBehaviours.Count;
            while (i < count)
            {
                generalBehaviours[i].OnGamePaused();
                i++;
            }
            Dictionary<byte, List<TrueSyncManagedBehaviour>>.Enumerator enumerator = behaviorsByPlayer.GetEnumerator();
            while (enumerator.MoveNext())
            {
                KeyValuePair<byte, List<TrueSyncManagedBehaviour>> current = enumerator.Current;
                List<TrueSyncManagedBehaviour> value = current.Value;
                int j = 0;
                int count2 = value.Count;
                while (j < count2)
                {
                    value[j].OnGamePaused();
                    j++;
                }
            }
        }

        // 取消暂停
        public static void OnGameUnPaused(List<TrueSyncManagedBehaviour> generalBehaviours, Dictionary<byte, List<TrueSyncManagedBehaviour>> behaviorsByPlayer)
        {
            int i = 0;
            int count = generalBehaviours.Count;
            while (i < count)
            {
                generalBehaviours[i].OnGameUnPaused();
                i++;
            }
            Dictionary<byte, List<TrueSyncManagedBehaviour>>.Enumerator enumerator = behaviorsByPlayer.GetEnumerator();
            while (enumerator.MoveNext())
            {
                KeyValuePair<byte, List<TrueSyncManagedBehaviour>> current = enumerator.Current;
                List<TrueSyncManagedBehaviour> value = current.Value;
                int j = 0;
                int count2 = value.Count;
                while (j < count2)
                {
                    value[j].OnGameUnPaused();
                    j++;
                }
            }
        }

        // 游戏结束
        public static void OnGameEnded(List<TrueSyncManagedBehaviour> generalBehaviours, Dictionary<byte, List<TrueSyncManagedBehaviour>> behaviorsByPlayer)
        {
            int i = 0;
            int count = generalBehaviours.Count;
            while (i < count)
            {
                generalBehaviours[i].OnGameEnded();
                i++;
            }
            Dictionary<byte, List<TrueSyncManagedBehaviour>>.Enumerator enumerator = behaviorsByPlayer.GetEnumerator();
            while (enumerator.MoveNext())
            {
                KeyValuePair<byte, List<TrueSyncManagedBehaviour>> current = enumerator.Current;
                List<TrueSyncManagedBehaviour> value = current.Value;
                int j = 0;
                int count2 = value.Count;
                while (j < count2)
                {
                    value[j].OnGameEnded();
                    j++;
                }
            }
        }

        // 玩家断开连接
        public static void OnPlayerDisconnection(List<TrueSyncManagedBehaviour> generalBehaviours, Dictionary<byte, List<TrueSyncManagedBehaviour>> behaviorsByPlayer, byte playerId)
        {
            int i = 0;
            int count = generalBehaviours.Count;
            while (i < count)
            {
                generalBehaviours[i].OnPlayerDisconnection((int)playerId);
                i++;
            }
            Dictionary<byte, List<TrueSyncManagedBehaviour>>.Enumerator enumerator = behaviorsByPlayer.GetEnumerator();
            while (enumerator.MoveNext())
            {
                KeyValuePair<byte, List<TrueSyncManagedBehaviour>> current = enumerator.Current;
                List<TrueSyncManagedBehaviour> value = current.Value;
                int j = 0;
                int count2 = value.Count;
                while (j < count2)
                {
                    value[j].OnPlayerDisconnection((int)playerId);
                    j++;
                }
            }
        }
        #endregion 生命周期方法

        #region ITrueSyncBehaviourCallbacks 接口方法
        // 开始同步
        public void OnSyncedStart()
		{
			bool flag = this.trueSyncBehavior is ITrueSyncBehaviourCallbacks;
			if (flag)
			{
				((ITrueSyncBehaviourCallbacks)this.trueSyncBehavior).OnSyncedStart();
				bool flag2 = this.localOwner.Id == this.owner.Id;
				if (flag2) // 本地玩家
				{
					((ITrueSyncBehaviourCallbacks)this.trueSyncBehavior).OnSyncedStartLocalPlayer();
				}
			}
		}

        // 开始同步本地玩家
        public void OnSyncedStartLocalPlayer()
        {
            throw new NotImplementedException();
        }

        // 游戏暂停
		public void OnGamePaused()
		{
			bool flag = this.trueSyncBehavior is ITrueSyncBehaviourCallbacks;
			if (flag)
			{
				((ITrueSyncBehaviourCallbacks)this.trueSyncBehavior).OnGamePaused();
			}
		}

        // 取消暂停
		public void OnGameUnPaused()
		{
			bool flag = this.trueSyncBehavior is ITrueSyncBehaviourCallbacks;
			if (flag)
			{
				((ITrueSyncBehaviourCallbacks)this.trueSyncBehavior).OnGameUnPaused();
			}
		}

        // 游戏结束
		public void OnGameEnded()
		{
			bool flag = this.trueSyncBehavior is ITrueSyncBehaviourCallbacks;
			if (flag)
			{
				((ITrueSyncBehaviourCallbacks)this.trueSyncBehavior).OnGameEnded();
			}
		}

        // 玩家断开连接
		public void OnPlayerDisconnection(int playerId)
		{
			bool flag = this.trueSyncBehavior is ITrueSyncBehaviourCallbacks;
			if (flag)
			{
				((ITrueSyncBehaviourCallbacks)this.trueSyncBehavior).OnPlayerDisconnection(playerId);
			}
		}
        #endregion ITrueSyncBehaviourCallbacks 接口方法
    }
}
