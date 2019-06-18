using System;
using System.Collections.Generic;

namespace TrueSync
{
	internal class DefaultLockstep : AbstractLockstep
	{
		public DefaultLockstep(float deltaTime, ICommunicator communicator, IPhysicsManagerBase physicsManager, int syncWindow, int panicTime, int rollbackWindow, TrueSyncEventCallback OnGameStarted, TrueSyncEventCallback OnGamePaused, TrueSyncEventCallback OnGameUnPaused, TrueSyncEventCallback OnGameEnded, TrueSyncPlayerDisconnectionCallback OnPlayerDisconnection, TrueSyncUpdateCallback OnStepUpdate, TrueSyncInputCallback GetLocalData, TrueSyncInputDataProvider InputDataProvider) : base(deltaTime, communicator, physicsManager, syncWindow, panicTime, rollbackWindow, OnGameStarted, OnGamePaused, OnGameUnPaused, OnGameEnded, OnPlayerDisconnection, OnStepUpdate, GetLocalData, InputDataProvider)
		{
		}

		protected override void OnSyncedDataReceived(TSPlayer player, List<SyncedData> data)
		{
			player.AddData(data);
		}

		protected override int GetRefTick(int syncedDataTick)
		{
			return syncedDataTick;
		}

		protected override void BeforeStepUpdate(int syncedDataTick, int referenceTick)
		{
			base.BeforeStepUpdate(syncedDataTick, referenceTick);
			base.CheckSafeRemotion(syncedDataTick);
		}

		protected override int GetSimulatedTick(int syncedDataTick)
		{
			return syncedDataTick;
		}

		protected override string GetChecksumForSyncedInfo()
		{
			return ChecksumExtractor.GetEncodedChecksum();
		}

		protected override bool IsStepReady(int syncedDataTick)
		{
			bool flag = true;
			int i = 0;
			int count = this.activePlayers.Count;
			while (i < count)
			{
				flag = (flag && this.activePlayers[i].IsDataReady(syncedDataTick));
				i++;
			}
			return flag;
		}
	}
}
