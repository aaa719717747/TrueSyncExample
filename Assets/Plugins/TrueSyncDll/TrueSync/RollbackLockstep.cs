using System;
using System.Collections.Generic;
using UnityEngine;

namespace TrueSync
{
	internal class RollbackLockstep : AbstractLockstep
	{
		private GenericBufferWindow<IWorldClone> bufferWorldClone;

		internal int rollbackIndex;

		internal int rollbackIndexOffset;

		public RollbackLockstep(float deltaTime, ICommunicator communicator, IPhysicsManagerBase physicsManager, int syncWindow, int panicTime, int rollbackWindow, TrueSyncEventCallback OnGameStarted, TrueSyncEventCallback OnGamePaused, TrueSyncEventCallback OnGameUnPaused, TrueSyncEventCallback OnGameEnded, TrueSyncPlayerDisconnectionCallback OnPlayerDisconnection, TrueSyncUpdateCallback OnStepUpdate, TrueSyncInputCallback GetLocalData, TrueSyncInputDataProvider InputDataProvider) : base(deltaTime, communicator, physicsManager, syncWindow, panicTime, rollbackWindow, OnGameStarted, OnGamePaused, OnGameUnPaused, OnGameEnded, OnPlayerDisconnection, OnStepUpdate, GetLocalData, InputDataProvider)
		{
			this.bufferWorldClone = new GenericBufferWindow<IWorldClone>(rollbackWindow, new GenericBufferWindow<IWorldClone>.NewInstance(physicsManager.GetWorldClone));
			this.rollbackIndex = rollbackWindow;
		}

		protected override int GetRefTick(int syncedDataTick)
		{
			return syncedDataTick - this.rollbackWindow;
		}

		protected override void OnSyncedDataReceived(TSPlayer player, List<SyncedData> data)
		{
			player.AddDataRollback(data);
		}

		protected override string GetChecksumForSyncedInfo()
		{
			return this.bufferWorldClone.Current().checksum;
		}

		protected override bool IsStepReady(int syncedDataTick)
		{
			bool flag = this.localPlayer.IsDataReady(syncedDataTick);
			int num = syncedDataTick - this.rollbackWindow;
			bool flag2 = num >= 0;
			if (flag2)
			{
				int i = 0;
				int count = this.activePlayers.Count;
				while (i < count)
				{
					flag = (flag && this.activePlayers[i].IsDataReady(num));
					i++;
				}
			}
			return flag;
		}

		protected override void BeforeStepUpdate(int syncedDataTick, int referenceTick)
		{
			base.BeforeStepUpdate(syncedDataTick, referenceTick);
			this.rollbackIndexOffset = 0;
			int num = this.rollbackWindow;
			bool flag = false;
			bool flag2 = referenceTick >= 0;
			if (flag2)
			{
				bool flag3 = this.bodiesToDestroy.ContainsKey(referenceTick);
				if (flag3)
				{
					flag = true;
				}
				else
				{
					for (int i = 0; i < this.rollbackWindow; i++)
					{
						int j = 0;
						int count = this.activePlayers.Count;
						while (j < count)
						{
							flag = (flag || this.activePlayers[j].IsDataDirty(referenceTick));
							j++;
						}
						bool flag4 = flag;
						if (flag4)
						{
							break;
						}
						this.rollbackIndexOffset++;
						referenceTick++;
						num--;
						this.bufferWorldClone.MoveNext();
						StateTracker.instance.MoveNextState();
					}
				}
			}
			bool flag5 = flag;
			if (flag5)
			{
				this.compoundStats.Increment("rollback");
				this.Rollback(referenceTick, num);
			}
			this.rollbackIndexOffset = 0;
			this.SaveWorldClone(syncedDataTick);
		}

		private void Rollback(int rollbackTick, int temporaryRollbackWindow)
		{
			int i = 0;
			int count = this.activePlayers.Count;
			while (i < count)
			{
				TSPlayer tSPlayer = this.activePlayers[i];
				tSPlayer.GetData(rollbackTick).dirty = false;
				tSPlayer.AddDataProjected(rollbackTick, this.syncWindow + temporaryRollbackWindow);
				i++;
			}
			this.RestorePreviousState();
			base.CheckSafeRemotion(rollbackTick);
			this.rollbackIndex = 0;
			while (this.rollbackIndex < temporaryRollbackWindow)
			{
				List<SyncedData> tickData = base.GetTickData(rollbackTick + this.rollbackIndex);
				bool flag = this.rollbackIndex > 0;
				if (flag)
				{
					this.SaveWorldClone(rollbackTick + this.rollbackIndex);
				}
				else
				{
					this.bufferWorldClone.MoveNext();
					StateTracker.instance.MoveNextState();
				}
				base.ExecutePhysicsStep(tickData, rollbackTick + this.rollbackIndex);
				this.rollbackIndex++;
			}
			this.rollbackIndex = this.rollbackWindow;
		}

		protected override int GetSimulatedTick(int syncedDataTick)
		{
			int num = syncedDataTick - this.rollbackWindow;
			return num + this.rollbackIndex + this.rollbackIndexOffset;
		}

		private void RestorePreviousState()
		{
			StateTracker.instance.RestoreState();
			this.bufferWorldClone.Current().Restore(this.physicsManager.GetWorld());
		}

		private void SaveWorldClone(int refTicks)
		{
			this.bufferWorldClone.Current().Clone(this.physicsManager.GetWorld(), refTicks % 100 == 0);
			this.bufferWorldClone.MoveNext();
			StateTracker.instance.SaveState();
		}
	}
}
