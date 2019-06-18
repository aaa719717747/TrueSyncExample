using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace TrueSync
{
	[Serializable]
	public class ReplayRecord
	{
		public static ReplayRecordSave ReplayRecordSave;

		public static ReplayRecord replayToLoad;

		public static ReplayMode replayMode = ReplayMode.NO_REPLAY;

		[SerializeField]
		private SerializableDictionaryBytePlayer players = new SerializableDictionaryBytePlayer();

		internal void AddSyncedData(List<SyncedData> data)
		{
			int i = 0;
			int count = data.Count;
			while (i < count)
			{
				SyncedData syncedData = data[i].clone();
				this.players[syncedData.inputData.ownerID].AddData(syncedData);
				i++;
			}
		}

		internal void AddPlayer(TSPlayer player)
		{
			this.players[player.ID] = new TSPlayer(player.ID, player.playerInfo.name);
		}

		public static void SaveRecord(ReplayRecord replay)
		{
			bool flag = ReplayRecord.ReplayRecordSave == null;
			if (!flag)
			{
				try
				{
					ReplayRecord.ReplayRecordSave(ReplayRecord.ToReplayBytes(replay), replay.players.Count);
				}
				catch (Exception)
				{
				}
			}
		}

		public static ReplayRecord GetReplayRecord(byte[] replayContent)
		{
			ReplayRecord result = null;
			try
			{
				result = ReplayRecord.FromReplayBytes(replayContent);
			}
			catch (Exception)
			{
			}
			return result;
		}

		private static byte[] ToReplayBytes(ReplayRecord replay)
		{
			List<byte> list = new List<byte>();
			Dictionary<byte, TSPlayer>.Enumerator enumerator = replay.players.GetEnumerator();
			while (enumerator.MoveNext())
			{
				KeyValuePair<byte, TSPlayer> current = enumerator.Current;
				TSPlayer value = current.Value;
				list.Add(value.playerInfo.id);
				Utils.GetBytes(value.playerInfo.name.Length, list);
				list.AddRange(Encoding.ASCII.GetBytes(value.playerInfo.name));
				Utils.GetBytes(value.controls.Count, list);
				Dictionary<int, SyncedData>.Enumerator enumerator2 = value.controls.GetEnumerator();
				while (enumerator2.MoveNext())
				{
					KeyValuePair<int, SyncedData> current2 = enumerator2.Current;
					Utils.GetBytes(current2.Key, list);
					current2 = enumerator2.Current;
					current2.Value.inputData.Serialize(list);
				}
			}
			return list.ToArray();
		}

		private static ReplayRecord FromReplayBytes(byte[] replayBytes)
		{
			ReplayRecord replayRecord = new ReplayRecord();
			int i = 0;
			while (i < replayBytes.Length)
			{
				byte b = replayBytes[i++];
				int num = BitConverter.ToInt32(replayBytes, i);
				i += 4;
				string @string = Encoding.ASCII.GetString(replayBytes, i, num);
				i += num;
				TSPlayer tSPlayer = new TSPlayer(b, @string);
				replayRecord.players.Add(b, tSPlayer);
				int num2 = BitConverter.ToInt32(replayBytes, i);
				i += 4;
				for (int j = 0; j < num2; j++)
				{
					SyncedData @new = SyncedData.pool.GetNew();
					@new.tick = BitConverter.ToInt32(replayBytes, i);
					i += 4;
					@new.inputData.Deserialize(replayBytes, ref i);
					@new.inputData.ownerID = b;
					tSPlayer.controls.Add(@new.tick, @new);
				}
			}
			return replayRecord;
		}

		internal void ApplyRecord(AbstractLockstep lockStep)
		{
			foreach (KeyValuePair<byte, TSPlayer> current in this.players)
			{
				bool flag = lockStep.localPlayer == null;
				if (flag)
				{
					lockStep.localPlayer = current.Value;
				}
				lockStep.players.Add(current.Key, current.Value);
				lockStep.activePlayers.Add(current.Value);
				lockStep.UpdateActivePlayers();
			}
		}
	}
}
