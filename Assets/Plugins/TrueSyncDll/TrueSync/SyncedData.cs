using System;
using System.Collections.Generic;

namespace TrueSync
{
	[Serializable]
	public class SyncedData : ResourcePoolItem
	{
		internal static ResourcePoolSyncedData pool = new ResourcePoolSyncedData();

		internal static ResourcePoolListSyncedData poolList = new ResourcePoolListSyncedData();

		public InputDataBase inputData;

		public int tick;

		[NonSerialized]
		public bool fake;

		[NonSerialized]
		public bool dirty;

		[NonSerialized]
		public bool dropPlayer;

		[NonSerialized]
		public byte dropFromPlayerId;

		private static List<byte> bytesToEncode = new List<byte>();

		public SyncedData()
		{
			this.inputData = AbstractLockstep.instance.InputDataProvider();
		}

		public void Init(byte ownerID, int tick)
		{
			this.inputData.ownerID = ownerID;
			this.tick = tick;
			this.fake = false;
			this.dirty = false;
		}

		public void GetEncodedHeader(List<byte> bytes)
		{
			Utils.GetBytes(this.tick, bytes);
			bytes.Add(this.inputData.ownerID);
			bytes.Add(this.dropFromPlayerId);
			bytes.Add((byte)(this.dropPlayer ? 1 : 0));
		}

		public void GetEncodedActions(List<byte> bytes)
		{
			this.inputData.Serialize(bytes);
		}

		public static List<SyncedData> Decode(byte[] data)
		{
			List<SyncedData> @new = SyncedData.poolList.GetNew();
			@new.Clear();
			int i = 0;
			int num = BitConverter.ToInt32(data, i);
			i += 4;
			byte ownerID = data[i++];
			byte b = data[i++];
			bool flag = data[i++] == 1;
			int num2 = num;
			while (i < data.Length)
			{
				SyncedData new2 = SyncedData.pool.GetNew();
				new2.Init(ownerID, num2--);
				new2.inputData.Deserialize(data, ref i);
				@new.Add(new2);
			}
			bool flag2 = @new.Count > 0;
			if (flag2)
			{
				@new[0].dropPlayer = flag;
				@new[0].dropFromPlayerId = b;
			}
			return @new;
		}

        // ±àÂë
		public static byte[] Encode(SyncedData[] syncedData)
		{
			SyncedData.bytesToEncode.Clear();
			bool flag = syncedData.Length != 0;
			if (flag) // ÓÐÊý¾Ý
			{
				syncedData[0].GetEncodedHeader(SyncedData.bytesToEncode);
				for (int i = 0; i < syncedData.Length; i++)
				{
					syncedData[i].GetEncodedActions(SyncedData.bytesToEncode);
				}
			}
			byte[] array = new byte[SyncedData.bytesToEncode.Count];
			int j = 0;
			int num = array.Length;
			while (j < num)
			{
				array[j] = SyncedData.bytesToEncode[j];
				j++;
			}
			return array;
		}

		public SyncedData clone()
		{
			SyncedData @new = SyncedData.pool.GetNew();
			@new.Init(this.inputData.ownerID, this.tick);
			@new.inputData.CopyFrom(this.inputData);
			return @new;
		}

		public bool EqualsData(SyncedData other)
		{
			return this.inputData.EqualsData(other.inputData);
		}

		public void CleanUp()
		{
			this.inputData.CleanUp();
		}
	}
}
