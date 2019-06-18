using System;

namespace TrueSync
{
	internal class ResourcePoolSyncedData : ResourcePool<SyncedData>
	{
		protected override SyncedData NewInstance()
		{
			return new SyncedData();
		}

		public void FillStack(int instances)
		{
			for (int i = 0; i < instances; i++)
			{
				this.stack.Push(new SyncedData());
			}
		}
	}
}
