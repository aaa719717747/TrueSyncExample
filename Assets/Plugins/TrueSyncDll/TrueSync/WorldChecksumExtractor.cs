using System;
using System.Collections.Generic;
using System.Text;

namespace TrueSync
{
	public class WorldChecksumExtractor : ChecksumExtractor
	{
		private StringBuilder sb = new StringBuilder();

		public WorldChecksumExtractor(IPhysicsManagerBase physicsManager) : base(physicsManager)
		{
		}

		protected override string GetChecksum()
		{
			this.sb.Length = 0;
			List<IBody> list = this.physicsManager.GetWorld().Bodies();
			int i = 0;
			int count = list.Count;
			while (i < count)
			{
				IBody body = list[i];
				this.sb.Append(body.Checkum());
				this.sb.Append("|");
				i++;
			}
			return this.sb.ToString();
		}
	}
}
