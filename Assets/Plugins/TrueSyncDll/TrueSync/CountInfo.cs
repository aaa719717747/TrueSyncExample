using System;

namespace TrueSync
{
	public class CountInfo
	{
		public long sum;

		public long count;

		public float average()
		{
			bool flag = this.count == 0L;
			float result;
			if (flag)
			{
				result = 0f;
			}
			else
			{
				result = (float)this.sum / (float)this.count;
			}
			return result;
		}

		public string averageFormatted()
		{
			return this.average().ToString("F2");
		}
	}
}
