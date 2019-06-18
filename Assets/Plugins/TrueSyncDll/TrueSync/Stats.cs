using System;
using System.Collections.Generic;

namespace TrueSync
{
	public class Stats
	{
		private Dictionary<string, CountInfo> counts = new Dictionary<string, CountInfo>();

		private static CountInfo emptyInfo = new CountInfo();

		public void Clear()
		{
			this.counts.Clear();
		}

		public void Increment(string key)
		{
			bool flag = !this.counts.ContainsKey(key);
			if (flag)
			{
				this.counts[key] = new CountInfo();
			}
			this.counts[key].count += 1L;
		}

		public void AddValue(string key, long value)
		{
			this.Increment(key);
			this.counts[key].sum += value;
		}

		public CountInfo GetInfo(string key)
		{
			bool flag = this.counts.ContainsKey(key);
			CountInfo result;
			if (flag)
			{
				result = this.counts[key];
			}
			else
			{
				result = Stats.emptyInfo;
			}
			return result;
		}
	}
}
