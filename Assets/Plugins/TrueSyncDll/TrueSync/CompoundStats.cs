using System;

namespace TrueSync
{
	public class CompoundStats
	{
		private const float BUFFER_LIFETIME = 2f;

		private const int BUFFER_WINDOW = 10;

		public Stats globalStats;

		public GenericBufferWindow<Stats> bufferStats;

		private float timerAcc;

		public CompoundStats()
		{
			this.bufferStats = new GenericBufferWindow<Stats>(10);
			this.globalStats = new Stats();
			this.timerAcc = 0f;
		}

		public void UpdateTime(float time)
		{
			this.timerAcc += time;
			bool flag = this.timerAcc >= 2f;
			if (flag)
			{
				this.bufferStats.MoveNext();
				this.bufferStats.Current().Clear();
				this.timerAcc = 0f;
			}
		}

		public void AddValue(string key, long value)
		{
			this.bufferStats.Current().AddValue(key, value);
			this.globalStats.AddValue(key, value);
		}

		public void Increment(string key)
		{
			this.bufferStats.Current().Increment(key);
			this.globalStats.Increment(key);
		}
	}
}
