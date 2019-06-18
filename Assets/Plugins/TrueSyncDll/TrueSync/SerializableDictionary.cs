using System;
using System.Collections.Generic;
using UnityEngine;

namespace TrueSync
{
	[Serializable]
	public class SerializableDictionary<TKey, TValue> : Dictionary<TKey, TValue>, ISerializationCallbackReceiver
	{
		[SerializeField]
		private List<TKey> keys = new List<TKey>();

		[SerializeField]
		private List<TValue> values = new List<TValue>();

		public void OnBeforeSerialize()
		{
			this.keys.Clear();
			this.values.Clear();
			foreach (KeyValuePair<TKey, TValue> current in this)
			{
				this.keys.Add(current.Key);
				this.values.Add(current.Value);
			}
		}

		public void OnAfterDeserialize()
		{
			base.Clear();
			bool flag = this.keys.Count != this.values.Count;
			if (flag)
			{
				throw new Exception(string.Format("there are {0} keys and {1} values after deserialization. Make sure that both key and value types are serializable.", new object[0]));
			}
			for (int i = 0; i < this.keys.Count; i++)
			{
				base.Add(this.keys[i], this.values[i]);
			}
		}
	}
}
