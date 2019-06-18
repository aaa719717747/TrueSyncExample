using System;
using System.Collections;
using System.Collections.Generic;

namespace TrueSync
{
	public class HashList<T> : ICollection<T>, IEnumerable<T>, IEnumerable
	{
		private readonly List<T> collection = new List<T>();

		private readonly IComparer<T> comparer = Comparer<T>.Default;

		public T this[int index]
		{
			get
			{
				return this.collection[index];
			}
		}

		public int Count
		{
			get
			{
				return this.collection.Count;
			}
		}

		public bool IsReadOnly
		{
			get
			{
				return false;
			}
		}

		public void Add(T item)
		{
			bool flag = this.Count == 0;
			if (flag)
			{
				this.collection.Add(item);
			}
			else
			{
				int i = 0;
				int num = this.collection.Count - 1;
				while (i <= num)
				{
					int num2 = (i + num) / 2;
					int num3 = this.comparer.Compare(this.collection[num2], item);
					bool flag2 = num3 == 0;
					if (flag2)
					{
						return;
					}
					bool flag3 = num3 < 0;
					if (flag3)
					{
						i = num2 + 1;
					}
					else
					{
						num = num2 - 1;
					}
				}
				this.collection.Insert(i, item);
			}
		}

		public bool Contains(T item)
		{
			return this.collection.BinarySearch(item) != -1;
		}

		public bool Remove(T item)
		{
			return this.collection.Remove(item);
		}

		public IEnumerator<T> GetEnumerator()
		{
			return this.collection.GetEnumerator();
		}

		IEnumerator IEnumerable.GetEnumerator()
		{
			return this.GetEnumerator();
		}

		public void Clear()
		{
			this.collection.Clear();
		}

		public void CopyTo(T[] array, int arrayIndex)
		{
			this.collection.CopyTo(array, arrayIndex);
		}

		public void AddRange(IEnumerable<T> iCollection)
		{
			this.collection.AddRange(iCollection);
		}
	}
}
