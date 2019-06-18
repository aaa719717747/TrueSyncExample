using System;
using System.Collections.Generic;

namespace TrueSync
{
	public class ArrayResourcePool<T>
	{
		private Stack<T[]> stack = new Stack<T[]>();

		private int arrayLength;

		public int Count
		{
			get
			{
				return this.stack.Count;
			}
		}

		public ArrayResourcePool(int arrayLength)
		{
			this.arrayLength = arrayLength;
		}

		public void ResetResourcePool()
		{
			Stack<T[]> obj = this.stack;
			lock (obj)
			{
				this.stack.Clear();
			}
		}

		public void GiveBack(T[] obj)
		{
			Stack<T[]> obj2 = this.stack;
			lock (obj2)
			{
				this.stack.Push(obj);
			}
		}

		public T[] GetNew()
		{
			Stack<T[]> obj = this.stack;
			T[] array;
			lock (obj)
			{
				bool flag = this.stack.Count == 0;
				if (flag)
				{
					array = new T[this.arrayLength];
					this.stack.Push(array);
				}
				array = this.stack.Pop();
			}
			return array;
		}
	}
}
