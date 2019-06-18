using System;

namespace TrueSync
{
    // 通用缓冲窗口(循环列表结构)
	public class GenericBufferWindow<T>
	{
		public delegate T NewInstance(); // 新实例

		public T[] buffer; // 缓冲数组

		public int size; // 容量

		public int currentIndex; // 当前索引

        #region 构造器
        public GenericBufferWindow(int size)
		{
			this.size = size;
			this.currentIndex = 0;
			this.buffer = new T[size];
			for (int i = 0; i < size; i++)
			{
				this.buffer[i] = Activator.CreateInstance<T>();
			}
		}

		public GenericBufferWindow(int size, GenericBufferWindow<T>.NewInstance NewInstance)
		{
			this.size = size;
			this.currentIndex = 0;
			this.buffer = new T[size];
			for (int i = 0; i < size; i++)
			{
				this.buffer[i] = NewInstance();
			}
		}
        #endregion 构造器

        #region 公共方法
        // 重置容量
        public void Resize(int newSize)
		{
			bool flag = newSize == this.size;
			if (!flag) // 新容量与就容量不一致，需要重置
			{
				T[] array = new T[newSize]; // 新容量数组
				int num = newSize - this.size; // 容量差
				bool flag2 = newSize > this.size;
				if (flag2) // 扩容
				{
					for (int i = 0; i < this.size; i++)
					{
						bool flag3 = i < this.currentIndex;
						if (flag3)
						{
							array[i] = this.buffer[i];
						}
                        else // 把旧缓冲数组中currentIndex之后的数据，放到了新缓冲数组之后 // 不明白
						{
							array[i + num] = this.buffer[i];
						}
					}
					for (int j = 0; j < num; j++) // 在缓冲数组的空白处补充上新的实例
					{
						array[this.currentIndex + j] = Activator.CreateInstance<T>();
					}
				}
				else // 减容
				{
					for (int k = 0; k < newSize; k++)
					{
						bool flag4 = k < this.currentIndex;
						if (flag4)
						{
							array[k] = this.buffer[k];
						}
						else
						{
							array[k] = this.buffer[k - num];
						}
					}
					this.currentIndex %= newSize;
				}
				this.buffer = array;
				this.size = newSize;
			}
		}

        // 设置缓冲元素
        public void Set(T instance)
        {
            this.buffer[this.currentIndex] = instance;
        }

        // 返回前一个缓冲元素
        public T Previous()
        {
            int num = this.currentIndex - 1;
            bool flag = num < 0;
            if (flag) // 当没有前一个缓冲元素时，则返回最后一个
            {
                num = this.size - 1;
            }
            return this.buffer[num];
        }

        // 返回当前元素
        public T Current()
        {
            return this.buffer[this.currentIndex];
        }

        // 移动到下一个索引
        public void MoveNext()
        {
            this.currentIndex = (this.currentIndex + 1) % this.size;
        }
        #endregion 公共方法
	}
}
