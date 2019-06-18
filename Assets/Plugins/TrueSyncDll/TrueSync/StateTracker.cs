using System;
using System.Collections.Generic;
using System.Reflection;

namespace TrueSync
{
    // 状态追踪类
	public class StateTracker
    {
        #region 状态类
        internal class State
		{
			private StateTracker.TrackedInfo trackedInfo;

			private object value;

			private Array _auxReferenceArray;

			public void SetInfo(StateTracker.TrackedInfo trackedInfo)
			{
				this.trackedInfo = trackedInfo;
				this.SaveValue();
			}

			public void SaveValue()
			{
				object obj = this.trackedInfo.propInfo.GetValue(this.trackedInfo.relatedObj);
				bool flag = obj != null;
				if (flag)
				{
					bool isArray = obj.GetType().IsArray;
					if (isArray)
					{
						bool flag2 = this.value == null;
						if (flag2)
						{
							this.value = Array.CreateInstance(obj.GetType().GetElementType(), ((Array)obj).Length);
							this._auxReferenceArray = (Array)obj;
						}
						Array.Copy(this._auxReferenceArray, (Array)this.value, this._auxReferenceArray.Length);
					}
					else
					{
						this.value = obj;
					}
				}
				else
				{
					this.value = null;
				}
			}

			public void RestoreValue()
			{
				bool flag = this.trackedInfo.relatedObj != null;
				if (flag)
				{
					bool flag2 = this.value is Array;
					if (flag2)
					{
						Array.Copy((Array)this.value, this._auxReferenceArray, ((Array)this.value).Length);
					}
					else
					{
						this.trackedInfo.propInfo.SetValue(this.trackedInfo.relatedObj, this.value);
					}
				}
			}
		}
        #endregion 状态类

        #region 追踪信息类
        internal class TrackedInfo
		{
			public object relatedObj; // 相关的对象

			public MemberInfo propInfo; // 成员信息
		}
        #endregion 追踪信息类

        private static ResourcePoolStateTrackerState resourcePool = new ResourcePoolStateTrackerState();

		private HashSet<string> trackedInfosAdded = new HashSet<string>();

		private List<StateTracker.TrackedInfo> trackedInfos = new List<StateTracker.TrackedInfo>();

		private GenericBufferWindow<List<StateTracker.State>> states;

		internal static StateTracker instance;

        #region 公共方法
        // 初始化
        public static void Init(int rollbackWindow)
		{
			StateTracker.instance = new StateTracker();
			StateTracker.instance.states = new GenericBufferWindow<List<StateTracker.State>>(rollbackWindow);
		}

        // 清空
		public static void CleanUp()
		{
			StateTracker.instance = null;
		}
        
        // 添加追踪
        public static void AddTracking(object obj, string path)
		{
			bool flag = StateTracker.instance != null;
			if (flag)
			{
				string item = string.Format("{0}_{1}_{2}", obj.GetType().FullName, obj.GetHashCode(), path);
				bool flag2 = !StateTracker.instance.trackedInfosAdded.Contains(item);
				if (flag2)
				{
					StateTracker.TrackedInfo trackedInfo = StateTracker.GetTrackedInfo(obj, path);
					StateTracker.instance.trackedInfos.Add(trackedInfo);
					StateTracker.instance.trackedInfosAdded.Add(item);
					int i = 0;
					int size = StateTracker.instance.states.size;
					while (i < size)
					{
						StateTracker.State @new = StateTracker.resourcePool.GetNew();
						@new.SetInfo(trackedInfo);
						StateTracker.instance.states.Current().Add(@new);
						StateTracker.instance.states.MoveNext();
						i++;
					}
				}
			}
		}

        // 添加追踪
		public static void AddTracking(object obj)
		{
			bool flag = StateTracker.instance != null;
			if (flag)
			{
				List<MemberInfo> membersInfo = Utils.GetMembersInfo(obj.GetType());
				int i = 0;
				int count = membersInfo.Count;
				while (i < count)
				{
					MemberInfo memberInfo = membersInfo[i];
					object[] customAttributes = memberInfo.GetCustomAttributes(true);
					bool flag2 = customAttributes != null;
					if (flag2)
					{
						int j = 0;
						int num = customAttributes.Length;
						while (j < num)
						{
							bool flag3 = customAttributes[j] is AddTracking;
							if (flag3)
							{
								StateTracker.AddTracking(obj, memberInfo.Name);
							}
							j++;
						}
					}
					i++;
				}
			}
		}
        #endregion 公共方法

        #region 内部方法
        // 保存状态
        internal void SaveState()
		{
			List<StateTracker.State> list = this.states.Current();
			int i = 0;
			int count = list.Count;
			while (i < count)
			{
				list[i].SaveValue();
				i++;
			}
			this.MoveNextState();
		}

        // 重新保存状态
		internal void RestoreState()
		{
			List<StateTracker.State> list = this.states.Current();
			int i = 0;
			int count = list.Count;
			while (i < count)
			{
				list[i].RestoreValue();
				i++;
			}
		}

        // 移动到下一个状态
		internal void MoveNextState()
		{
			this.states.MoveNext();
		}
        #endregion 内部方法

        #region 私有方法
        // 获取追踪信息
        private static StateTracker.TrackedInfo GetTrackedInfo(object obj, string name)
		{
			string[] array = name.Split(new char[]
			{
				'.'
			});
			int i = 0;
			int num = array.Length;
			StateTracker.TrackedInfo result;
			while (i < num)
			{
				string name2 = array[i];
				bool flag = obj == null;
				if (flag)
				{
					result = null;
				}
				else
				{
					Type type = obj.GetType();

                    //MemberInfo memberInfo = type.GetProperty(name2, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic) ?? type.GetField(name2, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                    MemberInfo memberInfo = null;
                    if (type.GetProperty(name2, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic) != null)
                        memberInfo = type.GetProperty(name2, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                    else
                        memberInfo = type.GetField(name2, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                    
                    bool flag2 = memberInfo == null;
					if (flag2)
					{
						result = null;
					}
					else
					{
						bool flag3 = i == num - 1;
						if (!flag3)
						{
							obj = memberInfo.GetValue(obj);
							i++;
							continue;
						}
						result = new StateTracker.TrackedInfo
						{
							relatedObj = obj,
							propInfo = memberInfo
						};
					}
				}
				return result;
			}
			result = null;
			return result;
        }
        #endregion 私有方法
    }
}
