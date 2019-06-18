using System;
using System.Reflection;

public static class TrueSyncExtensionsCore
{
	public static object GetValue(this MemberInfo memberInfo, object obj)
	{
		bool flag = memberInfo is PropertyInfo;
		object result;
		if (flag)
		{
			result = ((PropertyInfo)memberInfo).GetValue(obj, null);
		}
		else
		{
			bool flag2 = memberInfo is FieldInfo;
			if (flag2)
			{
				result = ((FieldInfo)memberInfo).GetValue(obj);
			}
			else
			{
				result = null;
			}
		}
		return result;
	}

	public static void SetValue(this MemberInfo memberInfo, object obj, object value)
	{
		bool flag = memberInfo is PropertyInfo;
		if (flag)
		{
			((PropertyInfo)memberInfo).SetValue(obj, value, null);
		}
		else
		{
			bool flag2 = memberInfo is FieldInfo;
			if (flag2)
			{
				((FieldInfo)memberInfo).SetValue(obj, value);
			}
		}
	}
}
