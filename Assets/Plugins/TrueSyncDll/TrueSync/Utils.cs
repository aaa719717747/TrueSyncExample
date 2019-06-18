using System;
using System.Collections.Generic;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;

namespace TrueSync
{
	public class Utils
	{
		public static List<MemberInfo> GetMembersInfo(Type type)
		{
			List<MemberInfo> list = new List<MemberInfo>();
			list.AddRange(type.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic));
			list.AddRange(type.GetProperties(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic));
			return list;
		}

		public static void GetBytes(int value, List<byte> bytes)
		{
			bytes.Add((byte)value);
			bytes.Add((byte)(value >> 8));
			bytes.Add((byte)(value >> 16));
			bytes.Add((byte)(value >> 24));
		}

		public static void GetBytes(long value, List<byte> bytes)
		{
			bytes.Add((byte)value);
			bytes.Add((byte)(value >> 8));
			bytes.Add((byte)(value >> 16));
			bytes.Add((byte)(value >> 24));
			bytes.Add((byte)(value >> 32));
			bytes.Add((byte)(value >> 40));
			bytes.Add((byte)(value >> 48));
			bytes.Add((byte)(value >> 56));
		}

		public static string GetMd5Sum(string str)
		{
			Encoder encoder = Encoding.Unicode.GetEncoder();
			byte[] array = new byte[str.Length * 2];
			encoder.GetBytes(str.ToCharArray(), 0, str.Length, array, 0, true);
			MD5 mD = new MD5CryptoServiceProvider();
			byte[] array2 = mD.ComputeHash(array);
			StringBuilder stringBuilder = new StringBuilder();
			for (int i = 0; i < array2.Length; i++)
			{
				stringBuilder.Append(array2[i].ToString("X2"));
			}
			return stringBuilder.ToString();
		}
	}
}
