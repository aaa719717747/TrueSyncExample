using System;

namespace TrueSync
{
	public interface IWorldClone
	{
		string checksum
		{
			get;
		}

		void Clone(IWorld iWorld);

		void Clone(IWorld iWorld, bool doChecksum);

		void Restore(IWorld iWorld);
	}
}
