using System;

namespace TrueSync
{
	public interface ITrueSyncBehaviour
	{
		void SetGameInfo(TSPlayerInfo localOwner, int numberOfPlayers);
	}
}
