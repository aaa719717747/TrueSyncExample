using System;

namespace TrueSync
{
	public interface ITrueSyncBehaviourCallbacks : ITrueSyncBehaviour
	{
		void OnSyncedStart();

		void OnSyncedStartLocalPlayer();

		void OnGamePaused();

		void OnGameUnPaused();

		void OnGameEnded();

		void OnPlayerDisconnection(int playerId);
	}
}
