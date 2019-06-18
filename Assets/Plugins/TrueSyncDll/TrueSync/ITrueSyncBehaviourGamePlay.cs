using System;

namespace TrueSync
{
	public interface ITrueSyncBehaviourGamePlay : ITrueSyncBehaviour
	{
		void OnPreSyncedUpdate();

		void OnSyncedInput();

		void OnSyncedUpdate();
	}
}
