using System.Collections;


/// <summary>
/// CoroutineNode.cs
/// 
/// Port of the Javascript version from 
/// http://www.unifycommunity.com/wiki/index.php?title=CoroutineScheduler
/// 
/// Linked list node type used by coroutine scheduler to track scheduling of coroutines.
///  
/// BMBF Researchproject http://playfm.htw-berlin.de
/// PlayFM - Serious Games für den IT-gestützten Wissenstransfer im Facility Management 
///	Gefördert durch das bmb+f - Programm Forschung an Fachhochschulen profUntFH
///	
///	<author>Frank.Otto@htw-berlin.de</author>
///
/// </summary>

namespace TrueSync {
	public class CoroutineNode {
		public CoroutineNode listPrevious = null;
		public CoroutineNode listNext = null;
		public IEnumerator fiber;
		public bool finished = false;
		public int waitForFrame = -1;
		public FP waitForTime = -1.0f;
		public CoroutineNode waitForCoroutine;
		public int playerId = -1;

		public CoroutineNode (IEnumerator _fiber) {
			this.fiber = _fiber;
			if (TrueSyncInput.CurrentSimulationData != null) {
				playerId = TrueSyncInput.CurrentSimulationData.ownerID;
			}
		}

	}
}