using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;

namespace TrueSync {
    /**
     * @brief Manages creation of player prefabs and lockstep execution.
     **/
    [AddComponentMenu("")]
    public class TrueSyncManager : MonoBehaviour {

        private const float JitterTimeFactor = 0.001f;

        private const string serverSettingsAssetFile = "TrueSyncGlobalConfig";

        private enum StartState { BEHAVIOR_INITIALIZED, FIRST_UPDATE, STARTED };

        private StartState startState;

        /** 
         * @brief Player prefabs to be instantiated in each machine.
         **/
        public GameObject[] playerPrefabs;

        public static TrueSyncConfig _TrueSyncGlobalConfig;

        public static TrueSyncConfig TrueSyncGlobalConfig {
            get {
                if (_TrueSyncGlobalConfig == null) {
                    _TrueSyncGlobalConfig = (TrueSyncConfig) Resources.Load(serverSettingsAssetFile, typeof(TrueSyncConfig));
                }

                return _TrueSyncGlobalConfig;
            }
        }

        public static TrueSyncConfig TrueSyncCustomConfig = null;

        public TrueSyncConfig customConfig;

        private Dictionary<int, List<GameObject>> gameOjectsSafeMap = new Dictionary<int, List<GameObject>>();

        /**
         * @brief Instance of the lockstep engine.
         **/
        private AbstractLockstep lockstep;

        private FP lockedTimeStep;

        /**
         * @brief A list of {@link TrueSyncBehaviour} not linked to any player.
         **/
        private List<TrueSyncManagedBehaviour> generalBehaviours = new List<TrueSyncManagedBehaviour>();

        /**
         * @brief A dictionary holding a list of {@link TrueSyncBehaviour} belonging to each player.
         **/
        private Dictionary<byte, List<TrueSyncManagedBehaviour>> behaviorsByPlayer;

        /**
         * @brief The coroutine scheduler.
         **/
        private CoroutineScheduler scheduler;

        /**
         * @brief List of {@link TrueSyncBehaviour} that should be included next update.
         **/
        private List<TrueSyncManagedBehaviour> queuedBehaviours = new List<TrueSyncManagedBehaviour>();

        private Dictionary<ITrueSyncBehaviour, TrueSyncManagedBehaviour> mapBehaviorToManagedBehavior = new Dictionary<ITrueSyncBehaviour, TrueSyncManagedBehaviour>();

        private FP time = 0;

        /**
         * @brief Returns the deltaTime between two simulation calls.
         **/
        public static FP DeltaTime {
            get {
                if (instance == null) {
                    return 0;
                }

                return instance.lockedTimeStep;
            }
        }

        /**
         * @brief Returns the time elapsed since the beginning of the simulation.
         **/
        public static FP Time {
            get {
                if (instance == null || instance.lockstep == null) {
                    return 0;
                }

                return instance.time;
            }
        }

        /**
         * @brief Returns the number of the last simulated tick.
         **/
        public static int Ticks {
            get {
                if (instance == null || instance.lockstep == null) {
                    return 0;
                }

                return instance.lockstep.Ticks;
            }
        }

        /**
         * @brief Returns the last safe simulated tick.
         **/
        public static int LastSafeTick {
            get {
                if (instance == null || instance.lockstep == null) {
                    return 0;
                }

                return instance.lockstep.LastSafeTick;
            }
        }

        /** 
         * @brief Returns the simulated gravity.
         **/
        public static TSVector Gravity {
            get {
                if (instance == null) {
                    return TSVector.zero;
                }

                return instance.ActiveConfig.gravity3D;
            }
        }

        /** 
         * @brief Returns the list of players connected.
         **/
        public static List<TSPlayerInfo> Players {
            get {
                if (instance == null || instance.lockstep == null) {
                    return null;
                }

                List<TSPlayerInfo> allPlayers = new List<TSPlayerInfo>();
                foreach (TSPlayer tsp in instance.lockstep.Players.Values) {
                    if (!tsp.dropped) {
                        allPlayers.Add(tsp.playerInfo);
                    }
                }

                return allPlayers;
            }
        }

        /** 
         * @brief Returns the local player.
         **/
        public static TSPlayerInfo LocalPlayer {
            get {
                if (instance == null || instance.lockstep == null) {
                    return null;
                }

                return instance.lockstep.LocalPlayer.playerInfo;
            }
        }

        /** 
         * @brief Returns the active {@link TrueSyncConfig} used by the {@link TrueSyncManager}.
         **/
        public static TrueSyncConfig Config {
            get {
                if (instance == null) {
                    return null;
                }

                return instance.ActiveConfig;
            }
        }

        private static TrueSyncManager instance;

        private TrueSyncConfig ActiveConfig {
            get {
                if (TrueSyncCustomConfig != null) {
                    customConfig = TrueSyncCustomConfig;
                    TrueSyncCustomConfig = null;
                }

                if (customConfig != null) {
                    return customConfig;
                }

                return TrueSyncGlobalConfig;
            }
        }

        void Awake() {
            TrueSyncConfig currentConfig = ActiveConfig;
            lockedTimeStep = currentConfig.lockedTimeStep;

            StateTracker.Init(currentConfig.rollbackWindow);
            TSRandom.Init();

            if (currentConfig.physics2DEnabled || currentConfig.physics3DEnabled) {
                PhysicsManager.New(currentConfig);
                PhysicsManager.instance.LockedTimeStep = lockedTimeStep;
                PhysicsManager.instance.Init();
            }

            StateTracker.AddTracking(this, "time");
        }

        void Start() {
            instance = this;
            Application.runInBackground = true;

            ICommunicator communicator = null;
            //if (!PhotonNetwork.connected || !PhotonNetwork.inRoom) {
            //    Debug.LogWarning("You are not connected to Photon. TrueSync will start in offline mode.");
            //} else {
            //    communicator = new PhotonTrueSyncCommunicator(PhotonNetwork.networkingPeer);
            //}
            Debug.LogWarning("You are not connected to Photon. TrueSync will start in offline mode.");

            TrueSyncConfig activeConfig = ActiveConfig;

            lockstep = AbstractLockstep.NewInstance(
                lockedTimeStep.AsFloat(),
                communicator,
                PhysicsManager.instance,
                activeConfig.syncWindow,
                activeConfig.panicWindow,
                activeConfig.rollbackWindow,
                OnGameStarted,
                OnGamePaused,
                OnGameUnPaused,
                OnGameEnded,
                OnPlayerDisconnection,
                OnStepUpdate,
                GetLocalData,
                ProvideInputData
            );

            //if (ReplayRecord.replayMode == ReplayMode.LOAD_REPLAY) {
            //    ReplayPicker.replayToLoad.Load();

            //    ReplayRecord replayRecord = ReplayRecord.replayToLoad;
            //    if (replayRecord == null) {
            //        Debug.LogError("Replay Record can't be loaded");
            //        gameObject.SetActive(false);
            //        return;
            //    } else {
            //        lockstep.ReplayRecord = replayRecord;
            //    }
            //}

            if (activeConfig.showStats) {
                this.gameObject.AddComponent<TrueSyncStats>().Lockstep = lockstep;
            }

            scheduler = new CoroutineScheduler(lockstep);

            if (ReplayRecord.replayMode != ReplayMode.LOAD_REPLAY) {
                lockstep.AddPlayer(0, "Local_Player", true);
                //if (communicator == null) {
                //    lockstep.AddPlayer(0, "Local_Player", true);
                //} else {
                //    List<PhotonPlayer> players = new List<PhotonPlayer>(PhotonNetwork.playerList);
                //    players.Sort(UnityUtils.playerComparer);

                //    for (int index = 0, length = players.Count; index < length; index++) {
                //        PhotonPlayer p = players[index];
                //        lockstep.AddPlayer((byte) p.ID, p.NickName, p.IsLocal);
                //    }
                //}
            }

            TrueSyncBehaviour[] behavioursArray = FindObjectsOfType<TrueSyncBehaviour>();
            for (int index = 0, length = behavioursArray.Length; index < length; index++) {
                generalBehaviours.Add(NewManagedBehavior(behavioursArray[index]));
            }

            initBehaviors();
            initGeneralBehaviors(generalBehaviours, false);

            PhysicsManager.instance.OnRemoveBody(OnRemovedRigidBody);

            startState = StartState.BEHAVIOR_INITIALIZED;
        }

        private TrueSyncManagedBehaviour NewManagedBehavior(ITrueSyncBehaviour trueSyncBehavior) {
            TrueSyncManagedBehaviour result = new TrueSyncManagedBehaviour(trueSyncBehavior);
            mapBehaviorToManagedBehavior[trueSyncBehavior] = result;

            return result;
        }

        private void initBehaviors() {
            behaviorsByPlayer = new Dictionary<byte, List<TrueSyncManagedBehaviour>>();

            var playersEnum = lockstep.Players.GetEnumerator();
            while (playersEnum.MoveNext()) {
                TSPlayer p = playersEnum.Current.Value;

                List<TrueSyncManagedBehaviour> behaviorsInstatiated = new List<TrueSyncManagedBehaviour>();

                for (int index = 0, length = playerPrefabs.Length; index < length; index++) {
                    GameObject prefab = playerPrefabs[index];

                    GameObject prefabInst = Instantiate(prefab);
                    InitializeGameObject(prefabInst, prefabInst.transform.position.ToTSVector(), prefabInst.transform.rotation.ToTSQuaternion());

                    TrueSyncBehaviour[] behaviours = prefabInst.GetComponentsInChildren<TrueSyncBehaviour>();
                    for (int index2 = 0, length2 = behaviours.Length; index2 < length2; index2++) {
                        TrueSyncBehaviour behaviour = behaviours[index2];

                        behaviour.owner = p.playerInfo;
                        behaviour.localOwner = lockstep.LocalPlayer.playerInfo;
                        behaviour.numberOfPlayers = lockstep.Players.Count;

                        TrueSyncManagedBehaviour tsmb = NewManagedBehavior(behaviour);
                        tsmb.owner = behaviour.owner;
                        tsmb.localOwner = behaviour.localOwner;

                        behaviorsInstatiated.Add(tsmb);
                    }
                }

                behaviorsByPlayer.Add(p.ID, behaviorsInstatiated);
            }
        }

        private void initGeneralBehaviors(IEnumerable<TrueSyncManagedBehaviour> behaviours, bool realOwnerId) {
            List<TSPlayer> playersList = new List<TSPlayer>(lockstep.Players.Values);
            List<TrueSyncManagedBehaviour> itemsToRemove = new List<TrueSyncManagedBehaviour>();

            var behavioursEnum = behaviours.GetEnumerator();
            while (behavioursEnum.MoveNext()) {
                TrueSyncManagedBehaviour tsmb = behavioursEnum.Current;

                if (!(tsmb.trueSyncBehavior is TrueSyncBehaviour)) {
                    continue;
                }

                TrueSyncBehaviour bh = (TrueSyncBehaviour)tsmb.trueSyncBehavior;

                if (realOwnerId) {
                    bh.ownerIndex = bh.owner.Id;
                } else {
                    if (bh.ownerIndex >= 0 && bh.ownerIndex < playersList.Count) {
                        bh.ownerIndex = playersList[bh.ownerIndex].ID;
                    }
                }

                if (behaviorsByPlayer.ContainsKey((byte)bh.ownerIndex)) {
                    bh.owner = lockstep.Players[(byte)bh.ownerIndex].playerInfo;

                    behaviorsByPlayer[(byte)bh.ownerIndex].Add(tsmb);
                    itemsToRemove.Add(tsmb);
                } else {
                    bh.ownerIndex = -1;
                }

                bh.localOwner = lockstep.LocalPlayer.playerInfo;
                bh.numberOfPlayers = lockstep.Players.Count;

                tsmb.owner = bh.owner;
                tsmb.localOwner = bh.localOwner;
            }

            for (int index = 0, length = itemsToRemove.Count; index < length; index++) {
                generalBehaviours.Remove(itemsToRemove[index]);
            }
        }

        private void CheckQueuedBehaviours() {
            if (queuedBehaviours.Count > 0) {
                generalBehaviours.AddRange(queuedBehaviours);
                initGeneralBehaviors(queuedBehaviours, true);

                for (int index = 0, length = queuedBehaviours.Count; index < length; index++) {
                    TrueSyncManagedBehaviour tsmb = queuedBehaviours[index];

                    tsmb.SetGameInfo(lockstep.LocalPlayer.playerInfo, lockstep.Players.Count);
                    tsmb.OnSyncedStart();
                }

                queuedBehaviours.Clear();
            }
        }

        void Update() {
            if (lockstep != null && startState != StartState.STARTED) {
                if (startState == StartState.BEHAVIOR_INITIALIZED) {
                    startState = StartState.FIRST_UPDATE;
                } else if (startState == StartState.FIRST_UPDATE) {
                    lockstep.RunSimulation(true);
                    startState = StartState.STARTED;
                }
            }
        }

        /**
         * @brief Run/Unpause the game simulation.
         **/
        public static void RunSimulation() {
            if (instance != null && instance.lockstep != null) {
                instance.lockstep.RunSimulation(false);
            }
        }

        /**
         * @brief Pauses the game simulation.
         **/
        public static void PauseSimulation() {
            if (instance != null && instance.lockstep != null) {
                instance.lockstep.PauseSimulation();
            }
        }

        /**
         * @brief End the game simulation.
         **/
        public static void EndSimulation() {
            if (instance != null && instance.lockstep != null) {
                instance.lockstep.EndSimulation();
            }
        }

        /**
         * @brief Update all coroutines created.
         **/
        public static void UpdateCoroutines() {
            if (instance != null && instance.lockstep != null) {
                instance.scheduler.UpdateAllCoroutines();
            }
        }

        /**
         * @brief Starts a new coroutine.
         * 
         * @param coroutine An IEnumerator that represents the coroutine.
         **/
        public static void SyncedStartCoroutine(IEnumerator coroutine) {
            if (instance != null && instance.lockstep != null) {
                instance.scheduler.StartCoroutine(coroutine);
            }
        }

        /**
         * @brief Instantiate a new prefab in a deterministic way.
         * 
         * @param prefab GameObject's prefab to instantiate.
         **/
        public static GameObject SyncedInstantiate(GameObject prefab) {
            return SyncedInstantiate(prefab, prefab.transform.position.ToTSVector(), prefab.transform.rotation.ToTSQuaternion());
        }

        /**
         * @brief Instantiates a new prefab in a deterministic way.
         * 
         * @param prefab GameObject's prefab to instantiate.
         * @param position Position to place the new GameObject.
         * @param rotation Rotation to set in the new GameObject.
         **/
        public static GameObject SyncedInstantiate(GameObject prefab, TSVector position, TSQuaternion rotation) {
            if (instance != null && instance.lockstep != null) {
                GameObject go = GameObject.Instantiate(prefab, position.ToVector(), rotation.ToQuaternion()) as GameObject;

                if (ReplayRecord.replayMode != ReplayMode.LOAD_REPLAY) {
                    AddGameObjectOnSafeMap(go);
                }

                MonoBehaviour[] monoBehaviours = go.GetComponentsInChildren<MonoBehaviour>();
                for (int index = 0, length = monoBehaviours.Length; index < length; index++) {
                    MonoBehaviour bh = monoBehaviours[index];

                    if (bh is ITrueSyncBehaviour) {
                        instance.queuedBehaviours.Add(instance.NewManagedBehavior((ITrueSyncBehaviour)bh));
                    }
                }

                InitializeGameObject(go, position, rotation);

                return go;
            }

            return null;
        }

        private static void AddGameObjectOnSafeMap(GameObject go) {

            Dictionary<int, List<GameObject>> safeMap = instance.gameOjectsSafeMap;

            int currentTick = TrueSyncManager.Ticks + 1;
            if (!safeMap.ContainsKey(currentTick)) {
                safeMap.Add(currentTick, new List<GameObject>());
            }

            safeMap[currentTick].Add(go);
        }

        private static void CheckGameObjectsSafeMap() {
            Dictionary<int, List<GameObject>> safeMap = instance.gameOjectsSafeMap;

            int currentTick = TrueSyncManager.Ticks + 1;
            if (safeMap.ContainsKey(currentTick)) {
                List<GameObject> gos = safeMap[currentTick];
                for (int i = 0, l = gos.Count; i < l; i++) {
                    GameObject go = gos[i];
                    if (go != null) {
                        Renderer rend = go.GetComponent<Renderer>();
                        if (rend != null) {
                            rend.enabled = false;
                        }

                        GameObject.Destroy(go);
                    }
                }

                gos.Clear();
            }

            safeMap.Remove(TrueSyncManager.LastSafeTick);
        }

        private static void InitializeGameObject(GameObject go, TSVector position, TSQuaternion rotation) {
            ICollider[] tsColliders = go.GetComponentsInChildren<ICollider>();
            if (tsColliders != null) {
                for (int index = 0, length = tsColliders.Length; index < length; index++) {
                    PhysicsManager.instance.AddBody(tsColliders[index]);
                }
            }

            TSTransform rootTSTransform = go.GetComponent<TSTransform>();
            if (rootTSTransform != null) {
                rootTSTransform.Initialize();

                rootTSTransform.position = position;
                rootTSTransform.rotation = rotation;
            }

            TSTransform[] tsTransforms = go.GetComponentsInChildren<TSTransform>();
            if (tsTransforms != null) {
                for (int index = 0, length = tsTransforms.Length; index < length; index++) {
                    TSTransform tsTransform = tsTransforms[index];

                    if (tsTransform != rootTSTransform) {
                        tsTransform.Initialize();
                    }
                }
            }

            TSTransform2D rootTSTransform2D = go.GetComponent<TSTransform2D>();
            if (rootTSTransform2D != null) {
                rootTSTransform2D.Initialize();

                rootTSTransform2D.position = new TSVector2(position.x, position.y);
                rootTSTransform2D.rotation = rotation.ToQuaternion().eulerAngles.z;
            }

            TSTransform2D[] tsTransforms2D = go.GetComponentsInChildren<TSTransform2D>();
            if (tsTransforms2D != null) {
                for (int index = 0, length = tsTransforms2D.Length; index < length; index++) {
                    TSTransform2D tsTransform2D = tsTransforms2D[index];

                    if (tsTransform2D != rootTSTransform2D) {
                        tsTransform2D.Initialize();
                    }
                }
            }
        }

        /**
         * @brief Instantiates a new prefab in a deterministic way.
         * 
         * @param prefab GameObject's prefab to instantiate.
         * @param position Position to place the new GameObject.
         * @param rotation Rotation to set in the new GameObject.
         **/
        public static GameObject SyncedInstantiate(GameObject prefab, TSVector2 position, TSQuaternion rotation) {
            return SyncedInstantiate(prefab, new TSVector(position.x, position.y, 0), rotation);
        }

        /**
         * @brief Destroys a GameObject in a deterministic way.
         * 
         * The method {@link #DestroyTSRigidBody} is called and attached TrueSyncBehaviors are disabled.
         * 
         * @param rigidBody Instance of a {@link TSRigidBody}
         **/
        public static void SyncedDestroy(GameObject gameObject) {
            if (instance != null && instance.lockstep != null) {
                SyncedDisableBehaviour(gameObject);

                TSCollider[] tsColliders = gameObject.GetComponentsInChildren<TSCollider>();
                if (tsColliders != null) {
                    for (int index = 0, length = tsColliders.Length; index < length; index++) {
                        TSCollider tsCollider = tsColliders[index];
                        DestroyTSRigidBody(tsCollider.gameObject, tsCollider.Body);
                    }
                }

                TSCollider2D[] tsColliders2D = gameObject.GetComponentsInChildren<TSCollider2D>();
                if (tsColliders2D != null) {
                    for (int index = 0, length = tsColliders2D.Length; index < length; index++) {
                        TSCollider2D tsCollider2D = tsColliders2D[index];
                        DestroyTSRigidBody(tsCollider2D.gameObject, tsCollider2D.Body);
                    }
                }
            }
        }

        /**
         * @brief Disables 'OnSyncedInput' and 'OnSyncUpdate' calls to every {@link ITrueSyncBehaviour} attached.
         **/
        public static void SyncedDisableBehaviour(GameObject gameObject) {
            MonoBehaviour[] monoBehaviours = gameObject.GetComponentsInChildren<MonoBehaviour>();

            for (int index = 0, length = monoBehaviours.Length; index < length; index++) {
                MonoBehaviour tsb = monoBehaviours[index];

                if (tsb is ITrueSyncBehaviour && instance.mapBehaviorToManagedBehavior.ContainsKey((ITrueSyncBehaviour)tsb)) {
                    instance.mapBehaviorToManagedBehavior[(ITrueSyncBehaviour)tsb].disabled = true;
                }
            }
        }

        /**
         * @brief The related GameObject is firstly set to be inactive then in a safe moment it will be destroyed.
         * 
         * @param rigidBody Instance of a {@link TSRigidBody}
         **/
        private static void DestroyTSRigidBody(GameObject tsColliderGO, IBody body) {
            tsColliderGO.gameObject.SetActive(false);
            instance.lockstep.Destroy(body);
        }

        /**
         * @brief Registers an implementation of {@link ITrueSyncBehaviour} to be included in the simulation.
         * 
         * @param trueSyncBehaviour Instance of an {@link ITrueSyncBehaviour}
         **/
        public static void RegisterITrueSyncBehaviour(ITrueSyncBehaviour trueSyncBehaviour) {
            if (instance != null && instance.lockstep != null) {
                instance.queuedBehaviours.Add(instance.NewManagedBehavior(trueSyncBehaviour));
            }
        }

        /**
         * @brief Register a {@link TrueSyncIsReady} delegate to that returns true if the game can proceed or false otherwise.
         * 
         * @param IsReadyChecker A {@link TrueSyncIsReady} delegate
         **/
        public static void RegisterIsReadyChecker(TrueSyncIsReady IsReadyChecker) {
            if (instance != null && instance.lockstep != null) {
                instance.lockstep.GameIsReady += IsReadyChecker;
            }
        }

        /**
         * @brief Removes objets related to a provided player.
         * 
         * @param playerId Target player's id.
         **/
        public static void RemovePlayer(int playerId) {
            if (instance != null && instance.lockstep != null) {
                List<TrueSyncManagedBehaviour> behaviorsList = instance.behaviorsByPlayer[(byte)playerId];

                for (int index = 0, length = behaviorsList.Count; index < length; index++) {
                    TrueSyncManagedBehaviour tsmb = behaviorsList[index];
                    tsmb.disabled = true;

                    TSCollider[] tsColliders = ((TrueSyncBehaviour)tsmb.trueSyncBehavior).gameObject.GetComponentsInChildren<TSCollider>();
                    if (tsColliders != null) {
                        for (int index2 = 0, length2 = tsColliders.Length; index2 < length2; index2++) {
                            TSCollider tsCollider = tsColliders[index2];

                            if (!tsCollider.Body.TSDisabled) {
                                DestroyTSRigidBody(tsCollider.gameObject, tsCollider.Body);
                            }
                        }
                    }

                    TSCollider2D[] tsCollider2Ds = ((TrueSyncBehaviour)tsmb.trueSyncBehavior).gameObject.GetComponentsInChildren<TSCollider2D>();
                    if (tsCollider2Ds != null) {
                        for (int index2 = 0, length2 = tsCollider2Ds.Length; index2 < length2; index2++) {
                            TSCollider2D tsCollider2D = tsCollider2Ds[index2];

                            if (!tsCollider2D.Body.TSDisabled) {
                                DestroyTSRigidBody(tsCollider2D.gameObject, tsCollider2D.Body);
                            }
                        }
                    }
                }
            }
        }

        private FP tsDeltaTime = 0;

        void FixedUpdate() {
            if (lockstep != null) {
                tsDeltaTime += UnityEngine.Time.deltaTime;

                if (tsDeltaTime >= (lockedTimeStep - JitterTimeFactor)) {
                    tsDeltaTime = 0;

                    instance.scheduler.UpdateAllCoroutines();
                    lockstep.Update();
                }
            }
        }

        InputDataBase ProvideInputData() {
            return new InputData();
        }

        void GetLocalData(InputDataBase playerInputData) {
            TrueSyncInput.CurrentInputData = (InputData) playerInputData;

            if (behaviorsByPlayer.ContainsKey(playerInputData.ownerID)) {
                List<TrueSyncManagedBehaviour> managedBehavioursByPlayer = behaviorsByPlayer[playerInputData.ownerID];
                for (int index = 0, length = managedBehavioursByPlayer.Count; index < length; index++) {
                    TrueSyncManagedBehaviour bh = managedBehavioursByPlayer[index];

                    if (bh != null && !bh.disabled) {
                        bh.OnSyncedInput();
                    }
                }
            }

            TrueSyncInput.CurrentInputData = null;
        }

        void OnStepUpdate(List<InputDataBase> allInputData) {
            time += lockedTimeStep;

            if (ReplayRecord.replayMode != ReplayMode.LOAD_REPLAY) {
                CheckGameObjectsSafeMap();
            }

            TrueSyncInput.SetAllInputs(null);

            for (int index = 0, length = generalBehaviours.Count; index < length; index++) {
                TrueSyncManagedBehaviour bh = generalBehaviours[index];

                if (bh != null && !bh.disabled) {
                    bh.OnPreSyncedUpdate();
                    instance.scheduler.UpdateAllCoroutines();
                }
            }

            for (int index = 0, length = allInputData.Count; index < length; index++) {
                InputDataBase playerInputData = allInputData[index];

                if (behaviorsByPlayer.ContainsKey(playerInputData.ownerID)) {
                    List<TrueSyncManagedBehaviour> managedBehavioursByPlayer = behaviorsByPlayer[playerInputData.ownerID];
                    for (int index2 = 0, length2 = managedBehavioursByPlayer.Count; index2 < length2; index2++) {
                        TrueSyncManagedBehaviour bh = managedBehavioursByPlayer[index2];

                        if (bh != null && !bh.disabled) {
                            bh.OnPreSyncedUpdate();
                            instance.scheduler.UpdateAllCoroutines();
                        }
                    }
                }
            }

            TrueSyncInput.SetAllInputs(allInputData);

            TrueSyncInput.CurrentSimulationData = null;
            for (int index = 0, length = generalBehaviours.Count; index < length; index++) {
                TrueSyncManagedBehaviour bh = generalBehaviours[index];

                if (bh != null && !bh.disabled) {
                    bh.OnSyncedUpdate();
                    instance.scheduler.UpdateAllCoroutines();
                }
            }

            for (int index = 0, length = allInputData.Count; index < length; index++) {
                InputDataBase playerInputData = allInputData[index];

                if (behaviorsByPlayer.ContainsKey(playerInputData.ownerID)) {
                    TrueSyncInput.CurrentSimulationData = (InputData) playerInputData;

                    List<TrueSyncManagedBehaviour> managedBehavioursByPlayer = behaviorsByPlayer[playerInputData.ownerID];
                    for (int index2 = 0, length2 = managedBehavioursByPlayer.Count; index2 < length2; index2++) {
                        TrueSyncManagedBehaviour bh = managedBehavioursByPlayer[index2];

                        if (bh != null && !bh.disabled) {
                            bh.OnSyncedUpdate();
                            instance.scheduler.UpdateAllCoroutines();
                        }
                    }
                }

                TrueSyncInput.CurrentSimulationData = null;
            }

            CheckQueuedBehaviours();
        }

        private void OnRemovedRigidBody(IBody body) {
            GameObject go = PhysicsManager.instance.GetGameObject(body);

            if (go != null) {
                List<TrueSyncBehaviour> behavioursToRemove = new List<TrueSyncBehaviour>(go.GetComponentsInChildren<TrueSyncBehaviour>());
                RemoveFromTSMBList(queuedBehaviours, behavioursToRemove);
                RemoveFromTSMBList(generalBehaviours, behavioursToRemove);

                var behaviorsByPlayerEnum = behaviorsByPlayer.GetEnumerator();
                while (behaviorsByPlayerEnum.MoveNext()) {
                    List<TrueSyncManagedBehaviour> listBh = behaviorsByPlayerEnum.Current.Value;
                    RemoveFromTSMBList(listBh, behavioursToRemove);
                }
            }
        }

        private void RemoveFromTSMBList(List<TrueSyncManagedBehaviour> tsmbList, List<TrueSyncBehaviour> behaviours) {
            List<TrueSyncManagedBehaviour> toRemove = new List<TrueSyncManagedBehaviour>();
            for (int index = 0, length = tsmbList.Count; index < length; index++) {
                TrueSyncManagedBehaviour tsmb = tsmbList[index];

                if ((tsmb.trueSyncBehavior is TrueSyncBehaviour) && behaviours.Contains((TrueSyncBehaviour)tsmb.trueSyncBehavior)) {
                    toRemove.Add(tsmb);
                }
            }

            for (int index = 0, length = toRemove.Count; index < length; index++) {
                TrueSyncManagedBehaviour tsmb = toRemove[index];
                tsmbList.Remove(tsmb);
            }
        }

        /** 
         * @brief Clean up references to be collected by gc.
         **/
        public static void CleanUp() {
            ResourcePool.CleanUpAll();
            StateTracker.CleanUp();
            instance = null;
        }

        void OnPlayerDisconnection(byte playerId) {
            TrueSyncManagedBehaviour.OnPlayerDisconnection(generalBehaviours, behaviorsByPlayer, playerId);
        }

        void OnGameStarted() {
            TrueSyncManagedBehaviour.OnGameStarted(generalBehaviours, behaviorsByPlayer);
            instance.scheduler.UpdateAllCoroutines();

            CheckQueuedBehaviours();
        }

        void OnGamePaused() {
            TrueSyncManagedBehaviour.OnGamePaused(generalBehaviours, behaviorsByPlayer);
            instance.scheduler.UpdateAllCoroutines();
        }

        void OnGameUnPaused() {
            TrueSyncManagedBehaviour.OnGameUnPaused(generalBehaviours, behaviorsByPlayer);
            instance.scheduler.UpdateAllCoroutines();
        }

        void OnGameEnded() {
            TrueSyncManagedBehaviour.OnGameEnded(generalBehaviours, behaviorsByPlayer);
            instance.scheduler.UpdateAllCoroutines();
        }

        void OnApplicationQuit() {
            EndSimulation();
        }

    }

}