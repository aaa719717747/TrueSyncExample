using System.Collections.Generic;

namespace TrueSync {

    /**
    * @brief Manages player's input information, it is used both to set and get this data.
    **/
    public class TrueSyncInput {

        /**
        * @brief {@link InputData} from every player.
        **/
        private static List<InputData> currentAllInputsData = new List<InputData>();

        private static Dictionary<byte, InputData> currentAllInputsDataMap = new Dictionary<byte, InputData>();

        /**
        * @brief {@link InputData} from the local player to be used in OnSyncedInput callback.
        **/
        private static InputData currentInputData;

        /**
        * @brief {@link InputData} from the local player to be used in OnSyncedUpdate  callback.
        **/
        private static InputData currentSimulationData;

        /**
        * @brief {@link InputData} from the local player to be used in OnSyncedInput callback.
        **/
        public static InputData CurrentInputData {
            set {
                currentInputData = value;
            }
        }

        /**
        * @brief {@link InputData} from the local player to be used in OnSyncedUpdate  callback.
        **/
        public static InputData CurrentSimulationData {
			get {
				return currentSimulationData;
			}
            set {
                currentSimulationData = value;
            }
        }

        /**
        * @brief Adds a string value in player's input.
        **/
        public static void SetString(byte key, string value){
			if (currentInputData != null)
                currentInputData.AddString (key, value);
		}

        /**
        * @brief Adds a byte value in player's input.
        **/
        public static void SetByte(byte key, byte value){
			if (currentInputData != null)
                currentInputData.AddByte (key, value);
		}

        /**
        * @brief Adds a byte[] value in player's input.
        **/
        public static void SetByteArray(byte key, byte[] value) {
            if (currentInputData != null)
                currentInputData.AddByteArray(key, value);
        }

        /**
        * @brief Adds a bool value in player's input.
        **/
        public static void SetBool(byte key, bool value) {
            if (currentInputData != null)
                currentInputData.AddByte(key, (byte) (value ? 1 : 0));
        }

        /**
        * @brief Adds a int value in player's input.
        **/
        public static void SetInt(byte key, int value){
			if (currentInputData != null)
                currentInputData.AddInt (key, value);
		}

        /**
        * @brief Adds a FP value in player's input.
        **/
        public static void SetFP(byte key, FP value) {
            if (currentInputData != null)
                currentInputData.AddFP(key, value);
        }

        /**
        * @brief Adds a TSVector value in player's input.
        **/
        public static void SetTSVector(byte key, TSVector value) {
            if (currentInputData != null)
                currentInputData.AddTSVector(key, value);
        }

        /**
        * @brief Adds a TSVector2 value in player's input.
        **/
        public static void SetTSVector2(byte key, TSVector2 value) {
            if (currentInputData != null)
                currentInputData.AddTSVector2(key, value);
        }

        /**
        * @brief Converts a Vector3 to a TSVector and adds its value in player's input.
        **/
        public static void SetTSVector(byte key, UnityEngine.Vector3 value) {
            SetTSVector(key, value.ToTSVector());
        }

        /**
        * @brief Converts a Vector2 to a TSVector and adds its value in player's input.
        **/
        public static void SetTSVector2(byte key, UnityEngine.Vector2 value) {
            SetTSVector2(key, value.ToTSVector2());
        }

        public static void SetAllInputs(List<InputDataBase> allInputData) {
            currentAllInputsData.Clear();
            currentAllInputsDataMap.Clear();

            if (allInputData == null) {
                return;
            }

            for (int index = 0, length = allInputData.Count; index < length; index++) {
                InputData input = (InputData) allInputData[index];

                currentAllInputsData.Add(input);
                currentAllInputsDataMap.Add(input.ownerID, input);
            }
        }

        /**
        * @brief Returns a list of {@link InputData} from all players.
        **/
        public static List<InputData> GetAllInputs() {
            return currentAllInputsData;
        }

        /**
        * @brief Returns a {@link InputData} from a specific player.
        **/
        public static InputData GetPlayerInput(byte ownerId) {
            if (!currentAllInputsDataMap.ContainsKey(ownerId)) {
                return null;
            }

            return currentAllInputsDataMap[ownerId];
        }

        private static void LogInvalidKeyAccess() {
            UnityEngine.Debug.LogWarning("You can't access an input in a TrueSyncBehaviour that has no player owner.");
        }

        /**
        * @brief Returns a string value from player's inputs for the provided key.
        **/
        public static string GetString(byte key) {
            if (currentSimulationData == null) {
                LogInvalidKeyAccess();
                return null;
            }

			return currentSimulationData.GetString (key);
		}

        /**
        * @brief Returns a string value from a specific player's inputs.
        **/
        public static string GetString(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return null;
            }

            return playerData.GetString(key);
        }

        /**
        * @brief Returns a byte value from player's inputs for the provided key.
        **/
        public static byte GetByte(byte key) {
            if (currentSimulationData == null) {
                LogInvalidKeyAccess();
                return 0;
            }

            return currentSimulationData.GetByte (key);
		}

        /**
        * @brief Returns a byte value from a specific player's inputs.
        **/
        public static byte GetByte(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return 0;
            }

            return playerData.GetByte(key);
        }

        /**
        * @brief Returns a byte[] value from player's inputs for the provided key.
        **/
        public static byte[] GetByteArray(byte key) {
            if (currentSimulationData == null) {
                LogInvalidKeyAccess();
                return null;
            }

            return currentSimulationData.GetByteArray(key);
        }

        /**
        * @brief Returns a byte[] value from a specific player's inputs.
        **/
        public static byte[] GetByteArray(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return null;
            }

            return playerData.GetByteArray(key);
        }

        /**
        * @brief Returns a bool value from player's inputs for the provided key.
        **/
        public static bool GetBool(byte key) {
            if (currentSimulationData == null) {
                LogInvalidKeyAccess();
                return false;
            }

            return currentSimulationData.GetByte(key) == 1;
        }

        /**
        * @brief Returns a bool value from a specific player's inputs.
        **/
        public static bool GetBool(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return false;
            }

            return playerData.GetByte(key) == 1;
        }

        /**
        * @brief Returns an int value from player's inputs for the provided key.
        **/
        public static int GetInt(byte key) {
            if (currentSimulationData == null) {
                LogInvalidKeyAccess();
                return 0;
            }

            return currentSimulationData.GetInt (key);
		}

        /**
        * @brief Returns an int value from a specific player's inputs.
        **/
        public static int GetInt(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return 0;
            }

            return playerData.GetInt(key);
        }

        /**
        * @brief Returns a FP value from player's inputs for the provided key.
        **/
        public static FP GetFP(byte key) {
            if (currentSimulationData == null) {
                LogInvalidKeyAccess();
                return FP.Zero;
            }

            return currentSimulationData.GetFP(key);
        }

        /**
        * @brief Returns a FP value from a specific player's inputs.
        **/
        public static FP GetFP(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return FP.Zero;
            }

            return playerData.GetFP(key);
        }

        /**
        * @brief Returns a TSVector value from player's inputs for the provided key.
        **/
        public static TSVector GetTSVector(byte key) {
            if (currentSimulationData == null) {
                LogInvalidKeyAccess();
                return TSVector.zero;
            }

            return currentSimulationData.GetTSVector(key);
        }

        /**
        * @brief Returns a TSVector value from a specific player's inputs.
        **/
        public static TSVector GetTSVector(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return TSVector.zero;
            }

            return playerData.GetTSVector(key);
        }

        /**
        * @brief Returns a TSVector2 value from player's inputs for the provided key.
        **/
        public static TSVector2 GetTSVector2(byte key) {
            if (currentSimulationData == null) {
                LogInvalidKeyAccess();
                return TSVector2.zero;
            }

            return currentSimulationData.GetTSVector2(key);
        }

        /**
        * @brief Returns a TSVector2 value from a specific player's inputs.
        **/
        public static TSVector2 GetTSVector2(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return TSVector2.zero;
            }

            return playerData.GetTSVector2(key);
        }

        /**
        * @brief Returns true if there is a string for the provided key.
        **/
        public static bool HasString(byte key) {
            if (currentSimulationData == null) {
                return false;
            }

            return currentSimulationData.HasString(key);
        }

        /**
        * @brief Returns true if there is a string for the provided key for a specific player.
        **/
        public static bool HasString(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return false;
            }

            return playerData.HasString(key);
        }

        /**
        * @brief Returns true if there is a byte for the provided key.
        **/
        public static bool HasByte(byte key) {
            if (currentSimulationData == null) {
                return false;
            }

            return currentSimulationData.HasByte(key);
        }

        /**
        * @brief Returns true if there is a byte for the provided key for a specific player.
        **/
        public static bool HasByte(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return false;
            }

            return playerData.HasByte(key);
        }

        /**
        * @brief Returns true if there is a byte[] for the provided key.
        **/
        public static bool HasByteArray(byte key) {
            if (currentSimulationData == null) {
                return false;
            }

            return currentSimulationData.HasByteArray(key);
        }

        /**
        * @brief Returns true if there is a byte[] for the provided key for a specific player.
        **/
        public static bool HasByteArray(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return false;
            }

            return playerData.HasByteArray(key);
        }

        /**
        * @brief Returns true if there is a bool for the provided key.
        **/
        public static bool HasBool(byte key) {
            if (currentSimulationData == null) {
                return false;
            }

            return currentSimulationData.HasByte(key);
        }

        /**
        * @brief Returns true if there is a bool for the provided key for a specific player.
        **/
        public static bool HasBool(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return false;
            }

            return playerData.HasByte(key);
        }

        /**
        * @brief Returns true if there is an int for the provided key.
        **/
        public static bool HasInt(byte key) {
            if (currentSimulationData == null) {
                return false;
            }

            return currentSimulationData.HasInt(key);
        }

        /**
        * @brief Returns true if there is an int for the provided key for a specific player.
        **/
        public static bool HasInt(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return false;
            }

            return playerData.HasInt(key);
        }

        /**
        * @brief Returns true if there is a FP for the provided key.
        **/
        public static bool HasFP(byte key) {
            if (currentSimulationData == null) {
                return false;
            }

            return currentSimulationData.HasFP(key);
        }

        /**
        * @brief Returns true if there is a FP for the provided key for a specific player.
        **/
        public static bool HasFP(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return false;
            }

            return playerData.HasFP(key);
        }

        /**
        * @brief Returns true if there is a TSVector for the provided key.
        **/
        public static bool HasTSVector(byte key) {
            if (currentSimulationData == null) {
                return false;
            }

            return currentSimulationData.HasTSVector(key);
        }

        /**
        * @brief Returns true if there is a TSVector for the provided key for a specific player.
        **/
        public static bool HasTSVector(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return false;
            }

            return playerData.HasTSVector(key);
        }

        /**
        * @brief Returns true if there is a TSVector2 for the provided key.
        **/
        public static bool HasTSVector2(byte key) {
            if (currentSimulationData == null) {
                return false;
            }

            return currentSimulationData.HasTSVector2(key);
        }

        /**
        * @brief Returns true if there is a TSVector2 for the provided key for a specific player.
        **/
        public static bool HasTSVector2(byte ownerId, byte key) {
            InputData playerData = GetPlayerInput(ownerId);
            if (playerData == null) {
                return false;
            }

            return playerData.HasTSVector2(key);
        }

    }

}