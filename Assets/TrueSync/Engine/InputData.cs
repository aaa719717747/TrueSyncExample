using System;
using System.Collections.Generic;
using UnityEngine;

namespace TrueSync {

    /**
    * @brief A concrete class based on {@link SerializableDictionary} with 'byte' as key and 'FP' as value.
    * 
    * This is necessary because Unity's json engine doesn't work with generic types.
    **/
    [Serializable]
    public class SerializableDictionaryByteFP : SerializableDictionary<byte, FP> { }

    /**
    * @brief A concrete class based on {@link SerializableDictionary} with 'byte' as key and 'TSVector' as value.
    * 
    * This is necessary because Unity's json engine doesn't work with generic types.
    **/
    [Serializable]
    public class SerializableDictionaryByteTSVector : SerializableDictionary<byte, TSVector> { }

    /**
    * @brief A concrete class based on {@link SerializableDictionary} with 'byte' as key and 'TSVector2' as value.
    * 
    * This is necessary because Unity's json engine doesn't work with generic types.
    **/
    [Serializable]
    public class SerializableDictionaryByteTSVector2 : SerializableDictionary<byte, TSVector2> { }

    /**
     * @brief Provides information about a player's inputs.
     **/
    [Serializable]
	public class InputData : InputDataBase {

        /**
         * @brief Contains data about string values.
         **/
        [SerializeField]
        internal SerializableDictionaryByteString stringTable;

        /**
         * @brief Contains data about byte values.
         **/
        [SerializeField]
        internal SerializableDictionaryByteByte byteTable;

        /**
         * @brief Contains data about int values.
         **/
        [SerializeField]
        internal SerializableDictionaryByteInt intTable;

        /**
         * @brief Contains data about FP values.
         **/
        [SerializeField]
        internal SerializableDictionaryByteFP fpTable;

        /**
         * @brief Contains data about byte[] values.
         **/
        [SerializeField]
        internal SerializableDictionaryByteByteArray byteArrayTable;

        /**
         * @brief Contains data about TSVector values.
         **/
        [SerializeField]
        internal SerializableDictionaryByteTSVector tsVectorTable;

        /**
         * @brief Contains data about TSVector values.
         **/
        [SerializeField]
        internal SerializableDictionaryByteTSVector2 tsVectorTable2;

        /**
        * @brief Possible types of input data.
        **/
        private enum Types : byte { Byte = 0, String = 1, Integer = 2, FP = 3, TSVector = 4, TSVector2 = 5, ByteArray = 6 };

        public InputData() {
            this.stringTable = new SerializableDictionaryByteString ();
			this.byteTable = new SerializableDictionaryByteByte ();
			this.intTable = new SerializableDictionaryByteInt ();
            this.fpTable = new SerializableDictionaryByteFP();
            this.byteArrayTable = new SerializableDictionaryByteByteArray();
            this.tsVectorTable = new SerializableDictionaryByteTSVector();
            this.tsVectorTable2 = new SerializableDictionaryByteTSVector2();
        }

        public override void Serialize(List<byte> bytes) {
            byte numberOfActions = (byte)(Count);

            bytes.Add(numberOfActions);

            var byteTableEnum = byteTable.GetEnumerator();
            while (byteTableEnum.MoveNext()) {
                KeyValuePair<byte, byte> pair = byteTableEnum.Current;

                bytes.Add(pair.Key);
                bytes.Add((byte) Types.Byte);
                bytes.Add(pair.Value);
            }

            var byteArrayTableEnum = byteArrayTable.GetEnumerator();
            while (byteArrayTableEnum.MoveNext()) {
                KeyValuePair<byte, byte[]> pair = byteArrayTableEnum.Current;

                bytes.Add(pair.Key);
                bytes.Add((byte)Types.ByteArray);
                Utils.GetBytes(pair.Value.Length, bytes);

                for (int index = 0, length = pair.Value.Length; index < length; index++) {
                    bytes.Add(pair.Value[index]);
                }
            }

            var intTableEnum = intTable.GetEnumerator();
            while (intTableEnum.MoveNext()) {
                KeyValuePair<byte, int> pair = intTableEnum.Current;

                bytes.Add(pair.Key);
                bytes.Add((byte)Types.Integer);
                Utils.GetBytes(pair.Value, bytes);
            }

            var stringTableEnum = stringTable.GetEnumerator();
            while (stringTableEnum.MoveNext()) {
                KeyValuePair<byte, string> pair = stringTableEnum.Current;
                bytes.Add(pair.Key);
                bytes.Add((byte)Types.String);
                byte[] strAscii = System.Text.Encoding.ASCII.GetBytes(pair.Value);
                Utils.GetBytes(strAscii.Length, bytes);

                for (int index = 0, length = strAscii.Length; index < length; index++) {
                    bytes.Add(strAscii[index]);
                }
            }

            var fpTableEnum = fpTable.GetEnumerator();
            while (fpTableEnum.MoveNext()) {
                KeyValuePair<byte, FP> pair = fpTableEnum.Current;

                bytes.Add(pair.Key);
                bytes.Add((byte)Types.FP);
                Utils.GetBytes(pair.Value.RawValue, bytes);
            }

            var tsVectorTableEnum = tsVectorTable.GetEnumerator();
            while (tsVectorTableEnum.MoveNext()) {
                KeyValuePair<byte, TSVector> pair = tsVectorTableEnum.Current;

                bytes.Add(pair.Key);
                bytes.Add((byte)Types.TSVector);
                Utils.GetBytes(pair.Value.x.RawValue, bytes);
                Utils.GetBytes(pair.Value.y.RawValue, bytes);
                Utils.GetBytes(pair.Value.z.RawValue, bytes);
            }

            var tsVector2TableEnum = tsVectorTable2.GetEnumerator();
            while (tsVector2TableEnum.MoveNext()) {
                KeyValuePair<byte, TSVector2> pair = tsVector2TableEnum.Current;

                bytes.Add(pair.Key);
                bytes.Add((byte)Types.TSVector2);
                Utils.GetBytes(pair.Value.x.RawValue, bytes);
                Utils.GetBytes(pair.Value.y.RawValue, bytes);
            }
        }

        public override void Deserialize(byte[] data, ref int offset) {
            byte numberOfActions = data[offset++];

            for (int i = 0; i < numberOfActions; i++) {
                byte key = data[offset++];
                byte type = data[offset++];

                switch (type) {
                    case (byte)Types.Integer:
                        int intValue = BitConverter.ToInt32(data, offset);
                        AddInt(key, intValue);
                        offset += sizeof(int);
                        break;

                    case (byte)Types.Byte:
                        byte byteValue = data[offset++];
                        AddByte(key, byteValue);
                        break;

                    case (byte)Types.ByteArray:
                        int byteArrayLen = BitConverter.ToInt32(data, offset);
                        offset += sizeof(int);

                        byte[] bArray = new byte[byteArrayLen];
                        for (int indexArray = 0; indexArray < byteArrayLen; indexArray++) {
                            bArray[indexArray] = data[offset + indexArray];
                        }

                        offset += byteArrayLen;

                        AddByteArray(key, bArray);
                        break;

                    case (byte)Types.String:
                        int strlen = BitConverter.ToInt32(data, offset);
                        offset += sizeof(int);
                        string stringValue = System.Text.Encoding.ASCII.GetString(data, offset, strlen);
                        offset += strlen;
                        AddString(key, stringValue);
                        break;

                    case (byte)Types.FP:
                        FP fpValue = FP.FromRaw(BitConverter.ToInt64(data, offset));
                        AddFP(key, fpValue);
                        offset += sizeof(long);
                        break;

                    case (byte)Types.TSVector:
                        FP fpValueX = FP.FromRaw(BitConverter.ToInt64(data, offset));
                        offset += sizeof(long);

                        FP fpValueY = FP.FromRaw(BitConverter.ToInt64(data, offset));
                        offset += sizeof(long);

                        FP fpValueZ = FP.FromRaw(BitConverter.ToInt64(data, offset));
                        offset += sizeof(long);

                        AddTSVector(key, new TSVector(fpValueX, fpValueY, fpValueZ));
                        break;

                    case (byte)Types.TSVector2:
                        FP fpValueX2 = FP.FromRaw(BitConverter.ToInt64(data, offset));
                        offset += sizeof(long);

                        FP fpValueY2 = FP.FromRaw(BitConverter.ToInt64(data, offset));
                        offset += sizeof(long);

                        AddTSVector2(key, new TSVector2(fpValueX2, fpValueY2));
                        break;

                    default:
                        //Not Implemented
                        break;
                }
            }
        }

        public override void CleanUp() {
            this.stringTable.Clear();
            this.byteTable.Clear();
            this.intTable.Clear();
            this.fpTable.Clear();
            this.byteArrayTable.Clear();
            this.tsVectorTable.Clear();
            this.tsVectorTable2.Clear();
        }

        public override void CopyFrom(InputDataBase fromBase) {
            InputData from = (InputData) fromBase;

            var stringTableEnum = from.stringTable.GetEnumerator();
            while (stringTableEnum.MoveNext()) {
                var kv = stringTableEnum.Current;
                this.stringTable.Add(kv.Key, kv.Value);
            }

            var byteTableEnum = from.byteTable.GetEnumerator();
            while (byteTableEnum.MoveNext()) {
                var kv = byteTableEnum.Current;
                this.byteTable.Add(kv.Key, kv.Value);
            }

            var intTableEnum = from.intTable.GetEnumerator();
            while (intTableEnum.MoveNext()) {
                var kv = intTableEnum.Current;
                this.intTable.Add(kv.Key, kv.Value);
            }

            var fpTableEnum = from.fpTable.GetEnumerator();
            while (fpTableEnum.MoveNext()) {
                var kv = fpTableEnum.Current;
                this.fpTable.Add(kv.Key, kv.Value);
            }

            var byteArrayTableEnum = from.byteArrayTable.GetEnumerator();
            while (byteArrayTableEnum.MoveNext()) {
                var kv = byteArrayTableEnum.Current;
                this.byteArrayTable.Add(kv.Key, kv.Value);
            }

            var tsVectorTableEnum = from.tsVectorTable.GetEnumerator();
            while (tsVectorTableEnum.MoveNext()) {
                var kv = tsVectorTableEnum.Current;
                this.tsVectorTable.Add(kv.Key, kv.Value);
            }

            var tsVectorTable2Enum = from.tsVectorTable2.GetEnumerator();
            while (tsVectorTable2Enum.MoveNext()) {
                var kv = tsVectorTable2Enum.Current;
                this.tsVectorTable2.Add(kv.Key, kv.Value);
            }
        }

        /**
        * @brief Returns true if this {@link SyncedData} has all actions information equals to the provided one.
        **/
        public override bool EqualsData(InputDataBase otherBase) {
            InputData other = (InputData) otherBase;

            if (this.stringTable.Count != other.stringTable.Count ||
                this.byteTable.Count != other.byteTable.Count ||
                this.intTable.Count != other.intTable.Count ||
                this.fpTable.Count != other.fpTable.Count ||
                this.byteArrayTable.Count != other.byteArrayTable.Count ||
                this.tsVectorTable.Count != other.tsVectorTable.Count ||
                this.tsVectorTable2.Count != other.tsVectorTable2.Count) {

                return false;
            }

            if (!checkEqualsTable(this, other)) {
                return false;
            }

            return true;
        }

        /**
        * @brief Returns true if all information in two {@link InputData} are equals.
        **/
        private static bool checkEqualsTable(InputData id1, InputData id2) {
            var stringTableEnum = id1.stringTable.GetEnumerator();

            while (stringTableEnum.MoveNext()) {
                var pair = stringTableEnum.Current;

                if (!id2.stringTable.ContainsKey(pair.Key) || pair.Value != id2.stringTable[pair.Key]) {
                    return false;
                }
            }

            var byteTableEnum = id1.byteTable.GetEnumerator();

            while (byteTableEnum.MoveNext()) {
                var pair = byteTableEnum.Current;

                if (!id2.byteTable.ContainsKey(pair.Key) || pair.Value != id2.byteTable[pair.Key]) {
                    return false;
                }
            }

            var intTableEnum = id1.intTable.GetEnumerator();

            while (intTableEnum.MoveNext()) {
                var pair = intTableEnum.Current;

                if (!id2.intTable.ContainsKey(pair.Key) || pair.Value != id2.intTable[pair.Key]) {
                    return false;
                }
            }

            var fpTableEnum = id1.fpTable.GetEnumerator();

            while (fpTableEnum.MoveNext()) {
                var pair = fpTableEnum.Current;

                if (!id2.fpTable.ContainsKey(pair.Key) || pair.Value != id2.fpTable[pair.Key]) {
                    return false;
                }
            }

            var byteArrayTableEnum = id1.byteArrayTable.GetEnumerator();

            while (byteArrayTableEnum.MoveNext()) {
                var pair = byteArrayTableEnum.Current;

                if (!id2.byteArrayTable.ContainsKey(pair.Key) || pair.Value != id2.byteArrayTable[pair.Key]) {
                    return false;
                }
            }

            var tsVectorTableEnum = id1.tsVectorTable.GetEnumerator();

            while (tsVectorTableEnum.MoveNext()) {
                var pair = tsVectorTableEnum.Current;

                if (!id2.tsVectorTable.ContainsKey(pair.Key) || pair.Value != id2.tsVectorTable[pair.Key]) {
                    return false;
                }
            }

            var tsVectorTable2Enum = id1.tsVectorTable2.GetEnumerator();

            while (tsVectorTable2Enum.MoveNext()) {
                var pair = tsVectorTable2Enum.Current;

                if (!id2.tsVectorTable2.ContainsKey(pair.Key) || pair.Value != id2.tsVectorTable2[pair.Key]) {
                    return false;
                }
            }

            return true;
        }

        /**
         * @brief Returns how many key were added.
         **/
        public int Count {
            get {
                return (this.stringTable.Count + this.byteTable.Count + this.intTable.Count + this.fpTable.Count + this.byteArrayTable.Count + this.tsVectorTable.Count + this.tsVectorTable2.Count);
            }
        }

        /**
         * @brief Returns true if there is no input information.
         **/
        internal bool IsEmpty() {
			return Count == 0;
		}

        /**
         * @brief Adds a new string value.
         **/
        internal void AddString(byte key, string value){
			this.stringTable[key] = value;
		}

        /**
         * @brief Adds a new byte value.
         **/
        internal void AddByte(byte key, byte value){
			this.byteTable[key] = value;
		}

        /**
         * @brief Adds a new byte[] value.
         **/
        internal void AddByteArray(byte key, byte[] value) {
            byte[] newValue = new byte[value.Length];
            for (int index = 0, length = newValue.Length; index < length;  index++) {
                newValue[index] = value[index];
            }

            this.byteArrayTable[key] = newValue;
        }

        /**
         * @brief Adds a new int value.
         **/
        internal void AddInt(byte key, int value){
			this.intTable[key] = value;
		}

        /**
         * @brief Adds a new FP value.
         **/
        internal void AddFP(byte key, FP value) {
            this.fpTable[key] = value;
        }

        /**
         * @brief Adds a new TSVector value.
         **/
        internal void AddTSVector(byte key, TSVector value) {
            this.tsVectorTable[key] = value;
        }

        /**
         * @brief Adds a new TSVector2 value.
         **/
        internal void AddTSVector2(byte key, TSVector2 value) {
            this.tsVectorTable2[key] = value;
        }

        /**
         * @brief Gets a string value.
         **/
        public string GetString(byte key) {
			if (!this.stringTable.ContainsKey(key)) {
				return "";
			}

            return  this.stringTable[key];
        }

        /**
         * @brief Gets a byte value.
         **/
        public byte GetByte(byte key) {
			if (!this.byteTable.ContainsKey(key)) {
				return 0;
			}

            return this.byteTable[key];
        }

        /**
         * @brief Gets a byte[] value.
         **/
        public byte[] GetByteArray(byte key) {
            if (!this.byteArrayTable.ContainsKey(key)) {
                return null;
            }

            return this.byteArrayTable[key];
        }

        /**
         * @brief Gets a int value.
         **/
        public int GetInt(byte key) {
            if (!this.intTable.ContainsKey(key)) {
                return 0;
            }

            return this.intTable[key];
        }

        /**
         * @brief Gets a FP value.
         **/
        public FP GetFP(byte key) {
            if (!this.fpTable.ContainsKey(key)) {
                return 0;
            }

            return this.fpTable[key];
        }

        /**
         * @brief Gets a TSVector value.
         **/
        public TSVector GetTSVector(byte key) {
            if (!this.tsVectorTable.ContainsKey(key)) {
                return TSVector.zero;
            }

            return this.tsVectorTable[key];
        }

        /**
         * @brief Gets a TSVector2 value.
         **/
        public TSVector2 GetTSVector2(byte key) {
            if (!this.tsVectorTable2.ContainsKey(key)) {
                return TSVector2.zero;
            }

            return this.tsVectorTable2[key];
        }

        /**
         * @brief Returns true if the key exists.
         **/
        public bool HasString(byte key) {
            return this.stringTable.ContainsKey(key);
        }

        /**
         * @brief Returns true if the key exists.
         **/
        public bool HasByte(byte key) {
            return this.byteTable.ContainsKey(key);
        }

        /**
         * @brief Returns true if the key exists.
         **/
        public bool HasByteArray(byte key) {
            return this.byteArrayTable.ContainsKey(key);
        }

        /**
         * @brief Returns true if the key exists.
         **/
        public bool HasInt(byte key) {
            return this.intTable.ContainsKey(key);
        }

        /**
         * @brief Returns true if the key exists.
         **/
        public bool HasFP(byte key) {
            return this.fpTable.ContainsKey(key);
        }

        /**
         * @brief Returns true if the key exists.
         **/
        public bool HasTSVector(byte key) {
            return this.tsVectorTable.ContainsKey(key);
        }

        /**
         * @brief Returns true if the key exists.
         **/
        public bool HasTSVector2(byte key) {
            return this.tsVectorTable2.ContainsKey(key);
        }

    }

}