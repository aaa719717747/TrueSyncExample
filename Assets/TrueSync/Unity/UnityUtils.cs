using System.Collections.Generic;
using System;
using System.Reflection;

namespace TrueSync {

    /**
    *  @brief Provides a few utilities to be used on TrueSync exposed classes.
    **/
    public class UnityUtils {

        /**
         *  @brief Comparer class to guarantee {@link TSCollider} order.
         **/
        public class TSBodyComparer : Comparer<TSCollider> {

            public override int Compare(TSCollider x, TSCollider y) {
                return x.gameObject.name.CompareTo(y.gameObject.name);
            }

        }

        /**
         *  @brief Comparer class to guarantee {@link TSCollider2D} order.
         **/
        public class TSBody2DComparer : Comparer<TSCollider2D> {

            public override int Compare(TSCollider2D x, TSCollider2D y) {
                return x.gameObject.name.CompareTo(y.gameObject.name);
            }

        }

        /**
         *  @brief Instance of a {@link TSBodyComparer}.
         **/
        public static TSBodyComparer bodyComparer = new TSBodyComparer();

        /**
         *  @brief Instance of a {@link TSBody2DComparer}.
         **/
        public static TSBody2DComparer body2DComparer = new TSBody2DComparer();

    }

}