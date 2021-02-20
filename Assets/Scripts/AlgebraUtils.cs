
using System.Collections.Generic;
using UnityEngine;

namespace Game.Utils.Algebra
{
    public static class AlgebraUtils
    {
        public static float CalculateMatrix3x3Determinant(float m00, float m10, float m20,
                                                          float m01, float m11, float m21,
                                                          float m02, float m12, float m22)
        {
            return m00 * m11 * m22 + m10 * m21 * m02 + m20 * m01 * m12 - m20 * m11 * m02 - m10 * m01 * m22 - m00 * m21 * m12;
        }
    }
}

