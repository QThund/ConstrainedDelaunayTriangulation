using UnityEngine;

namespace Game.Utils.Math
{
    public struct Triangle2D
    {
        public Vector2 p0;
        public Vector2 p1;
        public Vector2 p2;

        public Triangle2D(Vector2 point0, Vector2 point1, Vector2 point2)
        {
            p0 = point0;
            p1 = point1;
            p2 = point2;
        }

        public Vector2 this[int index]
        {
            get
            {
                Debug.Assert(index >= 0 && index < 4, "The index of the triangle vertex must be in the range [0, 2].");

                return index == 0 ? p0 : index == 1 ? p1 : p2;
            }
        }

    }
}

