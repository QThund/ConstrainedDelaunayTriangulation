using System.Collections.Generic;

namespace Game.Utils.Math
{
    public unsafe struct DelaunayTriangle
    {
        public fixed int p[3];
        public fixed int adjacent[3];

        private const int NO_ADJACENT_TRIANGLE = -1;

        public DelaunayTriangle(int point0, int point1, int point2)
        {
            p[0] = point0;
            p[1] = point1;
            p[2] = point2;

            adjacent[0] = NO_ADJACENT_TRIANGLE;
            adjacent[1] = NO_ADJACENT_TRIANGLE;
            adjacent[2] = NO_ADJACENT_TRIANGLE;
        }

        public DelaunayTriangle(int point0, int point1, int point2, int adjacent0, int adjacent1, int adjacent2)
        {
            p[0] = point0;
            p[1] = point1;
            p[2] = point2;

            adjacent[0] = adjacent0;
            adjacent[1] = adjacent1;
            adjacent[2] = adjacent2;
        }

        public List<int> DebugP
        {
            get
            {
                List<int> debugArray = new List<int>(3);
                for(int i = 0; i < 3; ++i)
                {
                    debugArray.Add(p[i]);
                }
                return debugArray;
            }
        }

        public List<int> DebugAdjacent
        {
            get
            {
                List<int> debugArray = new List<int>(3);
                for (int i = 0; i < 3; ++i)
                {
                    debugArray.Add(adjacent[i]);
                }
                return debugArray;
            }
        }
    }
}

