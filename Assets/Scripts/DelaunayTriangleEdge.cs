
namespace Game.Utils.Geometry
{
    public struct DelaunayTriangleEdge
    {
        public int TriangleIndex;
        public int EdgeIndex;

        public int EdgeVertexA;
        public int EdgeVertexB;

        public DelaunayTriangleEdge(int triangleIndex, int edgeIndex, int edgeVertexA, int edgeVertexB)
        {
            TriangleIndex = triangleIndex;
            EdgeIndex = edgeIndex;
            EdgeVertexA = edgeVertexA;
            EdgeVertexB = edgeVertexB;
        }
    }
}

