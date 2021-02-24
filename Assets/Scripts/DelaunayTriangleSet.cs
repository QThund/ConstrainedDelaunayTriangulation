using System.Collections.Generic;
using UnityEngine;

namespace Game.Utils.Math
{
    public unsafe class DelaunayTriangleSet
    {
        protected List<int> m_adjacentTriangles;
        protected List<int> m_triangleVertices;
        protected List<Vector2> m_points;

        private const int NOT_FOUND = -1;
        private const int NO_ADJACENT_TRIANGLE = -1;

        public DelaunayTriangleSet(int expectedTriangles)
        {
            m_adjacentTriangles = new List<int>(expectedTriangles * 3);
            m_triangleVertices = new List<int>(expectedTriangles * 3);
            m_points = new List<Vector2>(expectedTriangles);
        }

        public List<Vector2> Points
        {
            get
            {
                return m_points;
            }
        }

        public int TriangleCount
        {
            get
            {
                return m_triangleVertices.Count / 3;
            }
        }

        public int AddTriangle(DelaunayTriangle newTriangle)
        {
            return AddTriangle(newTriangle.p[0], newTriangle.p[1], newTriangle.p[2], newTriangle.adjacent[0], newTriangle.adjacent[1], newTriangle.adjacent[2]);
        }

        public int AddPoint(Vector2 point)
        {
            m_points.Add(point);
            return m_points.Count - 1;
        }

        public int AddTriangle(int p0, int p1, int p2, int adjacentTriangle0, int adjacentTriangle1, int adjacentTriangle2)
        {
            m_adjacentTriangles.Add(adjacentTriangle0);
            m_adjacentTriangles.Add(adjacentTriangle1);
            m_adjacentTriangles.Add(adjacentTriangle2);
            m_triangleVertices.Add(p0);
            m_triangleVertices.Add(p1);
            m_triangleVertices.Add(p2);

            return TriangleCount - 1;
        }

        public int AddTriangle(Vector2 p0, Vector2 p1, Vector2 p2, int adjacentTriangle0, int adjacentTriangle1, int adjacentTriangle2)
        {
            m_adjacentTriangles.Add(adjacentTriangle0);
            m_adjacentTriangles.Add(adjacentTriangle1);
            m_adjacentTriangles.Add(adjacentTriangle2);
            m_triangleVertices.Add(AddPoint(p0));
            m_triangleVertices.Add(AddPoint(p1));
            m_triangleVertices.Add(AddPoint(p2));

            return TriangleCount - 1;
        }

        public void GetTrianglesWithVertex(int vertexIndex, List<int> outputTriangles)
        {
            for(int i = 0; i < TriangleCount; ++i)
            {
                for(int j = 0; j < 3; ++j)
                {
                    if(m_triangleVertices[i * 3 + j] == vertexIndex)
                    {
                        outputTriangles.Add(i);
                        break;
                    }
                }
            }
        }

        public Triangle2D GetTrianglePoints(int triangleIndex)
        {
            return new Triangle2D(m_points[m_triangleVertices[triangleIndex * 3]],
                                  m_points[m_triangleVertices[triangleIndex * 3 + 1]],
                                  m_points[m_triangleVertices[triangleIndex * 3 + 2]]);
        }

        public DelaunayTriangle GetTriangle(int triangleIndex)
        {
            return new DelaunayTriangle(m_triangleVertices[triangleIndex * 3],
                                        m_triangleVertices[triangleIndex * 3 + 1],
                                        m_triangleVertices[triangleIndex * 3 + 2],
                                        m_adjacentTriangles[triangleIndex * 3],
                                        m_adjacentTriangles[triangleIndex * 3 + 1],
                                        m_adjacentTriangles[triangleIndex * 3 + 2]);
        }

        // This method assumes that the edges of the triangles to find were created using the same vertex order
        // It also assumes all triangles are inside a supertriangle, so no adjacent triangles are -1
        public void GetTrianglesInPolygon(List<int> polygonOutline, List<int> outputTrianglesInPolygon)
        {
            Stack<int> adjacentTriangles = new Stack<int>();

            // First it gets all the triangles of the outline
            for (int i = 0; i < polygonOutline.Count; ++i)
            {
                // For every edge, it gets the inner triangle that contains such edge
                DelaunayTriangleEdge triangleEdge = FindTriangleThatContainsEdge(polygonOutline[i], polygonOutline[(i + 1) % polygonOutline.Count]);

                // A triangle may form a corner, with 2 outline edges, this avoids adding it twice
                if(outputTrianglesInPolygon.Count > 0 &&
                   (outputTrianglesInPolygon[outputTrianglesInPolygon.Count - 1] == triangleEdge.TriangleIndex || // The last added triangle is the same as current?
                    outputTrianglesInPolygon[0] == triangleEdge.EdgeIndex)) // The first added triangle is the same as the current, which the last to be added (closes the polygon)?
                {
                    continue;
                }

                outputTrianglesInPolygon.Add(triangleEdge.TriangleIndex);

                for(int j = 1; j < 3; ++j) // For the 2 adjacent triangles of the other 2 edges
                {
                    int adjacentTriangle = m_adjacentTriangles[triangleEdge.TriangleIndex * 3 + (triangleEdge.EdgeIndex + j) % 3];
                    bool isAdjacentTriangleInOutline = false;

                    for (int k = 0; k < 3; ++k)
                    {
                        // Compares the contiguous edges, to the right and to the left, (flipped) with the adjacent triangle's edges
                        if ((m_triangleVertices[adjacentTriangle * 3 + (k + 1) % 3] == polygonOutline[(i + 1) % polygonOutline.Count] &&
                             m_triangleVertices[adjacentTriangle * 3 + k] == polygonOutline[(i + 2) % polygonOutline.Count])
                            ||
                            (m_triangleVertices[adjacentTriangle * 3 + (k + 1) % 3] == polygonOutline[(i + polygonOutline.Count - 1) % polygonOutline.Count] &&
                             m_triangleVertices[adjacentTriangle * 3 + k] == polygonOutline[(i + polygonOutline.Count) % polygonOutline.Count]))
                        {
                            isAdjacentTriangleInOutline = true;
                        }
                    }

                    if (!isAdjacentTriangleInOutline && !outputTrianglesInPolygon.Contains(adjacentTriangle))
                    {
                        adjacentTriangles.Push(adjacentTriangle);
                    }

                }
            }

            // Then it propagates by adjacency, stopping when an adjacent triangle has already been included in the list
            // Since all the outline triangles have been added previously, it will not propagate outside of the polygon
            while(adjacentTriangles.Count > 0)
            {
                int currentTriangle = adjacentTriangles.Pop();

                // The triangle may have been added already in a previous iteration
                if(outputTrianglesInPolygon.Contains(currentTriangle))
                {
                    continue;
                }

                for (int i = 0; i < 3; ++i)
                {
                    Debug.Log(currentTriangle);
                    int adjacentTriangle = m_adjacentTriangles[currentTriangle * 3 + i];

                    if (adjacentTriangle != NO_ADJACENT_TRIANGLE && !outputTrianglesInPolygon.Contains(adjacentTriangle))
                    {
                        adjacentTriangles.Push(adjacentTriangle);
                    }
                }
                        
                outputTrianglesInPolygon.Add(currentTriangle);
            }
        }

        public void GetIntersectingEdges(Vector2 edgeEndpointA, Vector2 edgeEndpointB, int startTriangle, List<DelaunayTriangleEdge> intersectingEdges)
        {
            bool isTriangleContainingBFound = false;
            int triangleIndex = startTriangle;

            while (!isTriangleContainingBFound)
            {
                DrawTriangle(triangleIndex, Color.green);

                bool hasCrossedEdge = false;
                int tentativeAdjacentTriangle = NO_ADJACENT_TRIANGLE;

                for (int i = 0; i < 3; ++i)
                {
                    if (m_points[m_triangleVertices[triangleIndex * 3 + i]] == edgeEndpointB ||
                       m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]] == edgeEndpointB)
                    {
                        isTriangleContainingBFound = true;
                        break;
                    }

                    if (MathUtils.IsPointToTheRightOfEdge(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], edgeEndpointB))
                    {
                        tentativeAdjacentTriangle = i;

                        Debug.DrawLine(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], Color.green, 10.0f);

                        Vector2 intersectionPoint;

                        if (MathUtils.InsersectionBetweenLines(m_points[m_triangleVertices[triangleIndex * 3 + i]],
                                                                   m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]],
                                                                   edgeEndpointA,
                                                                   edgeEndpointB,
                                                                   out intersectionPoint))
                        {
                            hasCrossedEdge = true;

                            intersectingEdges.Add(new DelaunayTriangleEdge(NOT_FOUND, NOT_FOUND, m_triangleVertices[triangleIndex * 3 + i], m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]));

                            //Debug.DrawLine(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], Color.yellow, 10.0f);
                            //const float xlineLength = 0.008f;
                            //Debug.DrawRay(intersectionPoint - new Vector2(xlineLength * 0.5f, xlineLength * 0.5f), new Vector2(xlineLength, xlineLength), Color.red, 10.0f);
                            //Debug.DrawRay(intersectionPoint + new Vector2(-xlineLength * 0.5f, xlineLength * 0.5f), new Vector2(xlineLength, -xlineLength), Color.red, 10.0f);

                            // The point is in the exterior of the triangle (vertices are sorted CCW, the right side is always the exterior from the perspective of the A->B edge)
                            triangleIndex = m_adjacentTriangles[triangleIndex * 3 + i];

                            break;
                        }

                    }
                }

                // Continue searching at a different adjacent triangle
                if (!hasCrossedEdge)
                {
                    triangleIndex = m_adjacentTriangles[triangleIndex * 3 + tentativeAdjacentTriangle];
                }
            }
        }

        public Vector2 GetPointByIndex(int pointIndex)
        {
            return m_points[pointIndex];
        }

        public int GetIndexOfPoint(Vector2 point)
        {
            int index = 0;

            while(index < m_points.Count && m_points[index] != point)
            {
                ++index;
            }

            return index == m_points.Count ? -1 : index;
        }

        public DelaunayTriangleEdge FindTriangleThatContainsEdge(int edgeVertexA, int edgeVertexB)
        {
            DelaunayTriangleEdge foundTriangle = new DelaunayTriangleEdge(NOT_FOUND, NOT_FOUND, edgeVertexA, edgeVertexB);

            for(int i = 0; i < TriangleCount; ++i)
            {
                for(int j = 0; j < 3; ++j)
                {
                    if(m_triangleVertices[i * 3 + j] == edgeVertexA && m_triangleVertices[i * 3 + (j + 1) % 3] == edgeVertexB)
                    {
                        foundTriangle.TriangleIndex = i;
                        foundTriangle.EdgeIndex = j;
                        break;
                    }
                }
            }

            return foundTriangle;
        }

        public int FindTriangleThatContainsPoint(Vector2 point, int startTriangle)
        { 
            bool isTriangleFound = false;
            int triangleIndex = startTriangle;
            
            while(!isTriangleFound)
            {
                isTriangleFound = true;

                for (int i = 0; i < 3; ++i)
                {
                    if (MathUtils.IsPointToTheRightOfEdge(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], point))
                    {
                        // The point is in the exterior of the triangle (vertices are sorted CCW, the right side is always the exterior from the perspective of the A->B edge)
                        triangleIndex = m_adjacentTriangles[triangleIndex * 3 + i];
                        
                        isTriangleFound = false;
                        break;
                    }
                }
            }
            
            return triangleIndex;
        }

        public int FindTriangleThatContainsLineEndpoint(int endpointAIndex, int endpointBIndex)
        {
            List<int> trianglesWithEndpoint = new List<int>();
            GetTrianglesWithVertex(endpointAIndex, trianglesWithEndpoint);

            int foundTriangle = NOT_FOUND;
            Vector2 endpointA = m_points[endpointAIndex];
            Vector2 endpointB = m_points[endpointBIndex];
            Debug.DrawLine(endpointA + Vector2.up * 0.01f, endpointB + Vector2.up * 0.01f, Color.yellow, 10.0f);

            for (int i = 0; i < trianglesWithEndpoint.Count; ++i)
            {
                //DelaunayTriangle triangleDebug = GetTriangle(trianglesWithEndpoint[i]);
                //List<int> pointsDebug = triangleDebug.DebugP;

                int vertexPositionInTriangle = m_triangleVertices[trianglesWithEndpoint[i] * 3] == endpointAIndex ? 0 
                                                                                                                 : m_triangleVertices[trianglesWithEndpoint[i] * 3 + 1] == endpointAIndex ? 1 
                                                                                                                                                                                         : 2;
                Vector2 triangleEdgePoint1 = m_points[m_triangleVertices[trianglesWithEndpoint[i] * 3 + (vertexPositionInTriangle + 1) % 3]];
                Vector2 triangleEdgePoint2 = m_points[m_triangleVertices[trianglesWithEndpoint[i] * 3 + (vertexPositionInTriangle + 2) % 3]];

                // Is the line in the angle between the 2 contiguous edges of the triangle?
                if (MathUtils.IsPointToTheRightOfEdge(triangleEdgePoint1, endpointA, endpointB) &&
                    MathUtils.IsPointToTheRightOfEdge(endpointA, triangleEdgePoint2, endpointB))
                {
                    foundTriangle = trianglesWithEndpoint[i];
                    break;
                }
            }
            
            return foundTriangle;
        }

        public void SetTriangleAdjacency(int triangleIndex, int* adjacentsToTriangle)
        {
            for(int i = 0; i < 3; ++i)
            {
                m_adjacentTriangles[triangleIndex * 3 + i] = adjacentsToTriangle[i];
            }
        }

        public void ReplaceAdjacent(int triangleIndex, int oldAdjacentTriangle, int newAdjacentTriangle)
        {
            for(int i = 0; i < 3; ++i)
            {
                if(m_adjacentTriangles[triangleIndex * 3 + i] == oldAdjacentTriangle)
                {
                    m_adjacentTriangles[triangleIndex * 3 + i] = newAdjacentTriangle;
                }
            }
        }

        public void ReplaceTriangle(int triangleToReplace, DelaunayTriangle newTriangle)
        {
            for(int i = 0; i < 3; ++i)
            {
                m_triangleVertices[triangleToReplace * 3 + i] = newTriangle.p[i];
                m_adjacentTriangles[triangleToReplace * 3 + i] = newTriangle.adjacent[i];
            }
        }

        public void DrawTriangle(int triangleIndex, Color color)
        {
            for(int i = 0; i < 3; ++i)
            {
                Debug.DrawLine(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], color, 10.0f);
            }
        }

        public void LogDump()
        {
            for(int i = 0; i < TriangleCount; ++i)
            {
                string logEntry = "Triangle " + i + "<color=yellow>(";

                for(int j = 0; j < 3; ++j)
                {
                    logEntry += m_triangleVertices[i * 3 + j];

                    if(j < 2)
                    {
                        logEntry += ", ";
                    }
                }

                logEntry += ")</color>-A(";

                for (int j = 0; j < 3; ++j)
                {
                    logEntry += m_adjacentTriangles[i * 3 + j];

                    if (j < 2)
                    {
                        logEntry += ", ";
                    }
                }

                logEntry += ")-v(";

                for (int j = 0; j < 3; ++j)
                {
                    logEntry += m_points[m_triangleVertices[i * 3 + j]].ToString("F6");

                    if (j < 2)
                    {
                        logEntry += ", ";
                    }
                }

                logEntry += ")";

                Debug.Log(logEntry);
            }
        }
    }
}

