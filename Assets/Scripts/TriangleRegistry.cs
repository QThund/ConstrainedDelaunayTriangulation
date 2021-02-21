
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Game.Utils.Geometry
{
    public unsafe class TriangleRegistry
    {
        protected List<int> m_adjacentTriangles;
        protected List<int> m_triangleVertices;
        protected List<Vector2> m_points;

        public TriangleRegistry(int expectedTriangles)
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

        public int AddTriangle(Triangle2D newTriangle, int[] adjacentTriangles)
        {
            return AddTriangle(newTriangle.p0, newTriangle.p1, newTriangle.p2, adjacentTriangles[0], adjacentTriangles[1], adjacentTriangles[2]);
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

        public void RemoveTriangle(int triangleIndex)
        {
            for(int i = 0; i < 3; ++i)
            {
                m_adjacentTriangles.RemoveAt(triangleIndex * 3);
                m_triangleVertices.RemoveAt(triangleIndex * 3);
            }

            for(int i = 0; i < m_adjacentTriangles.Count; ++i)
            {
                if(m_adjacentTriangles[i] == triangleIndex)
                {
                    m_adjacentTriangles[i] = 0;
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

        public int[] GetAdjacentTriangles(int triangleIndex)
        {
            int[] adjacentTriangles = new int[3];

            for(int i = 0; i < 3; ++i)
            {
                adjacentTriangles[i] = m_adjacentTriangles[triangleIndex + i];
            }

            return adjacentTriangles;
        }

        public void GetIntersectingTriangles(Vector2 edgeEndpointA, Vector2 edgeEndpointB, int startTriangle, List<DelaunayTriangleEdge> intersectingTriangles)
        {
            Debug.Log("Edge to intersect: " + edgeEndpointA.ToString("F6") + " || " + edgeEndpointB.ToString("F6"));

            bool isTriangleContainingBFound = false;
            int triangleIndex = startTriangle;
            int traversedTriangles = 0;

            while (!isTriangleContainingBFound && traversedTriangles != TriangleCount)
            {
                DrawTriangle(triangleIndex, Color.green);

                isTriangleContainingBFound = true;
                bool hasCrossedEdge = false;
                int tentativeAdjacentTriangle = -1;

                for (int i = 0; i < 3; ++i)
                {
                    if(m_points[m_triangleVertices[triangleIndex * 3 + i]] == edgeEndpointB ||
                       m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]] == edgeEndpointB)
                    {
                        break;
                    }

                    if (GeometryUtils.IsPointToTheRightOfEdge(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], edgeEndpointB))
                    {
                        tentativeAdjacentTriangle = i;

                        Debug.Log($"Edge examined (triangle {triangleIndex}, edge {i}->{(i + 1) % 3}): " + m_points[m_triangleVertices[triangleIndex * 3 + i]].ToString("F6") + " || " + m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]].ToString("F6"));

                        Debug.DrawLine(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], Color.green, 10.0f);

                        isTriangleContainingBFound = false;
                        Vector2 intersectionPoint;

                        if (GeometryUtils.InsersectionBetweenLines(m_points[m_triangleVertices[triangleIndex * 3 + i]],
                                                                   m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]],
                                                                   edgeEndpointA,
                                                                   edgeEndpointB,
                                                                   out intersectionPoint))
                        {
                            hasCrossedEdge = true;

                            intersectingTriangles.Add(new DelaunayTriangleEdge(triangleIndex, i, -1, -1));

                            Debug.DrawLine(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], Color.yellow, 10.0f);
                            const float xlineLength = 0.008f;
                            Debug.DrawRay(intersectionPoint - new Vector2(xlineLength * 0.5f, xlineLength * 0.5f), new Vector2(xlineLength, xlineLength), Color.red, 10.0f);
                            Debug.DrawRay(intersectionPoint + new Vector2(-xlineLength * 0.5f, xlineLength * 0.5f), new Vector2(xlineLength, -xlineLength), Color.red, 10.0f);

                            // The point is in the exterior of the triangle (vertices are sorted CCW, the right side is always the exterior from the perspective of the A->B edge)
                            if (m_adjacentTriangles[triangleIndex * 3 + i] != -1)
                            {
                                triangleIndex = m_adjacentTriangles[triangleIndex * 3 + i];
                            }

                            break;
                        }

                    }
                }

                // Continue searching at a different adjacent triangle
                if(!hasCrossedEdge && tentativeAdjacentTriangle != -1 && m_adjacentTriangles[triangleIndex * 3 + tentativeAdjacentTriangle] != -1)
                {
                    triangleIndex = m_adjacentTriangles[triangleIndex * 3 + tentativeAdjacentTriangle];
                }

                ++traversedTriangles;
            }
        }

        // This method assumes that the edges of the triangles to find were created using the same vertex order
        public void GetTrianglesInPolygon(List<int> polygonOutline, List<int> outputTrianglesInPolygon)
        {
            Stack<int> adjacentTriangles = new Stack<int>();

            for(int i = 0; i < polygonOutline.Count; ++i)
            {
                // For an edge, check which of the 2 triangles that share the edge is inside of the polygon
                DelaunayTriangleEdge triangleEdge = FindTriangleThatContainsEdge(polygonOutline[i], polygonOutline[(i + 1) % polygonOutline.Count]);
                /*bool isTriangleInsidePolygon = false;
                
                for(int j = 0; j < 3; ++j)
                {
                    for (int k = 0; k < polygonOutline.Count; ++k)
                    {
                        if(m_triangleVertices[triangleEdge.TriangleIndex * 3 + j] == polygonOutline[k])
                        {
                            isTriangleInsidePolygon = true;
                            break;
                        }
                    }

                    if (isTriangleInsidePolygon)
                    {
                        break;
                    }
                }

                // If it's not one of them, then obviously it's the other one
                if(!isTriangleInsidePolygon)
                { 
                    triangleEdge = FindTriangleThatContainsEdge(polygonOutline[(i + 1) % polygonOutline.Count], polygonOutline[i]);
                }*/

                // Adds the 2 adjacent triangles
                adjacentTriangles.Push(m_adjacentTriangles[triangleEdge.TriangleIndex * 3 + (triangleEdge.EdgeIndex + 1) % 3]);
                adjacentTriangles.Push(m_adjacentTriangles[triangleEdge.TriangleIndex * 3 + (triangleEdge.EdgeIndex + 2) % 3]);

                outputTrianglesInPolygon.Add(triangleEdge.TriangleIndex);
            }

            // TODO: Clean the stack duplicates
            //return;

            while(adjacentTriangles.Count > 0)
            {
                int currentAdjacentTriangle = adjacentTriangles.Pop();
                List<int> adjacentsAlreadyInPolygon = new List<int>();

                bool isCurrentAdjacentTriangleAlreadyInPolygon = false;

                for (int j = 0; j < outputTrianglesInPolygon.Count; ++j)
                {
                    if (currentAdjacentTriangle == outputTrianglesInPolygon[j])
                    {
                        isCurrentAdjacentTriangleAlreadyInPolygon = true;
                    }
                }

                if(isCurrentAdjacentTriangleAlreadyInPolygon)
                {
                    continue;
                }

                for (int j = 0; j < 3; ++j)
                {
                    int currentAdjacentOfAdjacentTriangle = m_adjacentTriangles[currentAdjacentTriangle * 3 + j];

                    // Checks how many adjacent triangles (of the adjacent triangle) are already in the list
                    for (int k = 0; k < outputTrianglesInPolygon.Count; ++k)
                    {
                        if (currentAdjacentOfAdjacentTriangle == outputTrianglesInPolygon[k])
                        {
                            adjacentsAlreadyInPolygon.Add(currentAdjacentOfAdjacentTriangle);
                            break;
                        }
                    }
                }

                // If a triangle has 2 or more adjacent triangles already in the list, it means it is also inside of the polygon
                if (adjacentsAlreadyInPolygon.Count > 1)
                {
                    for (int k = 0; k < 3; ++k)
                    {
                        int adjacentTriangleToAdd = m_adjacentTriangles[currentAdjacentTriangle * 3 + k];

                        bool isAdjacentTriangleAlreadyInPolygon = false;

                        for (int m = 0; m < adjacentsAlreadyInPolygon.Count; ++m)
                        {
                            if (adjacentTriangleToAdd == adjacentsAlreadyInPolygon[m])
                            {
                                isAdjacentTriangleAlreadyInPolygon = true;
                                break;
                            }
                        }

                        if (!isAdjacentTriangleAlreadyInPolygon)
                        {
                            for (int m = 0; m < outputTrianglesInPolygon.Count; ++m)
                            {
                                if (adjacentTriangleToAdd == outputTrianglesInPolygon[m])
                                {
                                    isAdjacentTriangleAlreadyInPolygon = true;
                                    break;
                                }
                            }
                        }

                        if (!isAdjacentTriangleAlreadyInPolygon)
                        {
                            adjacentTriangles.Push(adjacentTriangleToAdd);
                        }

                        outputTrianglesInPolygon.Add(currentAdjacentTriangle);
                    }
                }
            }
        }

        public void GetIntersectingEdges(Vector2 edgeEndpointA, Vector2 edgeEndpointB, int startTriangle, List<DelaunayTriangleEdge> intersectingEdges)
        {
            Debug.Log("Edge to intersect: " + edgeEndpointA.ToString("F6") + " || " + edgeEndpointB.ToString("F6"));

            bool isTriangleContainingBFound = false;
            int triangleIndex = startTriangle;
            int traversedTriangles = 0;

            while (!isTriangleContainingBFound && traversedTriangles != TriangleCount)
            {
                DrawTriangle(triangleIndex, Color.green);

                isTriangleContainingBFound = true;
                bool hasCrossedEdge = false;
                int tentativeAdjacentTriangle = -1;

                for (int i = 0; i < 3; ++i)
                {
                    if (m_points[m_triangleVertices[triangleIndex * 3 + i]] == edgeEndpointB ||
                       m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]] == edgeEndpointB)
                    {
                        break;
                    }

                    if (GeometryUtils.IsPointToTheRightOfEdge(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], edgeEndpointB))
                    {
                        tentativeAdjacentTriangle = i;

                        Debug.Log($"Edge examined (triangle {triangleIndex}, edge {i}->{(i + 1) % 3}): " + m_points[m_triangleVertices[triangleIndex * 3 + i]].ToString("F6") + " || " + m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]].ToString("F6"));

                        Debug.DrawLine(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], Color.green, 10.0f);

                        isTriangleContainingBFound = false;
                        Vector2 intersectionPoint;

                        if (GeometryUtils.InsersectionBetweenLines(m_points[m_triangleVertices[triangleIndex * 3 + i]],
                                                                   m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]],
                                                                   edgeEndpointA,
                                                                   edgeEndpointB,
                                                                   out intersectionPoint))
                        {
                            hasCrossedEdge = true;

                            intersectingEdges.Add(new DelaunayTriangleEdge(-1, -1, m_triangleVertices[triangleIndex * 3 + i], m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]));

                            Debug.DrawLine(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], Color.yellow, 10.0f);
                            const float xlineLength = 0.008f;
                            Debug.DrawRay(intersectionPoint - new Vector2(xlineLength * 0.5f, xlineLength * 0.5f), new Vector2(xlineLength, xlineLength), Color.red, 10.0f);
                            Debug.DrawRay(intersectionPoint + new Vector2(-xlineLength * 0.5f, xlineLength * 0.5f), new Vector2(xlineLength, -xlineLength), Color.red, 10.0f);

                            // The point is in the exterior of the triangle (vertices are sorted CCW, the right side is always the exterior from the perspective of the A->B edge)
                            if (m_adjacentTriangles[triangleIndex * 3 + i] != -1)
                            {
                                triangleIndex = m_adjacentTriangles[triangleIndex * 3 + i];
                            }

                            break;
                        }

                    }
                }

                // Continue searching at a different adjacent triangle
                if (!hasCrossedEdge && tentativeAdjacentTriangle != -1 && m_adjacentTriangles[triangleIndex * 3 + tentativeAdjacentTriangle] != -1)
                {
                    triangleIndex = m_adjacentTriangles[triangleIndex * 3 + tentativeAdjacentTriangle];
                }

                ++traversedTriangles;
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

        public void GetIntersectingTriangles(Vector2 edgeEndpointA, Vector2 edgeEndpointB, List<DelaunayTriangleEdge> intersectingTriangles)
        {
            Debug.Log("Edge to intersect: " + edgeEndpointA.ToString("F6") + " || " + edgeEndpointB.ToString("F6"));

            List<DelaunayTriangleEdge> tentativeTriangles = new List<DelaunayTriangleEdge>();

            for (int i = 0; i < TriangleCount; ++i)
            {
                for(int j = 0; j < 3; ++j)
                {
                    Vector2 intersectionPoint;

                    if (GeometryUtils.InsersectionBetweenLines(m_points[m_triangleVertices[i * 3 + j]],
                                                                m_points[m_triangleVertices[i * 3 + (j + 1) % 3]],
                                                                edgeEndpointA,
                                                                edgeEndpointB,
                                                                out intersectionPoint))
                    {
                        tentativeTriangles.Add(new DelaunayTriangleEdge(i, j, -1, -1));

                        Debug.DrawLine(m_points[m_triangleVertices[i * 3 + j]], m_points[m_triangleVertices[i * (j + 1) % 3]], Color.yellow, 10.0f);
                        const float xlineLength = 0.005f;
                        Debug.DrawRay(intersectionPoint - new Vector2(xlineLength * 0.5f, xlineLength * 0.5f), new Vector2(xlineLength, xlineLength), Color.red, 10.0f);
                        Debug.DrawRay(intersectionPoint + new Vector2(-xlineLength * 0.5f, xlineLength * 0.5f), new Vector2(xlineLength, -xlineLength), Color.red, 10.0f);
                    }
                }
            }

            // Removes repeated edges
            for(int i = 0; i < tentativeTriangles.Count; ++i)
            {
                bool isRepeated = false;

                DelaunayTriangleEdge currentEdge = tentativeTriangles[i];

                for(int j = 0; j < intersectingTriangles.Count; ++j)
                {
                    if((m_triangleVertices[currentEdge.TriangleIndex * 3 + currentEdge.EdgeIndex] == m_triangleVertices[intersectingTriangles[j].TriangleIndex * 3 + intersectingTriangles[j].EdgeIndex] &&
                        m_triangleVertices[currentEdge.TriangleIndex * 3 + (currentEdge.EdgeIndex + 1) % 3] == m_triangleVertices[intersectingTriangles[j].TriangleIndex * 3 + (intersectingTriangles[j].EdgeIndex + 1) % 3])
                        || // Edge vertices may be in the opposite order in the other triangle's edge
                       (m_triangleVertices[currentEdge.TriangleIndex * 3 + currentEdge.EdgeIndex] == m_triangleVertices[intersectingTriangles[j].TriangleIndex * 3 + (intersectingTriangles[j].EdgeIndex + 1) % 3] &&
                        m_triangleVertices[currentEdge.TriangleIndex * 3 + (currentEdge.EdgeIndex + 1) % 3] == m_triangleVertices[intersectingTriangles[j].TriangleIndex * 3 + intersectingTriangles[j].EdgeIndex]))
                    {
                        isRepeated = true;
                        break;
                    }
                }

                if(!isRepeated)
                {
                    intersectingTriangles.Add(currentEdge);
                }
            }
        }

        public DelaunayTriangleEdge FindTriangleThatContainsEdge(int edgeVertexA, int edgeVertexB)
        {
            DelaunayTriangleEdge foundTriangle = new DelaunayTriangleEdge(-1, -1, edgeVertexA, edgeVertexB);

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
            int traversedTriangles = 0; 
            
            while(!isTriangleFound && traversedTriangles != TriangleCount)
            {
                isTriangleFound = true;

                for (int i = 0; i < 3; ++i)
                {
                    if (GeometryUtils.IsPointToTheRightOfEdge(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], point))
                    {
                        // The point is in the exterior of the triangle (vertices are sorted CCW, the right side is always the exterior from the perspective of the A->B edge)
                        if(m_adjacentTriangles[triangleIndex * 3 + i] != -1)
                        {
                            triangleIndex = m_adjacentTriangles[triangleIndex * 3 + i];
                        }
                        
                        isTriangleFound = false;
                        break;
                    }
                }

                ++traversedTriangles;
            }
            
            return isTriangleFound ? triangleIndex : -1;
        }

        public int FindTriangleThatContainsLineEndpoint(int endpointIndex, Vector2 lineDirection)
        {
            List<int> trianglesWithEndpoint = new List<int>();
            GetTrianglesWithVertex(endpointIndex, trianglesWithEndpoint);

            int foundTriangle = -1;
            Vector2 endpoint = m_points[endpointIndex];
            Vector2 projectedLinePoint = endpoint + lineDirection;
            Debug.DrawLine(endpoint + Vector2.up * 0.01f, projectedLinePoint + Vector2.up * 0.01f, Color.yellow, 10.0f);

            for (int i = 0; i < trianglesWithEndpoint.Count; ++i)
            {
                DelaunayTriangle triangleDebug = GetTriangle(trianglesWithEndpoint[i]);
                List<int> pointsDebug = triangleDebug.DebugP;

                int vertexPositionInTriangle = m_triangleVertices[trianglesWithEndpoint[i] * 3] == endpointIndex ? 0 
                                                                                                                 : m_triangleVertices[trianglesWithEndpoint[i] * 3 + 1] == endpointIndex ? 1 
                                                                                                                                                                                         : 2;
                Vector2 triangleEdgePoint1 = m_points[m_triangleVertices[trianglesWithEndpoint[i] * 3 + (vertexPositionInTriangle + 1) % 3]];
                Vector2 triangleEdgePoint2 = m_points[m_triangleVertices[trianglesWithEndpoint[i] * 3 + (vertexPositionInTriangle + 2) % 3]];

                // Is the line in the angle between the 2 contiguous edges of the triangle?
                if (GeometryUtils.IsPointToTheRightOfEdge(triangleEdgePoint1, endpoint, projectedLinePoint) &&
                    GeometryUtils.IsPointToTheRightOfEdge(endpoint, triangleEdgePoint2, projectedLinePoint))
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

