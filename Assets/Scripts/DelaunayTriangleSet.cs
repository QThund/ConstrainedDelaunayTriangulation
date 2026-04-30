// Copyright 2021 Alejandro Villalba Avila
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
// IN THE SOFTWARE.

using Game.Utils.Math;
using System.Collections.Generic;
using UnityEngine;

namespace Game.Utils.Triangulation
{
    /// <summary>
    /// Stores data related to triangles, their vertices and their adjacency and provides methods to gather and process that data.
    /// </summary>
    public unsafe class DelaunayTriangleSet
    {
        /// <summary>
        /// The indices of the adjacent triangles of every triangle, so there are 3 indices per triangle, and each index is the position of the triangle in groups of 3.
        /// </summary>
        protected List<int> m_adjacentTriangles;

        /// <summary>
        /// The indices of the vertices of every triangle, so there are 3 indices per triangle, and each index is the position of the point in the points array.
        /// </summary>
        protected List<int> m_triangleVertices;

        /// <summary>
        /// The real points in the 2D space.
        /// </summary>
        protected List<Vector2> m_points;
        
        // Indicates that the index of a vertex, edge or triangle is not defined or was not found
        protected const int NOT_FOUND = -1;

        // Indicates that there is no adjacent triangle
        protected const int NO_ADJACENT_TRIANGLE = -1;

        /// <summary>
        /// Constructor that receives the expected number of triangles to store. It will reserve memory accordingly.
        /// </summary>
        /// <param name="expectedTriangles">The expected number of triangles to store.</param>
        public DelaunayTriangleSet(int expectedTriangles)
        {
            m_adjacentTriangles = new List<int>(expectedTriangles * 3);
            m_triangleVertices = new List<int>(expectedTriangles * 3);
            m_points = new List<Vector2>(expectedTriangles);
        }

        /// <summary>
        /// Removes all the data stored in the buffers, while keeping the memory.
        /// </summary>
        public void Clear()
        {
            m_adjacentTriangles.Clear();
            m_triangleVertices.Clear();
            m_points.Clear();;
        }

        /// <summary>
        /// Modifies the capacity of the buffer, reserving new memory if necessary, according to the new expected number of triangles.
        /// </summary>
        /// <param name="expectedTriangles">The expected number of triangles to store.</param>
        public void SetCapacity(int expectedTriangles)
        {
            if(m_adjacentTriangles.Capacity < expectedTriangles * 3)
            {
                m_adjacentTriangles.Capacity = expectedTriangles * 3;
            }
            
            if(m_triangleVertices.Capacity < expectedTriangles * 3)
            {
                m_triangleVertices.Capacity = expectedTriangles * 3;
            }

            if(m_triangleVertices.Capacity < expectedTriangles * 3)
            {
                m_points.Capacity = expectedTriangles;
            }
            
        }

        /// <summary>
        /// Gets all the points of the stored triangles.
        /// </summary>
        public List<Vector2> Points
        {
            get
            {
                return m_points;
            }
        }

        /// <summary>
        /// Gets the indices of the vertices of all the stored triangles.
        /// </summary>
        public List<int> Triangles
        {
            get
            {
                return m_triangleVertices;
            }
        }

        /// <summary>
        ///  Gets the amount triangles store.
        /// </summary>
        public int TriangleCount
        {
            get
            {
                return m_triangleVertices.Count / 3;
            }
        }

        /// <summary>
        /// Forms a new triangle using the existing points.
        /// </summary>
        /// <param name="newTriangle">All the data that describe triangle.</param>
        /// <returns>The index of the new triangle.</returns>
        public int AddTriangle(DelaunayTriangle newTriangle)
        {
            m_adjacentTriangles.Add(newTriangle.adjacent[0]);
            m_adjacentTriangles.Add(newTriangle.adjacent[1]);
            m_adjacentTriangles.Add(newTriangle.adjacent[2]);
            m_triangleVertices.Add(newTriangle.p[0]);
            m_triangleVertices.Add(newTriangle.p[1]);
            m_triangleVertices.Add(newTriangle.p[2]);

            return TriangleCount - 1;
        }

        /// <summary>
        /// Adds a new point to the triangle set. This does neither form triangles nor edges.
        /// </summary>
        /// <param name="point">The new point.</param>
        /// <returns>The index of the point.</returns>
        public int AddPoint(Vector2 point)
        {
            m_points.Add(point);
            return m_points.Count - 1;
        }

        /// <summary>
        /// Forms a new triangle using new points.
        /// </summary>
        /// <param name="p0">The point for the first vertex.</param>
        /// <param name="p1">The point for the second vertex.</param>
        /// <param name="p2">The point for the third vertex.</param>
        /// <param name="adjacentTriangle0">The index of the first adjacent triangle.</param>
        /// <param name="adjacentTriangle1">The index of the second adjacent triangle.</param>
        /// <param name="adjacentTriangle2">The index of the third adjacent triangle.</param>
        /// <returns>The index of the new triangle.</returns>
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

        /// <summary>
        /// Given the index of a point, it obtains all the existing triangles that share that point.
        /// </summary>
        /// <param name="vertexIndex">The index of the point that is a vertex of the triangles.</param>
        /// <param name="outputTriangles">The indices of the triangles that have that point as one of their vertices. No elements will be removed from the list.</param>
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

        /// <summary>
        /// Gets the points of a triangle.
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle.</param>
        /// <returns>The triangle.</returns>
        public Triangle2D GetTrianglePoints(int triangleIndex)
        {
            return new Triangle2D(m_points[m_triangleVertices[triangleIndex * 3]],
                                  m_points[m_triangleVertices[triangleIndex * 3 + 1]],
                                  m_points[m_triangleVertices[triangleIndex * 3 + 2]]);
        }

        /// <summary>
        /// Gets the data of a triangle.
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle.</param>
        /// <returns>The triangle data.</returns>
        public DelaunayTriangle GetTriangle(int triangleIndex)
        {
            return new DelaunayTriangle(m_triangleVertices[triangleIndex * 3],
                                        m_triangleVertices[triangleIndex * 3 + 1],
                                        m_triangleVertices[triangleIndex * 3 + 2],
                                        m_adjacentTriangles[triangleIndex * 3],
                                        m_adjacentTriangles[triangleIndex * 3 + 1],
                                        m_adjacentTriangles[triangleIndex * 3 + 2]);
        }

        /// <summary>
        /// Given the outline of a closed polygon, expressed as a list of vertices, it finds all the triangles that lay inside of the figure.
        /// </summary>
        /// <param name="polygonOutline">The outline, a list of vertex indices sorted counter-clockwise.</param>
        /// <param name="outputTrianglesInPolygon">The list where the triangles found inside the polygon will be added. No elements are removed from this list.</param>
        public void GetTrianglesInPolygon(List<int> polygonOutline, List<int> outputTrianglesInPolygon)
        {
            // This method assumes that the edges of the triangles to find were created using the same vertex order
            // It also assumes all triangles are inside a supertriangle, so no adjacent triangles are -1

            Stack<int> adjacentTriangles = new Stack<int>();

            // First it gets all the triangles of the outline
            for (int i = 0; i < polygonOutline.Count; ++i)
            {
                int outlineVertexA = polygonOutline[i];
                int outlineVertexB = polygonOutline[(i + 1) % polygonOutline.Count];

                // Skip zero-length outline edges. Two consecutive hole vertices can resolve to the same triangulation
                // point index when AddPointToTriangulation dedupes near-coincident points via GetIndexOfPoint.
                if (outlineVertexA == outlineVertexB)
                {
                    continue;
                }

                // For every edge, it gets the inner triangle that contains such edge
                DelaunayTriangleEdge triangleEdge = FindTriangleThatContainsEdge(outlineVertexA, outlineVertexB);

                // The directed edge may be absent if AddConstrainedEdgeToTriangulation bailed out (e.g. its no-progress
                // assert path on near-collinear input). Skipping here prevents the negative-index OOB at the adjacency
                // lookup below; the flood-fill may leak through the gap, but the upstream insertion already logged.
                if (triangleEdge.TriangleIndex == NOT_FOUND)
                {
                    UnityEngine.Debug.LogWarning($"GetTrianglesInPolygon: directed outline edge ({outlineVertexA} -> {outlineVertexB}) is not present in the triangulation. The corresponding constrained edge was not fully inserted; flood-fill may produce an incorrect triangle set for this polygon.");
                    continue;
                }

                // A triangle may form a corner, with 2 consecutive outline edges. This avoids adding it twice
                if(outputTrianglesInPolygon.Count > 0 &&
                   (outputTrianglesInPolygon[outputTrianglesInPolygon.Count - 1] == triangleEdge.TriangleIndex || // Is the last added triangle the same as current?
                    outputTrianglesInPolygon[0] == triangleEdge.TriangleIndex)) // Is the first added triangle the same as the current, which is the last to be added (closes the polygon)?
                {
                    continue;
                }

                outputTrianglesInPolygon.Add(triangleEdge.TriangleIndex);

                int previousOutlineEdgeVertexA = polygonOutline[(i + polygonOutline.Count - 1) % polygonOutline.Count];
                int previousOutlineEdgeVertexB = polygonOutline[i];
                int nextOutlineEdgeVertexA = polygonOutline[(i + 1) % polygonOutline.Count];
                int nextOutlineEdgeVertexB = polygonOutline[(i + 2) % polygonOutline.Count];

                for (int j = 1; j < 3; ++j) // For the 2 adjacent triangles of the other 2 edges
                {
                    int adjacentTriangle = m_adjacentTriangles[triangleEdge.TriangleIndex * 3 + (triangleEdge.EdgeIndex + j) % 3];
                    bool isAdjacentTriangleInOutline = false;

                    // Compares the contiguous edges of the outline, to the right and to the left of the current one, flipped and not flipped, with the adjacent triangle's edges
                    for (int k = 0; k < 3; ++k)
                    {
                        int currentTriangleEdgeVertexA = m_triangleVertices[adjacentTriangle * 3 + k];
                        int currentTriangleEdgeVertexB = m_triangleVertices[adjacentTriangle * 3 + (k + 1) % 3];

                        if ((currentTriangleEdgeVertexA == previousOutlineEdgeVertexA && currentTriangleEdgeVertexB == previousOutlineEdgeVertexB) ||
                            (currentTriangleEdgeVertexA == previousOutlineEdgeVertexB && currentTriangleEdgeVertexB == previousOutlineEdgeVertexA) ||
                            (currentTriangleEdgeVertexA == nextOutlineEdgeVertexA && currentTriangleEdgeVertexB == nextOutlineEdgeVertexB) ||
                            (currentTriangleEdgeVertexA == nextOutlineEdgeVertexB && currentTriangleEdgeVertexB == nextOutlineEdgeVertexA))
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
                    int adjacentTriangle = m_adjacentTriangles[currentTriangle * 3 + i];

                    if (adjacentTriangle != NO_ADJACENT_TRIANGLE && !outputTrianglesInPolygon.Contains(adjacentTriangle))
                    {
                        adjacentTriangles.Push(adjacentTriangle);
                    }
                }
                        
                outputTrianglesInPolygon.Add(currentTriangle);
            }
        }

        /// <summary>
        /// Calculates which edges of the triangulation intersect with a proposed line segment AB.
        /// </summary>
        /// <param name="lineEndpointA">The first point of the line segment.</param>
        /// <param name="lineEndpointB">The second point of the line segment.</param>
        /// <param name="startTriangle">The index of the triangle from which to start searching for intersections.</param>
        /// <param name="intersectingEdges">The list where the intersected triangle edges will be added. No elements will be removed from this list.</param>
        public void GetIntersectingEdges(Vector2 lineEndpointA, Vector2 lineEndpointB, int startTriangle, List<DelaunayTriangleEdge> intersectingEdges)
        {
            bool isTriangleContainingBFound = false;
            int triangleIndex = startTriangle;

            while (!isTriangleContainingBFound)
            {
                //DrawTriangle(triangleIndex, Color.green);

                bool hasCrossedEdge = false;
                int tentativeAdjacentTriangle = NO_ADJACENT_TRIANGLE;

                for (int i = 0; i < 3; ++i)
                {
                    if (m_points[m_triangleVertices[triangleIndex * 3 + i]] == lineEndpointB ||
                        m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]] == lineEndpointB)
                    {
                        isTriangleContainingBFound = true;
                        break;
                    }

                    if (MathUtils.IsPointToTheRightOfEdge(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], lineEndpointB))
                    {
                        tentativeAdjacentTriangle = i;

                        //Debug.DrawLine(m_points[m_triangleVertices[triangleIndex * 3 + i]], m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]], Color.green, 10.0f);

                        Vector2 intersectionPoint;

                        if (MathUtils.IntersectionBetweenLines(m_points[m_triangleVertices[triangleIndex * 3 + i]],
                                                               m_points[m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]],
                                                               lineEndpointA,
                                                               lineEndpointB,
                                                               out intersectionPoint))
                        {
                            hasCrossedEdge = true;

                            intersectingEdges.Add(new DelaunayTriangleEdge(triangleIndex, i, m_triangleVertices[triangleIndex * 3 + i], m_triangleVertices[triangleIndex * 3 + (i + 1) % 3]));

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
                    // tentativeAdjacentTriangle == NO_ADJACENT_TRIANGLE means B was not to the right of any edge
                    // and was not found as a vertex. In a valid triangulation where no intermediate vertex lies on
                    // AB (guaranteed by FindIntermediateVerticesOnSegment), this state is unreachable.
                    if (tentativeAdjacentTriangle == NO_ADJACENT_TRIANGLE)
                    {
                        Debug.LogWarning("GetIntersectingEdges: B is not to the right of any edge and not a vertex of the current triangle. This indicates a broken triangulation adjacency invariant or a constraint edge that was not split at an intermediate vertex. Is any of the vertices that define the hole outside of the main convex hull?");
                    }

                    triangleIndex = m_adjacentTriangles[triangleIndex * 3 + tentativeAdjacentTriangle];
                }
            }
        }

        /// <summary>
        /// Gets a point by its index.
        /// </summary>
        /// <param name="pointIndex">The index of the point.</param>
        /// <returns>The point that corresponds to the index.</returns>
        public Vector2 GetPointByIndex(int pointIndex)
        {
            return m_points[pointIndex];
        }

        /// <summary>
        /// Gets the index of a point, if there is any that coincides with it in the triangulation.
        /// </summary>
        /// <param name="point">The point that is expected to exist already.</param>
        /// <returns>The index of the point. If the point does not exist, -1 is returned.</returns>
        public int GetIndexOfPoint(Vector2 point)
        {
            int index = 0;

            while(index < m_points.Count && m_points[index] != point)
            {
                ++index;
            }

            return index == m_points.Count ? -1 : index;
        }

        /// <summary>
        /// Given an edge AB, it searches for the triangle that has an edge with the same vertices in the same order.
        /// </summary>
        /// <remarks>
        /// Remember that the vertices of a triangle are sorted counter-clockwise.
        /// </remarks>
        /// <param name="edgeVertexA">The index of the first vertex of the edge.</param>
        /// <param name="edgeVertexB">The index of the second vertex of the edge.</param>
        /// <returns>The data of the triangle.</returns>
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

        /// <summary>
        /// Constant-time check for whether the given (triangle, local edge) slot still references the directed edge
        /// (edgeVertexA → edgeVertexB). Used to detect when an edge captured earlier in a swap loop has been
        /// destroyed by an intermediate swap, so the caller can skip it without falling back to a full search.
        /// </summary>
        /// <param name="triangleIndex">The triangle index to verify.</param>
        /// <param name="edgeIndex">The local edge index in the triangle (0, 1 or 2).</param>
        /// <param name="edgeVertexA">The expected first vertex of the directed edge.</param>
        /// <param name="edgeVertexB">The expected second vertex of the directed edge.</param>
        /// <returns>True if the slot still holds the directed edge; false otherwise.</returns>
        public bool IsEdgeStillAt(int triangleIndex, int edgeIndex, int edgeVertexA, int edgeVertexB)
        {
            if (triangleIndex < 0 || triangleIndex >= TriangleCount || 
                edgeIndex < 0 || edgeIndex > 2)
            {
                return false;
            }

            return m_triangleVertices[triangleIndex * 3 + edgeIndex] == edgeVertexA &&
                   m_triangleVertices[triangleIndex * 3 + (edgeIndex + 1) % 3] == edgeVertexB;
        }

        /// <summary>
        /// Given a point, it searches for a triangle that contains it.
        /// </summary>
        /// <param name="point">The point expected to be contained by a triangle.</param>
        /// <param name="startTriangle">The index of the first triangle to check.</param>
        /// <returns>The index of the triangle that contains the point.</returns>
        public int FindTriangleThatContainsPoint(Vector2 point, int startTriangle)
        { 
            bool isTriangleFound = false;
            int triangleIndex = startTriangle;
            int checkedTriangles = 0; 

            while(!isTriangleFound && checkedTriangles < TriangleCount)
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

                checkedTriangles++;
            }

            if(checkedTriangles >= TriangleCount && TriangleCount > 1)
            {
                Debug.LogError("Unable to find a triangle that contains the point (" + point.ToString("F6") + "), starting at triangle " + startTriangle + ". Are you generating very small triangles?");
            }

            return triangleIndex;
        }

        /// <summary>
        /// Given an edge AB, it searches for a triangle that contains the first point and the beginning of the edge.
        /// </summary>
        /// <param name="endpointAIndex">The index of the first point.</param>
        /// <param name="endpointBIndex">The index of the second point.</param>
        /// <returns>The index of the triangle that contains the first line endpoint.</returns>
        public int FindTriangleThatContainsLineEndpoint(int endpointAIndex, int endpointBIndex)
        {
            List<int> trianglesWithEndpoint = new List<int>();
            GetTrianglesWithVertex(endpointAIndex, trianglesWithEndpoint);

            int foundTriangle = NOT_FOUND;
            Vector2 endpointA = m_points[endpointAIndex];
            Vector2 endpointB = m_points[endpointBIndex];
            //Debug.DrawLine(endpointA + Vector2.up * 0.01f, endpointB + Vector2.up * 0.01f, Color.yellow, 10.0f);

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

        /// <summary>
        /// Finds all existing triangulation vertices, other than A and B, that lie strictly on the segment AB.
        /// The output list is sorted by ascending distance from A, so iterating it in order yields the sub-segments A→P0→P1→...→B that together tile the original constrained edge.
        /// </summary>
        /// <remarks>
        /// If a constraint edge passes through an existing vertex P_k, the edge must be split into L_ik and L_kj before the intersection walk is attempted, otherwise the walk can
        /// enter an infinite cycle when it reaches P_k and finds no crossing edge toward B.
        /// </remarks>
        /// <param name="indexA">Index of the segment start vertex in the points list.</param>
        /// <param name="indexB">Index of the segment end vertex in the points list.</param>
        /// <param name="output">Receives the sorted intermediate vertex indices. Not cleared on entry.</param>
        public void FindIntermediateVerticesOnSegment(int indexA, int indexB, List<int> output)
        {
            Vector2 a = m_points[indexA];
            Vector2 b = m_points[indexB];
            Vector2 ab = b - a;
            float abLenSq = ab.x * ab.x + ab.y * ab.y;

            if (abLenSq == 0.0f)
            {
                return;
            }

            for (int i = 0; i < m_points.Count; ++i)
            {
                if (i == indexA || i == indexB)
                {
                    continue;
                }

                Vector2 ap = m_points[i] - a;
                float apLenSq = ap.x * ap.x + ap.y * ap.y;

                if (apLenSq == 0.0f)
                {
                    continue; // P coincides with A; defensive — already filtered by indexA equality above
                }               

                // Collinearity check matched to MathUtils.IsPointToTheRightOfEdge: |sin θ| > 1e-4 on normalised vectors.
                // cross² = |AB|²·|AP|²·sin²θ, so the equivalent un-normalised form is cross² > 1e-8 · |AB|² · |AP|².
                float cross = ab.x * ap.y - ab.y * ap.x;
                
                if (cross * cross > 1e-8f * abLenSq * apLenSq)
                {
                    continue;
                }

                // Strictly-between check: project AP onto AB and verify 0 < t < 1
                float proj = ap.x * ab.x + ap.y * ab.y;
                
                if (proj <= 0.0f || proj >= abLenSq)
                {
                    continue;
                }

                output.Add(i);
            }

            // Sort by squared distance from A (monotone proxy for distance, so no sqrt needed)
            output.Sort((i1, i2) =>
                {
                    Vector2 ap1 = m_points[i1] - a;
                    Vector2 ap2 = m_points[i2] - a;
                    float d1 = ap1.x * ap1.x + ap1.y * ap1.y;
                    float d2 = ap2.x * ap2.x + ap2.y * ap2.y;
                    return d1.CompareTo(d2);
                });
        }

        /// <summary>
        /// Stores the adjacency data of a triangle.
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle whose adjacency data is to be written.</param>
        /// <param name="adjacentsToTriangle">The adjacency data, 3 triangle indices sorted counter-clockwise.</param>
        public void SetTriangleAdjacency(int triangleIndex, int* adjacentsToTriangle)
        {
            for(int i = 0; i < 3; ++i)
            {
                m_adjacentTriangles[triangleIndex * 3 + i] = adjacentsToTriangle[i];
            }
        }

        /// <summary>
        /// Given a triangle, it searches for an adjacent triangle and replaces it with another adjacent triangle.
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle whose adjacency data is to be modified.</param>
        /// <param name="oldAdjacentTriangle">The index of the adjacent triangle to be replaced.</param>
        /// <param name="newAdjacentTriangle">The new index of an adjacent triangle that will replace the existing one.</param>
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

        /// <summary>
        /// Replaces all the data of a given triangle. The index of the triangle will remain the same.
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle whose data is to be replaced.</param>
        /// <param name="newTriangle">The new data that will replace the existing one.</param>
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

