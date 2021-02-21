
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Game.Utils.Geometry
{
    public unsafe class DelaunayTriangulation
    {
        public TriangleRegistry TriangleStorage
        {
            get
            {
                return m_triangles;
            }
        }

        private PointBinGrid m_grid;
        private TriangleRegistry m_triangles;
        private Stack<int> m_triangleStack;

        public void Triangulation(List<Vector2> inputPoints, List<Triangle2D> outputTriangles, List<List<Vector2>> constrainedEdges = null)
        {
            //Vector2 x; 
            //GeometryUtils.InsersectionBetweenLines(new Vector2(0.229538f, 0.254069f), new Vector2(0.239803f, 0.140073f), new Vector2(0.270452f, 0.206589f), new Vector2(0.217924f, 0.152985f), out x);

            // Initialize containers
            outputTriangles.Clear();
            m_triangles = new TriangleRegistry(inputPoints.Count);
            m_triangleStack = new Stack<int>(inputPoints.Count);

            // 1
            Bounds pointCloudBounds = CalculateBoundsWithLeftBottomCornerAtOrigin(inputPoints);

            List<Vector2> normalizedPoints = new List<Vector2>(inputPoints.Count);
            NormalizePoints(inputPoints, pointCloudBounds, normalizedPoints);

            //DelaunayTriangulation.DrawPoints(normalizedPoints, 30.0f);

            // 2
            Bounds normalizedCloudBounds = CalculateBoundsWithLeftBottomCornerAtOrigin(normalizedPoints);

            AddPointsToGrid(normalizedPoints, normalizedCloudBounds, out m_grid);

            //m_grid.DrawGrid(new Color(0.0f, 0.0f, 1.0f, 0.2f), 30.0f);

            // 3
            Triangle2D supertriangle = new Triangle2D(new Vector2(-100.0f, -100.0f), new Vector2(100.0f, -100.0f), new Vector2(0.0f, 100.0f)); // CCW
            
            m_triangles.AddTriangle(supertriangle.p0, supertriangle.p1, supertriangle.p2, -1, -1, -1);

            // 4

            // Points are added one at a time, and points that are close together are inserted together, so a later step for finding their containing triangle is faster
            for(int i = 0; i < m_grid.cells.Length; ++i)
            {
                if(m_grid.cells[i] != null)
                {
                    for (int j = 0; j < m_grid.cells[i].Count; ++j)
                    {
                        AddPointToTriangulation(m_grid.cells[i][j], m_triangles);
                    }
                }
            }

            List<int> trianglesToRemove = new List<int>();

            // CONSTRAINED EDGES
            if (constrainedEdges != null)
            {
                List<List<int>> constrainedEdgeIndices = new List<List<int>>();

                // Adds all the points
                for(int i = 0; i < constrainedEdges.Count; ++i)
                {
                    List<Vector2> normalizedConstrainedEdges = new List<Vector2>(inputPoints.Count);
                    NormalizePoints(constrainedEdges[i], pointCloudBounds, normalizedConstrainedEdges);

                    List<int> polygonEdgeIndices = new List<int>(normalizedConstrainedEdges.Count);

                    // 1
                    for (int j = 0; j < normalizedConstrainedEdges.Count - 0; ++j)
                    {
                        if (normalizedConstrainedEdges[j] == normalizedConstrainedEdges[(j + 1) % normalizedConstrainedEdges.Count])
                        {
                            Debug.LogWarning($"The list of constrained edges contains a zero-length edge (2 consecutive coinciding points, indices {j} and {(j + 1) % normalizedConstrainedEdges.Count}). It will be ignored.");
                            continue;
                        }

                        int addedPointIndex = AddPointToTriangulation(normalizedConstrainedEdges[j], m_triangles);
                        polygonEdgeIndices.Add(addedPointIndex);

                        Debug.DrawLine(normalizedConstrainedEdges[j], normalizedConstrainedEdges[(j + 1) % normalizedConstrainedEdges.Count], Color.cyan, 5.0f);
                    }

                    constrainedEdgeIndices.Add(polygonEdgeIndices);
                }

                for(int i = 0; i < constrainedEdgeIndices.Count; ++i)
                {
                    for (int j = 0; j < constrainedEdgeIndices[i].Count - 0; ++j)
                    {
                        AddConstrainedEdgeToTriangulation(constrainedEdgeIndices[i][j], constrainedEdgeIndices[i][(j + 1) % constrainedEdgeIndices[i].Count]);
                    }
                }

                // Removes all the constrained triangles
                for (int i = 0; i < constrainedEdgeIndices.Count; ++i)
                {
                    m_triangles.GetTrianglesInPolygon(constrainedEdgeIndices[i], trianglesToRemove);
                }

                // Remove all the triangles left that are not part of the main cloud
                // TODO: How?
            }

            // Last: Remove supertriangle vertices
            GetSupertriangleTriangles(trianglesToRemove);

            for (int i = 0; i < trianglesToRemove.Count; ++i)
            {
                m_triangles.DrawTriangle(trianglesToRemove[i], Color.red);
            }

            // OUTPUT
            List<Vector2> denormalizedPoints = new List<Vector2>(m_triangles.TriangleCount);
            DenormalizePoints(m_triangles.Points, pointCloudBounds, denormalizedPoints);

            for(int i = 0; i < m_triangles.TriangleCount; ++i)
            {
                int j = 0;

                for (; j < trianglesToRemove.Count; ++j)
                {
                    if(trianglesToRemove[j] == i)
                    {
                        break;
                    }
                }

                if(j == trianglesToRemove.Count) // The triangle is not in the "To Remove" list
                {
                    DelaunayTriangle triangle = m_triangles.GetTriangle(i);
                    outputTriangles.Add(new Triangle2D(denormalizedPoints[triangle.p[0]], denormalizedPoints[triangle.p[1]], denormalizedPoints[triangle.p[2]]));
                }
            }
            
            m_triangles.LogDump();
        }

        private void GetSupertriangleTriangles(List<int> outputTriangles)
        {
            for(int i = 0; i < 3; ++i) // Vertices of the supertriangle
            {
                List<int> trianglesThatShareVertex = new List<int>();

                m_triangles.GetTrianglesWithVertex(i, trianglesThatShareVertex);

                outputTriangles.AddRange(trianglesThatShareVertex);
            }
        }

        private int AddPointToTriangulation(Vector2 pointToInsert, TriangleRegistry triangles)
        {
            // Note: Adjacent triangle, opposite to the inserted point, is always at index 1
            // Note 2: Adjacent triangles are stored CCW automatically, their index matches the index of the first vertex in every edge, and it is known that vertices are stored CCW

            int existingPointIndex = m_triangles.GetIndexOfPoint(pointToInsert);

            if (existingPointIndex != -1)
            {
                return existingPointIndex;
            }

            // 5
            int containingTriangleIndex = triangles.FindTriangleThatContainsPoint(pointToInsert, m_triangles.TriangleCount - 1);

            DelaunayTriangle containingTriangle = triangles.GetTriangle(containingTriangleIndex);

            // Inserting a new point into a triangle splits it into 3 pieces, 3 new triangles
            int insertedPoint = m_triangles.AddPoint(pointToInsert);

            // Adds 2 new triangles
            DelaunayTriangle newTriangle1 = new DelaunayTriangle(insertedPoint, containingTriangle.p[0], containingTriangle.p[1]);
            newTriangle1.adjacent[0] = -1;
            newTriangle1.adjacent[1] = containingTriangle.adjacent[0];
            newTriangle1.adjacent[2] = containingTriangleIndex;
            int triangle1Index = m_triangles.AddTriangle(newTriangle1);

            DelaunayTriangle newTriangle2 = new DelaunayTriangle(insertedPoint, containingTriangle.p[2], containingTriangle.p[0]);
            newTriangle2.adjacent[0] = containingTriangleIndex;
            newTriangle2.adjacent[1] = containingTriangle.adjacent[2];
            newTriangle2.adjacent[2] = -1;
            int triangle2Index = m_triangles.AddTriangle(newTriangle2);

            // Sets adjacency between the 2 new triangles
            newTriangle1.adjacent[0] = triangle2Index;
            newTriangle2.adjacent[2] = triangle1Index;
            m_triangles.SetTriangleAdjacency(triangle1Index, newTriangle1.adjacent);
            m_triangles.SetTriangleAdjacency(triangle2Index, newTriangle2.adjacent);

            // Sets the adjacency of the triangles that were adjacent to the original containing triangle
            if (newTriangle1.adjacent[1] != -1)
            {
                m_triangles.ReplaceAdjacent(newTriangle1.adjacent[1], containingTriangleIndex, triangle1Index);
            }

            if (newTriangle2.adjacent[1] != -1)
            {
                m_triangles.ReplaceAdjacent(newTriangle2.adjacent[1], containingTriangleIndex, triangle2Index);
            }

            // Original triangle is transformed into the third triangle after the point has split the containing triangle into 3
            containingTriangle.p[0] = insertedPoint;
            containingTriangle.adjacent[0] = triangle1Index;
            containingTriangle.adjacent[2] = triangle2Index;
            m_triangles.ReplaceTriangle(containingTriangleIndex, containingTriangle);

            // Triangles that contain the inserted point are added to the stack for them to be processed by the Delaunay swapping algorithm
            if(containingTriangle.adjacent[1] != -1)
            {
                m_triangleStack.Push(containingTriangleIndex);
            }

            if(newTriangle1.adjacent[1] != -1)
            {
                m_triangleStack.Push(triangle1Index);
            }

            if (newTriangle2.adjacent[1] != -1)
            {
                m_triangleStack.Push(triangle2Index);
            }

            // 6
            EdgeSwapping();

            return insertedPoint;
        }

        private void EdgeSwapping()
        {
            while(m_triangleStack.Count > 0)
            {
                int currentTriangleToSwap = m_triangleStack.Pop();
                DelaunayTriangle triangle = m_triangles.GetTriangle(currentTriangleToSwap);
                Triangle2D trianglePoints = m_triangles.GetTrianglePoints(currentTriangleToSwap);

                if(triangle.adjacent[1] == -1)
                {
                    continue;
                }

                DelaunayTriangle oppositeTriangle = m_triangles.GetTriangle(triangle.adjacent[1]);
                Triangle2D oppositeTrianglePoints = m_triangles.GetTrianglePoints(triangle.adjacent[1]);

                if(GeometryUtils.IsPointInsideCircumcircle(oppositeTrianglePoints.p0, oppositeTrianglePoints.p1, oppositeTrianglePoints.p2, trianglePoints.p0))
                {
                    // Finds the edge of the opposite triangle that is shared with the other triangle, this edge will be swapped
                    int sharedEdgeVertex = 0;

                    for (; sharedEdgeVertex < 3; ++sharedEdgeVertex)
                    {
                        if (oppositeTriangle.adjacent[sharedEdgeVertex] == currentTriangleToSwap)
                        {
                            break;
                        }
                    }

                    SwapEdges(currentTriangleToSwap, triangle, 0, oppositeTriangle, sharedEdgeVertex);

                    // Adds the 2 triangles that were adjacent to the opposite triangle, to be processed too
                    if(oppositeTriangle.adjacent[(sharedEdgeVertex + 1) % 3] != -1)
                    {
                        m_triangleStack.Push(oppositeTriangle.adjacent[(sharedEdgeVertex + 1) % 3]);
                    }

                    if (oppositeTriangle.adjacent[(sharedEdgeVertex + 2) % 3] != -1)
                    {
                        m_triangleStack.Push(oppositeTriangle.adjacent[(sharedEdgeVertex + 2) % 3]);
                    }
                }
            }
        }

        // sharedEdgeVertex: the index in the first triangle, 0-2
        // for the first triangle, its shared edge vertex is moved, so the new shared edge vertex is 1 position behind / or 2 forward (if it was 1, now the shared edge is 0)
        private void SwapEdges(int triangleIndex, DelaunayTriangle triangle, int notInEdgeTriangleVertex, DelaunayTriangle oppositeTriangle, int oppositeTriangleSharedEdgeVertex)
        {
            List<int> debugP = triangle.DebugP;
            List<int> debugA = triangle.DebugAdjacent;
            List<int> debugP2 = oppositeTriangle.DebugP;
            List<int> debugA2 = oppositeTriangle.DebugAdjacent;

            int oppositeVertex = (oppositeTriangleSharedEdgeVertex + 2) % 3;

            //           2 _|_ a
            //       A2 _   |   _
            //       _      |      _
            //   0 _     A1 |         _  c (opposite vertex)
            //       _      |      _
            //          _   |   _
            //       A0   _ |_
            //              |
            //            1    b

            //           2 _|_ 
            //       A2 _       _ A1
            //       _             _
            //   0 _________A0_______ 1
            //   a   _             _  c
            //          _       _
            //             _ _
            //              | b
            //            

            // Only one vertex of each triangle is moved
            int oppositeTriangleIndex = triangle.adjacent[(notInEdgeTriangleVertex + 1) % 3];
            triangle.p[(notInEdgeTriangleVertex + 1) % 3] = oppositeTriangle.p[oppositeVertex];
            oppositeTriangle.p[oppositeTriangleSharedEdgeVertex] = triangle.p[notInEdgeTriangleVertex];
            oppositeTriangle.adjacent[oppositeTriangleSharedEdgeVertex] = triangle.adjacent[notInEdgeTriangleVertex];
            triangle.adjacent[notInEdgeTriangleVertex] = oppositeTriangleIndex;
            triangle.adjacent[(notInEdgeTriangleVertex + 1) % 3] = oppositeTriangle.adjacent[oppositeVertex];
            oppositeTriangle.adjacent[oppositeVertex] = triangleIndex;

            m_triangles.ReplaceTriangle(triangleIndex, triangle);
            m_triangles.ReplaceTriangle(oppositeTriangleIndex, oppositeTriangle);

            // Adjacent triangles are updated too
            if (triangle.adjacent[(notInEdgeTriangleVertex + 1) % 3] != -1)
            {
                m_triangles.ReplaceAdjacent(triangle.adjacent[(notInEdgeTriangleVertex + 1) % 3], oppositeTriangleIndex, triangleIndex);
            }

            if (oppositeTriangle.adjacent[oppositeTriangleSharedEdgeVertex] != -1)
            {
                m_triangles.ReplaceAdjacent(oppositeTriangle.adjacent[oppositeTriangleSharedEdgeVertex], triangleIndex, oppositeTriangleIndex);
            }
        }

        private void AddConstrainedEdgeToTriangulation(int endpointAIndex, int endpointBIndex)
        {
            // 2
            // Detects if the edge already exists
            if (m_triangles.FindTriangleThatContainsEdge(endpointAIndex, endpointBIndex).TriangleIndex != -1)
            {
                return;
            }

            Vector2 edgeEndpointA = m_triangles.GetPointByIndex(endpointAIndex);
            Vector2 edgeEndpointB = m_triangles.GetPointByIndex(endpointBIndex);

            List<DelaunayTriangleEdge> intersectedTriangleEdges = new List<DelaunayTriangleEdge>();
            int triangleContainingA = m_triangles.FindTriangleThatContainsLineEndpoint(endpointAIndex, (edgeEndpointB - edgeEndpointA));

            List<DelaunayTriangleEdge> newEdges = new List<DelaunayTriangleEdge>();

            bool debugStop = false;
            bool isConstrainedEdgeFinished = false;

            //while(!isConstrainedEdgeFinished)
            {
                isConstrainedEdgeFinished = true;
                intersectedTriangleEdges.Clear();

                if (triangleContainingA != -1)
                {
//                    m_triangles.GetIntersectingTriangles(edgeEndpointA, edgeEndpointB, triangleContainingA, intersectedTriangleEdges);
                    m_triangles.GetIntersectingEdges(edgeEndpointA, edgeEndpointB, triangleContainingA, intersectedTriangleEdges);
                }
                /*else
                {
                    m_triangles.GetIntersectingTriangles(edgeEndpointA, edgeEndpointB, intersectedTriangleEdges);
                }*/

                // 3
                while (intersectedTriangleEdges.Count > 0 && !debugStop)
                {
                    // 3.1
                    DelaunayTriangleEdge currentIntersectedTriangleEdge = intersectedTriangleEdges[intersectedTriangleEdges.Count - 1];
                    intersectedTriangleEdges.RemoveAt(intersectedTriangleEdges.Count - 1);

                    // 3.2
                    currentIntersectedTriangleEdge = m_triangles.FindTriangleThatContainsEdge(currentIntersectedTriangleEdge.EdgeVertexA, currentIntersectedTriangleEdge.EdgeVertexB);
                    DelaunayTriangle intersectedTriangle = m_triangles.GetTriangle(currentIntersectedTriangleEdge.TriangleIndex);
                    DelaunayTriangle oppositeTriangle = m_triangles.GetTriangle(intersectedTriangle.adjacent[currentIntersectedTriangleEdge.EdgeIndex]);
                    Triangle2D trianglePoints = m_triangles.GetTrianglePoints(currentIntersectedTriangleEdge.TriangleIndex);

                    // Gets the opposite vertex of adjacent triangle, knowing the fisrt vertex of the shared edge
                    int oppositeVertex = -1;

                    List<int> debugP = intersectedTriangle.DebugP;
                    List<int> debugA = intersectedTriangle.DebugAdjacent;
                    List<int> debugP2 = oppositeTriangle.DebugP;
                    List<int> debugA2 = oppositeTriangle.DebugAdjacent;

                    int oppositeSharedEdgeVertex = -1; // The first vertex in the shared edge of the opposite triangle

                    for (int j = 0; j < 3; ++j)
                    {
                        if (oppositeTriangle.p[j] == intersectedTriangle.p[currentIntersectedTriangleEdge.EdgeIndex])
                        {
                            oppositeVertex = oppositeTriangle.p[(j + 1) % 3];
                            oppositeSharedEdgeVertex = (j + 2) % 3;
                            break;
                        }
                    }

                    Vector2 oppositePoint = m_triangles.GetPointByIndex(oppositeVertex);

                    if (GeometryUtils.IsQuadrilateralConvex(trianglePoints.p0, trianglePoints.p1, trianglePoints.p2, oppositePoint))
                    {
                        // Swap
                        int notInEdgeTriangleVertex = (currentIntersectedTriangleEdge.EdgeIndex + 2) % 3;
                        SwapEdges(currentIntersectedTriangleEdge.TriangleIndex, intersectedTriangle, notInEdgeTriangleVertex, oppositeTriangle, oppositeSharedEdgeVertex);

                        // Refreshes triangle data after swapping
                        intersectedTriangle = m_triangles.GetTriangle(currentIntersectedTriangleEdge.TriangleIndex);
                        oppositeTriangle = m_triangles.GetTriangle(intersectedTriangle.adjacent[(currentIntersectedTriangleEdge.EdgeIndex + 2) % 3]);

                        debugP = intersectedTriangle.DebugP;
                        debugA = intersectedTriangle.DebugAdjacent;
                        debugP2 = oppositeTriangle.DebugP;
                        debugA2 = oppositeTriangle.DebugAdjacent;

                        // Check new diagonal against the intersecting edge
                        Vector2 intersectionPoint;
                        int newTriangleSharedEdgeVertex = (currentIntersectedTriangleEdge.EdgeIndex + 2) % 3; // Read SwapEdges method to understand the +2
                        Vector2 newTriangleSharedEdgePointA = m_triangles.GetPointByIndex(intersectedTriangle.p[newTriangleSharedEdgeVertex]);
                        Vector2 newTriangleSharedEdgePointB = m_triangles.GetPointByIndex(intersectedTriangle.p[(newTriangleSharedEdgeVertex  + 1) % 3]);

                        DelaunayTriangleEdge newEdge = new DelaunayTriangleEdge(currentIntersectedTriangleEdge.TriangleIndex, newTriangleSharedEdgeVertex, intersectedTriangle.p[newTriangleSharedEdgeVertex], intersectedTriangle.p[(newTriangleSharedEdgeVertex + 1) % 3]);

                        if (newTriangleSharedEdgePointA != edgeEndpointB && newTriangleSharedEdgePointB != edgeEndpointB && // Watch out! It thinks the line intersects with the edge when an endpoint coincides with a triangle vertex
                            newTriangleSharedEdgePointA != edgeEndpointA && newTriangleSharedEdgePointB != edgeEndpointA &&
                            GeometryUtils.InsersectionBetweenLines(edgeEndpointA, edgeEndpointB, newTriangleSharedEdgePointA, newTriangleSharedEdgePointB, out intersectionPoint))
                        {
                            // New triangles edge still intersects with the constrained edge, so it is returned to the list
                            intersectedTriangleEdges.Insert(0, newEdge);
                            isConstrainedEdgeFinished = false;
                        }
                        else
                        {
                            newEdges.Add(newEdge);
                        }
                    }
                    else
                    {
                        // Back to the list
                        intersectedTriangleEdges.Insert(0, currentIntersectedTriangleEdge);
                        isConstrainedEdgeFinished = false;
                    }
                }
            }

            // 4
//return;
            // 4.1
            if(newEdges.Count > 0)
            {
                int i = -1;

                do
                {
                    ++i;

                    // 4.2
                    // Checks if the constrained edge coincides with the new edge
                    DelaunayTriangleEdge currentEdge = m_triangles.FindTriangleThatContainsEdge(newEdges[i].EdgeVertexA, newEdges[i].EdgeVertexB);
                    DelaunayTriangle currentEdgeTriangle = m_triangles.GetTriangle(currentEdge.TriangleIndex);
//                    DelaunayTriangle currentEdgeTriangle = m_triangles.GetTriangle(newEdges[i].TriangleIndex);
                    Vector2 triangleEdgePointA = m_triangles.GetPointByIndex(currentEdgeTriangle.p[currentEdge.EdgeIndex]);
                    Vector2 triangleEdgePointB = m_triangles.GetPointByIndex(currentEdgeTriangle.p[(currentEdge.EdgeIndex + 1) % 3]);

                    if ((triangleEdgePointA == edgeEndpointA && triangleEdgePointB == edgeEndpointB) ||
                        (triangleEdgePointB == edgeEndpointA && triangleEdgePointA == edgeEndpointB))
                    {
                        continue;
                    }

                    // 4.3
                    if (currentEdgeTriangle.adjacent[1] == -1)
                    {
                        // TODO: Should add a new point that splits the triangle in 2?
                        continue;
                    }

                    int triangleVertexNotShared = (currentEdge.EdgeIndex + 2) % 3;
                    Vector2 trianglePointNotShared = m_triangles.GetPointByIndex(currentEdgeTriangle.p[triangleVertexNotShared]);
                    DelaunayTriangle oppositeTriangle = m_triangles.GetTriangle(currentEdgeTriangle.adjacent[currentEdge.EdgeIndex]);
                    Triangle2D oppositeTrianglePoints = m_triangles.GetTrianglePoints(currentEdgeTriangle.adjacent[currentEdge.EdgeIndex]);

                    List<int> debugP = currentEdgeTriangle.DebugP;
                    List<int> debugA = currentEdgeTriangle.DebugAdjacent;
                    List<int> debugP2 = oppositeTriangle.DebugP;
                    List<int> debugA2 = oppositeTriangle.DebugAdjacent;

                    if (GeometryUtils.IsPointInsideCircumcircle(oppositeTrianglePoints.p0, oppositeTrianglePoints.p1, oppositeTrianglePoints.p2, trianglePointNotShared))
                    {
                        // Finds the edge of the opposite triangle that is shared with the other triangle, this edge will be swapped
                        int sharedEdgeVertex = 0;

                        for (; sharedEdgeVertex < 3; ++sharedEdgeVertex)
                        {
                            if (oppositeTriangle.adjacent[sharedEdgeVertex] == currentEdge.TriangleIndex)
                            {
                                break;
                            }
                        }

                        SwapEdges(currentEdge.TriangleIndex, currentEdgeTriangle, triangleVertexNotShared, oppositeTriangle, sharedEdgeVertex);
                    }

                } while (i < newEdges.Count - 1);
            }

            Debug.DrawLine(edgeEndpointA, edgeEndpointB, Color.magenta, 10.0f);
        }

        private Bounds CalculateBoundsWithLeftBottomCornerAtOrigin(List<Vector2> points)
        {
            Vector2 newMin = new Vector2(float.MaxValue, float.MaxValue);
            Vector2 newMax = new Vector2(float.MinValue, float.MinValue);

            for (int i = 0; i < points.Count; ++i)
            {
                if(points[i].x > newMax.x)
                {
                    newMax.x = points[i].x;
                }

                if (points[i].y > newMax.y)
                {
                    newMax.y = points[i].y;
                }

                if (points[i].x < newMin.x)
                {
                    newMin.x = points[i].x;
                }

                if (points[i].y < newMin.y)
                {
                    newMin.y = points[i].y;
                }
            }

            Vector2 size = new Vector2(Mathf.Abs(newMax.x - newMin.x), Mathf.Abs(newMax.y - newMin.y));

            return new Bounds(size * 0.5f + newMin, size);
        }

        private void NormalizePoints(List<Vector2> inputPoints, Bounds bounds, List<Vector2> outputNormalizedPoints)
        {
            float maximumDimension = Mathf.Max(bounds.size.x, bounds.size.y);

            for(int i = 0; i < inputPoints.Count; ++i)
            {
                outputNormalizedPoints.Add((inputPoints[i] - (Vector2)bounds.min) / maximumDimension);
            }
        }

        private void DenormalizePoints(List<Vector2> inputPoints, Bounds bounds, List<Vector2> outputNormalizedPoints)
        {
            float maximumDimension = Mathf.Max(bounds.size.x, bounds.size.y);

            for (int i = 0; i < inputPoints.Count; ++i)
            {
                outputNormalizedPoints.Add(inputPoints[i] * maximumDimension + (Vector2)bounds.min);
            }
        }

        private void AddPointsToGrid(List<Vector2> inputPoints, Bounds gridBounds, out PointBinGrid outputGrid)
        {
            outputGrid = new PointBinGrid(Mathf.FloorToInt(Mathf.Sqrt(inputPoints.Count) * 0.5f + 1.0f), gridBounds.size);

            for (int i = 0; i < inputPoints.Count; ++i)
            {
                outputGrid.AddPoint(inputPoints[i]);
            }
        }

        public static void DrawPoints(List<Vector2> points, float duration)
        {
            for(int i = 0; i < points.Count; ++i)
            {
                Debug.DrawRay(points[i], Vector2.up * 0.2f, Color.red, duration);
                Debug.DrawRay(points[i], Vector2.right * 0.2f, Color.green, duration);
            }
        }

        private bool IsEdgeInTheTriangulation(Vector2 edgeEndpointA, Vector2 edgeEndpointB)
        {
            List<int> trianglesWithVertex = new List<int>();
            int pointOfTriangleA = m_triangles.GetIndexOfPoint(edgeEndpointA);
            int pointOfTriangleB = m_triangles.GetIndexOfPoint(edgeEndpointB);
            m_triangles.GetTrianglesWithVertex(pointOfTriangleA, trianglesWithVertex);

            for (int i = 0; i < trianglesWithVertex.Count; ++i)
            {
                DelaunayTriangle currentTriangle = m_triangles.GetTriangle(trianglesWithVertex[i]);
                int indexOfTrianglePoint = 0;

                for(; indexOfTrianglePoint < 3; ++indexOfTrianglePoint)
                {
                    if (currentTriangle.p[indexOfTrianglePoint] == pointOfTriangleA &&
                        (currentTriangle.p[(indexOfTrianglePoint + 1) % 3] == pointOfTriangleB ||
                         currentTriangle.p[(indexOfTrianglePoint + 2) % 3] == pointOfTriangleB))
                    {
                        return true;
                    }
                }
            }

            return false;
        }
    }
}

