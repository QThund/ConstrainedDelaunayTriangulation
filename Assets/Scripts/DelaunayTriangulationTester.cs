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

/// <summary>
/// A test client for the triangulation algorithm. Intended to be used via Inspector.
/// </summary>
public class DelaunayTriangulationTester : MonoBehaviour
{
    [Header("Polygons")]
    [Tooltip("Put here the collider that contains the main point cloud.")]
    public PolygonCollider2D MainPointCloud;

    [Tooltip("Put here the collider that contains the polygons formed by constrained edges.")]
    public PolygonCollider2D ConstrainedEdges;

    [Header("Tilemaps")]
    [Tooltip("Put here the collider of the tilemap that is used as background.")]
    public CompositeCollider2D TilemapBackground;

    [Tooltip("Put here the collider of the tilemap that is used as walls, ground, platforms...")]
    public CompositeCollider2D TilemapBlockers;

    [Header("Settings")]
    [Tooltip("When enabled, the output triangle edges are displayed.")]
    public bool DrawTriangles = true;

    [Tooltip("The mesh that displays the output triangles.")]
    public MeshFilter VisualRepresentation;

    protected List<Triangle2D> m_outputTriangles = new List<Triangle2D>();

    protected DelaunayTriangulation m_triangulation = new DelaunayTriangulation();

    protected void RunTestPolygonColliders()
    {
        Debug.Log("Running Delaunay triangulation test...");

        List<Vector2> pointsToTriangulate = new List<Vector2>();
        ExtractPointsFromCollider(MainPointCloud, pointsToTriangulate);

        List<List<Vector2>> constrainedEdgePoints = new List<List<Vector2>>();

        if (ConstrainedEdges != null)
        {
            ExtractPointsFromCollider(ConstrainedEdges, constrainedEdgePoints);
        }

        m_triangulation.Triangulate(pointsToTriangulate, m_outputTriangles, constrainedEdgePoints);

        VisualRepresentation.mesh = CreateMeshFromTriangles(m_outputTriangles);

        Debug.Log("Test finished.");
    }

    protected void RunTestTilemapColliders()
    {
        Debug.Log("Running Delaunay triangulation test...");

        List<Vector2> pointsToTriangulate = new List<Vector2>();
        ExtractPointsFromCollider(TilemapBackground, pointsToTriangulate);

        List<List<Vector2>> constrainedEdgePoints = new List<List<Vector2>>();

        if (TilemapBlockers != null)
        {
            ExtractPointsFromCollider(TilemapBlockers, constrainedEdgePoints);
        }

        m_triangulation.Triangulate(pointsToTriangulate, m_outputTriangles, constrainedEdgePoints);

        VisualRepresentation.mesh = CreateMeshFromTriangles(m_outputTriangles);

        Debug.Log("Test finished.");
    }

    private void ExtractPointsFromCollider(CompositeCollider2D collider, List<Vector2> outputPoints)
    {
        int pathCount = collider.pathCount;

        for (int i = 0; i < pathCount; ++i)
        {
            List<Vector2> pathPoints = new List<Vector2>();
            collider.GetPath(i, pathPoints);
            outputPoints.AddRange(pathPoints);
        }
    }

    private void ExtractPointsFromCollider(CompositeCollider2D collider, List<List<Vector2>> outpuColliderPolygons)
    {
        int pathCount = collider.pathCount;

        for (int i = 0; i < pathCount; ++i)
        {
            List<Vector2> pathPoints = new List<Vector2>();
            collider.GetPath(i, pathPoints);
            outpuColliderPolygons.Add(pathPoints);
        }
    }

    private Mesh CreateMeshFromTriangles(List<Triangle2D> triangles)
    {
        List<Vector3> vertices = new List<Vector3>(triangles.Count * 3);
        List<int> indices = new List<int>(triangles.Count * 3);

        for (int i = 0; i < triangles.Count; ++i)
        {
            vertices.Add(triangles[i].p0);
            vertices.Add(triangles[i].p1);
            vertices.Add(triangles[i].p2);
            indices.Add(i * 3 + 2); // Changes order
            indices.Add(i * 3 + 1);
            indices.Add(i * 3);
        }

        Mesh mesh = new Mesh();
        mesh.subMeshCount = 1;
        mesh.SetVertices(vertices);
        mesh.SetIndices(indices, MeshTopology.Triangles, 0);
        return mesh;
    }

    private void ExtractPointsFromCollider(PolygonCollider2D collider, List<Vector2> outputPoints)
    {
        int pathCount = collider.pathCount;

        for (int i = 0; i < pathCount; ++i)
        {
            List<Vector2> pathPoints = new List<Vector2>();
            MainPointCloud.GetPath(i, pathPoints);
            outputPoints.AddRange(pathPoints);
        }
    }

    private void ExtractPointsFromCollider(PolygonCollider2D collider, List<List<Vector2>> outpuColliderPolygons)
    {
        int pathCount = collider.pathCount;

        for (int i = 0; i < pathCount; ++i)
        {
            List<Vector2> pathPoints = new List<Vector2>();
            ConstrainedEdges.GetPath(i, pathPoints);
            outpuColliderPolygons.Add(pathPoints);
        }
    }

    private void OnDrawGizmos()
    {
        if(m_outputTriangles == null || !DrawTriangles)
            return;

        Color triangleColor = Color.black;

        for(int i = 0; i < m_outputTriangles.Count; ++i)
        {
            Debug.DrawLine(m_outputTriangles[i].p0, m_outputTriangles[i].p1, triangleColor);
            Debug.DrawLine(m_outputTriangles[i].p1, m_outputTriangles[i].p2, triangleColor);
            Debug.DrawLine(m_outputTriangles[i].p2, m_outputTriangles[i].p0, triangleColor);
        }
    }

#if UNITY_EDITOR

    [UnityEditor.CustomEditor(typeof(DelaunayTriangulationTester))]
    public class DelaunayTriangulationTesterInspector : UnityEditor.Editor
    {
        private int m_triangleToDraw = 0;
        private int m_pointToDraw = 0;
        private string m_triangleInfo;

        public override void OnInspectorGUI()
        {
            UnityEditor.EditorGUILayout.HelpBox("You can use different colliders of the scene as test cases, then press the Triangulate button. When the triangulation is generated, the mesh will be updated and the Draw Triangle and Draw point will be available for debugging.", UnityEditor.MessageType.Info);

            base.OnInspectorGUI();

            if(GUILayout.Button("Triangulate!"))
            {
                ((DelaunayTriangulationTester)target).RunTestPolygonColliders();
            }

            if (GUILayout.Button("Triangulate tilemap!"))
            {
                ((DelaunayTriangulationTester)target).RunTestTilemapColliders();
            }

            if (((DelaunayTriangulationTester)target).m_triangulation == null)
            {
                return;
            }

            UnityEditor.EditorGUILayout.LabelField("Triangles", UnityEditor.EditorStyles.boldLabel);

            m_triangleToDraw = UnityEditor.EditorGUILayout.IntSlider(new GUIContent("Triangle index:"), m_triangleToDraw, 0, ((DelaunayTriangulationTester)target).m_triangulation.TriangleSet.TriangleCount - 1);

            UnityEditor.EditorGUILayout.LabelField("Triangle vertices: " + m_triangleInfo);

            if (GUILayout.Button(new GUIContent("Draw triangle", "Draws a green triangle according to the selected index and a red cross at the position of the first vertex.")))
            {
                ((DelaunayTriangulationTester)target).DrawTriangle(m_triangleToDraw);

                unsafe
                {
                    DelaunayTriangle triangle = ((DelaunayTriangulationTester)target).m_triangulation.TriangleSet.GetTriangle(m_triangleToDraw);
                    m_triangleInfo = "(" + triangle.p[0] + ", " + triangle.p[1] + ", " + triangle.p[2] + ")";
                    ((DelaunayTriangulationTester)target).DrawPoint(triangle.p[0]);
                }
            }

            UnityEditor.EditorGUILayout.LabelField("Points", UnityEditor.EditorStyles.boldLabel);

            m_pointToDraw = UnityEditor.EditorGUILayout.IntSlider(new GUIContent("Point index:"), m_pointToDraw, 0, ((DelaunayTriangulationTester)target).m_triangulation.TriangleSet.Points.Count - 1);

            if (GUILayout.Button(new GUIContent("Draw point", "Draws a red cross at the position of the point.")))
            {
                ((DelaunayTriangulationTester)target).DrawPoint(m_pointToDraw);
            }
        }
    }

    private void DrawPoint(int pointIndex)
    {
        Vector2 point = m_triangulation.TriangleSet.GetPointByIndex(pointIndex);

        Debug.DrawRay(point + Vector2.down * 0.02f * 0.5f, Vector2.up * 0.02f, Color.red, 10.0f);
        Debug.DrawRay(point + Vector2.left * 0.02f * 0.5f, Vector2.right * 0.02f, Color.red, 10.0f);
    }

    private void DrawTriangle(int triangleIndex)
    {
        m_triangulation.TriangleSet.DrawTriangle(triangleIndex, Color.green);
    }

#endif

}
