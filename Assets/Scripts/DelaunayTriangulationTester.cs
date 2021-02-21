
using Game.Utils.Geometry;
using System;
using System.Collections.Generic;
using UnityEngine;

public class DelaunayTriangulationTester : MonoBehaviour
{
    public PolygonCollider2D Collider;

    public List<Triangle2D> Triangles;

    public PolygonCollider2D Edges;

    public bool DrawTriangles;

    public MeshFilter Representation;

    protected DelaunayTriangulation m_triangulation;

    protected void RunTest()
    {
        Debug.Log("Running Delaunay triangulation test...");

        int pathCount = Collider.pathCount;

        List<Vector2> pointsToTriangulate = new List<Vector2>();
        List<Vector2> pathPoints = new List<Vector2>();


        for (int i = 0; i < pathCount; ++i)
        {
            pathPoints.Clear();
            Collider.GetPath(i, pathPoints);
            pointsToTriangulate.AddRange(pathPoints);
        }

        List<List<Vector2>> edgePoints = new List<List<Vector2>>();

        if (Edges != null)
        {
            pathCount = Edges.pathCount;

            for (int i = 0; i < pathCount; ++i)
            {
                pathPoints = new List<Vector2>(pathPoints);
                Edges.GetPath(i, pathPoints);
                edgePoints.Add(pathPoints);
            }
        }
        
        Triangles = new List<Triangle2D>(pointsToTriangulate.Count / 3);
        m_triangulation = new DelaunayTriangulation();
        m_triangulation.Triangulation(pointsToTriangulate, Triangles, edgePoints);

        List<Vector3> vertices = new List<Vector3>(Triangles.Count * 3);
        List<int> indices = new List<int>(Triangles.Count * 3);

        for(int i = 0; i < Triangles.Count; ++i)
        {
            vertices.Add(Triangles[i].p0);
            vertices.Add(Triangles[i].p1);
            vertices.Add(Triangles[i].p2);
            indices.Add(i * 3 + 2); // Changes order
            indices.Add(i * 3 + 1);
            indices.Add(i * 3);
        }

        Mesh mesh = new Mesh();
        mesh.subMeshCount = 1;
        mesh.SetVertices(vertices);
        mesh.SetIndices(indices, MeshTopology.Triangles, 0);
        Representation.mesh = mesh;

        Debug.Log("Test finished.");
    }

    private void OnDrawGizmos()
    {
        if(Triangles == null || !DrawTriangles)
            return;

        Color triangleColor = Color.black;

        for(int i = 0; i < Triangles.Count; ++i)
        {
            Debug.DrawLine(Triangles[i].p0, Triangles[i].p1, triangleColor);
            Debug.DrawLine(Triangles[i].p1, Triangles[i].p2, triangleColor);
            Debug.DrawLine(Triangles[i].p2, Triangles[i].p0, triangleColor);
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
            base.OnInspectorGUI();

            if(GUILayout.Button("Run test!"))
            {
                ((DelaunayTriangulationTester)target).RunTest();
            }

            if(((DelaunayTriangulationTester)target).m_triangulation == null)
            {
                return;
            }

            m_triangleToDraw = UnityEditor.EditorGUILayout.IntSlider(new GUIContent("Triangle:"), m_triangleToDraw, 0, ((DelaunayTriangulationTester)target).m_triangulation.TriangleStorage.TriangleCount - 1);

            UnityEditor.EditorGUILayout.LabelField(m_triangleInfo);

            if (GUILayout.Button("Draw triangle"))
            {
                ((DelaunayTriangulationTester)target).DrawTriangle(m_triangleToDraw);

                unsafe
                {
                    DelaunayTriangle triangle = ((DelaunayTriangulationTester)target).m_triangulation.TriangleStorage.GetTriangle(m_triangleToDraw);
                    m_triangleInfo = "(" + triangle.p[0] + ", " + triangle.p[1] + ", " + triangle.p[2] + ")";
                    ((DelaunayTriangulationTester)target).DrawPoint(triangle.p[0]);
                }
            }

            m_pointToDraw = UnityEditor.EditorGUILayout.IntSlider(new GUIContent("Point:"), m_pointToDraw, 0, ((DelaunayTriangulationTester)target).m_triangulation.TriangleStorage.Points.Count - 1);

            if (GUILayout.Button("Draw vertex"))
            {
                ((DelaunayTriangulationTester)target).DrawPoint(m_pointToDraw);
            }
        }
    }

    private void DrawPoint(int pointIndex)
    {
        Vector2 point = m_triangulation.TriangleStorage.GetPointByIndex(pointIndex);

        Debug.DrawRay(point + Vector2.down * 0.02f * 0.5f, Vector2.up * 0.02f, Color.red, 10.0f);
        Debug.DrawRay(point + Vector2.left * 0.02f * 0.5f, Vector2.right * 0.02f, Color.red, 10.0f);
    }

    private void DrawTriangle(int triangleIndex)
    {
        m_triangulation.TriangleStorage.DrawTriangle(triangleIndex, Color.green);
    }

#endif

}
