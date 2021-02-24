
using Game.Utils.Math;
using System.Collections.Generic;
using UnityEngine;

public class DelaunayTriangulationTester : MonoBehaviour
{
    public PolygonCollider2D Collider;

    public List<Triangle2D> Triangles;

    public PolygonCollider2D Edges;

    public bool DrawTriangles;

    public MeshFilter Representation;

    public CompositeCollider2D TilemapFullRectCollider;

    public CompositeCollider2D TilemapCollider;

    protected DelaunayTriangulation m_triangulation;

    protected void RunTestPolygonColliders()
    {
        Debug.Log("Running Delaunay triangulation test...");

        List<Vector2> pointsToTriangulate = new List<Vector2>();
        ExtractPointsFromCollider(Collider, pointsToTriangulate);

        List<List<Vector2>> constrainedEdgePoints = new List<List<Vector2>>();

        if (Edges != null)
        {
            ExtractPointsFromCollider(Edges, constrainedEdgePoints);
        }

        Triangles = new List<Triangle2D>(pointsToTriangulate.Count / 3);
        m_triangulation = new DelaunayTriangulation();
        m_triangulation.Triangulation(pointsToTriangulate, Triangles, constrainedEdgePoints);

        Representation.mesh = CreateMeshFromTriangles(Triangles);

        Debug.Log("Test finished.");
    }

    protected void RunTestTilemapColliders()
    {
        Debug.Log("Running Delaunay triangulation test...");

        List<Vector2> pointsToTriangulate = new List<Vector2>();
        ExtractPointsFromCollider(TilemapFullRectCollider, pointsToTriangulate);

        List<List<Vector2>> constrainedEdgePoints = new List<List<Vector2>>();

        if (TilemapCollider != null)
        {
            ExtractPointsFromCollider(TilemapCollider, constrainedEdgePoints);
        }

        Triangles = new List<Triangle2D>(pointsToTriangulate.Count / 3);
        m_triangulation = new DelaunayTriangulation();
        m_triangulation.Triangulation(pointsToTriangulate, Triangles, constrainedEdgePoints);

        Representation.mesh = CreateMeshFromTriangles(Triangles);

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
            Collider.GetPath(i, pathPoints);
            outputPoints.AddRange(pathPoints);
        }
    }

    private void ExtractPointsFromCollider(PolygonCollider2D collider, List<List<Vector2>> outpuColliderPolygons)
    {
        int pathCount = collider.pathCount;

        for (int i = 0; i < pathCount; ++i)
        {
            List<Vector2> pathPoints = new List<Vector2>();
            Edges.GetPath(i, pathPoints);
            outpuColliderPolygons.Add(pathPoints);
        }
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

            if(GUILayout.Button("Run test PolygonColliders!"))
            {
                ((DelaunayTriangulationTester)target).RunTestPolygonColliders();
            }

            if (GUILayout.Button("Run test TilemapColliders!"))
            {
                ((DelaunayTriangulationTester)target).RunTestTilemapColliders();
            }

            if (((DelaunayTriangulationTester)target).m_triangulation == null)
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
