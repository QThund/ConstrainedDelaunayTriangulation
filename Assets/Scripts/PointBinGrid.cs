
using System.Collections.Generic;
using UnityEngine;

namespace Game.Utils.Geometry
{
    public class PointBinGrid
    {
        public List<Vector2>[] cells;

        private Vector2 m_cellSize;
        private Vector2 m_gridSize; // Xmax, Ymax
        private int m_cellsPerSide; // n

        public PointBinGrid(int cellsPerSide, Vector2 gridSize)
        {
            cells = new List<Vector2>[cellsPerSide * cellsPerSide];
            m_cellSize = gridSize / cellsPerSide;
            m_gridSize = gridSize;
            m_cellsPerSide = cellsPerSide;
        }

        public void AddPoint(Vector2 newPoint)
        {
            int rowIndex = (int)(0.99f * m_cellsPerSide * newPoint.y / m_gridSize.y); // i
            int columnIndex = (int)(0.99f * m_cellsPerSide * newPoint.x / m_gridSize.x); // j

            int binIndex = 0; // b

            if (rowIndex % 2 == 0)
            {
                binIndex = rowIndex * m_cellsPerSide + columnIndex + 1;
            }
            else
            {
                binIndex = (rowIndex + 1) * m_cellsPerSide - columnIndex;
            }

            binIndex--; // zero-based index

            if (cells[binIndex] == null)
            {
                cells[binIndex] = new List<Vector2>();
            }

            cells[binIndex].Add(newPoint);

            //DrawPointAddition(newPoint, columnIndex, rowIndex);
        }

        public void DrawGrid(Color color, float duration)
        {
            for(int i = 0; i < m_cellsPerSide; ++i)
            {
                Debug.DrawRay(new Vector3(0.0f, i * m_cellSize.y, 1.0f), Vector2.right * m_gridSize.x, color, duration);

                for (int j = 0; j < m_cellsPerSide; ++j)
                {
                    Debug.DrawRay(new Vector3(j * m_cellSize.x, 0.0f, 1.0f), Vector2.up * m_gridSize.y, color, duration);
                }
            }

            Debug.DrawRay(new Vector3(0.0f, m_cellsPerSide * m_cellSize.y, 1.0f), Vector2.right * m_gridSize.x, color, duration);
            Debug.DrawRay(new Vector3(m_cellsPerSide * m_cellSize.x, 0.0f, 1.0f), Vector2.up * m_gridSize.y, color, duration);
        }

        protected void DrawPointAddition(Vector2 point, int columnIndex, int rowIndex)
        {
            Vector2 cellBottomLeftCorner = new Vector2(columnIndex * m_cellSize.x, rowIndex * m_cellSize.y);
            Debug.DrawLine(point, cellBottomLeftCorner + m_cellSize * 0.5f, Color.cyan, 5.0f);
        }
    }
}

