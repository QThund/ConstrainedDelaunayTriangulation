
using Game.Utils.Algebra;
using System.Collections.Generic;
using UnityEngine;

namespace Game.Utils.Geometry
{
    public static class GeometryUtils
    {
		public static bool IsPointToTheRightOfEdge(Vector2 edgeEndpointA, Vector2 edgeEndpointB, Vector2 point)
        {
			Vector2 aToB = edgeEndpointB - edgeEndpointA;
			Vector2 aToP = point - edgeEndpointA;
			Vector3 ab_x_p = Vector3.Cross(aToB, aToP);
			return ab_x_p.z < -0.0001f; // Note: Due to extremely small negative values were causing wrong results, a tolerance is used
		}

		public static bool IsPointInsideTriangle(Vector2 triangleP0, Vector2 triangleP1, Vector2 triangleP2, Vector2 pointToCheck)
		{ 
			Vector3 ab_x_p = Vector3.Cross(triangleP1 - triangleP0, pointToCheck);
			Vector3 bc_x_p = Vector3.Cross(triangleP2 - triangleP1, pointToCheck);
			Vector3 ca_x_p = Vector3.Cross(triangleP0 - triangleP2, pointToCheck);

			return ab_x_p.z == bc_x_p.z && ab_x_p.z == ca_x_p.z;
		}

		// https://gamedev.stackexchange.com/questions/71328/how-can-i-add-and-subtract-convex-polygons
		public static bool IsPointInsideCircumcircle(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 pointToChecl)
		{
			// This first part will simplify how we calculate the determinant
			float a = p0.x - pointToChecl.x;
			float d = p1.x - pointToChecl.x;
			float g = p2.x - pointToChecl.x;

			float b = p0.y - pointToChecl.y;
			float e = p1.y - pointToChecl.y;
			float h = p2.y - pointToChecl.y;

			float c = a * a + b * b;
			float f = d * d + e * e;
			float i = g * g + h * h;

			float determinant = (a * e * i) + (b * f * g) + (c * d * h) - (g * e * c) - (h * f * a) - (i * d * b);

			return determinant >= 0; // zero means on the perimeter
		}

		// https://stackoverflow.com/questions/4543506/algorithm-for-intersection-of-2-lines
		public static bool InsersectionBetweenLines(Vector2 endpointA1, Vector2 endpointB1, Vector2 endpointA2, Vector2 endpointB2, out Vector2 intersectionPoint)
        {
			intersectionPoint = new Vector2(float.MaxValue, float.MaxValue);

			bool isLine1Vertical = endpointB1.x == endpointA1.x;
			bool isLine2Vertical = endpointB2.x == endpointA2.x;

			float x = float.MaxValue;
			float y = float.MaxValue;

			if (isLine1Vertical && !isLine2Vertical)
            {
				// First it calculates the standard form (Ax + By = C)
				float m2 = (endpointB2.y - endpointA2.y) / (endpointB2.x - endpointA2.x);

				float A2 = m2;
				float C2 = endpointA2.x * m2 - endpointA2.y;
				
				x = endpointA1.x;
				y = m2 * endpointA1.x - C2;
			}
			else if(isLine2Vertical && !isLine1Vertical)
            {
				// First it calculates the standard form (Ax + By = C)
				float m1 = (endpointB1.y - endpointA1.y) / (endpointB1.x - endpointA1.x);

				float A1 = m1;
				float C1 = endpointA1.x * m1 - endpointA1.y;

				x = endpointA2.x;
				y = m1 * endpointA2.x - C1;
			}
			else if(!isLine1Vertical && !isLine2Vertical)
            {
				// First it calculates the standard form of both lines (Ax + By = C)
				float m1 = (endpointB1.y - endpointA1.y) / (endpointB1.x - endpointA1.x);

				float A1 = m1;
				float B1 = -1.0f;
				float C1 = endpointA1.x * m1 - endpointA1.y;

				float m2 = (endpointB2.y - endpointA2.y) / (endpointB2.x - endpointA2.x);

				float A2 = m2;
				float B2 = -1.0f;
				float C2 = endpointA2.x * m2 - endpointA2.y;

				float determinant = A1 * B2 - A2 * B1;

				if (determinant == 0)
				{
					// Lines do not intersect
					return false;
				}

				x = (B2 * C1 - B1 * C2) / determinant;
				y = (A1 * C2 - A2 * C1) / determinant;
			}
			// else : no intersection

			bool result = false;

			//Debug.DrawLine(endpointA1, new Vector2(x, y), Color.yellow, 10.0f);
			//Debug.DrawLine(endpointA2, new Vector2(x, y), Color.yellow, 10.0f);

			// Checks whether the point is in the segment determined by the endpoints of both lines
			if (x <= Mathf.Max(endpointA1.x, endpointB1.x) && x >= Mathf.Min(endpointA1.x, endpointB1.x) &&
				y <= Mathf.Max(endpointA1.y, endpointB1.y) && y >= Mathf.Min(endpointA1.y, endpointB1.y) &&
				x <= Mathf.Max(endpointA2.x, endpointB2.x) && x >= Mathf.Min(endpointA2.x, endpointB2.x) &&
				y <= Mathf.Max(endpointA2.y, endpointB2.y) && y >= Mathf.Min(endpointA2.y, endpointB2.y))
			{
				intersectionPoint.x = x;
				intersectionPoint.y = y;
				result = true;
			}

			return result;
		}

		public static bool IsTriangleVerticesCW(Vector2 point0, Vector2 point1, Vector2 point2)
        {
			return AlgebraUtils.CalculateMatrix3x3Determinant(point0.x, point0.y, 1.0f,
															  point1.x, point1.y, 1.0f,
															  point2.x, point2.y, 1.0f) < 0.0f;
		}

		//Is a quadrilateral convex? Assume no 3 points are colinear and the shape doesnt look like an hourglass
		public static bool IsQuadrilateralConvex(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
		{
			bool isConvex = false;

			bool abc = IsTriangleVerticesCW(a, b, c);
			bool abd = IsTriangleVerticesCW(a, b, d);
			bool bcd = IsTriangleVerticesCW(b, c, d);
			bool cad = IsTriangleVerticesCW(c, a, d);

			if (abc && abd && bcd & !cad)
			{
				isConvex = true;
			}
			else if (abc && abd && !bcd & cad)
			{
				isConvex = true;
			}
			else if (abc && !abd && bcd & cad)
			{
				isConvex = true;
			}
			//The opposite sign, which makes everything inverted
			else if (!abc && !abd && !bcd & cad)
			{
				isConvex = true;
			}
			else if (!abc && !abd && bcd & !cad)
			{
				isConvex = true;
			}
			else if (!abc && abd && !bcd & !cad)
			{
				isConvex = true;
			}


			return isConvex;
		}
	}
}

