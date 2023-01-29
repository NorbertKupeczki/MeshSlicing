using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PolygonScript : MonoBehaviour
{
    [SerializeField] List<GameObject> points = new List<GameObject>();
    [SerializeField] Transform point;

    // Start is called before the first frame update
    void Start()
    {
        TriangulatePolygon();
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;

        for (int i = 0; i < points.Count; ++i)
        {
            if (i < points.Count - 1)
            {
                Gizmos.DrawLine(points[i].transform.position, points[i + 1].transform.position);
            }
            else
            {
                Gizmos.DrawLine(points[i].transform.position, points[0].transform.position);
            }

            Gizmos.DrawSphere(points[i].transform.position, 0.05f);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private bool IsConvex(Vector3 point, Vector3 lhs_vector, Vector3 rhs_vector)
    {
        Vector3 v1 = lhs_vector - point;
        Vector3 v2 = rhs_vector - point;

        // Instead of Vector3.up the final implementation needs to use the cutting plane normal.
        return 0 < Vector3.SignedAngle(v1, v2, Vector3.up);
    }

    private void DrawLine(Vector3 from, Vector3 to)
    {
        Debug.DrawLine(from, to, Color.green, 5.0f);
    }

    public void TriangulatePolygon()
    {
        List<int> pointIndices = new List<int>();
        List<Vector3> concavePoints = new List<Vector3>();
        int pointer = 0;
        int safety = 100;

        for (int i = 0; i < points.Count; i++)
        {
            pointIndices.Add(i);

            int p_lh = i == points.Count - 1 ? 0 : i + 1;
            int p_rh = i == 0 ? points.Count - 1 : i - 1;

            if (!IsConvex(points[i].transform.position, points[p_lh].transform.position, points[p_rh].transform.position))
            {
                concavePoints.Add(points[i].transform.position);
            }
        }

        while (pointIndices.Count > 3 && safety > 0)
        {
            for (int i = pointIndices[pointer]; pointer < pointIndices[pointIndices.Count - 1]; ++i)
            {
                if (!pointIndices.Contains(i))
                {
                    if (i > pointIndices[pointIndices.Count - 1])
                    {
                        break;
                    }
                    --safety;
                    continue;
                }

                int pointerIndex = pointIndices.IndexOf(i);
                int p_lh = i == pointIndices[pointIndices.Count - 1] ? pointIndices[0] : pointIndices[pointerIndex + 1];
                int p_rh = i == pointIndices[0] ? pointIndices[pointIndices.Count - 1] : pointIndices[pointerIndex - 1];

                if (IsConvex(points[i].transform.position, points[p_lh].transform.position, points[p_rh].transform.position) &&
                    !CheckConcavePoints(points[i].transform.position, points[p_lh].transform.position, points[p_rh].transform.position, concavePoints))
                {
                    DrawLine(points[p_lh].transform.position, points[p_rh].transform.position);
                    pointer = pointIndices.IndexOf(i) >= pointIndices.Count - 2 ? pointIndices[0]: pointIndices.IndexOf(i) + 1;
                    pointIndices.Remove(i);
                    break;
                }
            }

            --safety;
            if (safety < 1)
            {
                Debug.Log("Safety threshold reached");
            }
        }
    }

    private bool SameSide(Vector3 tri1, Vector3 tri2, Vector3 tri3, Vector3 point)
    {
        Vector3 cp1 = Vector3.Cross(tri2 - tri1, point - tri1);
        Vector3 cp2 = Vector3.Cross(tri2 - tri1, tri3 - tri1);
        return Vector3.Dot(cp1, cp2) >= 0;
    }

    /// <summary>
    /// Returns True if the point is within the triengle defined by three points (triA, triB, triC) of the triangle.
    /// </summary>
    /// <param name="point"></param>
    /// <param name="triA"></param>
    /// <param name="triB"></param>
    /// <param name="triC"></param>
    /// <returns>True if the point is within the triengle</returns>
    private bool PointInTriangle(Vector3 point, Vector3 triA, Vector3 triB, Vector3 triC)
    {
        return SameSide(triA, triB, triC, point) && SameSide(triB, triC, triA, point) && SameSide(triC, triA, triB, point);
    }

    private bool CheckConcavePoints(Vector3 a, Vector3 b, Vector3 c, List<Vector3> concavePoints)
    {
        foreach (Vector3 point in concavePoints)
        {
            if(a == point ||
               b == point ||
               c == point)
            {
                continue;
            }

            if(PointInTriangle(point, a, b, c))
            {
                return true;
            }            
        }
        return false;
    }
}
