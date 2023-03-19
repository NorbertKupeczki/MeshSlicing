using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PolygonScript : MonoBehaviour
{
    [SerializeField] List<Transform> points = new List<Transform>();
    [SerializeField] List<MeshClass.Edge> edges = new List<MeshClass.Edge>();
    [SerializeField] Transform parent;

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < points.Count; i++)
        {
            if (i < points.Count - 1)
            {
                edges.Add(new MeshClass.Edge(points[i], points[i+1]));
            }
            else
            {
                edges.Add(new MeshClass.Edge(points[i], points[0]));
            }
        }

        TriangulatePolygon(parent.up, edges);
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;

        for (int i = 0; i < points.Count; ++i)
        {
            if (i < points.Count - 1)
            {
                Gizmos.DrawLine(points[i].position, points[i + 1].position);
            }
            else
            {
                Gizmos.DrawLine(points[i].position, points[0].position);
            }

            Gizmos.DrawSphere(points[i].position, 0.05f);
        }
    }

    private List<Transform> BuildLoop(List<MeshClass.Edge> edgeList)
    {
        bool loopComplete = false;
        int safety = 10;
        List<Transform> resultLoop = new List<Transform>();

        resultLoop.Add(edgeList[0].PointA);
        resultLoop.Add(edgeList[0].PointB);

        while(!loopComplete)
        {
            for (int i = 1; i < edgeList.Count; ++i)
            {
                Transform lastLoopPoint = resultLoop[^1];

                if (edgeList[i].CheckPoints(lastLoopPoint))
                {
                    Transform otherPoint = edgeList[i].GetOtherPoint(lastLoopPoint);
                    if (otherPoint == resultLoop[0])
                    {
                        loopComplete = true;
                    }
                    else
                    {
                        resultLoop.Add(edgeList[i].GetOtherPoint(lastLoopPoint));
                    }
                }
            }

            if(safety < 1)
            {
                loopComplete = true;
            }

            --safety;
        }
        return resultLoop;
    }

    private bool IsConvex(Vector3 point, Vector3 lhs_vector, Vector3 rhs_vector, Vector3 planeNormal)
    {
        Vector3 v1 = lhs_vector - point;
        Vector3 v2 = rhs_vector - point;

        // Instead of Vector3.up the final implementation needs to use the cutting plane normal.
        return 0 < Vector3.SignedAngle(v1, v2, planeNormal);
    }

    private void DrawLine(Vector3 from, Vector3 to)
    {
        Debug.DrawLine(from, to, Color.green, 5.0f);
    }
    
    public void TriangulatePolygon(Vector3 planeNormal, List<MeshClass.Edge> edgeList)
    {
        List<Transform> loop = BuildLoop(edgeList);

        MeshClass newMesh = new() { };
        int subMeshID = 0;
        Vector3 normal = planeNormal;

        List<int> pointIndices = new();
        List<Vector3> concavePoints = new();
        int pointer = 0;
        int safety = 100;

        for (int i = 0; i < loop.Count; i++)
        {
            pointIndices.Add(i);

            int p_lh = i == loop.Count - 1 ? 0 : i + 1;
            int p_rh = i == 0 ? loop.Count - 1 : i - 1;

            if (!IsConvex(loop[i].position, loop[p_lh].position, loop[p_rh].position, normal))
            {
                concavePoints.Add(loop[i].position);
            }
        }

        while (pointIndices.Count > 3 && safety > 0)
        {
            for (int i = pointIndices[pointer]; pointer < pointIndices[^1]; ++i)
            {
                if (!pointIndices.Contains(i))
                {
                    if (i > pointIndices[^1])
                    {
                        break;
                    }
                    --safety;
                    continue;
                }

                Vector3[] trianglePoints = GetTrianglePoints(i, pointIndices, loop);

                if (IsConvex(trianglePoints[0], trianglePoints[1], trianglePoints[2], normal) &&
                    !CheckConcavePoints(trianglePoints[0], trianglePoints[1], trianglePoints[2], concavePoints))
                {
                    newMesh.AddTriangle(subMeshID, trianglePoints[0], trianglePoints[1], trianglePoints[2], normal, normal, normal, trianglePoints[0], trianglePoints[1], trianglePoints[2]);

                    DrawLine(trianglePoints[1], trianglePoints[2]);
                    pointer = pointIndices.IndexOf(i) >= pointIndices.Count - 2 ? pointIndices[0]: pointIndices.IndexOf(i) + 1;
                    pointIndices.Remove(i);
                    break;
                }
            }

            --safety;
        }

        Vector3[] lastPoints = GetTrianglePoints(pointIndices[0], pointIndices, loop);
        newMesh.AddTriangle(0, lastPoints[0], lastPoints[1], lastPoints[2], normal, normal, normal, lastPoints[0], lastPoints[1], lastPoints[2]);

        newMesh.FillArrays();
        newMesh.CreateNewCutMesh();
    }

    /// <summary>
    /// Gets an array with the coordinates of the triangle starting from point A, followed by its left hand and right hand points.
    /// </summary>
    /// <param name="pointAIndex"></param>
    /// <param name="_pointIndices"></param>
    /// <param name="_points"></param>
    /// <returns>Vector3</returns>
    private Vector3[] GetTrianglePoints(int pointAIndex, List<int> _pointIndices, List<Transform> _points)
    {
        int pointerIndex = _pointIndices.IndexOf(pointAIndex);
        int p_lh = pointAIndex == _pointIndices[^1] ? _pointIndices[0] : _pointIndices[pointerIndex + 1];
        int p_rh = pointAIndex == _pointIndices[0] ? _pointIndices[^1] : _pointIndices[pointerIndex - 1];

        return new Vector3[] { _points[pointAIndex].position,
                               _points[p_lh].position,
                               _points[p_rh].position };
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
