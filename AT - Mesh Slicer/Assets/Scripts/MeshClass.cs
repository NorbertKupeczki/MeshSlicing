using System.Collections;
using System.Collections.Generic;
using UnityEditorInternal;
using UnityEngine;

public class MeshClass
{
    private List<Vector3> _Verticies = new();
    private List<Vector3> _Normals = new();
    private List<List<int>> _Triangles = new();
    private List<Vector2> _UVs = new();
    private List<Edge> _Edges = new();
    public Vector3[] Vertices;
    public Vector3[] Normals;
    public int[][] Triangles;
    public Vector2[] UV;
    public GameObject NewGameObject;
    public Bounds Bounds = new();

    public MeshClass()
    {

    }

    public class Edge
    {
        public Edge(Vector3 _pointA, Vector3 _pointB)
        {
            PointA = _pointA;
            PointB = _pointB;
            Processed = false;
        }

        public Vector3 PointA;
        public Vector3 PointB;
        public bool Processed;

        public bool CheckPoints(Vector3 check)
        {
            if (check == PointA || check == PointB)
            {
                return true;
            }
            return false;
        }

        public Vector3 GetOtherPoint(Vector3 point)
        {
            if (point == PointA)
            {
                return PointB;
            }
            return PointA;
        }
    }
    
    public class EdgePoint
    {
        public EdgePoint(Vector3 position, Vector3 normal)
        {
            Position = position;
            Normal = normal;
        }
        public Vector3 Position { get; private set; }
        public Vector3 Normal { get; private set; }
        public EdgePoint LeftNeighbour { get; private set; }
        public EdgePoint RightNeighbour { get; private set; }
        public bool IsConvex { get; private set; }

        public void SetLeftNeighbour(EdgePoint neighbour)
        {
            LeftNeighbour = neighbour;
        }

        public void SetRightNeighbour(EdgePoint neighbour)
        {
            RightNeighbour = neighbour;
        }

        public void SetConvex(bool value)
        {
            IsConvex = value;
        }

        public void UpdateConvex()
        {
            IsConvex = MeshClass.IsConvex(Position, LeftNeighbour.Position, RightNeighbour.Position, Normal);
        }
    }

    public void AddTriangle(int submesh,
                            Vector3 vert1,
                            Vector3 vert2,
                            Vector3 vert3,
                            Vector3 normal1,
                            Vector3 normal2,
                            Vector3 normal3,
                            Vector2 uv1,
                            Vector2 uv2,
                            Vector2 uv3)
    {
        if (_Triangles.Count - 1 < submesh)
            _Triangles.Add(new List<int>());

        _Triangles[submesh].Add(_Verticies.Count);
        _Verticies.Add(vert1);
        _Triangles[submesh].Add(_Verticies.Count);
        _Verticies.Add(vert2);
        _Triangles[submesh].Add(_Verticies.Count);
        _Verticies.Add(vert3);
        _Normals.Add(normal1);
        _Normals.Add(normal2);
        _Normals.Add(normal3);
        _UVs.Add(uv1);
        _UVs.Add(uv2);
        _UVs.Add(uv3);

        Bounds.min = Vector3.Min(Bounds.min, vert1);
        Bounds.min = Vector3.Min(Bounds.min, vert2);
        Bounds.min = Vector3.Min(Bounds.min, vert3);
        Bounds.max = Vector3.Min(Bounds.max, vert1);
        Bounds.max = Vector3.Min(Bounds.max, vert2);
        Bounds.max = Vector3.Min(Bounds.max, vert3);
    }

    public void AddEdge(Vector3 pointA, Vector3 pointB)
    {
        _Edges.Add(new Edge(pointA, pointB));
    }

    public void FillArrays()
    {
        Vertices = _Verticies.ToArray();
        Normals = _Normals.ToArray();
        UV = _UVs.ToArray();
        Triangles = new int[_Triangles.Count][];
        for (var i = 0; i < _Triangles.Count; i++)
            Triangles[i] = _Triangles[i].ToArray();
    }

    public void CreateNewGameObject(MeshDestroy original)
    {
        NewGameObject = new GameObject(original.name);
        NewGameObject.transform.position = original.transform.position;
        NewGameObject.transform.rotation = original.transform.rotation;
        NewGameObject.transform.localScale = original.transform.localScale;

        Mesh newMesh = new Mesh
        {
            name = original.GetComponent<MeshFilter>().mesh.name,
            vertices = Vertices,
            normals = Normals,
            uv = UV
        };
        for (var i = 0; i < Triangles.Length; i++)
            newMesh.SetTriangles(Triangles[i], i, true);
        Bounds = newMesh.bounds;

        MeshRenderer renderer = NewGameObject.AddComponent<MeshRenderer>();
        renderer.materials = original.GetComponent<MeshRenderer>().materials;

        MeshFilter filter = NewGameObject.AddComponent<MeshFilter>();
        filter.mesh = newMesh;

        MeshCollider collider = NewGameObject.AddComponent<MeshCollider>();
        collider.convex = true;
        collider.material = original.GetComponent<Collider>().material;

        MeshDestroy meshDestroy = NewGameObject.AddComponent<MeshDestroy>();
        meshDestroy.SetObjectVolume(original.CalculateVolumeOfMesh(newMesh));
        meshDestroy.SetOriginalVolume(original.GetOriginalVolume());
        meshDestroy.SetNumberOfCuts(original.GetNumberOfCuts());
        meshDestroy.SetCentreOfExplosion(original.GetCentreOfExplosion());
        meshDestroy.addParticles = original.addParticles;
        meshDestroy.smokeParticles = original.smokeParticles;

        Rigidbody rigidbody = NewGameObject.AddComponent<Rigidbody>();
        rigidbody.mass = CalculateProportionalMass(original, meshDestroy.GetObjectVolume());
        rigidbody.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rigidbody.drag = original.gameObject.GetComponent<Rigidbody>().drag;
        rigidbody.angularDrag = original.gameObject.GetComponent<Rigidbody>().angularDrag;

        if (original.IsAddingParticles())
        {
            ParticleSystem particleSystem = NewGameObject.AddComponent<ParticleSystem>();
            ComponentUtility.CopyComponent(original.smokeParticles);
            ComponentUtility.PasteComponentValues(particleSystem);
        }

    }

    public void CreateNewPlaneMesh()
    {
        NewGameObject = new GameObject();

        Mesh newMesh = new Mesh
        {
            name = "TestMesh",
            vertices = Vertices,
            normals = Normals,
            uv = UV
        };
        for (var i = 0; i < Triangles.Length; i++)
            newMesh.SetTriangles(Triangles[i], i, true);
        Bounds = newMesh.bounds;

        NewGameObject.AddComponent<MeshRenderer>();

        MeshFilter filter = NewGameObject.AddComponent<MeshFilter>();
        filter.mesh = newMesh;
    }

    public float CalculateProportionalMass(MeshDestroy originalObject, float newVolume)
    {
        float originalMass = originalObject.gameObject.GetComponent<Rigidbody>().mass;
        float originalVolume = originalObject.GetObjectVolume();
        return newVolume * originalMass / originalVolume;
    }

    private List<Vector3> BuildLoop(Vector3 normal)
    {
        bool loopComplete = false;
        int safety = _Edges.Count;
        List<Vector3> resultLoop = new()
        {
            _Edges[0].PointA,
            _Edges[0].PointB
        };
        _Edges[0].Processed = true;
        int processedPoints = 2;

        while (!loopComplete)
        {
            for (int i = 1; i < _Edges.Count; ++i)
            {
                Vector3 lastLoopPoint = resultLoop[^1];

                if (_Edges[i].CheckPoints(lastLoopPoint) && !_Edges[i].Processed)
                {
                    Vector3 otherPoint = _Edges[i].GetOtherPoint(lastLoopPoint);
                    if (otherPoint == resultLoop[0])
                    {
                        loopComplete = true;
                    }
                    else
                    {
                        resultLoop.Add(_Edges[i].GetOtherPoint(lastLoopPoint));
                        _Edges[i].Processed = true;
                        ++processedPoints;
                    }
                }

                if (_Edges.Count == processedPoints)
                {
                    loopComplete = true;
                    break;
                }
            }

            if (safety < 1)
            {
                loopComplete = true;
                Debug.LogWarning("Safety limit at generating the cut edge loop is reached!");
            }

            --safety;
        }

        AlignLoopWithNormal(resultLoop, normal);
        return resultLoop;
    }

    private void AlignLoopWithNormal(List<Vector3> loop, Vector3 normal)
    {
        float polygonInternalAngle = (loop.Count - 2) * 180.0f;
        float internalAngle = 0.0f;

        for (int i = 0; i < loop.Count; ++i)
        {
            Vector3 leftVector;
            Vector3 rightVector;
            if (i == 0)
            {
                leftVector = loop[1] - loop[i];
                rightVector = loop[^1] - loop[i];
                
            }
            else if (i == loop.Count - 1)
            {
                leftVector = loop[0] - loop[i];
                rightVector = loop[^2] - loop[i];
            }
            else
            {
                leftVector = loop[i+1] - loop[i];
                rightVector = loop[i-1] - loop[i];
            }
            float angle = GetSignedAngle(leftVector.normalized, rightVector.normalized, normal.normalized);
            
            if(angle == 180.0f || angle == -180.0f)
            {
                polygonInternalAngle -= 180.0f;
            }
            else if (angle < 0)
            {
                internalAngle += (360.0f + angle);
            }
            else
            {
                internalAngle += angle;
            }
        }
        bool insideOut = (int)internalAngle > (int)polygonInternalAngle;
        if (insideOut)
        {
            Debug.Log(internalAngle + " : " + polygonInternalAngle);
            loop.Reverse();
        }
    }

    public static bool IsConvex(Vector3 point, Vector3 lhs_vector, Vector3 rhs_vector, Vector3 planeNormal)
    {
        Vector3 v1 = lhs_vector - point;
        Vector3 v2 = rhs_vector - point;
        bool isConvex = 0 < GetSignedAngle(v1, v2, planeNormal);
        return isConvex;
    }

    private static float GetSignedAngle(Vector3 leftVector, Vector3 rightVector, Vector3 normalVector)
    {
        return Vector3.SignedAngle(leftVector, rightVector, normalVector);
    }

    private void DrawLine(Vector3 from, Vector3 to)
    {
        Debug.DrawLine(from, to, Color.magenta, 5.0f);
    }

    public void TriangulatePolygon(Plane plane, bool above)
    {
        Vector3 normal = -plane.normal;
        if (!above)
        {
            normal *= -1;
            //Debug.DrawLine(Vector3.zero, normal, Color.yellow, 15.0f);
        }
        else
        {
            //Debug.DrawLine(Vector3.zero, normal, Color.blue, 15.0f);
        }
        List<Vector3> loop = BuildLoop(normal);

        int subMeshID = 0;

        List<int> pointIndices = new();
        List<Vector3> concavePoints = new();
        int pointer = 0;
        int safety = 300000;

        for (int i = 0; i < loop.Count; i++)
        {
            pointIndices.Add(i);

            int p_lh = i == loop.Count - 1 ? 0 : i + 1;
            int p_rh = i == 0 ? loop.Count - 1 : i - 1;

            if (!IsConvex(loop[i], loop[p_lh], loop[p_rh], normal))
            {
                concavePoints.Add(loop[i]);
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
                    continue;
                }

                Vector3[] trianglePoints = GetTrianglePoints(i, pointIndices, loop);

                if (IsConvex(trianglePoints[0], trianglePoints[1], trianglePoints[2], normal) &&
                    !CheckConcavePoints(trianglePoints[0], trianglePoints[1], trianglePoints[2], concavePoints))
                {
                    AddTriangle(subMeshID,
                                trianglePoints[0], trianglePoints[1], trianglePoints[2],
                                normal, normal, normal,
                                trianglePoints[0], trianglePoints[1], trianglePoints[2]);
                    //DrawLine(trianglePoints[1], trianglePoints[2]);
                    pointer = pointIndices.IndexOf(i) >= pointIndices.Count - 2 ? pointIndices[0] : pointIndices.IndexOf(i) + 1;
                    pointIndices.Remove(i);
                    break;
                }
            }
            --safety;
            if (safety < 1)
            {
                Debug.LogWarning("Safety limit at cut surface triangulation is reached!");
            }
        }

        Vector3[] lastPoints = GetTrianglePoints(pointIndices[0], pointIndices, loop);
        AddTriangle(subMeshID, lastPoints[0], lastPoints[1], lastPoints[2], normal, normal, normal, lastPoints[0], lastPoints[1], lastPoints[2]);
    }

    /// <summary>
    /// Gets an array with the coordinates of the triangle starting from point A, followed by its left hand and right hand points.
    /// </summary>
    /// <param name="pointAIndex"></param>
    /// <param name="_pointIndices"></param>
    /// <param name="_points"></param>
    /// <returns>Vector3</returns>
    private Vector3[] GetTrianglePoints(int pointAIndex, List<int> _pointIndices, List<Vector3> _points)
    {
        int pointerIndex = _pointIndices.IndexOf(pointAIndex);
        int p_lh = pointAIndex == _pointIndices[^1] ? _pointIndices[0] : _pointIndices[pointerIndex + 1];
        int p_rh = pointAIndex == _pointIndices[0] ? _pointIndices[^1] : _pointIndices[pointerIndex - 1];

        return new Vector3[] { _points[pointAIndex],
                               _points[p_lh],
                               _points[p_rh]};
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
            if (a == point ||
               b == point ||
               c == point)
            {
                continue;
            }

            if (PointInTriangle(point, a, b, c))
            {
                return true;
            }
        }
        return false;
    }
}
