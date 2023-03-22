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
        public EdgePoint(Vector3 position, Vector3 normal, int iD)
        {
            Position = position;
            Normal = normal;
            ID = iD;
            IsConvex = false;
            OnEdgePoint = false;
        }
        public int ID { get; private set; }
        public Vector3 Position { get; private set; }
        public Vector3 Normal { get; private set; }
        public EdgePoint LeftNeighbour { get; private set; }
        public EdgePoint RightNeighbour { get; private set; }
        public bool IsConvex { get; private set; }
        public bool OnEdgePoint { get; private set; }

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
            if(LeftNeighbour != null && RightNeighbour != null)
            {
                IsConvex = MeshClass.IsConvex(Position, LeftNeighbour.Position, RightNeighbour.Position, Normal);
            }

            if(Mathf.Abs(MeshClass.GetSignedAngle(LeftNeighbour.Position - Position, RightNeighbour.Position - Position, Normal)) == 180.0f)
            {
                OnEdgePoint = true;
            }
            else
            {
                OnEdgePoint = false;
            }
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
        float signedAngle = GetSignedAngle(v1, v2, planeNormal);          
        return (0 < signedAngle || Mathf.Abs(signedAngle) == 180.0f);
    }

    public static float GetSignedAngle(Vector3 leftVector, Vector3 rightVector, Vector3 normalVector)
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
        }

        List<Vector3> loop = BuildLoop(normal);
        int subMeshID = 0;

        List<EdgePoint> points = new();
        for (int i = 0; i < loop.Count; ++i)
        {            
            points.Add(new EdgePoint(loop[i], normal, i));

            if (i == loop.Count - 1)
            {
                points[i].SetRightNeighbour(points[i - 1]);
                points[i - 1].SetLeftNeighbour(points[i]);
                points[i].SetLeftNeighbour(points[0]);
                points[0].SetRightNeighbour(points[i]);
            }
            else if (i != 0)
            {
                points[i].SetRightNeighbour(points[i - 1]);
                points[i - 1].SetLeftNeighbour(points[i]);
            }
        }

        List<EdgePoint> concavePoints = new();
        for (int i = 0; i < points.Count; ++i)
        {
            points[i].UpdateConvex();
            if (!points[i].IsConvex)
            {
                concavePoints.Add(points[i]);
            }
        }

        int activePoints = points.Count;
        int safety = 300000;

        EdgePoint pointer = points[0];

        while (activePoints > 3 && safety > 0)
        {
            if (pointer.IsConvex &&
                !pointer.OnEdgePoint &&
                !CheckConcavePoints(pointer.Position, pointer.LeftNeighbour.Position, pointer.RightNeighbour.Position, concavePoints))
            {
                AddTriangle(subMeshID,
                            pointer.Position, pointer.LeftNeighbour.Position, pointer.RightNeighbour.Position,
                            normal, normal, normal,
                            pointer.Position, pointer.LeftNeighbour.Position, pointer.RightNeighbour.Position);
                pointer.RightNeighbour.SetLeftNeighbour(pointer.LeftNeighbour);
                pointer.LeftNeighbour.SetRightNeighbour(pointer.RightNeighbour);
                pointer.RightNeighbour.UpdateConvex();
                pointer.LeftNeighbour.UpdateConvex();
                EdgePoint tempPoint = pointer;
                pointer = pointer.LeftNeighbour;
                tempPoint.SetLeftNeighbour(null);
                tempPoint.SetRightNeighbour(null);
                --activePoints;
            }
            else
            {
                pointer.UpdateConvex();
            }
            pointer = pointer.LeftNeighbour;
                        
            --safety;
            if (safety < 1)
            {
                Debug.LogWarning("Safety limit at cut surface triangulation is reached!");
            }
        }
        Debug.Log("Steps used: " + (300000 - safety));
        AddTriangle(subMeshID,
                    pointer.Position, pointer.LeftNeighbour.Position, pointer.RightNeighbour.Position,
                    normal, normal, normal,
                    pointer.Position, pointer.LeftNeighbour.Position, pointer.RightNeighbour.Position);
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

    private bool CheckConcavePoints(Vector3 a, Vector3 b, Vector3 c, List<EdgePoint> concavePoints)
    {
        foreach (EdgePoint point in concavePoints)
        {
            if (a == point.Position ||
                b == point.Position ||
                c == point.Position)
            {
                continue;
            }

            if (PointInTriangle(point.Position, a, b, c))
            {
                return true;
            }
        }
        return false;
    }
}
