using System.Collections.Generic;
using UnityEditorInternal;
using UnityEngine;

public class MeshClass
{
    private List<Vector3> _Verticies = new List<Vector3>();
    private List<Vector3> _Normals = new List<Vector3>();
    private List<List<int>> _Triangles = new List<List<int>>();
    private List<Vector2> _UVs = new List<Vector2>();
    public Vector3[] Vertices;
    public Vector3[] Normals;
    public int[][] Triangles;
    public Vector2[] UV;
    public GameObject NewGameObject;
    public Bounds Bounds = new Bounds();

    public MeshClass()
    {

    }

    public struct Edge
    {
        public Edge(Transform _pointA, Transform _pointB)
        {
            PointA = _pointA;
            PointB = _pointB;
        }

        public Transform PointA;
        public Transform PointB;

        public bool CheckPoints(Transform check)
        {
            if (check == PointA || check == PointB)
            {
                return true;
            }
            return false;
        }

        public Transform GetOtherPoint(Transform point)
        {
            if (point == PointA)
            {
                return PointB;
            }
            return PointA;
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

        Mesh newMesh = new Mesh();
        newMesh.name = original.GetComponent<MeshFilter>().mesh.name;

        newMesh.vertices = Vertices;
        newMesh.normals = Normals;
        newMesh.uv = UV;
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
    public float CalculateProportionalMass(MeshDestroy originalObject, float newVolume)
    {
        float originalMass = originalObject.gameObject.GetComponent<Rigidbody>().mass;
        float originalVolume = originalObject.GetObjectVolume();
        return newVolume * originalMass / originalVolume;
    }

    public void CreateNewCutMesh()
    {
        NewGameObject = new GameObject();

        Mesh newMesh = new Mesh();

        newMesh.vertices = Vertices;
        newMesh.normals = Normals;
        newMesh.uv = UV;
        for (var i = 0; i < Triangles.Length; i++)
            newMesh.SetTriangles(Triangles[i], i, true);
        Bounds = newMesh.bounds;

        MeshRenderer renderer = NewGameObject.AddComponent<MeshRenderer>();

        MeshFilter filter = NewGameObject.AddComponent<MeshFilter>();
        filter.mesh = newMesh;
    }
}
