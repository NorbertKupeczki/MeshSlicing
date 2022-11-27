//using System;
//using System.Collections;
using System.Collections;
using System.Collections.Generic;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.InputSystem;

public class MeshDestroy : MonoBehaviour
{
    private bool edgeSet = false;
    private Vector3 edgeVertex = Vector3.zero;
    private Vector2 edgeUV = Vector2.zero;
    private Plane edgePlane = new Plane();
    private ExplosionPoint explosionData;
    private float ObjectVolume { get; set; } // Mesh volume
    private float OriginalVolume { get; set; } = 0.0f; // Original volume of the mesh
    
    [SerializeField] int numberOfCuts  = 1;
    [SerializeField] Transform centreOfExplosion;
    [Header("Cutting properties")]
    [SerializeField] bool randomizeCuts = true;
    [SerializeField] List<GameObject> cuttingObjects = new List<GameObject>();
    [SerializeField] List<Plane> cuttingPlanes = new List<Plane> { };
    [SerializeField] Vector3 fixedCuttingOffset = new Vector3(0.0f, 0.0f, 0.0f);
    [Header("Extras")]
    [SerializeField] public bool addParticles = false;
    [SerializeField] public ParticleSystem smokeParticles;
    private void Awake()
    {
        ObjectVolume = CalculateVolumeOfMesh(gameObject.GetComponent<MeshFilter>().mesh);
    }

    void Start()
    {
        explosionData = centreOfExplosion.gameObject.GetComponent<ExplosionPoint>();
        if (OriginalVolume == 0.0f)
        {
            OriginalVolume = ObjectVolume;
        }
    }

    public void StartDestruction()
    {
        ExplodeMesh(randomizeCuts);
    }

    public void CascadingDestruction()
    {
        StartCoroutine(ExplodeCoroutine());
    }

    // Update is called once per frame
    void Update()
    {

    }

    public bool IsAddingParticles()
    {
        return addParticles;
    }

    private IEnumerator ExplodeCoroutine()
    {
        yield return null;
        ExplodeMesh(true);
    }

    private void ExplodeMesh(bool _randomCuts)
    {
        var originalMesh = GetComponent<MeshFilter>().mesh;
        originalMesh.RecalculateBounds();
        var parts = new List<PartMesh>();
        var subParts = new List<PartMesh>();

        var mainPart = new PartMesh()
        {
            UV = originalMesh.uv,
            Vertices = originalMesh.vertices,
            Normals = originalMesh.normals,
            Triangles = new int[originalMesh.subMeshCount][],
            Bounds = originalMesh.bounds
        };
        for (int i = 0; i < originalMesh.subMeshCount; i++)
            mainPart.Triangles[i] = originalMesh.GetTriangles(i);

        parts.Add(mainPart);

        if (_randomCuts)
        {
            for (var c = 0; c < numberOfCuts; c++)
            {
                for (var i = 0; i < parts.Count; i++)
                {
                    Bounds bounds = parts[i].Bounds;
                    bounds.Expand(0.25f);

                    Plane plane = GetRandomPlane(bounds);
                    subParts.AddRange(SeparateMeshToSubParts(parts[i], plane));
                }
                parts = new List<PartMesh>(subParts);
                subParts.Clear();
            }
        }
        else
        {
            cuttingPlanes.AddRange(CreateFixedCuttingPlanes(transform.position + fixedCuttingOffset, originalMesh.bounds));

            for (var c = 0; c < cuttingPlanes.Count; c++)
            {
                for (var i = 0; i < parts.Count; i++)
                {   
                    subParts.AddRange(SeparateMeshToSubParts(parts[i], cuttingPlanes[c]));
                }
                parts = new List<PartMesh>(subParts);
                subParts.Clear();
            }
        }
        
        foreach (var part in parts)
        {
            ApplyExplosionOnPart(part);
            MeshDestroy md = part._GameObject.GetComponent<MeshDestroy>();
            if (md.GetVolumeProportion() > explosionData.volumeProportion)
            {
                md.CascadingDestruction();
            }
        }
        
        
        Destroy(gameObject);
    }

    private PartMesh GenerateMesh(PartMesh original, Plane plane, bool left)
    {
        var partMesh = new PartMesh() { };
        var ray1 = new Ray();
        var ray2 = new Ray();


        for (var i = 0; i < original.Triangles.Length; i++)
        {
            var triangles = original.Triangles[i];
            edgeSet = false;

            for (var j = 0; j < triangles.Length; j = j + 3)
            {
                var sideA = plane.GetSide(original.Vertices[triangles[j]]) == left;
                var sideB = plane.GetSide(original.Vertices[triangles[j + 1]]) == left;
                var sideC = plane.GetSide(original.Vertices[triangles[j + 2]]) == left;

                var sideCount = (sideA ? 1 : 0) +
                                (sideB ? 1 : 0) +
                                (sideC ? 1 : 0);
                if (sideCount == 0)
                {
                    continue;
                }
                if (sideCount == 3)
                {
                    partMesh.AddTriangle(i,
                                         original.Vertices[triangles[j]], original.Vertices[triangles[j + 1]], original.Vertices[triangles[j + 2]],
                                         original.Normals[triangles[j]], original.Normals[triangles[j + 1]], original.Normals[triangles[j + 2]],
                                         original.UV[triangles[j]], original.UV[triangles[j + 1]], original.UV[triangles[j + 2]]);
                    continue;
                }

                //cut points
                var singleIndex = sideB == sideC ? 0 : sideA == sideC ? 1 : 2;

                ray1.origin = original.Vertices[triangles[j + singleIndex]];
                var dir1 = original.Vertices[triangles[j + ((singleIndex + 1) % 3)]] - original.Vertices[triangles[j + singleIndex]];
                ray1.direction = dir1;
                plane.Raycast(ray1, out var enter1);
                var lerp1 = enter1 / dir1.magnitude;

                ray2.origin = original.Vertices[triangles[j + singleIndex]];
                var dir2 = original.Vertices[triangles[j + ((singleIndex + 2) % 3)]] - original.Vertices[triangles[j + singleIndex]];
                ray2.direction = dir2;
                plane.Raycast(ray2, out var enter2);
                var lerp2 = enter2 / dir2.magnitude;

                //first vertex = ancor
                AddEdge(i,
                        partMesh,
                        left ? plane.normal * -1f : plane.normal,
                        ray1.origin + ray1.direction.normalized * enter1,
                        ray2.origin + ray2.direction.normalized * enter2,
                        Vector2.Lerp(original.UV[triangles[j + singleIndex]], original.UV[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                        Vector2.Lerp(original.UV[triangles[j + singleIndex]], original.UV[triangles[j + ((singleIndex + 2) % 3)]], lerp2));

                if (sideCount == 1)
                {
                    partMesh.AddTriangle(i,
                                        original.Vertices[triangles[j + singleIndex]],
                                        //Vector3.Lerp(originalMesh.vertices[triangles[j + singleIndex]], originalMesh.vertices[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        //Vector3.Lerp(originalMesh.vertices[triangles[j + singleIndex]], originalMesh.vertices[triangles[j + ((singleIndex + 2) % 3)]], lerp2),
                                        ray1.origin + ray1.direction.normalized * enter1,
                                        ray2.origin + ray2.direction.normalized * enter2,
                                        original.Normals[triangles[j + singleIndex]],
                                        Vector3.Lerp(original.Normals[triangles[j + singleIndex]], original.Normals[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        Vector3.Lerp(original.Normals[triangles[j + singleIndex]], original.Normals[triangles[j + ((singleIndex + 2) % 3)]], lerp2),
                                        original.UV[triangles[j + singleIndex]],
                                        Vector2.Lerp(original.UV[triangles[j + singleIndex]], original.UV[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        Vector2.Lerp(original.UV[triangles[j + singleIndex]], original.UV[triangles[j + ((singleIndex + 2) % 3)]], lerp2));

                    continue;
                }

                if (sideCount == 2)
                {
                    partMesh.AddTriangle(i,
                                        ray1.origin + ray1.direction.normalized * enter1,
                                        original.Vertices[triangles[j + ((singleIndex + 1) % 3)]],
                                        original.Vertices[triangles[j + ((singleIndex + 2) % 3)]],
                                        Vector3.Lerp(original.Normals[triangles[j + singleIndex]], original.Normals[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        original.Normals[triangles[j + ((singleIndex + 1) % 3)]],
                                        original.Normals[triangles[j + ((singleIndex + 2) % 3)]],
                                        Vector2.Lerp(original.UV[triangles[j + singleIndex]], original.UV[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        original.UV[triangles[j + ((singleIndex + 1) % 3)]],
                                        original.UV[triangles[j + ((singleIndex + 2) % 3)]]);
                    partMesh.AddTriangle(i,
                                        ray1.origin + ray1.direction.normalized * enter1,
                                        original.Vertices[triangles[j + ((singleIndex + 2) % 3)]],
                                        ray2.origin + ray2.direction.normalized * enter2,
                                        Vector3.Lerp(original.Normals[triangles[j + singleIndex]], original.Normals[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        original.Normals[triangles[j + ((singleIndex + 2) % 3)]],
                                        Vector3.Lerp(original.Normals[triangles[j + singleIndex]], original.Normals[triangles[j + ((singleIndex + 2) % 3)]], lerp2),
                                        Vector2.Lerp(original.UV[triangles[j + singleIndex]], original.UV[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        original.UV[triangles[j + ((singleIndex + 2) % 3)]],
                                        Vector2.Lerp(original.UV[triangles[j + singleIndex]], original.UV[triangles[j + ((singleIndex + 2) % 3)]], lerp2));
                    continue;
                }


            }
        }

        partMesh.FillArrays();

        return partMesh;
    }

    private void AddEdge(int subMesh, PartMesh partMesh, Vector3 normal, Vector3 vertex1, Vector3 vertex2, Vector2 uv1, Vector2 uv2)
    {
        if (!edgeSet)
        {
            edgeSet = true;
            edgeVertex = vertex1;
            edgeUV = uv1;
        }
        else
        {
            edgePlane.Set3Points(edgeVertex, vertex1, vertex2);

            partMesh.AddTriangle(subMesh,
                                edgeVertex,
                                edgePlane.GetSide(edgeVertex + normal) ? vertex1 : vertex2,
                                edgePlane.GetSide(edgeVertex + normal) ? vertex2 : vertex1,
                                normal,
                                normal,
                                normal,
                                edgeUV,
                                uv1,
                                uv2);
        }
    }

    public class PartMesh
    {
        private List<Vector3> _Verticies = new List<Vector3>();
        private List<Vector3> _Normals = new List<Vector3>();
        private List<List<int>> _Triangles = new List<List<int>>();
        private List<Vector2> _UVs = new List<Vector2>();
        public Vector3[] Vertices;
        public Vector3[] Normals;
        public int[][] Triangles;
        public Vector2[] UV;
        public GameObject _GameObject;
        public Bounds Bounds = new Bounds();

        public PartMesh()
        {

        }

        public void AddTriangle(int submesh, Vector3 vert1, Vector3 vert2, Vector3 vert3, Vector3 normal1, Vector3 normal2, Vector3 normal3, Vector2 uv1, Vector2 uv2, Vector2 uv3)
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

        public void MakeGameobject(MeshDestroy original)
        {
            _GameObject = new GameObject(original.name);
            _GameObject.transform.position = original.transform.position;
            _GameObject.transform.rotation = original.transform.rotation;
            _GameObject.transform.localScale = original.transform.localScale;

            var mesh = new Mesh();
            mesh.name = original.GetComponent<MeshFilter>().mesh.name;

            mesh.vertices = Vertices;
            mesh.normals = Normals;
            mesh.uv = UV;
            for (var i = 0; i < Triangles.Length; i++)
                mesh.SetTriangles(Triangles[i], i, true);
            Bounds = mesh.bounds;

            var renderer = _GameObject.AddComponent<MeshRenderer>();
            renderer.materials = original.GetComponent<MeshRenderer>().materials;

            var filter = _GameObject.AddComponent<MeshFilter>();
            filter.mesh = mesh;

            var collider = _GameObject.AddComponent<MeshCollider>();
            collider.convex = true;
            collider.material = original.GetComponent<Collider>().material;
                        
            var meshDestroy = _GameObject.AddComponent<MeshDestroy>();
            meshDestroy.ObjectVolume = CalculateVolumeOfMesh(mesh);
            meshDestroy.OriginalVolume = original.OriginalVolume;
            meshDestroy.numberOfCuts = original.numberOfCuts;
            meshDestroy.centreOfExplosion = original.centreOfExplosion;
            meshDestroy.addParticles = original.addParticles;
            meshDestroy.smokeParticles = original.smokeParticles;

            var rigidbody = _GameObject.AddComponent<Rigidbody>();
            rigidbody.mass = CalculateProportionalMass(original, meshDestroy.ObjectVolume);
            rigidbody.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rigidbody.drag = original.gameObject.GetComponent<Rigidbody>().drag;
            rigidbody.angularDrag = original.gameObject.GetComponent<Rigidbody>().angularDrag;

            if (original.IsAddingParticles())
            {
                Debug.Log("Adding particle system");
                ParticleSystem particleSystem = _GameObject.AddComponent<ParticleSystem>();
                ComponentUtility.CopyComponent(original.smokeParticles);
                ComponentUtility.PasteComponentValues(particleSystem);
            }

        }
        public float CalculateProportionalMass(MeshDestroy originalObject, float newVolume)
        {
            float originalMass = originalObject.gameObject.GetComponent<Rigidbody>().mass;
            float originalVolume = originalObject.ObjectVolume;
            return newVolume * originalMass / originalVolume;
        }
    }

    public static float SignedVolumeOfTriangle(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        float v321 = p3.x * p2.y * p1.z;
        float v231 = p2.x * p3.y * p1.z;
        float v312 = p3.x * p1.y * p2.z;
        float v132 = p1.x * p3.y * p2.z;
        float v213 = p2.x * p1.y * p3.z;
        float v123 = p1.x * p2.y * p3.z;

        return (1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123);
    }

    public static float CalculateVolumeOfMesh(Mesh mesh)
    {
        float volume = 0;

        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 p1 = vertices[triangles[i + 0]];
            Vector3 p2 = vertices[triangles[i + 1]];
            Vector3 p3 = vertices[triangles[i + 2]];
            volume += SignedVolumeOfTriangle(p1, p2, p3);
        }
        return Mathf.Abs(volume);
    }

    private Plane CreateCuttingPlane(GameObject planeObject)
    {
        Mesh mesh = planeObject.GetComponent<MeshFilter>().mesh;

        Vector3 a = transform.TransformPoint(mesh.vertices[49]);
        Vector3 b = transform.TransformPoint(mesh.vertices[59]);
        Vector3 c = transform.TransformPoint(mesh.vertices[60]);
        
        Debug.DrawLine(a, b, Color.green, 30.0f);
        Debug.DrawLine(b, c, Color.red, 30.0f);
        Debug.DrawLine(c, a, Color.blue, 30.0f);

        Plane planeToReturn = new Plane(a, b, c);

        return planeToReturn;
    }

    private List<Plane> CreateFixedCuttingPlanes(Vector3 origin, Bounds bounds)
    {
        List<Plane> planeList = new List<Plane>(9);

        planeList.Add(new Plane(new Vector3(1.0f,0.0f,0.0f), origin.x + bounds.extents.x * 0.5f));
        planeList.Add(new Plane(new Vector3(-1.0f,0.0f,0.0f), origin.x + bounds.extents.x * 0.5f));
        planeList.Add(new Plane(new Vector3(0.0f,1.0f,0.0f), origin.y + bounds.extents.y * 0.5f));
        planeList.Add(new Plane(new Vector3(0.0f,-1.0f,0.0f), origin.y + bounds.extents.y * 0.5f));
        planeList.Add(new Plane(new Vector3(0.0f,0.0f,1.0f), origin.z + bounds.extents.z * 0.5f));
        planeList.Add(new Plane(new Vector3(0.0f,0.0f,-1.0f), origin.z + bounds.extents.z * 0.5f));
        planeList.Add(new Plane(new Vector3(1.0f,0.0f,0.0f), 0.0f));
        planeList.Add(new Plane(new Vector3(0.0f,1.0f,0.0f), 0.0f));
        planeList.Add(new Plane(new Vector3(0.0f,0.0f,1.0f), 0.0f));

        return planeList;
    }

    private Plane GetRandomPlane(Bounds _bounds)
    {
        return new Plane(Random.onUnitSphere * 0.5f, new Vector3(Random.Range(_bounds.min.x, _bounds.max.x),
                                                                 Random.Range(_bounds.min.y, _bounds.max.y),
                                                                 Random.Range(_bounds.min.z, _bounds.max.z)));
    }

    private List<PartMesh> SeparateMeshToSubParts(PartMesh _meshToSeparate, Plane _plane)
    {
        List<PartMesh> result = new List<PartMesh>();
        result.Add(GenerateMesh(_meshToSeparate, _plane, true));
        result.Add(GenerateMesh(_meshToSeparate, _plane, false));

        return result;
    }

    private void ApplyExplosionOnPart(PartMesh _parts)
    {
        _parts.MakeGameobject(this);
        _parts._GameObject.GetComponent<Rigidbody>().AddExplosionForce(explosionData.explosionForce, centreOfExplosion.position, explosionData.explosionDistance);
     
    }

    public float GetVolumeProportion()
    {
        return ObjectVolume / OriginalVolume;
    }
}