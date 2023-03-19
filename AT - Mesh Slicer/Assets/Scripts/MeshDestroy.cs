using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MeshDestroy : MonoBehaviour
{
    private bool edgeSet = false;
    private Vector3 edgeVertex = Vector3.zero;
    private Vector2 edgeUV = Vector2.zero;
    private Plane edgePlane = new Plane();
    private ExplosionPoint explosionData;
    private float ObjectVolume; // Mesh volume
    private float OriginalVolume = 0.0f; // Original volume of the mesh

    [Header ("Base settings")]
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
    [SerializeField] public bool christmasSpecial = false;
    
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

    #region "Getters and Setters"
    
    public float GetObjectVolume()
    {
        return ObjectVolume;
    }
    public void SetObjectVolume(float value)
    {
        ObjectVolume = value;
    }

    public float GetOriginalVolume()
    {
        return OriginalVolume;
    }
    public void SetOriginalVolume(float value)
    {
        OriginalVolume = value;
    }
    
    public int GetNumberOfCuts()
    {
        return numberOfCuts;
    }
    public void SetNumberOfCuts(int value)
    {
        numberOfCuts = value;
    }

    public Transform GetCentreOfExplosion()
    {
        return centreOfExplosion;
    }
    public void SetCentreOfExplosion(Transform value)
    {
        centreOfExplosion = value;
    }

    #endregion

    public void StartDestruction()
    {
        ExplodeMesh(randomizeCuts);
    }

    public void CascadingDestruction()
    {
        StartCoroutine(ExplodeCoroutine());
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
        List<MeshClass> meshParts = new List<MeshClass>();
        List<MeshClass> meshSubParts = new List<MeshClass>();

        MeshClass mainMesh = new MeshClass()
        {
            UV = originalMesh.uv,
            Vertices = originalMesh.vertices,
            Normals = originalMesh.normals,
            Triangles = new int[originalMesh.subMeshCount][],
            Bounds = originalMesh.bounds
        };
        for (int i = 0; i < originalMesh.subMeshCount; i++)
            mainMesh.Triangles[i] = originalMesh.GetTriangles(i);

        meshParts.Add(mainMesh);

        if (_randomCuts)
        {
            for (int c = 0; c < numberOfCuts; c++)
            {
                for (int i = 0; i < meshParts.Count; i++)
                {
                    Bounds bounds = meshParts[i].Bounds;
                    bounds.Expand(0.25f);

                    Plane plane = GetRandomPlane(bounds);
                    meshSubParts.AddRange(SeparateMeshToSubParts(meshParts[i], plane));
                }
                meshParts = new List<MeshClass>(meshSubParts);
                meshSubParts.Clear();
            }
        }
        else
        {
            cuttingPlanes.AddRange(CreateFixedCuttingPlanes(transform.position + fixedCuttingOffset, originalMesh.bounds));

            for (int c = 0; c < cuttingPlanes.Count; c++)
            {
                for (int i = 0; i < meshParts.Count; i++)
                {   
                    meshSubParts.AddRange(SeparateMeshToSubParts(meshParts[i], cuttingPlanes[c]));
                }
                meshParts = new List<MeshClass>(meshSubParts);
                meshSubParts.Clear();
            }
        }
        
        foreach (MeshClass part in meshParts)
        {
            ApplyExplosionOnPart(part);
            MeshDestroy md = part.NewGameObject.GetComponent<MeshDestroy>();
            if (md.GetVolumeProportion() > explosionData.volumeProportion)
            {
                md.CascadingDestruction();
            }
        }
        
        Destroy(gameObject);
    }

    private MeshClass GenerateMesh(MeshClass originalMesh, Plane plane, bool above)
    {
        MeshClass newMesh = new MeshClass() { };
        Ray ray1 = new Ray();
        Ray ray2 = new Ray();

        for (int i = 0; i < originalMesh.Triangles.Length; ++i)
        {
            int[] triangles = originalMesh.Triangles[i];
            edgeSet = false;

            for (int j = 0; j < triangles.Length; j += 3)
            {
                bool vertA = plane.GetSide(originalMesh.Vertices[triangles[j]])     == above;
                bool vertB = plane.GetSide(originalMesh.Vertices[triangles[j + 1]]) == above;
                bool vertC = plane.GetSide(originalMesh.Vertices[triangles[j + 2]]) == above;

                int sideCount = (vertA ? 1 : 0) + (vertB ? 1 : 0) + (vertC ? 1 : 0);
                
                if (sideCount == 0)
                {
                    continue;
                }
                if (sideCount == 3)
                {
                    newMesh.AddTriangle(i,
                                        originalMesh.Vertices[triangles[j]], originalMesh.Vertices[triangles[j + 1]], originalMesh.Vertices[triangles[j + 2]],
                                        originalMesh.Normals[triangles[j]], originalMesh.Normals[triangles[j + 1]], originalMesh.Normals[triangles[j + 2]],
                                        originalMesh.UV[triangles[j]], originalMesh.UV[triangles[j + 1]], originalMesh.UV[triangles[j + 2]]);
                    continue;
                }

                //cut points
                int singleIndex = vertB == vertC ? 0 : vertA == vertC ? 1 : 2;

                float enter1 = CastRayToPlane(ref ray1, plane, originalMesh.Vertices[triangles[j + singleIndex]], originalMesh.Vertices[triangles[j + ((singleIndex + 1) % 3)]]);
                float lerp1 = enter1 / ray1.direction.magnitude;

                float enter2 = CastRayToPlane(ref ray2, plane, originalMesh.Vertices[triangles[j + singleIndex]], originalMesh.Vertices[triangles[j + ((singleIndex + 2) % 3)]]);
                float lerp2 = enter2 / ray2.direction.magnitude;
                /*
                //first vertex = anchor
                AddEdge(i,
                        newMesh,
                        left ? plane.normal * -1f : plane.normal,
                        ray1.origin + ray1.direction.normalized * enter1,
                        ray2.origin + ray2.direction.normalized * enter2,
                        Vector2.Lerp(originalMesh.UV[triangles[j + singleIndex]], originalMesh.UV[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                        Vector2.Lerp(originalMesh.UV[triangles[j + singleIndex]], originalMesh.UV[triangles[j + ((singleIndex + 2) % 3)]], lerp2));
                */
                if (sideCount == 1)
                {
                    newMesh.AddTriangle(i,
                                        originalMesh.Vertices[triangles[j + singleIndex]],                                        
                                        ray1.origin + ray1.direction.normalized * enter1,
                                        ray2.origin + ray2.direction.normalized * enter2,
                                        originalMesh.Normals[triangles[j + singleIndex]],
                                        Vector3.Lerp(originalMesh.Normals[triangles[j + singleIndex]], originalMesh.Normals[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        Vector3.Lerp(originalMesh.Normals[triangles[j + singleIndex]], originalMesh.Normals[triangles[j + ((singleIndex + 2) % 3)]], lerp2),
                                        originalMesh.UV[triangles[j + singleIndex]],
                                        Vector2.Lerp(originalMesh.UV[triangles[j + singleIndex]], originalMesh.UV[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        Vector2.Lerp(originalMesh.UV[triangles[j + singleIndex]], originalMesh.UV[triangles[j + ((singleIndex + 2) % 3)]], lerp2));

                    continue;
                }

                if (sideCount == 2)
                {
                    newMesh.AddTriangle(i,
                                        ray1.origin + ray1.direction.normalized * enter1,
                                        originalMesh.Vertices[triangles[j + ((singleIndex + 1) % 3)]],
                                        originalMesh.Vertices[triangles[j + ((singleIndex + 2) % 3)]],
                                        Vector3.Lerp(originalMesh.Normals[triangles[j + singleIndex]], originalMesh.Normals[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        originalMesh.Normals[triangles[j + ((singleIndex + 1) % 3)]],
                                        originalMesh.Normals[triangles[j + ((singleIndex + 2) % 3)]],
                                        Vector2.Lerp(originalMesh.UV[triangles[j + singleIndex]], originalMesh.UV[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        originalMesh.UV[triangles[j + ((singleIndex + 1) % 3)]],
                                        originalMesh.UV[triangles[j + ((singleIndex + 2) % 3)]]);
                    newMesh.AddTriangle(i,
                                        ray1.origin + ray1.direction.normalized * enter1,
                                        originalMesh.Vertices[triangles[j + ((singleIndex + 2) % 3)]],
                                        ray2.origin + ray2.direction.normalized * enter2,
                                        Vector3.Lerp(originalMesh.Normals[triangles[j + singleIndex]], originalMesh.Normals[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        originalMesh.Normals[triangles[j + ((singleIndex + 2) % 3)]],
                                        Vector3.Lerp(originalMesh.Normals[triangles[j + singleIndex]], originalMesh.Normals[triangles[j + ((singleIndex + 2) % 3)]], lerp2),
                                        Vector2.Lerp(originalMesh.UV[triangles[j + singleIndex]], originalMesh.UV[triangles[j + ((singleIndex + 1) % 3)]], lerp1),
                                        originalMesh.UV[triangles[j + ((singleIndex + 2) % 3)]],
                                        Vector2.Lerp(originalMesh.UV[triangles[j + singleIndex]], originalMesh.UV[triangles[j + ((singleIndex + 2) % 3)]], lerp2));
                    continue;
                }
                

            }
        }

        newMesh.FillArrays();

        return newMesh;
    }

    private void AddEdge(int subMesh, MeshClass partMesh, Vector3 normal, Vector3 vertex1, Vector3 vertex2, Vector2 uv1, Vector2 uv2)
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
    
    private float CastRayToPlane(ref Ray ray, Plane targetPlane, Vector3 rayOrigin, Vector3 rayTarget)
    {
        ray.origin = rayOrigin;
        ray.direction = rayTarget - rayOrigin;
        targetPlane.Raycast(ray, out float enter);
        return enter;
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

    public float CalculateVolumeOfMesh(Mesh mesh)
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

    private List<MeshClass> SeparateMeshToSubParts(MeshClass _meshToSeparate, Plane _plane)
    {
        List<MeshClass> result = new List<MeshClass>();
        result.Add(GenerateMesh(_meshToSeparate, _plane, true));
        result.Add(GenerateMesh(_meshToSeparate, _plane, false));

        return result;
    }

    private void ApplyExplosionOnPart(MeshClass _parts)
    {
        _parts.CreateNewGameObject(this);
        _parts.NewGameObject.GetComponent<Rigidbody>().AddExplosionForce(explosionData.explosionForce, centreOfExplosion.position, explosionData.explosionDistance);
     
    }

    public float GetVolumeProportion()
    {
        return ObjectVolume / OriginalVolume;
    }
}