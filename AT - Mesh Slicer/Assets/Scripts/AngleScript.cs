using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;

public class AngleScript : MonoBehaviour
{
    public GameObject vector1;
    public GameObject vector2;
    public float _timeStep = 0.1f;

    private delegate IEnumerator CheckAngle_CR(GameObject v1, GameObject v2);
    private CheckAngle_CR _checkAngle;
    
    private Vector3 origin = Vector3.zero;
    private DisplayScript ds;

    private void Awake()
    {
        ds = gameObject.GetComponent<DisplayScript>();
        _checkAngle = CheckAngles;
    }

    void Start()
    {
        StartCoroutine(_checkAngle(vector1, vector2));
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawLine(origin, vector1.transform.position);

        Gizmos.color = Color.red;
        Gizmos.DrawLine(origin, vector2.transform.position);   
    }
    IEnumerator CheckAngles(GameObject v1, GameObject v2)
    {
        Vector3 planeNormal = Vector3.Cross(v1.transform.position, v2.transform.position);

        while (true)
        {
            yield return new WaitForSeconds(_timeStep);
            ds.angle = Vector3.SignedAngle(v1.transform.position, v2.transform.position, planeNormal);
        }
    }

}
