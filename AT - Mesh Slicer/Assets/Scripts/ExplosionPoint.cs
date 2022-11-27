using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExplosionPoint : MonoBehaviour
{
    [SerializeField] public float explosionForce;
    [SerializeField] public float explosionDistance;
    [Range(0.01f, 1.0f)]
    [SerializeField] public float volumeProportion = 0.2f;
}
