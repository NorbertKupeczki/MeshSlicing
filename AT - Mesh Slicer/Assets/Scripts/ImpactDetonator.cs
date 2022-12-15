using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ImpactDetonator : MonoBehaviour
{
    private void OnCollisionEnter(Collision c)
    {
        MeshDestroy collision = c.gameObject.GetComponent<MeshDestroy>();
        if (collision != null)
        {
            if (collision.christmasSpecial)
            {
                c.gameObject.GetComponent<MeshDestroy>().StartDestruction();
            }
        }
    }
}
