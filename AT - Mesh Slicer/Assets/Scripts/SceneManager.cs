using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class SceneManager : MonoBehaviour
{
    [SerializeField] List<MeshDestroy> destroyables = new List<MeshDestroy> { };
    [SerializeField] bool timedDestruction = false;
    [SerializeField] float detonationDelay = 1.0f;
    private void Awake()
    {
        
    }

    public void ExplodeMeshes(InputAction.CallbackContext context)
    {
        if (context.performed)
        {
            if (timedDestruction)
            {
                StartCoroutine(TimedDestruction(detonationDelay));
            }
            else
            {
                foreach (var destroyable in destroyables)
                {
                    destroyable.StartDestruction();
                }
            }
            
        }
    }

    IEnumerator TimedDestruction(float delay)
    {
        for (int i = 0; i < destroyables.Count; i++)
        {
            destroyables[i].StartDestruction();
            yield return new WaitForSeconds(delay);
        }
        yield break;
    }
}
