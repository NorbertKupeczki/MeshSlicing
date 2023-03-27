using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class SceneManager : MonoBehaviour
{
    [SerializeField] List<MeshDestroy> destroyables = new List<MeshDestroy> { };
    [SerializeField] bool timedExecution = false;
    [Range (0.5f,5.0f)]
    [SerializeField] float delay = 1.0f;
    private void Awake()
    {
        
    }

    public void StartProcess()
    {
        if (timedExecution)
        {
            StartCoroutine(TimedDestruction(delay));
        }
        else
        {
            foreach (MeshDestroy destroyable in destroyables)
            {
                destroyable.StartDestruction();
            }
        }
    }

    public void ExplodeMeshes(InputAction.CallbackContext context)
    {
        if (context.performed)
        {
            StartProcess();            
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
