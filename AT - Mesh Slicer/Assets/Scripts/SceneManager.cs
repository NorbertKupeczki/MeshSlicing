using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class SceneManager : MonoBehaviour
{
    [SerializeField] List<MeshDestroy> destroyables = new List<MeshDestroy> { };

    private void Awake()
    {
        
    }

    public void ExplodeMeshes(InputAction.CallbackContext context)
    {
        if (context.performed)
        {
            foreach(var destroyable in destroyables)
            {
                destroyable.StartDestruction();
            }
        }
    }
}
