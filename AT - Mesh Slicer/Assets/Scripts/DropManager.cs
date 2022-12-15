using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DropManager : MonoBehaviour
{
    [SerializeField] public List<GameObject> baubles;
    [SerializeField] float startDelay = 2.0f;
    [SerializeField] float dropDelay = 1.0f;

    private Queue<GameObject> queue = new Queue<GameObject>();

    private void Awake()
    {
        foreach (GameObject obj in baubles)
        {
            queue.Enqueue(obj);
        }
    }

    private void Start()
    {
        StartCoroutine(DropBaubles(startDelay, dropDelay));   
    }

    IEnumerator DropBaubles(float _startDelay, float _dropDelay)
    {
        yield return new WaitForSeconds(_startDelay);
        while (queue.Count > 0)
        {
            queue.Dequeue().gameObject.GetComponent<Rigidbody>().useGravity = true;
            queue.Dequeue().gameObject.GetComponent<Rigidbody>().useGravity = true;
            yield return new WaitForSeconds(_dropDelay);
        }
        yield break;
    }
}
