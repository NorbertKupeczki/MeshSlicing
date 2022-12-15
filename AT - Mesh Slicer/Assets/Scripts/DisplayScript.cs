using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DisplayScript : MonoBehaviour
{
    public float angle { get; set; } = 0.0f;

    void OnGUI()
    {
        GUI.Label(new Rect(25, 25, 200, 40), "Angle to Obj1: " + angle);
    }
}
