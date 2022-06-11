using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[ExecuteInEditMode]
public class PhyTest : MonoBehaviour
{
    public float radius = 0.1f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        var bflag = Physics.CheckSphere(transform.position, transform.localScale.x);

        Color color = bflag ? Color.red : Color.blue;

        var renderer = GetComponent<MeshRenderer>();

        renderer.material.color = color;
    }
}
