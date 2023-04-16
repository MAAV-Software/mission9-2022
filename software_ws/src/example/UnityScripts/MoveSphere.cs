using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveSphere : MonoBehaviour
{
    public float total_time = 0.0f;

    public float x_amplitude = 1.0f;
    public float x_period = 1.0f;

    public float x_offset = 2.0f;

    public float z_amplitude = 1.0f;
    public float z_period = 1.0f;

    public float z_offset = 0.0f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        total_time += Time.deltaTime;

        transform.position = new Vector3(x_amplitude * Mathf.Sin(x_period * total_time + x_offset), 1, z_amplitude * Mathf.Sin(z_period * total_time + z_offset));
    }
}
