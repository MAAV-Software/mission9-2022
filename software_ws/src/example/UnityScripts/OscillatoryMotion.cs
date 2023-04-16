using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OscillatoryMotion : MonoBehaviour
{

    GameObject moveable_sphere;
    // Start is called before the first frame update
    void Start()
    {
        moveable_sphere = GameObject.Find("MoveableSphere");
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 desired_direction = moveable_sphere.transform.position - transform.position;


        //Rotate around z axis first
        Vector2 desired_xy = new Vector2(desired_direction.x, desired_direction.y);
        Vector2 current_xy = new Vector2(transform.up.x, transform.up.y);

        float z_angle = Vector2.SignedAngle(current_xy, desired_xy);
        //transform.Rotate(new Vector3(0, 0, ));

        //Rotate around the x-axis next
        Vector2 desired_zy = new Vector2(desired_direction.z, desired_direction.y);
        Vector2 current_zy = new Vector2(transform.up.z, transform.up.y);

        float x_angle = Vector2.SignedAngle(current_zy, desired_zy);
        transform.Rotate(new Vector3(-x_angle, 0, z_angle));
    }
}
