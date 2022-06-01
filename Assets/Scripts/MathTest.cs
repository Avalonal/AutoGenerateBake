using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class MathTest : MonoBehaviour
{
    public Vector3 input = Vector3.zero;
    public Vector3 output = Vector3.zero;

    void Start()
    {
        
    }

    void Update()
    {
        input = input.normalized;
        Debug.DrawRay(transform.position, input, Color.black);

        var vel = CalVelocityDir(transform, transform.position, transform.position + input);

        output = CalOutput(vel);
    }

    public Vector3 CalVelocityDir(Transform self, Vector3 pt0, Vector3 pt1)
    {
        var velocityDir = pt1 - pt0;

        var rot = self.rotation;

        //人物自身x轴
        var right = rot * Vector3.right;
        var forward = rot * Vector3.forward;

        //将最终的速度方向分解到自身x和z
        var dirX = Vector2.Dot(right.GetXZ(), velocityDir.GetXZ());
        var dirZ = Vector2.Dot(forward.GetXZ(), velocityDir.GetXZ());

        Debug.DrawLine(transform.position, transform.position + right * dirX, Color.red);
        Debug.DrawLine(transform.position, transform.position + forward * dirZ, Color.blue);

        return new Vector3(dirX, 0, dirZ).normalized;
    }

    public Vector3 CalOutput(Vector3 vel)
    {
        var rot = transform.rotation;

        //人物自身x轴
        var right = rot * Vector3.right;
        var forward = rot * Vector3.forward;

        var dirX = (vel.x * right).x + (vel.z * forward).x;
        var dirZ = (vel.x * right).z + (vel.z * forward).z;

        return new Vector3(dirX, 0, dirZ).normalized;
    }
}

public static class RobotVectorExtend
{
    public static Vector2 GetXZ(this Vector3 v)
    {
        return new Vector2(v.x, v.z);
    }

    public static Vector3 ToV3(this Vector2 v)
    {
        return new Vector3(v.x, 0, v.y);
    }

    public static bool Approximately(this Vector2 v1, Vector2 v2)
    {
        if (Math.Abs(v1.x - v2.x) <= 0.0001f && Math.Abs(v1.y - v2.y) <= 0.0001f)
            return true;
        return false;
    }

    public static bool Approximately(this float f1, float f2)
    {
        return Math.Abs(f1 - f2) <= 0.0001f;
    }

}