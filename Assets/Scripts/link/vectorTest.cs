using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class vectorTest : MonoBehaviour
{
    public List<Vector3> segments = new List<Vector3>();
    public const float eps = 1e-5f;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        if (segments.Count < 4)
            return;

        var source = new Line { Start = segments[2], End = segments[3] };
        var target = new Line { Start = segments[0], End = segments[1] };
        var projectedSource = GetVectorProjection(source, target);

        source.Draw(Color.yellow);
        target.Draw(Color.blue);
        projectedSource.Draw(Color.red);

        if (IsVectorIntersected(projectedSource, target))
        {
            var intersection = GetVectorIntersection(projectedSource, target);
            intersection.Draw(Color.green);
        }
    }

    private Line GetVectorProjection(Line source, Line target)
    {
        var v1 = target.Vector;
        var v2 = source.Vector;
        var v3 = source.Start - target.Start;

        var nv1 = v1.normalized;

        var pv2 = Vector3.Dot(nv1, v2) * nv1;
        var pv3 = Vector3.Dot(nv1, v3) * nv1;

        var pStart = pv3 + target.Start;
        var pEnd = pStart + pv2;

        return new Line { Start = pStart, End = pEnd };
    }

    private bool IsVectorIntersected(Line line1, Line line2)
    {
        var length1 = line1.Vector.sqrMagnitude;
        var length2 = line2.Vector.sqrMagnitude;
        if (length1 <= eps || length2 <= eps)
        {
            return false;
        }

        if (IsSameDirection(line1, line2) < 0)
        {
            Reverse(ref line2);
        }

        var score1 = IsSameDirection(line1.End - line2.Start, line1.Vector);
        var score2 = IsSameDirection(line2.End - line1.Start, line1.Vector);

        var score = score1 + score2;

        switch (score)
        {
            case 0:
                return false;
            case 1:
                return false;
            case 2:
                return true;
            default:
                Debug.LogError("error!!!\n" + StackTraceUtility.ExtractStackTrace().ToString());
                return false;
        }
    }

    private Line GetVectorIntersection(Line line1, Line line2)
    {
        var origin = line1.Start - line1.Vector.normalized * 100000;

        List<Vector3> p = new List<Vector3>
        {
            line1.Start - origin,
            line1.End - origin,
            line2.Start - origin,
            line2.End - origin,
        };

        p.Sort((a, b) => a.sqrMagnitude.CompareTo(b.sqrMagnitude));

        return new Line { Start = p[1] + origin, End = p[2] + origin };
    }

    private int IsSameDirection(Line line1, Line line2)
    {
        return IsSameDirection(line1.Vector, line2.Vector);
    }

    private int IsSameDirection(Vector3 v1, Vector3 v2)
    {
        var val = Vector3.Dot(v1, v2);

        if (Mathf.Abs(val) < eps)
            return 0;
        else if (val < 0)
            return -1;
        else
            return 1;
    }

    private void Reverse(ref Line line)
    {
        var p = line.Start;
        line.Start = line.End;
        line.End = p;
    }
}

public struct Line
{
    public Vector3 Start;
    public Vector3 End;

    public Vector3 Vector
    {
        get { return End - Start; }
    }

    public void Draw(Color color)
    {
        Debug.DrawLine(Start, End, color);
    }
}