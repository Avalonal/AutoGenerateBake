using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.AI;

[ExecuteInEditMode]
public class linkTest : MonoBehaviour
{
    public Transform start;

    public Transform end;

    public bool generateLink = false;

    public float width = 0;

    public NavMeshLink linkComp;

    public Vector3 startPos;

    public Vector3 endPos;

    private NavMeshLinkInstance linkInstance = new NavMeshLinkInstance();

    // Update is called once per frame
    void Update()
    {
        if (generateLink)
        {
            AddLink();
        }
    }

    void OnDrawGizmos()
    {
        Draw();
        Test();
    }

    private void Draw()
    {
        NavMeshHit hit;

        startPos = start.position;
        endPos = end.position;

        if (!NavMesh.SamplePosition(startPos, out hit, 2, -1))
        {
            startPos = Vector3.zero;
            Debug.LogErrorFormat("link error: failed to sample start point at {0}", startPos);
        }
        else
            startPos = hit.position;

        if (!NavMesh.SamplePosition(endPos, out hit, 2, -1))
        {
            endPos = Vector3.zero;
            Debug.LogErrorFormat("link error: failed to sample end point at {0}", endPos);
        }
        else
            endPos = hit.position;

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(startPos, 0.5f);
        Gizmos.DrawSphere(endPos, 0.5f);
    }

    void AddLink()
    {
        if (start != null && end != null)
        {
            NavMeshLinkData link = new NavMeshLinkData();

            link.startPosition = startPos;
            link.endPosition = endPos;
            link.bidirectional = false;
            link.agentTypeID = 0;
            link.costModifier = -1;

            link.width = width;
            link.area = 0;

            if (linkInstance.valid)
            {
                linkInstance.Remove();
            }

            linkInstance = NavMesh.AddLink(link);

            Debug.LogFormat("add link {0}", linkInstance.valid ? "success" : "fail");
        }
    }

    public int Index = 0;

    private void Test()
    {
        var data = NavMesh.CalculateTriangulation();

        var vertices = data.vertices;
        var indices = data.indices;

        Index = Mathf.Clamp(Index, 0, indices.Length/3 - 1);

        var i = 3 * Index;
        // for (int i = 0; i < indices.Length; i += 3)
        {
            var p0 = vertices[indices[i]];
            var p1 = vertices[indices[i+1]];
            var p2 = vertices[indices[i+2]];
            Debug.DrawLine(p0,p1, Color.red);
            Debug.DrawLine(p1,p2,Color.red);
            Debug.DrawLine(p2,p0,Color.red);
            Handles.Label(p0, "0");
            Handles.Label(p1, "1");
            Handles.Label(p2, "2");
        }
    }
}