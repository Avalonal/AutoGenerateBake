using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.AI;

[ExecuteAlways]
public class NavigationAgent : MonoBehaviour
{
    public Vector3 SelectPos;

    private NavMeshAgent agentComp;

    // Start is called before the first frame update
    void Start()
    {
        agentComp = GetComponent<NavMeshAgent>();
        SceneView.duringSceneGui -= OnSceneGUI;
        SceneView.duringSceneGui += OnSceneGUI;
    }

    // Update is called once per frame
    void Update()
    {
        var tmpTarget = agentComp.destination;

        if (tmpTarget != SelectPos)
        {
            agentComp.SetDestination(SelectPos);
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(SelectPos, 0.1f);
    }

    private void OnSceneGUI(SceneView sceneView)
    {
        Event currentEvent = Event.current;
        if (currentEvent.type == EventType.MouseDown && currentEvent.button == 0)
        {
            Ray ray = HandleUtility.GUIPointToWorldRay(currentEvent.mousePosition);
            RaycastHit rayHit;
            if (Physics.Raycast(ray, out rayHit))
            {
                SelectPos = rayHit.point;
                NavMeshHit hit;
                if (NavMesh.SamplePosition(rayHit.point, out hit, 0.1f, -1))
                {
                    SelectPos = hit.position;
                }
            }
        }
    }
}