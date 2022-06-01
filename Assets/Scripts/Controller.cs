using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Controller : MonoBehaviour
{
    public Vector3 Center = Vector3.zero;
    public int Count = 2;
    public float Radius = 5;
    public float Speed = 2;
    public GameObject AgentGameObject;
    public bool RandomPriority = false;

    private List<Agent> Agents;

    void Start()
    {
        Agents = new List<Agent>();

        if (AgentGameObject != null)
            GenerateAgent();
    }

    void Update()
    {
        UpdateAgents();
    }

    void FixedUpdate()
    {
        FixedUpdateAgents();
    }

    void OnGUI()
    {
        if (GUI.Button(new Rect(10, 10, 300, 30), "ReStart"))
        {
            foreach (var agent in Agents)
            {
                Destroy(agent.Root);
            }

            Agents.Clear();
            GenerateAgent();
        }
    }

    private void GenerateAgent()
    {
        var step = 360f / Count;

        for (int i = 0; i < Count; ++i)
        {
            var pos = Center + new Vector3(Mathf.Sin(Mathf.Deg2Rad * i * step), 0, Mathf.Cos(Mathf.Deg2Rad * i * step)) * Radius;
            GameObject agentRoot = Instantiate(AgentGameObject, pos, Quaternion.identity);

            var renderer = agentRoot.GetComponent<MeshRenderer>();
            
            renderer.material.color = new Color(Mathf.Sin(Mathf.Deg2Rad * i * step), 0, Mathf.Cos(Mathf.Deg2Rad * i * step), 1);

            var agent = new Agent
            {
                Root = agentRoot,
                Target = Center + new Vector3(Mathf.Sin(Mathf.Deg2Rad * (i * step + 180)), 0, Mathf.Cos(Mathf.Deg2Rad * (i * step + 180))) * Radius,
                Speed = Speed,
                RandomPriority = RandomPriority,
            };
            agent.Init();

            Agents.Add(agent);
        }
    }

    private void UpdateAgents()
    {
        foreach (var agent in Agents)
        {
            agent.Update();
        }
    }

    private void FixedUpdateAgents()
    {
        foreach (var agent in Agents)
        {
            agent.FixedUpdate();
        }
    }
}