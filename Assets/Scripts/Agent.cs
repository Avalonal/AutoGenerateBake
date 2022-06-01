using System.Text;
using UnityEngine;
using UnityEngine.AI;

public class Agent
{
    public Vector3 Velocity;
    public GameObject Root;
    public Vector3 Target;
    public float Speed;
    public bool RandomPriority;

    private Rigidbody RigidBody;
    private NavMeshAgent NavMeshAgent;

    public void Init()
    {
        RigidBody = Root.GetComponent<Rigidbody>();
        NavMeshAgent = Root.GetComponent<NavMeshAgent>();

        NavMeshAgent.updatePosition = false;
        NavMeshAgent.updateRotation = false;

        NavMeshAgent.SetDestination(Target);

        if (RandomPriority)
            NavMeshAgent.avoidancePriority = (int) Random.Range(50, 99);
    }

    public void Update()
    {
        NavMeshAgent.nextPosition = Root.transform.position;

        UpdateVelocity();
        
        if(Vector3.SqrMagnitude(NavMeshAgent.desiredVelocity) > 0.1f)
            Velocity = NavMeshAgent.desiredVelocity;

        UpdatePosition();

        StringBuilder stringBuilder = new StringBuilder();

        foreach (var posCorner in NavMeshAgent.path.corners)
        {
            stringBuilder.Append(posCorner + ",");
        }

        Debug.Log(stringBuilder.ToString());
    }

    public void FixedUpdate()
    {
        RigidBody.velocity = Vector3.zero;
    }

    private void UpdateVelocity()
    {
        Velocity = (Target - Root.transform.position).normalized * Speed;
    }

    private void UpdatePosition()
    {
        if(Vector3.Distance(Root.transform.position,Target) > 0.3f)
            Root.transform.Translate(Velocity * Time.deltaTime);
    }
}
