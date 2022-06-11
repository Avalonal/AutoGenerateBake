using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace LinkBake
{
    public class DropDownLinkGenerator : LinkGeneratorBase
    {
        private float _dropHeight;
        private float _dropDis;
        public DropDownLinkGenerator(ELinkGeneratorType type, Vector3[] vertices, List<int> edges, NavMeshAgentBuildSetting agentSetting) : base(type, vertices, edges, agentSetting)
        {
            
        }

        public void UpdateParameters(float dropHeight, float dropDis)
        {
            _dropDis = dropDis;
            _dropHeight = dropHeight;
        }

        public override void GenerateLinkInfos(Vector3 normal, ref List<Vector3> startPositions, out List<LinkInfo> linkInfos)
        {
            List<Vector3> samplePositions;
            GenerateDropTopPositions(normal, ref startPositions, out samplePositions);

            GenerateDropDownLinkInfos(_agentSetting.AgentRadius, normal, ref samplePositions, out linkInfos);
        }

        private void GenerateDropTopPositions(Vector3 normal, ref List<Vector3> startPositions, out List<Vector3> samplePositions)
        {
            var offset = _dropDis * normal;

            for (int i = 0; i < startPositions.Count; ++i)
            {
                startPositions[i] += offset;
            }

            samplePositions = startPositions;
        }

        private void GenerateDropDownLinkInfos(float radius, Vector3 normal, ref List<Vector3> samplePositions, out List<LinkInfo> dropDownLinkInfos)
        {
            var offset = _dropDis * normal;
            dropDownLinkInfos = new List<LinkInfo>();
            for (int i = 0; i < samplePositions.Count; i++)
            {
                var samplePosition = samplePositions[i];
                if (Physics.CheckSphere(samplePosition, radius, -1))
                    continue;

                RaycastHit hit;
                if (!Physics.Raycast(samplePosition, Vector3.down, out hit, _dropHeight, -1))
                    continue;

                NavMeshHit navHit;
                if (NavMesh.SamplePosition(hit.point, out navHit, radius, -1))
                {
                    dropDownLinkInfos.Add(new LinkInfo { Start = samplePosition - offset, End = navHit.position });
                }
            }
        }
    }
}
