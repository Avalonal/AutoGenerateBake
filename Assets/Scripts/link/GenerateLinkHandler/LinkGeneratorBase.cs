using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace LinkBake
{

    public abstract class LinkGeneratorBase
    {
        public ELinkGeneratorType GeneratorType { get; }

        protected Vector3[] _vertices;
        protected List<int> _edges;
        protected NavMeshAgentBuildSetting _agentSetting;
        protected float _eps;

        protected LinkGeneratorBase(ELinkGeneratorType type, Vector3[] vertices, List<int> edges, NavMeshAgentBuildSetting agentSetting)
        {
            GeneratorType = type;
            _vertices = vertices;
            _edges = edges;
            _agentSetting = agentSetting;
        }

        public virtual void GenerateLinks(out List<LinkInfo> linkInfos, float eps = 0.001f)
        {
            _eps = eps;
            linkInfos = new List<LinkInfo>();
            var total = (int)(_edges.Count * 0.5f);
            for (int i = 0; i < total; ++i)
            {
                GenerateLinksAlongSelectEdge(i, ref linkInfos);
            }
        }

        public abstract void GenerateLinkInfos(Vector3 normal, ref List<Vector3> startPositions, out List<LinkInfo> linkInfos);

        private void GenerateLinksAlongSelectEdge(int index, ref List<LinkInfo> totalLinkInfos)
        {
            var vertices = _vertices;

            var start = vertices[_edges[2 * index]];
            var end = vertices[_edges[2 * index + 1]];

            var normal = end - start;
            normal.y = 0;
            normal = new Vector3(-normal.z, 0, normal.x);
            normal = normal.normalized;

            List<Vector3> startPositions;
            List<LinkInfo> linkInfos;
            List<LinkInfo> mergedLinkInfos;
            GenerateStartPositionsAlongEdge(normal, ref start, ref end, out startPositions);

            if(startPositions == null || startPositions.Count <= 0)
                return;

            GenerateLinkInfos(normal, ref startPositions, out linkInfos);
            MergeLinks(ref linkInfos, out mergedLinkInfos);

            totalLinkInfos.AddRange(mergedLinkInfos);
        }

        private void GenerateStartPositionsAlongEdge(Vector3 normal, ref Vector3 start, ref Vector3 end, out List<Vector3> startPositions)
        {
            NavMeshHit hit;

            var cellErrorOffset = -_agentSetting.CellSize * 0.1f * normal;
            var mid = (start + end) * 0.5f - cellErrorOffset;

            var radius = _agentSetting.AgentRadius;
            var checkPos = mid + normal * radius * 0.5f;
            var checkRadius = Mathf.Max(_agentSetting.CellSize, radius * 0.4f - _agentSetting.CellSize);

            startPositions = null;

            if (!NavMesh.SamplePosition(mid, out hit, checkRadius, -1))
            {
                return;
            }

            var errorOffset = hit.position - mid;
            mid = hit.position;
            start += errorOffset;
            end += errorOffset;

            if (NavMesh.SamplePosition(checkPos, out hit, checkRadius, -1))
            {
                return;
            }

            var sampleStart = start;
            var sampleEnd = end;
            var sampleStep = radius * 2f;

            GenerateSamplePositions(sampleStart, sampleEnd, sampleStep, out startPositions);
        }

        public virtual void GenerateSamplePositions(Vector3 start, Vector3 end, float stepSize, out List<Vector3> samplePosition)
        {
            var mid = (start + end) * 0.5f;
            var halfVec = end - mid;
            var unit = halfVec.normalized;
            var halfLength = halfVec.magnitude;

            var maxCount = (int)(halfLength / stepSize);
            samplePosition = new List<Vector3>();

            var step = unit * stepSize;
            var pos = mid - step * maxCount;

            maxCount = 2 * maxCount + 1;

            for (int i = 0; i < maxCount; ++i)
            {
                samplePosition.Add(pos);
                pos += step;
            }
        }

        private void MergeLinks(ref List<LinkInfo> totalLinkInfos, out List<LinkInfo> mergedLinkInfos)
        {
            mergedLinkInfos = new List<LinkInfo>();
            if (totalLinkInfos.Count <= 0)
            {
                return;
            }

            var startSamplePos = totalLinkInfos[0].Start;
            var endSamplePos = totalLinkInfos[totalLinkInfos.Count - 1].Start;
            var dir = endSamplePos - startSamplePos;
            dir = dir.normalized;

            var stepSize = _agentSetting.AgentRadius * 2;
            var step = stepSize * dir;

            var linkLeftBound = totalLinkInfos[0];
            var linkRightBound = totalLinkInfos[0];

            for (int i = 1; i < totalLinkInfos.Count; ++i)
            {
                var linkInfo = totalLinkInfos[i];

                var tmpStep = linkInfo.End - linkRightBound.End;
                var diff = (tmpStep - step).sqrMagnitude;

                if (diff < _eps)
                {
                    linkLeftBound.Width += stepSize;
                    linkRightBound = linkInfo;
                }
                else
                {
                    GenerateMergedLink(linkLeftBound, linkRightBound, ref mergedLinkInfos);

                    linkLeftBound = linkRightBound = linkInfo;
                }
            }

            GenerateMergedLink(linkLeftBound, linkRightBound, ref mergedLinkInfos);
        }

        private void GenerateMergedLink(LinkInfo left, LinkInfo right, ref List<LinkInfo> mergedLinkInfos)
        {
            var startMid = (left.Start + right.Start) * 0.5f;
            var endMid = (left.End + right.End) * 0.5f;
            var width = left.Width;

            var origin = startMid;
            startMid -= origin;
            endMid -= origin;

            var from = right.Start - left.Start;
            var to = from;
            from.y = 0;

            var rot = Quaternion.FromToRotation(from, to);

            mergedLinkInfos.Add(new LinkInfo
            {
                Start = startMid,
                End = endMid,
                Width = width,
                Origin = origin,
                Rotation = rot,
            });
        }
    }

    
}
