using System;
using System.Collections.Generic;
using DataStructures.ViliWonka.KDTree;
using Sirenix.OdinInspector;
using Sirenix.Utilities.Editor;
using UnityEditor;
using UnityEngine;
using UnityEngine.AI;

namespace LinkBake
{
    [ExecuteInEditMode]
    public class BakeLink : MonoBehaviour
    {
        [FoldoutGroup("参数", Order = 1), LabelText("DropDown距离")]
        public float DropDis = 0;

        [FoldoutGroup("参数"), LabelText("是否合并link")]
        public bool MergeLink = false;

        [FoldoutGroup("参数"), LabelText("是否使用KDTree")]
        public bool UseKDTree = false;

        [FoldoutGroup("调试", Order = 9), LabelText("显示所有三角面")]
        public bool ShowAllTriangulations = false;

        [FoldoutGroup("调试"), LabelText("指定边id")]
        public int EdgeId = 0;

        [FoldoutGroup("调试"), LabelText("显示指定边")]
        public bool ShowSelectedEdge = false;

        [FoldoutGroup("调试"), LabelText("显示所有边")]
        public bool ShowAllEdges = true;

        private NavMeshAgentBuildSetting _agentSetting;
        private NavMeshTriangulation _triangulationData;
        private List<NavMeshLinkInstance> _links;
        private List<int> _edges;

        private bool _showDebugInfo = false;
        private float eps = 1e-4f;

        void Start()
        {
            Init();
        }

        void Update()
        {
            EdgeId = Mathf.Clamp(EdgeId, 0, _edges.Count / 2 - 1);
        }

        void OnDrawGizmos()
        {
            if (_showDebugInfo)
            {
                if (ShowAllTriangulations)
                    DrawAllTriangulation();
                if (ShowAllEdges)
                    DrawEdges();
                if (ShowSelectedEdge)
                {
                    DrawEdge(EdgeId, Color.blue);
                    RemoveLinks();
                    GenerateDropDownLinksAlongSelectEdge(EdgeId);
                }
            }
        }

        #region 初始化

        [FoldoutGroup("操作", Order = 0), Button("初始化")]
        private void Init()
        {
            InitAgentSetting();
            InitTriangulationData();
            InitDataStructure();
        }

        private void InitAgentSetting()
        {
            var serializedAssetInterfaceSingleton = Unsupported.GetSerializedAssetInterfaceSingleton("NavMeshProjectSettings");
            var meshProjectSettingsObject = new SerializedObject(serializedAssetInterfaceSingleton);
            _agentSetting = new NavMeshAgentBuildSetting(meshProjectSettingsObject, 0);
        }

        private void InitTriangulationData()
        {
            _triangulationData = NavMesh.CalculateTriangulation();
        }

        private void InitDataStructure()
        {
            _links = new List<NavMeshLinkInstance>();
            DropDis = _agentSetting.AgentRadius * 2 + _agentSetting.CellSize * 4;

            if (UseKDTree)
                CalcVerticesWithKDTree();
            else 
                CalcVertices();

            CalcEdges(out _edges);
            MergeEdges(ref _edges);
            CalcEdgesAgain(ref _edges);
        }

        #endregion

        #region 从NavMesh生成Edge数据

        private void CalcVertices()
        {
            var vertices = _triangulationData.vertices;
            var indices = _triangulationData.indices;

            Dictionary<Vector3, int> verticesId = new Dictionary<Vector3, int>();
            List<int> father = new List<int>();

            for (int i = 0; i < vertices.Length; ++i)
                father.Add(i);

            for (int i = 0; i < vertices.Length; ++i)
            {
                var v = vertices[i];

                int id;

                if (verticesId.TryGetValue(v, out id))
                {
                    father[i] = id;
                }
                else
                {
                    verticesId.Add(v, i);
                }
            }

            for (int i = 0; i < indices.Length; ++i)
            {
                indices[i] = father[indices[i]];
            }
        }

        private void CalcVerticesWithKDTree()
        {
            var vertices = _triangulationData.vertices;
            var indices = _triangulationData.indices;

            var kdTree = new KDTree(vertices, 8);
            var query = new KDQuery();

            List<int> results = new List<int>();
            List<float> resultDis = new List<float>();
            List<int> father = new List<int>();
            List<int> newIds = new List<int>();
            List<Vector3> newVertices = new List<Vector3>();

            for (int i = 0; i < vertices.Length; ++i)
                father.Add(i);

            for (int i = 0; i < vertices.Length; ++i)
            {
                var fi = Find(i, ref father);

                if (fi != i)
                    continue;

                var v = vertices[i];

                results.Clear();
                resultDis.Clear();

                query.KNearest(kdTree, v, 10, results, resultDis);

                for (int j = resultDis.Count - 1; j >= 0; --j)
                {
                    if (resultDis[j] < eps)
                    {
                        Union(results[j], i, ref father);
                    }
                    else 
                        break;
                }
            }

            for (int i = 0; i < vertices.Length; ++i)
            {
                var fi = Find(i, ref father);

                if (fi == i)
                {
                    newIds.Add(newVertices.Count);
                    newVertices.Add(vertices[i]);
                }
                else
                {
                    newIds.Add(-1);
                }
            }

            for (int i = 0; i < indices.Length; ++i)
            {
                var index = indices[i];
                var fi = Find(index, ref father);

                indices[i] = newIds[fi];
            }

            _triangulationData.vertices = newVertices.ToArray();
        }

        private void CalcEdges(out List<int> edges)
        {
            var indices = _triangulationData.indices;

            var edgeCompare = new EdgeCompare();
            HashSet<KeyValuePair<int, int>> hashSet = new HashSet<KeyValuePair<int, int>>(edgeCompare);
            HashSet<KeyValuePair<int, int>> duplicate = new HashSet<KeyValuePair<int, int>>(edgeCompare);

            for (int i = 0; i < indices.Length; i += 3)
            {
                var p0 = indices[i];
                var p1 = indices[i + 1];
                var p2 = indices[i + 2];

                HandleDuplicateEdge(p0, p1, ref hashSet, ref duplicate);
                HandleDuplicateEdge(p1, p2, ref hashSet, ref duplicate);
                HandleDuplicateEdge(p2, p0, ref hashSet, ref duplicate);
            }

            edges = new List<int>();
            foreach (var pair in hashSet)
            {
                if (EdgeExisted(pair, ref duplicate))
                    continue;

                edges.Add(pair.Key);
                edges.Add(pair.Value);
            }
        }

        private void CalcEdgesAgain(ref List<int> edges)
        {
            var edgeCompare = new EdgeCompare();
            HashSet<KeyValuePair<int, int>> hashSet = new HashSet<KeyValuePair<int, int>>(edgeCompare);
            HashSet<KeyValuePair<int, int>> duplicate = new HashSet<KeyValuePair<int, int>>(edgeCompare);

            for (int i = 0; i < edges.Count; i += 2)
            {
                var p0 = edges[i];
                var p1 = edges[i + 1];

                HandleDuplicateEdge(p0, p1, ref hashSet, ref duplicate);
            }

            edges = new List<int>();
            foreach (var pair in hashSet)
            {
                if (EdgeExisted(pair, ref duplicate))
                    continue;

                edges.Add(pair.Key);
                edges.Add(pair.Value);
            }
        }

        private bool EdgeExisted(KeyValuePair<int, int> pair, ref HashSet<KeyValuePair<int, int>> hashSet)
        {
            return hashSet.Contains(pair);
        }

        private void HandleDuplicateEdge(int p0, int p1, ref HashSet<KeyValuePair<int, int>> hashSet, ref HashSet<KeyValuePair<int, int>> duplicate)
        {
            var pair = new KeyValuePair<int, int>(p0, p1);

            if (EdgeExisted(pair, ref hashSet))
                duplicate.Add(pair);
            else
                hashSet.Add(pair);
        }

        private void MergeEdges(ref List<int> edges)
        {
            var vertices = _triangulationData.vertices;

            List<Dictionary<int, int>> inGraph;
            List<Dictionary<int, int>> outGraph;
            List<List<int>> inNodes;
            List<List<int>> outNodes;

            GenerateGraph(ref edges, out inGraph, out outGraph, out inNodes, out outNodes);

            for (int i = 0; i < vertices.Length; ++i)
            {
                var v = vertices[i];

                for (int inIndex = 1; inIndex <= inNodes[i][0]; ++inIndex)
                {
                    var inNode = inNodes[i][inIndex];
                    var inVec = v - vertices[inNode];
                    inVec = inVec.normalized;

                    for (int outIndex = 0; outIndex <= outNodes[i][0]; ++outIndex)
                    {
                        var outNode = outNodes[i][outIndex];
                        var outVec = vertices[outNode] - v;
                        outVec = outVec.normalized;

                        var diff = (inVec - outVec).sqrMagnitude;

                        if (diff < eps)
                        {
                            RemoveNodeFromGraph(inNode, inGraph[i], inNodes[i]);
                            RemoveNodeFromGraph(outNode, outGraph[i], outNodes[i]);
                            RemoveNodeFromGraph(i, outGraph[inNode], outNodes[inNode]);
                            RemoveNodeFromGraph(i, inGraph[outNode], inNodes[outNode]);

                            AddNodeToGraph(outNode, outGraph[inNode], outNodes[inNode]);
                            AddNodeToGraph(inNode, inGraph[outNode], inNodes[outNode]);
                        }
                    }
                }
            }

            edges.Clear();

            for (int i = 0; i < vertices.Length; ++i)
            {
                for (int outIndex = 1; outIndex <= outNodes[i][0]; ++outIndex)
                {
                    var outNode = outNodes[i][outIndex];
                    
                    edges.Add(i);
                    edges.Add(outNode);
                }
            }
        }

        private void GenerateGraph(ref List<int> edges, out List<Dictionary<int, int>> inGraph, out List<Dictionary<int, int>> outGraph, out List<List<int>> inNodes, out List<List<int>> outNodes)
        {
            var vertices = _triangulationData.vertices;

            inGraph = new List<Dictionary<int, int>>();
            outGraph = new List<Dictionary<int, int>>();
            inNodes = new List<List<int>>();
            outNodes = new List<List<int>>();

            for (int i = 0; i < vertices.Length; ++i)
            {
                inGraph.Add(new Dictionary<int, int>());
                outGraph.Add(new Dictionary<int, int>());
                inNodes.Add(new List<int>{0});
                outNodes.Add(new List<int>{0});
            }

            for (int i = 0; i < edges.Count; i += 2)
            {
                var p0 = edges[i];
                var p1 = edges[i + 1];

                AddNodeToGraph(p0, inGraph[p1], inNodes[p1]);

                AddNodeToGraph(p1, outGraph[p0], outNodes[p0]);
            }
        }

        private void AddNodeToGraph(int point, Dictionary<int, int> graph, List<int> nodes)
        {
            int index;
            if (!graph.TryGetValue(point, out index))
            {
                if (nodes[0] + 1 < nodes.Count)
                {
                    nodes[nodes[0] + 1] = point;
                }
                else
                {
                    nodes.Add(point);
                }
                ++nodes[0];
                graph.Add(point, nodes[0]);
            }
        }

        private void RemoveNodeFromGraph(int point, Dictionary<int, int> graph, List<int> nodes)
        {
            int index;
            if (graph.TryGetValue(point, out index))
            {
                graph.Remove(point);
                nodes[index] = nodes[nodes[0]];
                nodes[nodes[0]] = -1;
                --nodes[0];
            }
            else
            {
                Debug.LogErrorFormat("error remove node from graph at {0}", point);
            }
           
        }

        #endregion

        #region 并查集

        int Find(int x, ref List<int> f)
        {
            return x == f[x] ? x : f[x] = Find(f[x], ref f);
        }

        void Union(int x, int y, ref List<int> f)
        {
            var fx = Find(f[x], ref f);
            var fy = Find(f[y], ref f);
            if (fx != fy)
            {
                f[fx] = fy;
            }
        }

        #endregion

        #region dropdown link

        [FoldoutGroup("操作"), Button("生成dropdown link")]
        private void GenerateDropDownLinks()
        {
            RemoveLinks();

            var total = (int)(_edges.Count * 0.5f);
            for (int i = 0; i < total; ++i)
            {
                GenerateDropDownLinksAlongSelectEdge(i);
            }
        }

        private void GenerateDropDownLinksAlongSelectEdge(int index)
        {
            var vertices = _triangulationData.vertices;

            var start = vertices[_edges[2 * index]];
            var end = vertices[_edges[2 * index + 1]];

            var normal = end - start;
            normal.y = 0;
            normal = new Vector3(-normal.z, 0, normal.x);
            normal = normal.normalized;

            var errorOffset = -_agentSetting.CellSize * normal;
            var mid = (start + end) * 0.5f + errorOffset;

            var radius = _agentSetting.AgentRadius;
            var dropHeight = _agentSetting.LedgeDropHeight;

            var sampleOffset = DropDis * normal;

            var sampleStart = start + sampleOffset + errorOffset;
            var sampleEnd = end + sampleOffset + errorOffset;

            var checkPos = mid + normal * radius * 0.5f;
            var sampleStep = radius * 2f;

            NavMeshHit hit;
            if (NavMesh.SamplePosition(checkPos, out hit, radius * 0.4f - _agentSetting.CellSize, -1))
            {
                return;
            }

            List<Vector3> samplePositions;
            GenerateSamplePositions(sampleStart, sampleEnd, sampleStep, out samplePositions);

            List<LinkInfo> totalLinkInfos;
            GenerateDropDownLinkInfos(radius, dropHeight, sampleOffset, ref samplePositions, out totalLinkInfos);

            if (MergeLink)
            {
                List<LinkInfo> mergedLinkInfos;
                MergeLinks(sampleStep, ref totalLinkInfos, out mergedLinkInfos);

                AddDropDownLinks(ref mergedLinkInfos);
            }
            else
                AddDropDownLinks(ref totalLinkInfos);
        }

        private void AddDropDownLinks(ref List<LinkInfo> dropDownLinkInfos)
        {
            foreach (var info in dropDownLinkInfos)
            {
                AddLink(info, NavMeshLayer.Drop);
            }
        }

        private void GenerateDropDownLinkInfos(float radius, float dropHeight, Vector3 offset, ref List<Vector3> samplePositions, out List<LinkInfo> dropDownLinkInfos)
        {
            dropDownLinkInfos = new List<LinkInfo>();
            for (int i = 0; i < samplePositions.Count; i++)
            {
                var samplePosition = samplePositions[i];
                if(Physics.CheckSphere(samplePosition, radius, -1))
                    continue;

                RaycastHit hit;
                if(!Physics.Raycast(samplePosition, Vector3.down, out hit, dropHeight, -1))
                    continue;

                NavMeshHit navHit;
                if (NavMesh.SamplePosition(hit.point, out navHit, radius, -1))
                {
                    dropDownLinkInfos.Add(new LinkInfo{Start = samplePosition - offset, End = navHit.position});
                }
            }
        }

        private void GenerateSamplePositions(Vector3 start, Vector3 end, float stepSize, out List<Vector3> samplePosition)
        {
            var mid = (start + end) * 0.5f;
            var halfVec = end - mid;
            var unit = halfVec.normalized;
            var halfLength = halfVec.magnitude;

            var maxCount = (int)(halfLength / stepSize);
            samplePosition = new List<Vector3>();

            var step = unit * stepSize;
            var pos =  mid - step * maxCount ;

            maxCount = 2 * maxCount + 1;

            for (int i = 0; i < maxCount; ++i)
            {
                samplePosition.Add(pos);
                pos += step;
            }
        }

        #endregion

        #region manage link

        private void AddLink(Vector3 start, Vector3 end, NavMeshLayer type)
        {
            NavMeshLinkData link = new NavMeshLinkData();

            link.startPosition = start;
            link.endPosition = end;
            link.bidirectional = true;
            link.agentTypeID = 0;
            link.costModifier = -1;

            link.width = 0;
            link.area = (int)type;

            var linkInstance = NavMesh.AddLink(link);

            if(linkInstance.valid)
                _links.Add(linkInstance);
            else
            {
                Debug.LogError("add link failed!");
            }
        }

        private void AddLink(LinkInfo linkInfo, NavMeshLayer type)
        {
            NavMeshLinkData link = new NavMeshLinkData();

            link.startPosition = linkInfo.Start;
            link.endPosition = linkInfo.End;
            link.bidirectional = true;
            link.agentTypeID = 0;
            link.costModifier = -1;

            link.width = linkInfo.Width;
            link.area = (int)type;

            var linkInstance = NavMesh.AddLink(link);
            if (linkInstance.valid)
                _links.Add(linkInstance);
            else
            {
                Debug.LogError("add link failed!");
            }
        }

        [FoldoutGroup("操作"), Button("移除所有link")]
        private void RemoveLinks()
        {
            foreach (var link in _links)
            {
                if (link.valid)
                    NavMesh.RemoveLink(link);
            }

            _links.Clear();
        }

        private void MergeLinks(float stepSize, ref List<LinkInfo> totalLinkInfos, out List<LinkInfo> mergedLinkInfos)
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

            var step = stepSize * dir;

            var linkLeftBound = totalLinkInfos[0];
            var linkRightBound = totalLinkInfos[0];

            for (int i = 1; i < totalLinkInfos.Count; ++i)
            {
                var linkInfo = totalLinkInfos[i];

                var tmpStep = linkInfo.End - linkRightBound.End;
                var diff = (tmpStep - step).sqrMagnitude;

                if (diff <= 0.001f)
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

            mergedLinkInfos.Add(new LinkInfo
            {
                Start = startMid,
                End = endMid,
                Width = width,
            });
        }

        #endregion


        #region Debug

        [FoldoutGroup("调试"), Button("显示调试信息")]
        void ShowDebugInfo()
        {
            _showDebugInfo = !_showDebugInfo;
        }

        private void DrawEdge(int index, Color color)
        {
            var vertices = _triangulationData.vertices;

            var p0 = vertices[_edges[2 * index]];
            var p1 = vertices[_edges[2 * index + 1]];

            Debug.DrawLine(p0, p1, color);
            Handles.Label(p0, "s: " +  _edges[2 * index].ToString(), SirenixGUIStyles.BlackLabel);
            Handles.Label(p1, "e: " + _edges[2 * index + 1].ToString(), SirenixGUIStyles.BlackLabel);
        }

        private void DrawTriangulation(int index, Color color, bool showLabel = true)
        {
            var vertices = _triangulationData.vertices;
            var indices = _triangulationData.indices;
            var i = 3 * index;

            var p0 = vertices[indices[i]];
            var p1 = vertices[indices[i + 1]];
            var p2 = vertices[indices[i + 2]];

            Debug.DrawLine(p0, p1, color);
            Debug.DrawLine(p1, p2, color);
            Debug.DrawLine(p2, p0, color);

            if (showLabel)
            {
                Handles.Label(p0, indices[i].ToString());
                Handles.Label(p1, indices[i + 1].ToString());
                Handles.Label(p2, indices[i + 2].ToString());
            }
        }

        private void DrawAllTriangulation()
        {
            var indices = _triangulationData.indices;
            for (int i = 0; i < indices.Length / 3; ++i)
            {
                DrawTriangulation(i, Color.yellow, false);
            }
        }

        private void DrawEdges()
        {
            var vertices = _triangulationData.vertices;
            for (int i = 0; i < _edges.Count; i += 2)
            {
                var p0 = vertices[_edges[i]];
                var p1 = vertices[_edges[i + 1]];
                Debug.DrawLine(p0, p1, Color.red);
            }
        }

        #endregion
    }

    public class EdgeCompare : IEqualityComparer<KeyValuePair<int, int>>
    {
        public bool Equals(KeyValuePair<int, int> x, KeyValuePair<int, int> y)
        {
            return (x.Key == y.Key && x.Value == y.Value) || (x.Key == y.Value && x.Value == y.Key);
        }

        public int GetHashCode(KeyValuePair<int, int> obj)
        {
            unchecked
            {
                return ((obj.Key * 397) ^ obj.Value) + ((obj.Value * 397) ^ obj.Key);
            }
        }
    }

    public struct LinkInfo
    {
        public Vector3 Start;
        public Vector3 End;
        public float Width;
    }
}