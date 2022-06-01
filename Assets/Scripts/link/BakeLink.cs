using System.Collections.Generic;
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
        [FoldoutGroup("调试", Order = 1), LabelText("显示所有三角面")]
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
                    GenerateDropDownLinksAlongSelectEdge(EdgeId);
                }
            }
        }

        #region 初始化

        [Button("初始化")]
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

            CalcVertices();
            CalcEdges(out _edges);
        }

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

        #endregion

        #region dropdown link

        private void GenerateDropDownLinks()
        {

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

            var mid = (start + end) * 0.5f;

            var radius = _agentSetting.AgentRadius;
            var dropHeight = _agentSetting.LedgeDropHeight;
            var cellSize = _agentSetting.CellSize;

            var sampleOffset = radius * 2f + cellSize * 4f;

            var sampleStart = start + normal * sampleOffset;
            var sampleEnd = end + normal * sampleOffset;

            var checkPos = mid + normal * radius * 0.5f;

            Debug.DrawLine(mid, checkPos, Color.green);

            NavMeshHit hit;
            if (NavMesh.SamplePosition(checkPos, out hit, radius * 0.4f, -1))
            {
                return;
            }

            Debug.DrawLine(checkPos, checkPos + normal * radius * 0.4f, Color.red);

            List<Vector3> samplePositions;
            GenerateSamplePositions(sampleStart, sampleEnd, radius * 2f, out samplePositions);

            foreach (var samplePosition in samplePositions)
            {
                Debug.DrawLine(samplePosition, samplePosition + Vector3.up, Color.blue);
            }

            List<Vector3> dropDownPositions;
            GenerateDropDownPositions(radius, dropHeight, ref samplePositions, out dropDownPositions);

            foreach (var dropDownPosition in dropDownPositions)
            {
                Debug.DrawLine(dropDownPosition, dropDownPosition + normal * radius, Color.green);
            }

        }

        private void GenerateDropDownPositions(float radius, float dropHeight, ref List<Vector3> samplePositions, out List<Vector3> dropDownPositions)
        {
            dropDownPositions = new List<Vector3>();
            foreach (var samplePosition in samplePositions)
            {
                if(Physics.CheckSphere(samplePosition, radius, -1))
                    continue;

                RaycastHit hit;
                if(!Physics.Raycast(samplePosition, Vector3.down, out hit, dropHeight, -1))
                    continue;

                NavMeshHit navHit;
                if (NavMesh.SamplePosition(hit.point, out navHit, radius, -1))
                {
                    dropDownPositions.Add(navHit.position);
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

        private void RemoveLinks()
        {
            foreach (var link in _links)
            {
                if (link.valid)
                    link.Remove();
            }

            _links.Clear();
        }

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
            Handles.Label(p0, _edges[2 * index].ToString(), SirenixGUIStyles.BlackLabel);
            Handles.Label(p1, _edges[2 * index + 1].ToString(), SirenixGUIStyles.BlackLabel);
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
}