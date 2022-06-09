using DataStructures.ViliWonka.KDTree;
using LinkUtils;
using Sirenix.OdinInspector;
using Sirenix.OdinInspector.Editor;
using Sirenix.Utilities.Editor;
using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.AI;

namespace LinkBake
{
    [ExecuteInEditMode]
    public class BakeLink : OdinEditorWindow
    {
        [FoldoutGroup("参数"), LabelText("DropDown距离")]
        public float DropDis = 0;

        [FoldoutGroup("参数"), LabelText("DropDown高度")]
        public float DropHeight = 4;

        [FoldoutGroup("参数"), LabelText("浮点误差")]
        public float Eps = 0.0001f;

        [FoldoutGroup("参数"), LabelText("是否合并link")]
        public bool MergeLink = false;

        [FoldoutGroup("参数"), LabelText("是否使用KDTree")]
        public bool UseKDTree = false;

        [FoldoutGroup("参数"), LabelText("需要link区域的采样点集")]
        public List<Vector3> SampleRoot;

        [FoldoutGroup("调试"), LabelText("显示所有三角面")]
        public bool ShowAllTriangulations = false;

        [FoldoutGroup("调试"), LabelText("指定边id")]
        public int EdgeId = 0;

        [FoldoutGroup("调试"), LabelText("显示指定边")]
        public bool ShowSelectedEdge = false;

        [FoldoutGroup("调试"), LabelText("显示指定边起点的所有出边")]
        public bool ShowAllEdgesFromStart = false;

        [FoldoutGroup("调试"), LabelText("显示所有边")]
        public bool ShowAllEdges = true;

        [FoldoutGroup("调试"), LabelText("是否实时更新link")]
        public bool RealTimeUpdateLinks = false;

        [FoldoutGroup("调试"), LabelText("当前选中点")]
        public Vector3 SelectPos;

        private NavMeshAgentBuildSetting _agentSetting;
        private NavMeshTriangulation _triangulationData;
        private List<NavMeshLinkInstance> _links;
        private List<LinkInfo> _linkInfos;
        private List<int> _edges;
        private Vector3[] _vertices;
        private int[] _indices;

        private NavMeshPath _path;

        private bool _showDebugInfo = false;
        private static BakeLink _instance;
        private LinkBakeDrawHelper _drawHelper;
        private GameObject _drawHelperGo;


        [MenuItem("Tools/link烘焙工具")]
        public static void OpenWindow()
        {
            var window = GetWindow<BakeLink>("link烘焙工具", typeof(BakeLink));
            _instance = window;
            _instance.Init();
        }

        void Update()
        {
            EdgeId = Mathf.Clamp(EdgeId, 0, _edges.Count / 2 - 1);
            try
            {
                if(_triangulationData.indices!=null && RealTimeUpdateLinks)
                    GenerateDropDownLinks();
            }
            catch (NullReferenceException e)
            {
                Debug.LogWarning(e.Message);
            }
        }

        public void OnDrawGizmos()
        {
            if (_showDebugInfo)
            {
                if (ShowAllTriangulations)
                    DrawAllTriangulation();
                if (ShowAllEdges)
                    DrawEdges();
                if (ShowAllEdgesFromStart)
                    DrawAllEdgesFromSelectEdgeStartPoint(EdgeId, Color.green);
                if (ShowSelectedEdge)
                {
                    DrawEdge(EdgeId, Color.blue);
                    RemoveLinks();
                    GenerateDropDownLinksAlongSelectEdge(EdgeId);
                }

                DrawAllSampleRoot(ColorUtils.orange);
                DrawSelectPos(Color.red);
            }

            SceneView.RepaintAll();
        }

        #region 初始化

        [FoldoutGroup("操作"), Button("初始化")]
        private void Init()
        {
            RemoveLinks();
            InitAgentSetting();
            InitTriangulationData();
            InitDataStructure();
            InitDraw();
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
            _vertices = _triangulationData.vertices;
            _indices = _triangulationData.indices;

            _triangulationData = NavMesh.CalculateTriangulation();
        }

        private void InitDataStructure()
        {
            _links = _links?? new List<NavMeshLinkInstance>();
            _links.Clear();

            _linkInfos = _linkInfos ?? new List<LinkInfo>();
            _linkInfos.Clear();

            SampleRoot = SampleRoot ?? new List<Vector3>();
            SampleRoot.Clear();

            _path = new NavMeshPath();

            DropDis = _agentSetting.AgentRadius * 2 + _agentSetting.CellSize * 4;

            if (UseKDTree)
                CalcVerticesWithKdTree();
            else 
                CalcVertices();

            FilterVerticesWithPhysics();
            CalcEdges(out _edges);
            MergeEdges(ref _edges);
            CalcEdgesAgain(ref _edges);
        }

        private void InitDraw()
        {
            _drawHelperGo = GetDrawHelper("linkBakeHelper");
            _drawHelper = _drawHelperGo.GetComponent<LinkBakeDrawHelper>();
            if (_drawHelper == null)
                _drawHelper = _drawHelperGo.AddComponent<LinkBakeDrawHelper>();

            _instance = this;
            _drawHelper.Init(_instance);

            SceneView.onSceneGUIDelegate -= OnSceneGUI;
            SceneView.onSceneGUIDelegate += OnSceneGUI;
        }

        private GameObject GetDrawHelper(string findObjName)
        {
            var scene = EditorSceneManager.GetActiveScene();
            GameObject[] objects = scene.GetRootGameObjects();

            for (int j = 0; j < objects.Length; j++)
            {
                if (objects[j].name == findObjName)
                {
                    return objects[j];
                }
            }

            return new GameObject(findObjName);
        }

        #endregion

        #region 从NavMesh生成Edge数据

        private void CalcVertices()
        {
            var vertices = _vertices;
            var indices = _indices;

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

        private void CalcVerticesWithKdTree()
        {
            var vertices = _vertices;
            var indices = _indices;

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
                    if (resultDis[j] < Eps)
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

            _vertices = newVertices.ToArray();
        }

        private void CalcEdges(out List<int> edges)
        {
            var indices = _indices;

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

        private void FilterVerticesWithPhysics()
        {
            var vertices = _vertices;
            var indices = _indices;

            var radius = 0.01f;

            QuickHashList<int> filteredVertices = new QuickHashList<int>();
            List<int> newIndices = new List<int>();
            List<Vector3> newVertices = new List<Vector3>();

            for (int i = 0; i < vertices.Length; ++i)
            {
                var v = vertices[i];

                var pos = v + radius * 1.1f * Vector3.up;

                if(Physics.CheckSphere(pos, radius, -1))
                    continue;

                filteredVertices.Add(i);
            }

            for (int i = 0; i < indices.Length; i += 3)
            {
                var p0 = indices[i];
                var p1 = indices[i + 1];
                var p2 = indices[i + 2];

                var np0 = filteredVertices.Find(p0);
                var np1 = filteredVertices.Find(p1);
                var np2 = filteredVertices.Find(p2);

                if (np0 >= 0 && np1 >= 0 && np2 >= 0)
                {
                    newIndices.Add(np0);
                    newIndices.Add(np1);
                    newIndices.Add(np2);
                }
            }

            for (int i = 0; i < filteredVertices.Count; ++i)
            {
                var index = filteredVertices[i];

                newVertices.Add(vertices[index]);
            }

            _vertices = newVertices.ToArray();
            _indices = newIndices.ToArray();
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
            var vertices = _vertices;

            List<QuickHashList<int>> inGraph;
            List<QuickHashList<int>> outGraph;

            GenerateGraph(ref edges, out inGraph, out outGraph);

            for (int i = 0; i < vertices.Length; ++i)
            {
                var v = vertices[i];

                for (int inIndex = 0; inIndex < inGraph[i].Count; ++inIndex)
                {
                    var inNode = inGraph[i][inIndex];
                    var inVec = v - vertices[inNode];
                    inVec = inVec.normalized;

                    for (int outIndex = 0; outIndex < outGraph[i].Count; ++outIndex)
                    {
                        var outNode = outGraph[i][outIndex];
                        var outVec = vertices[outNode] - v;
                        outVec = outVec.normalized;

                        var diff = (inVec - outVec).sqrMagnitude;

                        if (diff < Eps)
                        {
                            inGraph[i].Remove(inNode);
                            outGraph[i].Remove(outNode);

                            outGraph[inNode].Remove(i);
                            inGraph[outNode].Remove(i);

                            outGraph[inNode].Add(outNode);
                            inGraph[outNode].Add(inNode);
                        }
                    }
                }
            }

            edges.Clear();

            for (int i = 0; i < vertices.Length; ++i)
            {
                for (int outIndex = 0; outIndex < outGraph[i].Count; ++outIndex)
                {
                    var outNode = outGraph[i][outIndex];
                    
                    edges.Add(i);
                    edges.Add(outNode);
                }
            }
        }

        private void GenerateGraph(ref List<int> edges, out List<QuickHashList<int>> inGraph, out List<QuickHashList<int>> outGraph)
        {
            var vertices = _vertices;

            inGraph = new List<QuickHashList<int>>();
            outGraph = new List<QuickHashList<int>>();

            for (int i = 0; i < vertices.Length; ++i)
            {
                inGraph.Add(new QuickHashList<int>());
                outGraph.Add(new QuickHashList<int>());
            }

            for (int i = 0; i < edges.Count; i += 2)
            {
                var p0 = edges[i];
                var p1 = edges[i + 1];

                inGraph[p1].Add(p0);
                outGraph[p0].Add(p1);
            }
        }

        #endregion

        #region 并查集

        int Find(int x, ref List<int> f)
        {
            return x == f[x] ? x : f[x] = Find(f[x], ref f);
        }

        /// <summary>
        /// 合并x和y的并查集, y的根节点将成为新的根节点
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="f"></param>
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
            var vertices = _vertices;

            var start = vertices[_edges[2 * index]];
            var end = vertices[_edges[2 * index + 1]];

            NavMeshHit hit;

            var normal = end - start;
            normal.y = 0;
            normal = new Vector3(-normal.z, 0, normal.x);
            normal = normal.normalized;

            var cellErrorOffset = _agentSetting.CellSize;
            var mid = (start + end) * 0.5f;

            var radius = _agentSetting.AgentRadius;
            var checkPos = mid + normal * radius * 0.5f;
            var checkRadius = Mathf.Max(_agentSetting.CellSize, radius * 0.4f - _agentSetting.CellSize);
            var dropHeight = DropHeight;

            if (!NavMesh.SamplePosition(mid, out hit, checkRadius, -1))
            {
                return;
            }

            var errorOffset = hit.position - mid;
            mid = hit.position;
            start += errorOffset;
            end += errorOffset;

            var sampleOffset = DropDis * normal;

            var sampleStart = start + sampleOffset;
            var sampleEnd = end + sampleOffset;
            var sampleStep = radius * 2f;

            if (NavMesh.SamplePosition(checkPos, out hit, checkRadius, -1))
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
                AddLink(info, (int)NavmeshLayer.JumpOffWall);
                _linkInfos.Add(info);
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

        private void AddLink(Vector3 start, Vector3 end, int type)
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

        private void AddLink(LinkInfo linkInfo, int type)
        {
            NavMeshLinkData link = new NavMeshLinkData();

            link.startPosition = linkInfo.Start;
            link.endPosition = linkInfo.End;
            link.bidirectional = true;
            link.agentTypeID = 0;
            link.costModifier = -1;

            link.width = linkInfo.Width;
            link.area = type;

            var linkInstance = NavMesh.AddLink(link, linkInfo.Origin, linkInfo.Rotation);
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
            if (_links == null)
                return;

            foreach (var link in _links)
            {
                if (link.valid)
                    NavMesh.RemoveLink(link);
            }

            _links.Clear();
            _linkInfos.Clear();
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

                if (diff < Eps)
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

        #endregion

        #region 剔除不联通区域

        [FoldoutGroup("操作"), Button("将当前选中点添加到link采样点集")]
        void AddSelectPosToSampleSet()
        {
            NavMeshHit hit;
            if (!NavMesh.SamplePosition(SelectPos, out hit, 0.1f, -1))
            {
                Debug.LogWarningFormat("select pos {0} invalid!", SelectPos);
                return;
            }

            SampleRoot.Add(hit.position);
        }

        [FoldoutGroup("操作"), Button("剔除额外区域")]
        private void CullUseless()
        {
            CullVertices();
            GenerateDropDownLinks();
        }

        private void CullVertices()
        {
            if(SampleRoot.Count <= 0)
                return;

            var vertices = _vertices;

            List<int> father = new List<int>();
            for (int i = 0; i < vertices.Length; i++)
            {
                father.Add(i);
            }

            var rootOffset = father.Count;

            father.Add(rootOffset);
            for (int i = 1; i < SampleRoot.Count; ++i)
            {
                father.Add(rootOffset + i);
                Union(rootOffset + i - 1, rootOffset + i, ref father);
            }

            //按边的连接关系构建一遍并查集,缩小采样目标集合大小
            for (int i = 0; i < _edges.Count; i += 2)
            {
                var p0 = _edges[i];
                var p1 = _edges[i + 1];

                Union(p0, p1, ref father);
            }

            for (int i = 0; i < vertices.Length; ++i)
            {
                var fa = Find(i, ref father);
                if(fa != i)
                    continue;

                var p0 = vertices[i];

                for (int j = 0; j < SampleRoot.Count; ++j)
                {
                    var rootIndex = rootOffset + j;
                    var p1 = SampleRoot[j];

                    var hasPath = HasPath(p0, p1);

                    if (hasPath)
                    {
                        Union(i, rootIndex, ref father);
                        break;
                    }
                }
            }

            QuickHashList<int> usefulVerticeIndex = new QuickHashList<int>();
            List<int> usefulEdges = new List<int>();

            for (int i = 0; i < vertices.Length; ++i)
            {
                var fa = Find(i, ref father);

                if (fa >= rootOffset)
                {
                    usefulVerticeIndex.Add(i);
                }
            }

            for (int i = 0; i < _edges.Count; i+=2)
            {
                var p0 = _edges[i];
                var p1 = _edges[i + 1];

                var np0 = usefulVerticeIndex.Find(p0);
                var np1 = usefulVerticeIndex.Find(p1);

                if (np0 >= 0 && np1 >= 0)
                {
                    usefulEdges.Add(np0);
                    usefulEdges.Add(np1);
                }
            }

            _edges = usefulEdges;

            Vector3[] usefulVertices = new Vector3[usefulVerticeIndex.Count];
            for (int i = 0; i < usefulVerticeIndex.Count; i++)
            {
                var v = vertices[usefulVerticeIndex[i]];
                usefulVertices[i] = v;
            }

            _vertices = usefulVertices;
        }

        private bool HasPath(Vector3 p0, Vector3 p1)
        {
            var hasPath = NavMesh.CalculatePath(p0, p1, -1, _path)
                          && _path.corners.Length > 1
                          && ((p1 - _path.corners[_path.corners.Length - 1]).sqrMagnitude < _agentSetting.CellSize);

            return hasPath;
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
            var vertices = _vertices;

            var p0 = vertices[_edges[2 * index]];
            var p1 = vertices[_edges[2 * index + 1]];

            Gizmos.color = color;
            Gizmos.DrawLine(p0, p1);

            GUI.color = Color.black;
            Handles.Label(p0, "s: " +  _edges[2 * index].ToString(), SirenixGUIStyles.BoldLabel);
            Handles.Label(p1, "e: " + _edges[2 * index + 1].ToString(), SirenixGUIStyles.BoldLabel);

            var vec = (p1 - p0).normalized;
            Handles.Label((p0+p1)*0.5f, "normal: " + vec.ToString(), SirenixGUIStyles.BoldLabel);

            GUI.color = Color.white;
        }

        private void DrawAllEdgesFromSelectEdgeStartPoint(int index, Color color)
        {
            var start = _edges[2 * index];
            var bound = (int)(_edges.Count * 0.5f);
            for (int i = 0; i < bound; ++i)
            {
                var p0 = _edges[2 * i];
                var p1 = _edges[2 * i + 1];
                if(p0 != start && p1 != start)
                    continue;

                DrawEdge(i, color);
            }
        }

        private void DrawTriangulation(int index, Color color, bool showLabel = true)
        {
            var vertices = _triangulationData.vertices;
            var indices = _triangulationData.indices;
            var i = 3 * index;

            var p0 = vertices[indices[i]];
            var p1 = vertices[indices[i + 1]];
            var p2 = vertices[indices[i + 2]];

            Gizmos.color = color;
            Gizmos.DrawLine(p0, p1);
            Gizmos.DrawLine(p1, p2);
            Gizmos.DrawLine(p2, p0);

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
            var vertices = _vertices;
            for (int i = 0; i < _edges.Count; i += 2)
            {
                var p0 = vertices[_edges[i]];
                var p1 = vertices[_edges[i + 1]];

                Gizmos.color = Color.red;
                Gizmos.DrawLine(p0, p1);
            }
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

        private void DrawAllSampleRoot(Color color)
        {
            Gizmos.color = color;
            foreach (var pos in SampleRoot)
            {
                Gizmos.DrawSphere(pos, 0.5f);
            }
        }

        private void DrawSelectPos(Color color)
        {
            Gizmos.color = color;
            Gizmos.DrawSphere(SelectPos, 0.5f);
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
        public Vector3 Origin;
        public Quaternion Rotation;
        public float Width;
    }
}