using UnityEditor;

namespace LinkBake
{
    public class NavMeshAgentBuildSetting
    {
        private SerializedObject m_navMeshProjectSettingsObject;
        private int agentIndex;
        private SerializedProperty m_agentName;
        private SerializedProperty m_agentRadius;
        private SerializedProperty m_agentHeight;
        private SerializedProperty m_agentStepHeight;
        private SerializedProperty m_agentMaxSlope;
        private SerializedProperty m_ledgeDropHeight;
        private SerializedProperty m_cellSize;

        public NavMeshAgentBuildSetting(SerializedObject navMeshProjectSettingsObject, int agentIndex)
        {
            this.m_navMeshProjectSettingsObject = navMeshProjectSettingsObject;
            this.agentIndex = agentIndex;
            var _agents = m_navMeshProjectSettingsObject.FindProperty("m_Settings");
            var _settingNames = m_navMeshProjectSettingsObject.FindProperty("m_SettingNames");
            var agent = _agents.GetArrayElementAtIndex(agentIndex);
            m_agentName = _settingNames.GetArrayElementAtIndex(agentIndex);
            m_agentRadius = agent.FindPropertyRelative("agentRadius");
            m_agentHeight = agent.FindPropertyRelative("agentHeight");
            m_agentStepHeight = agent.FindPropertyRelative("agentClimb");
            m_agentMaxSlope = agent.FindPropertyRelative("agentSlope");
            m_ledgeDropHeight = agent.FindPropertyRelative("ledgeDropHeight");
            m_cellSize = agent.FindPropertyRelative("cellSize");
        }
        public int AgentIndex
        {
            get { return agentIndex; }
        }
        public string AgentName
        {
            get { return m_agentName.stringValue; }
            set
            {
                m_agentName.stringValue = value;
                m_navMeshProjectSettingsObject.ApplyModifiedProperties();
            }
        }
        public float AgentRadius
        {
            get { return m_agentRadius.floatValue; }
            set
            {
                m_agentRadius.floatValue = value;
                m_navMeshProjectSettingsObject.ApplyModifiedProperties();
            }
        }
        public float AgentHeight
        {
            get { return m_agentHeight.floatValue; }
            set
            {
                m_agentHeight.floatValue = value;
                m_navMeshProjectSettingsObject.ApplyModifiedProperties();
            }
        }
        public float AgentStepHeight
        {
            get { return m_agentStepHeight.floatValue; }
            set
            {
                m_agentStepHeight.floatValue = value;
                m_navMeshProjectSettingsObject.ApplyModifiedProperties();
            }
        }
        public float AgentMaxSlope
        {
            get { return m_agentMaxSlope.floatValue; }
            set
            {
                m_agentMaxSlope.floatValue = value;
                m_navMeshProjectSettingsObject.ApplyModifiedProperties();
            }
        }

        public float LedgeDropHeight
        {
            get { return m_ledgeDropHeight.floatValue; }
            set
            {
                m_ledgeDropHeight.floatValue = value;
                m_navMeshProjectSettingsObject.ApplyModifiedProperties();
            }
        }

        public float CellSize
        {
            get { return m_cellSize.floatValue; }
            set
            {
                m_cellSize.floatValue = value;
                m_navMeshProjectSettingsObject.ApplyModifiedProperties();
            }
        }
    }
}
