using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using UnityEditor;
using UnityEngine;
using UnityEngine.LowLevel;

namespace Editor
{
    [InitializeOnLoad]
    public static class RapierEntry
    {
        static RapierEntry()
        {
            // Check if the override is enabled
            if (!EditorPrefs.GetBool("RapierPhysicsOverrideEnabled", true))
                return;
            
            if (GetSelectedPhysicSDKInfoProperty() is {} idProp)
            {
                // Set the physics backend to 'Rapier' (by setting the id to the null physics backend id)
                if (idProp.uintValue != 3737844653)
                {
                    idProp.uintValue = 3737844653;
                    idProp.serializedObject.ApplyModifiedProperties();
                    Debug.LogWarning("Physics backend set to 'None' as Rapier is enabled.\n<b><color=\"#ff00ffff\">Please restart the editor for changes to take effect.</color></b>");
                }
            }
        }

        static IReadOnlyCollection<(string Name, uint Id)> GetPhysicSDKInfos()
        {
            var returnVal = new List<(string Name, uint Id)>();
            
            var getIntegrationInfos = typeof(Physics).GetMethods(BindingFlags.NonPublic|BindingFlags.Static).First(m=> m.GetParameters().Length == 2 && m.Name=="GetIntegrationInfos");
            var args = new object[] { null, null };
            getIntegrationInfos.Invoke(null, args);
            var integrationType = Type.GetType("UnityEngine.IntegrationInfo, UnityEngine.PhysicsModule");
            var integrationInfos = (IntPtr)args[0];
            var integrationCount = (ulong)args[1];
            for (var i = 0; i < (int)integrationCount; i++)
            {
                var integrationInfo = Marshal.PtrToStructure(integrationInfos, integrationType);
                var id = integrationType.GetField("Id").GetValue(integrationInfo);
                var name = integrationType.GetProperty("Name").GetValue(integrationInfo);
                returnVal.Add(((string)name, (uint)id));
                integrationInfos += Marshal.SizeOf(integrationType);
            }
            
            return returnVal;
        }

        static SerializedProperty GetSelectedPhysicSDKInfoProperty()
        {
            var found = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/DynamicsManager.asset");
            var serializedObject = found is not null ? new SerializedObject(found[0]) : null;
            return serializedObject?.FindProperty("m_CurrentBackendId");
        }
    }
}
