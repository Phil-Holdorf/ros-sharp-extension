using RosSharp.RosBridgeClient;
using RosSharpExtension;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;

internal class MessageImportEditorPrefs
{
    public void DeleteEditorPrefs() {
        EditorPrefs.DeleteKey("UrdfImporterProtocolNumber");
        EditorPrefs.DeleteKey("UrdfImporterAddress");
    }
    public void GetEditorPrefs(out Protocol protocolType, out string address) {
        protocolType = (Protocol)(EditorPrefs.HasKey("UrdfImporterProtocolNumber") ?
            EditorPrefs.GetInt("UrdfImporterProtocolNumber") : 1);

        address = (EditorPrefs.HasKey("UrdfImporterAddress") ?
            EditorPrefs.GetString("UrdfImporterAddress") :
            "ws://192.168.0.1:9090");
    }
    public void SetEditorPrefs(Protocol protocolType, string address) {
        EditorPrefs.SetInt("UrdfImporterProtocol", protocolType.GetHashCode());
        EditorPrefs.SetString("UrdfImporterAddress", address);
    }
}
