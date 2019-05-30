/* MIT License

Copyright (c) 2019 Phil-Holdorf

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace RosSharpExtension {
    /// <summary>
    /// Handles layout and rendering only. User input from this window is delegated to the <see cref="MessageImportHandler"/> and <see cref="MessageImportResultsHandler"/>
    /// </summary>
    internal class MessageImportEditorWindow : EditorWindow {
        private MessageImportHandler importHandler;
        private MessageImportResultsHandler resultHandler;
        private Vector2 scrollPosition;

        [MenuItem("RosBridgeClient/Transfer Message files from ROS...")]
        private static void Init() {
            MessageImportEditorWindow editorWindow = GetWindow<MessageImportEditorWindow>();
            editorWindow.minSize = new Vector2(500, 300);

            editorWindow.Show();
        }

        private void Awake() {
            resultHandler = new MessageImportResultsHandler();
            importHandler = new MessageImportHandler(resultHandler);
        }

        private void OnGUI() {
            CreateSetupComponents();
            CreateResultsPage();
            CreateMessageGeneratingComponents();

            CheckForError();
            CheckForRefresh();
        }

        private void CheckForError() {
            if (importHandler.StatusEvents["connectionFailed"].WaitOne(0)) {
                importHandler.StatusEvents["connectionFailed"].Reset();
                EditorUtility.DisplayDialog("Message Import Status",
                    "Could not connect to ROS or a required ROS service.\n\n" + 
                    "Make sure to run the included launch file.",
                    "OK");
            }

        }

        private void CheckForRefresh() {
            if (importHandler.StatusEvents["generationComplete"].WaitOne(0)) {
                importHandler.StatusEvents["generationComplete"].Reset();
                AssetDatabase.Refresh();
                Close();
                EditorUtility.DisplayDialog("Message Import Status", "Message generation complete", "OK");
            }
        }

        private void CreateSetupComponents() {
            GUILayout.Label("Transfer Message files from ROS", EditorStyles.boldLabel);
            CreateAddressComponents();
            CreateProtocolComponents();
            CreateOverwriteToggle();
            CreateLoadButtons();
        }

        private void CreateAddressComponents() {
            EditorGUILayout.BeginHorizontal();
            importHandler.Address = EditorGUILayout.TextField("Address", importHandler.Address);
            EditorGUILayout.EndHorizontal();
        }

        private void CreateProtocolComponents() {
            EditorGUILayout.BeginHorizontal();
            importHandler.ProtocolType = (Protocol)EditorGUILayout.EnumPopup("Protocol", importHandler.ProtocolType);
            EditorGUILayout.EndHorizontal();
        }

        private void CreateOverwriteToggle() {
            EditorGUILayout.BeginHorizontal();
            importHandler.OverwriteMessages = GUILayout.Toggle(importHandler.OverwriteMessages, "Overwrite existing files");
            EditorGUILayout.EndHorizontal();
        }

        private void CreateLoadButtons() {
            GUILayout.Space(20);

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.BeginVertical();
            if (GUILayout.Button("Display all messages")) {
                importHandler.LoadAllMessagesAsync();
            }
            if (GUILayout.Button("Display single package:")) {
                importHandler.LoadMessagesInPackageAsync();
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.BeginVertical();
            if (GUILayout.Button("Display messages from active topics")) {
                importHandler.LoadMessagesFromTopicsAsync();
            }
            
            importHandler.Package = EditorGUILayout.TextField("std_msgs");
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();
        }

        private void CreateResultsPage() {
            GUILayout.Space(20);
            GUILayout.Label("Results", EditorStyles.boldLabel);
            scrollPosition = GUILayout.BeginScrollView(scrollPosition);

            CreateSelectAllToggle();
            if (importHandler.StatusEvents["loadingComplete"].WaitOne(0)) {
                if (resultHandler.MessageHierarchy.Keys.Count == 0) {
                    CreateNoResultsMessage();
                }
                foreach (string packageName in resultHandler.MessageHierarchy.Keys) {
                    CreateComponentsAtPackageLevel(packageName);
                    CreateComponentsInsidePackage(packageName);
                }
            } else if (importHandler.StatusEvents["loadingStarted"].WaitOne(0)){
                CreateLoadingMessage();
            }

            GUILayout.EndScrollView();
        }

        private void CreateLoadingMessage() {
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Loading in progress...");
            EditorGUILayout.EndHorizontal();
        }

        private void CreateNoResultsMessage() {
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("No results found.");
            EditorGUILayout.EndHorizontal();
        }

        private void CreateSelectAllToggle() {
            if (resultHandler.CheckedMessages.Keys.Count > 0) {
                bool newState = GUILayout.Toggle(resultHandler.ToggleSelectAll, "Select All");
                if (newState != resultHandler.ToggleSelectAll) {
                    resultHandler.MarkAll(newState);
                }
                resultHandler.ToggleSelectAll = newState;
                resultHandler.SelectAllToggleMatchChildren();
            }
        }
        private void CreateComponentsAtPackageLevel(string packageName) {
            EditorGUILayout.BeginHorizontal();
            List<string> messagesInThisPackage = resultHandler.MessageHierarchy[packageName];
            if (messagesInThisPackage.Count == 1) {
                string messageName = resultHandler.MessageHierarchy[packageName][0];
                string text = packageName + "/" + messageName;
                
                bool newState = GUILayout.Toggle(resultHandler.CheckedMessages[messageName], GUIContent.none, GUILayout.ExpandWidth(false));
                GUILayout.Label(text);
                resultHandler.CheckedMessages[messageName] = newState;
            }
            else {
                bool newState = GUILayout.Toggle(resultHandler.CheckedPackages[packageName], GUIContent.none, GUILayout.ExpandWidth(false));
                if (newState != resultHandler.CheckedPackages[packageName]) {
                    resultHandler.MarkAllInPackage(newState, packageName);
                }
                resultHandler.CheckedPackages[packageName] = newState;
                resultHandler.PackageToggleMatchChildren(packageName);
                resultHandler.OpenedPackages[packageName] = EditorGUILayout.Foldout(resultHandler.OpenedPackages[packageName], packageName, true);
            }
            EditorGUILayout.EndHorizontal();
        }

        private void CreateComponentsInsidePackage(string packageName) {
            if (resultHandler.OpenedPackages[packageName]) {
                for (int i = 0; i < resultHandler.MessageHierarchy[packageName].Count; i++) {
                    EditorGUILayout.BeginHorizontal();
                    GUILayout.Space(20);
                    string messageName = resultHandler.MessageHierarchy[packageName][i];
                    bool newState = GUILayout.Toggle(resultHandler.CheckedMessages[messageName], messageName);
                    resultHandler.CheckedMessages[messageName] = newState;
                    EditorGUILayout.EndHorizontal();
                }
            }
        }

        private void CreateMessageGeneratingComponents() {
            if (resultHandler.CheckedMessages.Keys.Count == 0) {
                return;
            }
            GUILayout.Space(20);
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Generate message files in Assets folder")) {
                importHandler.TransferMessagesAsync();
            }
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.EndHorizontal();
            if (importHandler.StatusEvents["transferStarted"].WaitOne(0) && !importHandler.StatusEvents["generationComplete"].WaitOne(0)) {
                GUILayout.Label("Message generation in progress...");
            } else {
                GUILayout.Label("");
            }
        }

        private void OnFocus() {
            importHandler?.GetEditorPrefs();
        }

        private void OnLostFocus() {
            importHandler?.SetEditorPrefs();
        }

        private void OnDestroy() {
            importHandler?.SetEditorPrefs();
        }
    }
}
