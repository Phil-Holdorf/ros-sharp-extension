using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RosSharpExtension {
    internal class MessageImportResultsHandler {
        public bool ToggleSelectAll { get; set; }
        public SortedDictionary<string, List<string>> MessageHierarchy { get; private set; }
        public Dictionary<string, bool> OpenedPackages { get; private set; }
        public Dictionary<string, bool> CheckedMessages { get; private set; }
        public Dictionary<string, bool> CheckedPackages { get; private set; }

        public MessageImportResultsHandler() {
            ResetImportResults();
        }

        /// <summary>
        /// create message hierarchy to be displayed in the UI
        /// </summary>
        public void OnMessagesLoaded(string[] messages) {
            ResetImportResults();
            for (int i = 0; i < messages.Length; i++) {
                new MessageNameSplitter().Split(messages[i], out string packageName, out string messageName);

                if (!MessageHierarchy.ContainsKey(packageName)) {
                    MessageHierarchy.Add(packageName, new List<string>());
                    OpenedPackages[packageName] = false;
                    CheckedPackages[packageName] = false;
                }
                CheckedMessages[messageName] = false;
                MessageHierarchy[packageName].Add(messageName);
            }
        }

        public void MarkAll(bool select) {
            foreach (string package in MessageHierarchy.Keys) {
                CheckedPackages[package] = select;
                MarkAllInPackage(select, package);
            }
        }

        public void MarkAllInPackage(bool select, string packageName) {
            if (!MessageHierarchy.ContainsKey(packageName)) {
                return;
            }
            for (int i = 0; i < MessageHierarchy[packageName].Count; i++) {
                CheckedMessages[MessageHierarchy[packageName][i]] = select;
            }
        }

        public void SelectAllToggleMatchChildren() {
            bool hasUnselectedChildren = false;
            if (CheckedMessages.Values.Contains(false) || CheckedPackages.Values.Contains(false)) {
                hasUnselectedChildren = true;
            }
            ToggleSelectAll = !hasUnselectedChildren;
        }

        public void PackageToggleMatchChildren(string packageName) {
            bool hasUnselectedMessages = false;
            foreach (string messageType in MessageHierarchy[packageName]) {
                if (!CheckedMessages[messageType]) {
                    hasUnselectedMessages = true;
                }
            }
            CheckedPackages[packageName] = !hasUnselectedMessages;
        }

        public List<string> GetSelectedMessages() {
            List<string> selected = new List<string>();
            foreach (var pair in MessageHierarchy) {
                foreach(string name in pair.Value) {
                    if (CheckedMessages.ContainsKey(name) && CheckedMessages[name] == true) {
                        selected.Add(pair.Key + "/" + name);
                    }
                }
            }
            return selected;
        }

        private void ResetImportResults() {
            MessageHierarchy = new SortedDictionary<string, List<string>>();
            OpenedPackages = new Dictionary<string, bool>();
            CheckedMessages = new Dictionary<string, bool>();
            CheckedPackages = new Dictionary<string, bool>();
            ToggleSelectAll = false;
        }
    }
}
