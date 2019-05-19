using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEngine;

namespace RosSharpExtension {
    internal class MessageGenerationHandler {
        private const string assetPath = "Assets/Ros Messages";

        //elements saved as ros type strings ("package_name/msg_name")
        private HashSet<string> dependencies = new HashSet<string>();
        private HashSet<string> completedDependencies = new HashSet<string>();
        
        private CustomMessageGenerator generator;
        private MessageParser parser;

        public MessageGenerationHandler() {
            generator = new CustomMessageGenerator();
            parser = new MessageParser();
        }

        public List<string> OnMessageTransferComplete(List<string> messageNames, List<string> fileContents, bool overwriteFiles) {

            List<string> dependenciesToTransfer = new List<string>();

            for (int i = 0; i < messageNames.Count; i++) {
                if (dependencies.Contains(messageNames[i]) && !completedDependencies.Contains(messageNames[i])) {
                    completedDependencies.Add(messageNames[i]);
                }

                new MessageNameSplitter().Split(messageNames[i], out string packageName, out string messageName);

                List<CustomMessageElement> parsedElements = parser.ParseFile(packageName, messageName, fileContents[i]);


                //add dependencies for this message file
                foreach (var element in parsedElements) {
                    if (element.IsPrimitive) {
                        continue;
                    }
                    if (!dependencies.Contains(element.FullName)) {
                        dependencies.Add(element.FullName);
                    }
                }

                //handle special types 'time' and 'duration'
                if (packageName == "std_msgs" && messageName == "Time") {
                    generator.Generate("std_msgs", "Time", TimeTemplate.elements, assetPath, overwriteFiles);
                } else if (packageName == "std_msgs" && messageName == "Duration") {
                    generator.Generate("std_msgs", "Duration", DurationTemplate.elements, assetPath, overwriteFiles);
                }
                else {
                    generator.Generate(packageName, messageName, parsedElements, assetPath, overwriteFiles);
                }

                foreach (var element in parsedElements) {
                    string rosType = element.FullName;
                    if (dependencies.Contains(rosType) && !completedDependencies.Contains(rosType)) {
                        if (!dependenciesToTransfer.Contains(rosType))
                        dependenciesToTransfer.Add(rosType);
                    }
                }
            }
            return dependenciesToTransfer;
        }

        private HashSet<string> GetRosSharpMessages() {
            Assembly[] assemblies = AppDomain.CurrentDomain.GetAssemblies();
            HashSet<string> rosSharpMessages = new HashSet<string>();
            foreach (Assembly a in assemblies) {
                string name = a.GetName().Name;
                if (name == "RosBridgeClient") {
                    Type[] types = a.GetTypes();
                    foreach (Type type in types) {
                        if (type.Namespace != null && type.Namespace.StartsWith("RosSharp.RosBridgeClient.Messages")) {
                            string[] split = type.Namespace.Split('.');
                            string packageName = split[split.Length - 1];
                            string rosType;
                            if (packageName == "Messages") {
                                rosType = type.Name;
                            }
                            else {
                                rosType = packageName + "/" + type.Name;
                            }
                            rosSharpMessages.Add(rosType);
                        }
                    }
                }
            }
            return rosSharpMessages;
        }
    }
}
