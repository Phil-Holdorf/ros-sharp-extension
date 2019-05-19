using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Microsoft.CSharp;
using System;

namespace RosSharpExtension {
    public class CustomMessageGenerator {

        private readonly string nl = Environment.NewLine;

        public void Generate(string packageName, string messageName, List<CustomMessageElement> messageElements, string assetPath, bool overwriteFile) {
            string path = assetPath + "/" + packageName + "/" + messageName + ".cs";
            if (File.Exists(path) && !overwriteFile) {
                return;
            }
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            WriteFileContent(packageName, messageName, messageElements, path);
        }

        private void WriteFileContent(string packageName, string messageName, List<CustomMessageElement> messageElements, string path) {
            using (StreamWriter outfile = new StreamWriter(path, false)) {
                WriteTopComment(outfile);
                WriteUsings(outfile, messageElements);
                WriteNamespaceDeclaration(outfile, packageName);
                WriteClassDeclaration(outfile, messageName);
                WriteRosMessageName(outfile, packageName, messageName);
                WriteFieldDeclarations(outfile, messageElements);
                WriteConstructor(outfile, messageName, messageElements);
                outfile.WriteLine("    }"); //closing class definition
                outfile.WriteLine("}"); //closing namespace
            }
        }

        private void WriteTopComment(StreamWriter outfile) {
            outfile.WriteLine("/*");
            outfile.WriteLine("This message class is generated automatically with 'CustomMessageGenerator' of RosSharpExtension");
            outfile.WriteLine("*/" + nl);
        }

        private void WriteUsings(StreamWriter outfile, List<CustomMessageElement> messageElements) {
            foreach (string u in GetUsings(messageElements)) {
                outfile.WriteLine("using " + u + ";");
            }
            outfile.WriteLine();
        }

        private void WriteNamespaceDeclaration(StreamWriter outfile, string packageName) {
            packageName = MakeValidIdentifier(packageName);
            outfile.WriteLine("namespace RosSharp.RosBridgeClient.Messages." + packageName + " {");
        }

        private void WriteClassDeclaration(StreamWriter outfile, string messageName) {
            messageName = MakeValidIdentifier(messageName);
            outfile.WriteLine("    public class " + messageName + " : Message {");
        }

        private void WriteRosMessageName(StreamWriter outfile, string packageName, string messageName) {
            outfile.WriteLine(
                "        [JsonIgnore]" + nl +
                "        public const string RosMessageName = \"" + packageName + "/" + messageName + "\";" + nl
            );
        }

        private void WriteFieldDeclarations(StreamWriter outfile, List<CustomMessageElement> messageElements) {
            for (int i = 0; i < messageElements.Count; i++) {
                outfile.WriteLine("        " + GetDeclarationString(messageElements[i]));
            }
            outfile.WriteLine();
        }

        private void WriteConstructor(StreamWriter outfile, string messageName, List<CustomMessageElement> messageElements) {
            messageName = MakeValidIdentifier(messageName);
            outfile.WriteLine("        public " + messageName + "() {");

            for (int i = 0; i < messageElements.Count; i++) {
                string def = GetDefinitionString(messageElements[i]);
                if (def.Length > 0) {
                    outfile.WriteLine("            " + def);
                }
            }

            outfile.WriteLine("        }"); //closing constructor
        }

        private HashSet<string> GetUsings(List<CustomMessageElement> elements) {
            HashSet<string> usings = new HashSet<string> {
                "Newtonsoft.Json",
                "RosSharp.RosBridgeClient",
                "RosSharp.RosBridgeClient.Messages"
            };
            foreach (var element in elements) {
                string packageName = element.PackageName;
                if (packageName != "") {
                    packageName = MakeValidIdentifier(packageName);
                    string nspace = "RosSharp.RosBridgeClient.Messages." + packageName;
                    if (!usings.Contains(nspace)) {
                        usings.Add(nspace);
                    }
                }
            }
            return usings;
        }

        private string GetDeclarationString(CustomMessageElement element) {
            string identifier = MakeValidIdentifier(element.FieldName);
            string type = element.MessageName;
            if (!element.IsPrimitive) {
                type = MakeValidIdentifier(type);
            }
            if (!element.IsArray) {
                return "public " + type + " " + identifier + ";";
            } else {
                return "public " + type + "[] " + identifier + ";";
            }
        }

        private string GetDefinitionString(CustomMessageElement element) {
            string identifier = MakeValidIdentifier(element.FieldName);
            string type = element.MessageName;
            if (!element.IsPrimitive) {
                type = MakeValidIdentifier(type);
            }
            if (element.IsArray) {
                return identifier + " = new " + type + "[0];";
            } else if (type == ("string")) {
                return identifier + " = \"\";";
            } else if (element.IsPrimitive) {
                return ""; //primitive types other than string don't need to be initialized.
            } else {
                return identifier + " = new " + type + "();";
            }
        }

        private string MakeValidIdentifier(string type) {
            var cs = new CSharpCodeProvider();
            if (!cs.IsValidIdentifier(type)) {
                return "@" + type;
            }
            else {
                return type;
            }
        }
    }
}
