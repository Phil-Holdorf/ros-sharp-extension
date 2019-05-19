using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.CSharp;

namespace RosSharpExtension {
    public class MessageParser {
        
        protected static readonly Dictionary<string, string> primitives = new Dictionary<string, string>() {
            {"bool", "bool" },
            {"int8", "sbyte" },
            {"uint8", "byte" },
            {"int16", "short" },
            {"uint16", "ushort" },
            {"int32", "int" },
            {"uint32", "uint" },
            {"int64", "long" },
            {"uint64", "ulong" },
            {"float32", "float" },
            {"float64", "double" },
            {"string", "string" },
            {"char", "byte" },
            {"byte", "sbyte" }
        };

        public List<CustomMessageElement> ParseFile(string filePackageName, string fileMessageName, string fileContent) {
            List<CustomMessageElement> elementsInFile = new List<CustomMessageElement>();
            using (System.IO.StringReader reader = new System.IO.StringReader(fileContent)) {
                string line;
                while ((line = reader.ReadLine()) != null) {
                    line = line.Trim();
                    if (line.StartsWith("#")) {
                        continue;
                    }

                    //if there is a comment at end of line, get the side that is left of the comment.
                    string uncommentedSide = line.Split('#')[0];

                    string[] words = uncommentedSide.Split(new char[0], StringSplitOptions.RemoveEmptyEntries);

                    if (uncommentedSide.Contains("=")) {
                        //constant definitions will not be parsed currently.
                        continue;
                    }

                    if (words.Length < 2) {
                        continue;
                    }
                    CustomMessageElement element = ParseElement(words[0], words[1], filePackageName);
                    elementsInFile.Add(element);
                }
            }

            return elementsInFile;
        }

        private CustomMessageElement ParseElement(string typeName, string fieldName, string filePackageName) {
            new MessageNameSplitter().Split(typeName, out string elementPackageName, out string elementMessageName);
            bool isArray = elementMessageName.EndsWith("]");
            if (isArray) {
                elementMessageName = elementMessageName.Split('[')[0];
            }
            bool isPrimitive = (elementPackageName.Equals("") && primitives.ContainsKey(elementMessageName));
            if (isPrimitive) {
                elementMessageName = primitives[elementMessageName];
            } else if (elementMessageName.Equals("time")) {
                    elementPackageName = "std_msgs";
                    elementMessageName = "Time";
            }
            else if (elementMessageName.Equals("duration")) {
                elementPackageName = "std_msgs";
                elementMessageName = "Duration";
            }
            else if (elementMessageName.Equals("Header")) {
                elementPackageName = "std_msgs";
            }
            else if (elementPackageName.Equals("")) {
                elementPackageName = filePackageName;
            }
            return new CustomMessageElement(elementPackageName, elementMessageName, fieldName, isArray, isPrimitive);
        }
    }
}
