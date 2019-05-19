using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharpExtension {
    public class MessageNameSplitter{
        public void Split(string fullName, out string packageName, out string messageName) {
            string[] split = fullName.Split('/');
            if (split.Length == 1) {
                packageName = "";
                messageName = fullName;
                return;
            }
            packageName = split[0];
            messageName = split[1];
        }
    }
}
