using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharpExtension {
    public class CustomMessageElement {

        public string PackageName { get; protected set; }
        public string MessageName { get; protected set; }
        public string FullName { get; protected set; }
        public string FieldName { get; protected set; }
        public bool IsArray { get; protected set; }
        public bool IsPrimitive { get; protected set; }

        public CustomMessageElement(string pkgName, string msgName, string fieldName, bool isArray, bool isPrimitive) {
            PackageName = pkgName;
            MessageName = msgName;
            FieldName = fieldName;
            IsArray = isArray;
            IsPrimitive = isPrimitive;
            FullName = (pkgName == "") ? (msgName) : (pkgName + "/" + msgName);
        }
    }
}
