using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

namespace RosSharpExtension {
    public class ListMessagesInPackageRequest : Message {
        public string package_name;

        public ListMessagesInPackageRequest(string package) {
            package_name = package;
        }
    }

    public class ListMessagesInPackageResponse : Message {
        public string[] messages;
    }
}
