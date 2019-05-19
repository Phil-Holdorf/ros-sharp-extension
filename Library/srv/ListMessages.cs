using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

namespace RosSharpExtension {
    public class ListMessagesRequest : Message {

    }

    public class ListMessagesResponse : Message {
        public string[] message_types;
    }
}
