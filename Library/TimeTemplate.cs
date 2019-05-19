using RosSharpExtension;
using System.Collections.Generic;

namespace RosSharpExtension {
    public class TimeTemplate {
        private static readonly CustomMessageElement secs = new CustomMessageElement("", "uint", "secs", false, true);
        private static readonly CustomMessageElement nsecs = new CustomMessageElement("", "uint", "nsecs", false, true);
        public static List<CustomMessageElement> elements = new List<CustomMessageElement> { secs, nsecs };
    }
}
