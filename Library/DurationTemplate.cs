using RosSharpExtension;
using System.Collections.Generic;

namespace RosSharpExtension {
    public class DurationTemplate
    {
        private static readonly CustomMessageElement secs = new CustomMessageElement("", "int", "secs", false, true);
        private static readonly CustomMessageElement nsecs = new CustomMessageElement("", "int", "nsecs", false, true);
        public static List<CustomMessageElement> elements = new List<CustomMessageElement> { secs, nsecs };
    }
}
