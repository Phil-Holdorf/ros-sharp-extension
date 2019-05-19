﻿/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class PoseStampedPublisher : Publisher<Messages.geometry_msgs.PoseStamped>
    {
        public Transform PublishedTransform;
        public string FrameId = "Unity";

        private Messages.geometry_msgs.PoseStamped message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new Messages.geometry_msgs.PoseStamped
            {
                header = new Messages.std_msgs.Header()
                {
                    frame_id = FrameId
                }
            };
        }

        private void UpdateMessage()
        {
            message.header.Update();
            message.pose.position = GetGeometryPoint(PublishedTransform.position.Unity2Ros());
            message.pose.orientation = GetGeometryQuaternion(PublishedTransform.rotation.Unity2Ros());

            Publish(message);
        }

        private Messages.geometry_msgs.Point GetGeometryPoint(Vector3 position)
        {
            Messages.geometry_msgs.Point geometryPoint = new Messages.geometry_msgs.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }

        private Messages.geometry_msgs.Quaternion GetGeometryQuaternion(Quaternion quaternion)
        {
            Messages.geometry_msgs.Quaternion geometryQuaternion = new Messages.geometry_msgs.Quaternion();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }

    }
}