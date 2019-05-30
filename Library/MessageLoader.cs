/* MIT License

Copyright (c) 2019 Phil-Holdorf

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Services.RosApi;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharpExtension {
    public class MessageLoader {
        public delegate void LoadMessagesFromTopicsCallback(string[] topics, string[] types);
        public delegate void LoadMessagesCallback(string[] types);

        private LoadMessagesFromTopicsCallback topicsCallback;
        private LoadMessagesCallback allMessagesCallback;
        private LoadMessagesCallback packageMessagesCallback;

        private RosSocket rosSocket;

        public MessageLoader(RosSocket socket) {
            rosSocket = socket;
        }

        public void LoadMessagesFromTopics(LoadMessagesFromTopicsCallback callback) {
            topicsCallback = callback;
            rosSocket.CallService<TopicsRequest, TopicsResponse>("rosapi/topics", ReceiveTopics, new TopicsRequest());
        }

        public void LoadAllMessages(LoadMessagesCallback callback) {
            allMessagesCallback = callback;
            rosSocket.CallService<ListMessagesRequest, ListMessagesResponse>("ros_sharp_extension/list_messages", ReceiveAllMessages, new ListMessagesRequest());
        }

        public void LoadMessagesInPackage(string package, LoadMessagesCallback callback) {
            packageMessagesCallback = callback;
            rosSocket.CallService<ListMessagesInPackageRequest,
                ListMessagesInPackageResponse>("ros_sharp_extension/list_messages_in_package",
                ReceivePackage,
                new ListMessagesInPackageRequest(package));
        }

        private void ReceiveTopics(TopicsResponse response) {
            topicsCallback.Invoke(response.topics, response.types);
        }

        private void ReceiveAllMessages(ListMessagesResponse response) {
            allMessagesCallback.Invoke(response.message_types);
        }

        private void ReceivePackage(ListMessagesInPackageResponse response) {
            packageMessagesCallback.Invoke(response.messages);
        }
    }
}
