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
