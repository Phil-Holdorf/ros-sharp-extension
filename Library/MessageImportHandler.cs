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
using RosSharp.RosBridgeClient.Protocols;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEditor;
using UnityEngine;

namespace RosSharpExtension {
    /// <summary>
    /// Manages the UI state of the <see cref="IMessageImportUI"/> and handles the user input from it.
    /// <para/>
    /// When pressing one of the 'Load Messages' button, a ROS service call is made.
    /// The results are delegated to <see cref="MessageImportResultsHandler"/>.
    /// <para/>
    /// When pressing the 'Generate Messages' button, several ROS service calls are made.
    /// The message files requested by the user and its dependencies are transferred from ROS.
    /// Finally, .cs files are generated for each message.
    /// </summary>
    internal class MessageImportHandler {
        public Protocol ProtocolType { get; set; }
        public string Address { get; set; }
        public bool OverwriteMessages { get; set; } = false;
        public string Package { get; set; }
        public readonly Dictionary<string, ManualResetEvent> StatusEvents = new Dictionary<string, ManualResetEvent> {
            {"connected", new ManualResetEvent(false) },
            {"connectionFailed", new ManualResetEvent(false) },
            {"loadingStarted" , new ManualResetEvent(false) },
            {"loadingComplete", new ManualResetEvent(false) },
            {"transferStarted" , new ManualResetEvent(false) },
            {"generationComplete", new ManualResetEvent(false) }
        };

        private const ushort timeoutMilliseconds = 3000;

        private MessageImportResultsHandler resultsHandler;
        private MessageGenerationHandler messageGenerationHandler;
        private MessageImportEditorPrefs prefs;
        private RosSocket rosSocket;

        public MessageImportHandler(MessageImportResultsHandler resultsHandler) {
            this.resultsHandler = resultsHandler;
            messageGenerationHandler = new MessageGenerationHandler();
            prefs = new MessageImportEditorPrefs();
        }

        public void LoadAllMessagesAsync() {
            StatusEvents["loadingComplete"].Reset();
            Thread rosSocketConnectThread = new Thread(() => {
                if (Connect(ProtocolType, Address)) {
                    new MessageLoader(rosSocket).LoadAllMessages(OnLoadAllMessagesComplete);
                }
            });
            rosSocketConnectThread.Start();
            StatusEvents["loadingStarted"].Set();
        }

        public void LoadMessagesFromTopicsAsync() {
            StatusEvents["loadingComplete"].Reset();
            Thread rosSocketConnectThread = new Thread(() => {
                if (Connect(ProtocolType, Address)) {
                    new MessageLoader(rosSocket).LoadMessagesFromTopics(OnLoadMessagesFromTopicsComplete);
                }
            });
            rosSocketConnectThread.Start();
            StatusEvents["loadingStarted"].Set();
        }

        public void LoadMessagesInPackageAsync() {
            StatusEvents["loadingComplete"].Reset();
            Thread rosSocketConnectThread = new Thread(() => {
                if (Connect(ProtocolType, Address)) {
                    new MessageLoader(rosSocket).LoadMessagesInPackage(Package, OnLoadComplete);
                }
            });
            rosSocketConnectThread.Start();
            StatusEvents["loadingStarted"].Set();
        }

        public void TransferMessagesAsync() {
            StatusEvents["generationComplete"].Reset();
            Thread rosSocketConnectThread = new Thread(() => {
                if (Connect(ProtocolType, Address)) {

                    List<string> messagesToTransfer = resultsHandler.GetSelectedMessages();

                    new MessageTransfer(rosSocket).Transfer(messagesToTransfer, OnMessageTransferComplete);
                }
            });
            rosSocketConnectThread.Start();
            StatusEvents["transferStarted"].Set();
        }

        public void GetEditorPrefs() {
            prefs.GetEditorPrefs(out Protocol protocolType, out string address);
            ProtocolType = protocolType;
            Address = address;
        }

        public void SetEditorPrefs() {
            prefs.SetEditorPrefs(ProtocolType, Address);
        }

        private void OnLoadAllMessagesComplete(string[] messages) {
            OnLoadComplete(messages);
        }

        private void OnLoadMessagesFromTopicsComplete(string[] topics, string[] messages) {
            OnLoadComplete(messages);
        }

        private void OnLoadComplete(string[] messages) {
            rosSocket?.Close();
            rosSocket = null;
            resultsHandler.OnMessagesLoaded(messages);
            StatusEvents["loadingComplete"].Set();
        }

        private void OnMessageTransferComplete(List<string> messageNames, List<string> fileContents) {
            List<string> dependencies = messageGenerationHandler.OnMessageTransferComplete(messageNames, fileContents, OverwriteMessages);
            if (dependencies.Count > 0) {
                new MessageTransfer(rosSocket).Transfer(dependencies, OnMessageTransferComplete);
            } else {
                rosSocket?.Close();
                rosSocket = null;
                StatusEvents["generationComplete"].Set();
            }
        }

        /// <summary>
        /// this method blocks until socket is connected or until timeout
        /// </summary>
        private bool Connect(Protocol protocolType, string address) {
            StatusEvents["connected"].Reset();
            StatusEvents["connectionFailed"].Reset();
            if (rosSocket == null) {
                IProtocol protocol;
                if (protocolType == Protocol.WebSocketNET) {
                    protocol = new WebSocketNetProtocol(address);
                } else {
                    protocol = new WebSocketSharpProtocol(address);
                }
                protocol.OnConnected += OnConnected;
                protocol.OnClosed += OnClosed;
                rosSocket = new RosSocket(protocol);
            }
            if (!StatusEvents["connected"].WaitOne(timeoutMilliseconds)) {
                rosSocket.Close();
                rosSocket = null;
                StatusEvents["connectionFailed"].Set();
                return false;
            }
            return true;
        }

        private void OnClosed(object sender, EventArgs e) {
            StatusEvents["connected"].Reset();
        }

        private void OnConnected(object sender, EventArgs e) {
            StatusEvents["connected"].Set();
        }

        private void OnApplicationQuit() {
            rosSocket.Close();
        }

    }
}
