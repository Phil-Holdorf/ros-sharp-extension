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
using RosSharp.RosBridgeClient.Services.FileServer;
using RosSharp.RosBridgeClient.Services.RosApi;
using System.Collections.Generic;

namespace RosSharpExtension
{
    public class MessageTransfer
    {
        public delegate void TransferCompleteCallback(List<string> messageNames, List<string> files);
        private TransferCompleteCallback transferCallback;

        private List<string> messageNames = new List<string>();
        private List<string> fileContentList = new List<string>();
        private RosSocket rosSocket;

        public MessageTransfer(RosSocket socket) {
            rosSocket = socket;
        }

        public void Transfer(List<string> messageNames, TransferCompleteCallback callback)
        {
            transferCallback = callback;
            if (messageNames.Count == 0)
            {
                return;
            }
            this.messageNames = messageNames;
            TransferSingleFile(messageNames[0]);
        }

        private void TransferSingleFile(string message)
        {
            message = message.Trim('/');
            string[] split = message.Split('/');
            string packageName = split[0];
            string messageName = message.Substring(packageName.Length+1);
            //do messages always go into package root/msg ? what if the message is in a subfolder?
            string requestString = "package://" + packageName + "/msg/" + messageName + ".msg";

            //command:
            //rosservice call file_server/get_file package://ros_sharp_test/msg/Test1.msg
            var requestMessage = new GetBinaryFileRequest(requestString);
            rosSocket.CallService<GetBinaryFileRequest, GetBinaryFileResponse>
                ("file_server/get_file", ReceiveFile, requestMessage);
        }

        private void ReceiveFile(GetBinaryFileResponse fileContent)
        {
            string fileContentsString = "";
            fileContentsString = System.Text.Encoding.ASCII.GetString(fileContent.value);
            fileContentList.Add(fileContentsString);

            if (fileContentList.Count == messageNames.Count)
            {
                transferCallback.Invoke(messageNames, fileContentList);
            } else
            {
                TransferNextFile();
            }
        }

        private void TransferNextFile()
        {
            string nextMessage = messageNames[fileContentList.Count];
            TransferSingleFile(nextMessage);
        }


    }

}
