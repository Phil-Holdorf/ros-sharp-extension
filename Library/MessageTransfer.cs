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
