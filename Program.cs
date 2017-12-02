using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.IO;

namespace SocketClient
{
    class Program
    {
        private static byte[] result = new byte[1024];
        static void Main(string[] args)
        {
            //设定服务器IP地址
            IPAddress ip = IPAddress.Parse("127.0.0.1");
            Socket clientSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            try
            {
                clientSocket.Connect(new IPEndPoint(ip, 8800)); //配置服务器IP与端口
                Console.WriteLine("successed in connecting to server");
            }
            catch
            {
                Console.WriteLine("failed in connecting to server, press enter to exit！");
                return;
            }
            //通过clientSocket接收数据
            int receiveLength = clientSocket.Receive(result);           
            Console.WriteLine("receieve from the server：{0}",Encoding.ASCII.GetString(result,0,receiveLength));





            //通过文本读取数据
            /* byte[] byData = new byte[100];
             char[] charData = new char[1000];
             try
             {
                 FileStream file = new FileStream("E:\\test.text", FileMode.Open);
                 file.Seek(0, SeekOrigin.Begin);
                 file.Read(byData, 0, 100); //byData传进来的字节数组,用以接受FileStream对象中的数据,第2个参数是字节数组中开始写入数据的位置,它通常是0,表示从数组的开端文件中向数组写数据,最后一个参数规定从文件读多少字符.
                 Decoder d = Encoding.Default.GetDecoder();
                 d.GetChars(byData, 0, byData.Length, charData, 0);
                 Console.WriteLine(charData);
                 file.Close();
             }
             catch (IOException e)
             {
                 Console.WriteLine(e.ToString());
             }

             string str =test.Text().Trim();
             string[] strs = str.Split(new string[] { "\r\n" }, StringSplitOptions.RemoveEmptyEntries);
             List<double> datas = new List<double>();
             foreach (string textStr in strs)
             {
                 datas.Add(double.Parse(textStr));
             }

             double[] myResult = datas.ToArray();
             StreamReader rd = File.OpenText("E:\\test.text");
            string s = rd.ReadLine();
            string[] ss = s.Split(',');
            int row = 1;
            int col = 9;  //每行数据的个数

            double[,] p1 = new double[row, col]; //数组

            for (int i = 0; i < row; i++)  //读入数据并赋予数组
            {
                string line = rd.ReadLine();
                string[] data = line.Split(',');
                for (int j = 0; j < col; j++)
                {
                    p1[i, j] = double.Parse(data[j]);
                }
            }
            */

            //通过 clientSocket 发送数据
            double[] a = { 1, 2, 3 }; // double数组，假定长度为3
            string sendMessage = "1";
            for (int i = 0; i < 10; i++)
            {
                try
                {
                    Thread.Sleep(1000);    //等待0.1秒钟
                    int j = 0;
                    for (j = 0; j <3; j++)
                    {
                        //string sendMessage = new string(a);
                        string[] strings = new string[3];//空的string数组，假定长度为3（string数组的长度>=double数组的长度）
                        for (int k = 0; k < 3; k++)
                        {
                            strings[k] = a[k].ToString();//将double数组中的元素转换为string，插入string数组中
                        }
                        sendMessage = string.Join(" ", strings);
                        

                        clientSocket.Send(Encoding.ASCII.GetBytes(Convert.ToString(sendMessage)));

                       if (j == 3)
                        { 
                            Console.WriteLine(sendMessage + "\n" + "send to the server：next"  );
                        }
                        else
                        {
                            Console.WriteLine(sendMessage);
                        }
                    }
                }
                catch
                {
                    clientSocket.Shutdown(SocketShutdown.Both);
                    clientSocket.Close();
                    break;
                }
            }
            Console.WriteLine("send over, press enter to exit");
            Console.ReadLine();
        }   
    }
}
