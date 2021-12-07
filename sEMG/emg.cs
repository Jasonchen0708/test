using System;
using System.Collections.Generic;
using System.Text;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Linq;
namespace semg_data_process
{
    class read_semg_data
    {
        //private DateTime TimeStart = DateTime.Now;       
        static int data_legth = 44;
        byte[] byteTemp = new byte[data_legth]; //申请存放数据的缓冲区
        byte[] RxBuffer = new byte[4*data_legth];
        //public double[] sensor_Data_1 = new double[10];
        //public double[] sensor_Data_2 = new double[10];
        //public double[] sensor_Data_3 = new double[10];
        //public double[] sensor_Data_4 = new double[10];
        //public List<List<Queue<double>>> EmgDataGui = new List<List<Queue<double>>>();
        private static int channel = 4;
        public List<List<double>> EmgRecords = new List<List<double>>(capacity: channel);
        public SerialPort SerialPort1 = new SerialPort("COM5");//根据电脑串口       
        private void UpdateQueue(int header, double[] bytes)
        {
            int data_length = bytes.Length;
            for (int j = 0; j < data_length; j += 1)
            {
                EmgRecords[header].Add(bytes[j]);
            }
        }
        public void btnOpenCom_Click()
        {
            if (SerialPort1.IsOpen == false)
            {
                SerialPort1.BaudRate = 1000000;
                try
                {
                    SerialPort1.Open();
                    Console.WriteLine("打开串口成功");
                }
                catch (Exception)
                {
                    Console.WriteLine("串口连接失败！\r\n可能原因：串口被占用");
                }
            }
            else
            {
                Console.WriteLine("串口已经打开");
            }
        }
        public void SerialPort1_DataReceived()
        {
            // read_data Update = new read_data();
    
            //UInt16 usLength = 0;
            //serialPort1.Read(Rcedata, 0, serialPort1.BytesToRead);//读取数据到data中 
            SerialPort1.Read(RxBuffer, 0, 4* data_legth);
            SerialPort1.DiscardInBuffer();
            //DataReceivedTime = DateTime.Now;
            List<int> locatedBytes = LocateStartBytes(ref RxBuffer);
            foreach (int offset in locatedBytes)
            {
                Array.Copy(RxBuffer, offset, byteTemp, 0, 44);
                if ((byteTemp[0] == 0xAA) && (byteTemp[1] == 0xAA))
                {
                    int header = byteTemp[43];
                    if (0x01 <= header && header <= 0x04)
                    {
                        double[] data = DecodeData(byteTemp);
                        UpdateQueue(header - 1, data);
                    }
                }
            }
        }
            //SerialPort1.DiscardInBuffer();
        private List<int> LocateStartBytes(ref byte[] src)
        {
            List<int> ans = new List<int>(capacity: 4);
            for (int i = 0; i < src.Length - 43; i++)
            {
                if (src[i] == 0xAA && src[i + 1] == 0xAA)
                {
                    if (i + 43 < src.Length)
                    {
                        ans.Add(i);
                        i += 43;
                    }
                }
            }
            return ans;
        }
        private double[] DecodeData(byte[] byteTemp)
        {
            double[] Data = new double[10];
            for (int i = 0; i < 9; i++)
            {
                Data[i] = (short)((byteTemp[3+2*i] << 8) + byteTemp[4+2*i]);
            }
            return Data;
        }
    }
}
