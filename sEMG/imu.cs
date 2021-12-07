using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Linq;

namespace IMU_pose_process
{
    class data_init
    {
        public List<double[,]> imu = new List<double[,]>();//用于缓存imu数据的链表
        public double[,] node_trans = new double[3, 7];
        double road2angle = (180 / Math.PI);
        public void get_data(double[,] inputdata) // 计算根节点转换矩阵 
        {
            imu.Add(inputdata);
        }
        public double[,] node_init(List<double[,]> data)
        {
            double[,] node_data = new double[data[0].GetLength(0), data[0].GetLength(1)];// node_data计算静息状态下的平均值
            Console.WriteLine("node_data行：{0},node_data列：{1}", node_data.GetLength(0), node_data.GetLength(1));
            double norm_data = 0;
            for (int i = 0; i < data[0].GetLength(1); i++)//7传感器数量 数据结构[4，7]
            {
                for (int j = 0; j < data[0].GetLength(0); j++)//4 四元数值
                {
                    for (int k = 0; k < data.Count; k++)
                    {
                        node_data[j, i] += data[k][j, i];
                    }
                    //计算标定平均值
                    node_data[j, i] = node_data[j, i] / ((double)data.Count);
                    norm_data += Math.Pow(node_data[j, i], 2);
                    //Console.Write("arv:{0},", node_data[j, i]);
                }
                //四元数归一化
                node_data[0, i] /= Math.Sqrt(norm_data);
                node_data[1, i] /= Math.Sqrt(norm_data);
                node_data[2, i] /= Math.Sqrt(norm_data);
                node_data[3, i] /= Math.Sqrt(norm_data);
                norm_data = 0;
                //根据加速度值，陀螺仪及磁力计计算初始姿态四元数q0
                //将四元数转换成欧拉角zyx
                //roll           
                node_trans[0, i] = Math.Atan2((-2 * ((node_data[2, i] * node_data[3, i]) - (node_data[0, i] * node_data[1, i]))), Math.Pow(node_data[0, i], 2) + Math.Pow(node_data[3, i], 2) - Math.Pow(node_data[1, i], 2) - Math.Pow(node_data[2, i], 2)) * road2angle;
                Console.Write("roll:{0},\n", node_trans[0, i]);
                //pitch
                node_trans[1, i] = Math.Asin(2 * ((node_data[1, i] * node_data[3, i]) + (node_data[0, i] * node_data[2, i]))) * road2angle;//pitch
                Console.Write("pitch:{0},\n", node_trans[1, i]);
                //yaw
                node_trans[2, i] = Math.Atan2(2 * ((node_data[1, i] * node_data[2, i]) - (node_data[0, i] * node_data[3, i])), (Math.Pow(node_data[0, i], 2) + Math.Pow(node_data[1, i], 2) - Math.Pow(node_data[2, i], 2) - Math.Pow(node_data[3, i], 2))) * road2angle;
                Console.Write("yaw:{0},\n", node_trans[2, i]);

            }
            data.Clear();
            return node_data;
        }
    }
    class data_update
    {
        public List<double[]> node_rotation = new List<double[]>();
        public List<double[]> angle = new List<double[]>();
        math_computation computation = new math_computation();
        double trans = 0;
        double[] norm_data = new double[4];
        double[] trans_data = new double[4];
        public List<double[]> node_update(double[,] inputdata, double[,] node_trans) //获得实时数据并对各节点做坐标轴修正，根据上一时刻进行修改 主函数进行调用时候将传出值赋予node_trans
        {
            node_rotation.Clear();
            for (int i = 0; i < inputdata.GetLength(1); i++)//7传感器数量 数据结构[4，7]
            {
                trans = 0;
                for (int j = 0; j < inputdata.GetLength(0); j++)
                {
                    trans += Math.Pow(inputdata[j, i], 2); ;
                }
                double norm = Math.Sqrt(trans);
                //四元数归一化
                norm_data[0] = inputdata[0, i] / norm;
                norm_data[1] = inputdata[1, i] / norm;
                norm_data[2] = inputdata[2, i] / norm;
                norm_data[3] = inputdata[3, i] / norm;
                trans_data[0] = node_trans[0, i];
                trans_data[1] = -1 * node_trans[1, i];
                trans_data[2] = -1 * node_trans[2, i];
                trans_data[3] = -1 * node_trans[3, i];
                //右乘初始矩阵的逆进行转换
                double[] cor_data = computation.Quaternioncov(norm_data, trans_data);
                node_rotation.Add(cor_data);
            }
            return node_rotation;
        }
        public List<double[]> node_update_1(double[,] inputdata, double[,] node_trans) //获得实时数据并对各节点做坐标轴修正，根据上一时刻进行修改 主函数进行调用时候将传出值赋予node_trans
        {
            List<double[]> node_rotation = new List<double[]>();
            for (int i = 0; i < inputdata.GetLength(1); i++)//7传感器数量 数据结构[4，7]
            {
                trans = 0;
                for (int j = 0; j < inputdata.GetLength(0); j++)
                {
                    trans += Math.Pow(inputdata[j, i], 2);
                }
                double norm = Math.Sqrt(trans);
                Console.Write(norm);
                Console.Write("\n");
                //四元数归一化
                norm_data[0] = inputdata[0, i]/norm;
                /*Console.Write(inputdata[0, i]);
                Console.Write("\n");
                Console.Write(norm_data[0]);
                Console.Write("\n");*/
                norm_data[1] = inputdata[1, i] / norm;
                norm_data[2] = inputdata[2, i] / norm;
                norm_data[3] = inputdata[3, i] / norm;
                trans_data[0] = node_trans[0, i];
                trans_data[1] = -1 * node_trans[1, i];
                trans_data[2] = -1 * node_trans[2, i];
                trans_data[3] = -1 * node_trans[3, i];
                //右乘初始矩阵的逆进行转换               
                //double[] cor_data = computation.Quaternioncov(trans_data, norm_data);
                double[] cor_data = computation.Quaternioncov(norm_data, trans_data);//norm_data*trans_data
                node_rotation.Add(cor_data);
            }
            return node_rotation;
        }
        public List<double[]> node_update_2(double[,] inputdata, double[,] node_trans) //获得实时数据并对各节点做坐标轴修正，根据上一时刻进行修改 主函数进行调用时候将传出值赋予node_trans
        {
            node_rotation.Clear();
            for (int i = 0; i < inputdata.GetLength(1); i++)//7传感器数量 数据结构[4，7]
            {
                trans = 0;
                for (int j = 0; j < inputdata.GetLength(0); j++)
                {
                    trans += Math.Pow(inputdata[j, i], 2);
                }
                double norm = Math.Sqrt(trans);
                //四元数归一化
                norm_data[0] = inputdata[0, i] / norm;
                norm_data[1] = inputdata[1, i] / norm;
                norm_data[2] = inputdata[2, i] / norm;
                norm_data[3] = inputdata[3, i] / norm;
                trans_data[0] = node_trans[0, i];
                trans_data[1] = node_trans[1, i];
                trans_data[2] = node_trans[2, i];
                trans_data[3] = node_trans[3, i];
                //右乘初始矩阵的逆进行转换
                double[] cor_data = computation.Quaternioncov(trans_data, norm_data);
                double norm_c = Math.Sqrt(Math.Pow(cor_data[0], 2) + Math.Pow(cor_data[1], 2) + Math.Pow(cor_data[2], 2) + Math.Pow(cor_data[3], 2));
                cor_data[0] /= norm_c;
                cor_data[1] /= norm_c;
                cor_data[2] /= norm_c;
                cor_data[3] /= norm_c;
                node_rotation.Add(cor_data);
            }
            return node_rotation;
        }

        public List<double[]> angle_update(List<double[]> inputdata, bool type)
        /*设定输入inputdata[4,7]的数据格式为[根,左大腿，左小腿，左踝，右大腿，右小腿，右踝]
        根据节点关系更新对应节点转换关系 [根-左大腿，左小腿-左大腿，左踝-左小腿，根-右大腿，右小腿-右大腿，右踝-右小腿]
        type设置为T:返回值四元数[4,6]
        type设置为F:返回值zyx欧拉角[3,6]
        */
        {
            angle.Clear();
            for (int i = 0; i < inputdata.Count() - 1; i++)
            {
                //double[] c_trans = computation.Quaternioncov(inputdata[i+1],new double[]{0.5,0.5,-0.5,-0.5});
                //double[] c_T = computation.Quaterniontranspose(c_trans);
                //double[] f_trans = computation.Quaternioncov(inputdata[i],new double[]{0.5,-0.5,-0.5,0.5});

                double[] f_trans = computation.Quaterniontranspose(inputdata[i]);
                double[] a = computation.Quaternioncov(f_trans, inputdata[i + 1]);//此处进行转换计算换成图形中的坐标系
                angle.Add(a);
            }
            if (type == true) { return angle; }
            else
            {
                List<double[]> euler_angle = new List<double[]>();
                for (int i = 0; i < angle.Count(); i++)
                {
                    double[,] matrix = computation.Quaternion2matrix(angle[i]);
                    double[] euler = computation.Quaternion2euler(matrix);
                    euler_angle.Add(euler);
                }
                return euler_angle;
            }
        }
        public int SubstringCount(string str, string substring)
        {
            if (str.Contains(substring))
            {
                string strReplaced = str.Replace(substring, "");
                return (str.Length - strReplaced.Length) / substring.Length;
            }

            return 0;
        }
        /// <summary>
        /// 计算特定字符串在某字符串中出现第N次位置
        /// </summary>
        /// <param name="str">源字符串</param>
        /// <param name="customChar">特定字符串</param>
        /// <param name="nIndex">特定字符串出现的第N次</param>
        /// <returns></returns>
        public int IndexOfCustomChar(string str, char customChar, int nIndex)
        {
            int num = 0;
            for (int i = 0; i < str.Length; i++)
            {
                if (str[i] == customChar)
                {
                    num++;
                    if (num == nIndex)
                    {
                        return i;
                    }
                }
            }
            return -1;
        }
        public string get_string(string str, int start, int finish)
        {
            string b = "";
            if (finish > start)
            {
                int len = finish - start - 1;

                for (int i = 0; i < len; i++)
                {
                    b += str[start + 1 + i];
                }
                return b;
            }
            else
            {
                string c = "flase";
                return c;
            }
        }
        public void SaveToFile(List<double[]> angle, string txt)
        {
            int totalnum = angle.Count();
            string file_path = "E:\\butaidata\\" + txt + ".csv";
            if (totalnum > 0)
            {
                if (!File.Exists(file_path))
                {
                    File.Create(file_path).Close();
                }
                else
                {
                    File.Delete(file_path);
                }
                StreamWriter sw = new StreamWriter(file_path, true);
                /*写入表头
                sw.Write("Time" + ",");
                sw.Write("roll" + ",");
                sw.Write("pitch" + ",");
                sw.Write("yaw" + ",");
                sw.Write("\r\n");*/
                //写入数据
                for (int i = 0; i < totalnum; i++)
                {
                    //sw.Write(i - start_point + ",");
                    for (int j = 0; j < angle[i].GetLength(0); j++)
                    {
                        sw.Write(angle[i][j] + ",");
                    }
                    sw.Write("\r\n");
                }
                sw.Flush();
                sw.Close();
            }

        }
    }
    class math_computation
    {
        public double[,] Quaternion2matrix(double[] inputdata)// 转换成旋转矩阵
        {
            double[,] trans = new double[3, 3];
            trans[0, 0] = Math.Pow(inputdata[0], 2) + Math.Pow(inputdata[1], 2) - Math.Pow(inputdata[2], 2) - Math.Pow(inputdata[3], 2);
            trans[0, 1] = 2 * ((inputdata[1] * inputdata[2]) - (inputdata[0] * inputdata[3]));
            trans[0, 2] = 2 * ((inputdata[1] * inputdata[3]) + (inputdata[0] * inputdata[2]));

            trans[1, 0] = 2 * ((inputdata[1] * inputdata[2]) + (inputdata[0] * inputdata[3]));
            trans[1, 1] = Math.Pow(inputdata[0], 2) + Math.Pow(inputdata[2], 2) - Math.Pow(inputdata[1], 2) - Math.Pow(inputdata[3], 2);
            trans[1, 2] = 2 * ((inputdata[2] * inputdata[3]) - (inputdata[0] * inputdata[1]));

            trans[2, 0] = 2 * ((inputdata[1] * inputdata[3]) - (inputdata[0] * inputdata[2]));
            trans[2, 1] = 2 * ((inputdata[2] * inputdata[3]) + (inputdata[0] * inputdata[1]));
            trans[2, 2] = Math.Pow(inputdata[0], 2) + Math.Pow(inputdata[3], 2) - Math.Pow(inputdata[1], 2) - Math.Pow(inputdata[2], 2);
            return trans;
        }
        public double[] Quaternion2euler(double[,] inputdata)// 转换成旋转欧拉角
        {
            double[] trans = new double[3];
            //xyz
            //roll
            trans[0] = Math.Atan(-1 * inputdata[1, 2] / inputdata[2, 2]) * (180 / Math.PI);
            //Console.Write("(x)roll:{0},\n", trans[0]);
            //pitch
            trans[1] = Math.Asin(inputdata[0, 2]) * (180 / Math.PI);//pitch
            //Console.Write("(y)pitch:{0},\n", trans[1]);
            //yaw
            trans[2] = Math.Atan(-1 * inputdata[0, 1] / inputdata[0, 0]) * (180 / Math.PI);
           // Console.Write("(z)yaw:{0},\n", trans[2]);
            /*/yxz
            //roll
            trans[0] = Math.Asin(-1*inputdata[1, 2]) * (180 / Math.PI);
            //pitch
            trans[1] = Math.Atan2(inputdata[2, 2],inputdata[0, 2]) * (180 / Math.PI);//pitch
            /yaw
            trans[2] = Math.Atan2(inputdata[1, 1], inputdata[1, 0]) * (180 / Math.PI);*/
            return trans;
        }
        public double[] Quaternioncov(double[] inputdata, double[] covdata)//四元数乘法 inputdata*covdata,先旋转covdata再旋转inputdata

        {
            /*
             * covdata=(q_0,q_1,q_2,q_3)
             * inputdata=(p_0,p_1,p_2,p_3)
             * inputdata*covdata=
             * [p_0,-p_1,-p_2,-p_3; [q_0;
             *  p_1, p_0,-p_3, p_2;  q_1;
             *  p_2, p_3, p_0,-p_1;  q_2;
             *  p_3,-p_2, p_1, p_0]  q_3]
             */
            double[] trans = new double[4];
            trans[0] = inputdata[0] * covdata[0] - inputdata[1] * covdata[1] - inputdata[2] * covdata[2] - inputdata[3] * covdata[3];
            trans[1] = inputdata[1] * covdata[0] + inputdata[0] * covdata[1] - inputdata[3] * covdata[2] + inputdata[2] * covdata[3];
            trans[2] = inputdata[2] * covdata[0] + inputdata[3] * covdata[1] + inputdata[0] * covdata[2] - inputdata[1] * covdata[3];
            trans[3] = inputdata[3] * covdata[0] - inputdata[2] * covdata[1] + inputdata[1] * covdata[2] + inputdata[0] * covdata[3];
            double norm_data = Math.Sqrt(Math.Pow(trans[0], 2) + Math.Pow(trans[1], 2) +
                                         Math.Pow(trans[2], 2) + Math.Pow(trans[3], 2));
            trans[0] /= norm_data;
            trans[1] /= norm_data;
            trans[2] /= norm_data;
            trans[3] /= norm_data;
            return trans;
        }
        public double[] Quaterniontranspose(double[] inputdata)//四元数转置

        {
            double[] trans = new double[4];
            trans[0] = inputdata[0];
            trans[1] = (-1) * inputdata[1];
            trans[2] = (-1) * inputdata[2];
            trans[3] = (-1) * inputdata[3];
            return trans;
        }
        public double[] Quaternionnorm(double[] inputdata)// 四元数归一化
        {
            double trans = 0;
            for (int i = 0; i < inputdata.GetLength(0); i++)
            {
                trans += inputdata[i];
            }
            for (int i = 0; i < inputdata.GetLength(0); i++)
            {
                inputdata[i] /= trans;
            }
            return inputdata;
        }
        public double[] vectormutiply(double[] inputdata, double[] convdata)//向量叉乘

        {
            double[] trans = new double[3];
            trans[0] = (inputdata[1] * convdata[2]) - (inputdata[2] * convdata[1]);
            trans[1] = (inputdata[2] * convdata[0]) - (inputdata[0] * convdata[2]);
            trans[2] = (inputdata[0] * convdata[1]) - (inputdata[1] * convdata[0]);
            return trans;
        }
        public double[] vectorcount(double[] inputdata, double[] convdata)//向量叉乘

        {
            double[] trans = new double[3];
            trans[0] = inputdata[0] + convdata[0];
            trans[1] = inputdata[1] + convdata[1];
            trans[2] = inputdata[2] + convdata[2];
            return trans;
        }
    }
    class read_data
    {
        private DateTime TimeStart = DateTime.Now;
        //double T;//左脚上、下惯导角度，温度
        byte[] RxBuffer = new byte[99];
        UInt16 usRxLength = 0;
        Stopwatch st = new Stopwatch();
        public double[] MQ4 = new double[4];
        public double[] MQ4_2 = new double[4];
        public double[] sensor_Data = new double[8];
        public SerialPort SerialPort1 = new SerialPort("COM5");//根据电脑串口

        public void btnOpenCom_Click()
        {
            if (SerialPort1.IsOpen == false)
            {
                SerialPort1.BaudRate = 921600;
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
            byte[] byteTemp = new byte[99]; //申请存放数据的缓冲区           
            UInt16 usLength = 0;
            //serialPort1.Read(Rcedata, 0, serialPort1.BytesToRead);//读取数据到data中 
            usLength = (UInt16)SerialPort1.Read(RxBuffer, usRxLength, 66);
            usRxLength += usLength;
            while (usRxLength >= 29)
            {
                RxBuffer.CopyTo(byteTemp, 0);
                if (!((byteTemp[28] == 0xBB) && (byteTemp[0] == 0xAA) && (((byteTemp[1] & 0x0F) == 0x01) || ((byteTemp[1] & 0x0F) == 0x02) || ((byteTemp[1] & 0x0F) == 0x00))))
                {
                    for (int i = 1; i < usRxLength; i++) RxBuffer[i - 1] = RxBuffer[i];
                    usRxLength--;
                    continue;
                }
                //if (((byteTemp[0] + byteTemp[1] + byteTemp[2] + byteTemp[3] + byteTemp[4] + byteTemp[5] + byteTemp[6] + byteTemp[7] + byteTemp[8] + byteTemp[9]) & 0xff) == byteTemp[10])
                if ((byteTemp[28] == 0xBB) && (byteTemp[0] == 0xAA))
                    sensor_Data = DecodeData3(byteTemp);
                for (int i = 29; i < usRxLength; i++) RxBuffer[i - 29] = RxBuffer[i];
                usRxLength -= 29;
            }
            SerialPort1.DiscardInBuffer();
        }
        private double[] DecodeData3(byte[] byteTemp)
        {
            double[] Data = new double[12];
            double[] out_data = new double[8];
            double TimeElapse = (DateTime.Now - TimeStart).TotalMilliseconds / 1000;
            Data[0] = (short)((byteTemp[2] << 8) + byteTemp[3]); //mq1
            Data[1] = (short)((byteTemp[4] << 8) + byteTemp[5]); //mq2
            Data[2] = (short)((byteTemp[6] << 8) + byteTemp[7]); //mq3
            Data[3] = (short)((byteTemp[8] << 8) + byteTemp[9]); //mq4
            Data[4] = (short)((byteTemp[10] << 8) + byteTemp[11]); //Lq1 
            Data[5] = (short)((byteTemp[12] << 8) + byteTemp[13]); //Lq2
            Data[6] = (short)((byteTemp[14] << 8) + byteTemp[15]); //Lq3
            Data[7] = (short)((byteTemp[16] << 8) + byteTemp[17]); //Lq4
            if (byteTemp[18] == 0)
            {
                for (int i = 8; i < 12; i++)
                {
                    Data[i] = 0;
                }
            }
            else
            {
                Data[8] = (short)((byteTemp[19] << 8) + byteTemp[20]); //Rq1
                Data[9] = (short)((byteTemp[21] << 8) + byteTemp[22]); //Rq2
                Data[10] = (short)((byteTemp[23] << 8) + byteTemp[24]); //Rq3
                Data[11] = (short)((byteTemp[25] << 8) + byteTemp[26]); //Rq1
            }
            Data[0] /= 32768;  //q0
            Data[1] /= 32768;  //q1
            Data[2] /= 32768;  //q2
            Data[3] /= 32768;  //q3
            Data[4] /= 32768;  //q0_2
            Data[5] /= 32768;  //q1_2
            Data[6] /= 32768;  //q2_2
            Data[7] /= 32768;  //q3_2
            double norm_data = Math.Sqrt((Math.Pow(Data[0], 2) + Math.Pow(Data[1], 2) + Math.Pow(Data[2], 2) + Math.Pow(Data[3], 2)));
            out_data[0] = Data[0] / norm_data;
            out_data[1] = Data[1] / norm_data;
            out_data[2] = Data[2] / norm_data;
            out_data[3] = Data[3] / norm_data;
            double norm_data2 = Math.Sqrt((Math.Pow(Data[4], 2) + Math.Pow(Data[5], 2) + Math.Pow(Data[6], 2) + Math.Pow(Data[7], 2)));
            out_data[4] = Data[4] / norm_data2;
            out_data[5] = Data[5] / norm_data2;
            out_data[6] = Data[6] / norm_data2;
            out_data[7] = Data[7] / norm_data2;
            return out_data;
        }
    }
}

