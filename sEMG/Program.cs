using System;
using System.Collections.Generic;
using System.Threading;
using IMU_pose_process;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.IntegralTransforms;
using semg_data_process;
using MathNet.Filtering;


namespace ConsoleApp1
{ 
 class Program
{
        private enum sample_mode
  {
            semg = 0,
            imu = 1,
            semg_and_imu=2
  }
        static void Main(string[] args)
        {
            sample_mode mode;

            mode = sample_mode.semg;         
            //Console.WriteLine(aaa);
            math_computation data = new math_computation();
            filter_methods data_filter = new filter_methods();
            Console.WriteLine("保持静息2s");
            if (mode == 0)
            {
                read_semg_data sEMG_data = new read_semg_data();
                sEMG_data.btnOpenCom_Click();
                while (true)
                {
                    sEMG_data.SerialPort1_DataReceived();
                    if (sEMG_data.EmgRecords[0].Count >= 2000)
                    {
                        MathNet.Numerics.Complex32[] mathNetComplexArrRe = new MathNet.Numerics.Complex32[64];
                        float[] filter_data = new float[] { 0, 3, 2, 5, 3, -7, -6, -9, -5, -13, -12, -15, -13, 17, 6, 19, 10, 13, 22, 22, 3, 27, 36, 19, 25, 13, 52, 45, 33, 22, 6, 19, 0, 3, 2, 5, 3, -7, -6, -9, -5, -13, -12, -15, -13, 17, 6, 19, 10, 13, 22, 22, 3, 27, 36, 19, 25, 13, 52, 45, 33, 22, 6, 19 };
                        float[] resultArr = new float[64];
                    }
                }

            }
            #region
            //double[] a_init = new double[4] {1,0,0,0};
            //double[] b_init = new double[4] { 0.5,-0.5, - 0.5, -0.5 };
            //double[] a =new double[4] { Math.Cos(Math.PI / 4), Math.Cos(Math.PI / 4), 0, 0 };
            //double[] b= new double[4] { 0, -1*Math.Cos(Math.PI / 4), 0, -1 * Math.Cos(Math.PI / 4)};
            //double[] a1 = new double[4] { Math.Cos(Math.PI / 4), 0, Math.Cos(Math.PI / 4), 0 };
            //double[] b1 = new double[4] {0, -1 * Math.Cos(Math.PI / 4), -1 * Math.Cos(Math.PI / 4),0};
            //double[] a_T = data.Quaterniontranspose(a_init);
            //double[] b_T = data.Quaterniontranspose(b);
            //double[] b1_T = data.Quaterniontranspose(b1);          
            //double[] trans_A_1 = data.Quaternioncov(a_T,a);
            //double[] trans_B_1 = data.Quaternioncov(b_init, b_T);
            //double[] trans_A_2 = data.Quaternioncov(a_T, a1);
            //double[] trans_B_2 = data.Quaternioncov(b_init, b1_T);
            //double[] erfa1 = new double[3];
            //double[] erfa2 = new double[3];
            //double norm_1 = Math.Sqrt(Math.Pow(trans_A_1[1], 2) + Math.Pow(trans_A_1[2], 2) + Math.Pow(trans_A_1[3], 2));
            //double norm_2 = Math.Sqrt(Math.Pow(trans_A_2[1], 2) + Math.Pow(trans_A_2[2], 2) + Math.Pow(trans_A_2[3], 2));
            //erfa1[0] = trans_A_1[1] / norm_1; erfa1[1] = trans_A_1[2] / norm_1; erfa1[2] = trans_A_1[3] / norm_1;
            //erfa2[0] = trans_A_2[1] / norm_2; erfa2[1] = trans_A_2[2] / norm_2; erfa2[2] = trans_A_2[3] / norm_2;
            //double[] bita1 = new double[3];
            //double[] bita2 = new double[3];
            //double norm_3 = Math.Sqrt(Math.Pow(trans_B_1[1], 2) + Math.Pow(trans_B_1[2], 2) + Math.Pow(trans_B_1[3], 2));
            //double norm_4 = Math.Sqrt(Math.Pow(trans_B_2[1], 2) + Math.Pow(trans_B_2[2], 2) + Math.Pow(trans_B_2[3], 2));
            //bita1[0] = trans_B_1[1] / norm_3; bita1[1] = trans_B_1[2] / norm_3; bita1[2] = trans_B_1[3] / norm_3;
            //bita2[0] = trans_B_2[1] / norm_4; bita2[1] = trans_B_2[2] / norm_4; bita2[2] = trans_B_2[3] / norm_4;
            //double[] V2_1 = data.vectormutiply(erfa1, bita1);
            //double[] V2_1_1 = data.vectorcount(erfa1, bita1);

            //double m2_1 = Math.Sqrt(2 + 2 * ((erfa1[0] * bita1[0]) + (erfa1[1] * bita1[1]) + (erfa1[2] * bita1[2])));
            //var v1 = DenseMatrix.OfArray(new double[4, 2] { { -0.5 * m2_1, 0 }, { V2_1[0] / m2_1, V2_1_1[0] / m2_1 }, { V2_1[1] / m2_1, V2_1_1[1] / m2_1 }, { V2_1[2] / m2_1, V2_1_1[2] / m2_1 } });
            //Console.Write("第一次V2：\n{0}\n", v1.ToString());          
            //double[] V2_2_1 = data.vectorcount(erfa2, bita2);
            //double[] V2_2 = data.vectormutiply(erfa2, bita2);
            //double m2_2 = Math.Sqrt(2 + 2 * ((erfa2[0] * bita2[0]) + (erfa2[1] * bita2[1]) + (erfa2[2] * bita2[2])));
            //var v2 = DenseMatrix.OfArray(new double[4, 2] { { -0.5 * m2_2, 0 }, { V2_2[0] / m2_2, V2_2_1[0] / m2_2 }, { V2_2[1] / m2_2, V2_2_1[1] / m2_2 }, { V2_2[2] / m2_2, V2_2_1[2] / m2_2 } });
            //Console.Write("第二次V2：{0}\n", v2.ToString());

            //var W = (v1.Transpose() * v2).ToArray(); 
            // Console.Write("W：{0}\n", (v1.Transpose() * v2).ToString());
            //double aa = 1 - Math.Pow(W[0, 0], 2) - Math.Pow(W[1, 0], 2);
            //double bb = 1 - Math.Pow(W[0, 1], 2) - Math.Pow(W[1, 1], 2);
            //double c = -1 * (W[0, 0] * W[0, 1] + W[1, 0] * W[1, 1]);
            //double a_c = Math.Sqrt(Math.Pow(c, 2) + Math.Pow(aa, 2));
            //double b_c = Math.Sqrt(Math.Pow(c, 2) + Math.Pow(bb, 2));
            //double[] y = new double[2];
            //if ((aa == bb) && (bb == c))
            //{
            //    Console.Write("error\n");
            //}
            //else if (Math.Abs(aa) >= Math.Abs(aa))
            //{
            //    y[0] = c / a_c;
            //    y[1] = -1 * aa / a_c;
            //}
            //else if (Math.Abs(aa) < Math.Abs(bb))
            //{
            //    y[0] = bb / b_c;
            //    y[1] = -1 * c/b_c;
            //}
            //var v = Vector<double>.Build.DenseOfArray(y);
            //var p_x = v2 * v;
            //var Hx = p_x.ToArray();
            //double cc = Math.Sqrt(Math.Pow(Hx[0], 2) + Math.Pow(Hx[1], 2) + Math.Pow(Hx[2], 2) + Math.Pow(Hx[3], 2));
            //Console.Write("修正四元数为{0}\t{1}\t{2}\t{3}\n归一化系数：{4}\n", Hx[0], Hx[1], Hx[2], Hx[3], cc);
            //Hx[0] /= cc; Hx[1] /= cc; Hx[2] /= cc; Hx[3] /= cc;
            //Console.Write("归一化后修正四元数为{0}\t{1}\t{2}\t{3}\n", Hx[0], Hx[1], Hx[2], Hx[3]);
            //double[] out_A = data.Quaternioncov(trans_A_1, Hx);
            //double[] out_B = data.Quaternioncov( Hx, trans_B_1);
            //double[] out_A_1 = data.Quaternioncov(trans_A_2, Hx);
            //double[] out_B_1 = data.Quaternioncov(Hx, trans_B_2);
            //Console.Write("第一次计算结果\nHl*HX:{0},{1},{2},{3}\n HX*Hc:{4},{5},{6},{7}\n", out_A[0], out_A[1], out_A[2], out_A[3], out_B[0], out_B[1], out_B[2], out_B[3]);
            //Console.Write("第二次计算结果\nHl*HX:{0},{1},{2},{3}\n HX*Hc:{4},{5},{6},{7}\n", out_A_1[0], out_A_1[1], out_A_1[2], out_A_1[3], out_B_1[0], out_B_1[1], out_B_1[2], out_B_1[3]);
            //var martixA = DenseMatrix.OfArray(new double[,]//姿态0
            //             {
            //            { 0, -1 * (erfa1[0] - bita1[0]), -1*(erfa1[1] - bita1[1]), -1*(erfa1[2] -bita1[2]) },
            //            { (erfa1[0] - bita1[0]), 0, -1 *(erfa1[2] + bita1[2]), (erfa1[1] + bita1[1]) },
            //            { (erfa1[1] - bita1[1]), (erfa1[2] + bita1[2]), 0, -1 * (erfa1[0] + bita1[0]) },
            //            { (erfa1[2] - bita1[2]), -1 * (erfa1[1] + bita1[1]), (erfa1[0] + bita1[0]), 0 },});
            //var e_vule = martixA.Svd().S.ToString();
            //Console.Write("矩阵特征值{0}\n",e_vule);
            //Console.ReadLine();
            #endregion
        }
    }
}
