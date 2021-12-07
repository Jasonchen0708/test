using System;
using MathNet.Numerics.IntegralTransforms;
using MathNet.Filtering;

namespace ConsoleApp1
{
    class filter_methods
    {
        /// <summary>
        /// 取静息状态的能量谱
        /// </summary>
        /// <param name="inData"></param>
        /// <param name="window_length"></param>
        /// <returns name="nose_energy"></returns>
        public double[] get_online_nose(double[] inData,int window_length=128)
        {

            int data_length = inData.Length;
            double[] nose_energy = new double[window_length];
            int n = (int)(data_length / window_length);
            MathNet.Numerics.Complex32[] indata_fft = new MathNet.Numerics.Complex32[data_length];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < window_length; j++)
            {
                indata_fft[j] = new MathNet.Numerics.Complex32((float)inData[i*window_length+j], 0);
            }
                Fourier.Forward(indata_fft);
                for (int m = 0; m < window_length; m++)
                {
                    nose_energy[m] += (indata_fft[m].Real * indata_fft[m].Real) + (indata_fft[m].Imaginary * indata_fft[m].Imaginary);
                }
            }
            for (int j = 0; j < window_length; j++)
            {
                nose_energy[j] /= n;
            }
            return nose_energy;
        }

        /// <summary> 离线滤波算法，实现信号的谱减法滤波默认每128ms进行滤波
        /// <param name="inData"></待滤波信号>
        /// <param name="base_nose"></静息信号的能量谱>
        /// <returns name="outArr"> 返回滤波后信号
        /// </summary>
        public double[] offline_denose(double[] inData,double[] base_nose,int window_length = 128)
        {
            int data_length = inData.Length;
            float trans_real;
            float trans_imag;
            double inData_pow2;
            double[] outArr = new double[data_length];
            //outArr = inData;
            MathNet.Numerics.Complex32[] indata_fft = new MathNet.Numerics.Complex32[data_length];
            MathNet.Numerics.Complex32[] ArrFreq = new MathNet.Numerics.Complex32[data_length];
           // double[] ArrFreq = new double[data_length];
            //MathNet.Numerics.Complex32[] base_nose_fft = new MathNet.Numerics.Complex32[data_length];
            for (int i = 0; i < indata_fft.Length; i++)
            {
                indata_fft[i] = new MathNet.Numerics.Complex32((float)inData[i], 0);            
            }
            Fourier.Forward(indata_fft);//傅里叶变换
            for (int i = 0; i < ArrFreq.Length; i++)
            {
                inData_pow2 = (indata_fft[i].Real* indata_fft[i].Real)+(indata_fft[i].Imaginary * indata_fft[i].Imaginary);
                if (inData_pow2 - 1.5* base_nose[i] >0) // 判断谱能量是否超过阈值，若未超过阈值进行削减
                {
                    trans_real = (float)Math.Sqrt(inData_pow2 - base_nose[i]);
                }
                else
                {
                    trans_real = (float)(0.1 * base_nose[i]);
                }               
                trans_imag = trans_real * ((float)(indata_fft[i].Real/ indata_fft[i].Imaginary));
                ArrFreq[i] = new MathNet.Numerics.Complex32(trans_real, trans_imag);
            }
            Fourier.Inverse(ArrFreq);//逆傅里叶变换
            for (int i = 0; i < ArrFreq.Length; i++)
            {
                outArr[i] = ArrFreq[i].Real;
            }
            return outArr;
        }
        /// <summary>
        /// 6阶带通滤波
        /// </summary>
        /// <param name="inData"> 待处理数据</param>
        /// <param name="order"> 滤波器阶数</param>
        /// <returns name="denose"> </returns>
        public double[] Butterworth_transform(double[] inData,int order)
        {
            double fs = 1000; //sampling rate
            double fc1 = 20; //low cutoff frequency
            double fc2 = 500; //high cutoff frequency
            var bandpassnarrow = OnlineFilter.CreateBandpass(ImpulseResponse.Infinite, fs, fc1, fc2, order);
            double[] denose = bandpassnarrow.ProcessSamples(inData); //Bandpass Narrow
            return denose;
        }
    }
}
