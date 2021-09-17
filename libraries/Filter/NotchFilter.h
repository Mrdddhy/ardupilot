/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

/*
  notch filter with settable sample rate, center frequency, bandwidth and attenuation

  Design by Leonard Hall
 */

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <inttypes.h>
#include <AP_Param/AP_Param.h>


template <class T>
class NotchFilter {
public:
    // set parameters
    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
    void init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q);
    T apply(const T &sample);
    void reset();

    // calculate attenuation and quality from provided center frequency and bandwidth
    // 根据给定的中心频率和带宽来计算衰减分贝A和Q因子
    static void calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float& A, float& Q); 

private:

    bool initialised;
    float b0, b1, b2, a1, a2, a0_inv;//Notch Filter的输出公式中的核心系数
    T ntchsig, ntchsig1, ntchsig2, signal2, signal1;//2阶的Notch Filter的输入输出信号
};

/*
  notch filter enable and filter parameters
 */
class NotchFilterParams {
public:
    NotchFilterParams(void);
    static const struct AP_Param::GroupInfo var_info[];

    float center_freq_hz(void) const { return _center_freq_hz; }
    float bandwidth_hz(void) const { return _bandwidth_hz; }
    float attenuation_dB(void) const { return _attenuation_dB; }
    uint8_t enabled(void) const { return _enable; }
    
protected:
/*预先设定的几个参数*/
    AP_Int8 _enable;  /*该滤波器使能*/
    AP_Float _center_freq_hz;/*想要滤除的信号的中心频率*/
    AP_Float _bandwidth_hz;/*想要滤除的信号的带宽*/
    AP_Float _attenuation_dB;/*想要滤除的信号的分贝*/
};

typedef NotchFilter<float> NotchFilterFloat;
typedef NotchFilter<Vector3f> NotchFilterVector3f;

