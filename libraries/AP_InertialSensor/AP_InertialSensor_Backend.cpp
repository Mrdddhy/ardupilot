#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#if AP_MODULE_SUPPORTED
#include <AP_Module/AP_Module.h>
#include <stdio.h>
#endif

#define SENSOR_RATE_DEBUG 0

const extern AP_HAL::HAL& hal;

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &imu) :
    _imu(imu)
{
}

/*
  notify of a FIFO reset so we don't use bad data to update observed sensor rate
 */
void AP_InertialSensor_Backend::notify_accel_fifo_reset(uint8_t instance)
{
    _imu._sample_accel_count[instance] = 0;
    _imu._sample_accel_start_us[instance] = 0;    
}

/*
  notify of a FIFO reset so we don't use bad data to update observed sensor rate
 */
void AP_InertialSensor_Backend::notify_gyro_fifo_reset(uint8_t instance)
{
    _imu._sample_gyro_count[instance] = 0;
    _imu._sample_gyro_start_us[instance] = 0;
}

// set the amount of oversamping a accel is doing
void AP_InertialSensor_Backend::_set_accel_oversampling(uint8_t instance, uint8_t n)
{
    _imu._accel_over_sampling[instance] = n;
}

// set the amount of oversamping a gyro is doing
void AP_InertialSensor_Backend::_set_gyro_oversampling(uint8_t instance, uint8_t n)
{
    _imu._gyro_over_sampling[instance] = n;
}

/*
  update the sensor rate for FIFO sensors
*更新FIFO传感器的速率
  FIFO sensors produce samples at a fixed rate, but the clock in the
  sensor may vary slightly from the system clock. This slowly adjusts
  the rate to the observed rate
*先进先出传感器以固定的速率产生样本，但是传感器时钟可能与系统时钟略有变化，
 这慢慢调整速率与观测速率之比
*/
void AP_InertialSensor_Backend::_update_sensor_rate(uint16_t &count, uint32_t &start_us, float &rate_hz) const
{
    uint32_t now = AP_HAL::micros();/*获取当前的时间*/
    if (start_us == 0) {
        count = 0;
        start_us = now;
    } else {
        count++;
        if (now - start_us > 1000000UL) {
            float observed_rate_hz = count * 1.0e6f / (now - start_us);
#if SENSOR_RATE_DEBUG
            printf("RATE: %.1f should be %.1f\n", observed_rate_hz, rate_hz);
#endif
            float filter_constant = 0.98f;
            float upper_limit = 1.05f;
            float lower_limit = 0.95f;
            if (sensors_converging()) {
                // converge quickly for first 30s, then more slowly
                filter_constant = 0.8f;
                upper_limit = 2.0f;
                lower_limit = 0.5f;
            }
            observed_rate_hz = constrain_float(observed_rate_hz, rate_hz*lower_limit, rate_hz*upper_limit);
            rate_hz = filter_constant * rate_hz + (1-filter_constant) * observed_rate_hz;
            count = 0;
            start_us = now;
        }
    }
}

/*函数功能：旋转和校准加速度
  输入参数1：加速度计实例
  输入参数2：加速度计数据：单位为g
  */
void AP_InertialSensor_Backend::_rotate_and_correct_accel(uint8_t instance, Vector3f &accel) 
{
    /*
      accel calibration is always done in sensor frame with this
      version of the code. That means we apply the rotation after the
      offsets and scaling.
      这版代码里面加速度计校准总是要做的，这就意味着我们在做完偏移和尺度校准之后再进行旋转
     */

    // rotate for sensor orientation
    // 对传感器方向做旋转
    accel.rotate(_imu._accel_orientation[instance]);
    
    // apply offsets
    //减掉偏差值
    accel -= _imu._accel_offset[instance];

    // apply scaling
    // 乘以尺度因子
    const Vector3f &accel_scale = _imu._accel_scale[instance].get();
    accel.x *= accel_scale.x;
    accel.y *= accel_scale.y;
    accel.z *= accel_scale.z;

    // rotate to body frame
    //旋转到机体系
    if (_imu._board_orientation == ROTATION_CUSTOM && _imu._custom_rotation) {
        accel = *_imu._custom_rotation * accel;
    } else {
        accel.rotate(_imu._board_orientation);
    }
}

void AP_InertialSensor_Backend::_rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro) 
{
    // rotate for sensor orientation
    gyro.rotate(_imu._gyro_orientation[instance]);
    
    // gyro calibration is always assumed to have been done in sensor frame
    gyro -= _imu._gyro_offset[instance];

    if (_imu._board_orientation == ROTATION_CUSTOM && _imu._custom_rotation) {
        gyro = *_imu._custom_rotation * gyro;
    } else {
        gyro.rotate(_imu._board_orientation);
    }
}

/*
  rotate gyro vector and add the gyro offset
 */
void AP_InertialSensor_Backend::_publish_gyro(uint8_t instance, const Vector3f &gyro)
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    _imu._gyro[instance] = gyro;
    _imu._gyro_healthy[instance] = true;

    // publish delta angle
    _imu._delta_angle[instance] = _imu._delta_angle_acc[instance];
    _imu._delta_angle_dt[instance] = _imu._delta_angle_acc_dt[instance];
    _imu._delta_angle_valid[instance] = true;
}

void AP_InertialSensor_Backend::_notify_new_gyro_raw_sample(uint8_t instance,
                                                            const Vector3f &gyro,
                                                            uint64_t sample_us)
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    float dt;

    _update_sensor_rate(_imu._sample_gyro_count[instance], _imu._sample_gyro_start_us[instance],
                        _imu._gyro_raw_sample_rates[instance]);

    uint64_t last_sample_us = _imu._gyro_last_sample_us[instance];

    /*
      we have two classes of sensors. FIFO based sensors produce data
      at a very predictable overall rate, but the data comes in
      bunches, so we use the provided sample rate for deltaT. Non-FIFO
      sensors don't bunch up samples, but also tend to vary in actual
      rate, so we use the provided sample_us to get the deltaT. The
      difference between the two is whether sample_us is provided.
     */
    if (sample_us != 0 && _imu._gyro_last_sample_us[instance] != 0) {
        dt = (sample_us - _imu._gyro_last_sample_us[instance]) * 1.0e-6f;
        _imu._gyro_last_sample_us[instance] = sample_us;
    } else {
        // don't accept below 100Hz
        if (_imu._gyro_raw_sample_rates[instance] < 100) {
            return;
        }

        dt = 1.0f / _imu._gyro_raw_sample_rates[instance];
        _imu._gyro_last_sample_us[instance] = AP_HAL::micros64();
    }

#if AP_MODULE_SUPPORTED
    // call gyro_sample hook if any
    AP_Module::call_hook_gyro_sample(instance, dt, gyro);
#endif

    // push gyros if optical flow present
    if (hal.opticalflow) {
        hal.opticalflow->push_gyro(gyro.x, gyro.y, dt);
    }
    
    // compute delta angle
    /*计算增量角度*/
    Vector3f delta_angle = (gyro + _imu._last_raw_gyro[instance]) * 0.5f * dt;

    // compute coning correction(计算锥进校正)
    // see page 26 of:
    // Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
    // Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
    // see also examples/coning.py
    Vector3f delta_coning = (_imu._delta_angle_acc[instance] +
                             _imu._last_delta_angle[instance] * (1.0f / 6.0f));
    delta_coning = delta_coning % delta_angle;
    delta_coning *= 0.5f;

    {
        WITH_SEMAPHORE(_sem);
        uint64_t now = AP_HAL::micros64();

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            _imu._delta_angle_acc[instance].zero();
            _imu._delta_angle_acc_dt[instance] = 0;
            dt = 0;
            delta_angle.zero();
        }

        // integrate delta angle accumulator
        // the angles and coning corrections are accumulated separately in the
        // referenced paper, but in simulation little difference was found between
        // integrating together and integrating separately (see examples/coning.py)
        _imu._delta_angle_acc[instance] += delta_angle + delta_coning;
        _imu._delta_angle_acc_dt[instance] += dt;

        // save previous delta angle for coning correction
        _imu._last_delta_angle[instance] = delta_angle;
        _imu._last_raw_gyro[instance] = gyro;

        // apply the low pass filter
        Vector3f gyro_filtered = _imu._gyro_filter[instance].apply(gyro);

        // apply the notch filter
        if (_gyro_notch_enabled()) {
            gyro_filtered = _imu._gyro_notch_filter[instance].apply(gyro_filtered);
        }

        // apply the harmonic notch filter
        if (gyro_harmonic_notch_enabled()) {
            gyro_filtered = _imu._gyro_harmonic_notch_filter[instance].apply(gyro_filtered);
        }

        // if the filtering failed in any way then reset the filters and keep the old value
        if (gyro_filtered.is_nan() || gyro_filtered.is_inf()) {
            _imu._gyro_filter[instance].reset();
            _imu._gyro_notch_filter[instance].reset();
            _imu._gyro_harmonic_notch_filter[instance].reset();
        } else {
            _imu._gyro_filtered[instance] = gyro_filtered;
        }

        _imu._new_gyro_data[instance] = true;
    }

    if (!_imu.batchsampler.doing_post_filter_logging()) {
        log_gyro_raw(instance, sample_us, gyro);
    }
    else {
        log_gyro_raw(instance, sample_us, _imu._gyro_filtered[instance]);
    }
}

void AP_InertialSensor_Backend::log_gyro_raw(uint8_t instance, const uint64_t sample_us, const Vector3f &gyro)
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        // should not have been called
        return;
    }
    if (should_log_imu_raw()) {
        uint64_t now = AP_HAL::micros64();
        struct log_GYRO pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GYR1_MSG+instance)),
            time_us   : now,
            sample_us : sample_us?sample_us:now,
            GyrX      : gyro.x,
            GyrY      : gyro.y,
            GyrZ      : gyro.z
        };
        logger->WriteBlock(&pkt, sizeof(pkt));
    } else {
        if (!_imu.batchsampler.doing_sensor_rate_logging()) {
            _imu.batchsampler.sample(instance, AP_InertialSensor::IMU_SENSOR_TYPE_GYRO, sample_us, gyro);
        }
    }
}

/*
  rotate accel vector, scale and add the accel offset
 */
void AP_InertialSensor_Backend::_publish_accel(uint8_t instance, const Vector3f &accel)
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    _imu._accel[instance] = accel;/*_imu._accel被发布出去，最终被放置在Vector3f _accel[INS_MAX_INSTANCE]*/
    _imu._accel_healthy[instance] = true;

    // publish delta velocity
    /*发布增量速度，增量时间
      设置增量速度有效性
    */
    _imu._delta_velocity[instance] = _imu._delta_velocity_acc[instance];
    _imu._delta_velocity_dt[instance] = _imu._delta_velocity_acc_dt[instance];
    _imu._delta_velocity_valid[instance] = true;


    if (_imu._accel_calibrator != nullptr && _imu._accel_calibrator[instance].get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
        Vector3f cal_sample = _imu._delta_velocity[instance];

        //remove rotation
        cal_sample.rotate_inverse(_imu._board_orientation);

        // remove scale factors
        const Vector3f &accel_scale = _imu._accel_scale[instance].get();
        cal_sample.x /= accel_scale.x;
        cal_sample.y /= accel_scale.y;
        cal_sample.z /= accel_scale.z;
        
        //remove offsets
        cal_sample += _imu._accel_offset[instance].get() * _imu._delta_velocity_dt[instance] ;

        _imu._accel_calibrator[instance].new_sample(cal_sample, _imu._delta_velocity_dt[instance]);
    }
}

/*函数功能：通知数据到前端
  参数1：实例
  参数2：加速度原始数据
  参数3：采样频率us
  参数4：帧同步设置标志
*/
void AP_InertialSensor_Backend::_notify_new_accel_raw_sample(uint8_t instance,
                                                             const Vector3f &accel,
                                                             uint64_t sample_us,
                                                             bool fsync_set)
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    float dt;
    
    /*更新加速度计实例对应的采样起始时间、原始采样频率*/
    _update_sensor_rate(_imu._sample_accel_count[instance], _imu._sample_accel_start_us[instance],
                        _imu._accel_raw_sample_rates[instance]);

    /*更新上一采样时间*/
    uint64_t last_sample_us = _imu._accel_last_sample_us[instance];

    /*
      we have two classes of sensors. FIFO based sensors produce data
      at a very predictable overall rate, but the data comes in
      bunches, so we use the provided sample rate for deltaT. Non-FIFO
      sensors don't bunch up samples, but also tend to vary in actual
      rate, so we use the provided sample_us to get the deltaT. The
      difference between the two is whether sample_us is provided.
      *我们有两类传感器。基于FIFO的传感器产生的数据在一个非常可预测的总体速率，
      但是数据是成群的，所以我们使用提供的样本速率deltaT.
      NON-FIFO传感器不会聚集样本，但也往往在实际速率变化，
      所以我们使用提供的sample_us来获得deltaT,两者之间的区别在于是否提供了sample_us.

      * 参数：sample_us = 0，即采用原始采样方式计算dt
     */
    if (sample_us != 0 && _imu._accel_last_sample_us[instance] != 0) {
        dt = (sample_us - _imu._accel_last_sample_us[instance]) * 1.0e-6f;
        _imu._accel_last_sample_us[instance] = sample_us;
    } else {
        // don't accept below 100Hz
        /*不接受低于100Hz*/
        if (_imu._accel_raw_sample_rates[instance] < 100) {
            return;
        }

        dt = 1.0f / _imu._accel_raw_sample_rates[instance];/*用原始采样速率方式计算增量时间*/
        _imu._accel_last_sample_us[instance] = AP_HAL::micros64();/*将当前时间设置为加速度计上次采样时刻*/
    }

#if AP_MODULE_SUPPORTED
    // call accel_sample hook if any
    AP_Module::call_hook_accel_sample(instance, dt, accel, fsync_set);
#endif    
    
    _imu.calc_vibration_and_clipping(instance, accel, dt);

    {
        WITH_SEMAPHORE(_sem);

        uint64_t now = AP_HAL::micros64();/*获取当前时间*/

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            /*累积器置0如果传感器不健康超过0.1s*/
            _imu._delta_velocity_acc[instance].zero();/*增量速度置0*/
            _imu._delta_velocity_acc_dt[instance] = 0;/*增量速度计算dt=0*/
            dt = 0;
        }
        
        // delta velocity
        _imu._delta_velocity_acc[instance] += accel * dt;/*计算增量速度*/
        _imu._delta_velocity_acc_dt[instance] += dt;/*增量时间计算*/

        _imu._accel_filtered[instance] = _imu._accel_filter[instance].apply(accel);/*加速度数据滤波*/

        if (_imu._accel_filtered[instance].is_nan() || _imu._accel_filtered[instance].is_inf()) {
            _imu._accel_filter[instance].reset();
        }

        _imu.set_accel_peak_hold(instance, _imu._accel_filtered[instance]);/*峰值保持检测*/

        _imu._new_accel_data[instance] = true;/*设置新的加速度计数据标志位*/
    }

    if (!_imu.batchsampler.doing_post_filter_logging()) {
        log_accel_raw(instance, sample_us, accel);
    } else {
        log_accel_raw(instance, sample_us, _imu._accel_filtered[instance]);
    }
}

void AP_InertialSensor_Backend::_notify_new_accel_sensor_rate_sample(uint8_t instance, const Vector3f &accel)
{
    if (!_imu.batchsampler.doing_sensor_rate_logging()) {
        return;
    }

    _imu.batchsampler.sample(instance, AP_InertialSensor::IMU_SENSOR_TYPE_ACCEL, AP_HAL::micros64(), accel);
}

void AP_InertialSensor_Backend::_notify_new_gyro_sensor_rate_sample(uint8_t instance, const Vector3f &gyro)
{
    if (!_imu.batchsampler.doing_sensor_rate_logging()) {
        return;
    }
    _imu.batchsampler.sample(instance, AP_InertialSensor::IMU_SENSOR_TYPE_GYRO, AP_HAL::micros64(), gyro);
}

void AP_InertialSensor_Backend::log_accel_raw(uint8_t instance, const uint64_t sample_us, const Vector3f &accel)
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        // should not have been called
        return;
    }
    if (should_log_imu_raw()) {
        uint64_t now = AP_HAL::micros64();
        struct log_ACCEL pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ACC1_MSG+instance)),
            time_us   : now,
            sample_us : sample_us?sample_us:now,
            AccX      : accel.x,
            AccY      : accel.y,
            AccZ      : accel.z
        };
        logger->WriteBlock(&pkt, sizeof(pkt));
    } else {
        if (!_imu.batchsampler.doing_sensor_rate_logging()) {
            _imu.batchsampler.sample(instance, AP_InertialSensor::IMU_SENSOR_TYPE_ACCEL, sample_us, accel);
        }
    }
}

void AP_InertialSensor_Backend::_set_accel_max_abs_offset(uint8_t instance,
                                                          float max_offset)
{
    _imu._accel_max_abs_offsets[instance] = max_offset;
}

// set accelerometer error_count
void AP_InertialSensor_Backend::_set_accel_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._accel_error_count[instance] = error_count;
}

// set gyro error_count
void AP_InertialSensor_Backend::_set_gyro_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._gyro_error_count[instance] = error_count;
}

// increment accelerometer error_count
void AP_InertialSensor_Backend::_inc_accel_error_count(uint8_t instance)
{
    _imu._accel_error_count[instance]++;
}

// increment gyro error_count
void AP_InertialSensor_Backend::_inc_gyro_error_count(uint8_t instance)
{
    _imu._gyro_error_count[instance]++;
}

// return the requested sample rate in Hz
uint16_t AP_InertialSensor_Backend::get_sample_rate_hz(void) const
{
    // enum can be directly cast to Hz
    return (uint16_t)_imu._sample_rate;
}

/*
  publish a temperature value for an instance
 */
void AP_InertialSensor_Backend::_publish_temperature(uint8_t instance, float temperature)
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    _imu._temperature[instance] = temperature;

#if HAL_HAVE_IMU_HEATER
    /* give the temperature to the control loop in order to keep it constant*/
    if (instance == 0) {
        AP_BoardConfig *bc = AP::boardConfig();
        if (bc) {
            bc->set_imu_temp(temperature);
        }
    }
#endif
}

/*
  common gyro update function for all backends
 */
void AP_InertialSensor_Backend::update_gyro(uint8_t instance)
{    
    WITH_SEMAPHORE(_sem);

    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    if (_imu._new_gyro_data[instance]) {
        _publish_gyro(instance, _imu._gyro_filtered[instance]);
        _imu._new_gyro_data[instance] = false;
    }

    // possibly update filter frequency
    if (_last_gyro_filter_hz != _gyro_filter_cutoff() || sensors_converging()) {
        _imu._gyro_filter[instance].set_cutoff_frequency(_gyro_raw_sample_rate(instance), _gyro_filter_cutoff());
        _last_gyro_filter_hz = _gyro_filter_cutoff();
    }

    // possibly update the harmonic notch filter parameters
    /*有可能更新谐波陷波参数*/
    if (!is_equal(_last_harmonic_notch_bandwidth_hz, gyro_harmonic_notch_bandwidth_hz()) ||
        !is_equal(_last_harmonic_notch_attenuation_dB, gyro_harmonic_notch_attenuation_dB()) ||
        sensors_converging()) {
        _imu._gyro_harmonic_notch_filter[instance].init(_gyro_raw_sample_rate(instance), gyro_harmonic_notch_center_freq_hz(), gyro_harmonic_notch_bandwidth_hz(), gyro_harmonic_notch_attenuation_dB());
        _last_harmonic_notch_center_freq_hz = gyro_harmonic_notch_center_freq_hz();
        _last_harmonic_notch_bandwidth_hz = gyro_harmonic_notch_bandwidth_hz();
        _last_harmonic_notch_attenuation_dB = gyro_harmonic_notch_attenuation_dB();
    } else if (!is_equal(_last_harmonic_notch_center_freq_hz, gyro_harmonic_notch_center_freq_hz())) {
        _imu._gyro_harmonic_notch_filter[instance].update(gyro_harmonic_notch_center_freq_hz());
        _last_harmonic_notch_center_freq_hz = gyro_harmonic_notch_center_freq_hz();
    }
    // possibly update the notch filter parameters
    /*有可能更新陷波参数*/
    if (!is_equal(_last_notch_center_freq_hz, _gyro_notch_center_freq_hz()) ||
        !is_equal(_last_notch_bandwidth_hz, _gyro_notch_bandwidth_hz()) ||
        !is_equal(_last_notch_attenuation_dB, _gyro_notch_attenuation_dB()) ||
        sensors_converging()) {
        _imu._gyro_notch_filter[instance].init(_gyro_raw_sample_rate(instance), _gyro_notch_center_freq_hz(), _gyro_notch_bandwidth_hz(), _gyro_notch_attenuation_dB());
        _last_notch_center_freq_hz = _gyro_notch_center_freq_hz();
        _last_notch_bandwidth_hz = _gyro_notch_bandwidth_hz();
        _last_notch_attenuation_dB = _gyro_notch_attenuation_dB();
    }
}

/*
  common accel update function for all backends
  通用的accel更新功能为所有后端
 */
void AP_InertialSensor_Backend::update_accel(uint8_t instance)
{    
    WITH_SEMAPHORE(_sem);

    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    /*如果有新的加速度计数据标志位
      发布滤波的数据并且修改标志位
    */
    if (_imu._new_accel_data[instance]) {
        _publish_accel(instance, _imu._accel_filtered[instance]);/*发布加速度计数据，滤波后的*/
        _imu._new_accel_data[instance] = false;
    }
    
    // possibly update filter frequency
    // 可能更新滤波频率
    /*如果加速度计滤波的截止频率不等于上次加速度计的滤波频率
      设置对应的截止频率并将当前的截止频率赋值给上次的加速度计滤波频率
    */
    if (_last_accel_filter_hz != _accel_filter_cutoff()) {
        _imu._accel_filter[instance].set_cutoff_frequency(_accel_raw_sample_rate(instance), _accel_filter_cutoff());
        _last_accel_filter_hz = _accel_filter_cutoff();
    }
}

bool AP_InertialSensor_Backend::should_log_imu_raw() const
{
    if (_imu._log_raw_bit == (uint32_t)-1) {
        // tracker does not set a bit
        return false;
    }
    const AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return false;
    }
    if (!logger->should_log(_imu._log_raw_bit)) {
        return false;
    }
    return true;
}

