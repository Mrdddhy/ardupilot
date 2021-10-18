#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;


/********************************************************
*              OPT FLOW AND RANGE FINDER                *
********************************************************/

// Read the range finder and take new measurements if available
// Apply a median filter
void NavEKF2_core::readRangeFinder(void)
{
    uint8_t midIndex;
    uint8_t maxIndex;
    uint8_t minIndex;

    // get theoretical correct range when the vehicle is on the ground
    // don't allow range to go below 5cm because this can cause problems with optical flow processing
    rngOnGnd = MAX(frontend->_rng.ground_clearance_cm_orient(ROTATION_PITCH_270) * 0.01f, 0.05f);

    // read range finder at 20Hz
    // TODO better way of knowing if it has new data
    if ((imuSampleTime_ms - lastRngMeasTime_ms) > 50) {

        // reset the timer used to control the measurement rate
        lastRngMeasTime_ms =  imuSampleTime_ms;

        // store samples and sample time into a ring buffer if valid
        // use data from two range finders if available

        for (uint8_t sensorIndex = 0; sensorIndex <= 1; sensorIndex++) {
            AP_RangeFinder_Backend *sensor = frontend->_rng.get_backend(sensorIndex);
            if (sensor == nullptr) {
                continue;
            }
            if ((sensor->orientation() == ROTATION_PITCH_270) && (sensor->status() == RangeFinder::RangeFinder_Good)) {
                rngMeasIndex[sensorIndex] ++;
                if (rngMeasIndex[sensorIndex] > 2) {
                    rngMeasIndex[sensorIndex] = 0;
                }
                storedRngMeasTime_ms[sensorIndex][rngMeasIndex[sensorIndex]] = imuSampleTime_ms - 25;
                storedRngMeas[sensorIndex][rngMeasIndex[sensorIndex]] = sensor->distance_cm() * 0.01f;
            } else {
                continue;
            }

            // check for three fresh samples
            bool sampleFresh[2][3] = {};
            for (uint8_t index = 0; index <= 2; index++) {
                sampleFresh[sensorIndex][index] = (imuSampleTime_ms - storedRngMeasTime_ms[sensorIndex][index]) < 500;
            }

            // find the median value if we have three fresh samples
            if (sampleFresh[sensorIndex][0] && sampleFresh[sensorIndex][1] && sampleFresh[sensorIndex][2]) {
                if (storedRngMeas[sensorIndex][0] > storedRngMeas[sensorIndex][1]) {
                    minIndex = 1;
                    maxIndex = 0;
                } else {
                    minIndex = 0;
                    maxIndex = 1;
                }
                if (storedRngMeas[sensorIndex][2] > storedRngMeas[sensorIndex][maxIndex]) {
                    midIndex = maxIndex;
                } else if (storedRngMeas[sensorIndex][2] < storedRngMeas[sensorIndex][minIndex]) {
                    midIndex = minIndex;
                } else {
                    midIndex = 2;
                }

                // don't allow time to go backwards
                if (storedRngMeasTime_ms[sensorIndex][midIndex] > rangeDataNew.time_ms) {
                    rangeDataNew.time_ms = storedRngMeasTime_ms[sensorIndex][midIndex];
                }

                // limit the measured range to be no less than the on-ground range
                rangeDataNew.rng = MAX(storedRngMeas[sensorIndex][midIndex],rngOnGnd);

                // get position in body frame for the current sensor
                rangeDataNew.sensor_idx = sensorIndex;

                // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
                storedRange.push(rangeDataNew);

                // indicate we have updated the measurement
                rngValidMeaTime_ms = imuSampleTime_ms;

            } else if (!takeOffDetected && ((imuSampleTime_ms - rngValidMeaTime_ms) > 200)) {
                // before takeoff we assume on-ground range value if there is no data
                rangeDataNew.time_ms = imuSampleTime_ms;
                rangeDataNew.rng = rngOnGnd;
                rangeDataNew.time_ms = imuSampleTime_ms;

                // don't allow time to go backwards
                if (imuSampleTime_ms > rangeDataNew.time_ms) {
                    rangeDataNew.time_ms = imuSampleTime_ms;
                }

                // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
                storedRange.push(rangeDataNew);

                // indicate we have updated the measurement
                rngValidMeaTime_ms = imuSampleTime_ms;

            }
        }
    }
}

// write the raw optical flow measurements
// this needs to be called externally.
void NavEKF2_core::writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset)
{
    // The raw measurements need to be optical flow rates in radians/second averaged across the time since the last update
    // The PX4Flow sensor outputs flow rates with the following axis and sign conventions:
    // A positive X rate is produced by a positive sensor rotation about the X axis
    // A positive Y rate is produced by a positive sensor rotation about the Y axis
    // This filter uses a different definition of optical flow rates to the sensor with a positive optical flow rate produced by a
    // negative rotation about that axis. For example a positive rotation of the flight vehicle about its X (roll) axis would produce a negative X flow rate
    flowMeaTime_ms = imuSampleTime_ms;
    // calculate bias errors on flow sensor gyro rates, but protect against spikes in data
    // reset the accumulated body delta angle and time
    // don't do the calculation if not enough time lapsed for a reliable body rate measurement
    if (delTimeOF > 0.01f) {
        flowGyroBias.x = 0.99f * flowGyroBias.x + 0.01f * constrain_float((rawGyroRates.x - delAngBodyOF.x/delTimeOF),-0.1f,0.1f);
        flowGyroBias.y = 0.99f * flowGyroBias.y + 0.01f * constrain_float((rawGyroRates.y - delAngBodyOF.y/delTimeOF),-0.1f,0.1f);
        delAngBodyOF.zero();
        delTimeOF = 0.0f;
    }
    // by definition if this function is called, then flow measurements have been provided so we
    // need to run the optical flow takeoff detection
    detectOptFlowTakeoff();

    // calculate rotation matrices at mid sample time for flow observations
    stateStruct.quat.rotation_matrix(Tbn_flow);
    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sensor data)
    if ((rawFlowQuality > 0) && rawFlowRates.length() < 4.2f && rawGyroRates.length() < 4.2f) {
        // correct flow sensor body rates for bias and write
        ofDataNew.bodyRadXYZ.x = rawGyroRates.x - flowGyroBias.x;
        ofDataNew.bodyRadXYZ.y = rawGyroRates.y - flowGyroBias.y;
        // the sensor interface doesn't provide a z axis rate so use the rate from the nav sensor instead
        if (delTimeOF > 0.001f) {
            // first preference is to use the rate averaged over the same sampling period as the flow sensor
            ofDataNew.bodyRadXYZ.z = delAngBodyOF.z / delTimeOF;
        } else if (imuDataNew.delAngDT > 0.001f){
            // second preference is to use most recent IMU data
            ofDataNew.bodyRadXYZ.z = imuDataNew.delAng.z / imuDataNew.delAngDT;
        } else {
            // third preference is use zero
            ofDataNew.bodyRadXYZ.z =  0.0f;
        }
        // write uncorrected flow rate measurements
        // note correction for different axis and sign conventions used by the px4flow sensor
        ofDataNew.flowRadXY = - rawFlowRates; // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
        // write the flow sensor position in body frame
        ofDataNew.body_offset = &posOffset;
        // write flow rate measurements corrected for body rates
        ofDataNew.flowRadXYcomp.x = ofDataNew.flowRadXY.x + ofDataNew.bodyRadXYZ.x;
        ofDataNew.flowRadXYcomp.y = ofDataNew.flowRadXY.y + ofDataNew.bodyRadXYZ.y;
        // record time last observation was received so we can detect loss of data elsewhere
        flowValidMeaTime_ms = imuSampleTime_ms;
        // estimate sample time of the measurement
        ofDataNew.time_ms = imuSampleTime_ms - frontend->_flowDelay_ms - frontend->flowTimeDeltaAvg_ms/2;
        // Correct for the average intersampling delay due to the filter updaterate
        ofDataNew.time_ms -= localFilterTimeStep_ms/2;
        // Prevent time delay exceeding age of oldest IMU data in the buffer
        ofDataNew.time_ms = MAX(ofDataNew.time_ms,imuDataDelayed.time_ms);
        // Save data to buffer
        storedOF.push(ofDataNew);
        // Check for data at the fusion time horizon
        flowDataToFuse = storedOF.recall(ofDataDelayed, imuDataDelayed.time_ms);
    }
}


/********************************************************
*                      MAGNETOMETER                     *
********************************************************/

// check for new magnetometer data and update store measurements if available
/*检查新的磁强计数据，并更新存储测量值（如果可用）*/
void NavEKF2_core::readMagData()
{
    if (!_ahrs->get_compass()) {
        allMagSensorsFailed = true;
        return;        
    }
    // If we are a vehicle with a sideslip constraint to aid yaw estimation and we have timed out on our last avialable
    // magnetometer, then declare the magnetometers as failed for this flight
    /*如果我们是一辆带有侧滑约束以帮助进行偏航估计的车辆，并且我们的最后一个可飞行磁力计超时，那么宣布磁力计在这次飞行中失败*/
    uint8_t maxCount = _ahrs->get_compass()->get_count();
    if (allMagSensorsFailed || (magTimeout && assume_zero_sideslip() && magSelectIndex >= maxCount-1 && inFlight)) {
        allMagSensorsFailed = true;
        return;
    }

    if (_ahrs->get_compass()->learn_offsets_enabled()) {
        // 学习偏移时，保持所有磁感应状态复位----while learning offsets keep all mag states reset
        InitialiseVariablesMag();
        wasLearningCompass_ms = imuSampleTime_ms;
    } else if (wasLearningCompass_ms != 0 && imuSampleTime_ms - wasLearningCompass_ms > 1000) {
        wasLearningCompass_ms = 0;
        // force a new yaw alignment 1s after learning completes. The
        // delay is to ensure any buffered mag samples are discarded
        /*学习完成1s后强制进行新的偏航对准。延迟是为了确保丢弃任何缓冲的mag样本*/
        yawAlignComplete = false;
        InitialiseVariablesMag();
    }

    
    // do not accept new compass data faster than 14Hz (nominal rate is 10Hz) to prevent high processor loading
    // because magnetometer fusion is an expensive step and we could overflow the FIFO buffer
    /*不要接受速度超过14Hz（标称速率为10Hz）的新罗盘数据，以防止处理器负载过高
      因为磁力计融合是一个昂贵的步骤，我们可能会溢出FIFO缓冲区*/
    if (use_compass() && _ahrs->get_compass()->last_update_usec() - lastMagUpdate_us > 70000) {
        frontend->logging.log_compass = true;

        // If the magnetometer has timed out (been rejected too long) we find another magnetometer to use if available
        // Don't do this if we are on the ground because there can be magnetic interference and we need to know if there is a problem
        // before taking off. Don't do this within the first 30 seconds from startup because the yaw error could be due to large yaw gyro bias affsets
        /* 如果磁强计超时（被拒绝的时间太长），我们会找到另一个磁强计使用（如果可用）
           如果我们在地面上，不要这样做，因为可能会有磁干扰，我们需要在起飞前知道是否有问题
           不要在启动后的前30秒内执行此操作，因为偏航误差可能是由于偏航陀螺仪偏差过大造成的*/
        if (magTimeout && (maxCount > 1) && !onGround && imuSampleTime_ms - ekfStartTime_ms > 30000) {

            // search through the list of magnetometers
            // 在磁强计列表中搜索
            for (uint8_t i=1; i<maxCount; i++) {
                uint8_t tempIndex = magSelectIndex + i;
                // loop back to the start index if we have exceeded the bounds
                // 如果超出了界限，则循环回起始索引
                if (tempIndex >= maxCount) {
                    tempIndex -= maxCount;
                }
                // if the magnetometer is allowed to be used for yaw and has a different index, we start using it
                // 如果磁强计被允许用于偏航，并且有不同的指数，我们就开始使用它
                if (_ahrs->get_compass()->use_for_yaw(tempIndex) && tempIndex != magSelectIndex) {
                    magSelectIndex = tempIndex;
                    gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u switching to compass %u",(unsigned)imu_index,magSelectIndex);
                    // reset the timeout flag and timer
                    // 重置超时标志和计时器
                    magTimeout = false;
                    lastHealthyMagTime_ms = imuSampleTime_ms;
                    // zero the learned magnetometer bias states
                    // 将学习到的磁强计偏置状态归零
                    stateStruct.body_magfield.zero();
                    // clear the measurement buffer
                    // 清除测量缓冲区
                    storedMag.reset();
                    // clear the data waiting flag so that we do not use any data pending from the previous sensor
                    // 清除data waiting（数据等待）标志，这样我们就不会使用来自上一个传感器的任何挂起数据
                    magDataToFuse = false;
                    // request a reset of the magnetic field states
                    // 请求重置磁场状态
                    magStateResetRequest = true;
                    // declare the field unlearned so that the reset request will be obeyed
                    // 声明磁场未学习，以便遵守重置请求
                    magFieldLearned = false;
                    break;
                }
            }
        }

        // detect changes to magnetometer offset parameters and reset states
        // 检测磁力计偏移参数和重置状态的变化
        Vector3f nowMagOffsets = _ahrs->get_compass()->get_offsets(magSelectIndex);
        bool changeDetected = lastMagOffsetsValid && (nowMagOffsets != lastMagOffsets);
        if (changeDetected) {
            // zero the learned magnetometer bias states
            // 将学习到的磁力计偏置状态归零
            stateStruct.body_magfield.zero();
            // clear the measurement buffer
            // 清除测量缓冲区
            storedMag.reset();
        }
        lastMagOffsets = nowMagOffsets;
        lastMagOffsetsValid = true;

        // store time of last measurement update
        // 上次测量更新的存储时间
        lastMagUpdate_us = _ahrs->get_compass()->last_update_usec(magSelectIndex);

        // estimate of time magnetometer measurement was taken, allowing for delays
        // 考虑到延迟，估计进行磁力计测量的时间
        magDataNew.time_ms = imuSampleTime_ms - frontend->magDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        //考虑到延迟，估计进行磁强计测量的时间
        magDataNew.time_ms -= localFilterTimeStep_ms/2;

        // read compass data and scale to improve numerical conditioning
        // 读取罗盘数据和刻度以改善数值调节
        magDataNew.mag = _ahrs->get_compass()->get_field(magSelectIndex) * 0.001f;

        // check for consistent data between magnetometers
        // 检查磁力计之间的数据是否一致
        consistentMagData = _ahrs->get_compass()->consistent();

        // save magnetometer measurement to buffer to be fused later
        // 将磁力计测量保存到缓冲区，以便稍后融合
        storedMag.push(magDataNew);
    }
}

/********************************************************
*                Inertial Measurements                  *
********************************************************/

/*
 *  Read IMU delta angle and delta velocity measurements and downsample to 100Hz
 *  for storage in the data buffers used by the EKF. If the IMU data arrives at
 *  lower rate than 100Hz, then no downsampling or upsampling will be performed.
 *  Downsampling is done using a method that does not introduce coning or sculling
 *  errors.
 * 读取IMU增量角和增量速度测量值，并向下采样到100Hz，存储在EKF使用的数据缓冲区中。 
 * 如果IMU数据到达速率低于100Hz，则不进行下采样或上采样。 
 * 下采样是使用一种不引入圆锥或划桨误差的方法进行的。
 */
void NavEKF2_core::readIMUData()
{
    const AP_InertialSensor &ins = AP::ins();

    // average IMU sampling rate
    /*平均IMU采样速率*/
    dtIMUavg = ins.get_loop_delta_t();

    // the imu sample time is used as a common time reference throughout the filter
    /*IMU采样时间用作整个滤波器的公共时间基准*/
    imuSampleTime_ms = AP_HAL::millis();

    // use the nominated imu or primary if not available
    /*如果不可用，请使用指定的imu或主imu*/
    uint8_t accel_active, gyro_active;

    if (ins.use_accel(imu_index)) {
        accel_active = imu_index;
    } else {
        accel_active = ins.get_primary_accel();
    }

    if (ins.use_gyro(imu_index)) {
        gyro_active = imu_index;
    } else {
        gyro_active = ins.get_primary_gyro();
    }
    /*这种条件成立也就是说当我们初始化活跃状态gyro_index_active = 0,但是我们正在使用时gyro_active = 1
      那么就把1号的陀螺仪偏差和尺度给到状态变量，并相应地更新陀螺仪的活跃状态gyro_index_active */
    if (gyro_active != gyro_index_active) {
        // we are switching active gyro at runtime. Copy over the
        // biases we have learned from the previously inactive
        // gyro. We don't re-init the bias uncertainty as it should
        // have the same uncertainty as the previously active gyro
        /*我们正在运行时切换活跃陀螺仪。复制我们从之前未激活的陀螺仪中了解到的偏差。
          我们不重新初始化偏置不确定度，因为它应该与先前的活跃陀螺仪具有相同的不确定度*/
        stateStruct.gyro_bias = inactiveBias[gyro_active].gyro_bias;
        gyro_index_active = gyro_active;

        // use the gyro scale factor we have previously used on this
        // IMU (if any). We don't reset the variances as we don't want
        // errors after switching to be mis-assigned to the gyro scale
        // factor
        /*使用我们之前在此IMU上使用的陀螺比例因数（如果有）。我们不重置方差，
          因为我们不希望切换后的误差被错误分配给陀螺比例因子*/
        stateStruct.gyro_scale = inactiveBias[gyro_active].gyro_scale;
    }

    if (accel_active != accel_index_active) {
        // switch to the learned accel bias for this IMU
        // 切换到这个IMU已经学到的加速度计偏差
        stateStruct.accel_zbias = inactiveBias[accel_active].accel_zbias;
        accel_index_active = accel_active;
    }

    // update the inactive bias states
    /*更新不活跃的偏差状态*/
    learnInactiveBiases();

   /*读取增量速度*/
    readDeltaVelocity(accel_index_active, imuDataNew.delVel, imuDataNew.delVelDT);
    accelPosOffset = ins.get_imu_pos_offset(accel_index_active);
    imuDataNew.accel_index = accel_index_active;

    // Get delta angle data from primary gyro or primary if not available
    /*读取增量角度*/
    readDeltaAngle(gyro_index_active, imuDataNew.delAng, imuDataNew.delAngDT);
    imuDataNew.gyro_index = gyro_index_active;

    // Get current time stamp
    /*获取时间戳*/
    imuDataNew.time_ms = imuSampleTime_ms;

    // use the most recent IMU index for the downsampled IMU
    // data. This isn't strictly correct if we switch IMUs between
    // samples
    /*将最新的IMU索引用于下采样的IMU数据。如果我们在样本之间切换IMU，这就不是严格正确的*/
    imuDataDownSampledNew.gyro_index = imuDataNew.gyro_index;
    imuDataDownSampledNew.accel_index = imuDataNew.accel_index;

    // Accumulate the measurement time interval for the delta velocity and angle data
    /*积分算下采样的角度增量和速度增量测量时间区间长度*/
    imuDataDownSampledNew.delAngDT += imuDataNew.delAngDT;
    imuDataDownSampledNew.delVelDT += imuDataNew.delVelDT;

    //Rotate quaternon atitude from previous to new and normalise.
    //Accumulation using quaternions prevents introduction of coning errors due to downsampling
    /*角度得到旋转后的四元数矩阵，进行归一化处理
      使用四元数的累积防止由于下采样引起圆锥误差*/
    imuQuatDownSampleNew.rotate(imuDataNew.delAng);
    imuQuatDownSampleNew.normalize();

    // Rotate the latest delta velocity into body frame at the start of accumulation
    /*在开始累积时，将最新的增量速度旋转到机体系中*/
    Matrix3f deltaRotMat;
    imuQuatDownSampleNew.rotation_matrix(deltaRotMat);

    // Apply the delta velocity to the delta velocity accumulator
    /*将增量速度应用于增量速度累加器*/
    imuDataDownSampledNew.delVel += deltaRotMat*imuDataNew.delVel;

    // Keep track of the number of IMU frames since the last state prediction
    /*跟踪自上次状态预测以来的IMU帧数*/
    framesSincePredict++;

    /*
     * If the target EKF time step has been accumulated, and the frontend has allowed start of a new predict cycle,
     * then store the accumulated IMU data to be used by the state prediction, ignoring the frontend permission if more
     * than twice the target time has lapsed. Adjust the target EKF step time threshold to allow for timing jitter in the
     * IMU data.
     * 如果目标EKF时间步长已经累积，并且前端允许开始新的预测周期(startPredictEnabled == 1)，然后存储由状态预测所使用的累积IMU数据
     * ||如果超过目标时间的两倍，则忽略前端许可,调整目标EKF步进时间阈值，以允许IMU数据中的定时抖动。
     */
    if ((dtIMUavg*(float)framesSincePredict >= (EKF_TARGET_DT-(dtIMUavg*0.5)) &&
         startPredictEnabled) || (dtIMUavg*(float)framesSincePredict >= 2.0f*EKF_TARGET_DT)) {

        // convert the accumulated quaternion to an equivalent delta angle
        /*转换累积的四元数到姿态角*/
        imuQuatDownSampleNew.to_axis_angle(imuDataDownSampledNew.delAng);

        // Time stamp the data
        /*获取数据的时间戳*/
        imuDataDownSampledNew.time_ms = imuSampleTime_ms;

        // Write data to the FIFO IMU buffer
        /*IMU下采样后的新数据写入缓冲区*/
        storedIMU.push_youngest_element(imuDataDownSampledNew);

        // calculate the achieved average time step rate for the EKF
        /*计算获得的EKF平均时间步长率*/
        float dtNow = constrain_float(0.5f*(imuDataDownSampledNew.delAngDT+imuDataDownSampledNew.delVelDT),0.0f,10.0f*EKF_TARGET_DT);
        dtEkfAvg = 0.98f * dtEkfAvg + 0.02f * dtNow;

        // zero the accumulated IMU data and quaternion
        /*清空累计的IMU数据和四元数*/
        imuDataDownSampledNew.delAng.zero();
        imuDataDownSampledNew.delVel.zero();
        imuDataDownSampledNew.delAngDT = 0.0f;
        imuDataDownSampledNew.delVelDT = 0.0f;
        imuDataDownSampledNew.gyro_index = gyro_index_active;
        imuDataDownSampledNew.accel_index = accel_index_active;
        imuQuatDownSampleNew[0] = 1.0f;//重置imuQuatDownSampleNew=[1,0,0,0]
        imuQuatDownSampleNew[3] = imuQuatDownSampleNew[2] = imuQuatDownSampleNew[1] = 0.0f;

        // reset the counter used to let the frontend know how many frames have elapsed since we started a new update cycle
        /*重置计数器，用于让前端知道自从我们开始新的更新周期以来已经有多少帧已经过去了*/
        framesSincePredict = 0;

        // set the flag to let the filter know it has new IMU data and needs to run
        /*设置标志，让滤波器知道它有新的IMU数据需要运行*/
        runUpdates = true;

        // extract the oldest available data from the FIFO buffer
        /*从FIFO缓冲区提取最古老的可用数据*/
        imuDataDelayed = storedIMU.pop_oldest_element();

        // protect against delta time going to zero
        // TODO - check if calculations can tolerate 0
        /*防止增量时间为0*/
        float minDT = 0.1f*dtEkfAvg;
        imuDataDelayed.delAngDT = MAX(imuDataDelayed.delAngDT,minDT);
        imuDataDelayed.delVelDT = MAX(imuDataDelayed.delVelDT,minDT);

        updateTimingStatistics();//更新定时统计结构
            
        // correct the extracted IMU data for sensor errors
        /*纠正提取的IMU数据中的传感器错误，IMU数据在用于协方差预测之前进行了修正，而推导过程中的增量角和速度应未进行修正*/
        delAngCorrected = imuDataDelayed.delAng;
        delVelCorrected = imuDataDelayed.delVel;
        correctDeltaAngle(delAngCorrected, imuDataDelayed.delAngDT, imuDataDelayed.gyro_index);
        correctDeltaVelocity(delVelCorrected, imuDataDelayed.delVelDT, imuDataDelayed.accel_index);

    } else {
        // we don't have new IMU data in the buffer so don't run filter updates on this time step
        /*我们在缓冲区中没有新的IMU数据，所以不要在这个时间步骤上运行滤波器更新 */
        runUpdates = false;
    }
    /*对IMU进行下采样，并将输出状态观测器数据至100Hz，以便存储在缓冲器中。
      这将Copter的存储需求减少75%或6KB，但不会影响飞机所需的内存，因为该飞机的执行速度为50Hz，已经使用了短缓冲区。
      这意味着EKF滤波器操作的最大频率为100Hz。输出观测器继续在400Hz下工作，在下采样期间应用圆锥和划桨修正，因此不会损失精度*/
}

// read the delta velocity and corresponding time interval from the IMU
// return false if data is not available
/*从IMU读取增量速度和对应的增量时间区间，如果数据不可用则返回false*/
bool NavEKF2_core::readDeltaVelocity(uint8_t ins_index, Vector3f &dVel, float &dVel_dt) {
    const AP_InertialSensor &ins = AP::ins();

    if (ins_index < ins.get_accel_count()) {
        ins.get_delta_velocity(ins_index,dVel);
        dVel_dt = MAX(ins.get_delta_velocity_dt(ins_index),1.0e-4f);
        dVel_dt = MIN(dVel_dt,1.0e-1f);
        return true;
    }
    return false;
}

/********************************************************
*             Global Position Measurement               *
********************************************************/

// check for new valid GPS data and update stored measurement if available
/* 检查新的有效GPS数据，并更新存储的测量值（如果可用）*/
void NavEKF2_core::readGpsData()
{
    if (frontend->_fusionModeGPS == 3) {
        // don't read GPS data if GPS usage disabled
        return;//如果禁用GPS使用，则不读取GPS数据,直接返回
    }

    // check for new GPS data
    // do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    // 检查新的GPS数据,不要以高于14Hz的速率接受数据，以避免FIFO缓冲区溢出
    const AP_GPS &gps = AP::gps();
    if (gps.last_message_time_ms() - lastTimeGpsReceived_ms > 70) {//70ms = 0.07s ≈ 14Hz
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            // 报告GPS定位状态--report GPS fix status
            gpsCheckStatus.bad_fix = false;

            // store fix time from previous read
            secondLastGpsTime_ms = lastTimeGpsReceived_ms;//存储上次读取的固定时间

            // get current fix time
            lastTimeGpsReceived_ms = gps.last_message_time_ms(); //获取当前固定时间


            // estimate when the GPS fix was valid, allowing for GPS processing and other delays
            // ideally we should be using a timing signal from the GPS receiver to set this time
            /*估计GPS定位有效的时间，考虑GPS处理和其他延迟,理想情况下，我们应该使用来自GPS接收器的定时信号来设置这个时间*/
            float gps_delay = 0.0;
            gps.get_lag(gps_delay); // 忽略返回值--ignore the return value
            gpsDataNew.time_ms = lastTimeGpsReceived_ms - (uint32_t)(1e3f * gps_delay);//GPS有效时间 = 接收到数据的时间 - 延迟时间

            // Correct for the average intersampling delay due to the filter updaterate
            // 修正由于滤波器更新率导致的平均采样间延迟
            gpsDataNew.time_ms -= localFilterTimeStep_ms/2;

            // Prevent time delay exceeding age of oldest IMU data in the buffer
            // 防止时间延迟超过缓冲区中最旧IMU数据的期限
            gpsDataNew.time_ms = MAX(gpsDataNew.time_ms,imuDataDelayed.time_ms);

            // Get which GPS we are using for position information
            // 获取我们正用于位置信息的哪个GPS
            gpsDataNew.sensor_idx = gps.primary_sensor();

            // read the NED velocity from the GPS
            // 从GPS上读取NED速度
            gpsDataNew.vel = gps.velocity();

            // Use the speed and position accuracy from the GPS if available, otherwise set it to zero.
            // Apply a decaying envelope filter with a 5 second time constant to the raw accuracy data
            /* 如果可用，使用GPS的速度和位置精度，否则将其设置为零
               对原始精度数据应用具有5秒时间常数的衰减包络滤波器*/
            float alpha = constrain_float(0.0002f * (lastTimeGpsReceived_ms - secondLastGpsTime_ms),0.0f,1.0f);
            gpsSpdAccuracy *= (1.0f - alpha);
            float gpsSpdAccRaw;
            if (!gps.speed_accuracy(gpsSpdAccRaw)) {
                gpsSpdAccuracy = 0.0f;
            } else {
                gpsSpdAccuracy = MAX(gpsSpdAccuracy,gpsSpdAccRaw);
                gpsSpdAccuracy = MIN(gpsSpdAccuracy,50.0f);
                gpsSpdAccuracy = MAX(gpsSpdAccuracy,frontend->_gpsHorizVelNoise);
            }
            gpsPosAccuracy *= (1.0f - alpha);
            float gpsPosAccRaw;
            if (!gps.horizontal_accuracy(gpsPosAccRaw)) {
                gpsPosAccuracy = 0.0f;
            } else {
                gpsPosAccuracy = MAX(gpsPosAccuracy,gpsPosAccRaw);
                gpsPosAccuracy = MIN(gpsPosAccuracy,100.0f);
                gpsPosAccuracy = MAX(gpsPosAccuracy, frontend->_gpsHorizPosNoise);
            }
            gpsHgtAccuracy *= (1.0f - alpha);
            float gpsHgtAccRaw;
            if (!gps.vertical_accuracy(gpsHgtAccRaw)) {
                gpsHgtAccuracy = 0.0f;
            } else {
                gpsHgtAccuracy = MAX(gpsHgtAccuracy,gpsHgtAccRaw);
                gpsHgtAccuracy = MIN(gpsHgtAccuracy,100.0f);
                gpsHgtAccuracy = MAX(gpsHgtAccuracy, 1.5f * frontend->_gpsHorizPosNoise);
            }

            // check if we have enough GPS satellites and increase the gps noise scaler if we don't
            // 检查是否有足够的GPS卫星，如果没有，则增加GPS噪声定标器
            if (gps.num_sats() >= 6 && (PV_AidingMode == AID_ABSOLUTE)) {
                gpsNoiseScaler = 1.0f;//用于缩放GPS测量噪声和一致性门，以补偿小卫星计数的操作
            } else if (gps.num_sats() == 5 && (PV_AidingMode == AID_ABSOLUTE)) {
                gpsNoiseScaler = 1.4f;
            } else { // <= 4 satellites or in constant position mode
                gpsNoiseScaler = 2.0f;
            }

            // Check if GPS can output vertical velocity, if it is allowed to be used, and set GPS fusion mode accordingly
            // 检查GPS是否能输出垂直速度，是否允许使用，并相应设置GPS融合模式
            if (gps.have_vertical_velocity() && frontend->_fusionModeGPS == 0 && !frontend->inhibitGpsVertVelUse) {
                useGpsVertVel = true;
            } else {
                useGpsVertVel = false;
            }

            // Monitor quality of the GPS velocity data both before and after alignment. This updates
            // GpsGoodToAlign class variable
            /* 在对准前后监测GPS速度数据的质量。这将更新GpsGoodToAlign类变量*/
            calcGpsGoodToAlign();

            // Post-alignment checks
            calcGpsGoodForFlight();// 定位后检查

            // see if we can get origin from frontend
            // 看看能不能从前端找到原点
            if (!validOrigin && frontend->common_origin_valid) {
                setOrigin(frontend->common_EKF_origin);
            }

            // Read the GPS location in WGS-84 lat,long,height coordinates
            // 读取WGS-84经度、纬度和高度坐标中的GPS位置
            const struct Location &gpsloc = gps.location();

            // Set the EKF origin and magnetic field declination if not previously set  and GPS checks have passed
            // 如果之前未设置且GPS检查已通过，则设置EKF原点和磁场偏角
            if (gpsGoodToAlign && !validOrigin) {
                setOrigin(gpsloc);

                // set the NE earth magnetic field states using the published declination
                // and set the corresponding variances and covariances
                /* 使用公布的磁偏角设置东北地磁场状态并设置相应的方差和协方差*/
                alignMagStateDeclination();

                // Set the height of the NED origin
                // 设置原点的高度
                ekfGpsRefHgt = (double)0.01 * (double)gpsloc.alt + (double)outputDataNew.position.z;

                // Set the uncertainty of the GPS origin height
                // 设置GPS原点高度的不确定性
                ekfOriginHgtVar = sq(gpsHgtAccuracy);

            }

            if (gpsGoodToAlign && !have_table_earth_field) {
                const Compass *compass = _ahrs->get_compass();
                if (compass && compass->have_scale_factor(magSelectIndex) && compass->auto_declination_enabled()) {
                    table_earth_field_ga = AP_Declination::get_earth_field_ga(gpsloc);
                    table_declination = radians(AP_Declination::get_declination(gpsloc.lat*1.0e-7,
                                                                                gpsloc.lng*1.0e-7));
                    have_table_earth_field = true;
                    if (frontend->_mag_ef_limit > 0) {
                        // initialise earth field from tables
                        stateStruct.earth_magfield = table_earth_field_ga;//从表中初始化接地磁场
                    }
                }
            }
            
            // convert GPS measurements to local NED and save to buffer to be fused later if we have a valid origin
            /*将GPS测量值转换为本地NED，并保存到缓冲区，以便在我们有有效原点时进行融合*/
            if (validOrigin) {
                gpsDataNew.pos = EKF_origin.get_distance_NE(gpsloc);
                if ((frontend->_originHgtMode & (1<<2)) == 0) {
                    gpsDataNew.hgt = (float)((double)0.01 * (double)gpsloc.alt - ekfGpsRefHgt);
                } else {
                    gpsDataNew.hgt = 0.01 * (gpsloc.alt - EKF_origin.alt);
                }
                storedGPS.push(gpsDataNew);
                // declare GPS available for use
                gpsNotAvailable = false;//宣布GPS可用
            }

        } else {
            // report GPS fix status
            gpsCheckStatus.bad_fix = true;//报告GPS定位状态
            hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string), "Waiting for 3D fix");//等待3D定位完成
        }
    }
}

// read the delta angle and corresponding time interval from the IMU
// return false if data is not available
/*从IMU读取增量角度和相应的时间间隔,如果数据不可用，则返回false*/
bool NavEKF2_core::readDeltaAngle(uint8_t ins_index, Vector3f &dAng, float &dAng_dt) {
    const AP_InertialSensor &ins = AP::ins();

    if (ins_index < ins.get_gyro_count()) {
        ins.get_delta_angle(ins_index,dAng);
        frontend->logging.log_imu = true;
        dAng_dt = MAX(ins.get_delta_angle_dt(ins_index),1.0e-4f);/*限制增量角度时间在dAng_dt在0.0001~0.1*/
        dAng_dt = MIN(dAng_dt,1.0e-1f);
        return true;
    }
    return false;
}


/********************************************************
*                  Height Measurements                  *
********************************************************/

// check for new pressure altitude measurement data and update stored measurement if available
// 检查是否有新的压力高度测量数据，并更新存储的测量值（如有）
void NavEKF2_core::readBaroData()
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    // do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    /* 检查气压测量值是否发生变化，以便我们知道是否有新的测量值到达
       不要以高于14Hz的速率接受数据，以避免FIFO缓冲区溢出*/
    const AP_Baro &baro = AP::baro();
    if (baro.get_last_update() - lastBaroReceived_ms > 70) {
        frontend->logging.log_baro = true;

        baroDataNew.hgt = baro.get_altitude();

        // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
        // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
        // 如果我们处于起飞模式，高度测量值限制为不小于起飞开始时的测量值
        // 这防止了由于copter在初始上升期间因机翼所产生的下降气流（向下冲洗）产生的负气压扰动破坏EKF高度
        if (getTakeoffExpected()) {/*确定是否预计起飞，以便我们能够补偿由于地面效应导致的预期气压计误差*/
            baroDataNew.hgt = MAX(baroDataNew.hgt, meaHgtAtTakeOff);
        }

        // time stamp used to check for new measurement
        // 用于检查新测量值的时间戳
        lastBaroReceived_ms = baro.get_last_update();

        // estimate of time height measurement was taken, allowing for delays
        // 考虑到延迟，对高度测量的时间进行了估计
        baroDataNew.time_ms = lastBaroReceived_ms - frontend->_hgtDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        // 由于滤波器更新率导致的平均采样间延迟校正
        baroDataNew.time_ms -= localFilterTimeStep_ms/2;

        // Prevent time delay exceeding age of oldest IMU data in the buffer
        // 防止时间延迟超过缓冲区中最旧IMU数据的期限
        baroDataNew.time_ms = MAX(baroDataNew.time_ms,imuDataDelayed.time_ms);

        // save baro measurement to buffer to be fused later
        // 将大气压力测量值保存到缓冲器，以便稍后融合
        storedBaro.push(baroDataNew);
    }
}

// calculate filtered offset between baro height measurement and EKF height estimate
// offset should be subtracted from baro measurement to match filter estimate
// offset is used to enable reversion to baro from alternate height data source
void NavEKF2_core::calcFiltBaroOffset()
{
    // Apply a first order LPF with spike protection
    baroHgtOffset += 0.1f * constrain_float(baroDataDelayed.hgt + stateStruct.position.z - baroHgtOffset, -5.0f, 5.0f);
}

// correct the height of the EKF origin to be consistent with GPS Data using a Bayes filter.
void NavEKF2_core::correctEkfOriginHeight()
{
    // Estimate the WGS-84 height of the EKF's origin using a Bayes filter

    // calculate the variance of our a-priori estimate of the ekf origin height
    float deltaTime = constrain_float(0.001f * (imuDataDelayed.time_ms - lastOriginHgtTime_ms), 0.0f, 1.0f);
    if (activeHgtSource == HGT_SOURCE_BARO) {
        // Use the baro drift rate
        const float baroDriftRate = 0.05f;
        ekfOriginHgtVar += sq(baroDriftRate * deltaTime);
    } else if (activeHgtSource == HGT_SOURCE_RNG) {
        // use the worse case expected terrain gradient and vehicle horizontal speed
        const float maxTerrGrad = 0.25f;
        ekfOriginHgtVar += sq(maxTerrGrad * norm(stateStruct.velocity.x , stateStruct.velocity.y) * deltaTime);
    } else {
        // by definition our height source is absolute so cannot run this filter
        return;
    }
    lastOriginHgtTime_ms = imuDataDelayed.time_ms;

    // calculate the observation variance assuming EKF error relative to datum is independent of GPS observation error
    // when not using GPS as height source
    float originHgtObsVar = sq(gpsHgtAccuracy) + P[8][8];

    // calculate the correction gain
    float gain = ekfOriginHgtVar / (ekfOriginHgtVar + originHgtObsVar);

    // calculate the innovation
    float innovation = - stateStruct.position.z - gpsDataDelayed.hgt;

    // check the innovation variance ratio
    float ratio = sq(innovation) / (ekfOriginHgtVar + originHgtObsVar);

    // correct the EKF origin and variance estimate if the innovation is less than 5-sigma
    if (ratio < 25.0f && gpsAccuracyGood) {
        ekfGpsRefHgt -= (double)(gain * innovation);
        ekfOriginHgtVar -= MAX(gain * ekfOriginHgtVar , 0.0f);
    }
}

/********************************************************
*                Air Speed Measurements                 *
********************************************************/

// check for new airspeed data and update stored measurements if available
void NavEKF2_core::readAirSpdData()
{
    // if airspeed reading is valid and is set by the user to be used and has been updated then
    // we take a new reading, convert from EAS to TAS and set the flag letting other functions
    // know a new measurement is available
    const AP_Airspeed *aspeed = _ahrs->get_airspeed();
    if (aspeed &&
        aspeed->use() && aspeed->healthy() &&
        aspeed->last_update_ms() != timeTasReceived_ms) {
        tasDataNew.tas = aspeed->get_airspeed() * AP::ahrs().get_EAS2TAS();
        timeTasReceived_ms = aspeed->last_update_ms();
        tasDataNew.time_ms = timeTasReceived_ms - frontend->tasDelay_ms;

        // Correct for the average intersampling delay due to the filter update rate
        tasDataNew.time_ms -= localFilterTimeStep_ms/2;

        // Save data into the buffer to be fused when the fusion time horizon catches up with it
        storedTAS.push(tasDataNew);
    }
    // Check the buffer for measurements that have been overtaken by the fusion time horizon and need to be fused
    tasDataToFuse = storedTAS.recall(tasDataDelayed,imuDataDelayed.time_ms);
}

/********************************************************
*              Range Beacon Measurements                *
********************************************************/

// check for new range beacon data and push to data buffer if available
void NavEKF2_core::readRngBcnData()
{
    // get the location of the beacon data
    const AP_Beacon *beacon = AP::beacon();

    // exit immediately if no beacon object
    if (beacon == nullptr) {
        return;
    }

    // get the number of beacons in use
    N_beacons = beacon->count();

    // search through all the beacons for new data and if we find it stop searching and push the data into the observation buffer
    bool newDataToPush = false;
    uint8_t numRngBcnsChecked = 0;
    // start the search one index up from where we left it last time
    uint8_t index = lastRngBcnChecked;
    while (!newDataToPush && numRngBcnsChecked < N_beacons) {
        // track the number of beacons checked
        numRngBcnsChecked++;

        // move to next beacon, wrap index if necessary
        index++;
        if (index >= N_beacons) {
            index = 0;
        }

        // check that the beacon is healthy and has new data
        if (beacon->beacon_healthy(index) &&
                beacon->beacon_last_update_ms(index) != lastTimeRngBcn_ms[index])
        {
            // set the timestamp, correcting for measurement delay and average intersampling delay due to the filter update rate
            lastTimeRngBcn_ms[index] = beacon->beacon_last_update_ms(index);
            rngBcnDataNew.time_ms = lastTimeRngBcn_ms[index] - frontend->_rngBcnDelay_ms - localFilterTimeStep_ms/2;

            // set the range noise
            // TODO the range library should provide the noise/accuracy estimate for each beacon
            rngBcnDataNew.rngErr = frontend->_rngBcnNoise;

            // set the range measurement
            rngBcnDataNew.rng = beacon->beacon_distance(index);

            // set the beacon position
            rngBcnDataNew.beacon_posNED = beacon->beacon_position(index);

            // identify the beacon identifier
            rngBcnDataNew.beacon_ID = index;

            // indicate we have new data to push to the buffer
            newDataToPush = true;

            // update the last checked index
            lastRngBcnChecked = index;
        }
    }

    // Check if the beacon system has returned a 3D fix
    if (beacon->get_vehicle_position_ned(beaconVehiclePosNED, beaconVehiclePosErr)) {
        rngBcnLast3DmeasTime_ms = imuSampleTime_ms;
    }

    // Check if the range beacon data can be used to align the vehicle position
    if (imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250 && beaconVehiclePosErr < 1.0f && rngBcnAlignmentCompleted) {
        // check for consistency between the position reported by the beacon and the position from the 3-State alignment filter
        float posDiffSq = sq(receiverPos.x - beaconVehiclePosNED.x) + sq(receiverPos.y - beaconVehiclePosNED.y);
        float posDiffVar = sq(beaconVehiclePosErr) + receiverPosCov[0][0] + receiverPosCov[1][1];
        if (posDiffSq < 9.0f*posDiffVar) {
            rngBcnGoodToAlign = true;
            // Set the EKF origin and magnetic field declination if not previously set
            if (!validOrigin && PV_AidingMode != AID_ABSOLUTE) {
                // get origin from beacon system
                Location origin_loc;
                if (beacon->get_origin(origin_loc)) {
                    setOriginLLH(origin_loc);

                    // set the NE earth magnetic field states using the published declination
                    // and set the corresponding variances and covariances
                    alignMagStateDeclination();

                    // Set the uncertainty of the origin height
                    ekfOriginHgtVar = sq(beaconVehiclePosErr);
                }
            }
        } else {
            rngBcnGoodToAlign = false;
        }
    } else {
        rngBcnGoodToAlign = false;
    }

    // Save data into the buffer to be fused when the fusion time horizon catches up with it
    if (newDataToPush) {
        storedRangeBeacon.push(rngBcnDataNew);
    }

    // Check the buffer for measurements that have been overtaken by the fusion time horizon and need to be fused
    rngBcnDataToFuse = storedRangeBeacon.recall(rngBcnDataDelayed,imuDataDelayed.time_ms);

}

/*
  update timing statistics structure
 */
void NavEKF2_core::updateTimingStatistics(void)
{
    if (timing.count == 0) {
        timing.dtIMUavg_max = dtIMUavg;
        timing.dtIMUavg_min = dtIMUavg;
        timing.dtEKFavg_max = dtEkfAvg;
        timing.dtEKFavg_min = dtEkfAvg;
        timing.delAngDT_max = imuDataDelayed.delAngDT;
        timing.delAngDT_min = imuDataDelayed.delAngDT;
        timing.delVelDT_max = imuDataDelayed.delVelDT;
        timing.delVelDT_min = imuDataDelayed.delVelDT;
    } else {
        timing.dtIMUavg_max = MAX(timing.dtIMUavg_max, dtIMUavg);
        timing.dtIMUavg_min = MIN(timing.dtIMUavg_min, dtIMUavg);
        timing.dtEKFavg_max = MAX(timing.dtEKFavg_max, dtEkfAvg);
        timing.dtEKFavg_min = MIN(timing.dtEKFavg_min, dtEkfAvg);
        timing.delAngDT_max = MAX(timing.delAngDT_max, imuDataDelayed.delAngDT);
        timing.delAngDT_min = MIN(timing.delAngDT_min, imuDataDelayed.delAngDT);
        timing.delVelDT_max = MAX(timing.delVelDT_max, imuDataDelayed.delVelDT);
        timing.delVelDT_min = MIN(timing.delVelDT_min, imuDataDelayed.delVelDT);
    }
    timing.count++;
}

// get timing statistics structure
void NavEKF2_core::getTimingStatistics(struct ekf_timing &_timing)
{
    _timing = timing;
    memset(&timing, 0, sizeof(timing));
}

void NavEKF2_core::writeExtNavData(const Vector3f &sensOffset, const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint32_t resetTime_ms)
{
    // limit update rate to maximum allowed by sensor buffers and fusion process
    // don't try to write to buffer until the filter has been initialised
    if ((timeStamp_ms - extNavMeasTime_ms) < 70) {
        return;
    } else {
        extNavMeasTime_ms = timeStamp_ms;
    }

    if (resetTime_ms > extNavLastPosResetTime_ms) {
        extNavDataNew.posReset = true;
        extNavLastPosResetTime_ms = resetTime_ms;
    } else {
        extNavDataNew.posReset = false;
    }

    extNavDataNew.pos = pos;
    extNavDataNew.quat = quat;
    if (posErr > 0) {
        extNavDataNew.posErr = posErr;
    } else {
        extNavDataNew.posErr = frontend->_gpsHorizPosNoise;
    }
    extNavDataNew.angErr = angErr;
    extNavDataNew.body_offset = &sensOffset;
    timeStamp_ms = timeStamp_ms - frontend->_extnavDelay_ms;
    // Correct for the average intersampling delay due to the filter updaterate
    timeStamp_ms -= localFilterTimeStep_ms/2;
    // Prevent time delay exceeding age of oldest IMU data in the buffer
    timeStamp_ms = MAX(timeStamp_ms,imuDataDelayed.time_ms);
    extNavDataNew.time_ms = timeStamp_ms;

    storedExtNav.push(extNavDataNew);

}

/*
  return declination in radians
  以弧度的形式返回磁偏角
*/
float NavEKF2_core::MagDeclination(void) const
{
    // if we are using the WMM tables then use the table declination
    // to ensure consistency with the table mag field. Otherwise use
    // the declination from the compass library
    /*如果我们使用WMM表，那么使用表的磁偏角来确保与磁场表的一致性，否则使用罗盘库中的磁偏角*/
    if (have_table_earth_field && frontend->_mag_ef_limit > 0) {
        return table_declination;
    }
    if (!use_compass()) {
        return 0;
    }
    return _ahrs->get_compass()->get_declination();
}

/*
  update estimates of inactive bias states. This keeps inactive IMUs
  as hot-spares so we can switch to them without causing a jump in the
  error
  更新非活动偏差状态的估计值。这会将非活动的IMU保持为热备盘，以便我们可以切换到它们，而不会导致错误跳转
 */
void NavEKF2_core::learnInactiveBiases(void)
{
    const AP_InertialSensor &ins = AP::ins();

    // 学习陀螺偏差--learn gyro biases
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (!ins.use_gyro(i)) {
            // can't use this gyro
            continue;//不使用的直接跳过
        }
        if (gyro_index_active == i) {
            // use current estimates from main filter of gyro bias and scale
            // 使用陀螺仪偏差和尺度的主滤波器的当前估计
            inactiveBias[i].gyro_bias = stateStruct.gyro_bias;
            inactiveBias[i].gyro_scale = stateStruct.gyro_scale;
        } else {
            // get filtered gyro and use the difference between the
            // corrected gyro on the active IMU and the inactive IMU
            // to move the inactive bias towards the right value
            /*获取滤波陀螺仪，并使用活跃IMU的校正陀螺仪和非活跃IMU之间的差值，将非活跃偏置移向正确的值*/
            Vector3f filtered_gyro_active = ins.get_gyro(gyro_index_active) - (stateStruct.gyro_bias/dtEkfAvg);//激活的活跃状态陀螺仪的滤波数值
            Vector3f filtered_gyro_inactive = ins.get_gyro(i) - (inactiveBias[i].gyro_bias/dtEkfAvg);//未激活状态的陀螺仪的滤波数值
            Vector3f error = filtered_gyro_active - filtered_gyro_inactive;//两者差值

            // prevent a single large error from contaminating bias estimate
            /* 防止单个大误差污染偏差估计*/
            const float bias_limit = radians(5);
            error.x = constrain_float(error.x, -bias_limit, bias_limit);//误差约束
            error.y = constrain_float(error.y, -bias_limit, bias_limit);
            error.z = constrain_float(error.z, -bias_limit, bias_limit);

            // slowly bring the inactive gyro in line with the active gyro. This corrects a 5 deg/sec
            // gyro bias error in around 1 minute
            /*缓慢地使非活动陀螺仪与活动陀螺仪对齐。这将在大约1分钟内纠正5度/秒的陀螺仪偏差误差*/
            inactiveBias[i].gyro_bias -= error * (1.0e-4f * dtEkfAvg);
        }
    }

    // 学习加速度偏差---learn accel biases
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (!ins.use_accel(i)) {
            // can't use this accel
            continue;//不能使用该加速度计，则跳过
        }
        if (accel_index_active == i) {
            // use current estimate from main filter
            // 使用主滤波器的当前估计
            inactiveBias[i].accel_zbias = stateStruct.accel_zbias;
        } else {
            // get filtered accel and use the difference between the
            // corrected accel on the active IMU and the inactive IMU
            // to move the inactive bias towards the right value
            /*获取过滤后的加速度，并使用激活IMU和非激活IMU上的校正加速度之间的差值，将非激活偏差移向正确的值*/
            float filtered_accel_active = ins.get_accel(accel_index_active).z - (stateStruct.accel_zbias/dtEkfAvg);//激活状态下的修正后的加速度数值
            float filtered_accel_inactive = ins.get_accel(i).z - (inactiveBias[i].accel_zbias/dtEkfAvg);//未激活状态下的修正后的加速度数值
            float error = filtered_accel_active - filtered_accel_inactive;//计算两者差值

            // prevent a single large error from contaminating bias estimate
            const float bias_limit = 1; // m/s/s
            error = constrain_float(error, -bias_limit, bias_limit);//误差约束

            // slowly bring the inactive accel in line with the active accel
            // this learns 0.5m/s/s bias in about 1 minute
            /*缓慢地使非活动的加速度计与活动的加速度计对齐，这将在大约1分钟内学会0.5m/s/s偏差*/
            inactiveBias[i].accel_zbias -= error * (1.0e-4f * dtEkfAvg);
        }
    }
}

