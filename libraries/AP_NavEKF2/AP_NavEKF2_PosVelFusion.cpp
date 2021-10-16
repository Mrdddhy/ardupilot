#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
/*将速度状态重置为上一次GPS测量值(如果可用)，或将速度状态重置为0(如果处于恒定位置模式或PV辅助不是绝对值)， 
 *请勿使用GPS重置垂直速度，因为存在气压计高度可用于约束漂移*/
void NavEKF2_core::ResetVelocity(void)
{
    // Store the position before the reset so that we can record the reset delta
    // 在重置之前存储位置，以便我们可以记录重置增量
    velResetNE.x = stateStruct.velocity.x;
    velResetNE.y = stateStruct.velocity.y;

    // reset the corresponding covariances
    // 重置相应的协方差
    zeroRows(P,3,4);
    zeroCols(P,3,4);

    gps_elements gps_corrected = gpsDataNew;
    CorrectGPSForAntennaOffset(gps_corrected);
    
    if (PV_AidingMode != AID_ABSOLUTE) {
        stateStruct.velocity.zero();
        // set the variances using the measurement noise parameter
        // 使用测量噪声参数设置方差
        P[4][4] = P[3][3] = sq(frontend->_gpsHorizVelNoise);
    } else {
        // reset horizontal velocity states to the GPS velocity if available
        // 将水平速度状态重置为GPS速度（如果可用）
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            stateStruct.velocity.x  = gps_corrected.vel.x;
            stateStruct.velocity.y  = gps_corrected.vel.y;
            // set the variances using the reported GPS speed accuracy
            // 使用报告的GPS速度精度设置偏差
            P[4][4] = P[3][3] = sq(MAX(frontend->_gpsHorizVelNoise,gpsSpdAccuracy));
            // clear the timeout flags and counters
            // 清除超时标志和计数器
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        } else {
            stateStruct.velocity.x  = 0.0f;
            stateStruct.velocity.y  = 0.0f;
            // set the variances using the likely speed range
            P[4][4] = P[3][3] = sq(25.0f);
            // clear the timeout flags and counters
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        }
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.x = stateStruct.velocity.x;
        storedOutput[i].velocity.y = stateStruct.velocity.y;
    }
    outputDataNew.velocity.x = stateStruct.velocity.x;
    outputDataNew.velocity.y = stateStruct.velocity.y;
    outputDataDelayed.velocity.x = stateStruct.velocity.x;
    outputDataDelayed.velocity.y = stateStruct.velocity.y;

    // Calculate the position jump due to the reset
    // 计算由于复位引起的速度跳变
    velResetNE.x = stateStruct.velocity.x - velResetNE.x;
    velResetNE.y = stateStruct.velocity.y - velResetNE.y;

    // store the time of the reset
    // 存储重置的时间
    lastVelReset_ms = imuSampleTime_ms;


}

// resets position states to last GPS measurement or to zero if in constant position mode
// 将位置状态重置为上一次GPS测量，如果处于恒定位置模式，则重置为零
void NavEKF2_core::ResetPosition(void)
{
    // Store the position before the reset so that we can record the reset delta
    // 在重置之前存储位置，以便我们可以记录重置增量
    posResetNE.x = stateStruct.position.x;
    posResetNE.y = stateStruct.position.y;

    // reset the corresponding covariances
    // 重置相应的协方差
    zeroRows(P,6,7);
    zeroCols(P,6,7);

    gps_elements gps_corrected = gpsDataNew;
    CorrectGPSForAntennaOffset(gps_corrected);
    
    if (PV_AidingMode != AID_ABSOLUTE) {
        // reset all position state history to the last known position
        // 将所有位置状态历史记录重置为最后一个已知位置
        stateStruct.position.x = lastKnownPositionNE.x;
        stateStruct.position.y = lastKnownPositionNE.y;
        // set the variances using the position measurement noise parameter
        // 使用位置测量噪声参数设置方差
        P[6][6] = P[7][7] = sq(frontend->_gpsHorizPosNoise);
    } else  {
        // Use GPS data as first preference if fresh data is available
        // 如果有新数据可用，则将GPS数据作为首选项
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            // record the ID of the GPS for the data we are using for the reset
            // 记录我们用于重置的数据的GPS ID
            last_gps_idx = gps_corrected.sensor_idx;
            // write to state vector and compensate for offset  between last GPS measurement and the EKF time horizon
            // 写入状态向量并补偿上一次GPS测量和EKF时间范围之间的偏移
            stateStruct.position.x = gps_corrected.pos.x  + 0.001f*gps_corrected.vel.x*(float(imuDataDelayed.time_ms) - float(gps_corrected.time_ms));
            stateStruct.position.y = gps_corrected.pos.y  + 0.001f*gps_corrected.vel.y*(float(imuDataDelayed.time_ms) - float(gps_corrected.time_ms));
            // set the variances using the position measurement noise parameter
            // 使用位置测量噪声参数设置方差
            P[6][6] = P[7][7] = sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));
            // clear the timeout flags and counters
            // 清除超时标志和计数器
            posTimeout = false;
            lastPosPassTime_ms = imuSampleTime_ms;
        } else if (imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250) {
            // use the range beacon data as a second preference
            // 使用范围信标数据作为第二个首选项
            stateStruct.position.x = receiverPos.x;
            stateStruct.position.y = receiverPos.y;
            // set the variances from the beacon alignment filter
            // 设置与信标对齐滤波器的方差
            P[6][6] = receiverPosCov[0][0];
            P[7][7] = receiverPosCov[1][1];
            // clear the timeout flags and counters
            // 清除超时标志和计数器
            rngBcnTimeout = false;
            lastRngBcnPassTime_ms = imuSampleTime_ms;
        } else if (imuSampleTime_ms - extNavDataDelayed.time_ms < 250) {
            // use the range beacon data as a second preference
            // 使用外部导航数据作为第二个首选项
            stateStruct.position.x = extNavDataDelayed.pos.x;
            stateStruct.position.y = extNavDataDelayed.pos.y;
            // set the variances from the beacon alignment filter
            // 设置方差
            P[7][7] = P[6][6] = sq(extNavDataDelayed.posErr);
        }
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.x = stateStruct.position.x;
        storedOutput[i].position.y = stateStruct.position.y;
    }
    outputDataNew.position.x = stateStruct.position.x;
    outputDataNew.position.y = stateStruct.position.y;
    outputDataDelayed.position.x = stateStruct.position.x;
    outputDataDelayed.position.y = stateStruct.position.y;

    // Calculate the position jump due to the reset
    // 计算由于复位引起的位置跳变
    posResetNE.x = stateStruct.position.x - posResetNE.x;
    posResetNE.y = stateStruct.position.y - posResetNE.y;

    // store the time of the reset
    // 存储重置的时间
    lastPosReset_ms = imuSampleTime_ms;

}

// reset the vertical position state using the last height measurement
// 使用最后一次高度测量重置垂直位置状态
void NavEKF2_core::ResetHeight(void)
{
    // Store the position before the reset so that we can record the reset delta
    // 在重置之前存储位置，以便我们可以记录重置增量
    posResetD = stateStruct.position.z;

    // write to the state vector
    // 写入状态向量
    stateStruct.position.z = -hgtMea;
    outputDataNew.position.z = stateStruct.position.z;
    outputDataDelayed.position.z = stateStruct.position.z;

    // reset the terrain state height
    // 重置地形状态高度
    if (onGround) {
        // assume vehicle is sitting on the ground
        // 假设车辆停在地面上
        terrainState = stateStruct.position.z + rngOnGnd;
    } else {
        // can make no assumption other than vehicle is not below ground level
        // 只能假设车辆不在地面以下
        terrainState = MAX(stateStruct.position.z + rngOnGnd , terrainState);
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.z = stateStruct.position.z;
    }
    vertCompFiltState.pos = stateStruct.position.z;

    // Calculate the position jump due to the reset
    // 计算由于复位引起的z位置跳变
    posResetD = stateStruct.position.z - posResetD;

    // store the time of the reset
    // 存储重置的时间
    lastPosResetD_ms = imuSampleTime_ms;

    // clear the timeout flags and counters
    // 清除超时标志和计数器
    hgtTimeout = false;
    lastHgtPassTime_ms = imuSampleTime_ms;

    // reset the corresponding covariances
    // 重置相应的协方差
    zeroRows(P,8,8);
    zeroCols(P,8,8);

    // set the variances to the measurement variance
    // 将方差设置为测量方差
    P[8][8] = posDownObsNoise;

    // Reset the vertical velocity state using GPS vertical velocity if we are airborne
    // Check that GPS vertical velocity data is available and can be used
    // 如果我们在空中，使用GPS垂直速度重置垂直速度状态
    // 检查GPS垂直速度数据是否可用并可使用
    if (inFlight && !gpsNotAvailable && frontend->_fusionModeGPS == 0 && !frontend->inhibitGpsVertVelUse) {
        stateStruct.velocity.z =  gpsDataNew.vel.z;
    } else if (onGround) {
        stateStruct.velocity.z = 0.0f;
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.z = stateStruct.velocity.z;
    }
    outputDataNew.velocity.z = stateStruct.velocity.z;
    outputDataDelayed.velocity.z = stateStruct.velocity.z;
    vertCompFiltState.vel = outputDataNew.velocity.z;

    // reset the corresponding covariances
    // 重置相应的协方差
    zeroRows(P,5,5);
    zeroCols(P,5,5);

    // set the variances to the measurement variance
    // 将方差设置为测量方差
    P[5][5] = sq(frontend->_gpsVertVelNoise);

}

// Zero the EKF height datum
// Return true if the height datum reset has been performed
bool NavEKF2_core::resetHeightDatum(void)
{
    if (activeHgtSource == HGT_SOURCE_RNG || !onGround) {
        // only allow resets when on the ground.
        // If using using rangefinder for height then never perform a
        // reset of the height datum
        return false;
    }
    // record the old height estimate
    float oldHgt = -stateStruct.position.z;
    // reset the barometer so that it reads zero at the current height
    AP::baro().update_calibration();
    // reset the height state
    stateStruct.position.z = 0.0f;
    // adjust the height of the EKF origin so that the origin plus baro height before and after the reset is the same

    if (validOrigin) {
        if (!gpsGoodToAlign) {
            // if we don't have GPS lock then we shouldn't be doing a
            // resetHeightDatum, but if we do then the best option is
            // to maintain the old error
            EKF_origin.alt += (int32_t)(100.0f * oldHgt);
        } else {
            // if we have a good GPS lock then reset to the GPS
            // altitude. This ensures the reported AMSL alt from
            // getLLH() is equal to GPS altitude, while also ensuring
            // that the relative alt is zero
            EKF_origin.alt = AP::gps().location().alt;
        }
        ekfGpsRefHgt = (double)0.01 * (double)EKF_origin.alt;
    }

    // set the terrain state to zero (on ground). The adjustment for
    // frame height will get added in the later constraints
    terrainState = 0;
    return true;
}

/*
  correct GPS data for position offset of antenna phase centre relative to the IMU
  校正天线相位中心相对于IMU的位置偏移的GPS数据
*/
void NavEKF2_core::CorrectGPSForAntennaOffset(gps_elements &gps_data)
{
    const Vector3f &posOffsetBody = AP::gps().get_antenna_offset(gpsDataDelayed.sensor_idx) - accelPosOffset;
    if (posOffsetBody.is_zero()) {
        return;
    }

    // Don't fuse velocity data if GPS doesn't support it
    if (fuseVelData) {
        // TODO use a filtered angular rate with a group delay that matches the GPS delay
        Vector3f angRate = imuDataDelayed.delAng * (1.0f/imuDataDelayed.delAngDT);
        Vector3f velOffsetBody = angRate % posOffsetBody;
        Vector3f velOffsetEarth = prevTnb.mul_transpose(velOffsetBody);
        gps_data.vel.x -= velOffsetEarth.x;
        gps_data.vel.y -= velOffsetEarth.y;
        gps_data.vel.z -= velOffsetEarth.z;
    }

    Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
    gps_data.pos.x -= posOffsetEarth.x;
    gps_data.pos.y -= posOffsetEarth.y;
    gps_data.hgt += posOffsetEarth.z;
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/
// select fusion of velocity, position and height measurements
void NavEKF2_core::SelectVelPosFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    /*检查磁力计是否已在该时间步长上融合，且滤波器的运行速度是否超过200 Hz
     *如果是这样，请不要在此时间步长上融合测量值，以减少帧过度运行
     *仅允许一次时间滑动，以防止高速磁力计数据妨碍其他测量值的融合
    */
    if (magFusePerformed && dtIMUavg < 0.005f && !posVelFusionDelayed) {
        posVelFusionDelayed = true;
        return;
    } else {
        posVelFusionDelayed = false;
    }

    // Check for data at the fusion time horizon
    // 在融合时间范围内检查数据
    extNavDataToFuse = storedExtNav.recall(extNavDataDelayed, imuDataDelayed.time_ms);//外部导航数据需要融合标志?

    // read GPS data from the sensor and check for new data in the buffer
    // 从传感器读取GPS数据，并检查缓冲器中是否有新数据
    readGpsData();
    gpsDataToFuse = storedGPS.recall(gpsDataDelayed,imuDataDelayed.time_ms);
    // Determine if we need to fuse position and velocity data on this time step
    // 确定是否需要在此时间步长上融合位置和速度数据
    if (gpsDataToFuse && PV_AidingMode == AID_ABSOLUTE) {
        // set fusion request flags
        // 设置融合请求标志
        if (frontend->_fusionModeGPS <= 1) {
            fuseVelData = true;
        } else {
            fuseVelData = false;
        }
        fusePosData = true;
        extNavUsedForPos = false;

        //修正天线位置偏移-- correct for antenna position
        CorrectGPSForAntennaOffset(gpsDataDelayed);

        // copy corrected GPS data to observation vector
        // 将修正后的GPS数据复制到观测向量
        if (fuseVelData) {
            velPosObs[0] = gpsDataDelayed.vel.x;
            velPosObs[1] = gpsDataDelayed.vel.y;
            velPosObs[2] = gpsDataDelayed.vel.z;
        }
        velPosObs[3] = gpsDataDelayed.pos.x;
        velPosObs[4] = gpsDataDelayed.pos.y;

    } else if (extNavDataToFuse && PV_AidingMode == AID_ABSOLUTE) {
        // This is a special case that uses and external nav system for position
        // 这是一种特殊情况，使用外部导航系统进行定位
        extNavUsedForPos = true;
        activeHgtSource = HGT_SOURCE_EV;
        fuseVelData = false;
        fuseHgtData = true;
        fusePosData = true;
        velPosObs[3] = extNavDataDelayed.pos.x;
        velPosObs[4] = extNavDataDelayed.pos.y;
        velPosObs[5] = extNavDataDelayed.pos.z;

        // if compass is disabled, also use it for yaw
        // 如果罗盘被禁用，也可用于偏航
        if (!use_compass()) {
            extNavUsedForYaw = true;
            if (!yawAlignComplete) {
                extNavYawResetRequest = true;
                magYawResetRequest = false;
                gpsYawResetRequest = false;
                controlMagYawReset();
                finalInflightYawInit = true;
            } else {
                fuseEulerYaw();
            }
        } else {//罗盘没禁用
            extNavUsedForYaw = false;
        }

    } else {
        fuseVelData = false;
        fusePosData = false;
    }

    // we have GPS data to fuse and a request to align the yaw using the GPS course
    // 我们需要融合GPS数据，并请求使用GPS航向对齐偏航
    if (gpsYawResetRequest) {
        realignYawGPS();
    }

    // Select height data to be fused from the available baro, range finder and GPS sources
    // 从可用的baro、测距仪和GPS源中选择要融合的高度数据
    selectHeightForFusion();

    // if we are using GPS, check for a change in receiver and reset position and height
    // 如果我们使用的是GPS，请检查接收器是否有变化，并重置位置和高度
    if (gpsDataToFuse && PV_AidingMode == AID_ABSOLUTE && gpsDataDelayed.sensor_idx != last_gps_idx) {
        // record the ID of the GPS that we are using for the reset
        last_gps_idx = gpsDataDelayed.sensor_idx;//如果我们使用的是GPS，请检查接收器是否有变化，并重置位置和高度

        // Store the position before the reset so that we can record the reset delta
        posResetNE.x = stateStruct.position.x;
        posResetNE.y = stateStruct.position.y;//在重置之前存储位置，以便我们可以记录重置增量

        // Set the position states to the position from the new GPS
        stateStruct.position.x = gpsDataDelayed.pos.x;
        stateStruct.position.y = gpsDataDelayed.pos.y;//将位置状态设置为新GPS的位置

        // Calculate the position offset due to the reset
        posResetNE.x = stateStruct.position.x - posResetNE.x;
        posResetNE.y = stateStruct.position.y - posResetNE.y;//计算重置导致的位置偏移

        // Add the offset to the output observer states
        for (uint8_t i=0; i<imu_buffer_length; i++) {
            storedOutput[i].position.x += posResetNE.x;//将偏移量添加到输出观察者状态
            storedOutput[i].position.y += posResetNE.y;
        }
        outputDataNew.position.x += posResetNE.x;
        outputDataNew.position.y += posResetNE.y;
        outputDataDelayed.position.x += posResetNE.x;
        outputDataDelayed.position.y += posResetNE.y;

        // store the time of the reset
        lastPosReset_ms = imuSampleTime_ms;//存储重置的时间

        // If we are also using GPS as the height reference, reset the height
        // 如果我们也使用GPS作为高度参考，请重置高度
        if (activeHgtSource == HGT_SOURCE_GPS) {
            // Store the position before the reset so that we can record the reset delta
            posResetD = stateStruct.position.z;//在重置之前存储位置，以便我们可以记录重置增量

            // write to the state vector
            stateStruct.position.z = -hgtMea;//写入状态向量

            // Calculate the position jump due to the reset
            posResetD = stateStruct.position.z - posResetD;//计算由于重置引起的位置跳变

            // Add the offset to the output observer states
            outputDataNew.position.z += posResetD;//将偏移量添加到输出观察者状态
            vertCompFiltState.pos = outputDataNew.position.z;
            outputDataDelayed.position.z += posResetD;
            for (uint8_t i=0; i<imu_buffer_length; i++) {
                storedOutput[i].position.z += posResetD;
            }

            // store the time of the reset
            lastPosResetD_ms = imuSampleTime_ms;//存储重置的时间
        }
    }

    // If we are operating without any aiding, fuse in the last known position
    // to constrain tilt drift. This assumes a non-manoeuvring vehicle
    // Do this to coincide with the height fusion
    /*如果我们在没有任何辅助的情况下操作，则在最后已知的位置融合，以限制倾斜漂移。这假定为非机动车辆,这样做是为了与高度一致*/
    if (fuseHgtData && PV_AidingMode == AID_NONE) {
        velPosObs[3] = lastKnownPositionNE.x;
        velPosObs[4] = lastKnownPositionNE.y;
        fusePosData = true;
        fuseVelData = false;
    }

    //进行融合-- perform fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        FuseVelPosNED();
        // clear the flags to prevent repeated fusion of the same data
        fuseVelData = false;//清除标志以防止重复融合相同的数据
        fuseHgtData = false;
        fusePosData = false;
    }
}

// fuse selected position, velocity and height measurements
/*融合选定的位置、速度和高度测量值*/
void NavEKF2_core::FuseVelPosNED()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseVelPosNED);

    // health is set bad until test passed
    velHealth = false;//在测试通过之前，运行状况设置为坏
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    Vector3f velInnov;

    // declare variables used to control access to arrays
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    Vector6 R_OBS; // 用于融合的测量方差--Measurement variances used for fusion
    Vector6 R_OBS_DATA_CHECKS; //测量方差仅用于数据检查-- Measurement variances used for data checks only
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    /*执行GPS测量的顺序融合。这假设不同速度和位置分量中的误差不相关，这是不正确的。但在没有协方差的情况下，
      来自GPS接收器的数据这是我们唯一能做的假设,因此，我们不妨利用与顺序融合相关的计算效率*/
    if (fuseVelData || fusePosData || fuseHgtData) {

        // calculate additional error in GPS position caused by manoeuvring
        // 计算操纵引起的GPS位置附加误差
        float posErr = frontend->gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        // 估计GPS速度、GPS水平位置和高度测量方差。
        // Use different errors if operating without external aiding using an assumed position or velocity of zero
        // 如果在没有外部辅助的情况下使用假定的零位置或零速度进行操作，则使用不同的误差
        if (PV_AidingMode == AID_NONE) {
            if (tiltAlignComplete && motorsArmed) {
            // This is a compromise between corrections for gyro errors and reducing effect of manoeuvre accelerations on tilt estimate
            // 这是陀螺误差校正和减小操纵加速度对倾斜估计影响之间的折衷
                R_OBS[0] = sq(constrain_float(frontend->_noaidHorizNoise, 0.5f, 50.0f));
            } else {
                // Use a smaller value to give faster initial alignment
                // 这是陀螺误差校正和减小操纵加速度对倾斜估计影响之间的折衷
                R_OBS[0] = sq(0.5f);
            }
            R_OBS[1] = R_OBS[0];
            R_OBS[2] = R_OBS[0];
            R_OBS[3] = R_OBS[0];
            R_OBS[4] = R_OBS[0];
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];
        } else {
            if (gpsSpdAccuracy > 0.0f) {//gpsSpdAccuracy = 0
                // use GPS receivers reported speed accuracy if available and floor at value set by GPS velocity noise parameter
                // 使用GPS接收机报告的速度精度（如果可用），并使用由GPS速度噪声参数设置的值
                R_OBS[0] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsHorizVelNoise, 50.0f));
                R_OBS[2] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsVertVelNoise, 50.0f));
            } else {
                // calculate additional error in GPS velocity caused by manoeuvring
                // 计算操纵引起的GPS速度附加误差
                R_OBS[0] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
                R_OBS[2] = sq(constrain_float(frontend->_gpsVertVelNoise,  0.05f, 5.0f)) + sq(frontend->gpsDVelVarAccScale  * accNavMag);
            }
            R_OBS[1] = R_OBS[0];
            // Use GPS reported position accuracy if available and floor at value set by GPS position noise parameter
            // 使用GPS报告的位置精度（如果可用）和由GPS位置噪声参数设置的值
            if (gpsPosAccuracy > 0.0f) {
                R_OBS[3] = sq(constrain_float(gpsPosAccuracy, frontend->_gpsHorizPosNoise, 100.0f));
            } else if (extNavUsedForPos) { //extNavUsedForPos = 0
                R_OBS[3] = sq(constrain_float(extNavDataDelayed.posErr, 0.01f, 10.0f));
            } else {
                R_OBS[3] = sq(constrain_float(frontend->_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
            }
            R_OBS[4] = R_OBS[3];
            // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
            // For horizontal GPS velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPS perfomrance
            // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
            /*对于数据完整性检查，我们使用与用于计算除GPS水平速度以外的所有测量值的卡尔曼增益相同的测量方差
              对于水平GPS速度，我们不希望接受半径随报告的GPS精度增加，因此我们使用基于最佳GPS性能的值
              再加上机动的余地。最好尽早剔除GPS水平速度误差*/
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
        }
        R_OBS[5] = posDownObsNoise;//状态和协方差更新步骤使用的垂直位置上的观测噪声方差
        for (uint8_t i=3; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];

        // if vertical GPS velocity data and an independent height source is being used, check to see if the GPS vertical velocity and altimeter
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        /*如果正在使用垂直GPS速度数据和独立的高度源，请检查GPS垂直速度和高度计残差是否具有相同的标志且超出限制。
          如果是这样，那么可能是混叠影响了加速度计，我们应该禁用GPS和气压计残差一致性检查?*/
        if (useGpsVertVel && fuseVelData && (frontend->_altSource != 2)) {//默认是气压计:0
            // calculate innovations for height and vertical GPS vel measurements
            // 计算高度和垂直GPS水平测量的残差
            float hgtErr  = stateStruct.position.z - velPosObs[5];
            float velDErr = stateStruct.velocity.z - velPosObs[2];
            // check if they are the same sign and both more than 3-sigma out of bounds
            // 检查它们是否是相同的符号，并且都超过3σ^2
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > 9.0f * (P[8][8] + R_OBS_DATA_CHECKS[5])) && (sq(velDErr) > 9.0f * (P[5][5] + R_OBS_DATA_CHECKS[2]))) {
                badIMUdata = true;
            } else {
                badIMUdata = false;
            }
        }

        // calculate innovations and check GPS data validity using an innovation consistency check
        // 计算残差并使用残差一致性检查检查GPS数据有效性
        // 测试位置测量--test position measurements
        if (fusePosData) {
            // 测试水平位置测量值--test horizontal position measurements
            innovVelPos[3] = stateStruct.position.x - velPosObs[3];
            innovVelPos[4] = stateStruct.position.y - velPosObs[4];
            varInnovVelPos[3] = P[6][6] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[7][7] + R_OBS_DATA_CHECKS[4];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // 应用残差一致性阈值测试，但如果IMU数据不正确，则不失败
            float maxPosInnov2 = sq(MAX(0.01f * (float)frontend->_gpsPosInnovGate, 1.0f))*(varInnovVelPos[3] + varInnovVelPos[4]);
            posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // 如果正常或超时，则使用位置数据--use position data if healthy or timed out
            if (PV_AidingMode == AID_NONE) {
                posHealth = true;
                lastPosPassTime_ms = imuSampleTime_ms;
            } else if (posHealth || posTimeout) {
                posHealth = true;
                lastPosPassTime_ms = imuSampleTime_ms;
                // if timed out or outside the specified uncertainty radius, reset to the GPS
                // 如果超时或超出规定的不确定度半径，则重置为GPS
                if (posTimeout || ((P[6][6] + P[7][7]) > sq(float(frontend->_gpsGlitchRadiusMax)))) {
                    // 将位置重置为当前GPS位置--reset the position to the current GPS position
                    ResetPosition();
                    // 将速度重置为GPS速度--reset the velocity to the GPS velocity
                    ResetVelocity();
                    // 不要在此时间步长上融合GPS数据--don't fuse GPS data on this time step
                    fusePosData = false;
                    fuseVelData = false;
                    // 将位置方差和相应的方差重置为将通过检查的值--Reset the position variances and corresponding covariances to a value that will pass the checks
                    zeroRows(P,6,7);
                    zeroCols(P,6,7);
                    P[6][6] = sq(float(0.5f*frontend->_gpsGlitchRadiusMax));
                    P[7][7] = P[6][6];
                    // 重置残差创新，以避免未通过不良融合测试--Reset the normalised innovation to avoid failing the bad fusion tests
                    posTestRatio = 0.0f;
                    velTestRatio = 0.0f;
                }
            } else {
                posHealth = false;
            }
        }

        // 测试速度测量---test velocity measurements
        if (fuseVelData) {
            // test velocity measurements
            uint8_t imax = 2;
            // 如果用户禁止或使用合成数据，则不要融合垂直速度观测值--Don't fuse vertical velocity observations if inhibited by the user or if we are using synthetic data
            if (frontend->_fusionModeGPS > 0 || PV_AidingMode != AID_ABSOLUTE || frontend->inhibitGpsVertVelUse) {
                imax = 1;
            }
            float innovVelSumSq = 0; //速度残差的平方和-- sum of squares of velocity innovations
            float varVelSum = 0; // 速度残差方差之和--sum of velocity innovation variances
            for (uint8_t i = 0; i<=imax; i++) {
                // velocity states start at index 3
                stateIndex   = i + 3;
                // 使用混合和单个IMU预测状态计算创新--calculate innovations using blended and single IMU predicted states
                velInnov[i]  = stateStruct.velocity[i] - velPosObs[i]; // blended--混合速度残差
                // 计算残差方差calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
                // --sum the innovation and innovation variances
                innovVelSumSq += sq(velInnov[i]);//求速度残差平方之和
                varVelSum += varInnovVelPos[i];//求速度残差方差之和
            }
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01f * (float)frontend->_gpsVelInnovGate, 1.0f)));
            // 如果比率大于1，则失败--fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f)  || badIMUdata);
            // 如果正常、超时或处于恒定位置模式，则使用速度数据--use velocity data if healthy, timed out, or in constant position mode
            if (velHealth || velTimeout) {
                velHealth = true;
                // 重启超时计数--restart the timeout count
                lastVelPassTime_ms = imuSampleTime_ms;
                // If we are doing full aiding and velocity fusion times out, reset to the GPS velocity
                // 如果我们正在进行全辅助和速度融合超时，请重置为GPS速度
                if (PV_AidingMode == AID_ABSOLUTE && velTimeout) {
                    // 将速度重置为GPS速度--reset the velocity to the GPS velocity
                    ResetVelocity();
                    // 不要在这个时间步长上融合GPS速度数据--don't fuse GPS velocity data on this time step
                    fuseVelData = false;
                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    // 重置标准化残差，以避免未通过不良融合测试
                    velTestRatio = 0.0f;
                }
            } else {
                velHealth = false;
            }
        }

        // 测试高度测量--test height measurements
        if (fuseHgtData) {
            // 计算高度残差--calculate height innovations
            innovVelPos[5] = stateStruct.position.z - velPosObs[5];
            varInnovVelPos[5] = P[8][8] + R_OBS_DATA_CHECKS[5];
            // 计算残差一致性测试比率--calculate the innovation consistency test ratio
            hgtTestRatio = sq(innovVelPos[5]) / (sq(MAX(0.01f * (float)frontend->_hgtInnovGate, 1.0f)) * varInnovVelPos[5]);

            // when on ground we accept a larger test ratio to allow
            // the filter to handle large switch on IMU bias errors
            // without rejecting the height sensor
            /*在地面时，我们接受更大的测试比率，以允许滤波器处理IMU上的大开关偏置误差，而不拒绝高度传感器*/
            const float maxTestRatio = (PV_AidingMode == AID_NONE && onGround)? 3.0 : 1.0;

            // fail if the ratio is > maxTestRatio, but don't fail if bad IMU data
            // 如果比率>maxTestRatio，则失败，但如果IMU数据错误，则不失败
            hgtHealth = (hgtTestRatio < maxTestRatio) || badIMUdata;

            // Fuse height data if healthy or timed out or in constant position mode
            // 融合高度数据（如果正常、超时或处于恒定位置模式）
            if (hgtHealth || hgtTimeout) {
                // Calculate a filtered value to be used by pre-flight health checks
                // We need to filter because wind gusts can generate significant baro noise and we want to be able to detect bias errors in the inertial solution
                if (onGround) {
                    float dtBaro = (imuSampleTime_ms - lastHgtPassTime_ms)*1.0e-3f;
                    const float hgtInnovFiltTC = 2.0f;
                    float alpha = constrain_float(dtBaro/(dtBaro+hgtInnovFiltTC),0.0f,1.0f);
                    hgtInnovFiltState += (innovVelPos[5]-hgtInnovFiltState)*alpha;
                } else {
                    hgtInnovFiltState = 0.0f;
                }

                // 如果超时，请重置高度--if timed out, reset the height
                if (hgtTimeout) {
                    ResetHeight();
                }

                // If we have got this far then declare the height data as healthy and reset the timeout counter
                // 如果我们已经做到了这一点，那么声明高度数据为健康状态，并重置超时计数器
                hgtHealth = true;
                lastHgtPassTime_ms = imuSampleTime_ms;
            }
        }

        // set range for sequential fusion of velocity and position measurements depending on which data is available and its health
        // 根据可用数据及其满足健康状况，设置速度和位置测量顺序融合的范围
        if (fuseVelData && velHealth) {
            fuseData[0] = true;
            fuseData[1] = true;
            if (useGpsVertVel) {
                fuseData[2] = true;
            }
            tiltErrVec.zero();//基于Vel、Pos融合的最新姿态误差修正向量置零
        }
        if (fusePosData && posHealth) {
            fuseData[3] = true;
            fuseData[4] = true;
            tiltErrVec.zero();
        }
        if (fuseHgtData && hgtHealth) {
            fuseData[5] = true;
        }

        // 按顺序融合测量值--fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 3 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                // adjust scaling on GPS measurement noise variances if not enough satellites
                /*如果融合高度数据，则使用来自不同时间坐标的状态计算测量残差
                  如果没有足够的卫星，调整GPS测量噪声方差的比例*/
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] - velPosObs[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                }
                else if (obsIndex == 3 || obsIndex == 4) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - velPosObs[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else if (obsIndex == 5) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - velPosObs[obsIndex];
                    const float gndMaxBaroErr = 4.0f;
                    const float gndBaroInnovFloor = -0.5f;

                    if(getTouchdownExpected() && activeHgtSource == HGT_SOURCE_BARO) {
                        // when a touchdown is expected, floor the barometer innovation at gndBaroInnovFloor
                        // constrain the correction between 0 and gndBaroInnovFloor+gndMaxBaroErr
                        // this function looks like this:
                        //         |/
                        //---------|---------
                        //    ____/|
                        //   /     |
                        //  /      |
                        innovVelPos[5] += constrain_float(-innovVelPos[5]+gndBaroInnovFloor, 0.0f, gndBaroInnovFloor+gndMaxBaroErr);
                    }
                }

                // calculate the Kalman gain and calculate innovation variances
                // 计算卡尔曼增益并计算新息方差
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=15; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;//观测矩阵单位阵
                }

                // inhibit magnetic field state estimation by setting Kalman gains to zero
                // 通过将卡尔曼增益设置为零来抑制磁场状态估计
                if (!inhibitMagStates) {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = 0.0f;
                    }
                }

                // inhibit wind state estimation by setting Kalman gains to zero
                // 通过将卡尔曼增益设置为零来抑制风状态估计
                if (!inhibitWindStates) {
                    Kfusion[22] = P[22][stateIndex]*SK;
                    Kfusion[23] = P[23][stateIndex]*SK;
                } else {
                    Kfusion[22] = 0.0f;
                    Kfusion[23] = 0.0f;
                }

                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                // Check that we are not going to drive any variances negative and skip the update if so
                bool healthyFusion = true;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    if (KHP[i][i] > P[i][i]) {
                        healthyFusion = false;
                    }
                }
                if (healthyFusion) {
                    // update the covariance matrix
                    for (uint8_t i= 0; i<=stateIndexLim; i++) {
                        for (uint8_t j= 0; j<=stateIndexLim; j++) {
                            P[i][j] = P[i][j] - KHP[i][j];
                        }
                    }

                    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
                    ForceSymmetry();
                    ConstrainVariances();

                    // update the states
                    // zero the attitude error state - by definition it is assumed to be zero before each observation fusion
                    stateStruct.angErr.zero();

                    // calculate state corrections and re-normalise the quaternions for states predicted using the blended IMU data
                    for (uint8_t i = 0; i<=stateIndexLim; i++) {
                        statesArray[i] = statesArray[i] - Kfusion[i] * innovVelPos[obsIndex];
                    }

                    // the first 3 states represent the angular misalignment vector. This is
                    // is used to correct the estimated quaternion
                    stateStruct.quat.rotate(stateStruct.angErr);

                    // sum the attitude error from velocity and position fusion only
                    // used as a metric for convergence monitoring
                    /*速度和位置融合产生的姿态误差之和仅用作收敛监测的度量*/
                    if (obsIndex != 5) {
                        tiltErrVec += stateStruct.angErr;
                    }
                    // 记录良好的融合状态--record good fusion status
                    if (obsIndex == 0) {
                        faultStatus.bad_nvel = false;
                    } else if (obsIndex == 1) {
                        faultStatus.bad_evel = false;
                    } else if (obsIndex == 2) {
                        faultStatus.bad_dvel = false;
                    } else if (obsIndex == 3) {
                        faultStatus.bad_npos = false;
                    } else if (obsIndex == 4) {
                        faultStatus.bad_epos = false;
                    } else if (obsIndex == 5) {
                        faultStatus.bad_dpos = false;
                    }
                } else {
                    //记录不良融合状态-- record bad fusion status
                    if (obsIndex == 0) {
                        faultStatus.bad_nvel = true;
                    } else if (obsIndex == 1) {
                        faultStatus.bad_evel = true;
                    } else if (obsIndex == 2) {
                        faultStatus.bad_dvel = true;
                    } else if (obsIndex == 3) {
                        faultStatus.bad_npos = true;
                    } else if (obsIndex == 4) {
                        faultStatus.bad_epos = true;
                    } else if (obsIndex == 5) {
                        faultStatus.bad_dpos = true;
                    }
                }
            }
        }
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseVelPosNED);
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// select the height measurement to be fused from the available baro, range finder and GPS sources
void NavEKF2_core::selectHeightForFusion()
{
    // Read range finder data and check for new data in the buffer
    // This data is used by both height and optical flow fusion processing
    readRangeFinder();
    rangeDataToFuse = storedRange.recall(rangeDataDelayed,imuDataDelayed.time_ms);

    // correct range data for the body frame position offset relative to the IMU
    // the corrected reading is the reading that would have been taken if the sensor was
    // co-located with the IMU
    if (rangeDataToFuse) {
        AP_RangeFinder_Backend *sensor = frontend->_rng.get_backend(rangeDataDelayed.sensor_idx);
        if (sensor != nullptr) {
            Vector3f posOffsetBody = sensor->get_pos_offset() - accelPosOffset;
            if (!posOffsetBody.is_zero()) {
                Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
                rangeDataDelayed.rng += posOffsetEarth.z / prevTnb.c.z;
            }
        }
    }

    // read baro height data from the sensor and check for new data in the buffer
    readBaroData();
    baroDataToFuse = storedBaro.recall(baroDataDelayed, imuDataDelayed.time_ms);

    // select height source
    if (extNavUsedForPos) {
        // always use external vision as the height source if using for position.
        activeHgtSource = HGT_SOURCE_EV;
    } else if (((frontend->_useRngSwHgt > 0) && (frontend->_altSource == 1)) && (imuSampleTime_ms - rngValidMeaTime_ms < 500)) {
        if (frontend->_altSource == 1) {
            // always use range finder
            activeHgtSource = HGT_SOURCE_RNG;
        } else {
            // determine if we are above or below the height switch region
            float rangeMaxUse = 1e-4f * (float)frontend->_rng.max_distance_cm_orient(ROTATION_PITCH_270) * (float)frontend->_useRngSwHgt;
            bool aboveUpperSwHgt = (terrainState - stateStruct.position.z) > rangeMaxUse;
            bool belowLowerSwHgt = (terrainState - stateStruct.position.z) < 0.7f * rangeMaxUse;

            // If the terrain height is consistent and we are moving slowly, then it can be
            // used as a height reference in combination with a range finder
            // apply a hysteresis to the speed check to prevent rapid switching
            float horizSpeed = norm(stateStruct.velocity.x, stateStruct.velocity.y);
            bool dontTrustTerrain = ((horizSpeed > frontend->_useRngSwSpd) && filterStatus.flags.horiz_vel) || !terrainHgtStable;
            float trust_spd_trigger = MAX((frontend->_useRngSwSpd - 1.0f),(frontend->_useRngSwSpd * 0.5f));
            bool trustTerrain = (horizSpeed < trust_spd_trigger) && terrainHgtStable;

            /*
             * Switch between range finder and primary height source using height above ground and speed thresholds with
             * hysteresis to avoid rapid switching. Using range finder for height requires a consistent terrain height
             * which cannot be assumed if the vehicle is moving horizontally.
            */
            if ((aboveUpperSwHgt || dontTrustTerrain) && (activeHgtSource == HGT_SOURCE_RNG)) {
                // cannot trust terrain or range finder so stop using range finder height
                if (frontend->_altSource == 0) {
                    activeHgtSource = HGT_SOURCE_BARO;
                } else if (frontend->_altSource == 2) {
                    activeHgtSource = HGT_SOURCE_GPS;
                }
            } else if (belowLowerSwHgt && trustTerrain && (activeHgtSource != HGT_SOURCE_RNG)) {
                // reliable terrain and range finder so start using range finder height
                activeHgtSource = HGT_SOURCE_RNG;
            }
        }
    } else if ((frontend->_altSource == 2) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) < 500) && validOrigin && gpsAccuracyGood) {
        activeHgtSource = HGT_SOURCE_GPS;
    } else if ((frontend->_altSource == 3) && validOrigin && rngBcnGoodToAlign) {
        activeHgtSource = HGT_SOURCE_BCN;
    } else {
        activeHgtSource = HGT_SOURCE_BARO;
    }

    // Use Baro alt as a fallback if we lose range finder, GPS or external nav
    bool lostRngHgt = ((activeHgtSource == HGT_SOURCE_RNG) && ((imuSampleTime_ms - rngValidMeaTime_ms) > 500));
    bool lostGpsHgt = ((activeHgtSource == HGT_SOURCE_GPS) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) > 2000));
    bool lostExtNavHgt = ((activeHgtSource == HGT_SOURCE_EV) && ((imuSampleTime_ms - extNavMeasTime_ms) > 2000));
    if (lostRngHgt || lostGpsHgt || lostExtNavHgt) {
        activeHgtSource = HGT_SOURCE_BARO;
    }

    // if there is new baro data to fuse, calculate filtered baro data required by other processes
    if (baroDataToFuse) {
        // calculate offset to baro data that enables us to switch to Baro height use during operation
        if  (activeHgtSource != HGT_SOURCE_BARO) {
            calcFiltBaroOffset();
        }
        // filtered baro data used to provide a reference for takeoff
        // it is is reset to last height measurement on disarming in performArmingChecks()
        if (!getTakeoffExpected()) {
            const float gndHgtFiltTC = 0.5f;
            const float dtBaro = frontend->hgtAvg_ms*1.0e-3f;
            float alpha = constrain_float(dtBaro / (dtBaro+gndHgtFiltTC),0.0f,1.0f);
            meaHgtAtTakeOff += (baroDataDelayed.hgt-meaHgtAtTakeOff)*alpha;
        }
    }

    // If we are not using GPS as the primary height sensor, correct EKF origin height so that
    // combined local NED position height and origin height remains consistent with the GPS altitude
    // This also enables the GPS height to be used as a backup height source
    if (gpsDataToFuse &&
            (((frontend->_originHgtMode & (1 << 0)) && (activeHgtSource == HGT_SOURCE_BARO)) ||
            ((frontend->_originHgtMode & (1 << 1)) && (activeHgtSource == HGT_SOURCE_RNG)))
            ) {
        correctEkfOriginHeight();
    }

    // Select the height measurement source
    if (extNavDataToFuse && (activeHgtSource == HGT_SOURCE_EV)) {
        hgtMea = -extNavDataDelayed.pos.z;
        posDownObsNoise = sq(constrain_float(extNavDataDelayed.posErr, 0.01f, 10.0f));
    } else if (rangeDataToFuse && (activeHgtSource == HGT_SOURCE_RNG)) {
        // using range finder data
        // correct for tilt using a flat earth model
        if (prevTnb.c.z >= 0.7) {
            // calculate height above ground
            hgtMea  = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);
            // correct for terrain position relative to datum
            hgtMea -= terrainState;
            // enable fusion
            fuseHgtData = true;
            velPosObs[5] = -hgtMea;
            // set the observation noise
            posDownObsNoise = sq(constrain_float(frontend->_rngNoise, 0.1f, 10.0f));
            // add uncertainty created by terrain gradient and vehicle tilt
            posDownObsNoise += sq(rangeDataDelayed.rng * frontend->_terrGradMax) * MAX(0.0f , (1.0f - sq(prevTnb.c.z)));
        } else {
            // disable fusion if tilted too far
            fuseHgtData = false;
        }
    } else if  (gpsDataToFuse && (activeHgtSource == HGT_SOURCE_GPS)) {
        // using GPS data
        hgtMea = gpsDataDelayed.hgt;
        // enable fusion
        velPosObs[5] = -hgtMea;
        fuseHgtData = true;
        // set the observation noise using receiver reported accuracy or the horizontal noise scaled for typical VDOP/HDOP ratio
        if (gpsHgtAccuracy > 0.0f) {
            posDownObsNoise = sq(constrain_float(gpsHgtAccuracy, 1.5f * frontend->_gpsHorizPosNoise, 100.0f));
        } else {
            posDownObsNoise = sq(constrain_float(1.5f * frontend->_gpsHorizPosNoise, 0.1f, 10.0f));
        }
    } else if (baroDataToFuse && (activeHgtSource == HGT_SOURCE_BARO)) {
        // using Baro data
        hgtMea = baroDataDelayed.hgt - baroHgtOffset;
        // enable fusion
        velPosObs[5] = -hgtMea;
        fuseHgtData = true;
        // set the observation noise
        posDownObsNoise = sq(constrain_float(frontend->_baroAltNoise, 0.1f, 10.0f));
        // reduce weighting (increase observation noise) on baro if we are likely to be in ground effect
        if (getTakeoffExpected() || getTouchdownExpected()) {
            posDownObsNoise *= frontend->gndEffectBaroScaler;
        }
        // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
        // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
        if (motorsArmed && getTakeoffExpected()) {
            hgtMea = MAX(hgtMea, meaHgtAtTakeOff);
        }
    } else {
        fuseHgtData = false;
    }

    // If we haven't fused height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime_ms = (useGpsVertVel && !velTimeout) ? frontend->hgtRetryTimeMode0_ms : frontend->hgtRetryTimeMode12_ms;
    if (imuSampleTime_ms - lastHgtPassTime_ms > hgtRetryTime_ms) {
        hgtTimeout = true;
    } else {
        hgtTimeout = false;
    }
}

