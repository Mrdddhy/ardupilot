#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;


/* Monitor GPS data to see if quality is good enough to initialise the EKF
   Monitor magnetometer innovations to see if the heading is good enough to use GPS
   Return true if all criteria pass for 10 seconds
 * 监控GPS数据，查看质量是否足以初始化EKF
   监测磁力计的残差，看看航向是否足以使用GPS
   如果所有条件均通过10秒，则返回true

   We also record the failure reason so that prearm_failure_reason()
   can give a good report to the user on why arming is failing
*  我们还记录了故障原因，以便prearm_failure_reason()可以向用户很好地报告解锁失败的原因
   This sets gpsGoodToAlign class variable
*  这将设置gpsGoodToAlign类变量
*/
void NavEKF2_core::calcGpsGoodToAlign(void)
{
    const AP_GPS &gps = AP::gps();

    if (inFlight && assume_zero_sideslip() && !use_compass()) {
        // this is a special case where a plane has launched without magnetometer
        // is now in the air and needs to align yaw to the GPS and start navigating as soon as possible
        gpsGoodToAlign = true;
        return;
    }

    // User defined multiplier to be applied to check thresholds
    // 用于检查阈值的用户定义乘数
    float checkScaler = 0.01f*(float)frontend->_gpsCheckScaler;

    if (gpsGoodToAlign) {
        /*
          if we have already passed GPS alignment checks then raise
          the check threshold so that we have some hysterisis and
          don't continuously change from able to arm to not able to
          arm
          如果我们已经通过了GPS校准检查，那么提高检查阈值，这样我们就会有一些迟滞，不会不断地从“能够启用”更改为“无法启用”
         */
        checkScaler *= 1.3f;
    }

    // If we have good magnetometer consistency and bad innovations for longer than 5 seconds then we reset heading and field states
    // This enables us to handle large changes to the external magnetic field environment that occur before arming
    /* 如果我们有好的磁强计一致性和坏的残差超过5秒，然后我们重置航向和磁场状态
       这使我们能够处理在解锁前发生的外部磁场环境的巨大变化*/
    if ((magTestRatio.x <= 1.0f && magTestRatio.y <= 1.0f && yawTestRatio <= 1.0f) || !consistentMagData) {
        magYawResetTimer_ms = imuSampleTime_ms;
    }
    if ((imuSampleTime_ms - magYawResetTimer_ms > 5000) && !motorsArmed) {
        // request reset of heading and magnetic field states
        // 请求重置航向和磁场状态
        magYawResetRequest = true;
        // reset timer to ensure that bad magnetometer data cannot cause a heading reset more often than every 5 seconds
        // 重置计时器，以确保坏磁强计数据不会导致航向重置的频率超过每5秒一次
        magYawResetTimer_ms = imuSampleTime_ms;
    }

    // Check for significant change in GPS position if disarmed which indicates bad GPS
    // This check can only be used when the vehicle is stationary
    /* 如果解锁，检查GPS位置是否有明显变化，这表明GPS存在故障,此检查仅在车辆静止时使用*/
    const struct Location &gpsloc = gps.location(); //当前位置-- Current location
    const float posFiltTimeConst = 10.0f; //用于衰减位置漂移的时间常数-- time constant used to decay position drift
    // calculate time lapsed since last update and limit to prevent numerical errors
    // 计算自上次更新以来经过的时间和限制，以防止数值错误
    float deltaTime = constrain_float(float(imuDataDelayed.time_ms - lastPreAlignGpsCheckTime_ms)*0.001f,0.01f,posFiltTimeConst);
    lastPreAlignGpsCheckTime_ms = imuDataDelayed.time_ms;
    // 移动的总距离---Sum distance moved
    gpsDriftNE += gpsloc_prev.get_distance(gpsloc);
    gpsloc_prev = gpsloc;
    // 衰减距离以指数形式移动到零---Decay distance moved exponentially to zero
    gpsDriftNE *= (1.0f - deltaTime/posFiltTimeConst);
    // Clamp the filter state to prevent excessive persistence of large transients
    /* 钳制滤波器状态，以防止大瞬变的过度持续*/
    gpsDriftNE = MIN(gpsDriftNE,10.0f);
    // Fail if more than 3 metres drift after filtering whilst on-ground
    // This corresponds to a maximum acceptable average drift rate of 0.3 m/s or single glitch event of 3m
    /* 如果在地面滤波后漂移超过3米，则失败
       这对应于0.3 m/s的最大可接受平均漂移率或3m的单次故障事件*/
    bool gpsDriftFail = (gpsDriftNE > 3.0f*checkScaler) && onGround && (frontend->_gpsCheck & MASK_GPS_POS_DRIFT);

    // Report check result as a text string and bitmask
    // 将检查结果报告为文本字符串和位掩码
    if (gpsDriftFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS drift %.1fm (needs %.1f)", (double)gpsDriftNE, (double)(3.0f*checkScaler));
        gpsCheckStatus.bad_horiz_drift = true;
    } else {
        gpsCheckStatus.bad_horiz_drift = false;
    }

    // Check that the vertical GPS vertical velocity is reasonable after noise filtering
    // 噪声滤波后检查垂直GPS垂直速度是否合理
    bool gpsVertVelFail;
    if (gps.have_vertical_velocity() && onGround) {
        // check that the average vertical GPS velocity is close to zero
        // 检查平均垂直GPS速度是否接近零
        gpsVertVelFilt = 0.1f * gpsDataNew.vel.z + 0.9f * gpsVertVelFilt;
        gpsVertVelFilt = constrain_float(gpsVertVelFilt,-10.0f,10.0f);
        gpsVertVelFail = (fabsf(gpsVertVelFilt) > 0.3f*checkScaler) && (frontend->_gpsCheck & MASK_GPS_VERT_SPD);
    } else if ((frontend->_fusionModeGPS == 0) && !gps.have_vertical_velocity()) {
        // If the EKF settings require vertical GPS velocity and the receiver is not outputting it, then fail
        // 如果EKF设置要求垂直GPS速度，且接收器未输出，则失败
        gpsVertVelFail = true;
        // if we have a 3D fix with no vertical velocity and
        // EK2_GPS_TYPE=0 then change it to 1. It means the GPS is not
        // capable of giving a vertical velocity
        /*如果我们有一个没有垂直速度和EK2_GPS_TYPE=0，然后将其更改为1。这意味着GPS无法给出垂直速度*/
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            frontend->_fusionModeGPS.set(1);
            gcs().send_text(MAV_SEVERITY_WARNING, "EK2: Changed EK2_GPS_TYPE to 1");
        }
    } else {
        gpsVertVelFail = false;
    }

    // Report check result as a text string and bitmask
    // 将检查结果报告为文本字符串和位掩码
    if (gpsVertVelFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS vertical speed %.2fm/s (needs %.2f)", (double)fabsf(gpsVertVelFilt), (double)(0.3f*checkScaler));
        gpsCheckStatus.bad_vert_vel = true;
    } else {
        gpsCheckStatus.bad_vert_vel = false;
    }

    // Check that the horizontal GPS vertical velocity is reasonable after noise filtering
    // This check can only be used if the vehicle is stationary
    /* 噪声滤波后检查水平GPS垂直速度是否合理,此检查仅在车辆静止时使用*/
    bool gpsHorizVelFail;
    if (onGround) {
        gpsHorizVelFilt = 0.1f * norm(gpsDataDelayed.vel.x,gpsDataDelayed.vel.y) + 0.9f * gpsHorizVelFilt;
        gpsHorizVelFilt = constrain_float(gpsHorizVelFilt,-10.0f,10.0f);
        gpsHorizVelFail = (fabsf(gpsHorizVelFilt) > 0.3f*checkScaler) && (frontend->_gpsCheck & MASK_GPS_HORIZ_SPD);
    } else {
        gpsHorizVelFail = false;
    }

    // Report check result as a text string and bitmask
    // 将检查结果报告为文本字符串和位掩码
    if (gpsHorizVelFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS horizontal speed %.2fm/s (needs %.2f)", (double)gpsDriftNE, (double)(0.3f*checkScaler));
        gpsCheckStatus.bad_horiz_vel = true;
    } else {
        gpsCheckStatus.bad_horiz_vel = false;
    }

    // fail if horiziontal position accuracy not sufficient
    // 如果水平位置精度不够，则失败
    float hAcc = 0.0f;
    bool hAccFail;
    if (gps.horizontal_accuracy(hAcc)) {
        hAccFail = (hAcc > 5.0f*checkScaler)  && (frontend->_gpsCheck & MASK_GPS_POS_ERR);
    } else {
        hAccFail =  false;
    }

    // Report check result as a text string and bitmask
    // 将检查结果报告为文本字符串和位掩码
    if (hAccFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS horiz error %.1fm (needs %.1f)", (double)hAcc, (double)(5.0f*checkScaler));
        gpsCheckStatus.bad_hAcc = true;
    } else {
        gpsCheckStatus.bad_hAcc = false;
    }

    // Check for vertical GPS accuracy
    // 检查垂直GPS精度
    float vAcc = 0.0f;
    bool vAccFail = false;
    if (gps.vertical_accuracy(vAcc)) {
        vAccFail = (vAcc > 7.5f * checkScaler) && (frontend->_gpsCheck & MASK_GPS_POS_ERR);
    }
    // Report check result as a text string and bitmask
    // 将检查结果报告为文本字符串和位掩码
    if (vAccFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS vert error %.1fm (needs < %.1f)", (double)vAcc, (double)(7.5f * checkScaler));
        gpsCheckStatus.bad_vAcc = true;
    } else {
        gpsCheckStatus.bad_vAcc = false;
    }

    // fail if reported speed accuracy greater than threshold
    // 如果报告的速度精度大于阈值，则失败
    bool gpsSpdAccFail = (gpsSpdAccuracy > 1.0f*checkScaler) && (frontend->_gpsCheck & MASK_GPS_SPD_ERR);

    // Report check result as a text string and bitmask
    // 将检查结果报告为文本字符串和位掩码
    if (gpsSpdAccFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS speed error %.1f (needs < %.1f)", (double)gpsSpdAccuracy, (double)(1.0f*checkScaler));
        gpsCheckStatus.bad_sAcc = true;
    } else {
        gpsCheckStatus.bad_sAcc = false;
    }

    // fail if satellite geometry is poor
    // 如果卫星几何结构不良，则失败
    bool hdopFail = (gps.get_hdop() > 250)  && (frontend->_gpsCheck & MASK_GPS_HDOP);

    // Report check result as a text string and bitmask
    // 将检查结果报告为文本字符串和位掩码
    if (hdopFail) {
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string),
                           "GPS HDOP %.1f (needs 2.5)", (double)(0.01f * gps.get_hdop()));
        gpsCheckStatus.bad_hdop = true;
    } else {
        gpsCheckStatus.bad_hdop = false;
    }

    // fail if not enough sats
    // 如果没有足够的SAT，则失败
    bool numSatsFail = (gps.num_sats() < 6) && (frontend->_gpsCheck & MASK_GPS_NSATS);

    // Report check result as a text string and bitmask
    // 将检查结果报告为文本字符串和位掩码
    if (numSatsFail) {
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string),
                           "GPS numsats %u (needs 6)", gps.num_sats());
        gpsCheckStatus.bad_sats = true;
    } else {
        gpsCheckStatus.bad_sats = false;
    }

    // fail if magnetometer innovations are outside limits indicating bad yaw
    // with bad yaw we are unable to use GPS
    // 如果磁力计残差超出限制，则失败，表明偏航不良，我们无法使用GPS
    bool yawFail;
    if ((magTestRatio.x > 1.0f || magTestRatio.y > 1.0f || yawTestRatio > 1.0f) && (frontend->_gpsCheck & MASK_GPS_YAW_ERR)) {
        yawFail = true;
    } else {
        yawFail = false;
    }

    // Report check result as a text string and bitmask
    // 将检查结果报告为文本字符串和位掩码
    if (yawFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "Mag yaw error x=%.1f y=%.1f",
                           (double)magTestRatio.x,
                           (double)magTestRatio.y);
        gpsCheckStatus.bad_yaw = true;
    } else {
        gpsCheckStatus.bad_yaw = false;
    }

    // assume failed first time through and notify user checks have started
    // 假设首次通过失败，并已开始通知用户检查
    if (lastGpsVelFail_ms == 0) {
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string), "EKF starting GPS checks");
        lastGpsVelFail_ms = imuSampleTime_ms;
    }

    // record time of fail or pass
    // 记录失败或通过的时间
    if (gpsSpdAccFail || numSatsFail || hdopFail || hAccFail || vAccFail ||  yawFail || gpsDriftFail || gpsVertVelFail || gpsHorizVelFail) {
        lastGpsVelFail_ms = imuSampleTime_ms;
    } else {
        lastGpsVelPass_ms = imuSampleTime_ms;
    }

    // continuous period of 10s without fail required to set healthy
    // continuous period of 5s without pass required to set unhealthy
    /* 要求连续10秒，且无故障，以设置健康状态
       需要连续5秒无通行证才能设置不健康*/
    if (!gpsGoodToAlign && imuSampleTime_ms - lastGpsVelFail_ms > 10000) {
        gpsGoodToAlign = true;
    } else if (gpsGoodToAlign && imuSampleTime_ms - lastGpsVelPass_ms > 5000) {
        gpsGoodToAlign = false;
    }
}

// update inflight calculaton that determines if GPS data is good enough for reliable navigation
/* 更新确定GPS数据是否足以可靠导航的飞行计算*/
void NavEKF2_core::calcGpsGoodForFlight(void)
{
    // use a simple criteria based on the GPS receivers claimed speed accuracy and the EKF innovation consistency checks
    // 使用基于GPS接收机声称的速度精度和EKF残差一致性检查的简单标准
    // set up varaibles and constants used by filter that is applied to GPS speed accuracy
    // 设置应用于GPS速度精度的滤波器使用的变量和常量
    const float alpha1 = 0.2f; // 应用于原始速度精度数据的第一级LPF系数--coefficient for first stage LPF applied to raw speed accuracy data
    const float tau = 10.0f; // 峰值保持衰减的时间常数（秒）--time constant (sec) of peak hold decay
    if (lastGpsCheckTime_ms == 0) {
        lastGpsCheckTime_ms =  imuSampleTime_ms;
    }
    float dtLPF = (imuSampleTime_ms - lastGpsCheckTime_ms) * 1e-3f;
    lastGpsCheckTime_ms = imuSampleTime_ms;
    float alpha2 = constrain_float(dtLPF/tau,0.0f,1.0f);

    // get the receivers reported speed accuracy
    // 获得接收机报告的速度精度
    float gpsSpdAccRaw;
    if (!AP::gps().speed_accuracy(gpsSpdAccRaw)) {
        gpsSpdAccRaw = 0.0f;
    }

    // filter the raw speed accuracy using a LPF
    // 使用LPF过滤原始速度精度
    sAccFilterState1 = constrain_float((alpha1 * gpsSpdAccRaw + (1.0f - alpha1) * sAccFilterState1),0.0f,10.0f);

    // apply a peak hold filter to the LPF output
    // 对LPF输出应用峰值保持滤波器
    sAccFilterState2 = MAX(sAccFilterState1,((1.0f - alpha2) * sAccFilterState2));

    // Apply a threshold test with hysteresis to the filtered GPS speed accuracy data
    // 对过滤后的GPS速度精度数据应用滞后阈值测试
    if (sAccFilterState2 > 1.5f ) {
        gpsSpdAccPass = false;
    } else if(sAccFilterState2 < 1.0f) {
        gpsSpdAccPass = true;//GPS速度精度测试标志设置为true
    }

    // Apply a threshold test with hysteresis to the normalised position and velocity innovations
    // Require a fail for one second and a pass for 10 seconds to transition
    /* 对标准化位置和速度残差应用滞后阈值测试
       需要失败1秒钟，通过10秒钟才能过渡*/
    if (lastInnovFailTime_ms == 0) {
        lastInnovFailTime_ms = imuSampleTime_ms;
        lastInnovPassTime_ms = imuSampleTime_ms;
    }
    if (velTestRatio < 1.0f && posTestRatio < 1.0f) {
        lastInnovPassTime_ms = imuSampleTime_ms;
    } else if (velTestRatio > 0.7f || posTestRatio > 0.7f) {
        lastInnovFailTime_ms = imuSampleTime_ms;
    }
    if ((imuSampleTime_ms - lastInnovPassTime_ms) > 1000) {
        ekfInnovationsPass = false;
    } else if ((imuSampleTime_ms - lastInnovFailTime_ms) > 10000) {
        ekfInnovationsPass = true;
    }

    // both GPS speed accuracy and EKF innovations must pass
    // GPS速度精度和EKF残差都必须通过
    gpsAccuracyGood = gpsSpdAccPass && ekfInnovationsPass;
}

// Detect if we are in flight or on ground
void NavEKF2_core::detectFlight()
{
    /*
        If we are a fly forward type vehicle (eg plane), then in-air status can be determined through a combination of speed and height criteria.
        Because of the differing certainty requirements of algorithms that need the in-flight / on-ground status we use two booleans where
        onGround indicates a high certainty we are not flying and inFlight indicates a high certainty we are flying. It is possible for
        both onGround and inFlight to be false if the status is uncertain, but they cannot both be true.
     *  如果我们是向前飞的飞行器(如飞机)，那么空中状态可以通过速度和高度标准的组合来确定。
        由于算法对飞行/地面状态的确定性要求不同，我们使用两个布尔值，其中onGround表示高度确定我们不飞行，inFight表示高度确定我们在飞行。
        如果状态是不确定的，地面和飞行都有可能是假的，但他们不可能都是真的
        If we are a plane as indicated by the assume_zero_sideslip() status, then different logic is used

        TODO - this logic should be moved out of the EKF and into the flight vehicle code.
    */

    if (assume_zero_sideslip()) {
        // To be confident we are in the air we use a criteria which combines arm status, ground speed, airspeed and height change
        float gndSpdSq = sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y);
        bool highGndSpd = false;
        bool highAirSpd = false;
        bool largeHgtChange = false;

        // trigger at 8 m/s airspeed
        if (_ahrs->airspeed_sensor_enabled()) {
            const AP_Airspeed *airspeed = _ahrs->get_airspeed();
            if (airspeed->get_airspeed() * AP::ahrs().get_EAS2TAS() > 10.0f) {
                highAirSpd = true;
            }
        }

        // trigger at 10 m/s GPS velocity, but not if GPS is reporting bad velocity errors
        if (gndSpdSq > 100.0f && gpsSpdAccuracy < 1.0f) {
            highGndSpd = true;
        }

        // trigger if more than 10m away from initial height
        if (fabsf(hgtMea) > 10.0f) {
            largeHgtChange = true;
        }

        // Determine to a high certainty we are flying
        if (motorsArmed && highGndSpd && (highAirSpd || largeHgtChange)) {
            onGround = false;
            inFlight = true;
        }

        // if is possible we are in flight, set the time this condition was last detected
        if (motorsArmed && (highGndSpd || highAirSpd || largeHgtChange)) {
            airborneDetectTime_ms = imuSampleTime_ms;
            onGround = false;
        }

        // Determine to a high certainty we are not flying
        // after 5 seconds of not detecting a possible flight condition or we are disarmed, we transition to on-ground mode
        if(!motorsArmed || ((imuSampleTime_ms - airborneDetectTime_ms) > 5000)) {
            onGround = true;
            inFlight = false;
        }
    } else {
        // Non fly forward vehicle, so can only use height and motor arm status
        /*车辆不能向前飞，所以只能使用高度和电机解锁状态*/
        // If the motors are armed then we could be flying and if they are not armed then we are definitely not flying
        if (motorsArmed) {
            onGround = false;/*如果电机解锁，不确定，则将onGround置false*/
        } else {
            inFlight = false;/*相反，未解锁，则肯定是在地面onGround*/
            onGround = true;
        }
        /*如果电机解锁，则再根据高度变换判断是否在飞行状态*/
        if (!onGround) {
            // If height has increased since exiting on-ground, then we definitely are flying
            if ((stateStruct.position.z - posDownAtTakeoff) < -1.5f) {
                inFlight = true;/*如果从地面出来后高度增加，那么我们肯定是在飞行*/
            }

            // If rangefinder has increased since exiting on-ground, then we definitely are flying
            if ((rangeDataNew.rng - rngAtStartOfFlight) > 0.5f) {
                inFlight = true;/*如果离开地面后测距仪增加了，那么我们肯定是在飞行*/
            }

            // If more than 5 seconds since likely_flying was set
            // true, then set inFlight true
            if (_ahrs->get_time_flying_ms() > 5000) {
                inFlight = true;/*如果likely_flying被设置为true后超过5秒，则设置inFight为true*/
            }
        }

    }

    // store current on-ground  and in-air status for next time
    prevOnGround = onGround;
    prevInFlight = inFlight;

    // Store vehicle height and range prior to takeoff for use in post takeoff checks
    if (onGround) {
        // store vertical position at start of flight to use as a reference for ground relative checks
        posDownAtTakeoff = stateStruct.position.z;
        // store the range finder measurement which will be used as a reference to detect when we have taken off
        rngAtStartOfFlight = rangeDataNew.rng;
        // if the magnetic field states have been set, then continue to update the vertical position
        // quaternion and yaw innovation snapshots to use as a reference when we start to fly.
        if (magStateInitComplete) {
            posDownAtLastMagReset = stateStruct.position.z;
            quatAtLastMagReset = stateStruct.quat;
            yawInnovAtLastMagReset = innovYaw;
        }
    }

}


// determine if a takeoff is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF2_core::getTakeoffExpected()
{
    if (expectGndEffectTakeoff && imuSampleTime_ms - takeoffExpectedSet_ms > frontend->gndEffectTimeout_ms) {
        expectGndEffectTakeoff = false;
    }

    return expectGndEffectTakeoff;
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2_core::setTakeoffExpected(bool val)
{
    takeoffExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTakeoff = val;
}


// determine if a touchdown is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF2_core::getTouchdownExpected()
{
    if (expectGndEffectTouchdown && imuSampleTime_ms - touchdownExpectedSet_ms > frontend->gndEffectTimeout_ms) {
        expectGndEffectTouchdown = false;
    }

    return expectGndEffectTouchdown;
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2_core::setTouchdownExpected(bool val)
{
    touchdownExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTouchdown = val;
}

// Set to true if the terrain underneath is stable enough to be used as a height reference
// in combination with a range finder. Set to false if the terrain underneath the vehicle
// cannot be used as a height reference
void NavEKF2_core::setTerrainHgtStable(bool val)
{
    terrainHgtStableSet_ms = imuSampleTime_ms;
    terrainHgtStable = val;
}

// Detect takeoff for optical flow navigation
void NavEKF2_core::detectOptFlowTakeoff(void)
{
    if (!onGround && !takeOffDetected && (imuSampleTime_ms - timeAtArming_ms) > 1000) {
        // we are no longer confidently on the ground so check the range finder and gyro for signs of takeoff
        const AP_InertialSensor &ins = AP::ins();
        Vector3f angRateVec;
        Vector3f gyroBias;
        getGyroBias(gyroBias);
        bool dual_ins = ins.use_gyro(0) && ins.use_gyro(1);
        if (dual_ins) {
            angRateVec = (ins.get_gyro(0) + ins.get_gyro(1)) * 0.5f - gyroBias;
        } else {
            angRateVec = ins.get_gyro() - gyroBias;
        }

        takeOffDetected = (takeOffDetected || (angRateVec.length() > 0.1f) || (rangeDataNew.rng > (rngAtStartOfFlight + 0.1f)));
    } else if (onGround) {
        // we are confidently on the ground so set the takeoff detected status to false
        takeOffDetected = false;
    }
}

