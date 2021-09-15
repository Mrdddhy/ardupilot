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

/*
 *  main loop scheduler for APM
 *  Author: Andrew Tridgell, January 2013
 *
 */
#include "AP_Scheduler.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InternalError/AP_InternalError.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif
#include <stdio.h>

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define SCHEDULER_DEFAULT_LOOP_RATE 400
#else
#define SCHEDULER_DEFAULT_LOOP_RATE  50
#endif

#define debug(level, fmt, args...)   do { if ((level) <= _debug.get()) { hal.console->printf(fmt, ##args); }} while (0)

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Scheduler::var_info[] = {
    // @Param: DEBUG
    // @DisplayName: Scheduler debug level
    // @Description: Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.
    // @Values: 0:Disabled,2:ShowSlips,3:ShowOverruns
    // @User: Advanced
    AP_GROUPINFO("DEBUG",    0, AP_Scheduler, _debug, 0),

    // @Param: LOOP_RATE
    // @DisplayName: Scheduling main loop rate
    // @Description: This controls the rate of the main control loop in Hz. This should only be changed by developers. This only takes effect on restart. Values over 400 are considered highly experimental.
    // @Values: 50:50Hz,100:100Hz,200:200Hz,250:250Hz,300:300Hz,400:400Hz
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("LOOP_RATE",  1, AP_Scheduler, _loop_rate_hz, SCHEDULER_DEFAULT_LOOP_RATE),

    AP_GROUPEND
};

// constructor
AP_Scheduler::AP_Scheduler(scheduler_fastloop_fn_t fastloop_fn) :
    _fastloop_fn(fastloop_fn)
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many schedulers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    // only allow 50 to 2000 Hz
    if (_loop_rate_hz < 50) {
        _loop_rate_hz.set(50);
    } else if (_loop_rate_hz > 2000) {
        _loop_rate_hz.set(2000);
    }
    _last_loop_time_s = 1.0 / _loop_rate_hz;
}

/*
 * Get the AP_Scheduler singleton
 */
AP_Scheduler *AP_Scheduler::_singleton;
AP_Scheduler *AP_Scheduler::get_singleton()
{
    return _singleton;
}

// initialise the scheduler
/*函数功能：任务列表调度初始化---主要是设置初始一些滴答计时器
  参数1：全局任务列表数组首地址
  参数2：全局任务列表数组大小
  参数3：日志运行标志位(1<<3)
*/
void AP_Scheduler::init(const AP_Scheduler::Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit)
{
    _tasks = tasks;/*任务指针*/
    _num_tasks = num_tasks;/*总共的任务数*/

    /*数组，用于记录每个任务最后运行时记录的tickcount数*/
    _last_run = new uint16_t[_num_tasks];

    /*memset函数是内存赋值函数，用来给某一块内存空间进行赋值的,包含在<string.h>
     函数原型：void* memset(void *s,int v,size_t n)
    */
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);/*初始赋值均为0*/
    _tick_counter = 0;/*滴答计数器初始设置为0*/

    // setup initial performance counters
    /*设置初始性能计数器，干什么用的？*/
    perf_info.set_loop_rate(get_loop_rate_hz());
    perf_info.reset();

    _log_performance_bit = log_performance_bit;/*log执行位，1<<3*/
}

// one tick has passed
void AP_Scheduler::tick(void)
{
    _tick_counter++;/*滴答计数器加1*/
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
/*
  fill stack with NaN so we can catch use of uninitialised stack
  variables in SITL
 */
static void fill_nanf_stack(void)
{
    float v[1024];
    fill_nanf(v, ARRAY_SIZE(v));
}
#endif

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
  函数功能：这里在特定的时间里将会运行尽可能多的调度任务
 */
void AP_Scheduler::run(uint32_t time_available)
{
    uint32_t run_started_usec = AP_HAL::micros();/*记录开始时间是多少us*/
    uint32_t now = run_started_usec;/*把数据传递给now*/

   /*执行调试代码*/
    if (_debug > 1 && _perf_counters == nullptr) {
        _perf_counters = new AP_HAL::Util::perf_counter_t[_num_tasks];
        if (_perf_counters != nullptr) {
            for (uint8_t i=0; i<_num_tasks; i++) {
                _perf_counters[i] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, _tasks[i].name);
            }
        }
    }
    /*循环执行任务*/
    for (uint8_t i=0; i<_num_tasks; i++) {
        /*dt:执行一个loop,记录的任务列表中每个任务被执行的次数,
             因此dt表示上次运行的那个任务到现在这里的圈数
        */
        uint32_t dt = _tick_counter - _last_run[i];

        /*interval_ticks：这个值表示执行某个任务所需要的圈数
          假如该任务是20Hz，那就应该是运行了20圈(400/20 = 20)后执行该任务
        */
        uint32_t interval_ticks = _loop_rate_hz / _tasks[i].rate_hz;/*间隔时间，主循环频率(400)/任务频率，一定要大于1*/

        /*任务最大也只能400Hz,也就是一圈*/
        if (interval_ticks < 1) {
            interval_ticks = 1;  /*小于1被限制等于1*/
        }
        /*如果dt<interval_ticks将会结束继续允许该任务(所需圈数不够)，跳到下一个循环任务*/
        if (dt < interval_ticks) {
            // this task is not yet scheduled to run again 
            continue;
        }
        // this task is due to run. Do we have enough time to run it?
        /* 这里也就是说如果某一次任务时不满足上述条件，则此任务将运行，
           但是要先计算我们有足够的时间跑吗*/
        _task_time_allowed = _tasks[i].max_time_micros;/*获取该任务最大允许的运行时间*/
        
        /*圈数(时间)不应该超过这个值*/
        if (dt >= interval_ticks*2) {
            // we've slipped a whole run of this task!
            // 我们把这项任务搞砸了
            debug(2, "Scheduler slip task[%u-%s] (%u/%u/%u)\n",
                  (unsigned)i,
                  _tasks[i].name,
                  (unsigned)dt,
                  (unsigned)interval_ticks,
                  (unsigned)_task_time_allowed);
        }
        
        if (dt >= interval_ticks*max_task_slowdown) {
            // we are going beyond the maximum slowdown factor for a
            // task. This will trigger increasing the time budget
            /*我们超出了任务最大减速系数，这将增加时间预算*/
            task_not_achieved++;/*不能获取的任务加1*/
        }

        if (_task_time_allowed > time_available) {
            // not enough time to run this task.  Continue loop -
            // maybe another task will fit into time remaining
            // 没有足够的时间运行此任务，继续循环
            // 也许剩下的时间里会有另一项任务
            continue;
        }

        // run it
        //开始运行它
        _task_time_started = now;/*记录满足条件的任务开始运行的时间*/
        hal.util->persistent_data.scheduler_task = i;/*当前运行的是哪个任务*/
        if (_debug > 1 && _perf_counters && _perf_counters[i]) {
            hal.util->perf_begin(_perf_counters[i]); /*调试信息*/
        }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        fill_nanf_stack();
#endif
        _tasks[i].function();/*--------运行该任务函数------*/

        if (_debug > 1 && _perf_counters && _perf_counters[i]) {
            hal.util->perf_end(_perf_counters[i]);
        }
        hal.util->persistent_data.scheduler_task = -1; /*当前任务赋值-1*/

        // record the tick counter when we ran. This drives
        // when we next run the event
        // 我们运行的时候记录下滴答计数器，这会驱使我们下次运行任务事件
        _last_run[i] = _tick_counter;

        // work out how long the event actually took
        // 算出这件事实际花了多长时间
        now = AP_HAL::micros();/*获取当前时间*/
        uint32_t time_taken = now - _task_time_started;/*计算运行任务的两次时间消耗*/
         
         /*判断是否超时，这里的超时指的是超过其任务最长运行时间*/
        if (time_taken > _task_time_allowed) {
            // the event overran!
            /*任务事件超出了*/
            debug(3, "Scheduler overrun task[%u-%s] (%u/%u)\n",
                  (unsigned)i,
                  _tasks[i].name,
                  (unsigned)time_taken,
                  (unsigned)_task_time_allowed);
        }
        /*如果超过可用时间，记录现在可以使用的时间是0，直接跳出循环*/
        if (time_taken >= time_available) {
            time_available = 0;
            break;
        }
        /*否则就把有效时间继续减少*/
        time_available -= time_taken;
    }

    // update number of spare microseconds
    // 更新备用微妙数
    _spare_micros += time_available;

    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec(void)
{
    uint32_t dt = AP_HAL::micros() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
float AP_Scheduler::load_average()
{
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    const uint32_t loop_us = get_loop_period_us();
    const uint32_t used_time = loop_us - (_spare_micros/_spare_ticks);
    return used_time / (float)loop_us;
}

/*函数功能：*/
void AP_Scheduler::loop()
{
    // 等待INS采样频率---wait for an INS sample
    hal.util->persistent_data.scheduler_task = -3;
    AP::ins().wait_for_sample();/*获取采样时间，设置delta_time和下一采样时刻*/
    hal.util->persistent_data.scheduler_task = -1;

    const uint32_t sample_time_us = AP_HAL::micros();/*获取当前时间*/
    /*一开始先判断进入这个*/
    if (_loop_timer_start_us == 0) {
        _loop_timer_start_us = sample_time_us;/*loop定时器开始的时间：us，从当前时间开始*/
        _last_loop_time_s = get_loop_period_s();/*获取上一个loop周期时间:s*/
    } else {
        _last_loop_time_s = (sample_time_us - _loop_timer_start_us) * 1.0e-6;/*两次时间差，单位为s，第一次之后才会进入这个*/
    }

    //执行快速循环任务 Execute the fast loop
    // 这里怎么运行到fast_loop的呢？---------------------
    // 这里把 _fastloop_fn()和fast_loop()函数绑定到同一块地址上，就可以运行fast_loop了
    // 要理解一下怎么具体绑在一起的
    if (_fastloop_fn) {
        hal.util->persistent_data.scheduler_task = -2;
        _fastloop_fn();
        hal.util->persistent_data.scheduler_task = -1;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    {
        /*
          for testing low CPU conditions we can add an optional delay in SITL
        */
        auto *sitl = AP::sitl();
        uint32_t loop_delay_us = sitl->loop_delay.get();
        hal.scheduler->delay_microseconds(loop_delay_us);
    }
#endif

    // tell the scheduler one tick has passed
    tick();/*告诉任务调度器一个时钟节拍已经过去，滴答计数器加1*/

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    /*运行所有应运行的程序，注意，我们只需要在每次循环中调用这个函数一次，因为任务是以主循环的倍数
      来调度的。因此，如果它们没有对在调度程序的第一次调用中运行，它们也不会在以后的调用中运行，
      直到再次调用scheduler.tick()
    */
    const uint32_t loop_us = get_loop_period_us();/*主循环时间，400Hz，loop_us =1/400 = 2.5ms = 2500us*/
    uint32_t now = AP_HAL::micros();/*获取当前时间*/
    uint32_t time_available = 0;/*用来存放可用时间*/
    if (now - sample_time_us < loop_us) {
        // get remaining time available for this loop
        /*获得剩余可用的时间*/
        time_available = loop_us - (now - sample_time_us);
    }

    // add in extra loop time determined by not achieving scheduler tasks
    /*增加由未完成调度器任务所决定的任务循环时间*/
    time_available += extra_loop_us;

    // run the tasks
    /*------在可用时间内运行任务列表相关任务------*/
    run(time_available);/*注：非常重要的一个函数*/

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // move result of AP_HAL::micros() forward:
    hal.scheduler->delay_microseconds(1);
#endif

   /*这里是处理没有获得的任务，是因为在run(time_available)里面可能存在超时的可能*/
    if (task_not_achieved > 0) {
        // add some extra time to the budget
        extra_loop_us = MIN(extra_loop_us+100U, 5000U);
        task_not_achieved = 0;
        task_all_achieved = 0;
    } else if (extra_loop_us > 0) {
        task_all_achieved++;
        if (task_all_achieved > 50) {
            // we have gone through 50 loops without a task taking too
            // long. CPU pressure has eased, so drop the extra time we're
            // giving each loop
            task_all_achieved = 0;
            // we are achieving all tasks, slowly lower the extra loop time
            extra_loop_us = MAX(0U, extra_loop_us-50U);
        }
    }

    // check loop time
    /*检查一下loop_time*/
    perf_info.check_loop_time(sample_time_us - _loop_timer_start_us);
    
    /*赋值时间给loop 定时器的起始时间*/
    _loop_timer_start_us = sample_time_us;
}

void AP_Scheduler::update_logging()
{
    if (debug_flags()) {
        perf_info.update_logging();
    }
    if (_log_performance_bit != (uint32_t)-1 &&
        AP::logger().should_log(_log_performance_bit)) {
        Log_Write_Performance();
    }
    perf_info.set_loop_rate(get_loop_rate_hz());
    perf_info.reset();
}

// Write a performance monitoring packet
void AP_Scheduler::Log_Write_Performance()
{
    const AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_us          : AP_HAL::micros64(),
        num_long_running : perf_info.get_num_long_running(),
        num_loops        : perf_info.get_num_loops(),
        max_time         : perf_info.get_max_time(),
        mem_avail        : hal.util->available_memory(),
        load             : (uint16_t)(load_average() * 1000),
        internal_errors  : AP::internalerror().errors(),
        internal_error_count : AP::internalerror().count(),
        spi_count        : pd.spi_count,
        i2c_count        : pd.i2c_count,
        i2c_isr_count    : pd.i2c_isr_count,
        extra_loop_us    : extra_loop_us,
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
}

namespace AP {

AP_Scheduler &scheduler()
{
    return *AP_Scheduler::get_singleton();
}

};
