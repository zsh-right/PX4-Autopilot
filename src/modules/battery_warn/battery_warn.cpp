#include "battery_warn.h"
#include <px4_platform_common/events.h>
#include <px4_platform_common/time.h>

using namespace time_literals;

BatteryWarn::BatteryWarn() :
    ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

BatteryWarn::~BatteryWarn()
{
}

int BatteryWarn::task_spawn(int argc, char *argv[])
{
    BatteryWarn *instance = new BatteryWarn();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
        instance->ScheduleOnInterval(1_s);
        return PX4_OK;
    }
    PX4_ERR("Allocation failed");
    return PX4_ERROR;
}

int BatteryWarn::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int BatteryWarn::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR( ### 电池预警模块
                    监控电池状态并在电量低于阈值时发出警告。

                    主要功能：
                    - 实时监控电池状态
                    - 电量过低时发出多重警告
                    - 通过参数自定义预警阈值

                    使用示例：
                    # 启动模块
                    battery_warn start

                    # 查看帮助
                    battery_warn help

                    # 设置阈值到30%
                    param set BAT_WARN_THR 0.3
                    )DESCR_STR");

    PRINT_MODULE_USAGE_NAME("battery_warn", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("status");
    return 0;
}

void BatteryWarn::Run()
{
    updateParams();

    vehicle_status_s vehicle_status;
    _vehicle_status_sub.copy(&vehicle_status);

    if (vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
        _warning_active = false;
        return;
    }

    battery_status_s battery_status;
    if (!_battery_status_sub.update(&battery_status)) {
        return;
    }

    if (battery_status.warning == battery_status_s::WARNING_CRITICAL ||
        battery_status.remaining < _param_threshold.get()) {
        warn_low_battery();
    } else {
        _warning_active = false;
    }
}

void BatteryWarn::warn_low_battery()
{
    const hrt_abstime now = hrt_absolute_time();
    if (_warning_active && (now - _last_warn_time) < 30_s)
        return;

    battery_status_s battery_status;
    _battery_status_sub.copy(&battery_status);

    PX4_ERR("LOW BATTERY WARNING! Remaining: %.1f%%", (double)(battery_status.remaining * 100));
    events::send(events::ID("battery_warn_low"),
    		 events::Log::Critical,
                 "Low Battery");
    mavlink_log_critical(&_mavlink_log_pub, "LOW BATTERY: %.1f%% remaining\t",
                        (double)(battery_status.remaining * 100));

    _last_warn_time = now;
    _warning_active = true;
}

extern "C" __EXPORT int battery_warn_main(int argc, char *argv[])
{
    return BatteryWarn::main(argc, argv);
}
