#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class BatteryWarn : public ModuleBase<BatteryWarn>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    BatteryWarn();
    ~BatteryWarn() override;

    static int task_spawn(int argc, char *argv[]);

    static int custom_command(int argc, char *argv[]);

    static int print_usage(const char *reason = nullptr);

private:
    void warn_low_battery();
    void Run() override;

    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BAT_WARN_THR>) _param_threshold
    )
    orb_advert_t _mavlink_log_pub {nullptr};

    float _last_warn_time{0};
    bool _warning_active{false};
};
