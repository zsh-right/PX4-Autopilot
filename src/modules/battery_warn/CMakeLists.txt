px4_add_module(
    MODULE modules__battery_warn
    MAIN battery_warn
    STACK_MAX 1200
    COMPILE_FLAGS
    SRCS
        battery_warn.cpp
    DEPENDS
    	controllib
    	geo
    	SlewRate
    	uORB
    	systemlib
    	px4_work_queue
)
