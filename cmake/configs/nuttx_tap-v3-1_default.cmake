include(nuttx/px4_impl_nuttx)
px4_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT tap_common)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/led
	drivers/px4fmu
	drivers/boards
	drivers/rgbled_pwm
	drivers/tap_esc
	drivers/mpu6000
	drivers/ms5611
	drivers/mpc2520
	drivers/hmc5883
	drivers/ist8310
	drivers/hc_sr04
	drivers/gps
	drivers/airspeed
	drivers/realsense
	modules/sensors
	drivers/vmount
	drivers/camera_trigger
	drivers/gimbal_protocol_splitter
	drivers/mavlink_dup

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/hardfault_log
	systemcmds/motor_test
	systemcmds/reboot
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
	systemcmds/mtd
	systemcmds/dumpfile
	systemcmds/ver
	systemcmds/sd_bench
	systemcmds/topic_listener
	systemcmds/tune_control

	#
	# General system control
	#
	modules/commander
	modules/events
	modules/load_mon
	modules/navigator
	modules/mavlink
	modules/land_detector

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/fw_pos_control_l1
	#modules/fw_att_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Logging
	#
	modules/logger
	modules/sdlog2

	#
	# Library modules
	#
	modules/systemlib/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/launchdetection
	lib/led
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/version
	lib/DriverFramework/framework
	lib/rc
	lib/tunes
	platforms/nuttx

	# had to add for cmake, not sure why wasn't in original config
	platforms/common
	platforms/nuttx/px4_layer
)
