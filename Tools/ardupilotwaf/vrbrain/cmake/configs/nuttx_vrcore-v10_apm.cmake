include(configs/nuttx_vrbrain-common_apm)

list(APPEND config_module_list
    drivers/boards/vrcore-v10
    drivers/pwm_input
    lib/rc
)
