
#ifndef __AP_HAL_LINUX_RCOUTPUT_VRLXBRAIN_H__
#define __AP_HAL_LINUX_RCOUTPUT_VRLXBRAIN_H__

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_VRLXBRAIN

#include "AP_HAL_Linux.h"

class Linux::LinuxRCOutput_VRLXBrain : public AP_HAL::RCOutput {
    public:
    LinuxRCOutput_VRLXBrain();
    ~LinuxRCOutput_VRLXBrain();
    void     init(void* machtnichts);
    void     reset_all_channels();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

private:
    void reset();

    AP_HAL::Semaphore *_i2c_sem;
    AP_HAL::DigitalSource *enable_pin;
    uint16_t _frequency;

    uint16_t *_pulses_buffer;
};

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_VRLXBRAIN

#endif // __AP_HAL_LINUX_RCOUTPUT_VRLXBRAIN_H__
