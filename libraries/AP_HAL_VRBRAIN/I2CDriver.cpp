#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "I2CDriver.h"
#include <drivers/device/i2c.h>
#include <arch/board/board.h>
#include "board_config.h"

extern const AP_HAL::HAL& hal;

using namespace VRBRAIN;

/*
  wrapper class for I2C to expose protected functions from VRXFirmware
 */
class VRBRAIN::VRBRAIN_I2C : public device::I2C {
public:
    VRBRAIN_I2C(uint8_t bus);
    bool do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len);
};

// constructor
VRBRAIN_I2C::VRBRAIN_I2C(uint8_t bus) :
    I2C("AP_I2C", "/dev/api2c", bus, 0, 400000UL)
{
    /*
      attach to the bus. Return value can be ignored as we have no probe function
    */
    init();
}

/*
  main transfer function
 */
bool VRBRAIN_I2C::do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{
    set_address(address);
    return transfer(send, send_len, recv, recv_len) == OK;
}

// constructor for main i2c class
VRBRAINI2CDriver::VRBRAINI2CDriver(void)
{
    vrbrain_i2c = new VRBRAIN_I2C(PX4_I2C_BUS_EXPANSION);
    if (vrbrain_i2c == nullptr) {
        AP_HAL::panic("Unable to allocate VRBRAIN I2C driver");
    }
}

/*
  expose transfer function
 */
bool VRBRAINI2CDriver::do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{
    return vrbrain_i2c->do_transfer(address, send, send_len, recv, recv_len);
}

/* write: for i2c devices which do not obey register conventions */
uint8_t VRBRAINI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
    return do_transfer(addr, data, len, nullptr, 0) ? 0:1;
}

/* writeRegister: write a single 8-bit value to a register */
uint8_t VRBRAINI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t d[2] = { reg, val };
    return do_transfer(addr, d, sizeof(d), nullptr, 0) ? 0:1;
}

/* writeRegisters: write bytes to contigious registers */
uint8_t VRBRAINI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                                     uint8_t len, uint8_t* data)
{
    return write(addr, 1, &reg) == 0 && write(addr, len, data) == 0;
}

/* read: for i2c devices which do not obey register conventions */
uint8_t VRBRAINI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    return do_transfer(addr, nullptr, 0, data, len) ? 0:1;
}
    
/* readRegister: read from a device register - writes the register,
 * then reads back an 8-bit value. */
uint8_t VRBRAINI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    return do_transfer(addr, &reg, 1, data, 1) ? 0:1;
}

/* readRegister: read contigious device registers - writes the first 
 * register, then reads back multiple bytes */
uint8_t VRBRAINI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                    uint8_t len, uint8_t* data)
{
    return do_transfer(addr, &reg, 1, data, len) ? 0:1;
}

#endif // CONFIG_HAL_BOARD
