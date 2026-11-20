// CC2650 SensorTag - bare-metal HDC1000 temperature & humidity reader
// Sensors: HDC1000 at I2C addr 0x43, SDA=DIO5, SCL=DIO6, LED1=DIO10, LED2=DIO15
// Results published to fixed RAM location 0x20004000 (read via JTAG)

#include <stdint.h>
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "gpio.h"
#include "ioc.h"
#include "prcm.h"
#include "i2c.h"

#define HDC1000_ADDR        0x43
#define HDC1000_REG_TEMP    0x00
#define HDC1000_REG_CONFIG  0x02

typedef struct {
    uint32_t magic;      // 0xCAFEF00D once first measurement done
    uint32_t count;      // completed measurements
    int32_t  temp_raw;   // HDC1000 temperature register
    int32_t  hum_raw;    // HDC1000 humidity register
    int32_t  temp_milli; // milli-degrees C
    int32_t  hum_milli;  // milli-%RH
    uint32_t i2c_ok;     // last I2C transaction succeeded
} results_t;

__attribute__((section(".scratch"), used, aligned(4)))
volatile results_t results;

static void delay_ms(uint32_t ms)
{
    // ~48MHz core, 6 cycles/iter -> ~2.5M iters/sec
    volatile uint32_t n = ms * 20000;
    while (n--) { __asm volatile("nop"); }
}

static void led_init(void)
{
    // GPIO is in the PERIPH domain: power + clock it first or the bus faults
    PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON) { }
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    while (!PRCMLoadGet()) { }

    IOCPinTypeGpioOutput(IOID_10);   // LED1 (red)
    IOCPinTypeGpioOutput(IOID_15);   // LED2 (green)
    HWREG(GPIO_BASE + GPIO_O_DOUT31_0) &= ~GPIO_DIO_10_MASK;
    HWREG(GPIO_BASE + GPIO_O_DOUT31_0) &= ~GPIO_DIO_15_MASK;
}

static void led_toggle(void)
{
    static uint32_t on = 0;
    on ^= 1;
    if (on) {
        HWREG(GPIO_BASE + GPIO_O_DOUT31_0) |= GPIO_DIO_10_MASK | GPIO_DIO_15_MASK;
    } else {
        HWREG(GPIO_BASE + GPIO_O_DOUT31_0) &= ~(GPIO_DIO_10_MASK | GPIO_DIO_15_MASK);
    }
}

static void i2c_init(void)
{
    // Serial power domain on, I2C0 clocked
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL)
           != PRCM_DOMAIN_POWER_ON) { }
    PRCMPeripheralRunEnable(PRCM_PERIPH_I2C0);
    PRCMLoadSet();
    while (!PRCMLoadGet()) { }

    // DIO5 = SDA, DIO6 = SCL (open-drain, pull-up, I2C mux)
    IOCPinTypeI2c(I2C0_BASE, IOID_5, IOID_6);

    // I2C from 48MHz SCLK_HF (default RCOSC), standard mode
    I2CMasterInitExpClk(I2C0_BASE, 48000000, false);
}

static int hdc_read(uint16_t *temp, uint16_t *hum)
{
    uint32_t err;

    // Point at temperature register
    I2CMasterSlaveAddrSet(I2C0_BASE, HDC1000_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, HDC1000_REG_TEMP);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE)) { }
    err = I2CMasterErr(I2C0_BASE);
    if (err != I2C_MASTER_ERR_NONE) return -1;

    // Measurement takes ~15 ms at default resolution
    delay_ms(20);

    // Read back temp MSB, temp LSB, hum MSB, hum LSB
    I2CMasterSlaveAddrSet(I2C0_BASE, HDC1000_ADDR, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (I2CMasterBusy(I2C0_BASE)) { }
    err = I2CMasterErr(I2C0_BASE);
    if (err != I2C_MASTER_ERR_NONE) return -1;
    uint8_t t1 = I2CMasterDataGet(I2C0_BASE);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while (I2CMasterBusy(I2C0_BASE)) { }
    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) return -1;
    uint8_t t2 = I2CMasterDataGet(I2C0_BASE);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while (I2CMasterBusy(I2C0_BASE)) { }
    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) return -1;
    uint8_t h1 = I2CMasterDataGet(I2C0_BASE);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(I2C0_BASE)) { }
    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) return -1;
    uint8_t h2 = I2CMasterDataGet(I2C0_BASE);

    *temp = ((uint16_t)t1 << 8) | t2;
    *hum  = ((uint16_t)h1 << 8) | h2;
    return 0;
}

int main(void)
{
    uint16_t t_raw = 0, h_raw = 0;

    led_init();
    i2c_init();

    results.i2c_ok = 0;

    while (1) {
        if (hdc_read(&t_raw, &h_raw) == 0) {
            results.temp_raw = t_raw;
            results.hum_raw  = h_raw;
            // T[C] = raw/65536*165 - 40 ;  H[%] = raw/65536*100
            results.temp_milli = ((int64_t)t_raw * 165000) / 65536 - 40000;
            results.hum_milli  = ((int64_t)h_raw * 100000) / 65536;
            results.i2c_ok = 1;
            results.count++;
            results.magic = 0xCAFEF00Du;
        }
        led_toggle();
        delay_ms(1000);
    }
}
