#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <regex.h>
#include <time.h>
#include "arisc.h"




static char *app_name = 0;
static uint32_t *_shm_vrt_addr, *_gpio_vrt_addr, *_r_gpio_vrt_addr;

volatile _GPIO_PORT_REG_t *_GPIO[GPIO_PORTS_MAX_CNT] = {0};
static uint32_t _gpio_buf[GPIO_PORTS_MAX_CNT] = {0};

volatile uint32_t * _pwmc[PWM_CH_MAX_CNT][PWM_CH_DATA_CNT] = {0};
volatile uint32_t * _pwmd[PWM_DATA_CNT] = {0};

volatile uint32_t * _encc[ENC_CH_MAX_CNT][ENC_CH_DATA_CNT] = {0};
volatile uint32_t * _encd[ENC_DATA_CNT] = {0};




static inline
void _pwm_spin_lock()
{
    *_pwmd[PWM_ARM_LOCK] = 1;
    if ( ! *_pwmd[PWM_CH_CNT] ) return;
    while ( *_pwmd[PWM_ARISC_LOCK] );
}

static inline
void _pwm_spin_unlock()
{
    *_pwmd[PWM_ARM_LOCK] = 0;
}

static inline
void _enc_spin_lock()
{
    *_encd[ENC_ARM_LOCK] = 1;
    if ( ! *_encd[ENC_CH_CNT] ) return;
    while ( *_encd[ENC_ARISC_LOCK] );
}

static inline
void _enc_spin_unlock()
{
    *_encd[ENC_ARM_LOCK] = 0;
}

static inline
int32_t gpio_pin_pull_set(uint32_t port, uint32_t pin, uint32_t level, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
        if ( level >= GPIO_PULL_CNT ) return -3;
    }
    uint32_t slot = pin/16, pos = pin%16*2;
    _pwm_spin_lock();
    _enc_spin_lock();
    _GPIO[port]->pull[slot] &= ~(0b11 << pos);
    _GPIO[port]->pull[slot] |= (level << pos);
    _enc_spin_unlock();
    _pwm_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_pin_pull_get(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    }
    uint32_t slot = pin/16, pos = pin%16*2;
    return (_GPIO[port]->pull[slot] >> pos) & 0b11;
}

static inline
int32_t gpio_pin_multi_drive_set(uint32_t port, uint32_t pin, uint32_t level, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
        if ( level >= GPIO_MULTI_DRIVE_LEVEL_CNT ) return -3;
    }
    uint32_t slot = pin/16, pos = pin%16*2;
    _pwm_spin_lock();
    _enc_spin_lock();
    _GPIO[port]->drive[slot] &= ~(0b11 << pos);
    _GPIO[port]->drive[slot] |= (level << pos);
    _enc_spin_unlock();
    _pwm_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_pin_multi_drive_get(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    }
    uint32_t slot = pin/16, pos = pin%16*2;
    return (_GPIO[port]->drive[slot] >> pos) & 0b11;
}

static inline
int32_t gpio_pin_func_set(uint32_t port, uint32_t pin, uint32_t func, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
        if ( func >= GPIO_FUNC_CNT ) return -3;
    }
    uint32_t slot = pin/8, pos = pin%8*4;
    _pwm_spin_lock();
    _enc_spin_lock();
    _GPIO[port]->config[slot] &= ~(0b0111 << pos);
    _GPIO[port]->config[slot] |=    (func << pos);
    _enc_spin_unlock();
    _pwm_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_pin_func_get(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    }
    uint32_t slot = pin/8, pos = pin%8*4;
    return (_GPIO[port]->config[slot] >> pos) & 0b0111;
}

static inline
uint32_t gpio_pin_get(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
        if ( pin >= GPIO_PINS_MAX_CNT ) return 0;
    }
    return _GPIO[port]->data & (1UL << pin) ? HIGH : LOW;
}

static inline
int32_t gpio_pin_set(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
        if ( pin >= GPIO_PINS_MAX_CNT ) return 0;
    }
    _pwm_spin_lock();
    _enc_spin_lock();
    _GPIO[port]->data |= (1UL << pin);
    _enc_spin_unlock();
    _pwm_spin_unlock();
    return 0;
}

static inline
int32_t gpio_pin_clr(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
        if ( pin >= GPIO_PINS_MAX_CNT ) return 0;
    }
    _pwm_spin_lock();
    _enc_spin_lock();
    _GPIO[port]->data &= ~(1UL << pin);
    _enc_spin_unlock();
    _pwm_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_port_get(uint32_t port, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
    }
    return _GPIO[port]->data;
}

static inline
int32_t gpio_port_set(uint32_t port, uint32_t mask, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
    }
    _pwm_spin_lock();
    _enc_spin_lock();
    _GPIO[port]->data |= mask;
    _enc_spin_unlock();
    _pwm_spin_unlock();
    return 0;
}

static inline
int32_t gpio_port_clr(uint32_t port, uint32_t mask, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
    }
    _pwm_spin_lock();
    _enc_spin_lock();
    _GPIO[port]->data &= ~mask;
    _enc_spin_unlock();
    _pwm_spin_unlock();
    return 0;
}

static inline
uint32_t* gpio_all_get(uint32_t safe)
{
    uint32_t port;
    for ( port = GPIO_PORTS_MAX_CNT; port--; )
        _gpio_buf[port] = _GPIO[port]->data;
    return (uint32_t*) &_gpio_buf[0];
}

static inline
int32_t gpio_all_set(uint32_t* mask, uint32_t safe)
{
    uint32_t port;
    _pwm_spin_lock();
    _enc_spin_lock();
    for ( port = GPIO_PORTS_MAX_CNT; port--; )
    {
        _GPIO[port]->data |= mask[port];
    }
    _enc_spin_unlock();
    _pwm_spin_unlock();
    return 0;
}

static inline
int32_t gpio_all_clr(uint32_t* mask, uint32_t safe)
{
    uint32_t port;
    _pwm_spin_lock();
    _enc_spin_lock();
    for ( port = GPIO_PORTS_MAX_CNT; port--; )
    {
        _GPIO[port]->data &= ~mask[port];
    }
    _enc_spin_unlock();
    _pwm_spin_unlock();
    return 0;
}




static inline
int32_t pwm_cleanup(uint32_t safe)
{
    uint32_t c, d;

    if ( safe )
    {
    }

//    _pwm_spin_lock();
    for ( d = PWM_DATA_CNT; d--; ) *_pwmd[d] = 0;
    for ( c = PWM_CH_MAX_CNT; c--; ) {
        for ( d = PWM_CH_DATA_CNT; d--; ) *_pwmc[c][d] = 0;
    }
//    _pwm_spin_unlock();

    return 0;
}

static inline
int32_t pwm_data_set(uint32_t name, uint32_t value, uint32_t safe)
{
    if ( safe )
    {
        if ( name >= PWM_DATA_CNT ) return -1;
        if ( name == PWM_CH_CNT && value >= PWM_CH_MAX_CNT ) return -2;
    }
    _pwm_spin_lock();
    *_pwmd[name] = value;
    _pwm_spin_unlock();
    return 0;
}

static inline
uint32_t pwm_data_get(uint32_t name, uint32_t safe)
{
    if ( safe )
    {
        if ( name >= PWM_DATA_CNT ) return 0;
    }
    _pwm_spin_lock();
    uint32_t value = *_pwmd[name];
    _pwm_spin_unlock();
    return value;
}

static inline
int32_t pwm_ch_data_set(uint32_t c, uint32_t name, uint32_t value, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return -1;
        if ( name >= PWM_CH_DATA_CNT ) return -2;
    }
    _pwm_spin_lock();
    *_pwmc[c][name] = (name == PWM_CH_POS) ? (int32_t)value : value;
    _pwm_spin_unlock();
    return 0;
}

static inline
uint32_t pwm_ch_data_get(uint32_t c, uint32_t name, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return 0;
        if ( name >= PWM_CH_DATA_CNT ) return 0;
    }
    _pwm_spin_lock();
    uint32_t value = *_pwmc[c][name];
    _pwm_spin_unlock();
    return value;
}

static inline
int32_t pwm_ch_pins_setup (
    uint32_t c,
    uint32_t p_port, uint32_t p_pin, uint32_t p_inv,
    uint32_t d_port, uint32_t d_pin, uint32_t d_inv,
    uint32_t safe )
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return -1;
        if ( p_port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( p_pin >= GPIO_PINS_MAX_CNT ) return -1;
        if ( d_port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( d_pin >= GPIO_PINS_MAX_CNT ) return -1;
    }

    gpio_pin_func_set(p_port, p_pin, GPIO_FUNC_OUT, safe);
    gpio_pin_pull_set(p_port, p_pin, GPIO_PULL_DISABLE, safe);
    if ( p_inv ) gpio_pin_set(p_port, p_pin, safe);
    else         gpio_pin_clr(p_port, p_pin, safe);

    gpio_pin_func_set(d_port, d_pin, GPIO_FUNC_OUT, safe);
    gpio_pin_pull_set(d_port, d_pin, GPIO_PULL_DISABLE, safe);
    if ( d_inv ) gpio_pin_set(d_port, d_pin, safe);
    else         gpio_pin_clr(d_port, d_pin, safe);

    _pwm_spin_lock();
    *_pwmc[c][PWM_CH_P_PORT] = p_port;
    *_pwmc[c][PWM_CH_P_PIN_MSK] = 1UL << p_pin;
    *_pwmc[c][PWM_CH_P_PIN_MSKN] = ~(1UL << p_pin);
    *_pwmc[c][PWM_CH_D_PORT] = d_port;
    *_pwmc[c][PWM_CH_D_PIN_MSK] = 1UL << d_pin;
    *_pwmc[c][PWM_CH_D_PIN_MSKN] = ~(1UL << d_pin);
    _pwm_spin_unlock();

    return 0;
}

static inline
int32_t pwm_ch_times_setup (
    uint32_t c,
    int32_t p_freq_mHz, int32_t p_duty_s32,
    uint32_t d_hold_ns, uint32_t d_setup_ns,
    uint32_t safe )
{
    uint32_t p_t0, p_t1, p_period, d_t0, d_t1, d_change, ch_cnt, ch;

    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return -1;
        if ( !d_hold_ns ) d_hold_ns = 50000;
        if ( !d_setup_ns ) d_setup_ns = 50000;
    }

    ch_cnt = *_pwmd[PWM_CH_CNT];

    if ( !p_freq_mHz || !p_duty_s32 )
    {
        if ( !(*_pwmc[c][PWM_CH_P_BUSY]) || *_pwmc[c][PWM_CH_P_STOP] ) return 0;
        if ( (c+1) == ch_cnt )
        {
            for ( ch = c; ch < PWM_CH_MAX_CNT && *_pwmc[ch][PWM_CH_P_BUSY]; ch-- );
            if ( ch >= PWM_CH_MAX_CNT ) ch = 0;
            ch_cnt = ch + 1;
        }
        _pwm_spin_lock();
        *_pwmc[c][PWM_CH_P_STOP] = 1;
        *_pwmd[PWM_CH_CNT] = ch_cnt;
        _pwm_spin_unlock();
        return 0;
    }

    if ( c >= ch_cnt ) ch_cnt = c + 1;

    d_change = (p_freq_mHz > 0 && (*_pwmc[c][PWM_CH_D])) ||
               (p_freq_mHz < 0 && !(*_pwmc[c][PWM_CH_D])) ? 1 : 0;
    d_change = (p_duty_s32 > 0 && (*_pwmc[c][PWM_CH_D])) ||
               (p_duty_s32 < 0 && !(*_pwmc[c][PWM_CH_D])) ?
                   (d_change ? 0 : 1) :
                   (d_change ? 1 : 0) ;

    p_duty_s32 = p_duty_s32 < 0 ? -p_duty_s32 : p_duty_s32;
    p_freq_mHz = p_freq_mHz < 0 ? -p_freq_mHz : p_freq_mHz;

    p_period = (uint32_t) ( ((uint64_t)ARISC_CPU_FREQ) * ((uint64_t)1000) / ((uint64_t)p_freq_mHz) );
    p_period = p_period < (2*ARISC_WASTED_TICKS) ? 0 : p_period - (2*ARISC_WASTED_TICKS);
    p_t1 = (uint32_t) ( ((uint64_t)p_period) * ((uint64_t)p_duty_s32) / ((uint64_t)INT32_MAX) );
    p_t0 = p_period - p_t1;

    d_t0 = ARISC_CPU_FREQ / (1000000000 / d_hold_ns);
    d_t0 = d_t0 < ARISC_WASTED_TICKS ? 0 : d_t0 - ARISC_WASTED_TICKS;
    d_t1 = ARISC_CPU_FREQ / (1000000000 / d_setup_ns);
    d_t1 = d_t1 < ARISC_WASTED_TICKS ? 0 : d_t1 - ARISC_WASTED_TICKS;

    _pwm_spin_lock();
    *_pwmc[c][PWM_CH_P_BUSY] = 1;
    *_pwmc[c][PWM_CH_P_T0] = p_t0;
    *_pwmc[c][PWM_CH_P_T1] = p_t1;
    *_pwmc[c][PWM_CH_D_T0] = d_t0;
    *_pwmc[c][PWM_CH_D_T1] = d_t1;
    *_pwmc[c][PWM_CH_D_CHANGE] = d_change;
    *_pwmd[PWM_CH_CNT] = ch_cnt;
    _pwm_spin_unlock();

    return 0;
}

static inline
int32_t pwm_ch_pos_get(uint32_t c, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return 0;
    }
    _pwm_spin_lock();
    int32_t value = (int32_t) *_pwmc[c][PWM_CH_POS];
    _pwm_spin_unlock();
    return value;
}

static inline
int32_t pwm_ch_pos_set(uint32_t c, int32_t pos, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PWM_CH_MAX_CNT ) return -1;
    }
    _pwm_spin_lock();
    *_pwmc[c][PWM_CH_POS] = (uint32_t) pos;
    _pwm_spin_unlock();
    return 0;
}

static inline
int32_t enc_cleanup(uint32_t safe)
{
    uint32_t c, d;

    if ( safe )
    {
    }

//    _enc_spin_lock();
    for ( d = ENC_DATA_CNT; d--; ) *_encd[d] = 0;
    for ( c = ENC_CH_MAX_CNT; c--; ) {
        for ( d = ENC_CH_DATA_CNT; d--; ) *_encc[c][d] = 0;
    }
//    _enc_spin_unlock();

    return 0;
}

static inline
int32_t enc_data_set(uint32_t name, uint32_t value, uint32_t safe)
{
    if ( safe )
    {
        if ( name >= ENC_DATA_CNT ) return -1;
        if ( name == ENC_CH_CNT && value >= ENC_CH_MAX_CNT ) return -2;
    }
    _enc_spin_lock();
    *_encd[name] = value;
    _enc_spin_unlock();
    return 0;
}

static inline
uint32_t enc_data_get(uint32_t name, uint32_t safe)
{
    if ( safe )
    {
        if ( name >= ENC_DATA_CNT ) return 0;
    }
    _enc_spin_lock();
    uint32_t value = *_encd[name];
    _enc_spin_unlock();
    return value;
}

static inline
int32_t enc_ch_data_set(uint32_t c, uint32_t name, uint32_t value, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= ENC_CH_MAX_CNT ) return -1;
        if ( name >= ENC_CH_DATA_CNT ) return -2;
    }
    _enc_spin_lock();
    *_encc[c][name] = (name == ENC_CH_POS) ? (int32_t)value : value;
    _enc_spin_unlock();
    return 0;
}

static inline
uint32_t enc_ch_data_get(uint32_t c, uint32_t name, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= ENC_CH_MAX_CNT ) return 0;
        if ( name >= ENC_CH_DATA_CNT ) return 0;
    }
    _enc_spin_lock();
    uint32_t value = *_encc[c][name];
    _enc_spin_unlock();
    return value;
}

static inline
int32_t enc_ch_pins_setup(
    uint32_t c,
    uint32_t a_port, uint32_t a_pin, uint32_t a_inv, uint32_t a_all,
    uint32_t b_port, uint32_t b_pin,
    uint32_t z_port, uint32_t z_pin, uint32_t z_inv, uint32_t z_all,
    uint32_t safe
) {
    if ( safe )
    {
        if ( c >= ENC_CH_MAX_CNT ) return -1;
        if ( a_port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( a_pin >= GPIO_PINS_MAX_CNT ) return -1;
        if ( b_port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( b_pin >= GPIO_PINS_MAX_CNT ) return -1;
        if ( z_port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( z_pin >= GPIO_PINS_MAX_CNT ) return -1;
    }

    gpio_pin_func_set(a_port, a_pin, GPIO_FUNC_IN, safe);
    gpio_pin_pull_set(a_port, a_pin, GPIO_PULL_DISABLE, safe);
    gpio_pin_func_set(b_port, b_pin, GPIO_FUNC_IN, safe);
    gpio_pin_pull_set(b_port, b_pin, GPIO_PULL_DISABLE, safe);
    gpio_pin_func_set(z_port, z_pin, GPIO_FUNC_IN, safe);
    gpio_pin_pull_set(z_port, z_pin, GPIO_PULL_DISABLE, safe);

    _enc_spin_lock();
    *_encc[c][ENC_CH_A_PORT] = a_port;
    *_encc[c][ENC_CH_A_PIN_MSK] = 1UL << a_pin;
    *_encc[c][ENC_CH_A_INV] = a_inv;
    *_encc[c][ENC_CH_A_ALL] = a_all;
    *_encc[c][ENC_CH_B_PORT] = b_port;
    *_encc[c][ENC_CH_B_PIN_MSK] = 1UL << b_pin;
    *_encc[c][ENC_CH_Z_PORT] = z_port;
    *_encc[c][ENC_CH_Z_PIN_MSK] = 1UL << z_pin;
    *_encc[c][ENC_CH_Z_INV] = z_inv;
    *_encc[c][ENC_CH_Z_ALL] = z_all;
    _enc_spin_unlock();

    return 0;
}

static inline
int32_t enc_ch_state_set(uint32_t c, uint32_t enable, uint32_t safe )
{
    uint32_t ch_cnt, ch;

    if ( safe )
    {
        if ( c >= ENC_CH_MAX_CNT ) return -1;
    }

    ch_cnt = *_encd[ENC_CH_CNT];

    if ( !enable )
    {
        if ( !(*_encc[c][ENC_CH_BUSY]) ) return 0;
        if ( (c+1) == ch_cnt )
        {
            for ( ch = c; ch < ENC_CH_MAX_CNT && *_encc[ch][ENC_CH_BUSY]; ch-- );
            if ( ch >= PWM_CH_MAX_CNT ) ch = 0;
            ch_cnt = ch + 1;
        }
    }
    else
    {
        if ( *_encc[c][ENC_CH_BUSY] ) return 0;
        if ( c >= ch_cnt ) ch_cnt = c + 1;
    }

    _enc_spin_lock();
    *_encc[c][ENC_CH_BUSY] = enable;
    *_encd[ENC_CH_CNT] = ch_cnt;
    _enc_spin_unlock();

    return 0;
}

static inline
int32_t enc_ch_pos_get(uint32_t c, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= ENC_CH_MAX_CNT ) return 0;
    }
    _enc_spin_lock();
    int32_t value = (int32_t) *_encc[c][ENC_CH_POS];
    _enc_spin_unlock();
    return value;
}

static inline
int32_t enc_ch_pos_set(uint32_t c, int32_t pos, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= ENC_CH_MAX_CNT ) return -1;
    }
    _enc_spin_lock();
    *_encc[c][ENC_CH_POS] = (uint32_t) pos;
    _enc_spin_unlock();
    return 0;
}




void mem_init(void)
{
    int32_t mem_fd;
    uint32_t addr, off, port, ch, name, *p;

    // open physical memory file
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if ( mem_fd  < 0 ) { printf("ERROR: can't open /dev/mem file\n"); return; }

    // mmap shmem
    addr = ARISC_SHM_BASE & ~(ARISC_SHM_SIZE - 1);
    off = ARISC_SHM_BASE & (ARISC_SHM_SIZE - 1);
    _shm_vrt_addr = mmap(NULL, ARISC_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_shm_vrt_addr == MAP_FAILED) { printf("ERROR: shm mmap() failed\n"); return; }
    p = _shm_vrt_addr + off/4;
    for ( name = 0; name < PWM_DATA_CNT; name++, p++ ) _pwmd[name] = p;
    for ( ch = 0; ch < PWM_CH_MAX_CNT; ch++ ) {
        for ( name = 0; name < PWM_CH_DATA_CNT; name++, p++ ) _pwmc[ch][name] = p;
    }
    for ( name = 0; name < ENC_DATA_CNT; name++, p++ ) _encd[name] = p;
    for ( ch = 0; ch < ENC_CH_MAX_CNT; ch++ ) {
        for ( name = 0; name < ENC_CH_DATA_CNT; name++, p++ ) _encc[ch][name] = p;
    }

    // mmap gpio
    addr = GPIO_BASE & ~(4096 - 1);
    off = GPIO_BASE & (4096 - 1);
    _gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_gpio_vrt_addr == MAP_FAILED) { printf("ERROR: gpio mmap() failed\n"); return; }
    for ( port = PA; port <= PG; ++port )
    {
        _GPIO[port] = (_GPIO_PORT_REG_t *)(_gpio_vrt_addr + (off + port*0x24)/4);
    }

    // mmap r_gpio (PL)
    addr = GPIO_R_BASE & ~(4096 - 1);
    off = GPIO_R_BASE & (4096 - 1);
    _r_gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_r_gpio_vrt_addr == MAP_FAILED) { printf("ERROR: r_gpio mmap() failed\n"); return; }
    _GPIO[PL] = (_GPIO_PORT_REG_t *)(_r_gpio_vrt_addr + off/4);

    // no need to keep phy memory file open after mmap
    close(mem_fd);
}

void mem_deinit(void)
{
    munmap(_shm_vrt_addr, ARISC_SHM_SIZE);
    munmap(_gpio_vrt_addr, 4096);
    munmap(_r_gpio_vrt_addr, 4096);
}




int32_t reg_match(const char *source, const char *pattern, uint32_t *match_array, uint32_t array_size)
{
    regex_t re;
    regmatch_t matches[10] = {{0}};
    int32_t ret = 0;

    // on regex compilation fail
    if ( regcomp(&re, pattern, REG_EXTENDED|REG_NEWLINE) )
    {
        printf("  regex compilation fail: %s \n", pattern);
        ret = 1;
    }
    // on regex match fail
    else if ( regexec(&re, source, 10, &matches[0], 0) )
    {
        ret = 2;
    }
    // on regex match success
    else
    {
        uint32_t size;
        uint32_t n;
        char match[48] = {0};
        uint32_t *arg = match_array;

        // browse all matches
        for ( n = 1; (n < array_size + 1) && (n < 10) && matches[n].rm_so != -1; arg++, n++ )
        {
            // get match string size
            size = (uint32_t)(matches[n].rm_eo - matches[n].rm_so);

            // string size is limited to buffer size
            if ( size <= 0 || size >= sizeof match )
            {
                ret = 3;
                break;
            }

            // copy match string to the tmp buffer
            memcpy(&match[0], source + matches[n].rm_so, (size_t)size);
            match[size] = 0;

            // if we have a port name
            if ( size == 2 && match[0] == 'P' && ((match[1] >= 'A' && match[1] <= 'G') || match[1] == 'L') )
            {
                *arg = (match[1] == 'L') ? (PL) : (match[1] - 'A');
            }
            // if we have a hex number
            else if ( size >= 3 && match[0] == '0' && match[1] == 'x' )
            {
                *arg = (uint32_t) strtoul((const char *)&match, NULL, 16);
            }
            // if we have a binary number
            else if ( size >= 3 && match[0] == '0' && match[1] == 'b' )
            {
                *arg = (uint32_t) strtoul((const char *)&match[2], NULL, 2);
            }
            // if we have an unsigned integer
            else if ( match[0] >= '0' && match[0] <= '9' )
            {
                *arg = (uint32_t) strtoul((const char *)&match, NULL, 10);
            }
            // if we have a signed integer
            else if ( size >= 2 && match[0] == '-' && match[1] >= '0' && match[1] <= '9')
            {
                *arg = (uint32_t) strtol((const char *)&match, NULL, 10);
            }
            // if we have an unknown string
            else
            {
                ret = 4;
                break;
            }
        }
    }

    // free regex memory block
    regfree(&re);

    return ret;
}




int32_t parse_and_exec(const char *str)
{
    uint32_t arg[12] = {0};

    #define UINT " *([0-9]+|0x[A-Fa-f]+|0b[01]+|P[ABCDEFGL]) *"
    #define INT " *(\\-?[0-9]+) *"

    // --- HELP, EXAMPLES, EXIT ------

    if ( !reg_match(str, "(exit|quit|q)", &arg[0], 0) )
    {
        return -1;
    }

    if ( !reg_match(str, "help", &arg[0], 0) )
    {
        printf(
"\n\
  Usage:\n\
\n\
    %s \"function1(param1, param2, ..)\" \"function2(param1, param2, ..)\" ...\n\
\n\
    %s help         show help info \n\
    %s exit|quit|q  program quit \n\
\n\
  Functions: \n\
\n\
    i32  gpio_pin_func_set          (port, pin, function) \n\
    u32  gpio_pin_func_get          (port, pin) \n\
    i32  gpio_pin_pull_set          (port, pin, level) \n\
    u32  gpio_pin_pull_get          (port, pin) \n\
    i32  gpio_pin_multi_drive_set   (port, pin, level) \n\
    u32  gpio_pin_multi_drive_get   (port, pin) \n\
    u32  gpio_pin_get               (port, pin) \n\
    i32  gpio_pin_set               (port, pin) \n\
    i32  gpio_pin_clr               (port, pin) \n\
    u32  gpio_port_get              (port) \n\
    i32  gpio_port_set              (port, mask) \n\
    i32  gpio_port_clr              (port, mask) \n\
   *u32  gpio_all_get               () \n\
    i32  gpio_all_set               (mask, mask, .., mask) \n\
    i32  gpio_all_clr               (mask, mask, .., mask) \n\
\n\
    i32  pwm_cleanup        () \n\
    u32  pwm_data_get       (name) \n\
    i32  pwm_data_set       (name, value) \n\
    u32  pwm_ch_data_get    (channel, name) \n\
    i32  pwm_ch_data_set    (channel, name, value) \n\
    i32  pwm_ch_pins_setup  (channel, p_port, p_pin, p_inv, d_port, d_pin, d_inv) \n\
    i32  pwm_ch_times_setup (channel, p_freq_mHz, p_duty_s32, d_hold_ns, d_setup_ns) \n\
    i32  pwm_ch_pos_get     (channel) \n\
    i32  pwm_ch_pos_set     (channel, value) \n\
\n\
    i32  enc_cleanup        () \n\
    u32  enc_data_get       (name) \n\
    i32  enc_data_set       (name, value) \n\
    u32  enc_ch_data_get    (channel, name) \n\
    i32  enc_ch_data_set    (channel, name, value) \n\
    i32  enc_ch_pins_setup  (channel, a_port,a_pin,a_inv,a_all, b_port,b_pin, z_port,z_pin,z_inv,z_all) \n\
    i32  enc_ch_state_set   (channel, enable) \n\
    i32  enc_ch_pos_get     (channel) \n\
    i32  enc_ch_pos_set     (channel, value) \n\
\n\
  NOTE:\n\
    If you are using stdin/stdout mode, omit `%s` and any \" brackets\n\
\n",
            app_name, app_name, app_name, app_name
        );
        return 0;
    }

    // --- GPIO ------

    if ( !reg_match(str, "gpio_pin_pull_set *\\("UINT","UINT","UINT"\\)", &arg[0], 3) )
    {
        printf("%s\n", (gpio_pin_pull_set(arg[0], arg[1], arg[2], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_pull_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%u\n", gpio_pin_pull_get(arg[0], arg[1], 1));
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_multi_drive_set *\\("UINT","UINT","UINT"\\)", &arg[0], 3) )
    {
        printf("%s\n", (gpio_pin_multi_drive_set(arg[0], arg[1], arg[2], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_multi_drive_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%u\n", gpio_pin_multi_drive_get(arg[0], arg[1], 1));
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_func_set *\\("UINT","UINT","UINT"\\)", &arg[0], 3) )
    {
        printf("%s\n", (gpio_pin_func_set(arg[0], arg[1], arg[2], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_func_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%u\n", gpio_pin_func_get(arg[0], arg[1], 1));
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_pin_set(arg[0], arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_clr *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_pin_clr(arg[0], arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%u\n", gpio_pin_get(arg[0], arg[1], 1));
        return 0;
    }
    if ( !reg_match(str, "gpio_port_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_port_set(arg[0], arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_port_clr *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_port_clr(arg[0], arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_port_get *\\("UINT"\\)", &arg[0], 1) )
    {
        uint32_t s = gpio_port_get(arg[0], 1);
        uint32_t b = 32;
        printf("%u, 0x%X, 0b", s, s);
        for ( ; b--; ) printf("%u", (s & (1U << b) ? 1 : 0));
        printf("\n");
        return 0;
    }
    if ( !reg_match(str, "gpio_all_set *\\("UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 8) )
    {
        uint32_t port;
        for ( port = GPIO_PORTS_MAX_CNT; port--; ) _gpio_buf[port] = arg[port];
        printf("%s\n", (gpio_all_set(&_gpio_buf[0], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_all_clr *\\("UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 8) )
    {
        uint32_t port;
        for ( port = GPIO_PORTS_MAX_CNT; port--; ) _gpio_buf[port] = arg[port];
        printf("%s\n", (gpio_all_clr(&_gpio_buf[0], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_all_get *\\(\\)", &arg[0], 0) )
    {
        uint32_t* ports = gpio_all_get(1);
        uint32_t b, port, s;
        for ( port = 0; port < GPIO_PORTS_MAX_CNT; port++ )
        {
            s = *(ports+port);
            printf("PORT %u state: 0b", port);
            for ( b = 32; b--; ) printf("%u", (s & (1U << b) ? 1 : 0));
            printf(" (%u, 0x%X)\n", s, s);
        }
        return 0;
    }

    // --- PWM ------

    if ( !reg_match(str, "pwm_cleanup *\\(\\)", &arg[0], 0) )
    {
        printf("%s\n", (pwm_cleanup(1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "pwm_data_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%u\n", pwm_data_get(arg[0], 1));
        return 0;
    }
    if ( !reg_match(str, "pwm_data_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (pwm_data_set(arg[0], arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "pwm_ch_data_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        if ( arg[1] == PWM_CH_POS ) {
            printf("%i\n", (int32_t)pwm_ch_data_get(arg[0], arg[1], 1));
        } else {
            printf("%u\n", pwm_ch_data_get(arg[0], arg[1], 1));
        }
        return 0;
    }
    if ( !reg_match(str, "pwm_ch_data_set *\\("UINT","UINT","INT"\\)", &arg[0], 3) )
    {
        printf("%s\n", (pwm_ch_data_set(arg[0], arg[1], arg[2], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "pwm_ch_pins_setup *\\("UINT","UINT","UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 7) )
    {
        printf("%s\n", (pwm_ch_pins_setup(arg[0],arg[1],arg[2],arg[3],arg[4],arg[5],arg[6],1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "pwm_ch_times_setup *\\("UINT","INT","INT","UINT","UINT"\\)", &arg[0], 5) )
    {
        printf("%s\n", (pwm_ch_times_setup(arg[0],(int32_t)arg[1],(int32_t)arg[2],arg[3],arg[4],1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "pwm_ch_pos_set *\\("UINT","INT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (pwm_ch_pos_set(arg[0], (int32_t)arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "pwm_ch_pos_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%i\n", pwm_ch_pos_get(arg[0], 1));
        return 0;
    }

    // --- encoder ------

    if ( !reg_match(str, "enc_cleanup *\\(\\)", &arg[0], 0) )
    {
        printf("%s\n", (enc_cleanup(1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "enc_data_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%u\n", enc_data_get(arg[0], 1));
        return 0;
    }
    if ( !reg_match(str, "enc_data_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (enc_data_set(arg[0], arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "enc_ch_data_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        if ( arg[1] == PWM_CH_POS ) {
            printf("%i\n", (int32_t)enc_ch_data_get(arg[0], arg[1], 1));
        } else {
            printf("%u\n", enc_ch_data_get(arg[0], arg[1], 1));
        }
        return 0;
    }
    if ( !reg_match(str, "enc_ch_data_set *\\("UINT","UINT","INT"\\)", &arg[0], 3) )
    {
        printf("%s\n", (enc_ch_data_set(arg[0], arg[1], arg[2], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "enc_ch_pins_setup *\\("UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 11) )
    {
        printf("%s\n", (enc_ch_pins_setup(arg[0],arg[1],arg[2],arg[3],arg[4],arg[5],arg[6],arg[7],arg[8],arg[9],arg[10],1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "enc_ch_state_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (enc_ch_state_set(arg[0],arg[1],1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "enc_ch_pos_set *\\("UINT","INT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (enc_ch_pos_set(arg[0], (int32_t)arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "enc_ch_pos_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%i\n", enc_ch_pos_get(arg[0], 1));
        return 0;
    }

    printf("Unknown command! Type `help` \n");

    #undef UINT
    #undef INT

    return 1;
}








int main(int argc, char *argv[])
{
    mem_init();

    app_name = argv[0];

    // start STDIN/STDOUT mode if we have no arguments
    if ( argc < 2 )
    {
        char input_str[255] = {0};

        printf("\n\
  Welcome to stdin/stdout mode of ARISC CNC CLI.\n\
\n\
  Type `help` to see help info.\n\
  Type `q`|`quit`|`exit` to quit the program.\n\
            \n");

        for(;;)
        {
            fgets((char *)&input_str[0], 254, stdin);
            if ( parse_and_exec((char *)&input_str[0]) == -1 ) break;
        }
    }
    // parse and execute every argument
    else
    {
        uint32_t a;
        for ( a = 1; a < argc; a++ ) parse_and_exec(argv[a]);
    }

    mem_deinit();

    return 0;
}
