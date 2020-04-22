#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <byteswap.h>
#include <regex.h>
#include <time.h>
#include "arisc.h"




static char *app_name = 0;
static uint32_t *_shm_vrt_addr, *_gpio_vrt_addr, *_r_gpio_vrt_addr;

#if !GPIO_SPINLOCK_SOFT || !PG_SPINLOCK_SOFT
static uint32_t *_spinlock_vrt_addr;
volatile uint32_t * _spinlock_status = 0;
#endif
#if !GPIO_SPINLOCK_SOFT
volatile uint32_t * _gpio_spinlock = 0;
#endif
#if !PG_SPINLOCK_SOFT
volatile uint32_t * _pg_spinlock = 0;
#endif

volatile uint32_t _gpio_state[GPIO_PORTS_MAX_CNT] = {0};
volatile uint32_t * _gpio[GPIO_PORTS_MAX_CNT] = {0};
volatile uint32_t * _gpio_shm_set[GPIO_PORTS_MAX_CNT] = {0};
volatile uint32_t * _gpio_shm_clr[GPIO_PORTS_MAX_CNT] = {0};
volatile uint32_t * _gpio_shm_out[GPIO_PORTS_MAX_CNT] = {0};
volatile uint32_t * _gpio_shm_inp[GPIO_PORTS_MAX_CNT] = {0};
volatile uint32_t * _gpiod[GPIO_PORTS_MAX_CNT] = {0};

volatile uint32_t * _pgc[PG_CH_MAX_CNT][PG_PARAM_CNT] = {0};
volatile uint32_t * _pgd[PG_DATA_CNT] = {0};

#define d {0,{99,99},{99,99},{99,99},{0,0},{0,0},{0,0}}
volatile _stepgen_ch_t _sgc[STEPGEN_CH_MAX_CNT] = {d,d,d,d,d,d,d,d};
#undef d




static inline
void _gpio_spin_lock()
{
#if GPIO_SPINLOCK_SOFT
    while ( *_gpiod[GPIO_ARISC_LOCK] );
    *_gpiod[GPIO_ARM_LOCK] = 1;
#else
#if GPIO_SPINLOCK_CHECK
    // check the lock status
    while ( *_spinlock_status & GPIO_SPINLOCK_MASK );
#endif
    // try to lock
    while ( *_gpio_spinlock );
#endif
}

static inline
void _gpio_spin_unlock()
{
#if GPIO_SPINLOCK_SOFT
    *_gpiod[GPIO_ARM_LOCK] = 0;
#else
    *_gpio_spinlock = 0;
#endif
}

static inline
void _gpio_port_setup(uint32_t port)
{
    if ( *_gpiod[GPIO_USED] && port < *_gpiod[GPIO_PORTS_CNT] ) return;

    _gpio_spin_lock();

    *_gpiod[GPIO_USED] = 1;

    if ( port >= *_gpiod[GPIO_PORTS_CNT] )
    {
        uint32_t p = *_gpiod[GPIO_PORTS_CNT];
        for ( ; p <= port; p++ )
        {
            *_gpio_shm_set[p] = 0;
            *_gpio_shm_clr[p] = 0;
            *_gpio_shm_out[p] = 0;
            *_gpio_shm_inp[p] = 0;
        }

        *_gpiod[GPIO_PORTS_CNT] = port + 1;
    }

    _gpio_spin_unlock();
}

static inline
int32_t gpio_pin_setup_for_output(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
    if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    _gpio_port_setup(port);
    _gpio_spin_lock();
    *_gpio_shm_out[port] |= (1UL << pin);
    _gpio_spin_unlock();
    return 0;
}

static inline
int32_t gpio_pin_setup_for_input(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
    if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    _gpio_port_setup(port);
    _gpio_spin_lock();
    *_gpio_shm_inp[port] |= (1UL << pin);
    _gpio_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_pin_get(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
    if ( pin >= GPIO_PINS_MAX_CNT ) return 0;
    return *_gpio[port] & (1UL << pin) ? HIGH : LOW;
}

static inline
int32_t gpio_pin_set(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
    if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    _gpio_port_setup(port);
    _gpio_spin_lock();
    *_gpio_shm_set[port] |= (1UL << pin);
    _gpio_spin_unlock();
    return 0;
}

static inline
int32_t gpio_pin_clr(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
    if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    _gpio_port_setup(port);
    _gpio_spin_lock();
    *_gpio_shm_clr[port] |= (1UL << pin);
    _gpio_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_port_get(uint32_t port)
{
    if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
    return *_gpio[port];
}

static inline
int32_t gpio_port_set(uint32_t port, uint32_t mask)
{
    if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
    _gpio_port_setup(port);
    _gpio_spin_lock();
    *_gpio_shm_set[port] = mask;
    _gpio_spin_unlock();
    return 0;
}

static inline
int32_t gpio_port_clr(uint32_t port, uint32_t mask)
{
    if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
    _gpio_port_setup(port);
    _gpio_spin_lock();
    *_gpio_shm_clr[port] = mask;
    _gpio_spin_unlock();
    return 0;
}

static inline
uint32_t* gpio_all_get()
{
    uint32_t port;
    for ( port = GPIO_PORTS_MAX_CNT; port--; ) _gpio_state[port] = *_gpio[port];
    return (uint32_t*) &_gpio_state[0];
}

static inline
int32_t gpio_all_set(uint32_t* mask)
{
    uint32_t port, *msk;
    for ( port = 0, msk = mask; port < GPIO_PORTS_MAX_CNT; msk++ )
    {
        if ( *msk ) _gpio_port_setup(port);
    }
    _gpio_spin_lock();
    for ( port = 0, msk = mask; port < GPIO_PORTS_MAX_CNT; msk++ )
    {
        *_gpio_shm_set[port] = *msk;
    }
    _gpio_spin_unlock();
    return 0;
}

static inline
int32_t gpio_all_clr(uint32_t* mask)
{
    uint32_t port, *msk;
    for ( port = 0, msk = mask; port < GPIO_PORTS_MAX_CNT; msk++ )
    {
        if ( *msk ) _gpio_port_setup(port);
    }
    _gpio_spin_lock();
    for ( port = 0, msk = mask; port < GPIO_PORTS_MAX_CNT; msk++ )
    {
        *_gpio_shm_clr[port] = *msk;
    }
    _gpio_spin_unlock();
    return 0;
}

static inline
int32_t gpio_data_set(uint32_t name, uint32_t value)
{
    if ( name >= GPIO_DATA_CNT ) return -1;
    if ( name == GPIO_ARISC_LOCK ) return -2;
    if ( name == GPIO_PORTS_CNT && (value >= GPIO_PORTS_MAX_CNT) ) return -3;
    _gpio_spin_lock();
    *_gpiod[name] = value;
    _gpio_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_data_get(uint32_t name)
{
    if ( name >= GPIO_DATA_CNT ) return 0;
    _gpio_spin_lock();
    uint32_t value = *_gpiod[name];
    _gpio_spin_unlock();
    return value;
}




static inline
void _pg_spin_lock()
{
#if PG_SPINLOCK_SOFT
    while ( *_pgd[GPIO_ARISC_LOCK] );
    *_pgd[GPIO_ARM_LOCK] = 1;
#else
#if PG_SPINLOCK_CHECK
    // check the lock status
    while ( *_spinlock_status & PG_SPINLOCK_MASK );
#endif
    // try to lock
    while ( *_pg_spinlock );
#endif
}

static inline
void _pg_spin_unlock()
{
#if PG_SPINLOCK_SOFT
    *_pgd[GPIO_ARM_LOCK] = 0;
#else
    *_pg_spinlock = 0;
#endif
}

static inline
int32_t pg_data_set(uint32_t name, uint32_t value)
{
    if ( name >= PG_DATA_CNT ) return -1;
    if ( name == PG_ARISC_LOCK ) return -2;
    if ( name == PG_TIMER_FREQ ) return -3;
    if ( name == PG_CH_CNT && value >= PG_CH_MAX_CNT ) return -4;
    _pg_spin_lock();
    *_pgd[name] = value;
    _pg_spin_unlock();
    return 0;
}

static inline
uint32_t pg_data_get(uint32_t name)
{
    if ( name >= PG_DATA_CNT ) return 0;
    _pg_spin_lock();
    uint32_t value = *_pgd[name];
    _pg_spin_unlock();
    return value;
}

static inline
int32_t pg_ch_data_set(uint32_t c, uint32_t name, uint32_t value)
{
    if ( c >= PG_CH_MAX_CNT ) return -1;
    if ( name >= PG_PARAM_CNT ) return -2;
    _pg_spin_lock();
    *_pgc[c][name] = value;
    _pg_spin_unlock();
    return 0;
}

static inline
uint32_t pg_ch_data_get(uint32_t c, uint32_t name)
{
    if ( c >= PG_CH_MAX_CNT ) return 0;
    if ( name >= PG_PARAM_CNT ) return 0;
    _pg_spin_lock();
    uint32_t value = *_pgc[c][name];
    _pg_spin_unlock();
    return value;
}

static inline
void _stepgen_ch_setup(uint32_t c)
{
    if ( *_pgd[PG_USED] && _sgc[c].pg_ch[DIR] < *_pgd[PG_CH_CNT] ) return;

    _sgc[c].pg_ch[STEP] = 2*c;
    _sgc[c].pg_ch[DIR] = 2*c + 1;

    uint32_t pg_ch_cnt = 1 + _sgc[c].pg_ch[DIR];
    if ( pg_ch_cnt < *_pgd[PG_CH_CNT] ) pg_ch_cnt = *_pgd[PG_CH_CNT];

    _pg_spin_lock();
    *_pgd[PG_USED] = 1;
    *_pgd[PG_CH_CNT] = pg_ch_cnt;
    _pg_spin_unlock();
}

static inline
int32_t stepgen_pin_setup(uint32_t c, uint8_t type, uint32_t port, uint32_t pin, uint32_t invert)
{
    if ( c >= STEPGEN_CH_MAX_CNT ) return -1;
    if ( type >= 2 ) return -2;
    if ( port >= GPIO_PORTS_MAX_CNT ) return -3;
    if ( pin >= GPIO_PINS_MAX_CNT ) return -4;
    _stepgen_ch_setup(c);

    _sgc[c].inv[type] = invert ? 1UL << pin : 0;

    if ( port != _sgc[c].port[type] || pin != _sgc[c].pin[type] )
    {
        _sgc[c].port[type] = port;
        _sgc[c].pin[type] = pin;

        uint32_t msk, mskn;
        msk = (1UL << pin);
        mskn = ~msk;
        c = _sgc[c].pg_ch[type];

        _pg_spin_lock();
        *_pgc[c][PG_PORT] = port;
        *_pgc[c][PG_PIN_MSK] = msk;
        *_pgc[c][PG_PIN_MSKN] = mskn;
        *_pgc[c][PG_TASK_TOGGLES] = 0;
        _pg_spin_unlock();

        gpio_pin_setup_for_output(port, pin);
        if ( invert ) gpio_pin_set(port, pin);
        else gpio_pin_clr(port, pin);
    }

    return 0;
}

static inline
int32_t stepgen_time_setup(uint32_t c, uint32_t type, uint32_t t0, uint32_t t1)
{
    if ( c >= STEPGEN_CH_MAX_CNT ) return -1;
    if ( type >= 2 ) return -2;
    _stepgen_ch_setup(c);

    if ( t0 != _sgc[c].t0[type] || t1 != _sgc[c].t1[type] )
    {
        _sgc[c].t0[type] = t0;
        _sgc[c].t1[type] = t1;

        t0 = (uint32_t) ( (uint64_t)t0 * ((uint64_t)(*_pgd[PG_TIMER_FREQ]/1000000)) / (uint64_t)1000 );
        t1 = (uint32_t) ( (uint64_t)t1 * ((uint64_t)(*_pgd[PG_TIMER_FREQ]/1000000)) / (uint64_t)1000 );
        c = _sgc[c].pg_ch[type];

        _pg_spin_lock();
        *_pgc[c][PG_TASK_T0] = t0;
        *_pgc[c][PG_TASK_T1] = t1;
        _pg_spin_unlock();
    }

    return 0;
}

static inline
int32_t stepgen_task_add(uint8_t c, int32_t pulses)
{
    if ( c >= STEPGEN_CH_MAX_CNT ) return -1;
    if ( !pulses ) return -2;
    _stepgen_ch_setup(c);

    uint32_t s = _sgc[c].pg_ch[STEP];
    uint32_t d = _sgc[c].pg_ch[DIR];
    uint32_t step_timeout = *_pgc[d][PG_TASK_T0] + *_pgc[d][PG_TASK_T1];
    uint32_t dir;
    uint32_t dir_new = (pulses > 0) ? 0 : (1UL << _sgc[c].pin[DIR]);
    uint32_t t_old;
    uint32_t t_new = 2 * ((uint32_t)abs(pulses));

#if DEBUG
    struct timespec t1, t2;
    clock_gettime(CLOCK_MONOTONIC, &t1);
#endif
    _pg_spin_lock();

    // change DIR?
    dir = _sgc[c].inv[DIR] ^ GPIO_PIN_GET(*_pgc[d][PG_PORT], *_pgc[d][PG_PIN_MSK]);
    if ( dir != dir_new )
    {
        *_pgc[d][PG_TASK_TICK] = *_pgd[PG_TIMER_TICK];
        *_pgc[d][PG_TASK_TOGGLES] = 1;
        *_pgc[d][PG_TASK_TIMEOUT] = *_pgc[d][PG_TASK_T0];
        *_pgc[s][PG_TASK_TIMEOUT] = step_timeout;
    }
    else *_pgc[s][PG_TASK_TIMEOUT] = 0;

    // setup pulses to do
    t_old = *_pgc[s][PG_TASK_TOGGLES];
    *_pgc[s][PG_TASK_TICK] = *_pgd[PG_TIMER_TICK];
    *_pgc[s][PG_TASK_TOGGLES] = t_new;

    // complete a STEP
    if ( t_old % 2 )
    {
        if ( GPIO_PIN_GET(*_pgc[s][PG_PORT], *_pgc[s][PG_PIN_MSK]) )
            GPIO_PIN_CLR(*_pgc[s][PG_PORT], *_pgc[s][PG_PIN_MSKN]);
        else
            GPIO_PIN_SET(*_pgc[s][PG_PORT], *_pgc[s][PG_PIN_MSK]);
    }

    _pg_spin_unlock();
#if DEBUG
    clock_gettime(CLOCK_MONOTONIC, &t2);
    printf("stepgen_task_add: lock time = %ld nsec\n", t2.tv_nsec - t1.tv_nsec);
#endif

    _sgc[c].pos += pulses;

    // real position adjust
    if ( t_old )
    {
        if ( t_old % 2 ) t_old--;
        if ( t_old ) _sgc[c].pos += (dir ? 1 : -1) * (t_old/2);
    }

    return 0;
}

static inline
int32_t stepgen_pos_get(uint32_t c)
{
    if ( c >= STEPGEN_CH_MAX_CNT ) return 0;
    return _sgc[c].pos;
}

static inline
int32_t stepgen_pos_set(uint32_t c, int32_t pos)
{
    if ( c >= STEPGEN_CH_MAX_CNT ) return -1;
    _sgc[c].pos = pos;
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
    addr = ARISC_SHM_BASE & ~(4096 - 1);
    off = ARISC_SHM_BASE & (4096 - 1);
    _shm_vrt_addr = mmap(NULL, ARISC_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_shm_vrt_addr == MAP_FAILED) { printf("ERROR: shm mmap() failed\n"); return; }
    for ( port = 0; port < GPIO_PORTS_MAX_CNT; port++ )
    {
        _gpio_shm_set[port] = _shm_vrt_addr + (off + port*4 + (GPIO_SHM_SET_BASE - GPIO_SHM_BASE))/4;
        _gpio_shm_clr[port] = _shm_vrt_addr + (off + port*4 + (GPIO_SHM_CLR_BASE - GPIO_SHM_BASE))/4;
        _gpio_shm_out[port] = _shm_vrt_addr + (off + port*4 + (GPIO_SHM_OUT_BASE - GPIO_SHM_BASE))/4;
        _gpio_shm_inp[port] = _shm_vrt_addr + (off + port*4 + (GPIO_SHM_INP_BASE - GPIO_SHM_BASE))/4;
    }
    p = _shm_vrt_addr + (off + (GPIO_SHM_DATA_BASE - GPIO_SHM_BASE))/4;
    for ( name = 0; name < GPIO_DATA_CNT; name++, p++ ) _gpiod[name] = p;
    p = _shm_vrt_addr + (off + GPIO_SHM_SIZE)/4;
    for ( ch = 0; ch < PG_CH_MAX_CNT; ch++ )
        for ( name = 0; name < PG_PARAM_CNT; name++, p++ ) _pgc[ch][name] = p;
    for ( name = 0; name < PG_DATA_CNT; name++, p++ ) _pgd[name] = p;

    // mmap gpio
    addr = GPIO_BASE & ~(4096 - 1);
    off = GPIO_BASE & (4096 - 1);
    _gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_gpio_vrt_addr == MAP_FAILED) { printf("ERROR: gpio mmap() failed\n"); return; }
    for ( port = 0; port < (GPIO_PORTS_MAX_CNT - 1); port++ )
    {
        _gpio[port] = (uint32_t *) ( _gpio_vrt_addr + (off + port*GPIO_BANK_SIZE + 16)/4 );
    }

    // mmap r_gpio (PL)
    addr = GPIO_R_BASE & ~(4096 - 1);
    off = GPIO_R_BASE & (4096 - 1);
    _r_gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_r_gpio_vrt_addr == MAP_FAILED) { printf("ERROR: r_gpio mmap() failed\n"); return; }
    _gpio[GPIO_PORTS_MAX_CNT - 1] = (uint32_t *) ( _r_gpio_vrt_addr + (off+16)/4 );

#if !GPIO_SPINLOCK_SOFT || !PG_SPINLOCK_SOFT
    // mmap spinlock
    addr = SPINLOCK_BASE & ~(4096 - 1);
    off = SPINLOCK_BASE & (4096 - 1);
    _spinlock_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_spinlock_vrt_addr == MAP_FAILED) { printf("ERROR: r_gpio mmap() failed\n"); return; }
    _spinlock_status = (uint32_t *)
        ( _spinlock_vrt_addr + (off + (SPINLOCK_STATUS_REG - SPINLOCK_BASE))/4 );
#if !GPIO_SPINLOCK_SOFT
    _gpio_spinlock = (uint32_t *)
        ( _spinlock_vrt_addr + (off + (SPINLOCK_LOCK_REG(GPIO_SPINLOCK_ID) - SPINLOCK_BASE))/4 );
#endif
#if !PG_SPINLOCK_SOFT
    _pg_spinlock = (uint32_t *)
        ( _spinlock_vrt_addr + (off + (SPINLOCK_LOCK_REG(PG_SPINLOCK_ID) - SPINLOCK_BASE))/4 );
#endif
#endif

    // no need to keep phy memory file open after mmap
    close(mem_fd);
}

void mem_deinit(void)
{
    munmap(_shm_vrt_addr, ARISC_SHM_SIZE);
    munmap(_gpio_vrt_addr, 4096);
    munmap(_r_gpio_vrt_addr, 4096);
#if !GPIO_SPINLOCK_SOFT || !PG_SPINLOCK_SOFT
    munmap(_spinlock_vrt_addr, 4096);
#endif
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
    uint32_t arg[10] = {0};

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
    i32  gpio_pin_setup_for_output  (port, pin) \n\
    i32  gpio_pin_setup_for_input   (port, pin) \n\
    u32  gpio_pin_get               (port, pin) \n\
    i32  gpio_pin_set               (port, pin) \n\
    i32  gpio_pin_clr               (port, pin) \n\
    u32  gpio_port_get              (port) \n\
    i32  gpio_port_set              (port, mask) \n\
    i32  gpio_port_clr              (port, mask) \n\
   *u32  gpio_all_get               () \n\
    i32  gpio_all_set               (mask, mask, .., mask) \n\
    i32  gpio_all_clr               (mask, mask, .., mask) \n\
    u32  gpio_data_get              (name) \n\
    i32  gpio_data_set              (name, value) \n\
\n\
    i32  stepgen_pin_setup      (channel, type, port, pin, invert) \n\
    i32  stepgen_time_setup     (channel, type, t0, t1) \n\
    i32  stepgen_task_add       (channel, pulses) \n\
    i32  stepgen_pos_get        (channel) \n\
    i32  stepgen_pos_set        (channel, position) \n\
\n\
    u32  pg_data_get        (name) \n\
    i32  pg_data_set        (name, value) \n\
    u32  pg_ch_data_get     (channel, name) \n\
    i32  pg_ch_data_set     (channel, name, value) \n\
\n\
  Legend: \n\
\n\
    port        GPIO port (0..%u | PA/PB/PC/PD/PE/PF/PG/PL)\n\
    pin         GPIO pin (0..%u)\n\
    mask        GPIO pins mask (u32)\n\
    channel     channel ID (u32)\n\
    type        0 = STEP, 1 = DIR\n\
    invert      invert GPIO pin? (0/1)\n\
    pulses      number of pin pulses (i32)\n\
    t0          pin state setup time in nanoseconds (u32)\n\
    t1          pin state hold time in nanoseconds (u32)\n\
    position    position value in pulses (i32)\n\
    name        data name (u32)\n\
    value       data value (u32)\n\
\n\
  NOTE:\n\
    If you are using stdin/stdout mode, omit `%s` and any \" brackets\n\
\n",
            app_name, app_name, app_name,
            (GPIO_PORTS_MAX_CNT - 1),
            (GPIO_PINS_MAX_CNT - 1),
            app_name
        );
        return 0;
    }

    // --- GPIO ------

    if ( !reg_match(str, "gpio_pin_setup_for_output *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_pin_setup_for_output(arg[0], arg[1])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_setup_for_input *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_pin_setup_for_input(arg[0], arg[1])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_pin_set(arg[0], arg[1])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_clr *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_pin_clr(arg[0], arg[1])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%u\n", gpio_pin_get(arg[0], arg[1]));
        return 0;
    }
    if ( !reg_match(str, "gpio_port_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_port_set(arg[0], arg[1])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_port_clr *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_port_clr(arg[0], arg[1])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_port_get *\\("UINT"\\)", &arg[0], 1) )
    {
        uint32_t s = gpio_port_get(arg[0]);
        uint32_t b = 32;
        printf("%u, 0x%X, 0b", s, s);
        for ( ; b--; ) printf("%u", (s & (1U << b) ? 1 : 0));
        printf("\n");
        return 0;
    }
    if ( !reg_match(str, "gpio_all_set *\\("UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 8) )
    {
        uint32_t port;
        for ( port = GPIO_PORTS_MAX_CNT; port--; ) _gpio_state[port] = arg[port];
        printf("%s\n", (gpio_all_set(&_gpio_state[0])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_all_clr *\\("UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 8) )
    {
        uint32_t port;
        for ( port = GPIO_PORTS_MAX_CNT; port--; ) _gpio_state[port] = arg[port];
        printf("%s\n", (gpio_all_clr(&_gpio_state[0])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_all_get *\\(\\)", &arg[0], 0) )
    {
        uint32_t* ports = gpio_all_get();
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

    if ( !reg_match(str, "gpio_data_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%u\n", gpio_data_get(arg[0]));
        return 0;
    }
    if ( !reg_match(str, "gpio_data_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_data_set(arg[0], arg[1])) ? "ERROR" : "OK");
        return 0;
    }

    // --- STEPGEN ------

    if ( !reg_match(str, "stepgen_pin_setup *\\("UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 5) )
    {
        printf("%s\n", (stepgen_pin_setup(arg[0], arg[1], arg[2], arg[3], arg[4])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "stepgen_task_add *\\("UINT","INT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (stepgen_task_add(arg[0], arg[1])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "stepgen_time_setup *\\("UINT","UINT","UINT","UINT"\\)", &arg[0], 4) )
    {
        printf("%s\n", (stepgen_time_setup(arg[0], arg[1], arg[2], arg[3])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "stepgen_pos_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%i\n", stepgen_pos_get(arg[0]));
        return 0;
    }
    if ( !reg_match(str, "stepgen_pos_set *\\("UINT","INT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (stepgen_pos_set(arg[0], (int32_t)arg[1])) ? "ERROR" : "OK");
        return 0;
    }

    // --- PULSGEN ------

    if ( !reg_match(str, "pg_data_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%u\n", pg_data_get(arg[0]));
        return 0;
    }
    if ( !reg_match(str, "pg_data_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (pg_data_set(arg[0], arg[1])) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "pg_ch_data_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%u\n", pg_ch_data_get(arg[0], arg[1]));
        return 0;
    }
    if ( !reg_match(str, "pg_ch_data_set *\\("UINT","UINT","UINT"\\)", &arg[0], 3) )
    {
        printf("%s\n", (pg_ch_data_set(arg[0], arg[1], arg[2])) ? "ERROR" : "OK");
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
