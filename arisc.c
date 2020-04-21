#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <byteswap.h>
#include <regex.h>
#include "arisc.h"




static uint32_t *shm_vrt_addr, *gpio_vrt_addr, *r_gpio_vrt_addr;
static char *app_name = 0;

volatile uint32_t * gpio[GPIO_PORTS_CNT] = {0};
volatile uint32_t * gpio_shm_set[GPIO_PORTS_CNT] = {0};
volatile uint32_t * gpio_shm_clr[GPIO_PORTS_CNT] = {0};
volatile uint32_t * gpio_shm_out[GPIO_PORTS_CNT] = {0};
volatile uint32_t * gpio_shm_inp[GPIO_PORTS_CNT] = {0};
volatile uint32_t * gpiod[GPIO_PORTS_CNT] = {0};

volatile uint32_t * pgc[PG_CH_MAX_CNT][PG_PARAM_CNT] = {0};
volatile uint32_t * pgd[PG_DATA_CNT] = {0};

#define d {0,{99,99},{99,99},{99,99},{0,0},{0,0},{0,0}}
volatile _stepgen_ch_t _sgc[STEPGEN_CH_MAX_CNT] = {d,d,d,d,d,d,d,d};
#undef d




static inline
void _gpio_spin_lock()
{
    while ( *gpiod[GPIO_ARISC_LOCK] );
    *gpiod[GPIO_ARM_LOCK] = 1;
}

static inline
void _gpio_spin_unlock()
{
    *gpiod[GPIO_ARM_LOCK] = 0;
}

static inline
void gpio_port_setup(uint32_t port)
{
    if ( !*gpiod[GPIO_USED] || port > *gpiod[GPIO_PORT_MAX_ID] )
    {
        _gpio_spin_lock();
        *gpiod[GPIO_USED] = 1;
        *gpiod[GPIO_PORT_MAX_ID] = port;
        _gpio_spin_unlock();
    }
}

static inline
int32_t gpio_pin_setup_for_output(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return -1;
    if ( pin >= GPIO_PINS_CNT ) return -2;
    gpio_port_setup(port);
    _gpio_spin_lock();
    *gpio_shm_out[port] |= (1UL << pin);
    _gpio_spin_unlock();
    return 0;
}

static inline
int32_t gpio_pin_setup_for_input(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return -1;
    if ( pin >= GPIO_PINS_CNT ) return -2;
    gpio_port_setup(port);
    _gpio_spin_lock();
    *gpio_shm_inp[port] |= (1UL << pin);
    _gpio_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_pin_get(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return 0;
    if ( pin >= GPIO_PINS_CNT ) return 0;
    return *gpio[port] & (1UL << pin) ? HIGH : LOW;
}

static inline
int32_t gpio_pin_set(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return -1;
    if ( pin >= GPIO_PINS_CNT ) return -2;
    gpio_port_setup(port);
    _gpio_spin_lock();
    *gpio_shm_set[port] |= (1UL << pin);
    _gpio_spin_unlock();
    return 0;
}

static inline
int32_t gpio_pin_clr(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return -1;
    if ( pin >= GPIO_PINS_CNT ) return -2;
    gpio_port_setup(port);
    _gpio_spin_lock();
    *gpio_shm_clr[port] |= (1UL << pin);
    _gpio_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_port_get(uint32_t port)
{
    if ( port >= GPIO_PORTS_CNT ) return 0;
    return *gpio[port];
}

static inline
int32_t gpio_port_set(uint32_t port, uint32_t mask)
{
    if ( port >= GPIO_PORTS_CNT ) return -1;
    gpio_port_setup(port);
    _gpio_spin_lock();
    *gpio_shm_set[port] = mask;
    _gpio_spin_unlock();
    return 0;
}

static inline
int32_t gpio_port_clr(uint32_t port, uint32_t mask)
{
    if ( port >= GPIO_PORTS_CNT ) return -1;
    gpio_port_setup(port);
    _gpio_spin_lock();
    *gpio_shm_clr[port] = mask;
    _gpio_spin_unlock();
    return 0;
}

static inline
int32_t gpio_data_set(uint32_t name, uint32_t value)
{
    if ( name >= GPIO_DATA_CNT ) return -1;
    if ( name == GPIO_ARISC_LOCK ) return -2;
    if ( name == GPIO_PORT_MAX_ID && (value >= GPIO_PORTS_CNT) ) return -3;
    _gpio_spin_lock();
    if ( name == GPIO_PORT_MAX_ID || name == GPIO_USED )
    {
        uint32_t port;
        for ( port = GPIO_PORTS_CNT; port--; )
        {
            *gpio_shm_set[port] = 0;
            *gpio_shm_clr[port] = 0;
            *gpio_shm_out[port] = 0;
            *gpio_shm_inp[port] = 0;
        }
    }
    *gpiod[name] = value;
    _gpio_spin_unlock();
    return 0;
}

static inline
uint32_t gpio_data_get(uint32_t name)
{
    if ( name >= GPIO_DATA_CNT ) return 0;
    _gpio_spin_lock();
    uint32_t value = *gpiod[name];
    _gpio_spin_unlock();
    return value;
}




static inline
void _pg_spin_lock()
{
    while ( *pgd[GPIO_ARISC_LOCK] );
    *pgd[GPIO_ARM_LOCK] = 1;
}

static inline
void _pg_spin_unlock()
{
    *pgd[GPIO_ARM_LOCK] = 0;
}

static inline
int32_t pg_data_set(uint32_t name, uint32_t value)
{
    if ( name >= PG_DATA_CNT ) return -1;
    if ( name == PG_ARISC_LOCK ) return -2;
    if ( name == PG_TIMER_FREQ ) return -3;
    if ( name == PG_CH_CNT && value >= PG_CH_MAX_CNT ) return -4;
    _pg_spin_lock();
    *pgd[name] = value;
    _pg_spin_unlock();
    return 0;
}

static inline
uint32_t pg_data_get(uint32_t name)
{
    if ( name >= PG_DATA_CNT ) return 0;
    _pg_spin_lock();
    uint32_t value = *pgd[name];
    _pg_spin_unlock();
    return value;
}

static inline
int32_t pg_ch_data_set(uint32_t c, uint32_t name, uint32_t value)
{
    if ( c >= PG_CH_MAX_CNT ) return -1;
    if ( name >= PG_PARAM_CNT ) return -2;
    _pg_spin_lock();
    *pgc[c][name] = value;
    _pg_spin_unlock();
    return 0;
}

static inline
uint32_t pg_ch_data_get(uint32_t c, uint32_t name)
{
    if ( c >= PG_CH_MAX_CNT ) return 0;
    if ( name >= PG_PARAM_CNT ) return 0;
    _pg_spin_lock();
    uint32_t value = *pgc[c][name];
    _pg_spin_unlock();
    return value;
}

static inline
void _stepgen_ch_setup(uint32_t c)
{
    if ( *pgd[PG_USED] &&
         _sgc[c].pg_ch[DIR] < PG_CH_MAX_CNT &&
         _sgc[c].pg_ch[DIR] < *pgd[PG_CH_CNT] ) return;

    _sgc[c].pg_ch[STEP] = 2*c;
    _sgc[c].pg_ch[DIR] = 2*c + 1;

    uint32_t pg_ch_cnt = 1 + _sgc[c].pg_ch[DIR];
    if ( pg_ch_cnt < *pgd[PG_CH_CNT] ) pg_ch_cnt = *pgd[PG_CH_CNT];

    _pg_spin_lock();
    *pgd[PG_USED] = 1;
    *pgd[PG_CH_CNT] = _sgc[c].pg_ch[DIR] + 1;
    _pg_spin_unlock();
}

static inline
int32_t stepgen_pin_setup(uint32_t c, uint8_t type, uint32_t port, uint32_t pin, uint32_t invert)
{
    if ( c >= STEPGEN_CH_MAX_CNT ) return -1;
    if ( type >= 2 ) return -2;
    if ( port >= GPIO_PORTS_CNT ) return -3;
    if ( pin >= GPIO_PINS_CNT ) return -4;
    _stepgen_ch_setup(c);

    _sgc[c].inv[type] = invert ? 1UL << pin : 0;

    if ( port != _sgc[c].port[type] || pin != _sgc[c].pin[type] )
    {
        _sgc[c].port[type] = port;
        _sgc[c].pin[type] = pin;

        uint32_t t, msk, mskn;
        msk = (1UL << pin);
        mskn = ~msk;
        c = _sgc[c].pg_ch[type];

        _pg_spin_lock();
        *pgc[c][PG_PORT] = port;
        *pgc[c][PG_PIN_MSK] = msk;
        *pgc[c][PG_PIN_MSKN] = mskn;
        t = *pgc[c][PG_TASK_TOGGLES];
        _pg_spin_unlock();

        if ( !t )
        {
            gpio_pin_setup_for_output(port, pin);

            if ( invert ) gpio_pin_set(port, pin);
            else gpio_pin_clr(port, pin);
        }
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

        t0 = (uint32_t) ( (uint64_t)t0 * ((uint64_t)(*pgd[PG_TIMER_FREQ]/1000000)) / (uint64_t)1000 );
        t1 = (uint32_t) ( (uint64_t)t1 * ((uint64_t)(*pgd[PG_TIMER_FREQ]/1000000)) / (uint64_t)1000 );
        c = _sgc[c].pg_ch[type];

        _pg_spin_lock();
        *pgc[c][PG_TASK_T0] = t0;
        *pgc[c][PG_TASK_T1] = t1;
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
    uint32_t step_timeout = _sgc[c].t0[DIR] + _sgc[c].t1[DIR];
    uint32_t dir, dir_real;
    uint32_t dir_new = (pulses > 0) ? 0 : (1UL << _sgc[c].pin[DIR]);
    uint32_t t_old;
    uint32_t t_new = 2 * ((uint32_t)abs(pulses));

    _pg_spin_lock();

    dir_real = GPIO_PIN_GET(_sgc[c].port[DIR], *pgc[d][PG_PIN_MSK]);
    dir = _sgc[c].inv[DIR] ^ dir_real;

    if ( dir != dir_new )
    {
        *pgc[d][PG_TASK_TICK] = *pgc[c][PG_TIMER_TICK];
        *pgc[d][PG_TASK_TOGGLES] = 1;
        *pgc[d][PG_TASK_TIMEOUT] = _sgc[c].t0[DIR];
        if ( dir_real ) *pgc[d][PG_TASK_T0] = _sgc[c].t1[DIR];
        else            *pgc[d][PG_TASK_T1] = _sgc[c].t1[DIR];

        *pgc[s][PG_TASK_TIMEOUT] = step_timeout;
    }
    else
    {
        *pgc[s][PG_TASK_TIMEOUT] = 0;
    }

    t_old = *pgc[s][PG_TASK_TOGGLES];
    *pgc[s][PG_TASK_TICK] = *pgc[c][PG_TIMER_TICK];
    *pgc[s][PG_TASK_TOGGLES] = t_new;

    if ( t_old % 2 )
    {
        if ( GPIO_PIN_GET(_sgc[c].port[STEP], *pgc[s][PG_PIN_MSK]) )
            GPIO_PIN_CLR(_sgc[c].port[STEP], *pgc[s][PG_PIN_MSK]);
        else
            GPIO_PIN_SET(_sgc[c].port[STEP], *pgc[s][PG_PIN_MSK]);
    }

    _pg_spin_unlock();

    _sgc[c].pos += pulses;

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
    shm_vrt_addr = mmap(NULL, ARISC_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (shm_vrt_addr == MAP_FAILED) { printf("ERROR: shm mmap() failed\n"); return; }
    for ( port = 0; port < GPIO_PORTS_CNT; port++ )
    {
        gpio_shm_set[port] = shm_vrt_addr + (off + port*4 + (GPIO_SHM_SET_BASE - GPIO_SHM_BASE))/4;
        gpio_shm_clr[port] = shm_vrt_addr + (off + port*4 + (GPIO_SHM_CLR_BASE - GPIO_SHM_BASE))/4;
        gpio_shm_out[port] = shm_vrt_addr + (off + port*4 + (GPIO_SHM_OUT_BASE - GPIO_SHM_BASE))/4;
        gpio_shm_inp[port] = shm_vrt_addr + (off + port*4 + (GPIO_SHM_INP_BASE - GPIO_SHM_BASE))/4;
    }
    p = shm_vrt_addr + (off + (GPIO_SHM_DATA_BASE - GPIO_SHM_BASE))/4;
    for ( name = 0; name < GPIO_DATA_CNT; name++, p++ ) gpiod[name] = p;
    p = shm_vrt_addr + (off + GPIO_SHM_SIZE)/4;
    for ( ch = 0; ch < PG_CH_MAX_CNT; ch++ )
        for ( name = 0; name < PG_PARAM_CNT; name++, p++ ) pgc[ch][name] = p;
    for ( name = 0; name < PG_DATA_CNT; name++, p++ ) pgd[name] = p;

    // mmap gpio
    addr = GPIO_BASE & ~(4096 - 1);
    off = GPIO_BASE & (4096 - 1);
    gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (gpio_vrt_addr == MAP_FAILED) { printf("ERROR: gpio mmap() failed\n"); return; }
    for ( port = 0; port < (GPIO_PORTS_CNT - 1); port++ )
    {
        gpio[port] = (uint32_t *) ( gpio_vrt_addr + (off + port*GPIO_BANK_SIZE + 16)/4 );
    }

    // mmap r_gpio (PL)
    addr = GPIO_R_BASE & ~(4096 - 1);
    off = GPIO_R_BASE & (4096 - 1);
    r_gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (r_gpio_vrt_addr == MAP_FAILED) { printf("ERROR: r_gpio mmap() failed\n"); return; }
    gpio[GPIO_PORTS_CNT - 1] = (uint32_t *) ( r_gpio_vrt_addr + (off+16)/4 );

    // no need to keep phy memory file open after mmap
    close(mem_fd);
}

void mem_deinit(void)
{
    munmap(shm_vrt_addr, ARISC_SHM_SIZE);
    munmap(gpio_vrt_addr, 4096);
    munmap(r_gpio_vrt_addr, 4096);
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
            (GPIO_PORTS_CNT - 1), (GPIO_PINS_CNT - 1),
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
