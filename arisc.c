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

volatile uint32_t * _pgs[PG_CH_MAX_CNT] = {0};
volatile uint32_t * _pgc[PG_CH_MAX_CNT][PG_CH_SLOT_MAX_CNT][PG_CH_DATA_CNT] = {0};
volatile uint32_t * _pgd[PG_DATA_CNT] = {0};

volatile _stepgen_ch_t _sgc[STEPGEN_CH_MAX_CNT] = {0};




static inline
void _spin_lock()
{
    *_pgd[PG_ARM_LOCK] = 1;
    while ( *_pgd[PG_ARISC_LOCK] );
}

static inline
void _spin_unlock()
{
    *_pgd[PG_ARM_LOCK] = 0;
}

static inline
int32_t spin_lock_test(uint32_t usec)
{
    _spin_lock();
    usleep(usec);
    _spin_unlock();
    return 0;
}

static inline
int32_t gpio_pin_setup_for_output(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    }
    uint32_t slot = pin/8, pos = pin%8*4;
    _spin_lock();
    _GPIO[port]->config[slot] &= ~(0b1111 << pos);
    _GPIO[port]->config[slot] |=  (0b0001 << pos);
    _spin_unlock();
    return 0;
}

static inline
int32_t gpio_pin_setup_for_input(uint32_t port, uint32_t pin, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return -1;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -2;
    }
    uint32_t slot = pin/8, pos = pin%8*4;
    _spin_lock();
    _GPIO[port]->config[slot] &= ~(0b1111 << pos);
    _spin_unlock();
    return 0;
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
    _spin_lock();
    _GPIO[port]->data |= (1UL << pin);
    _spin_unlock();
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
    _spin_lock();
    _GPIO[port]->data &= ~(1UL << pin);
    _spin_unlock();
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
    _spin_lock();
    _GPIO[port]->data |= mask;
    _spin_unlock();
    return 0;
}

static inline
int32_t gpio_port_clr(uint32_t port, uint32_t mask, uint32_t safe)
{
    if ( safe )
    {
        if ( port >= GPIO_PORTS_MAX_CNT ) return 0;
    }
    _spin_lock();
    _GPIO[port]->data &= ~mask;
    _spin_unlock();
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
    _spin_lock();
    for ( port = GPIO_PORTS_MAX_CNT; port--; )
    {
        _GPIO[port]->data |= mask[port];
    }
    _spin_unlock();
    return 0;
}

static inline
int32_t gpio_all_clr(uint32_t* mask, uint32_t safe)
{
    uint32_t port;
    _spin_lock();
    for ( port = GPIO_PORTS_MAX_CNT; port--; )
    {
        _GPIO[port]->data &= ~mask[port];
    }
    _spin_unlock();
    return 0;
}




static inline
int32_t pg_data_set(uint32_t name, uint32_t value, uint32_t safe)
{
    if ( safe )
    {
        if ( name >= PG_DATA_CNT ) return -1;
        if ( name == PG_CH_CNT && value >= PG_CH_MAX_CNT ) return -4;
    }
    _spin_lock();
    *_pgd[name] = value;
    _spin_unlock();
    return 0;
}

static inline
uint32_t pg_data_get(uint32_t name, uint32_t safe)
{
    if ( safe )
    {
        if ( name >= PG_DATA_CNT ) return 0;
    }
    _spin_lock();
    uint32_t value = *_pgd[name];
    _spin_unlock();
    return value;
}

static inline
int32_t pg_ch_data_set(uint32_t c, uint32_t s, uint32_t name, uint32_t value, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PG_CH_MAX_CNT ) return -1;
        if ( s >= PG_CH_SLOT_MAX_CNT ) return -2;
        if ( name >= PG_CH_DATA_CNT ) return -3;
    }
    _spin_lock();
    *_pgc[c][s][name] = value;
    _spin_unlock();
    return 0;
}

static inline
uint32_t pg_ch_data_get(uint32_t c, uint32_t s, uint32_t name, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PG_CH_MAX_CNT ) return 0;
        if ( s >= PG_CH_SLOT_MAX_CNT ) return -1;
        if ( name >= PG_CH_DATA_CNT ) return -2;
    }
    _spin_lock();
    uint32_t value = *_pgc[c][s][name];
    _spin_unlock();
    return value;
}

static inline
int32_t pg_ch_slot_set(uint32_t c, uint32_t value, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PG_CH_MAX_CNT ) return -1;
    }
    _spin_lock();
    *_pgs[c] = value;
    _spin_unlock();
    return 0;
}

static inline
uint32_t pg_ch_slot_get(uint32_t c, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= PG_CH_MAX_CNT ) return 0;
    }
    _spin_lock();
    uint32_t value = *_pgs[c];
    _spin_unlock();
    return value;
}

static inline
void _stepgen_ch_setup(uint32_t c)
{
    if ( _sgc[c].busy ) return;

    _sgc[c].pos = 0;
    _sgc[c].dir = 0;
    _sgc[c].busy = 1;

    uint32_t pg_ch_cnt = *_pgd[PG_CH_CNT];
    if ( c >= *_pgd[PG_CH_CNT] ) pg_ch_cnt = c + 1;

    _spin_lock();
    *_pgd[PG_USED] = 1;
    *_pgd[PG_CH_CNT] = pg_ch_cnt;
    _spin_unlock();
}

static inline
int32_t stepgen_pin_setup(uint32_t c, uint8_t type, uint32_t port, uint32_t pin, uint32_t invert, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= STEPGEN_CH_MAX_CNT ) return -1;
        if ( type >= 2 ) return -2;
        if ( port >= GPIO_PORTS_MAX_CNT ) return -3;
        if ( pin >= GPIO_PINS_MAX_CNT ) return -4;
        _stepgen_ch_setup(c);
    }

    _spin_lock();

    *_pgc[c][type][PG_PORT] = port;
    *_pgc[c][type][PG_PIN_MSK] = 1UL << pin;
    *_pgc[c][type][PG_PIN_MSKN] = ~(1UL << pin);

    gpio_pin_setup_for_output(port, pin, safe);
    if ( invert ) gpio_pin_set(port, pin, safe);
    else gpio_pin_clr(port, pin, safe);

    _spin_unlock();

    return 0;
}

static inline
int32_t stepgen_task_add(uint32_t c, int32_t pulses, uint32_t time, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= STEPGEN_CH_MAX_CNT ) return -1;
        if ( !pulses ) return -2;
        _stepgen_ch_setup(c);
    }

    time = (uint32_t) ((uint64_t)time * 450 / 1000);
    uint32_t dir_new = (pulses > 0) ? 0 : 1;
    uint32_t dir_tgs = (_sgc[c].dir != dir_new) ? 2 : 0;
    uint32_t stp_tgs = 1 + 2*((uint32_t)abs(pulses));
    uint32_t s, i;

    _sgc[c].dir = dir_new;
    _sgc[c].pos += pulses;

    _spin_lock();

    s = *_pgs[c];

    if ( *_pgc[c][s][PG_TOGGLES] && !*_pgc[c][s][PG_TYPE] )
    {
        i = *_pgc[c][s][PG_TOGGLES] - 1;
        stp_tgs += i;
        *_pgc[c][s][PG_TOGGLES] = 1 + i%2;
        s = (s+1) & PG_CH_SLOT_MAX;
    }

    uint32_t t = time / ((stp_tgs - 1) + dir_tgs);

    if ( dir_tgs )
    {
        *_pgc[c][s][PG_TYPE] = DIR;
        *_pgc[c][s][PG_TIMEOUT] = t;
        *_pgc[c][s][PG_TICK] = *_pgd[PG_TIMER_TICK];
        *_pgc[c][s][PG_T0] = t;
        *_pgc[c][s][PG_T1] = t;
        *_pgc[c][s][PG_TOGGLES] = dir_tgs;
        s = (s+1) & PG_CH_SLOT_MAX;
    }
    else
    {
        *_pgc[c][s][PG_TICK] = *_pgd[PG_TIMER_TICK];
    }

    *_pgc[c][s][PG_TYPE] = STEP;
    *_pgc[c][s][PG_T0] = t;
    *_pgc[c][s][PG_T1] = t;
    *_pgc[c][s][PG_TOGGLES] = stp_tgs;

    _spin_unlock();

    return 0;
}

static inline
int32_t stepgen_pos_get(uint32_t c, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= STEPGEN_CH_MAX_CNT ) return 0;
    }
    return _sgc[c].pos;
}

static inline
int32_t stepgen_pos_set(uint32_t c, int32_t pos, uint32_t safe)
{
    if ( safe )
    {
        if ( c >= STEPGEN_CH_MAX_CNT ) return -1;
    }
    _sgc[c].pos = pos;
    return 0;
}

static inline
int32_t stepgen_cleanup()
{
    uint32_t *p, i;

    _spin_lock();
    for ( i = PG_CH_MAX_CNT, p = (uint32_t*)_pgs[0]; i--; p++ ) *p = 0;
    for ( i = PG_CH_MAX_CNT*PG_CH_SLOT_MAX_CNT*PG_CH_DATA_CNT, p = (uint32_t*)_pgc[0][0][0]; i--; p++ ) *p = 0;
    for ( i = PG_DATA_CNT, p = (uint32_t*)_pgd[0]; i--; p++ ) *p = 0;
    _spin_unlock();

    for ( i = sizeof(_stepgen_ch_t)*STEPGEN_CH_MAX_CNT/4, p = (uint32_t*)_sgc; i--; p++ ) *p = 0;

    return 0;
}




void mem_init(void)
{
    int32_t mem_fd;
    uint32_t addr, off, port, ch, name, *p, s;

    // open physical memory file
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if ( mem_fd  < 0 ) { printf("ERROR: can't open /dev/mem file\n"); return; }

    // mmap shmem
    addr = PG_SHM_BASE & ~(4096 - 1);
    off = PG_SHM_BASE & (4096 - 1);
    _shm_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (_shm_vrt_addr == MAP_FAILED) { printf("ERROR: shm mmap() failed\n"); return; }
    p = _shm_vrt_addr + off/4;
    for ( s = 0; s < PG_CH_MAX_CNT; s++, p++ ) _pgs[s] = p;
    for ( ch = 0; ch < PG_CH_MAX_CNT; ch++ ) {
        for ( s = 0; s < PG_CH_SLOT_MAX_CNT; s++ ) {
            for ( name = 0; name < PG_CH_DATA_CNT; name++, p++ ) _pgc[ch][s][name] = p;
        }
    }
    for ( name = 0; name < PG_DATA_CNT; name++, p++ ) _pgd[name] = p;

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
    munmap(_shm_vrt_addr, 4096);
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
\n\
    i32  stepgen_pin_setup  (channel, type, port, pin, invert) \n\
    i32  stepgen_task_add   (channel, pulses, time) \n\
    i32  stepgen_pos_get    (channel) \n\
    i32  stepgen_pos_set    (channel, position) \n\
    i32  stepgen_cleanup    () \n\
\n\
    u32  pg_data_get    (name) \n\
    i32  pg_data_set    (name, value) \n\
    u32  pg_ch_data_get (channel, slot, name) \n\
    i32  pg_ch_data_set (channel, slot, name, value) \n\
    u32  pg_ch_slot_get (channel) \n\
    i32  pg_ch_slot_set (channel, value) \n\
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
    time        time in nanoseconds (u32)\n\
    position    position value in pulses (i32)\n\
    slot        pulsgen channel's fifo slot (0..%u)\n\
    name        data name (u32)\n\
    value       data value (u32)\n\
\n\
  NOTE:\n\
    If you are using stdin/stdout mode, omit `%s` and any \" brackets\n\
\n",
            app_name, app_name, app_name,
            (GPIO_PORTS_MAX_CNT - 1),
            (GPIO_PINS_MAX_CNT - 1),
            (PG_CH_SLOT_MAX_CNT - 1),
            app_name
        );
        return 0;
    }

    if ( !reg_match(str, "spin_lock_test *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%s\n", (spin_lock_test(arg[0])) ? "ERROR" : "OK");
        return 0;
    }

    // --- GPIO ------

    if ( !reg_match(str, "gpio_pin_setup_for_output *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_pin_setup_for_output(arg[0], arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "gpio_pin_setup_for_input *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (gpio_pin_setup_for_input(arg[0], arg[1], 1)) ? "ERROR" : "OK");
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

    // --- STEPGEN ------

    if ( !reg_match(str, "stepgen_pin_setup *\\("UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 5) )
    {
        printf("%s\n", (stepgen_pin_setup(arg[0], arg[1], arg[2], arg[3], arg[4], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "stepgen_task_add *\\("UINT","INT","UINT"\\)", &arg[0], 3) )
    {
        printf("%s\n", (stepgen_task_add(arg[0], arg[1], arg[2], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "stepgen_pos_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%i\n", stepgen_pos_get(arg[0], 1));
        return 0;
    }
    if ( !reg_match(str, "stepgen_pos_set *\\("UINT","INT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (stepgen_pos_set(arg[0], (int32_t)arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "stepgen_cleanup *\\(\\)", &arg[0], 0) )
    {
        printf("%s\n", (stepgen_cleanup()) ? "ERROR" : "OK");
        return 0;
    }

    // --- PULSGEN ------

    if ( !reg_match(str, "pg_data_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%u\n", pg_data_get(arg[0], 1));
        return 0;
    }
    if ( !reg_match(str, "pg_data_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (pg_data_set(arg[0], arg[1], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "pg_ch_data_get *\\("UINT","UINT","UINT"\\)", &arg[0], 3) )
    {
        printf("%u\n", pg_ch_data_get(arg[0], arg[1], arg[2], 1));
        return 0;
    }
    if ( !reg_match(str, "pg_ch_data_set *\\("UINT","UINT","UINT","UINT"\\)", &arg[0], 4) )
    {
        printf("%s\n", (pg_ch_data_set(arg[0], arg[1], arg[2], arg[3], 1)) ? "ERROR" : "OK");
        return 0;
    }
    if ( !reg_match(str, "pg_ch_slot_get *\\("UINT"\\)", &arg[0], 1) )
    {
        printf("%u\n", pg_ch_slot_get(arg[0], 1));
        return 0;
    }
    if ( !reg_match(str, "pg_ch_slot_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%s\n", (pg_ch_slot_set(arg[0], arg[1], 1)) ? "ERROR" : "OK");
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
