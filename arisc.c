/**
 * @file    arisc.c
 *
 * @brief   ARISC firmware API source
 *
 * This test program implements an API to ARISC firmware
 */

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




// private vars

#define TEST 0 // 0 = real execution of ARISC functions, 1 = just a safe test

static uint32_t *shm_vrt_addr, *gpio_vrt_addr, *r_gpio_vrt_addr;
static char *app_name = 0;
volatile uint32_t * gpio_port_data[GPIO_PORTS_CNT] = {0};
volatile uint32_t * gpio_shm_set[GPIO_PORTS_CNT] = {0};
volatile uint32_t * gpio_shm_clr[GPIO_PORTS_CNT] = {0};
volatile uint32_t * gpio_shm_out[GPIO_PORTS_CNT] = {0};
volatile uint32_t * gpio_shm_inp[GPIO_PORTS_CNT] = {0};




// public methods

/**
 * @brief   set pin mode to OUTPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT-1)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT-1)
 * @retval  none
 */
void gpio_pin_setup_for_output(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    if ( pin >= GPIO_PINS_CNT ) return;
    *gpio_shm_out[port] |= (1UL << pin);
}

/**
 * @brief   set pin mode to INPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT-1)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT-1)
 * @retval  none
 */
void gpio_pin_setup_for_input(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    if ( pin >= GPIO_PINS_CNT ) return;
    *gpio_shm_inp[port] |= (1UL << pin);
}

/**
 * @brief   get pin state
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT-1)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT-1)
 * @retval  1 (HIGH)
 * @retval  0 (LOW)
 */
uint32_t gpio_pin_get(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return 0;
    if ( pin >= GPIO_PINS_CNT ) return 0;
    return *gpio_port_data[port] & (1UL << pin) ? HIGH : LOW;
}

/**
 * @brief   set pin state to HIGH (1)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT-1)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT-1)
 * @retval  none
 */
void gpio_pin_set(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    if ( pin >= GPIO_PINS_CNT ) return;
    *gpio_shm_set[port] |= (1UL << pin);
}

/**
 * @brief   set pin state to LOW (0)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT-1)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT-1)
 * @retval  none
 */
void gpio_pin_clear(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    if ( pin >= GPIO_PINS_CNT ) return;
    *gpio_shm_clr[port] |= (1UL << pin);
}

/**
 * @brief   get port state
 * @param   port    GPIO port number (0 .. GPIO_PORTS_CNT-1)
 * @note    each bit value of returned value represents port pin state
 * @retval  0 .. 0xFFFFFFFF
 */
uint32_t gpio_port_get(uint32_t port)
{
    if ( port >= GPIO_PORTS_CNT ) return 0;
    return *gpio_port_data[port];
}

/**
 * @brief   set port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT-1)
 * @param   mask    GPIO pins mask to set   (0 .. 0xFFFFFFFF) \n\n
 *                  mask examples: \n\n
 *                      mask = 0xFFFFFFFF (0b11111111111111111111111111111111) means <b>set all pins state to 1 (HIGH)</b> \n
 *                      mask = 0x00000001 (0b1) means <b>set pin 0 state to 1 (HIGH)</b> \n
 *                      mask = 0x0000000F (0b1111) means <b>set pins 0,1,2,3 states to 1 (HIGH)</b>
 *
 * @retval  none
 */
void gpio_port_set(uint32_t port, uint32_t mask)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    *gpio_shm_set[port] = mask;
}

/**
 * @brief   clear port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT-1)
 * @param   mask    GPIO pins mask to clear (0 .. 0xFFFFFFFF) \n\n
 *                  mask examples: \n\n
 *                  mask = 0xFFFFFFFF (0b11111111111111111111111111111111) means <b>set all pins state to 0 (LOW)</b> \n
 *                  mask = 0x00000003 (0b11) means <b>set pins 0,1 states to 0 (LOW)</b> \n
 *                  mask = 0x00000008 (0b1000) means <b>set pin 3 state to 0 (LOW)</b>
 *
 * @retval  none
 */
void gpio_port_clear(uint32_t port, uint32_t mask)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    *gpio_shm_clr[port] = mask;
}




void mem_init(void)
{
    int32_t mem_fd, port;
    uint32_t addr, off;

    // open physical memory file
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if ( mem_fd  < 0 ) { printf("ERROR: can't open /dev/mem file\n"); return; }

    addr = ARISC_SHM_BASE & ~(4096 - 1);
    off = ARISC_SHM_BASE & (4096 - 1);
    shm_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (shm_vrt_addr == MAP_FAILED) { printf("ERROR: shm mmap() failed\n"); return; }
    for ( port = 0; port < GPIO_PORTS_CNT; port++ )
    {
        gpio_shm_set[port] = (uint32_t *) ( shm_vrt_addr + (off + port*4 + (GPIO_SHM_SET_BASE - GPIO_SHM_BASE))/4 );
        gpio_shm_clr[port] = (uint32_t *) ( shm_vrt_addr + (off + port*4 + (GPIO_SHM_CLR_BASE - GPIO_SHM_BASE))/4 );
        gpio_shm_out[port] = (uint32_t *) ( shm_vrt_addr + (off + port*4 + (GPIO_SHM_OUT_BASE - GPIO_SHM_BASE))/4 );
        gpio_shm_inp[port] = (uint32_t *) ( shm_vrt_addr + (off + port*4 + (GPIO_SHM_INP_BASE - GPIO_SHM_BASE))/4 );
    }

    addr = GPIO_BASE & ~(4096 - 1);
    off = GPIO_BASE & (4096 - 1);
    gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (gpio_vrt_addr == MAP_FAILED) { printf("ERROR: gpio mmap() failed\n"); return; }
    for ( port = 0; port < (GPIO_PORTS_CNT - 1); port++ )
    {
        gpio_port_data[port] = (uint32_t *) ( gpio_vrt_addr + (off + port*GPIO_BANK_SIZE + 16)/4 );
    }

    addr = GPIO_R_BASE & ~(4096 - 1);
    off = GPIO_R_BASE & (4096 - 1);
    r_gpio_vrt_addr = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr);
    if (r_gpio_vrt_addr == MAP_FAILED) { printf("ERROR: r_gpio mmap() failed\n"); return; }
    gpio_port_data[GPIO_PORTS_CNT - 1] = (uint32_t *) ( r_gpio_vrt_addr + (off+16)/4 );

    // no need to keep phy memory file open after mmap
    close(mem_fd);
}

void mem_deinit(void)
{
    munmap(shm_vrt_addr, 4096);
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
//                printf("    unknown argument string `%s` \n", (const char *)&match[0]);
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

    #define UINT " *([0-9]+|0x[A-Fa-f]+|0b[01]+|P[ABCDEFGL]|PH_[ABZ]|PHASE_[ABZ]) *"
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
    %s examples     show a few examples \n\
    %s exit|quit|q  program quit \n\
\n\
  Functions: \n\
\n\
         gpio_pin_setup_for_output  (port, pin) \n\
         gpio_pin_setup_for_input   (port, pin) \n\
    int  gpio_pin_get               (port, pin) \n\
         gpio_pin_set               (port, pin) \n\
         gpio_pin_clear             (port, pin) \n\
    int  gpio_port_get              (port) \n\
         gpio_port_set              (port, mask) \n\
         gpio_port_clear            (port, mask) \n\
\n\
\n\
  Legend: \n\
\n\
    port            GPIO port (0..%u, PA, PB, PC, PD, PE, PF, PG, PL)\n\
    pin             GPIO pin (0..%u)\n\
    mask            GPIO pins mask (0b0 .. 0b11111111111111111111111111111111)\n\
\n\
  NOTE:\n\
    If you are using stdin/stdout mode, omit `%s` and any \" brackets\n\
\n",
            app_name, app_name, app_name, app_name,
            (GPIO_PORTS_CNT - 1), (GPIO_PINS_CNT - 1),
            app_name
        );
        return 0;
    }

    if ( !reg_match(str, "example", &arg[0], 0) )
    {
        printf(
"\n\
  GPIO examples:\n\
\n\
    %s \"gpio_pin_setup_for_output(PA,15)\" # setup PA15 pin as output \n\
    %s \"gpio_pin_clear(PA,15)\"            # set PA15 state to 0 \n\
    %s \"gpio_pin_set(PA,15)\"              # set PA15 state to 1 \n\
    %s \"gpio_pin_get(PA,15)\"              # get PA15 state \n\
    %s \"gpio_port_clear(PA, 0b11)\"        # set PA0,PA1 state to 0 \n\
    %s \"gpio_port_set(PA, 0b1011)\"        # set PA0,PA1,PA3 state to 1 \n\
    %s \"gpio_port_get(PA)\"                # get PA port all pins state \n\
\n\
  NOTE:\n\
    If you are using stdin/stdout mode, omit `%s` and any \" brackets\n\
\n",
            app_name, app_name, app_name, app_name, app_name, app_name, app_name,
            app_name
        );
        return 0;
    }

    // --- GPIO ------

    if ( !reg_match(str, "gpio_pin_setup_for_output *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
#if !TEST
        gpio_pin_setup_for_output(arg[0], arg[1]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_pin_setup_for_input *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
#if !TEST
        gpio_pin_setup_for_input(arg[0], arg[1]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_pin_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
#if !TEST
        gpio_pin_set(arg[0], arg[1]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_pin_clear *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
#if !TEST
        gpio_pin_clear(arg[0], arg[1]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_pin_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
#if !TEST
        printf("%u\n", gpio_pin_get(arg[0], arg[1]));
#else
        printf("%u\n", 1);
#endif
        return 0;
    }

    if ( !reg_match(str, "gpio_port_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
#if !TEST
        gpio_port_set(arg[0], arg[1]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_port_clear *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
#if !TEST
        gpio_port_clear(arg[0], arg[1]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_port_get *\\("UINT"\\)", &arg[0], 1) )
    {
#if !TEST
        uint32_t s = gpio_port_get(arg[0]);
#else
        uint32_t s = 0x12345678;
#endif
        uint32_t b = 32;
        printf("%u, 0x%X, 0b", s, s);
        for ( ; b--; ) printf("%u", (s & (1U << b) ? 1 : 0));
        printf("\n");
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
  Welcome to stdin/stdout mode of ARISC CNC API.\n\
\n\
  Type `help` to see help info.\n\
  Type `examples` to see working examples.\n\
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
