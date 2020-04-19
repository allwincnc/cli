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




static inline
void gpio_spin_lock()
{
    while ( *gpiod[GPIO_ARISC_LOCK] );
    *gpiod[GPIO_ARM_LOCK] = 1;
}

static inline
void gpio_spin_unlock()
{
    *gpiod[GPIO_ARM_LOCK] = 0;
}

static inline
void gpio_pin_setup_for_output(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    if ( pin >= GPIO_PINS_CNT ) return;
    gpio_spin_lock();
    *gpio_shm_out[port] |= (1UL << pin);
    gpio_spin_unlock();
}

static inline
void gpio_pin_setup_for_input(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    if ( pin >= GPIO_PINS_CNT ) return;
    gpio_spin_lock();
    *gpio_shm_inp[port] |= (1UL << pin);
    gpio_spin_unlock();
}

static inline
uint32_t gpio_pin_get(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return 0;
    if ( pin >= GPIO_PINS_CNT ) return 0;
    return *gpio[port] & (1UL << pin) ? HIGH : LOW;
}

static inline
void gpio_pin_set(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    if ( pin >= GPIO_PINS_CNT ) return;
    gpio_spin_lock();
    *gpio_shm_set[port] |= (1UL << pin);
    gpio_spin_unlock();
}

static inline
void gpio_pin_clr(uint32_t port, uint32_t pin)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    if ( pin >= GPIO_PINS_CNT ) return;
    gpio_spin_lock();
    *gpio_shm_clr[port] |= (1UL << pin);
    gpio_spin_unlock();
}

static inline
uint32_t gpio_port_get(uint32_t port)
{
    if ( port >= GPIO_PORTS_CNT ) return 0;
    return *gpio[port];
}

static inline
void gpio_port_set(uint32_t port, uint32_t mask)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    gpio_spin_lock();
    *gpio_shm_set[port] = mask;
    gpio_spin_unlock();
}

static inline
void gpio_port_clr(uint32_t port, uint32_t mask)
{
    if ( port >= GPIO_PORTS_CNT ) return;
    gpio_spin_lock();
    *gpio_shm_clr[port] = mask;
    gpio_spin_unlock();
}

static inline
void gpio_data_set(uint32_t name, uint32_t value)
{
    if ( name >= GPIO_DATA_CNT ) return;
    gpio_spin_lock();
    uint32_t port;
    if ( name == GPIO_PORT_MAX_ID || name == GPIO_USED )
    {
        for ( port = GPIO_PORTS_CNT; port--; )
        {
            *gpio_shm_set[port] = 0;
            *gpio_shm_clr[port] = 0;
            *gpio_shm_out[port] = 0;
            *gpio_shm_inp[port] = 0;
        }
        value = (value >= (GPIO_PORTS_CNT-1)) ? GPIO_PORTS_CNT-1 : value;
    }
    *gpiod[name] = value;
    gpio_spin_unlock();
}

static inline
uint32_t gpio_data_get(uint32_t name)
{
    if ( name >= GPIO_DATA_CNT ) return 0;
    gpio_spin_lock();
    uint32_t value = *gpiod[name];
    gpio_spin_unlock();
    return value;
}




void mem_init(void)
{
    int32_t mem_fd, port;
    uint32_t addr, off;

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
        gpio_shm_set[port] = (uint32_t *) ( shm_vrt_addr + (off + port*4 + (GPIO_SHM_SET_BASE - GPIO_SHM_BASE))/4 );
        gpio_shm_clr[port] = (uint32_t *) ( shm_vrt_addr + (off + port*4 + (GPIO_SHM_CLR_BASE - GPIO_SHM_BASE))/4 );
        gpio_shm_out[port] = (uint32_t *) ( shm_vrt_addr + (off + port*4 + (GPIO_SHM_OUT_BASE - GPIO_SHM_BASE))/4 );
        gpio_shm_inp[port] = (uint32_t *) ( shm_vrt_addr + (off + port*4 + (GPIO_SHM_INP_BASE - GPIO_SHM_BASE))/4 );
    }
    for ( port = 0; port < GPIO_DATA_CNT; port++ )
    {
        gpiod[port] = (uint32_t *) ( shm_vrt_addr + (off + port*4 + (GPIO_SHM_DATA_BASE - GPIO_SHM_BASE))/4 );
    }

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
         gpio_pin_setup_for_output  (port, pin) \n\
         gpio_pin_setup_for_input   (port, pin) \n\
    u32  gpio_pin_get               (port, pin) \n\
         gpio_pin_set               (port, pin) \n\
         gpio_pin_clr               (port, pin) \n\
    u32  gpio_port_get              (port) \n\
         gpio_port_set              (port, mask) \n\
         gpio_port_clr              (port, mask) \n\
    u32  gpio_data_get              (name) \n\
         gpio_data_set              (name, value) \n\
\n\
  Legend: \n\
\n\
    port    GPIO port (0..%u | PA/PB/PC/PD/PE/PF/PG/PL)\n\
    pin     GPIO pin (0..%u)\n\
    mask    GPIO pins mask (u32)\n\
    name    GPIO data name (0..%u)\n\
    value   GPIO data value (u32)\n\
\n\
  NOTE:\n\
    If you are using stdin/stdout mode, omit `%s` and any \" brackets\n\
\n",
            app_name, app_name, app_name,
            (GPIO_PORTS_CNT - 1), (GPIO_PINS_CNT - 1), (GPIO_DATA_CNT - 1),
            app_name
        );
        return 0;
    }

    // --- GPIO ------

    if ( !reg_match(str, "gpio_pin_setup_for_output *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        gpio_pin_setup_for_output(arg[0], arg[1]);
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_pin_setup_for_input *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        gpio_pin_setup_for_input(arg[0], arg[1]);
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_pin_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        gpio_pin_set(arg[0], arg[1]);
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_pin_clr *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        gpio_pin_clr(arg[0], arg[1]);
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_pin_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        printf("%u\n", gpio_pin_get(arg[0], arg[1]));
        return 0;
    }

    if ( !reg_match(str, "gpio_port_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        gpio_port_set(arg[0], arg[1]);
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_port_clr *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
        gpio_port_clr(arg[0], arg[1]);
        printf("OK\n");
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
        gpio_data_set(arg[0], arg[1]);
        printf("OK\n");
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
