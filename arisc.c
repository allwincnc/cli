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

static struct msg_t * msg_arisc[MSG_MAX_CNT] = {0};
static struct msg_t * msg_arm[MSG_MAX_CNT] = {0};
static uint8_t msg_buf[MSG_LEN] = {0};

static uint32_t *vrt_block_addr;

static char *app_name = 0;




// main entry

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








// public methods

/**
 * @brief   setup encoder pin for the selected channel and phase
 *
 * @param   c           channel id
 * @param   phase       PHASE_A..PHASE_Z
 * @param   port        GPIO port number
 * @param   pin         GPIO pin number
 *
 * @retval  none
 */
void encoder_pin_setup(uint8_t c, uint8_t phase, uint8_t port, uint8_t pin)
{
    struct encoder_msg_pin_setup_t tx = *((struct encoder_msg_pin_setup_t *) &msg_buf);

    tx.ch = c;
    tx.phase = phase;
    tx.port = port;
    tx.pin = pin;

    msg_send(ENCODER_MSG_PIN_SETUP, (uint8_t*)&tx, 4*4, 0);
}

/**
 * @brief   setup selected channel of encoder counter
 *
 * @param   c           channel id
 * @param   using_B     use phase B input?
 * @param   using_Z     use phase Z index input?
 *
 * @retval  none
 */
void encoder_setup(uint8_t c, uint8_t using_B, uint8_t using_Z)
{
    struct encoder_msg_setup_t tx = *((struct encoder_msg_setup_t *) &msg_buf);

    tx.ch = c;
    tx.using_B = using_B;
    tx.using_Z = using_Z;

    msg_send(ENCODER_MSG_SETUP, (uint8_t*)&tx, 3*4, 0);
}

/**
 * @brief   enable/disable selected channel of encoder counter
 * @param   c       channel id
 * @retval  none
 */
void encoder_state_set(uint8_t c, uint8_t state)
{
    struct encoder_msg_state_set_t tx = *((struct encoder_msg_state_set_t *) &msg_buf);

    tx.ch = c;
    tx.state = state;

    msg_send(ENCODER_MSG_STATE_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   change number of counts for the selected channel
 * @param   c       channel id
 * @param   counts  new value for encoder channel counts
 * @retval  none
 */
void encoder_counts_set(uint8_t c, int32_t counts)
{
    struct encoder_msg_counts_set_t tx = *((struct encoder_msg_counts_set_t *) &msg_buf);

    tx.ch = c;
    tx.counts = counts;

    msg_send(ENCODER_MSG_COUNTS_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   get state for the selected channel
 *
 * @param   c   channel id
 *
 * @retval  0   (channel is disabled)
 * @retval  1   (channel is enabled)
 */
uint8_t encoder_state_get(uint8_t c)
{
    struct encoder_msg_ch_t tx = *((struct encoder_msg_ch_t *) &msg_buf);
    struct encoder_msg_state_get_t rx = *((struct encoder_msg_state_get_t *) &msg_buf);

    tx.ch = c;

    msg_send(ENCODER_MSG_STATE_GET, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 9999 tries to read an answer
    uint32_t n = 0;
    for ( n = 9999; n--; )
    {
        if ( msg_read(ENCODER_MSG_STATE_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   get current counts for the selected channel
 * @param   c   channel id
 * @retval  signed 32-bit number
 */
int32_t encoder_counts_get(uint8_t c)
{
    struct encoder_msg_ch_t tx = *((struct encoder_msg_ch_t *) &msg_buf);
    struct encoder_msg_counts_get_t rx = *((struct encoder_msg_counts_get_t *) &msg_buf);

    tx.ch = c;

    msg_send(ENCODER_MSG_COUNTS_GET, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 9999 tries to read an answer
    uint32_t n = 0;
    for ( n = 9999; n--; )
    {
        if ( msg_read(ENCODER_MSG_COUNTS_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.counts;
    }

    return 0;
}




/**
 * @brief   setup GPIO pin for the selected channel
 *
 * @param   c               channel id
 * @param   type            0:step, 1:dir
 * @param   port            GPIO port number
 * @param   pin             GPIO pin number
 * @param   invert          invert pin state?
 *
 * @retval  none
 */
void stepgen_pin_setup(uint8_t c, uint8_t type, uint8_t port, uint8_t pin, uint8_t invert)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;
    tx->v[1] = type;
    tx->v[2] = port;
    tx->v[3] = pin;
    tx->v[4] = invert;

    msg_send(STEPGEN_MSG_PIN_SETUP, msg_buf, 5*4, 0);
}

/**
 * @brief   add a new task for the selected channel
 *
 * @param   c           channel id
 * @param   pulses      number of pulses
 *
 * @retval  none
 */
void stepgen_task_add(uint8_t c, int32_t pulses)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;
    tx->v[1] = pulses;

    msg_send(STEPGEN_MSG_TASK_ADD, msg_buf, 2*4, 0);
}

/**
 * @brief   update time values for the current task
 *
 * @param   c       channel id
 * @param   type    0:step, 1:dir
 * @param   t0      pin LOW state duration (in nanoseconds)
 * @param   t1      pin HIGH state duration (in nanoseconds)
 *
 * @retval  none
 */
void stepgen_time_setup(uint8_t c, uint8_t type, uint32_t t0, uint32_t t1)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;
    tx->v[1] = type;
    tx->v[2] = t0;
    tx->v[3] = t1;

    msg_send(STEPGEN_MSG_TIME_SETUP, msg_buf, 4*4, 0);
}

/**
 * @brief   set channel steps position
 * @param   c   channel id
 * @retval  integer 4-bytes
 */
int32_t stepgen_pos_get(uint8_t c)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;

    msg_send(STEPGEN_MSG_POS_GET, msg_buf, 1*4, 0);

    // finite loop, only 9999 tries to read an answer
    uint32_t n = 0;
    for ( n = 9999; n--; )
    {
        if ( msg_read(STEPGEN_MSG_POS_GET, msg_buf, 0) < 0 ) continue;
        else return (int32_t)tx->v[0];
    }

    return 0;
}

/**
 * @brief   set channel steps position
 * @param   c       channel id
 * @param   pos     integer 4-bytes
 * @retval  none
 */
void stepgen_pos_set(uint8_t c, int32_t pos)
{
    u32_10_t *tx = (u32_10_t*) msg_buf;

    tx->v[0] = c;
    tx->v[1] = (uint32_t)pos;

    msg_send(STEPGEN_MSG_POS_SET, msg_buf, 2*4, 0);
}

#if STEPGEN_DEBUG
    int32_t stepgen_param_get(uint8_t c, uint8_t p)
    {
        u32_10_t *tx = (u32_10_t*) msg_buf;

        tx->v[0] = c;
        tx->v[1] = p;

        msg_send(STEPGEN_MSG_PARAM_GET, msg_buf, 2*4, 0);

        // finite loop, only 9999 tries to read an answer
        uint32_t n = 0;
        for ( n = 9999; n--; )
        {
            if ( msg_read(STEPGEN_MSG_PARAM_GET, msg_buf, 0) < 0 ) continue;
            else return (int32_t)tx->v[0];
        }

        return 0;
    }

    void stepgen_param_set(uint8_t c, uint8_t p, int32_t v)
    {
        u32_10_t *tx = (u32_10_t*) msg_buf;

        tx->v[0] = c;
        tx->v[1] = p;
        tx->v[2] = (uint32_t)v;

        msg_send(STEPGEN_MSG_POS_SET, msg_buf, 3*4, 0);
    }
#endif




/**
 * @brief   set pin mode to OUTPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_setup_for_output(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_SETUP_FOR_OUTPUT, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   set pin mode to INPUT
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_setup_for_input(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_SETUP_FOR_INPUT, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   get pin state
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  1 (HIGH)
 * @retval  0 (LOW)
 */
uint32_t gpio_pin_get(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);
    struct gpio_msg_state_t rx = *((struct gpio_msg_state_t *) &msg_buf);

    tx.port = port;
    tx.pin = pin;

    msg_send(GPIO_MSG_PIN_GET, (uint8_t*)&tx, 2*4, 0);

    // finite loop, only 9999 tries to read an answer
    uint32_t n = 0;
    for ( n = 9999; n--; )
    {
        if ( msg_read(GPIO_MSG_PIN_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   set pin state to HIGH (1)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_set(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_PIN_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   set pin state to LOW (0)
 * @param   port    GPIO port number    (0 .. GPIO_PORTS_CNT)
 * @param   pin     GPIO pin number     (0 .. GPIO_PINS_CNT)
 * @retval  none
 */
void gpio_pin_clear(uint32_t port, uint32_t pin)
{
    struct gpio_msg_port_pin_t tx = *((struct gpio_msg_port_pin_t *) &msg_buf);

    tx.port = port;
    tx.pin  = pin;

    msg_send(GPIO_MSG_PIN_CLEAR, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   get port state
 * @param   port    GPIO port number (0 .. GPIO_PORTS_CNT)
 * @note    each bit value of returned value represents port pin state
 * @retval  0 .. 0xFFFFFFFF
 */
uint32_t gpio_port_get(uint32_t port)
{
    struct gpio_msg_port_t tx = *((struct gpio_msg_port_t *) &msg_buf);
    struct gpio_msg_state_t rx = *((struct gpio_msg_state_t *) &msg_buf);

    tx.port = port;

    msg_send(GPIO_MSG_PORT_GET, (uint8_t*)&tx, 1*4, 0);

    // finite loop, only 9999 tries to read an answer
    uint32_t n = 0;
    for ( n = 9999; n--; )
    {
        if ( msg_read(GPIO_MSG_PORT_GET, (uint8_t*)&rx, 0) < 0 ) continue;
        else return rx.state;
    }

    return 0;
}

/**
 * @brief   set port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT)
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
    struct gpio_msg_port_mask_t tx = *((struct gpio_msg_port_mask_t *) &msg_buf);

    tx.port = port;
    tx.mask = mask;

    msg_send(GPIO_MSG_PORT_SET, (uint8_t*)&tx, 2*4, 0);
}

/**
 * @brief   clear port pins state by mask
 *
 * @param   port    GPIO port number        (0 .. GPIO_PORTS_CNT)
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
    struct gpio_msg_port_mask_t tx = *((struct gpio_msg_port_mask_t *) &msg_buf);

    tx.port = port;
    tx.mask = mask;

    msg_send(GPIO_MSG_PORT_CLEAR, (uint8_t*)&tx, 2*4, 0);
}

uint32_t* gpio_all_get()
{
    static uint32_t n;

    msg_send(GPIO_MSG_ALL_GET, (uint8_t*)&msg_buf[0], 0*4, 0);
    for ( n = 9999; n--; )
    {
        if ( msg_read(GPIO_MSG_ALL_GET, (uint8_t*)&msg_buf[0], 0) < 0 ) continue;
        else return (uint32_t*) &msg_buf;
    }

    return (uint32_t*) 0;
}

void gpio_all_set(uint32_t* mask)
{
    msg_send(GPIO_MSG_ALL_SET, (uint8_t*)mask, GPIO_PORTS_CNT*4, 0);
}

void gpio_all_clear(uint32_t* mask)
{
    msg_send(GPIO_MSG_ALL_CLEAR, (uint8_t*)mask, GPIO_PORTS_CNT*4, 0);
}




/**
 * @brief   read a message from the ARISC cpu
 *
 * @param   type    user defined message type (0..0xFF)
 * @param   msg     pointer to the message buffer
 * @param   bswap   0 - if you sending an array of 32bit numbers, 1 - for the text
 *
 * @retval   0 (message read)
 * @retval  -1 (message not read)
 */
int8_t msg_read(uint8_t type, uint8_t * msg, uint8_t bswap)
{
    static uint8_t last = 0;
    static uint8_t m = 0;
    static uint8_t i = 0;
    static uint32_t * link;
    static int8_t msg_len;

    // find next unread message
    for ( i = MSG_MAX_CNT, m = last; i--; )
    {
        // process message only of current type
        if ( msg_arisc[m]->unread && !msg_arisc[m]->locked && msg_arisc[m]->type == type )
        {
            // message slot is busy
            msg_arisc[m]->locked = 1;

            msg_len = msg_arisc[m]->length;

            if ( bswap )
            {
                // swap message data bytes for correct reading by ARM
                link = ((uint32_t*) &msg_arisc[m]->msg);
                for ( i = msg_len / 4 + 1; i--; link++ )
                {
                    *link = __bswap_32(*link);
                }
            }

            // copy message to the buffer
            memcpy(msg, &msg_arisc[m]->msg, msg_len);

            // message read
            msg_arisc[m]->unread = 0;
            // message slot is free
            msg_arisc[m]->locked = 0;

            last = m;
            return msg_len;
        }

        m = (m + 1) & (MSG_MAX_CNT - 1);
    }

    return -1;
}

/**
 * @brief   send a message to the ARISC cpu
 *
 * @param   type    user defined message type (0..0xFF)
 * @param   msg     pointer to the message buffer
 * @param   length  the length of a message (0..MSG_LEN) )
 * @param   bswap   0 - if you sending an array of 32bit numbers, 1 - for the text
 *
 * @retval   0 (message sent)
 * @retval  -1 (message not sent)
 */
int8_t msg_send(uint8_t type, uint8_t * msg, uint8_t length, uint8_t bswap)
{
    static uint8_t last = 0;
    static uint8_t m = 0;
    static uint8_t i = 0;
    static uint32_t * link;

    // find next free message slot
    for ( i = MSG_MAX_CNT, m = last; i--; )
    {
        // sending message
        if ( !msg_arm[m]->unread && !msg_arisc[m]->locked )
        {
            // message slot is busy
            msg_arisc[m]->locked = 1;

            // copy message to the buffer
            memset( (uint8_t*)((uint8_t*)&msg_arm[m]->msg + length/4*4), 0, 4);
            memcpy(&msg_arm[m]->msg, msg, length);

            if ( bswap )
            {
                // swap message data bytes for correct reading by ARISC
                link = ((uint32_t*) &msg_arm[m]->msg);
                for ( i = length / 4 + 1; i--; link++ )
                {
                    *link = __bswap_32(*link);
                }
            }

            // set message data
            msg_arm[m]->type   = type;
            msg_arm[m]->length = length;
            msg_arm[m]->unread = 1;

            // message slot is free
            msg_arisc[m]->locked = 0;

            // message sent
            last = m;
            return 0;
        }

        m = (m + 1) & (MSG_MAX_CNT - 1);
    }

    // message not sent
    return -1;
}




void mem_init(void)
{
    int32_t     mem_fd;
    uint32_t    vrt_offset = 0;
    off_t       phy_block_addr = 0;
    int32_t     m = 0;

    // open physical memory file
    if ( (mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0 )
    {
       printf("ERROR: can't open /dev/mem file\n");
       return;
    }

    // calculate phy memory block start
    vrt_offset = MSG_ARISC_BLOCK_ADDR % PHY_MEM_BLOCK_SIZE;
    phy_block_addr = MSG_ARISC_BLOCK_ADDR - vrt_offset;

    // make a block of phy memory visible in our user space
    vrt_block_addr = mmap(NULL, 2*MSG_BLOCK_SIZE, PROT_READ | PROT_WRITE,
       MAP_SHARED, mem_fd, phy_block_addr);

    // exit program if mmap is failed
    if (vrt_block_addr == MAP_FAILED)
    {
       printf("ERROR: mmap() failed\n");
       return;
    }

    // no need to keep phy memory file open after mmap
    close(mem_fd);

    // adjust offset to correct value
    vrt_block_addr += (vrt_offset/4);

    // assign messages pointers
    for ( m = 0; m < MSG_MAX_CNT; ++m )
    {
        msg_arisc[m] = (struct msg_t *) (vrt_block_addr + (m * MSG_MAX_LEN)/4);
        msg_arm[m]   = (struct msg_t *) (vrt_block_addr + (m * MSG_MAX_LEN + MSG_CPU_BLOCK_SIZE)/4);
    }
}

void mem_deinit(void)
{
    munmap(vrt_block_addr, 2*MSG_BLOCK_SIZE);
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
            // if we have a phase name
            else if ( size >= 4 && match[0] == 'P' && match[1] == 'H' )
            {
                uint32_t phase;
                char phase_char = 40;

                if ( size == 4 )        phase_char = match[3]; // PH_X
                else if ( size == 7 )   phase_char = match[6]; // PHASE_X

                switch (phase_char) {
                    case 'A':   phase = PH_A; break;
                    case 'B':   phase = PH_B; break;
                    case 'Z':   phase = PH_Z; break;
                    default:    phase = 40;   break;
                }

                // if we have an unknown string
                if ( phase > PH_Z )
                {
                    ret = 4;
                    break;
                }

                *arg = phase;
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
         stepgen_pin_setup      (channel, type, port, pin, invert) \n\
         stepgen_time_setup     (channel, type, t0, t1) \n\
         stepgen_task_add       (channel, pulses) \n\
    int  stepgen_pos_get        (channel) \n\
         stepgen_pos_set        (channel, position) \n\
\n\
         encoder_pin_setup      (channel, phase, port, pin) \n\
         encoder_setup          (channel, using_B, using_Z) \n\
         encoder_state_set      (channel, state) \n\
         encoder_counts_set     (channel, counts) \n\
    int  encoder_state_get      (channel) \n\
    int  encoder_counts_get     (channel) \n\
\n\
  Legend: \n\
\n\
    port            GPIO port (0..7, PA, PB, PC, PD, PE, PF, PG, PL)\n\
    pin             GPIO pin (0..31)\n\
    mask            GPIO pins mask (0b0 .. 0b11111111111111111111111111111111)\n\
\n\
    channel         channel ID (0..16 for stepgen, 0-7 for encoder)\n\
    type            0 = STEP, 1 = DIR\n\
    invert          invert GPIO pin? (0..1)\n\
    pulses          number of pin pulses (-2147483648..2147483647)\n\
    t0              pin state setup time in nanoseconds (0..4294967295)\n\
    t1              pin state hold time in nanoseconds (0..4294967295)\n\
    position        position value in pulses (integer 4 bytes)\n\
\n\
    phase           encoder phase (0..2, PH_A, PH_B, PH_Z)\n\
    using_B         use phase B? (0..1)\n\
    using_Z         use phase Z? (0..1)\n\
    state           channel state (0..1)\n\
    counts          channel counts value (-2147483647 .. 2147483647)\n\
\n\
  NOTE:\n\
    If you are using stdin/stdout mode, omit `%s` and any \" brackets\n\
\n",
            app_name, app_name, app_name, app_name, app_name
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
  STEPGEN examples:\n\
\n\
    %s \"stepgen_pin_setup(0,0,PA,12,0)\"       # use PA12 as STEP pin \n\
    %s \"stepgen_pin_setup(0,1,PA,11,0)\"       # use PA11 as DIR pin \n\
    %s \"stepgen_time_setup(0,0,9000,1000)\"    # STEP frequency = 100KHz \n\
    %s \"stepgen_time_setup(0,1,50000,50000)\"  # DIR period = 100us \n\
    %s \"stepgen_task_add(0,100)\"              # make 100 forward pulses \n\
    %s \"stepgen_task_add(0,-34)\"              # make 34 reverse pulses \n\
    %s \"stepgen_pos_get(0)\"                   # current position is 66 \n\
    %s \"stepgen_pos_set(0,777)\"               # set position to 777 \n\
    %s \"stepgen_pos_get(0)\"                   # current position is 777 \n\
\n\
  ENCODER examples:\n\
\n\
    %s \"encoder_pin_setup(0,PH_A,PA,3)\"   # use PA3 pin as phase A input \n\
    %s \"encoder_pin_setup(0,PH_B,PA,5)\"   # use PA5 pin as phase B input \n\
    %s \"encoder_setup(0,1,0)\"             # use phase B, don't use phase Z\n\
    %s \"encoder_counts_set(0,0)\"          # reset channel 0 counts value \n\
    %s \"encoder_state_set(0,1)\"           # start channel 0\n\
    %s \"encoder_counts_get(0)\"            # get channel 0 counts value \n\
    %s \"encoder_state_get(0)\"             # get channel 0 state \n\
\n\
  NOTE:\n\
    If you are using stdin/stdout mode, omit `%s` and any \" brackets\n\
\n",
            app_name, app_name, app_name, app_name, app_name, app_name, app_name,
            app_name, app_name, app_name, app_name, app_name, app_name, app_name,
            app_name, app_name, app_name, app_name, app_name, app_name, app_name,
            app_name, app_name, app_name
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

    if ( !reg_match(str, "gpio_all_set *\\("UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 8) )
    {
#if !TEST
        gpio_all_set(&arg[0]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_all_clear *\\("UINT","UINT","UINT","UINT","UINT","UINT","UINT","UINT"\\)", &arg[0], 8) )
    {
#if !TEST
        gpio_all_clear(&arg[0]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "gpio_all_get *\\(\\)", &arg[0], 0) )
    {
#if !TEST
        uint32_t* ports = gpio_all_get();
#else
        uint32_t test[GPIO_PORTS_CNT] = {0x12345678};
        uint32_t* ports = &test[0];
#endif
        uint32_t b, port, s;
        for ( port = 0; port < GPIO_PORTS_CNT; port++ )
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
#if !TEST
        stepgen_pin_setup(arg[0], arg[1], arg[2], arg[3], arg[4]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "stepgen_task_add *\\("UINT","INT"\\)", &arg[0], 2) )
    {
#if !TEST
        stepgen_task_add(arg[0], arg[1]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "stepgen_time_setup *\\("UINT","UINT","UINT","UINT"\\)", &arg[0], 4) )
    {
#if !TEST
        stepgen_time_setup(arg[0], arg[1], arg[2], arg[3]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "stepgen_pos_get *\\("UINT"\\)", &arg[0], 1) )
    {
#if !TEST
        printf("%d\n", stepgen_pos_get(arg[0]));
#else
        printf("%d\n", 1);
#endif
        return 0;
    }

    if ( !reg_match(str, "stepgen_pos_set *\\("UINT","INT"\\)", &arg[0], 2) )
    {
#if !TEST
        stepgen_pos_set(arg[0], (int32_t)arg[1]);
#endif
        printf("OK\n");
        return 0;
    }
#if STEPGEN_DEBUG
    if ( !reg_match(str, "stepgen_param_get *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
#if !TEST
        printf("%d\n", stepgen_param_get(arg[0],arg[1]));
#else
        printf("%d\n", 1);
#endif
        return 0;
    }

    if ( !reg_match(str, "stepgen_param_set *\\("UINT","UINT","INT"\\)", &arg[0], 3) )
    {
#if !TEST
        stepgen_param_set(arg[0], arg[1], (int32_t)arg[2]);
#endif
        printf("OK\n");
        return 0;
    }
#endif

    // --- ENCODER ------

    if ( !reg_match(str, "encoder_pin_setup *\\("UINT","UINT","UINT","UINT"\\)", &arg[0], 4) )
    {
#if !TEST
        encoder_pin_setup(arg[0], arg[1], arg[2], arg[3]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "encoder_setup *\\("UINT","UINT","UINT"\\)", &arg[0], 3) )
    {
#if !TEST
        encoder_setup(arg[0], arg[1], arg[2]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "encoder_state_set *\\("UINT","UINT"\\)", &arg[0], 2) )
    {
#if !TEST
        encoder_state_set(arg[0], arg[1]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "encoder_counts_set *\\("UINT","INT"\\)", &arg[0], 2) )
    {
#if !TEST
        encoder_counts_set(arg[0], (int32_t)arg[1]);
#endif
        printf("OK\n");
        return 0;
    }

    if ( !reg_match(str, "encoder_state_get *\\("UINT"\\)", &arg[0], 1) )
    {
#if !TEST
        printf("%u\n", encoder_state_get(arg[0]));
#else
        printf("%i\n", 1);
#endif
        return 0;
    }

    if ( !reg_match(str, "encoder_counts_get *\\("UINT"\\)", &arg[0], 1) )
    {
#if !TEST
        printf("%i\n", encoder_counts_get(arg[0]));
#else
        printf("%i\n", -1);
#endif
        return 0;
    }

    printf("Unknown command! Type `help` \n");

    #undef UINT
    #undef INT

    return 1;
}
