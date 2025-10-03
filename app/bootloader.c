#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "bl_usart.h"
#include "ringbuffer.h"
#include "crc16.h"
#include "crc32.h"
#include "tim_delay.h"
#include "stm32_flash.h"
#include "board.h"
#include "magic_header.h"

#define BL_VERSION      "0.0.1"
#define BL_ADDRESS      0x08000000
#define BL_SIZE         (48 * 1024)
#define BOOT_DELAY      3000
#define APP_VTOR_ADDR   0x08010000
#define RX_BUFFER_SIZE  (5 * 1024)
#define RX_TIMEOUT_MS   20
#define PAYLOAD_SIZE_MAX (4096 + 8) // 4096为Program最大数据长度，8为Program指令的地址(4)和长度(4)
#define PACKET_SIZE_MAX (4 + PAYLOAD_SIZE_MAX + 2) // header(1) + opcode(1) + length(2) + payload + crc16(2)

typedef enum
{
    PACKET_STATE_HEADER,
    PACKET_STATE_OPCODE,
    PACKET_STATE_LENGTH,
    PACKET_STATE_PAYLOAD,
    PACKET_STATE_CRC16,
} packet_state_machine_t;

typedef enum
{
    PACKET_OPCODE_INQUERY = 0x01,
    PACKET_OPCODE_ERASE = 0x81,
    PACKET_OPCODE_PROGRAM = 0x82,
    PACKET_OPCODE_VERIFY  = 0x83,
    PACKET_OPCODE_RESET = 0x21,
    PACKET_OPCODE_BOOT = 0x22,
} packet_opcode_t;

typedef enum
{
    INQUERY_SUBCODE_VERSION = 0x00,
    INQUERY_SUBCODE_MTU = 0x01,
} inquery_subcode_t;

typedef enum
{
    PACKET_ERRCODE_OK = 0,
    PACKET_ERRCODE_OPCODE,
    PACKET_ERRCODE_OVERFLOW,
    PACKET_ERRCODE_TIMEOUT,
    PACKET_ERRCODE_FORMAT,
    PACKET_ERRCODE_VERIFY,
    PACKET_ERRCODE_PARAM,
    PACKET_ERRCODE_UNKNOWN = 0xff,
} packet_errcode_t;

static uint8_t rb_buffer[RX_BUFFER_SIZE];
static rb_t rxrb;

static uint8_t packet_buffer[PACKET_SIZE_MAX];
static uint32_t packet_index;
static packet_state_machine_t packet_state = PACKET_STATE_HEADER;
static packet_opcode_t packet_opcode;
static uint16_t packet_payload_length;

static bool application_validate(void)
{
    if (!magic_header_validate())
    {
        printf("magic header invalid\n");
        return false;
    }

    uint32_t addr = magic_header_get_address();
    uint32_t size = magic_header_get_length();
    uint32_t crc = magic_header_get_crc32();
    uint32_t ccrc = crc32((uint8_t *)addr, size);
    if (crc != ccrc)
    {
        printf("application crc error: expected %08X, got %08X\n", crc, ccrc);
        return false;
    }

    return true;
}

static void boot_application(void)
{
    if (!application_validate())
    {
        printf("application validate failed, cannot boot\n");
        return;
    }

    printf("booting application...\n");
    tim_delay_ms(2);

    led_off(led1);
    TIM_DeInit(TIM6);
    USART_DeInit(USART1);
    USART_DeInit(USART3);
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    NVIC_DisableIRQ(USART1_IRQn);
    NVIC_DisableIRQ(USART3_IRQn);

    SCB->VTOR = APP_VTOR_ADDR;
    extern void JumpApp(uint32_t base);
    JumpApp(APP_VTOR_ADDR);
}

static void bl_response(packet_opcode_t opcode, packet_errcode_t errcode, const uint8_t *data, uint16_t length)
{
    uint8_t *response = packet_buffer;
    response[0] = 0x55;
    response[1] = opcode;
    response[2] = errcode;
    response[3] = (uint8_t)(length & 0xFF);
    response[4] = (uint8_t)((length >> 8) & 0xFF);
    if (length > 0) memcpy(&response[5], data, length);
    uint16_t crc = crc16(response, 5 + length);
    response[5 + length] = (uint8_t)(crc & 0xFF);
    response[6 + length] = (uint8_t)((crc >> 8) & 0xFF);

    bl_usart_write(response, 7 + length);
}

//static inline void bl_response_ack(packet_opcode_t opcode, packet_errcode_t errcode)
//{
//    bl_response(opcode, errcode, NULL, 0);
//}

static void bl_opcode_inquery_handler(void)
{
    printf("inquery handler\n");

    if (packet_payload_length != 1)
    {
        printf("inquery packet length error\n");
        return;
    }

    uint8_t subcode = packet_buffer[4];

    switch (subcode)
    {
        case INQUERY_SUBCODE_VERSION:
        {
            bl_response(PACKET_OPCODE_INQUERY, PACKET_ERRCODE_OK, (const uint8_t *)BL_VERSION, strlen(BL_VERSION));
            break;
        }
        case INQUERY_SUBCODE_MTU:
        {
            uint8_t bmtu[2] = {PAYLOAD_SIZE_MAX & 0xFF, (PAYLOAD_SIZE_MAX >> 8) & 0xFF};
            bl_response(PACKET_OPCODE_INQUERY, PACKET_ERRCODE_OK, (const uint8_t *)&bmtu, sizeof(bmtu));
            break;
        }
        default:
        {
            printf("unknown inquery subcode: %02X\n", subcode);
            break;
        }
    }
}

static void bl_opcode_erase_handler(void)
{
    printf("erase handler\n");

    if (packet_payload_length != 8)
    {
        printf("erase packet length error: %d\n", packet_payload_length);
        bl_response(PACKET_OPCODE_ERASE, PACKET_ERRCODE_PARAM, NULL, 0);
        return;
    }

    uint32_t address = (packet_buffer[7] << 24) | (packet_buffer[6] << 16) | (packet_buffer[5] << 8) | packet_buffer[4];
    uint32_t size = (packet_buffer[11] << 24) | (packet_buffer[10] << 16) | (packet_buffer[9] << 8) | packet_buffer[8];

    if (address < STM32_FLASH_BASE || address + size > STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
        printf("erase address=0x%08X, size=%u out of range\n", address, size);
        bl_response(PACKET_OPCODE_ERASE, PACKET_ERRCODE_PARAM, NULL, 0);
        return;
    }

    if (address >= BL_ADDRESS && address < BL_ADDRESS + BL_SIZE)
    {
        printf("address 0x%08X is protected\n", address);
        bl_response(PACKET_OPCODE_ERASE, PACKET_ERRCODE_PARAM, NULL, 0);
        return;
    }

    printf("erase address=0x%08X, size=%u\n", address, size);

    stm32_flash_unlock();
    stm32_flash_erase(address, size);
    stm32_flash_lock();

    bl_response(PACKET_OPCODE_ERASE, PACKET_ERRCODE_OK, NULL, 0);
}

static void bl_opcode_program_handler(void)
{
    printf("program handler\n");

    if (packet_payload_length <= 8)
    {
        printf("program packet length error: %d\n", packet_payload_length);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_PARAM, NULL, 0);
        return;
    }

    uint32_t address = (packet_buffer[7] << 24) | (packet_buffer[6] << 16) | (packet_buffer[5] << 8) | packet_buffer[4];
    uint32_t size = (packet_buffer[11] << 24) | (packet_buffer[10] << 16) | (packet_buffer[9] << 8) | packet_buffer[8];
    uint8_t *data = &packet_buffer[12];

    if (address < STM32_FLASH_BASE || address + size > STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
        printf("program address=0x%08X, size=%u out of range\n", address, size);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_PARAM, NULL, 0);
        return;
    }

    if (address >= BL_ADDRESS && address < BL_ADDRESS + BL_SIZE)
    {
        printf("address 0x%08X is protected\n", address);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_PARAM, NULL, 0);
        return;
    }

    if (size != packet_payload_length - 8)
    {
        printf("program size %u does not match payload length %u\n", size, packet_payload_length - 8);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_PARAM, NULL, 0);
        return;
    }

    printf("program address=0x%08X, size=%u\n", address, size);

    stm32_flash_unlock();
    stm32_flash_program(address, data, size);
    stm32_flash_lock();

    bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_OK, NULL, 0);
}

static void bl_opcode_verify_handler(void)
{
    printf("verify handler\n");

    if (packet_payload_length != 12)
    {
        printf("verify packet length error: %d\n", packet_payload_length);
        bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_PARAM, NULL, 0);
        return;
    }

    uint32_t address = (packet_buffer[7] << 24) | (packet_buffer[6] << 16) | (packet_buffer[5] << 8) | packet_buffer[4];
    uint32_t size = (packet_buffer[11] << 24) | (packet_buffer[10] << 16) | (packet_buffer[9] << 8) | packet_buffer[8];
    uint32_t crc = (packet_buffer[15] << 24) | (packet_buffer[14] << 16) | (packet_buffer[13] << 8) | packet_buffer[12];

    if (address < STM32_FLASH_BASE || address + size > STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
        printf("verify address=0x%08X, size=%u out of range\n", address, size);
        bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_PARAM, NULL, 0);
        return;
    }

    printf("verify address=0x%08X, size=%u, crc=0x%08X\n", address, size, crc);

    uint32_t ccrc = crc32((uint8_t *)address, size);

    if (ccrc != crc)
    {
        printf("verify failed: expected 0x%08X, got 0x%08X\n", crc, ccrc);
        bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_VERIFY, NULL, 0);
        return;
    }

    bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_OK, NULL, 0);
}

static void bl_opcode_reset_handler(void)
{
    printf("reset handler\n");
    bl_response(PACKET_OPCODE_RESET, PACKET_ERRCODE_OK, NULL, 0);
    printf("system resetting...\n");
    tim_delay_ms(2);

    NVIC_SystemReset();
}

static void bl_opcode_boot_handler(void)
{
    printf("boot handler\n");
    bl_response(PACKET_OPCODE_BOOT, PACKET_ERRCODE_OK, NULL, 0);

    boot_application();
}

static void bl_packet_handler(void)
{
    switch (packet_opcode)
    {
        case PACKET_OPCODE_INQUERY:
            bl_opcode_inquery_handler();
            break;
        case PACKET_OPCODE_ERASE:
            bl_opcode_erase_handler();
            break;
        case PACKET_OPCODE_PROGRAM:
            bl_opcode_program_handler();
            break;
        case PACKET_OPCODE_VERIFY:
            bl_opcode_verify_handler();
            break;
        case PACKET_OPCODE_RESET:
            bl_opcode_reset_handler();
            break;
        case PACKET_OPCODE_BOOT:
            bl_opcode_boot_handler();
            break;
        default:
            // 未知指令
            printf("Unknown command: %02X\n", packet_opcode);
            break;
    }
}

static bool bl_byte_handler(uint8_t byte)
{
    bool full_packet = false;

    // 处理字节数据超时接收
    static uint64_t last_byte_ms;
    uint64_t now_ms = tim_get_ms();
    if (now_ms - last_byte_ms > RX_TIMEOUT_MS)
    {
        if (packet_state != PACKET_STATE_HEADER)
            printf("last packet rx timeout\n");
        packet_index = 0;
        packet_state = PACKET_STATE_HEADER;
    }
    last_byte_ms = now_ms;

    // printf("recv: %02X\n", byte);

    // 字节接收状态机处理
    packet_buffer[packet_index++] = byte;
    switch (packet_state)
    {
        case PACKET_STATE_HEADER:
            if (packet_buffer[0] == 0xAA)
            {
                printf("header ok\n");
                packet_state = PACKET_STATE_OPCODE;
            }
            else
            {
                printf("header error: %02X\n", packet_buffer[0]);
                packet_index = 0;
                packet_state = PACKET_STATE_HEADER;
            }
            break;
        case PACKET_STATE_OPCODE:
            if (packet_buffer[1] == PACKET_OPCODE_INQUERY ||
                packet_buffer[1] == PACKET_OPCODE_ERASE ||
                packet_buffer[1] == PACKET_OPCODE_PROGRAM ||
                packet_buffer[1] == PACKET_OPCODE_VERIFY ||
                packet_buffer[1] == PACKET_OPCODE_RESET ||
                packet_buffer[1] == PACKET_OPCODE_BOOT)
            {
                printf("opcode ok: %02X\n", packet_buffer[1]);
                packet_opcode = (packet_opcode_t)packet_buffer[1];
                packet_state = PACKET_STATE_LENGTH;
            }
            else
            {
                printf("opcode error: %02X\n", packet_buffer[1]);
                packet_index = 0;
                packet_state = PACKET_STATE_HEADER;
            }
            break;
        case PACKET_STATE_LENGTH:
            if (packet_index == 4)
            {
                uint16_t payload_length = (packet_buffer[3] << 8) | packet_buffer[2];
                if (payload_length <= PAYLOAD_SIZE_MAX)
                {
                    printf("length ok: %u\n", payload_length);
                    packet_payload_length = payload_length;
                    if (packet_payload_length > 0)
                        packet_state = PACKET_STATE_PAYLOAD;
                    else
                        packet_state = PACKET_STATE_CRC16;
                }
                else
                {
                    printf("length error: %u\n", payload_length);
                    packet_index = 0;
                    packet_state = PACKET_STATE_HEADER;
                }
            }
            break;
        case PACKET_STATE_PAYLOAD:
            if (packet_index == 4 + packet_payload_length)
            {
                printf("payload receive ok\n");
                packet_state = PACKET_STATE_CRC16;
            }
            break;
        case PACKET_STATE_CRC16:
            if (packet_index == 4 + packet_payload_length + 2)
            {
                uint16_t crc = (packet_buffer[4 + packet_payload_length + 1] << 8) |
                                packet_buffer[4 + packet_payload_length];
                uint16_t ccrc = crc16(packet_buffer, 4 + packet_payload_length);
                if (crc == ccrc)
                {
                    full_packet = true;
                    printf("crc16 ok: %04X\n", crc);
                    printf("packet received: opcode=%02X, length=%u\n", packet_opcode, packet_payload_length);
                    // printf("payload: ");
                    // for (uint32_t i = 0; i < packet_payload_length; i++)
                    // {
                    //     printf("%02X ", packet_buffer[4 + i]);
                    // }
                    printf("\n");
                }
                else
                {
                    printf("crc16 error: expected %04X, got %04X\n", crc, ccrc);
                }

                packet_index = 0;
                packet_state = PACKET_STATE_HEADER;
            }
            break;
        default:
            break;
    }

    return full_packet;
}

static void bl_usart_rx_handler(const uint8_t *data, uint32_t length)
{
    rb_puts(rxrb, data, length);
}

static bool key_trap_check(void)
{
    for (uint32_t t = 0; t < BOOT_DELAY; t+=10)
    {
        tim_delay_ms(10);
        if (!key_read(key2))
            return false;
    }
    printf("key pressed, trap into boot\n");
    return true;
}

static void wait_key_release(void)
{
    while (key_read(key2))
        tim_delay_ms(10);
}

static bool key_press_check(void)
{
    if (!key_read(key2))
        return false;

    tim_delay_ms(10);
    if (!key_read(key2))
        return false;

    return true;
}

bool magic_header_trap_boot(void)
{
    if (!magic_header_validate())
    {
        printf("magic header invalid, trap into boot\n");
        return true;
    }

    if (!application_validate())
    {
        printf("application validate failed, trap into boot\n");
        return true;
    }

    return false;
}

void bootloader_main(void)
{
    printf("Bootloader started.\n");

    rxrb = rb_new(rb_buffer, RX_BUFFER_SIZE);
    bl_usart_init();
    bl_usart_register_rx_callback(bl_usart_rx_handler);

    key_init(key2);

    bool trapboot = key_trap_check();
    if (!trapboot)
        trapboot = magic_header_trap_boot();

    if (!trapboot)
    {
        boot_application();
    }

    led_init(led1);
    led_on(led1);
    wait_key_release();

    while (1)
    {
        if (key_press_check())
        {
            printf("key pressed, rebooting...\n");
            tim_delay_ms(2);
            NVIC_SystemReset();
        }

        if (!rb_empty(rxrb))
        {
            uint8_t byte;
            rb_get(rxrb, &byte);
            if (bl_byte_handler(byte))
            {
                bl_packet_handler();
            }
        }
    }
}
