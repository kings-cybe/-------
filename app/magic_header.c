#include <stdbool.h>
#include <stdint.h>
#include "crc32.h"
#include "utils.h"
#include "magic_header.h"

#define MAGIC_HEADER_MAGIC 0x4D414749 // "MAGI"
#define MAGIC_HEADER_ADDR  0x0800C000


typedef struct
{
    uint32_t magic;         // ħ�������ڱ�ʶ����һ����Ч��ħ��ͷ
    uint32_t bitmask;       // λ���룬���ڱ�ʶ��Щ�ֶ���Ч
    uint32_t reserved1[6];  // �����ֶΣ���������չʹ��

    uint32_t data_type;     // ���ͣ�����type����ѡ��̼�����λ��
    uint32_t data_offset;   // �̼��ļ������magic header��ƫ��
    uint32_t data_address;  // �̼�д���ʵ�ʵ�ַ
    uint32_t data_length;   // �̼�����
    uint32_t data_crc32;    // �̼���CRC32У��ֵ
    uint32_t reserved2[11]; // �����ֶΣ���������չʹ��

    char version[128];      // �̼��汾�ַ���

    uint32_t reserved3[6];  // �����ֶΣ���������չʹ��
    uint32_t this_address;  // �ýṹ���ڴ洢�����е�ʵ�ʵ�ַ
    uint32_t this_crc32;    // �ýṹ�屾���CRC32У��ֵ
} magic_header_t;

bool magic_header_validate(void)
{
    magic_header_t *header = (magic_header_t *)MAGIC_HEADER_ADDR;

    if (header->magic != MAGIC_HEADER_MAGIC)
        return false;

    uint32_t ccrc = crc32((uint8_t *)header, offset_of(magic_header_t, this_crc32));
    if (ccrc != header->this_crc32)
        return false;

    return true;
}

magic_header_type_t magic_header_get_type(void)
{
    magic_header_t *header = (magic_header_t *)MAGIC_HEADER_ADDR;
    return (magic_header_type_t)header->data_type;
}

uint32_t magic_header_get_offset(void)
{
    magic_header_t *header = (magic_header_t *)MAGIC_HEADER_ADDR;
    return header->data_offset;
}

uint32_t magic_header_get_address(void)
{
    magic_header_t *header = (magic_header_t *)MAGIC_HEADER_ADDR;
    return header->data_address;
}

uint32_t magic_header_get_length(void)
{
    magic_header_t *header = (magic_header_t *)MAGIC_HEADER_ADDR;
    return header->data_length;
}

uint32_t magic_header_get_crc32(void)
{
    magic_header_t *header = (magic_header_t *)MAGIC_HEADER_ADDR;
    return header->data_crc32;
}
