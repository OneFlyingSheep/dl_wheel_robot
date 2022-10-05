#include "generaldef.h"

uint16_t uint8_t_to_uint16(uint8_t *b)
{
    uint16_t ret;
    ret = (uint16_t)(ByteCast(b[1]));
    ret |= (uint16_t)(ByteCast(b[0])) << 8;
    return ret;
}

void uint16_t_to_uint8(uint16_t b, uint8_t *p)
{
    uint8_t ret[2];
    ret[1] = (b & 0x00ff);
    ret[0] = (b & 0xff00) >> 8;
    memcpy(p, ret, 2);
}


uint32_t uint8_t_to_uint32(uint8_t *b)
{
    uint32_t ret;
    ret = (uint32_t)(ByteCast(b[3]));
    ret |= (uint32_t)(ByteCast(b[2])) << 8;
    ret |= (uint32_t)(ByteCast(b[1])) << 8 * 2;
    ret |= (uint32_t)(ByteCast(b[0])) << 8 * 3;
    return ret;
}


void uint32_t_to_uint8(uint32_t b, uint8_t *p)
{
    uint8_t ret[4];
    ret[3] = (b & 0x000000ff);
    ret[2] = (b & 0x0000ff00) >> 8;
    ret[1] = (b & 0x00ff0000) >> 16;
    ret[0] = (b & 0xff000000) >> 24;
    memcpy(p, ret, 4);
}