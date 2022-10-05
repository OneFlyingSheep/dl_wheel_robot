#include "msgHeader.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>

header::header()
{
    m_msgUINT8 = new uint8_t [MSG_HEADER_LENGTH];;
    memset(m_msgUINT8, 0, MSG_HEADER_LENGTH);
    m_head = new ProtocolHeader;
    memset(m_head, 0, sizeof(ProtocolHeader));

    setSyncID(0x5A);
    setVersion(1);
}

header::~header()
{
    if (NULL != m_msgUINT8)
    {
        delete [] m_msgUINT8;
        m_msgUINT8 = NULL;
    }

    if (NULL != m_head)
    {
        delete m_head;
        m_head = NULL;
    }
}

void header::parseHeader(uint8_t *msg)
{
//    char ss[128];
//    int cur___ = 0;
//    for(int i = 0; i < MSG_HEADER_LENGTH; i++)
//        cur___ += sprintf(ss + cur___, "%02X ", msg[i]);
//    printf("msg:%s\n", ss);
    if (m_msgUINT8 == NULL)
    {
        m_msgUINT8 = new uint8_t[MSG_HEADER_LENGTH];
    }

    memcpy(m_msgUINT8, msg, MSG_HEADER_LENGTH);
    

    uint8_t *p = m_msgUINT8;

    if (m_head == NULL)
    {
        m_head = new ProtocolHeader;
        memset(m_head, 0, sizeof(ProtocolHeader));
    }    

    m_head->m_sync = *p;
    p++;
    m_head->m_version = *p;
    p++;
    m_head->m_number = uint8_t_to_uint16(p);
    p += 2;
    m_head->m_length = uint8_t_to_uint32(p);
    p += 4;
    m_head->m_type = uint8_t_to_uint16(p);
    p += 2;
    memcpy(m_head->m_reserved, p, sizeof(m_head->m_reserved));
}

uint8_t *header::getUint8Header()
{
    return m_msgUINT8;
}

uint8_t *header::convertStructToUint8()
{
    uint8_t *p = m_msgUINT8;

//    ROS_INFO("number:%d, length:%d, type:%d", m_head->m_number, m_head->m_length, m_head->m_type);

    //char ss[128];
    //int cur___ = 0;
    //for(int i = 0; i < MSG_HEADER_LENGTH; i++)
    //    cur___ += sprintf(ss + cur___, "%02X ", m_msgUINT8[i]);
    //printf("msg-convert:%s\n", ss);

    memcpy(p, (void *)&m_head->m_sync, 1);
    p++;

    memcpy(p, (void *)&m_head->m_version, 1);
    p++;

//    p = uint16_t_to_uint8(&m_head->m_number);
    //memcpy(p, (void *)uint16_t_to_uint8(m_head->m_number), 2);
    uint16_t_to_uint8(m_head->m_number, p);
    p += 2;

//    p = uint32_t_to_uint8(&m_head->m_length);
    //memcpy(p, (void *)uint32_t_to_uint8(m_head->m_length), 4);
    uint32_t_to_uint8(m_head->m_length, p);
    p += 4;

    //memcpy(p, (void *)uint16_t_to_uint8(m_head->m_type), 2);
//    p = uint16_t_to_uint8(&m_head->m_type);
    uint16_t_to_uint8(m_head->m_type, p);
    p += 2;

    memcpy(p, m_head->m_reserved, sizeof(m_head->m_reserved));

    //char ss[128];
    //int cur___ = 0;
    //for (int i = 0; i < MSG_HEADER_LENGTH; i++)
    //    cur___ += sprintf(ss + cur___, "%02X ", m_msgUINT8[i]);
    //printf("msg-convert:%s\n", ss);

    return m_msgUINT8;
}

uint8_t header::getSyncID()
{
    return m_head->m_sync;
}

void header::setSyncID(uint8_t sync)
{
    m_head->m_sync = sync;
}

uint8_t header::getVersion()
{
    return m_head->m_version;
}

void header::setVersion(uint8_t ver)
{
    m_head->m_version = ver;
}

uint16_t header::getNumber()
{
    return m_head->m_number;
}

void header::setNumber(uint16_t num)
{
    m_head->m_number = num;
}

uint32_t header::getLength()
{
    return m_head->m_length;
}

void header::setLength(uint32_t len)
{
    m_head->m_length = len;
}

uint16_t header::getType()
{
    return m_head->m_type;
}

void header::setType(uint16_t type)
{
    m_head->m_type = type;
}

ProtocolHeader *header::getProtocolHeader()
{
    return m_head;
}

void header::setProtocolHeader(ProtocolHeader procHead)
{
    memcpy(m_head, &procHead, sizeof(ProtocolHeader));
}
