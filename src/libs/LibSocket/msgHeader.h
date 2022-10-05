#ifndef __MSG_HEADER_H__
#define __MSG_HEADER_H__

#include "generaldef.h"
#include <stddef.h>

class header : public boost::enable_shared_from_this <header>
{
public:
    header();
    ~header();

    uint8_t getSyncID();
    void setSyncID(uint8_t sync);

    uint8_t getVersion();
    void setVersion(uint8_t ver);

    uint16_t getNumber();
    void setNumber(uint16_t num);

    uint32_t getLength();
    void setLength(uint32_t len);

    uint16_t getType();
    void setType(uint16_t type);

    ProtocolHeader *getProtocolHeader();
    void setProtocolHeader(ProtocolHeader procHead);

    void parseHeader(uint8_t *msg);
    uint8_t *getUint8Header();

    uint8_t *convertStructToUint8();

protected:
    uint8_t *m_msgUINT8;
    ProtocolHeader *m_head;
};


#endif
