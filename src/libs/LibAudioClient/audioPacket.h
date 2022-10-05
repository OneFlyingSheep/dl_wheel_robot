#ifndef __AUTIO_PACKET_H__
#define __AUTIO_PACKET_H__

#include <stdint.h>
#include <cstring>

class audioPacket
{
public:
    audioPacket(uint32_t t, uint32_t l, uint8_t *v) : type_(t), length_(l), format_data_(NULL), format_len_(0)
    {
        value_ = new uint8_t[l];
        memcpy(value_, v, l);
        format_len_ = sizeof(uint32_t) * 2 + l;
    }

    ~audioPacket()
    {
        if (NULL != value_)
        {
            delete[]value_;
            value_ = NULL;
        }

        if (NULL != format_data_)
        {
            delete[]format_data_;
            format_data_ = NULL;
        }
    }

    uint32_t getFormatLength()
    {
        return format_len_;
    }

    uint8_t *getFormatData()
    {
        if (format_len_ == 0)
        {
            return NULL;
        }

        if (format_data_ == NULL)
        {
            
            format_data_ = new uint8_t[format_len_];
            uint8_t *p = format_data_;
            memcpy(p, (uint8_t *)&type_, sizeof(uint32_t));
            p += sizeof(uint32_t);
            memcpy(p, (uint8_t *)&length_, sizeof(uint32_t));
            p += sizeof(uint32_t);
            memcpy(p, value_, length_);
        }

        return format_data_;
    }

private:    
    uint32_t type_;
    uint32_t length_;
    uint8_t *value_;

    uint8_t *format_data_;
    uint32_t format_len_;
};


#endif



