#pragma once
#ifndef __COMMON_MSG_HEADER_H__
#define __COMMON_MSG_HEADER_H__

#define MSG_HEADER_BEGIN_LENGTH 2
#define MSG_HEADER_MSG_HEADER_LENGTH 4
#define MSG_HEADER_MSG_BODY_LENGTH 4
#define MSG_HEADER_END_LENGTH 2

#define MSG_HEADER_BEGIN_CHAR_1ST 0x52
#define MSG_HEADER_BEGIN_CHAR_2ND 0x54

#define MSG_HEADER_END_CHAR_1ST 0x0a
#define MSG_HEADER_END_CHAR_2ND 0x0d

#define MSG_HEADER_TOTAL_LENGTH MSG_HEADER_BEGIN_LENGTH + MSG_HEADER_MSG_HEADER_LENGTH + MSG_HEADER_MSG_BODY_LENGTH + MSG_HEADER_END_LENGTH

#include "JSON/json.h"
#include <stddef.h>
#include <boost/function.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include "common/DLHangRailSocketCommonDef.h"
#include <QUuid>
#include "LibSocket/generaldef.h"

struct protoHeader
{
    protoHeader()
    {
        beginChar[0] = 0x52;
        beginChar[1] = 0x54;
        msgHeaderLength = 0;
        msgBodyLength = 0;
        endChar[0] = 0x0a;
        endChar[1] = 0x0d;
    }
    uint8_t beginChar[2];
    uint32_t msgHeaderLength;
    uint32_t msgBodyLength;
    uint8_t endChar[2];

    void operator= (protoHeader header)
    {
        this->beginChar[0] = header.beginChar[0];
        this->beginChar[1] = header.beginChar[1];
        this->msgHeaderLength = header.msgHeaderLength;
        this->msgBodyLength = header.msgBodyLength;
        this->endChar[0] = header.endChar[0];
        this->endChar[1] = header.endChar[1];
    }
};

struct MsgHeader
{
    std::string ssid;
    std::string msgid;
    int msgtypeid;

    void operator=(MsgHeader header)
    {
        this->ssid = header.ssid;
        this->msgid = header.msgid;
        this->msgtypeid = header.msgtypeid;
    }
};

class commonMsg
    : public boost::enable_shared_from_this <commonMsg>
{
public:
    commonMsg();
    ~commonMsg();

    uint32_t commonMsg::uint8_t_to_uint32(uint8_t *b)
    {
        uint32_t ret;
        ret = (uint32_t)(ByteCast(b[3]));
        ret |= (uint32_t)(ByteCast(b[2])) << 8;
        ret |= (uint32_t)(ByteCast(b[1])) << 8 * 2;
        ret |= (uint32_t)(ByteCast(b[0])) << 8 * 3;
        return ret;
    }


    void commonMsg::uint32_t_to_uint8(uint32_t b, uint8_t *p)
    {
        uint8_t ret[4];
        ret[3] = (b & 0x000000ff);
        ret[2] = (b & 0x0000ff00) >> 8;
        ret[1] = (b & 0x00ff0000) >> 16;
        ret[0] = (b & 0xff000000) >> 24;
        memcpy(p, ret, 4);
    }

    uint16_t commonMsg::uint8_t_to_uint16(uint8_t *b)
    {
        uint16_t ret;
        ret = (uint16_t)(ByteCast(b[1]));
        ret |= (uint16_t)(ByteCast(b[0])) << 8;
        return ret;
    }

    void commonMsg::uint16_t_to_uint8(uint16_t b, uint8_t *p)
    {
        uint8_t ret[2];
        ret[1] = (b & 0x00ff);
        ret[0] = (b & 0xff00) >> 8;
        memcpy(p, ret, 2);
    }

    void setProtocolHeader(protoHeader header);
    protoHeader getProtocolHeader();
    unsigned char *getProtocolHeaderOrigin();

    int getMsgLeft2ReadLength();
    int getMsgTotalLength();

    void setMsgTypeId(int msgTypeId);
    void setMsgSsid(std::string ssid);
    void setMsgId(std::string msgId);

    std::string getMsgSsid();

    void setMsgHeaderElement(MsgHeader msgHeader);

    template<typename T>
    void AppendMsgBodyElement(std::string name, T t)
    {
        m_jsBodyVal[name.c_str()] = t;
    }

    void parseProtocolHeader(uint8_t *msg);

    int getMsgHeaderLength();
    int getMsgBodyLength();

    void fromUint8ToValHeader(uint8_t *msg, int length);
    MsgHeader getMsgHeader();

    void fromUint8ToValBody(uint8_t *msg, int length);
    void fromStdStringToValBody(std::string msg);

    std::string getHeaderJsonString();
    std::string getBodyJsonString();

    Json::Value getBodyJsonVal();

    void generateMsg();

private:
    uint8_t * m_pHeaderOrigin;
    protoHeader m_pHeader;
    MsgHeader m_msgHeader;
    Json::Value m_jsHeaderVal;
    Json::Value m_jsBodyVal;
};

typedef boost::function<void(boost::shared_ptr<commonMsg>) > commonMsgCallBack;

#endif
