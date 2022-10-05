#include "commonMsg.h"

commonMsg::commonMsg()
{
    m_pHeaderOrigin = new uint8_t[MSG_HEADER_TOTAL_LENGTH];
    memset(m_pHeaderOrigin, 0, MSG_HEADER_TOTAL_LENGTH);
}

commonMsg::~commonMsg()
{
    if (NULL != m_pHeaderOrigin)
    {
        delete[] m_pHeaderOrigin;
        m_pHeaderOrigin = NULL;
    }
}

void commonMsg::setProtocolHeader(protoHeader header)
{
    m_pHeader = header;

    uint8_t *p = m_pHeaderOrigin;

    //    ROS_INFO("number:%d, length:%d, type:%d", m_head->m_number, m_head->m_length, m_head->m_type);

    //char ss[128];
    //int cur___ = 0;
    //for(int i = 0; i < MSG_HEADER_LENGTH; i++)
    //    cur___ += sprintf(ss + cur___, "%02X ", m_msgUINT8[i]);
    //printf("msg-convert:%s\n", ss);

    memcpy(p, (void *)&m_pHeader.beginChar[0], 1);
    p++;

    memcpy(p, (void *)&m_pHeader.beginChar[1], 1);
    p++;

    uint32_t_to_uint8(m_pHeader.msgHeaderLength, p);
    p += 4;

    uint32_t_to_uint8(m_pHeader.msgBodyLength, p);
    p += 4;

    memcpy(p, (void *)&m_pHeader.endChar[0], 1);
    p++;

    memcpy(p, (void *)&m_pHeader.endChar[1], 1);
    p++;
}


protoHeader commonMsg::getProtocolHeader()
{
    return m_pHeader;
}


unsigned char * commonMsg::getProtocolHeaderOrigin()
{
    return m_pHeaderOrigin;
}


int commonMsg::getMsgLeft2ReadLength()
{
    return m_pHeader.msgBodyLength + m_pHeader.msgHeaderLength;
}

int commonMsg::getMsgTotalLength()
{
    return getMsgHeaderLength() + getMsgBodyLength();
}

void commonMsg::setMsgTypeId(int msgTypeId)
{
    m_jsHeaderVal["msgtypeid"] = msgTypeId;
    m_msgHeader.msgtypeid = msgTypeId;
}

void commonMsg::setMsgSsid(std::string ssid)
{
    m_jsHeaderVal["ssid"] = ssid;
    m_msgHeader.ssid = ssid;
}

void commonMsg::setMsgId(std::string msgId)
{
    m_jsHeaderVal["msgid"] = msgId;
    m_msgHeader.msgid = msgId;
}


std::string commonMsg::getMsgSsid()
{
    return m_jsHeaderVal["ssid"].asString();
}

void commonMsg::setMsgHeaderElement(MsgHeader msgHeader)
{
    m_jsHeaderVal["ssid"] = msgHeader.ssid;
    m_jsHeaderVal["msgid"] = msgHeader.msgid;
    m_jsHeaderVal["msgtypeid"] = msgHeader.msgtypeid;
    m_msgHeader = msgHeader;
}

void commonMsg::parseProtocolHeader(uint8_t *msg)
{
    if (m_pHeaderOrigin == NULL)
    {
        m_pHeaderOrigin = new uint8_t[MSG_HEADER_TOTAL_LENGTH];
    }

    memcpy(m_pHeaderOrigin, msg, MSG_HEADER_TOTAL_LENGTH);
    
    uint8_t *p = m_pHeaderOrigin;

    m_pHeader.beginChar[0] = *p;
    p++;
    m_pHeader.beginChar[1] = *p;
    p++;
    m_pHeader.msgHeaderLength = uint8_t_to_uint32(p);
    p += MSG_HEADER_MSG_HEADER_LENGTH;
    m_pHeader.msgBodyLength = uint8_t_to_uint32(p);
    p += MSG_HEADER_MSG_BODY_LENGTH;
    m_pHeader.endChar[0] = uint8_t_to_uint16(p);
    p++;
    m_pHeader.endChar[1] = uint8_t_to_uint16(p);
}

int commonMsg::getMsgHeaderLength()
{
    Json::FastWriter writer;
    std::string str = writer.write(m_jsHeaderVal);
    return str.length();
}

int commonMsg::getMsgBodyLength()
{
    Json::FastWriter writer;
    std::string str = writer.write(m_jsBodyVal);
    return str.length();
}

void commonMsg::fromUint8ToValHeader(uint8_t *msg, int length)
{
    Json::Reader read;
    read.parse((char*)msg, m_jsHeaderVal);
    m_msgHeader.ssid = m_jsHeaderVal["ssid"].asString();
    m_msgHeader.msgid = m_jsHeaderVal["msgid"].asString();
    m_msgHeader.msgtypeid = m_jsHeaderVal["msgtypeid"].asInt();
}


MsgHeader commonMsg::getMsgHeader()
{
    return m_msgHeader;
}

void commonMsg::fromUint8ToValBody(uint8_t *msg, int length)
{
    Json::Reader read;
    read.parse((char*)msg, m_jsBodyVal);
}


void commonMsg::fromStdStringToValBody(std::string msg)
{
    Json::Reader read;
    read.parse(msg.c_str(), m_jsBodyVal);
}

std::string commonMsg::getHeaderJsonString()
{
    Json::FastWriter writer;
    return writer.write(m_jsHeaderVal);
}

std::string commonMsg::getBodyJsonString()
{
    Json::FastWriter writer;
    return writer.write(m_jsBodyVal);
}

Json::Value commonMsg::getBodyJsonVal()
{
    return m_jsBodyVal;
}

void commonMsg::generateMsg()
{
    m_pHeader.msgHeaderLength = getMsgHeaderLength();
    m_pHeader.msgBodyLength = getMsgBodyLength();
    
    setProtocolHeader(m_pHeader);
    
    return;
}
