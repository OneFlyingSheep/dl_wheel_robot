#include "jsonBody.h"
#include "boost/atomic.hpp"

using namespace std;

jsonBody::jsonBody()
{
    m_msgType = -1;
}

jsonBody::~jsonBody()
{

//    Json::valueToString();
}

bool jsonBody::fromJsonStringToJsonVal(uint8_t *msg, int length)
{
//     char *str = new char[length];
//     memcpy(str, msg, length);
    Json::Reader read;
    if (read.parse((char*)msg, m_jsVal))
    {
        return true;
    }
    else
    {
        return false;
    }
}

string jsonBody::getJsonString()
{
    Json::FastWriter writer;
    return writer.write(m_jsVal);
}

uint16_t jsonBody::getMsgType()
{
    return m_msgType;
}


void jsonBody::setMsgType(uint16_t type)
{
    m_msgType = type;
}


uint16_t jsonBody::getMsgNumber()
{
    return m_number;
}


void jsonBody::setMsgNumber(uint16_t type)
{
    m_number = type;
}

Json::Value jsonBody::getJsonVal()
{
    return m_jsVal;
}
