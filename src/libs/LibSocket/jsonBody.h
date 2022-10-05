#ifndef __JSON_BODY_H__
#define __JSON_BODY_H__

#include "JSON/json.h"
#include <string>
#include <vector>
#include "msgHeader.h"

class jsonBody : public boost::enable_shared_from_this <jsonBody>
{
public:
    jsonBody();
    virtual ~jsonBody();

    template<typename T>
    void jsonAppendElement(std::string name, T t)
    {
        m_jsVal[name.c_str()] = t;
    }

    bool fromJsonStringToJsonVal(uint8_t *msg, int length);

    std::string getJsonString();

    uint16_t getMsgType();
    void setMsgType(uint16_t type);

    uint16_t getMsgNumber();
    void setMsgNumber(uint16_t type);

    Json::Value getJsonVal();

protected:
    Json::Value m_jsVal;
    std::vector<std::string> m_elementName;
    uint16_t m_msgType;
    uint16_t m_number;
};

struct roboKitMsg
{
    header msgHeader;
    jsonBody msgBody;
};

typedef boost::function<void(boost::shared_ptr<roboKitMsg>)> JsonMsgCallBack;


#endif
