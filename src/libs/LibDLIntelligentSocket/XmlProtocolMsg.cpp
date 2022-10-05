#include "XmlProtocolMsg.h"

XmlProtocolMsg::XmlProtocolMsg() : SendCode_(""), ReceiveCode_(""), Type_(-1), Code_(""), Command_(-1), Time_("")
{
    xmlItemData_ = NULL;
    xmlDoc_ = NULL;
    origMsg_ = NULL;
    xmlRobot_ = NULL;
}

XmlProtocolMsg::~XmlProtocolMsg()
{
    if (origMsg_)
    {
        delete origMsg_;
        origMsg_ = NULL;
    }

    if (xmlDoc_)
    {
        delete xmlDoc_;
        xmlDoc_ = NULL;
        xmlItemData_ = NULL;
    }
}

void XmlProtocolMsg::setOriginalMsg(uint8_t *buff, int length)
{
    origMsg_ = new OriginalBaseMsg;
    memcpy(origMsg_, buff, length);
}

void XmlProtocolMsg::setOriginalMsg(std::vector<unsigned char> buff)
{
    origMsg_ = new OriginalBaseMsg;

    uint8_t *p = &(*buff.begin());

    memcpy(origMsg_->start_flag_, p, sizeof(origMsg_->start_flag_));
    p += sizeof(origMsg_->start_flag_);

    memcpy(&origMsg_->sequence_id_, p, sizeof(origMsg_->sequence_id_));
    p += sizeof(origMsg_->sequence_id_);

    memcpy(&origMsg_->xml_length_, p, sizeof(origMsg_->xml_length_));
    p += sizeof(origMsg_->xml_length_);

    origMsg_->xml_string_ = std::string((char*)p, origMsg_->xml_length_);
    p += origMsg_->xml_length_;

    memcpy(&origMsg_->end_flag_, p, sizeof(origMsg_->end_flag_));
    p += sizeof(origMsg_->end_flag_);
}

OriginalBaseMsg *XmlProtocolMsg::getOriginalMsg()
{
    return origMsg_;
}

std::string XmlProtocolMsg::getString()
{
    return origMsg_->xml_string_;
}

std::string XmlProtocolMsg::getSendCode()
{
    return SendCode_;
}

void XmlProtocolMsg::setSendCode(std::string str)
{
    SendCode_ = str;
}

std::string XmlProtocolMsg::getReceiveCode()
{
    return ReceiveCode_;
}

void XmlProtocolMsg::setReceiveCode(std::string str)
{
    ReceiveCode_ = str;
}

int XmlProtocolMsg::getType()
{
    return Type_;
}

void XmlProtocolMsg::setType(int val)
{
    Type_ = val;
}

std::string XmlProtocolMsg::getCode()
{
    return Code_;
}

void XmlProtocolMsg::setCode(std::string val)
{
    Code_ = val;
}

int XmlProtocolMsg::getCmd()
{
    return Command_;
}

void XmlProtocolMsg::setCmd(int val)
{
    Command_ = val;
}

std::string XmlProtocolMsg::getTime()
{
    return Time_;
}

void XmlProtocolMsg::setTime(std::string str)
{
    Time_ = str;
}

void XmlProtocolMsg::setItem(TiXmlElement *item)
{
    if (xmlItemData_)
    {
        delete xmlItemData_;
        xmlItemData_ = NULL;
    }

    xmlItemData_ = new TiXmlElement(*item);
}

TiXmlElement * XmlProtocolMsg::getItem()
{
    return xmlItemData_;
}

void XmlProtocolMsg::setSequenceId(int seq)
{
    sequence_id_ = seq;
}

int XmlProtocolMsg::getSequenceId()
{
    return sequence_id_;
}

bool XmlProtocolMsg::decode()
{
    if (origMsg_ == NULL)
    {
        ROS_ERROR("no origMsg_");
        return false;
    }

    if (xmlDoc_ != NULL)
    {
        delete xmlDoc_;
        xmlDoc_ = NULL;
    }

    //decoding
    if (origMsg_->xml_string_.length() > 0)
    {
        xmlDoc_ = new TiXmlDocument;
        //xmlDocument->Parse((const char*)xmlString.c_str(), 0, TIXML_ENCODING_UTF8);
        xmlDoc_->Parse((const char*)origMsg_->xml_string_.c_str(), 0);
        xmlRobot_ = xmlDoc_->FirstChildElement();
        if (xmlRobot_->FirstChildElement() != NULL)
        {
            for (TiXmlElement* i = xmlRobot_->FirstChildElement(); i != NULL; i = (TiXmlElement*)xmlRobot_->IterateChildren(i))
            {
                std::string elementValue = i->Value();

                if ("SendCode" == elementValue && i->GetText() != NULL)
                {
                    SendCode_ = i->GetText();
                }
                else if ("ReceiveCode" == elementValue && i->GetText() != NULL)
                {
                    ReceiveCode_ = i->GetText();
                }
                else if ("Type" == elementValue && i->GetText() != NULL)
                {
                    Type_ = std::stoi(i->GetText());
                }
                else if ("Code" == elementValue && i->GetText() != NULL)
                {
                    Code_ = i->GetText();
                }
                else if ("Command" == elementValue && i->GetText() != NULL)
                {
                    Command_ = std::stoi(i->GetText());
                }
                else if ("Time" == elementValue && i->GetText() != NULL)
                {
                    Time_ = i->GetText();
                }
                else if ("Items" == elementValue)
                {
                    xmlItemData_ = i;
                }
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool XmlProtocolMsg::encode(std::string &str)
{
    if (xmlDoc_)
    {
        delete xmlDoc_;
        xmlDoc_ = NULL;
    }

    if (xmlRobot_)
    {
        delete xmlRobot_;
        xmlRobot_ = NULL;
    }

    xmlDoc_ = new TiXmlDocument;
    TiXmlDeclaration *xmlDecl = new TiXmlDeclaration("1.0", "UTF-8", "");
    xmlDoc_->LinkEndChild(xmlDecl);

    xmlRobot_ = new TiXmlElement("Robot");
    xmlDoc_->LinkEndChild(xmlRobot_);

    if (SendCode_.size() > 0)
    {
        TiXmlElement *xmlEle = new TiXmlElement("SendCode");
        xmlRobot_->LinkEndChild(xmlEle);

        TiXmlText *xmlTxt = new TiXmlText(SendCode_);
        xmlEle->LinkEndChild(xmlTxt);
    }
    else
    {
        ROS_ERROR("SendCode_ must not be empty");
        return false;
    }

    if (ReceiveCode_.size() > 0)
    {
        TiXmlElement *xmlEle = new TiXmlElement("ReceiveCode");
        xmlRobot_->LinkEndChild(xmlEle);

        TiXmlText *xmlTxt = new TiXmlText(ReceiveCode_);
        xmlEle->LinkEndChild(xmlTxt);
    }
    else
    {
        ROS_ERROR("ReceiveCode_ must not be empty");
        return false;
    }

    if (Type_ != -1)
    {
        TiXmlElement *xmlEle = new TiXmlElement("Type");
        xmlRobot_->LinkEndChild(xmlEle);

        TiXmlText *xmlTxt = new TiXmlText(std::to_string(Type_));
        xmlEle->LinkEndChild(xmlTxt);
    }
    else
    {
        ROS_ERROR("Type_ must not be empty");
        return false;
    }

    TiXmlElement *codeEle = new TiXmlElement("Code");
    xmlRobot_->LinkEndChild(codeEle);
    if (Code_.size() > 0)
    {
        TiXmlText *xmlTxt = new TiXmlText(Code_);
        codeEle->LinkEndChild(xmlTxt);
    }

    TiXmlElement *cmdEle = new TiXmlElement("Command");
    xmlRobot_->LinkEndChild(cmdEle);
    if (Command_ != -1)
    {
        TiXmlText *xmlTxt = new TiXmlText(std::to_string(Command_));
        cmdEle->LinkEndChild(xmlTxt);
    }

    TiXmlElement *timeEle = new TiXmlElement("Time");
    xmlRobot_->LinkEndChild(timeEle);
    if (Time_.size() > 0)
    {
        TiXmlText *xmlTxt = new TiXmlText(Time_);
        timeEle->LinkEndChild(xmlTxt);
    }

    if (xmlItemData_ != NULL)
    {
        xmlRobot_->LinkEndChild(xmlItemData_);
    }
    else
    {
        xmlItemData_ = new TiXmlElement("Items");
        xmlRobot_->LinkEndChild(xmlItemData_);
    }
    
    TiXmlPrinter printer;
    printer.SetIndent("");

    xmlDoc_->Accept(&printer);
    str = printer.CStr();

    return true;
}

void XmlProtocolMsg::generiateMsg(uint64_t id)
{
    if (origMsg_ != NULL)
    {
        delete origMsg_;
        origMsg_ = NULL;
    }
    origMsg_ = new OriginalBaseMsg;
    origMsg_->sequence_id_ = id;
    if (encode(origMsg_->xml_string_))
    {
        origMsg_->xml_length_ = origMsg_->xml_string_.length();
    }
    else
    {
        delete origMsg_;
        origMsg_ = NULL;
    }
}

void XmlProtocolMsg::generiateMsg()
{
    if (origMsg_ != NULL)
    {
        delete origMsg_;
        origMsg_ = NULL;
    }

    origMsg_ = new OriginalBaseMsg;
    origMsg_->sequence_id_ = sequence_id_;
    if (encode(origMsg_->xml_string_))
    {
        origMsg_->xml_length_ = origMsg_->xml_string_.length();
    }
    else
    {
        delete origMsg_;
        origMsg_ = NULL;
    }
}
