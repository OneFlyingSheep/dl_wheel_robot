#include "LibCommonFileOperation.h"

bool bDirExists(std::string dir)
{
    DWORD ftyp = GetFileAttributesA(dir.c_str());
    if (ftyp == INVALID_FILE_ATTRIBUTES)
        return false;  //something is wrong with your path!  

    if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
        return true;   // this is a directory!  

    return false;    // this is not a directory!  
}

bool bFileExists(std::string filePath)
{
    if ((_access(filePath.c_str(), 0)) != -1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::string& trim(std::string &s)
{
    if (s.empty())
    {
        return s;
    }

    s.erase(0, s.find_first_not_of(" "));
    s.erase(s.find_last_not_of(" ") + 1);
    return s;
}

void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (std::string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}

std::string getString(const int n)
{
    std::stringstream newstr;
    newstr << n;
    return newstr.str();
}

QString convertStdStr2QString(std::string str)
{
    QString qstring = QString(QString::fromLocal8Bit(str.c_str()));
    return qstring;
}

std::string convertQString2StdStr(QString qs)
{    
    std::string str = std::string((const char *)qs.toLocal8Bit());
    return str;
}
