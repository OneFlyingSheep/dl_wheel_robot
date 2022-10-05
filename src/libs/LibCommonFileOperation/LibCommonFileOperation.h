#ifndef __DL_COMMON_FILE_OPER_H__
#define __DL_COMMON_FILE_OPER_H__

#include <io.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <string>
#include <sstream>
#include <vector>
#include <windows.h>
#include <QString>

extern bool bDirExists(std::string dir);
extern bool bFileExists(std::string filePath);
extern std::string& trim(std::string &s);
extern void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c);
extern std::string getString(const int n);

extern QString convertStdStr2QString(std::string str);
extern std::string convertQString2StdStr(QString qs);

#endif