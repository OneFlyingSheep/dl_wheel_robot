#pragma once
#include "unzip.h"
#include "zip.h"
#include <QObject>

class LibZip 
{
public:
    LibZip();
    ~LibZip();

    void create_taskfile_zip(QString taskUUid);
    void create_zip(QString taskUUid);
    void unzip_taskfile_zip(QString taskUUid, QString unzipPath="");
    bool is_exits_file(QString taskUUid);
private:

};

