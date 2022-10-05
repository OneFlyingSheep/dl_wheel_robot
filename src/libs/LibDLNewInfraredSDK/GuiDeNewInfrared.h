#pragma once

#include <QWidget>
#include "GuideSDK.h"
// #include "common_type.h"
// #include "SDKParams.h"
#include <boost/thread/mutex.hpp>

#include "InfraredBase.h"

class GuiDeNewInfrared : public InfraredBase
{
public:
    GuiDeNewInfrared();
    ~GuiDeNewInfrared();

    void getInfraredRectTemp(QString filePath, CoorInfrared coor, ImageInfraredTemp &reData);
    void getInfraredPointTemp(QString filePath, InfraredPointBase pointBase, float &val);
    void fit_infrared_create(QString filePath, QString picPath);
    void getInfraredTemp(QString filePath, FLOAT_T *temp);
private:
    boost::mutex m_lock;
};

