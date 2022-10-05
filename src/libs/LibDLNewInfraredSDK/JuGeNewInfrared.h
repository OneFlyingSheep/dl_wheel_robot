#pragma once

#include <QWidget>
#include "InfraredBase.h"
#include "windows.h"
#include "typedef.h"
#include "ThermoGroupSDK.h"

class JuGeNewInfrared : public InfraredBase
{
public:
    JuGeNewInfrared();
    ~JuGeNewInfrared();

    void getInfraredRectTemp(QString filePath, CoorInfrared coor, ImageInfraredTemp &reData);

    void getInfraredPointTemp(QString filePath, InfraredPointBase pointBase, float &val);

    bool setInfraredDDTFile(QString filePath);
private:
    int m_iChannelIndex = 0;
    bool m_bInitChannel = false;
};