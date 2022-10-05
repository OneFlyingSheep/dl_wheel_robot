#include "GuiDeNewInfrared.h"
#include <QDebug>
#include <QFile>

GuiDeNewInfrared::GuiDeNewInfrared()
{
}

GuiDeNewInfrared::~GuiDeNewInfrared()
{
}

void GuiDeNewInfrared::getInfraredRectTemp(QString filePath, CoorInfrared coor, ImageInfraredTemp &reData)
{
    boost::mutex::scoped_lock lock(m_lock);
    char*  _filePath;
    QByteArray ba = filePath.toLocal8Bit(); // must
    _filePath = ba.data();

    RECT_T rect;
    rect.x = coor.coor_x;
    rect.y = coor.coor_y;
    rect.w = coor.coor_width;
    rect.h = coor.coor_height;

    GD_TEMP_INFO info;
    try
    {
        int irect = GetImgRectTemp(_filePath, rect, &info);

        reData.temp = float(info.highTemp.temp);
        reData.x = int(info.highTemp.x);
        reData.y = int(info.highTemp.y);

        reData.temp_min = float(info.lowTemp.temp);
        reData.x_min = int(info.lowTemp.x);
        reData.y_min = int(info.lowTemp.y);
    }
    catch (const std::exception&)
    {

    }
}

void GuiDeNewInfrared::getInfraredPointTemp(QString filePath, InfraredPointBase pointBase, float &val)
{
    filePath = filePath.replace("_result", "-temp");

    POINT_T point;
    point.x = pointBase.locate_x;
    point.y = pointBase.locate_y;

    char*  _filePath;
    QByteArray ba = filePath.toLocal8Bit(); 
    _filePath = ba.data();

    FLOAT_T temp;
    INT32_T irect = GetImgPointTemp(_filePath, point, &temp);

    if (irect == GUIDEIR_OK)
    {
        val = temp;
        qDebug() << "getInfraredPointTemp:infrared point success!";
    }
    else
    {
        qDebug() << "getInfraredPointTemp:infrared point error!";
    }
}

void GuiDeNewInfrared::fit_infrared_create(QString filePath, QString picPath)
{
    uint8_t head_fir[64]{ 0x00 };
    head_fir[0] = 0x46;
    head_fir[1] = 0x49;
    head_fir[2] = 0x52;
    head_fir[4] = 0x50;
    head_fir[8] = 0x0a;
    head_fir[12] = 0x05;
    head_fir[16] = 0x17;
    head_fir[20] = 0x46;
    head_fir[24] = 0x80;
    head_fir[25] = 0x02;
    head_fir[26] = 0xe0;
    head_fir[27] = 0x01;

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly))
    {
        return;
    }

    file.resize(0);
    QTextStream in(&file);
    in.setCodec("utf-8");
    FLOAT_T *temp = new FLOAT_T[640 * 480];
    getInfraredTemp(picPath, temp);

    for (int i = 0; i < 64; i++)
    {
        char *buf = new char;
        sprintf(buf, "%02x", head_fir[i]);
        in << buf;
        if (i != 64)
        {
            in << " ";
        }
    }
    in << "\n";

    for (int i = 0; i < 480; i++)
    {
        for (int j = 0; j < 640; j++)
        {
            uint8_t _val_1 = ((int(*temp) * 10) & 0xff00) >> 8;
            uint8_t _val_2 = (int(*temp) * 10) & 0xff;
            char *buf = new char;
            sprintf(buf, "%02x", _val_2);
            in << buf;
            in << " ";
            sprintf(buf, "%02x", _val_1);
            in << buf;
            if (j != 640)
            {
                in << " ";
            }
            temp++;
        }
        in << "\n";
    }
    file.close();
}

void GuiDeNewInfrared::getInfraredTemp(QString filePath, FLOAT_T *temp)
{
    char*  _filePath;
    QByteArray ba = filePath.toLocal8Bit();
    _filePath = ba.data();

    INT32_T irect = GetImgTemp(_filePath, temp);
    if (irect == GUIDEIR_OK)
    {

        qDebug() << "getInfraredPointTemp:infrared point success!";
    }
    else
    {
        qDebug() << "getInfraredPointTemp:infrared point error!";
    }
}