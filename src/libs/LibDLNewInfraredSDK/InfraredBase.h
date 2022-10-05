#pragma once

struct CoorInfrared
{
    int coor_x;	// 矩形左上角X坐标
    int coor_y;	// 矩形左上角Y坐标
    int coor_width;	// 矩形宽度
    int coor_height;	// 矩形高度
    CoorInfrared()
    {
        coor_x = 50;
        coor_y = 50;
        coor_width = 100;
        coor_height = 100;
    }
};

struct ImageInfraredTemp
{
    int x;
    int y;
    float temp;

    int x_min;
    int y_min;
    float temp_min;
    ImageInfraredTemp()
    {
        x = 100;
        y = 100;
        temp = 0.0;

        x_min = 100;
        y_min = 100;
        temp_min = 0.0;
    }
};

enum SDKInfraredType
{
    SDK_GUIDE_INFRARED = 0,
    SDK_JUGE_INFRARED, 
};

struct InfraredPointBase
{
    int locate_x;
    int locate_y;
};

class InfraredBase
{
public:
    InfraredBase() {}
    ~InfraredBase() {}


    virtual void getInfraredRectTemp(QString filePath, CoorInfrared coor, ImageInfraredTemp &reData) = 0;
    virtual void getInfraredPointTemp(QString filePath, InfraredPointBase pointBase, float &val) = 0;
    virtual bool setInfraredDDTFile(QString filePath) { return true; }
//     void fit_infrared_create(QString filePath, QString picPath);
//     void getInfraredTemp(QString filePath, FLOAT_T *temp);
private:

};

