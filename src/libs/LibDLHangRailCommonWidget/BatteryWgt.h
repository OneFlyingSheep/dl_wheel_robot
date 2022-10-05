#ifndef _BATTERY_WGT_H
#define _BATTERY_WGT_H
//电池图标界面
#include <QWidget>
#include <QTimer>

class BatteryWgt : public QWidget
{
public:
    BatteryWgt(QWidget* parent = NULL);
       
    // 设置当前置信度;
    void setCurrentValue(float value);

private:
    void paintEvent(QPaintEvent *event);
	virtual void showEvent(QShowEvent *event);

private:
    int m_iValue;
    //QTimer m_flashTimer;
	QTimer *m_pFlashTimer;
    bool m_bIsFlash;
};
#endif //_BATTERY_WGT_H