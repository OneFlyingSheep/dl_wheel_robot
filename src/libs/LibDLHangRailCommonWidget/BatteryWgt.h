#ifndef _BATTERY_WGT_H
#define _BATTERY_WGT_H
//���ͼ�����
#include <QWidget>
#include <QTimer>

class BatteryWgt : public QWidget
{
public:
    BatteryWgt(QWidget* parent = NULL);
       
    // ���õ�ǰ���Ŷ�;
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