#ifndef BELL_WIDGET
#define BELL_WIDGET

#include <QWidget>
#include <QPropertyAnimation>
#include "LibSocket/generaldef.h"

class BellWidget : public QWidget
{
    Q_OBJECT

public:
    BellWidget(QWidget *parent = Q_NULLPTR);

    void setBellStatus(RobotBodyWarnLight status);

private:
    void paintEvent(QPaintEvent *event);

    void mousePressEvent(QMouseEvent *event);

signals:
    void signalBellClicked();

private:
    // �任�Ƕ�;
    int m_bellAngle;
    QPropertyAnimation *m_moveAnimation;
    bool m_isAlarm;
    // ��ǰ����״̬;
    RobotBodyWarnLight m_bellStatus;
};
#endif