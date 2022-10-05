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
    // 变换角度;
    int m_bellAngle;
    QPropertyAnimation *m_moveAnimation;
    bool m_isAlarm;
    // 当前响铃状态;
    RobotBodyWarnLight m_bellStatus;
};
#endif