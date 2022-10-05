#include "BellWidget.h"
#include <QPainter>

BellWidget::BellWidget(QWidget *parent)
    : QWidget(parent)
    , m_isAlarm(false)
    , m_bellStatus(ROBOT_WARNLIGHT_RED)
{
    m_moveAnimation = new QPropertyAnimation(this, "");
    m_moveAnimation->setDuration(120);
    m_moveAnimation->setEasingCurve(QEasingCurve::Linear);
    m_moveAnimation->setStartValue(15);
    m_moveAnimation->setEndValue(-15);
    m_moveAnimation->start();

    connect(m_moveAnimation, &QPropertyAnimation::valueChanged, this, [=](const QVariant &value) {
        m_bellAngle = value.toInt();
        update();
    });
    connect(m_moveAnimation, &QPropertyAnimation::finished, this, [=] {
        QAbstractAnimation::Direction dir = m_moveAnimation->direction();
        if (dir == QAbstractAnimation::Forward)
        {
            m_moveAnimation->setDirection(QAbstractAnimation::Backward);
        }
        else if (dir == QAbstractAnimation::Backward)
        {
            m_moveAnimation->setDirection(QAbstractAnimation::Forward);
        }
        m_moveAnimation->start();
        update();
    });

    this->setFixedSize(QSize(80, 80));
}

void BellWidget::setBellStatus(RobotBodyWarnLight status)
{
    m_bellStatus = status;
    if (m_bellStatus == ROBOT_WARNLIGHT_RED)
    {
        m_moveAnimation->start();
    }
    else
    {
        m_moveAnimation->stop();
    }
    update();
}

void BellWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    int border = 20;
    switch (m_bellStatus)
    {
    case ROBOT_WARNLIGHT_GREEN:
        painter.drawPixmap(this->rect().adjusted(border, border, -border, -border), QPixmap(":/Resources/common/BellNormal.png"));
        break;
    case ROBOT_WARNLIGHT_BLUE:
        painter.drawPixmap(this->rect().adjusted(border, border, -border, -border), QPixmap(":/Resources/common/BellBlue.png"));
        break;
    case ROBOT_WARNLIGHT_RED:
    {
        painter.setRenderHint(QPainter::SmoothPixmapTransform);
        painter.translate(this->rect().center());
        painter.rotate(m_bellAngle);
        painter.translate(-this->rect().center());
        painter.drawPixmap(this->rect().adjusted(border, border, -border, -border), QPixmap(":/Resources/common/BellAlarm.png"));
    }
        break;
    default:
        break;
    }

}

void BellWidget::mousePressEvent(QMouseEvent *event)
{
    emit signalBellClicked();
}