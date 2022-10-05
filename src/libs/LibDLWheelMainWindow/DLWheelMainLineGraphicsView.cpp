#include "DLWheelMainLineGraphicsView.h"
#include <QDateTime>
#include <QCalendarWidget>
#include <QDesktopWidget>
#include <QApplication>

#pragma execution_character_set("utf-8")

DLWheelMainLineGraphicsView::DLWheelMainLineGraphicsView(QWidget* parent /* = NULL */)
    : QGraphicsView(parent)
{
    // 根据屏幕大小计算地图显示宽度,再根据图片大小计算地图高度(保持横纵比例不变);
    QDesktopWidget* desktopWidget = QApplication::desktop();
    QRect screenRect = desktopWidget->screenGeometry();
    this->setFixedWidth(screenRect.width() - 100);

    QString strMainLineImage = QApplication::applicationDirPath() + "/MainLineImage/MainLineShowBackImage.png";
    
    this->setStyleSheet(QString("QWidget{border-image:url(%1);}").arg(strMainLineImage));
}

void DLWheelMainLineGraphicsView::initWidget()
{

}

void DLWheelMainLineGraphicsView::mousePressEvent(QMouseEvent *event)
{

}

void DLWheelMainLineGraphicsView::mouseMoveEvent(QMouseEvent* event)
{

}

void DLWheelMainLineGraphicsView::mouseDoubleClickEvent(QMouseEvent *event)
{

}

