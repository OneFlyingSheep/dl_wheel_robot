#include "DLWheelCompareDetection.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QHBoxLayout>
#include <QGridLayout>

DLWheelCompareDetection::DLWheelCompareDetection(QWidget *parent /*= NULL*/)
{
    initWidget();
}

DLWheelCompareDetection::~DLWheelCompareDetection()
{

}

void DLWheelCompareDetection::initControlBackWidget()
{
    m_controlBackWidget = new QWidget;
    m_controlBackWidget->setStyleSheet("border:1px solid gray;");

    QPushButton* pButtonDetection = new QPushButton("¼ì²â");
    pButtonDetection->setFixedSize(QSize(75, 25));
    pButtonDetection->setStyleSheet("QPushButton{font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
								QPushButton:hover{background-color:rgb(44 , 137 , 255);}\
								QPushButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");

    // ¼ì²â°´Å¥µã»÷;
    connect(pButtonDetection, &QPushButton::clicked, this, [=] {
    
    });

    QHBoxLayout* hLayout = new QHBoxLayout(m_controlBackWidget);
    hLayout->addWidget(pButtonDetection);
    hLayout->addStretch();
}

void DLWheelCompareDetection::initWidget()
{
    initControlBackWidget();

    m_centerWidget = new BaseWidget;
    m_centerWidget->setTitleContent("ÒìÎï¼ì²â");

    m_leftImageLabel = new PictureLabel;
    m_rightImageLabel = new PictureLabel;

    m_detectionResultLabel = new QLabel;
    m_detectionResultLabel->setStyleSheet("background:lightGray;");
    m_detectionResultLabel->setFixedHeight(250);

    QGridLayout* gCenterlayout = new QGridLayout(m_centerWidget->getCenterWidget());
    gCenterlayout->addWidget(m_leftImageLabel, 0, 0);
    gCenterlayout->addWidget(m_rightImageLabel, 0, 1);
    gCenterlayout->addWidget(m_detectionResultLabel, 1, 0);
    gCenterlayout->addWidget(m_controlBackWidget, 1, 1);
    gCenterlayout->setSpacing(20);

    QHBoxLayout* hMainLayout = new QHBoxLayout(this);
    hMainLayout->addWidget(m_centerWidget);
    hMainLayout->setMargin(15);
}
