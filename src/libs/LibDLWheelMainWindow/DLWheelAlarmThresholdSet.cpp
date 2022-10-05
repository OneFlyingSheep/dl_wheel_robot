#include "DLWheelAlarmThresholdSet.h"
#include <QHBoxLayout>
#include <QStyle>
#include <QPainter>
#include <QButtonGroup>
#include <QPainter>
#include <QUuid>
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLWheelCustomWidget/CustomDragWidget.h"
#include "LibConvertSymbolsToLua/LibConvertSymbolsToLua.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLWheelCustomWidget/DefData.h"

#define TITLE_WIDGET_HEIGHT 150

DLWheelAlarmThresholdSet::DLWheelAlarmThresholdSet(QWidget* parent)
    : QWidget(parent)
    , m_conventSymbols2Lua(NULL)
    , m_currentButtonAlarmLevel(Alarm_Normal)
    , m_isInitWidget(false)
{
    this->setStyleSheet("QAbstractButton{border-radius:3px;border:1px solid gray;}\
                        QAbstractButton:pressed{padding-left:2px;padding-top:2px;}\
                         QPushButton#AlarmLevelButton:checked{background:lightgray;}");
}

DLWheelAlarmThresholdSet::~DLWheelAlarmThresholdSet()
{
    if (m_conventSymbols2Lua != NULL)
    {
        delete m_conventSymbols2Lua;
    }
}

void DLWheelAlarmThresholdSet::initStackedWidget()
{
    m_stackedBackWidget = new QWidget;
    QPushButton* pButtonTreeWidget = new QPushButton("点位树");
    pButtonTreeWidget->setFixedSize(QSize(80, 25));
    connect(pButtonTreeWidget, &QPushButton::clicked, this, [=] {
        m_stackedWidget->setCurrentIndex(0);
    });

    QPushButton* pButtonListWidget = new QPushButton("表计类型选择");
    pButtonListWidget->setFixedSize(QSize(80, 25));
    connect(pButtonListWidget, &QPushButton::clicked, this, [=] {
        m_stackedWidget->setCurrentIndex(1);
    });

    QHBoxLayout* hButtonLayout = new QHBoxLayout;
    hButtonLayout->addWidget(pButtonTreeWidget);
    hButtonLayout->addWidget(pButtonListWidget);
    hButtonLayout->addStretch();
    hButtonLayout->setSpacing(10);
    hButtonLayout->setMargin(0);

    m_stackedWidget = new QStackedWidget;
    m_stackedWidget->setStyleSheet("QStackedWidget{border:1px solid black;}");

    QVBoxLayout* vLayout = new QVBoxLayout(m_stackedBackWidget);
    vLayout->addLayout(hButtonLayout);
    vLayout->addWidget(m_stackedWidget);
    vLayout->setSpacing(10);
    vLayout->setMargin(0);

    m_pointNameTreeWidget = new CustomTreeWidget;
    m_pointNameTreeWidget->setFixedWidth(338);
    m_pointNameTreeWidget->setTreeWidgetType(TreeItemWidgetType::ColorRect_CheckBox_With);
    m_pointNameTreeWidget->setTitleWidgetVisible(false);
    m_pointNameTreeWidget->setSearchLineEditVisible(false);
    //m_pointNameTreeWidget->refreshTreeItemList();

    m_meterTypeListWidget = new QListWidget;
    m_meterTypeListWidget->addItems(WHEEL_DEVICE_CONFIG.getWheelMeterTypeNameQList());
    m_meterTypeListWidget->setStyleSheet("QListWidget::item{height:25px;font-size-16px;}");

    m_stackedWidget->insertWidget(0, m_pointNameTreeWidget);
    m_stackedWidget->insertWidget(1, m_meterTypeListWidget);
    
}

void DLWheelAlarmThresholdSet::initLeftWidget()
{
    initStackedWidget();

    m_leftBackWidget = new ThresholdTitleWidget;
    m_leftBackWidget->setTitleText("装置列表");
    m_leftBackWidget->setFixedWidth(350);

    m_leftButtonGroup = new QButtonGroup(this);
    connect(m_leftButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &DLWheelAlarmThresholdSet::onLeftButtonClicked);

    m_pointNameSearchLineEdit = new QLineEdit;
    m_pointNameSearchLineEdit->setPlaceholderText("点位名称查询");
    m_pointNameSearchLineEdit->setFixedHeight(25);
    
    m_pButtonPointNameSearch = new QToolButton;
    m_pButtonPointNameSearch->setIcon(QIcon(":/Resources/Common/image/Search.png"));
    m_pButtonPointNameSearch->setIconSize(QSize(25, 25));
    m_pButtonPointNameSearch->setText("查询");
    m_pButtonPointNameSearch->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

    m_leftButtonGroup->addButton(m_pButtonPointNameSearch, 0);

    QHBoxLayout* hLineEditLayout = new QHBoxLayout;
    hLineEditLayout->addWidget(m_pointNameSearchLineEdit);
    hLineEditLayout->addWidget(m_pButtonPointNameSearch);
    hLineEditLayout->setSpacing(10);
    hLineEditLayout->setMargin(0);

    m_pointPosLabel = new QLabel("点位:-");
    m_pathLabel = new QLabel("路径:-");

    QLabel* templateLabel = new QLabel("模板:");

    QPushButton* pButtonDelete = new QPushButton("删除");
    pButtonDelete->setFixedSize(QSize(75, 25));
    m_leftButtonGroup->addButton(pButtonDelete, 1);

    QPushButton* pButtonCopyTemplate = new QPushButton("复制模板");
    pButtonCopyTemplate->setFixedSize(QSize(75, 25));
    m_leftButtonGroup->addButton(pButtonCopyTemplate, 2);

    QPushButton* pButtonApplyTemplate = new QPushButton("应用模板");
    pButtonApplyTemplate->setFixedSize(QSize(75, 25));
    m_leftButtonGroup->addButton(pButtonApplyTemplate, 3);

    QHBoxLayout* hTemplateLayout = new QHBoxLayout;
    hTemplateLayout->addWidget(templateLabel);
    hTemplateLayout->addStretch();
    hTemplateLayout->addWidget(pButtonDelete);
    hTemplateLayout->addWidget(pButtonCopyTemplate);
    hTemplateLayout->addWidget(pButtonApplyTemplate);
    hTemplateLayout->setSpacing(10);
    hTemplateLayout->setMargin(0);

    m_templateTextEdit = new QTextEdit;
    
    QLabel* measureLabel = new QLabel("量测量");
    QPushButton* pButtonSave = new QPushButton("保存");
    pButtonSave->setFixedSize(QSize(50, 25));
    m_leftButtonGroup->addButton(pButtonSave, 4);

    QPushButton* pButtonReset = new QPushButton("重置");
    pButtonReset->setFixedSize(QSize(50, 25));
    m_leftButtonGroup->addButton(pButtonReset, 5);

    QHBoxLayout* hMeasureLayout = new QHBoxLayout;
    hMeasureLayout->addWidget(measureLabel);
    hMeasureLayout->addWidget(pButtonSave);
    hMeasureLayout->addWidget(pButtonReset);
    hMeasureLayout->setSpacing(10);
    hMeasureLayout->setMargin(0);

    m_measureTextEdit = new QTextEdit;

    QVBoxLayout* vLeftLayout = new QVBoxLayout(m_leftBackWidget->getCenterWidget());
    vLeftLayout->addLayout(hLineEditLayout);
    vLeftLayout->addWidget(m_stackedBackWidget);
    vLeftLayout->addWidget(m_pointPosLabel);
    vLeftLayout->addWidget(m_pathLabel);
    vLeftLayout->addLayout(hTemplateLayout);
    vLeftLayout->addWidget(m_templateTextEdit);
    vLeftLayout->addLayout(hMeasureLayout);
    vLeftLayout->addWidget(m_measureTextEdit);
    vLeftLayout->setSpacing(10);
    vLeftLayout->setMargin(5);
}

void DLWheelAlarmThresholdSet::initCenterWidget()
{
    m_centerBackWidget = new ThresholdTitleWidget;
    m_centerBackWidget->setTitleText("表达式");

    m_centerButtonGroup = new QButtonGroup(this);
    connect(m_centerButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &DLWheelAlarmThresholdSet::onCenterButtonClicked);

    QPushButton* pButtonSave = new QPushButton("保存");
    pButtonSave->setFixedSize(QSize(50, 25));
    m_centerButtonGroup->addButton(pButtonSave, 0);

    QPushButton* pButtonHold = new QPushButton("暂存");
    pButtonHold->setFixedSize(QSize(50, 25));
    m_centerButtonGroup->addButton(pButtonHold, 1);

    QPushButton* pButtonVerify = new QPushButton("验证");
    pButtonVerify->setFixedSize(QSize(50, 25));
    m_centerButtonGroup->addButton(pButtonVerify, 2);

    QPushButton* pButtonClear = new QPushButton("清除");
    pButtonClear->setFixedSize(QSize(50, 25));
    m_centerButtonGroup->addButton(pButtonClear, 3);

    QHBoxLayout* hOperateButtonLayout = new QHBoxLayout;
    hOperateButtonLayout->addWidget(pButtonSave);
    hOperateButtonLayout->addWidget(pButtonHold);
    hOperateButtonLayout->addWidget(pButtonVerify);
    hOperateButtonLayout->addWidget(pButtonClear);
    hOperateButtonLayout->addStretch();
    hOperateButtonLayout->setSpacing(0);
    hOperateButtonLayout->setMargin(0);

    QPushButton* pButtonNormal = new QPushButton("正常");
    pButtonNormal->setFixedSize(QSize(75, 25));
    pButtonNormal->setObjectName("AlarmLevelButton");
    pButtonNormal->setCheckable(true);
    pButtonNormal->setChecked(true);
    m_centerButtonGroup->addButton(pButtonNormal, 4);

    QPushButton* pButtonWaring = new QPushButton("预警");
    pButtonWaring->setFixedSize(QSize(75, 25));
    pButtonWaring->setObjectName("AlarmLevelButton");
    pButtonWaring->setCheckable(true);
    m_centerButtonGroup->addButton(pButtonWaring, 5);

    QPushButton* pButtonCommonAlarm = new QPushButton("一般告警");
    pButtonCommonAlarm->setFixedSize(QSize(75, 25));
    pButtonCommonAlarm->setObjectName("AlarmLevelButton");
    pButtonCommonAlarm->setCheckable(true);
    m_centerButtonGroup->addButton(pButtonCommonAlarm, 6);

    QPushButton* pButtonSeriousAlarm = new QPushButton("严重告警");
    pButtonSeriousAlarm->setFixedSize(QSize(75, 25));
    pButtonSeriousAlarm->setObjectName("AlarmLevelButton");
    pButtonSeriousAlarm->setCheckable(true);
    m_centerButtonGroup->addButton(pButtonSeriousAlarm, 7);

    QPushButton* pButtonDangerousAlarm = new QPushButton("危急告警");
    pButtonDangerousAlarm->setFixedSize(QSize(75, 25));
    pButtonDangerousAlarm->setObjectName("AlarmLevelButton");
    pButtonDangerousAlarm->setCheckable(true);
    m_centerButtonGroup->addButton(pButtonDangerousAlarm, 8);

    QHBoxLayout* hAlarmButtonLayout = new QHBoxLayout;
    hAlarmButtonLayout->addWidget(pButtonNormal);
    hAlarmButtonLayout->addWidget(pButtonWaring);
    hAlarmButtonLayout->addWidget(pButtonCommonAlarm);
    hAlarmButtonLayout->addWidget(pButtonSeriousAlarm);
    hAlarmButtonLayout->addWidget(pButtonDangerousAlarm);
    hAlarmButtonLayout->addStretch();
    hAlarmButtonLayout->setSpacing(0);
    hAlarmButtonLayout->setMargin(0);

    m_alarmTextEdit = new CustomDragArea;

    QLabel* normalLabel = new QLabel("正常");
    m_normalTextEdit = new QTextEdit;

    QLabel* warningLabel = new QLabel("预警");
    m_warningTextEdit = new QTextEdit;

    QLabel* commonAlarmLabel = new QLabel("一般告警");
    m_commonAlarmTextEdit = new QTextEdit;

    QLabel* seriousAlarmLabel = new QLabel("严重告警");
    m_SeriousAlarmTextEdit = new QTextEdit;

    QLabel* dangerousAlarmLabel = new QLabel("危急告警");
    m_dangerousAlarmTextEdit = new QTextEdit;

    QVBoxLayout* vCenterLayout = new QVBoxLayout(m_centerBackWidget->getCenterWidget());
    vCenterLayout->addLayout(hOperateButtonLayout);
    vCenterLayout->addLayout(hAlarmButtonLayout);
    vCenterLayout->addWidget(m_alarmTextEdit);
    vCenterLayout->addWidget(normalLabel);
    vCenterLayout->addWidget(m_normalTextEdit);
    vCenterLayout->addWidget(warningLabel);
    vCenterLayout->addWidget(m_warningTextEdit);
    vCenterLayout->addWidget(commonAlarmLabel);
    vCenterLayout->addWidget(m_commonAlarmTextEdit);
    vCenterLayout->addWidget(seriousAlarmLabel);
    vCenterLayout->addWidget(m_SeriousAlarmTextEdit);
    vCenterLayout->addWidget(dangerousAlarmLabel);
    vCenterLayout->addWidget(m_dangerousAlarmTextEdit);
    vCenterLayout->setSpacing(10);
    vCenterLayout->setMargin(5);
}

void DLWheelAlarmThresholdSet::initRightWidget()
{
    m_rightBackWidget = new QWidget;
    m_rightBackWidget->setFixedWidth(350);

    initOperatorWidget();
    initLogicOperatorWidget();
    initElementWidget();
    initFunctionWidget();
    initLegendWidget();

    QVBoxLayout* vRightLayout = new QVBoxLayout(m_rightBackWidget);
    vRightLayout->addWidget(m_rightOperatorWidget);
    vRightLayout->addWidget(m_rightLogicOperatorWidget);
    vRightLayout->addWidget(m_rightElementWidget);
    vRightLayout->addWidget(m_rightFunctionWidget);
    vRightLayout->addWidget(m_rightLegendWidget);
    vRightLayout->setSpacing(10);
    vRightLayout->setMargin(0);
}

void DLWheelAlarmThresholdSet::initOperatorWidget()
{
    m_rightOperatorWidget = new ThresholdTitleWidget;
    m_rightOperatorWidget->setTitleText("运算符");
    
    m_operatorButtonGroup = new QButtonGroup(this);

    for (int i = 0; i < 12; i++)
    {
        QPushButton* pButton = new QPushButton;
        pButton->setFixedHeight(25);
        m_operatorButtonGroup->addButton(pButton, i);
    }

    m_operatorButtonGroup->button(0)->setText("加");
    m_operatorButtonGroup->button(1)->setText("等于");
    m_operatorButtonGroup->button(2)->setText("不等于");
    m_operatorButtonGroup->button(3)->setText("减");
    m_operatorButtonGroup->button(4)->setText("大于");
    m_operatorButtonGroup->button(5)->setText("大于等于");
    m_operatorButtonGroup->button(6)->setText("乘");
    m_operatorButtonGroup->button(7)->setText("小于");
    m_operatorButtonGroup->button(8)->setText("小于等于");
    m_operatorButtonGroup->button(9)->setText("除");
    m_operatorButtonGroup->button(10)->setText("括号");
    m_operatorButtonGroup->button(11)->setText("中括号");

    QGridLayout* gLayout = new QGridLayout(m_rightOperatorWidget->getCenterWidget());
    int row = 0;
    for (int i = 0; i < 12; i ++)
    {
        gLayout->addWidget(m_operatorButtonGroup->button(i), row, i % 3);
        if ((i + 1) % 3 == 0)
        {
            row++;
        }
    }
    gLayout->setSpacing(5);
    gLayout->setContentsMargins(5, 5, 30, 10);

    connect(m_operatorButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &DLWheelAlarmThresholdSet::onOperatorButtonClicked);
}

void DLWheelAlarmThresholdSet::initLogicOperatorWidget()
{
    m_rightLogicOperatorWidget = new ThresholdTitleWidget;
    m_rightLogicOperatorWidget->setTitleText("逻辑运算符");

    m_logicOperatorButtonGroup = new QButtonGroup(this);

    for (int i = 0; i < 6; i++)
    {
        QPushButton* pButton = new QPushButton;
        pButton->setFixedHeight(25);
        m_logicOperatorButtonGroup->addButton(pButton, i);
    }

    m_logicOperatorButtonGroup->button(0)->setText("且");
    m_logicOperatorButtonGroup->button(1)->setText("或");
    m_logicOperatorButtonGroup->button(2)->setText("包含");
    m_logicOperatorButtonGroup->button(3)->setText("不包含");
    m_logicOperatorButtonGroup->button(4)->setText("在区间");
    m_logicOperatorButtonGroup->button(5)->setText("不在区间");

    connect(m_logicOperatorButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &DLWheelAlarmThresholdSet::onLogicOperatorButtonClicked);

    QGridLayout* gLayout = new QGridLayout(m_rightLogicOperatorWidget->getCenterWidget());
    int row = 0;
    for (int i = 0; i < 6; i++)
    {
        gLayout->addWidget(m_logicOperatorButtonGroup->button(i), row, i % 2);
        if ((i + 1) % 2 == 0)
        {
            row++;
        }
    }
    gLayout->setSpacing(5);
    gLayout->setContentsMargins(5, 5, 60, 10);
}

void DLWheelAlarmThresholdSet::initElementWidget()
{
    m_rightElementWidget = new ThresholdTitleWidget;
    m_rightElementWidget->setTitleText("元素");

    m_ElementButtonGroup = new QButtonGroup(this);

    for (int i = 0; i < 8; i++)
    {
        QPushButton* pButton = new QPushButton;
        pButton->setFixedHeight(25);
        m_ElementButtonGroup->addButton(pButton, i);
    }

    m_ElementButtonGroup->button(0)->setText("变化量");
    m_ElementButtonGroup->button(1)->setText("当前值");
    m_ElementButtonGroup->button(2)->setText("定值");
    m_ElementButtonGroup->button(3)->setText("初值");
    m_ElementButtonGroup->button(4)->setText("初值差绝对值");
    m_ElementButtonGroup->button(5)->setText("初值差");
    m_ElementButtonGroup->button(6)->setText("相间温差");
    m_ElementButtonGroup->button(7)->setText("变化率");

    connect(m_ElementButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &DLWheelAlarmThresholdSet::onElementButtonClicked);

    QGridLayout* gLayout = new QGridLayout(m_rightElementWidget->getCenterWidget());
    gLayout->addWidget(m_ElementButtonGroup->button(0), 0, 0);
    gLayout->addWidget(m_ElementButtonGroup->button(1), 0, 1);
    gLayout->addWidget(m_ElementButtonGroup->button(2), 1, 0);
    gLayout->addWidget(m_ElementButtonGroup->button(3), 1, 1);
    gLayout->addWidget(m_ElementButtonGroup->button(4), 2, 0, 1, 2);
    gLayout->addWidget(m_ElementButtonGroup->button(5), 3, 0);
    gLayout->addWidget(m_ElementButtonGroup->button(6), 3, 1);
    gLayout->addWidget(m_ElementButtonGroup->button(7), 4, 0);

    gLayout->setSpacing(5);
    gLayout->setContentsMargins(5, 5, 60, 10);
}

void DLWheelAlarmThresholdSet::initFunctionWidget()
{
    m_rightFunctionWidget = new ThresholdTitleWidget;
    m_rightFunctionWidget->setTitleText("函数方法");

    QPushButton* pButtonAbs = new QPushButton("绝对值");
    pButtonAbs->setFixedSize(QSize(75, 25));

    connect(pButtonAbs, &QPushButton::clicked, this, [=] {
        // 绝对值;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_ABS_FUNC;
        data.val.append("abs(");
        m_alarmTextEdit->addItemData(data);

        THRESHOLD_ELEMENT dataRight;
        dataRight.type = THRESHOLD_TYPE_RIGHT_PARENTHESES;
        dataRight.val.append(")");
        m_alarmTextEdit->addItemData(dataRight);
    });

    QHBoxLayout* hLayout = new QHBoxLayout(m_rightFunctionWidget->getCenterWidget());
    hLayout->addWidget(pButtonAbs);
    hLayout->addStretch();
    hLayout->setContentsMargins(5, 5, 0, 10);
}

void DLWheelAlarmThresholdSet::initLegendWidget()
{
    m_rightLegendWidget = new ThresholdTitleWidget;
    m_rightLegendWidget->setTitleText("图例");

    QVBoxLayout* vLayout = new QVBoxLayout(m_rightLegendWidget->getCenterWidget());
    QLabel* labelLegend[4];
    for (int i = 0; i < 4; i++)
    {
        labelLegend[i] = new QLabel;
        vLayout->addWidget(labelLegend[i]);
    }

    labelLegend[0]->setText("系统默认模板");
    labelLegend[1]->setText("装置应用模板");
    labelLegend[2]->setText("参数已编辑表达式");
    labelLegend[3]->setText("未保存的表达式");

    vLayout->addStretch();
    vLayout->setSpacing(10);
    vLayout->setContentsMargins(10, 10, 30, 0);
}

void DLWheelAlarmThresholdSet::initImageAndVideoOutDateSet()
{
    m_imageAndVideoOutDateSetBackWidget = new QWidget;
    m_imageAndVideoOutDateSetBackWidget->setFixedHeight(90);
    m_imageAndVideoOutDateSetBackWidget->setStyleSheet(".QWidget{background:white;}");

    m_imageOutDateSetWidget = new InputWidget(InputWidgetType::LineEditWithButton);
    m_imageOutDateSetWidget->setFixedWidth(350);
    m_imageOutDateSetWidget->setTipText("图片保存期限:(天数)");
    m_imageOutDateSetWidget->setButtonText("保存");

    // 图片保存期限;
    connect(m_imageOutDateSetWidget, &InputWidget::signalButtonClicked, this, [=] {
        
    });

    QRegExp rx("^[0-9]+(\\d+)?$");
    QValidator *validator = new QRegExpValidator(rx, m_imageOutDateSetWidget);
    m_imageOutDateSetWidget->setLineEditValidator(validator);

    m_videoOutDateSetWidget = new InputWidget(InputWidgetType::LineEditWithButton);
    m_videoOutDateSetWidget->setFixedWidth(350);
    m_videoOutDateSetWidget->setTipText("视频保存期限:(天数)");
    m_videoOutDateSetWidget->setButtonText("保存");
    m_videoOutDateSetWidget->setLineEditValidator(validator);

    // 视频保存期限;
    connect(m_videoOutDateSetWidget, &InputWidget::signalButtonClicked, this, [=] {

    });

    QVBoxLayout* vOutDateSetLayout = new QVBoxLayout(m_imageAndVideoOutDateSetBackWidget);
    vOutDateSetLayout->addWidget(m_imageOutDateSetWidget);
    vOutDateSetLayout->addWidget(m_videoOutDateSetWidget);
    vOutDateSetLayout->setSpacing(10);
    vOutDateSetLayout->setContentsMargins(15, 10, 0, 10);
}

void DLWheelAlarmThresholdSet::initWidget()
{
    if (m_isInitWidget)
    {
        return;
    }
    m_isInitWidget = true;

    initLeftWidget();
    initCenterWidget();
    initRightWidget();
    initImageAndVideoOutDateSet();

    m_conventSymbols2Lua = new conventSymbols2Lua();
    
    QHBoxLayout* hMainLayout = new QHBoxLayout();
    hMainLayout->addWidget(m_leftBackWidget);
    hMainLayout->addWidget(m_centerBackWidget);
    hMainLayout->addWidget(m_rightBackWidget);
    hMainLayout->setMargin(0);

    QVBoxLayout* vMainLayout = new QVBoxLayout(this);
    vMainLayout->addLayout(hMainLayout);
    vMainLayout->addWidget(m_imageAndVideoOutDateSetBackWidget);
    vMainLayout->addStretch();
    vMainLayout->setSpacing(10);
    vMainLayout->setContentsMargins(300, 0, 300, 10);
}

void DLWheelAlarmThresholdSet::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.fillRect(this->rect(), QColor(194, 228, 226));
}

void DLWheelAlarmThresholdSet::saveAlarmLevelExpression()
{
    switch (m_currentButtonAlarmLevel)
    {
    case Alarm_Normal:
        m_normalExpression = m_alarmTextEdit->getDataList();
        break;
    case Alarm_Waring:
        m_warningExpression = m_alarmTextEdit->getDataList();
        break;
    case Alarm_Common:
        m_commonAlarmExpression = m_alarmTextEdit->getDataList();
        break;
    case Alarm_Serious:
        m_seriousAlarmExpression = m_alarmTextEdit->getDataList();
        break;
    case Alarm_Dangerous:
        m_dangerousAlarmExpression = m_alarmTextEdit->getDataList();
        break;
    default:
        break;
    }
}

void DLWheelAlarmThresholdSet::saveButtonClicked()
{
    // 先保存当前告警等级的表达式;
    saveAlarmLevelExpression();

    if (m_normalExpression.isEmpty() || m_warningExpression.isEmpty() || m_commonAlarmExpression.isEmpty() || m_seriousAlarmExpression.isEmpty() || m_dangerousAlarmExpression.isEmpty())
    {
        DLMessageBox::showDLMessageBox(NULL, "提示", "请将表达式填写完整", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        return;
    }

    QString strUUid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
    QString errMsg;

    bool isSuccess = m_conventSymbols2Lua->getLuaScript(m_normalExpression.toVector(), m_warningExpression.toVector(), m_commonAlarmExpression.toVector(), m_seriousAlarmExpression.toVector(), m_dangerousAlarmExpression.toVector(), strUUid, errMsg);
    if (!isSuccess)
    {
        DLMessageBox::showDLMessageBox(NULL, "提示", "保存失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
    }
    else
    {
        if (m_stackedWidget->currentIndex() == 0)
        {
            QStringList deviceIdList = m_pointNameTreeWidget->getChoosedDeviceIdList();
            if (deviceIdList.isEmpty())
            {
                DLMessageBox::showDLMessageBox(NULL, "提示", "请选择设备", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
            else
            {
                WHEEL_BACK_TO_CORE_SOCKET.robot_threshold_set_by_device(m_normalExpression.toVector(), m_warningExpression.toVector(), m_commonAlarmExpression.toVector(), m_seriousAlarmExpression.toVector(), m_dangerousAlarmExpression.toVector(), deviceIdList.toVector());
            }
        }
        else if (m_stackedWidget->currentIndex() == 1)
        {
            QListWidgetItem* currentItem = m_meterTypeListWidget->currentItem();
            if (currentItem != NULL)
            {
                int meterType = WHEEL_DEVICE_CONFIG.getWheelMeterTypeIdInt(currentItem->text());
                WHEEL_BACK_TO_CORE_SOCKET.robot_threshold_set_by_meter_type(m_normalExpression.toVector(), m_warningExpression.toVector(), m_commonAlarmExpression.toVector(), m_seriousAlarmExpression.toVector(), m_dangerousAlarmExpression.toVector(), (WheelRobotMeterType)meterType);
            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "提示", "请选择表计类型", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        }
    }
}

void DLWheelAlarmThresholdSet::onLeftButtonClicked()
{

}

void DLWheelAlarmThresholdSet::onCenterButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
    {
        // 保存;
        saveButtonClicked();
    }
        break;
    case 1:
    {
        // 暂存;

    }
        break;
    case 2:
    {
        // 验证;
        QString luaString, errMsg;
        bool isSuccess = m_conventSymbols2Lua->convert(m_alarmTextEdit->getDataList().toVector(), luaString, m_currentButtonAlarmLevel, errMsg);
        if (!isSuccess)
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "验证失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    }
        break;
    case 3:
    {
        // 清除当前告警等级表达式;
        QList<THRESHOLD_ELEMENT> dataList;
        m_alarmTextEdit->setDataList( dataList);
        saveAlarmLevelExpression();
    }
        break;
    case 4:
    {
        // 正常;
        saveAlarmLevelExpression();
        m_currentButtonAlarmLevel = Alarm_Normal;
        m_alarmTextEdit->setDataList(m_normalExpression);
    }
    break;
    case 5:
    {
        // 预警;
        saveAlarmLevelExpression();
        m_currentButtonAlarmLevel = Alarm_Waring;
        m_alarmTextEdit->setDataList(m_warningExpression);
    }
        break;
    case 6:
    {
        // 一般告警;
        saveAlarmLevelExpression();
        m_currentButtonAlarmLevel = Alarm_Common;
        m_alarmTextEdit->setDataList(m_commonAlarmExpression);
    }
        break;
    case 7:
    {
        // 严重告警;
        saveAlarmLevelExpression();
        m_currentButtonAlarmLevel = Alarm_Serious;
        m_alarmTextEdit->setDataList(m_seriousAlarmExpression);
    }
        break;
    case 8:
    {
        // 危急告警;
        saveAlarmLevelExpression();
        m_currentButtonAlarmLevel = Alarm_Dangerous;
        m_alarmTextEdit->setDataList(m_dangerousAlarmExpression);
    }
        break;
    break;
    default:
        break;
    }
}

void DLWheelAlarmThresholdSet::onOperatorButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_PLUS;
        data.val.append("+");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 1:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_EQUAL;
        data.val.append("=");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 2:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_NOT_EQUAL;
        data.val.append("!=");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 3:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_MINUS;
        data.val.append("-");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 4:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_GREATER_THEN;
        data.val.append(">");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 5:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_GREATER_EQUAL;
        data.val.append(">=");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 6:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_MUTIPLE;
        data.val.append("*");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 7:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_LESS_THEN;
        data.val.append("<");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 8:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_LESS_EQUAL;
        data.val.append("<=");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 9:
    {
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_DEVIDED;
        data.val.append("/");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 10:
    {
        THRESHOLD_ELEMENT dataLeft;
        dataLeft.type = THRESHOLD_TYPE_LEFT_PARENTHESES;
        dataLeft.val.append("(");
        m_alarmTextEdit->addItemData(dataLeft);

        THRESHOLD_ELEMENT dataRight;
        dataRight.type = THRESHOLD_TYPE_RIGHT_PARENTHESES;
        dataRight.val.append(")");
        m_alarmTextEdit->addItemData(dataRight);
    }
    break;
    case 11:
    {
        THRESHOLD_ELEMENT dataLeft;
        dataLeft.type = THRESHOLD_TYPE_LEFT_BRACKET;
        dataLeft.val.append("{");
        m_alarmTextEdit->addItemData(dataLeft);

        THRESHOLD_ELEMENT dataRight;
        dataRight.type = THRESHOLD_TYPE_RIGHT_BRACKET;
        dataRight.val.append("}");
        m_alarmTextEdit->addItemData(dataRight);
    }
    break;
    default:
        break;
    }
}

void DLWheelAlarmThresholdSet::onLogicOperatorButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
    {
        // 且;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_AND;
        data.val.append("且");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 1:
    {
        // 或;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_OR;
        data.val.append("或");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 2:
    {
        m_includeWidget = new IncludeWidget(this, true);
        connect(m_includeWidget, &IncludeWidget::signalSendData, this, [=](QList<QString> dataList) {
            // 包含;
            THRESHOLD_ELEMENT data;
            data.type = THRESHOLD_TYPE_INCLUDE;
            for (int i = 0; i < dataList.count(); i++)
            {
                data.val.append(dataList[i]);
            }

            m_alarmTextEdit->addItemData(data);
            m_includeWidget->close();
        });
        m_includeWidget->show();
    }
    break;
    case 3:
    {
        m_includeWidget = new IncludeWidget(this);
        connect(m_includeWidget, &IncludeWidget::signalSendData, this, [=](QList<QString> dataList) {
            // 包含;
            THRESHOLD_ELEMENT data;
            data.type = THRESHOLD_TYPE_EXCLUDE;
            for (int i = 0; i < dataList.count(); i++)
            {
                data.val.append(dataList[i]);
            }

            m_alarmTextEdit->addItemData(data);
            m_includeWidget->close();
        });
        m_includeWidget->show();
    }
    break;
    case 4:
    {
        // 在区间;
        m_intervalWidget = new IntervalSetWidget(this, false);
        connect(m_intervalWidget, &IntervalSetWidget::signalSendIntervalData, this, [=](bool isLeftInclude, QString strLeftValue, bool isRightInclude, QString strRightValue) {
            THRESHOLD_ELEMENT data;
            data.type = THRESHOLD_TYPE_IN_RANGE;
            QString strData;
            if (isLeftInclude)
            {
                strData.append("[");
            }
            else
            {
                strData.append("(");
            }
            strData.append(strLeftValue);
            strData.append(",");
            strData.append(strRightValue);
            if (isRightInclude)
            {
                strData.append("]");
            }
            else
            {
                strData.append(")");
            }
            data.val.append(strData);
            m_alarmTextEdit->addItemData(data);
            m_intervalWidget->close();
        });
        m_intervalWidget->show();
    }
    break;
    case 5:
    {
        // 不在区间;
        m_intervalWidget = new IntervalSetWidget(this);
        connect(m_intervalWidget, &IntervalSetWidget::signalSendIntervalData, this, [=](bool isLeftInclude, QString strLeftValue, bool isRightInclude, QString strRightValue) {
            THRESHOLD_ELEMENT data;
            data.type = THRESHOLD_TYPE_NOT_IN_RANGE;
            QString strData;
            if (isLeftInclude)
            {
                strData.append("[");
            }
            else
            {
                strData.append("(");
            }
            strData.append(strLeftValue);
            strData.append(",");
            strData.append(strRightValue);
            if (isRightInclude)
            {
                strData.append("]");
            }
            else
            {
                strData.append(")");
            }
            data.val.append(strData);
            m_alarmTextEdit->addItemData(data);
            m_intervalWidget->close();
        });
        m_intervalWidget->show();
    }
    break;
    default:
        break;
    }
}

void DLWheelAlarmThresholdSet::onElementButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
    {
        // 变化量;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_VARIABLE;
        data.val.append("VARIABLE");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 1:
    {
        // 当前值;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_CURRENT_VARIABLE;
        data.val.append("CURRENT_VARIABLE");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 2:
    {
        m_numberSetWidget = new NumberSetWidget(this);
        connect(m_numberSetWidget, &NumberSetWidget::signalSendNumberData, this, [=](QString strNumber) {
            // 定值;
            THRESHOLD_ELEMENT data;
            data.type = THRESHOLD_TYPE_NUMBER;
            data.val.append(strNumber);
            m_alarmTextEdit->addItemData(data);
            m_numberSetWidget->close();
        });
        m_numberSetWidget->show();
    }
    break;
    case 3:
    {
        // 初值;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_INIT_VAL;
        data.val.append("INIT_VAL");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 4:
    {
        // 初值差绝对值;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_INITIAL_DIFFERENCE_ABSOLUTE_VALUE;
        data.val.append("DIFFERENCE_ABSOLUTE_VALUE");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 5:
    {
        // 初值差;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_INITIAL_DIFFERENCE_VALUE;
        data.val.append("DIFFERENCE_VALUE");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 6:
    {
        // 相间温差;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_DIFFERENCE_TEMPERATURE_BETWEEN_PHASES;
        data.val.append("DIFFERENCE_TEMPERATURE_BETWEEN_PHASES");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    case 7:
    {
        // 变化率;
        THRESHOLD_ELEMENT data;
        data.type = THRESHOLD_TYPE_CHANGE_RATE;
        data.val.append("CHANGE_RATE");
        m_alarmTextEdit->addItemData(data);
    }
    break;
    default:
        break;
    }
}
