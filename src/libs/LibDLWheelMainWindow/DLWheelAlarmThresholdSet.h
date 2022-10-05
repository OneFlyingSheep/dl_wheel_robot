#ifndef DL_WHEEL_ALARM_THRESHOLD_SET_H
#define DL_WHEEL_ALARM_THRESHOLD_SET_H

#include <QWidget>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QComboBox>
#include <QRadioButton>
#include <QTreeWidget>
#include <QPushButton>
#include <QTextEdit>
#include <QButtonGroup>
#include <QListWidget>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDLHangRailCommonWidget/BaseWidget.h"

#pragma execution_character_set("utf-8")

// 包含于参数设置窗口;
class IncludeWidget : public BaseWidget
{
    Q_OBJECT

public:
    IncludeWidget(QWidget* parent = NULL, bool isInclude = false)
        : BaseWidget(parent, BaseWidgetType::PopupWindow)
        , m_isInclude(isInclude)
    {
        initWidget();

        this->setShowCloseButton();
        this->setAttribute(Qt::WA_DeleteOnClose);
        this->setFixedSize(QSize(350, 200));
        this->setWindowModality(Qt::ApplicationModal);

        connect(this, &BaseWidget::signalCloseButtonClicked, this, &IncludeWidget::close);
    }

signals:
    // 发送数据;
    void signalSendData(QList<QString>);

private:
    // 初始化控件;
    void initWidget()
    {
        if (m_isInclude)
        {
            this->setTitleContent("包含设置");
        }
        else
        {
            this->setTitleContent("不包含设置");
        }
        
        m_inputValueLineEdit = new QLineEdit;
        m_inputValueLineEdit->setFixedSize(QSize(160, 25));
        m_inputValueLineEdit->setFocus();
        QRegExp regx("-?(\\d+\\.\\d+)");
        QValidator *numberValidator = new QRegExpValidator(regx, m_inputValueLineEdit);
        m_inputValueLineEdit->setValidator(numberValidator);

        QPushButton* pButtonAddElement = new QPushButton("添加元素");
        pButtonAddElement->setFixedSize(QSize(70, 25));
        connect(pButtonAddElement, &QPushButton::clicked, this, [=] {            
            if (m_inputValueLineEdit->text().isEmpty())
            {
                return;
            }

            QString strCurrentSet = m_currentDataSetLabel->text();
            if (!m_dataLsit.isEmpty())
            {
                strCurrentSet.append(",");
            }

            m_dataLsit.append(m_inputValueLineEdit->text());
            strCurrentSet.append(m_inputValueLineEdit->text());
            m_currentDataSetLabel->setText(strCurrentSet);
            m_currentDataSetLabel->setWordWrap(true);
            m_currentDataSetLabel->setScaledContents(true);
            m_inputValueLineEdit->setText("");
        });


        m_currentDataSetLabel = new QLabel;
        m_currentDataSetLabel->setFixedWidth(240);
        m_currentDataSetLabel->setFixedHeight(50);

        QGridLayout* gLayout = new QGridLayout;
        gLayout->addWidget(new QLabel("元素值:"), 0, 0);
        gLayout->addWidget(m_inputValueLineEdit, 0, 1);
        gLayout->addWidget(pButtonAddElement, 0, 2);
        gLayout->addWidget(new QLabel("当前集合:"), 1, 0);
        gLayout->addWidget(m_currentDataSetLabel, 1, 1, 1, 2);
        gLayout->setSpacing(10);
        gLayout->setMargin(10);


        QPushButton* pButtonOk = new QPushButton("确定");
        pButtonOk->setFixedSize(QSize(70, 25));
        connect(pButtonOk, &QPushButton::clicked, this, [=] {
            if (m_dataLsit.isEmpty())
            {
                this->close();
            }
            else
            {
                emit signalSendData(m_dataLsit);
            }
        });

        QPushButton* pButtonReset = new QPushButton("重置");
        pButtonReset->setFixedSize(QSize(70, 25));
        connect(pButtonReset, &QPushButton::clicked, this, [=] {
            m_dataLsit.clear();
            m_currentDataSetLabel->setText("");
        });

        QWidget* buttonBackWidget = new QWidget;

        QHBoxLayout* hButtonlayout = new QHBoxLayout(buttonBackWidget);
        hButtonlayout->addStretch();
        hButtonlayout->addWidget(pButtonOk);
        hButtonlayout->addWidget(pButtonReset);
        hButtonlayout->addStretch();
        hButtonlayout->setSpacing(10);

        QVBoxLayout* vMainLayout = new QVBoxLayout(this->getCenterWidget());
        vMainLayout->addLayout(gLayout);
        vMainLayout->addWidget(buttonBackWidget);
        vMainLayout->setSpacing(10);
        vMainLayout->setMargin(0);
    }

private:
    // 当前是否是包含逻辑运算;
    bool m_isInclude;

    QLineEdit* m_inputValueLineEdit;
    QLabel* m_currentDataSetLabel;

    QList<QString> m_dataLsit;
};

// 在区间参数设置窗口;
class IntervalSetWidget : public BaseWidget
{
    Q_OBJECT

public:
    IntervalSetWidget(QWidget* parent = NULL, bool isInInterval = false)
        : BaseWidget(parent, BaseWidgetType::PopupWindow)
        , m_isInInterval(isInInterval)
    {
        initWidget();

        this->setShowCloseButton();
        this->setAttribute(Qt::WA_DeleteOnClose);
        this->setFixedSize(QSize(350, 180));
        this->setWindowModality(Qt::ApplicationModal);

        connect(this, &BaseWidget::signalCloseButtonClicked, this, &IntervalSetWidget::close);
    }

signals:
    // 发送区间数据;
    void signalSendIntervalData(bool, QString, bool, QString);

private:
    void initWidget()
    {
        if (m_isInInterval)
        {
            this->setTitleContent("在区间设置");
        }
        else
        {
            this->setTitleContent("不在区间设置");
        }

        QCheckBox* leftBorderCheckBox = new QCheckBox("左边界:");
        leftBorderCheckBox->setStyleSheet("border:none");
        QLineEdit* leftBorderLineEdit = new QLineEdit;
        leftBorderLineEdit->setFixedSize(QSize(160, 25));
        leftBorderLineEdit->setFocus();
        QRegExp regx("-?(\\d+\\.\\d+)");
        QValidator *leftBorderValidator = new QRegExpValidator(regx, leftBorderLineEdit);
        leftBorderLineEdit->setValidator(leftBorderValidator);

        QCheckBox* rightBorderCheckBox = new QCheckBox("右边界:");
        rightBorderCheckBox->setStyleSheet("border:none");
        QLineEdit* rightBorderLineEdit = new QLineEdit;
        rightBorderLineEdit->setFixedSize(QSize(160, 25));
        QValidator *rightBorderValidator = new QRegExpValidator(regx, rightBorderLineEdit);
        rightBorderLineEdit->setValidator(rightBorderValidator);

        QGridLayout* gLayout = new QGridLayout;
        gLayout->addWidget(leftBorderCheckBox, 0, 0);
        gLayout->addWidget(leftBorderLineEdit, 0, 1);
        gLayout->addWidget(new QLabel("包含该值"), 0, 2);
        gLayout->addWidget(rightBorderCheckBox, 1, 0);
        gLayout->addWidget(rightBorderLineEdit, 1, 1);
        gLayout->addWidget(new QLabel("包含该值"), 1, 2);
        gLayout->setSpacing(10);
        gLayout->setMargin(10);

        QPushButton* pButtonOk = new QPushButton("确定");
        pButtonOk->setFixedSize(QSize(70, 25));
        connect(pButtonOk, &QPushButton::clicked, this, [=] {
            if (leftBorderLineEdit->text().isEmpty() || rightBorderLineEdit->text().isEmpty())
            {
                this->close();
                return;
            }

            emit signalSendIntervalData(leftBorderCheckBox->isChecked(), leftBorderLineEdit->text(), rightBorderCheckBox->isChecked(), rightBorderLineEdit->text());
        });

        QHBoxLayout* hButtonlayout = new QHBoxLayout();
        hButtonlayout->addStretch();
        hButtonlayout->addWidget(pButtonOk);
        hButtonlayout->addStretch();
        hButtonlayout->setSpacing(10);

        QVBoxLayout* vMainLayout = new QVBoxLayout(this->getCenterWidget());
        vMainLayout->addLayout(gLayout);
        vMainLayout->addLayout(hButtonlayout);
        vMainLayout->setSpacing(10);
        vMainLayout->setMargin(0);
    }

private:
    bool m_isInInterval;
};

// 定值设置窗口;
class NumberSetWidget : public BaseWidget
{
    Q_OBJECT

public:
    NumberSetWidget(QWidget* parent = NULL, bool isInInterval = false)
        : BaseWidget(parent, BaseWidgetType::PopupWindow)
    {
        initWidget();

        this->setTitleContent("定值设置");
        this->setShowCloseButton();
        this->setAttribute(Qt::WA_DeleteOnClose);
        this->setFixedSize(QSize(350, 150));
        this->setWindowModality(Qt::ApplicationModal);

        connect(this, &BaseWidget::signalCloseButtonClicked, this, &NumberSetWidget::close);
    }

signals:
    // 发送定制数据;
    void signalSendNumberData(QString);

private:
    // 初始化控件;
    void initWidget()
    {
        QLineEdit* numberLineEdit = new QLineEdit;
        numberLineEdit->setFixedSize(QSize(160, 25));
        QRegExp regx("-?(\\d+\\.\\d+)");
        QValidator *floatNumberValidator = new QRegExpValidator(regx, numberLineEdit);
        numberLineEdit->setValidator(floatNumberValidator);
        numberLineEdit->setFocus();

        QHBoxLayout* hLineEditLayout = new QHBoxLayout;
        hLineEditLayout->addStretch();
        hLineEditLayout->addWidget(numberLineEdit);
        hLineEditLayout->addStretch();
        hLineEditLayout->setMargin(0);

        QPushButton* pButtonOk = new QPushButton("确定");
        pButtonOk->setFixedSize(QSize(70, 25));
        connect(pButtonOk, &QPushButton::clicked, this, [=] {
            if (numberLineEdit->text().isEmpty())
            {
                this->close();
                return;
            }

            emit signalSendNumberData(numberLineEdit->text());
        });

        QHBoxLayout* hButtonlayout = new QHBoxLayout();
        hButtonlayout->addStretch();
        hButtonlayout->addWidget(pButtonOk);
        hButtonlayout->addStretch();
        hButtonlayout->setSpacing(10);

        QVBoxLayout* vMainLayout = new QVBoxLayout(this->getCenterWidget());
        vMainLayout->addLayout(hLineEditLayout);
        vMainLayout->addLayout(hButtonlayout);
        vMainLayout->setSpacing(10);
        vMainLayout->setMargin(0);
    }
};


class InputWidget;
class CustomDragArea;
class conventSymbols2Lua;
class CustomTreeWidget;

/************带标题样式的窗口**************/

class ThresholdTitleWidget : public QWidget
{
public:
    ThresholdTitleWidget(QWidget* parent = NULL)
    {
        initWidget();
        this->setStyleSheet("QWidget#TitleBackWidget{background:rgb(159,168,218);}\
								QWidget#CenterBackWidget{background:white;}");
    }

    // 设置标题;
    void setTitleText(const QString& text)
    {
        m_titleLabel->setText(text);
    }

    // 获取中心widget，以便于对此widget布局;
    QWidget* getCenterWidget()
    {
        return m_centerBackWidget;
    }

private:
    // 初始化控件;
    void initWidget()
    {
        QWidget* titleBackWidget = new QWidget;
        titleBackWidget->setObjectName("TitleBackWidget");
        titleBackWidget->setFixedHeight(30);
        titleBackWidget->setStyleSheet("QWidget{background:rgb(159,168,218);}");
        m_titleLabel = new QLabel;
        m_titleLabel->setStyleSheet("font-weight:bold;");
        QHBoxLayout* hTitleLayout = new QHBoxLayout(titleBackWidget);
        hTitleLayout->addWidget(m_titleLabel);
        hTitleLayout->addStretch();
        hTitleLayout->setContentsMargins(5, 0, 0, 0);

        m_centerBackWidget = new QWidget;
        m_centerBackWidget->setObjectName("CenterBackWidget");
        QVBoxLayout* vMainLayout = new QVBoxLayout(this);
        vMainLayout->addWidget(titleBackWidget);
        vMainLayout->addWidget(m_centerBackWidget);
        vMainLayout->setSpacing(0);
        vMainLayout->setMargin(0);
    }

private:
    QLabel * m_titleLabel;
    QWidget* m_centerBackWidget;
};

/********告警与之设置页面s*********/

class DLWheelAlarmThresholdSet : public QWidget
{
	Q_OBJECT

public:
    DLWheelAlarmThresholdSet(QWidget* parent = NULL);
    ~DLWheelAlarmThresholdSet();

    // 初始化控件;
    void initWidget();

private:
	// 窗口控件初始化;
    void initStackedWidget();
    void initLeftWidget();
    void initCenterWidget();
    void initRightWidget();
    void initOperatorWidget();
    void initLogicOperatorWidget();
    void initElementWidget();
    void initFunctionWidget();
    void initLegendWidget();

    // 图片、视频保存期限设置;
    void initImageAndVideoOutDateSet();

	// 窗口背景绘制事件;
	void paintEvent(QPaintEvent *event);

    // 保存当前不同告警等级的表达式;
    void saveAlarmLevelExpression();

    // 保存按钮点击;
    void saveButtonClicked();

private slots:
    void onLeftButtonClicked();

    // 中间button点击;
    void onCenterButtonClicked(int buttonId);

    // 运算符;
    void onOperatorButtonClicked(int buttonId);

    // 逻辑运算符;
    void onLogicOperatorButtonClicked(int buttonId);

    // 元素按钮点击;
    void onElementButtonClicked(int buttonId);

private:
    // 基本控件;
    ThresholdTitleWidget* m_leftBackWidget;
    QLineEdit* m_pointNameSearchLineEdit;
    QToolButton* m_pButtonPointNameSearch;
    QWidget* m_stackedBackWidget;
    QStackedWidget* m_stackedWidget;
    CustomTreeWidget* m_pointNameTreeWidget;
    QListWidget* m_meterTypeListWidget;
    QLabel* m_pointPosLabel;
    QLabel* m_pathLabel;

    QTextEdit* m_templateTextEdit;
    QTextEdit* m_measureTextEdit;
    QButtonGroup* m_leftButtonGroup;

    ThresholdTitleWidget* m_centerBackWidget;
    QButtonGroup* m_centerButtonGroup;
    QTextEdit* m_normalTextEdit;
    QTextEdit* m_warningTextEdit;
    QTextEdit* m_commonAlarmTextEdit;
    QTextEdit* m_SeriousAlarmTextEdit;
    QTextEdit* m_dangerousAlarmTextEdit;

    QWidget* m_rightBackWidget;
    ThresholdTitleWidget* m_rightOperatorWidget;
    QButtonGroup* m_operatorButtonGroup;

    ThresholdTitleWidget* m_rightLogicOperatorWidget;
    QButtonGroup* m_logicOperatorButtonGroup;

    ThresholdTitleWidget* m_rightElementWidget;
    QButtonGroup* m_ElementButtonGroup;

    ThresholdTitleWidget* m_rightFunctionWidget;

    ThresholdTitleWidget* m_rightLegendWidget;

    // 图片、视频保存期限设置;
    QWidget* m_imageAndVideoOutDateSetBackWidget;
    InputWidget* m_imageOutDateSetWidget;
    InputWidget* m_videoOutDateSetWidget;    

    // 可拖拽widget;
    CustomDragArea* m_alarmTextEdit;

    // 包含于参数设置窗口;
    IncludeWidget* m_includeWidget;

    // 在区间参数设置窗口;
    IntervalSetWidget* m_intervalWidget;

    // 定值设置窗口;
    NumberSetWidget* m_numberSetWidget;

    // 阈值验证;
    conventSymbols2Lua* m_conventSymbols2Lua;

    // 记录不同告警等级的表达式;
    QList<THRESHOLD_ELEMENT> m_normalExpression;
    QList<THRESHOLD_ELEMENT> m_warningExpression;
    QList<THRESHOLD_ELEMENT> m_commonAlarmExpression;
    QList<THRESHOLD_ELEMENT> m_seriousAlarmExpression;
    QList<THRESHOLD_ELEMENT> m_dangerousAlarmExpression;

    DeviceAlarmLevel m_currentButtonAlarmLevel;

    bool m_isInitWidget;
};

#endif // DL_WHEEL_ALARM_THRESHOLD_SET_H
