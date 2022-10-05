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

// �����ڲ������ô���;
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
    // ��������;
    void signalSendData(QList<QString>);

private:
    // ��ʼ���ؼ�;
    void initWidget()
    {
        if (m_isInclude)
        {
            this->setTitleContent("��������");
        }
        else
        {
            this->setTitleContent("����������");
        }
        
        m_inputValueLineEdit = new QLineEdit;
        m_inputValueLineEdit->setFixedSize(QSize(160, 25));
        m_inputValueLineEdit->setFocus();
        QRegExp regx("-?(\\d+\\.\\d+)");
        QValidator *numberValidator = new QRegExpValidator(regx, m_inputValueLineEdit);
        m_inputValueLineEdit->setValidator(numberValidator);

        QPushButton* pButtonAddElement = new QPushButton("���Ԫ��");
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
        gLayout->addWidget(new QLabel("Ԫ��ֵ:"), 0, 0);
        gLayout->addWidget(m_inputValueLineEdit, 0, 1);
        gLayout->addWidget(pButtonAddElement, 0, 2);
        gLayout->addWidget(new QLabel("��ǰ����:"), 1, 0);
        gLayout->addWidget(m_currentDataSetLabel, 1, 1, 1, 2);
        gLayout->setSpacing(10);
        gLayout->setMargin(10);


        QPushButton* pButtonOk = new QPushButton("ȷ��");
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

        QPushButton* pButtonReset = new QPushButton("����");
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
    // ��ǰ�Ƿ��ǰ����߼�����;
    bool m_isInclude;

    QLineEdit* m_inputValueLineEdit;
    QLabel* m_currentDataSetLabel;

    QList<QString> m_dataLsit;
};

// ������������ô���;
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
    // ������������;
    void signalSendIntervalData(bool, QString, bool, QString);

private:
    void initWidget()
    {
        if (m_isInInterval)
        {
            this->setTitleContent("����������");
        }
        else
        {
            this->setTitleContent("������������");
        }

        QCheckBox* leftBorderCheckBox = new QCheckBox("��߽�:");
        leftBorderCheckBox->setStyleSheet("border:none");
        QLineEdit* leftBorderLineEdit = new QLineEdit;
        leftBorderLineEdit->setFixedSize(QSize(160, 25));
        leftBorderLineEdit->setFocus();
        QRegExp regx("-?(\\d+\\.\\d+)");
        QValidator *leftBorderValidator = new QRegExpValidator(regx, leftBorderLineEdit);
        leftBorderLineEdit->setValidator(leftBorderValidator);

        QCheckBox* rightBorderCheckBox = new QCheckBox("�ұ߽�:");
        rightBorderCheckBox->setStyleSheet("border:none");
        QLineEdit* rightBorderLineEdit = new QLineEdit;
        rightBorderLineEdit->setFixedSize(QSize(160, 25));
        QValidator *rightBorderValidator = new QRegExpValidator(regx, rightBorderLineEdit);
        rightBorderLineEdit->setValidator(rightBorderValidator);

        QGridLayout* gLayout = new QGridLayout;
        gLayout->addWidget(leftBorderCheckBox, 0, 0);
        gLayout->addWidget(leftBorderLineEdit, 0, 1);
        gLayout->addWidget(new QLabel("������ֵ"), 0, 2);
        gLayout->addWidget(rightBorderCheckBox, 1, 0);
        gLayout->addWidget(rightBorderLineEdit, 1, 1);
        gLayout->addWidget(new QLabel("������ֵ"), 1, 2);
        gLayout->setSpacing(10);
        gLayout->setMargin(10);

        QPushButton* pButtonOk = new QPushButton("ȷ��");
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

// ��ֵ���ô���;
class NumberSetWidget : public BaseWidget
{
    Q_OBJECT

public:
    NumberSetWidget(QWidget* parent = NULL, bool isInInterval = false)
        : BaseWidget(parent, BaseWidgetType::PopupWindow)
    {
        initWidget();

        this->setTitleContent("��ֵ����");
        this->setShowCloseButton();
        this->setAttribute(Qt::WA_DeleteOnClose);
        this->setFixedSize(QSize(350, 150));
        this->setWindowModality(Qt::ApplicationModal);

        connect(this, &BaseWidget::signalCloseButtonClicked, this, &NumberSetWidget::close);
    }

signals:
    // ���Ͷ�������;
    void signalSendNumberData(QString);

private:
    // ��ʼ���ؼ�;
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

        QPushButton* pButtonOk = new QPushButton("ȷ��");
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

/************��������ʽ�Ĵ���**************/

class ThresholdTitleWidget : public QWidget
{
public:
    ThresholdTitleWidget(QWidget* parent = NULL)
    {
        initWidget();
        this->setStyleSheet("QWidget#TitleBackWidget{background:rgb(159,168,218);}\
								QWidget#CenterBackWidget{background:white;}");
    }

    // ���ñ���;
    void setTitleText(const QString& text)
    {
        m_titleLabel->setText(text);
    }

    // ��ȡ����widget���Ա��ڶԴ�widget����;
    QWidget* getCenterWidget()
    {
        return m_centerBackWidget;
    }

private:
    // ��ʼ���ؼ�;
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

/********�澯��֮����ҳ��s*********/

class DLWheelAlarmThresholdSet : public QWidget
{
	Q_OBJECT

public:
    DLWheelAlarmThresholdSet(QWidget* parent = NULL);
    ~DLWheelAlarmThresholdSet();

    // ��ʼ���ؼ�;
    void initWidget();

private:
	// ���ڿؼ���ʼ��;
    void initStackedWidget();
    void initLeftWidget();
    void initCenterWidget();
    void initRightWidget();
    void initOperatorWidget();
    void initLogicOperatorWidget();
    void initElementWidget();
    void initFunctionWidget();
    void initLegendWidget();

    // ͼƬ����Ƶ������������;
    void initImageAndVideoOutDateSet();

	// ���ڱ��������¼�;
	void paintEvent(QPaintEvent *event);

    // ���浱ǰ��ͬ�澯�ȼ��ı��ʽ;
    void saveAlarmLevelExpression();

    // ���水ť���;
    void saveButtonClicked();

private slots:
    void onLeftButtonClicked();

    // �м�button���;
    void onCenterButtonClicked(int buttonId);

    // �����;
    void onOperatorButtonClicked(int buttonId);

    // �߼������;
    void onLogicOperatorButtonClicked(int buttonId);

    // Ԫ�ذ�ť���;
    void onElementButtonClicked(int buttonId);

private:
    // �����ؼ�;
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

    // ͼƬ����Ƶ������������;
    QWidget* m_imageAndVideoOutDateSetBackWidget;
    InputWidget* m_imageOutDateSetWidget;
    InputWidget* m_videoOutDateSetWidget;    

    // ����קwidget;
    CustomDragArea* m_alarmTextEdit;

    // �����ڲ������ô���;
    IncludeWidget* m_includeWidget;

    // ������������ô���;
    IntervalSetWidget* m_intervalWidget;

    // ��ֵ���ô���;
    NumberSetWidget* m_numberSetWidget;

    // ��ֵ��֤;
    conventSymbols2Lua* m_conventSymbols2Lua;

    // ��¼��ͬ�澯�ȼ��ı��ʽ;
    QList<THRESHOLD_ELEMENT> m_normalExpression;
    QList<THRESHOLD_ELEMENT> m_warningExpression;
    QList<THRESHOLD_ELEMENT> m_commonAlarmExpression;
    QList<THRESHOLD_ELEMENT> m_seriousAlarmExpression;
    QList<THRESHOLD_ELEMENT> m_dangerousAlarmExpression;

    DeviceAlarmLevel m_currentButtonAlarmLevel;

    bool m_isInitWidget;
};

#endif // DL_WHEEL_ALARM_THRESHOLD_SET_H
