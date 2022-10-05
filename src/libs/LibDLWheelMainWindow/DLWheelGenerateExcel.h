#ifndef DL_WHEEL_GENERATE_EXCEL_H
#define DL_WHEEL_GENERATE_EXCEL_H

#include <QWidget>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QDateTimeEdit>
#include "LibDLWheelCustomWidget/CheckBoxWidget.h"
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"

class CustomButtonListWidget;
class CustomTableWidget;

/******字段选择窗口*******/

class FieldChooseWindow : public BaseWidget
{
    Q_OBJECT
public:
    FieldChooseWindow(QWidget* parent = NULL)
        : BaseWidget(parent, PopupWindow)
    {
        initWidget();
        this->setTitleContent("字段选择");
        this->setShowCloseButton();
        this->setFixedSize(QSize(400, 350));
        this->setWindowModality(Qt::ApplicationModal);
        this->setStyleSheet("QPushButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(145, 167, 255);border-radius:3px;}\
						QPushButton:pressed{padding-left:2px;padding-top:2px;}\
						QWidget#TitleBackWidget{border-top:3px solid rgb(145, 167, 255); }");
    }

    // 全选按钮点击;
    void onResetCheckBox(bool isChoose = false)
    {
        for (int i = 0; i < m_checkBoxList.count(); i++)
        {
            m_checkBoxList[i]->setChecked(isChoose);
        }
    }

    // 获取选择的字段ID;
    QList<int> getChooseIdList()
    {
        return m_choosedIdList;
    }

    // 获取选择的字段名;
    QStringList getChoosedNameList()
    {
        QStringList strNameList;
        for (int i = 0; i < m_checkBoxList.count(); i++)
        {
            if (m_checkBoxList[i]->isChecked())
            {
                QString checkBoxName = m_checkBoxList[i]->text();
                strNameList.append(checkBoxName);
                if (checkBoxName == "采集信息")
                {
                    strNameList.append(checkBoxName);
                }
            }
        }
        return strNameList;
    }

private:
    // 初始化控件;
    void initWidget()
    {
        QGridLayout* gCheckBoxLayout = new QGridLayout();
        int floorIndex = 0;
        for (int i = 0; i < 19; i++)
        {
            if (i % 3 == 0)
            {
                floorIndex++;
            }
            QCheckBox* checkBox = new QCheckBox;
            m_checkBoxList.append(checkBox);
            gCheckBoxLayout->addWidget(checkBox, floorIndex, i % 3);
        }

        m_checkBoxList[0]->setText("设备区域");
        m_checkBoxList[1]->setText("间隔");
        m_checkBoxList[2]->setText("设备类型");
        m_checkBoxList[3]->setText("小设备类型");
        m_checkBoxList[4]->setText("点位名称");
        m_checkBoxList[5]->setText("识别类型");
        m_checkBoxList[6]->setText("表计类型");
        m_checkBoxList[7]->setText("发热类型");
        m_checkBoxList[8]->setText("识别状态");
        m_checkBoxList[9]->setText("识别结果");
        m_checkBoxList[10]->setText("审核结果");
        m_checkBoxList[11]->setText("识别时间");
        m_checkBoxList[12]->setText("告警等级");
        m_checkBoxList[13]->setText("环境温度");
        m_checkBoxList[14]->setText("环境湿度");
        m_checkBoxList[15]->setText("环境风速");
        m_checkBoxList[16]->setText("环境风向");
        m_checkBoxList[17]->setText("PM2.5");
        m_checkBoxList[18]->setText("采集信息");

        gCheckBoxLayout->setSpacing(10);
        gCheckBoxLayout->setContentsMargins(40, 0, 40, 0);

        QPushButton* pButtonAllChoose = new QPushButton("全选");
        pButtonAllChoose->setFixedSize(QSize(60, 25));
        connect(pButtonAllChoose, &QPushButton::clicked, this, [=] {
            if (pButtonAllChoose->text() == "全选")
            {
                pButtonAllChoose->setText("反选");
                onResetCheckBox(true);
            }
            else
            {
                pButtonAllChoose->setText("全选");
                onResetCheckBox(false);
            }
        });

        QPushButton* pButtonOk = new QPushButton("确定");
        pButtonOk->setFixedSize(QSize(60, 25));
        connect(pButtonOk, &QPushButton::clicked, this, [=] {
            QList<int> choosedIdList;
            for (int i = 0; i < m_checkBoxList.count(); i++)
            {
                if (m_checkBoxList[i]->isChecked())
                {
                    choosedIdList.append(i);
                }
            }
            if (choosedIdList.isEmpty())
            {
                DLMessageBox::showDLMessageBox(NULL, "提示", "未选择字段项", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
            else
            {
                m_choosedIdList = choosedIdList;
                pButtonAllChoose->setText("全选");
                this->hide();
                emit signalFieldChoosed();
            }            
        });

        QPushButton* pButtonCancel = new QPushButton("取消");
        pButtonCancel->setFixedSize(QSize(60, 25));
        connect(pButtonCancel, &QPushButton::clicked, this, [=] {
            pButtonAllChoose->setText("全选");
            this->hide();
        });

        QHBoxLayout* hButtonLayout = new QHBoxLayout;
        hButtonLayout->addWidget(pButtonAllChoose);
        hButtonLayout->addStretch();
        hButtonLayout->addWidget(pButtonOk);
        hButtonLayout->addWidget(pButtonCancel);
        hButtonLayout->setSpacing(10);
        hButtonLayout->setMargin(15);

        QVBoxLayout* vMainLayout = new QVBoxLayout(this->getCenterWidget());
        vMainLayout->addLayout(gCheckBoxLayout);
        vMainLayout->addLayout(hButtonLayout);
        vMainLayout->setSpacing(10);
        vMainLayout->setMargin(0);
    }


signals:
    // 窗口关闭;
    void signalFieldChoosed();

private:
    QList<QCheckBox*> m_checkBoxList;
    QList<int> m_choosedIdList;
};

/*********生成报表页面*********/

class DLWheelGenerateExcel : public QWidget
{
	Q_OBJECT

public:
	DLWheelGenerateExcel();

	// 窗口控件初始化;
	void initWidget();

private:
	// 界面控件初始化;
	// 初始化CheckBoxWidget;
    void initCheckBoxList();
	void initButtonWidget();
	void initTopWidget();
	void initDeviceTreeWidget();
	void initTableWidget();
    void initTableData(WheelPatrolParameter wheelPatrolPara = WheelPatrolParameter());

	// 绘制事件;
	void paintEvent(QPaintEvent *event);

    // 查询按钮点击;
    void onSearchButtonClicked();

    // 重置按钮点击;
    void onResetButtonClicked();

    // 导出按钮点击;
    void onExportButtonClicked();

private slots:
    // 操作按钮点击;
    void onButtonClicked(int buttonId);
private:
	// TopWidget;
	QWidget* m_topBackWidget;

    QWidget* m_checkBoxBackWidget;
    QList<CheckBoxWidget*> m_checkBoxWidgetList;

	// 设备树;
	CustomTreeWidget * m_deviceTreeWidget;

	// 日期选择;
	QDateTimeEdit* m_startDateEdit;
	QDateTimeEdit* m_endDateEdit;

	// 查询等按钮;
	CustomButtonListWidget* m_customButtonListWidget;

	// table;
	CustomTableWidget* m_customTableWidget;

	// 当前页面是否进行初始化;
	bool m_isInitWidget;

    // 当前table的页数;
    int m_currentPageIndex;

    // 保存当前查询条件;
    WheelPatrolParameter m_currentSearchCondition;

    // 保存table表格中的数据;
    QList<DeviceAlarmSearchStruct> m_singleTableDataList;
    QList<QStringList> m_tableDataList;
	// 字段选择窗口;
    FieldChooseWindow* m_fieldChooseWindow;
};

#endif
