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

/******�ֶ�ѡ�񴰿�*******/

class FieldChooseWindow : public BaseWidget
{
    Q_OBJECT
public:
    FieldChooseWindow(QWidget* parent = NULL)
        : BaseWidget(parent, PopupWindow)
    {
        initWidget();
        this->setTitleContent("�ֶ�ѡ��");
        this->setShowCloseButton();
        this->setFixedSize(QSize(400, 350));
        this->setWindowModality(Qt::ApplicationModal);
        this->setStyleSheet("QPushButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(145, 167, 255);border-radius:3px;}\
						QPushButton:pressed{padding-left:2px;padding-top:2px;}\
						QWidget#TitleBackWidget{border-top:3px solid rgb(145, 167, 255); }");
    }

    // ȫѡ��ť���;
    void onResetCheckBox(bool isChoose = false)
    {
        for (int i = 0; i < m_checkBoxList.count(); i++)
        {
            m_checkBoxList[i]->setChecked(isChoose);
        }
    }

    // ��ȡѡ����ֶ�ID;
    QList<int> getChooseIdList()
    {
        return m_choosedIdList;
    }

    // ��ȡѡ����ֶ���;
    QStringList getChoosedNameList()
    {
        QStringList strNameList;
        for (int i = 0; i < m_checkBoxList.count(); i++)
        {
            if (m_checkBoxList[i]->isChecked())
            {
                QString checkBoxName = m_checkBoxList[i]->text();
                strNameList.append(checkBoxName);
                if (checkBoxName == "�ɼ���Ϣ")
                {
                    strNameList.append(checkBoxName);
                }
            }
        }
        return strNameList;
    }

private:
    // ��ʼ���ؼ�;
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

        m_checkBoxList[0]->setText("�豸����");
        m_checkBoxList[1]->setText("���");
        m_checkBoxList[2]->setText("�豸����");
        m_checkBoxList[3]->setText("С�豸����");
        m_checkBoxList[4]->setText("��λ����");
        m_checkBoxList[5]->setText("ʶ������");
        m_checkBoxList[6]->setText("�������");
        m_checkBoxList[7]->setText("��������");
        m_checkBoxList[8]->setText("ʶ��״̬");
        m_checkBoxList[9]->setText("ʶ����");
        m_checkBoxList[10]->setText("��˽��");
        m_checkBoxList[11]->setText("ʶ��ʱ��");
        m_checkBoxList[12]->setText("�澯�ȼ�");
        m_checkBoxList[13]->setText("�����¶�");
        m_checkBoxList[14]->setText("����ʪ��");
        m_checkBoxList[15]->setText("��������");
        m_checkBoxList[16]->setText("��������");
        m_checkBoxList[17]->setText("PM2.5");
        m_checkBoxList[18]->setText("�ɼ���Ϣ");

        gCheckBoxLayout->setSpacing(10);
        gCheckBoxLayout->setContentsMargins(40, 0, 40, 0);

        QPushButton* pButtonAllChoose = new QPushButton("ȫѡ");
        pButtonAllChoose->setFixedSize(QSize(60, 25));
        connect(pButtonAllChoose, &QPushButton::clicked, this, [=] {
            if (pButtonAllChoose->text() == "ȫѡ")
            {
                pButtonAllChoose->setText("��ѡ");
                onResetCheckBox(true);
            }
            else
            {
                pButtonAllChoose->setText("ȫѡ");
                onResetCheckBox(false);
            }
        });

        QPushButton* pButtonOk = new QPushButton("ȷ��");
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
                DLMessageBox::showDLMessageBox(NULL, "��ʾ", "δѡ���ֶ���", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
            else
            {
                m_choosedIdList = choosedIdList;
                pButtonAllChoose->setText("ȫѡ");
                this->hide();
                emit signalFieldChoosed();
            }            
        });

        QPushButton* pButtonCancel = new QPushButton("ȡ��");
        pButtonCancel->setFixedSize(QSize(60, 25));
        connect(pButtonCancel, &QPushButton::clicked, this, [=] {
            pButtonAllChoose->setText("ȫѡ");
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
    // ���ڹر�;
    void signalFieldChoosed();

private:
    QList<QCheckBox*> m_checkBoxList;
    QList<int> m_choosedIdList;
};

/*********���ɱ���ҳ��*********/

class DLWheelGenerateExcel : public QWidget
{
	Q_OBJECT

public:
	DLWheelGenerateExcel();

	// ���ڿؼ���ʼ��;
	void initWidget();

private:
	// ����ؼ���ʼ��;
	// ��ʼ��CheckBoxWidget;
    void initCheckBoxList();
	void initButtonWidget();
	void initTopWidget();
	void initDeviceTreeWidget();
	void initTableWidget();
    void initTableData(WheelPatrolParameter wheelPatrolPara = WheelPatrolParameter());

	// �����¼�;
	void paintEvent(QPaintEvent *event);

    // ��ѯ��ť���;
    void onSearchButtonClicked();

    // ���ð�ť���;
    void onResetButtonClicked();

    // ������ť���;
    void onExportButtonClicked();

private slots:
    // ������ť���;
    void onButtonClicked(int buttonId);
private:
	// TopWidget;
	QWidget* m_topBackWidget;

    QWidget* m_checkBoxBackWidget;
    QList<CheckBoxWidget*> m_checkBoxWidgetList;

	// �豸��;
	CustomTreeWidget * m_deviceTreeWidget;

	// ����ѡ��;
	QDateTimeEdit* m_startDateEdit;
	QDateTimeEdit* m_endDateEdit;

	// ��ѯ�Ȱ�ť;
	CustomButtonListWidget* m_customButtonListWidget;

	// table;
	CustomTableWidget* m_customTableWidget;

	// ��ǰҳ���Ƿ���г�ʼ��;
	bool m_isInitWidget;

    // ��ǰtable��ҳ��;
    int m_currentPageIndex;

    // ���浱ǰ��ѯ����;
    WheelPatrolParameter m_currentSearchCondition;

    // ����table����е�����;
    QList<DeviceAlarmSearchStruct> m_singleTableDataList;
    QList<QStringList> m_tableDataList;
	// �ֶ�ѡ�񴰿�;
    FieldChooseWindow* m_fieldChooseWindow;
};

#endif
