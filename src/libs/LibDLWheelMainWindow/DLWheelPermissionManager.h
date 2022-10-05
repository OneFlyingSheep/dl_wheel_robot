#ifndef DL_WHEEL_PERMISSION_MANAGER_SET_H
#define DL_WHEEL_PERMISSION_MANAGER_SET_H

#include <QWidget>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QComboBox>
#include <QRadioButton>
#include <QTreeWidget>
#include <QPushButton>
#include <QButtonGroup>
#include "common/DLWheelRobotGlobalDef.hpp"
#include <QTabWidget>
#include <QPainter>
#include <QStackedWidget>
#include <QTableWidget>
#include <QUuid>
#include <QHeaderView>
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotStationConfig.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"

#pragma execution_character_set("utf-8")

class CustomButtonListWidget;
class TitleWidget;

/*********tab��ť�ؼ�*********/

class TabButtonWidget : public QWidget
{
    Q_OBJECT
public:
    TabButtonWidget(QWidget* parent = NULL)
        : QWidget(parent)
    {
        m_tabButtonGroup = new QButtonGroup(this);
        connect(m_tabButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &TabButtonWidget::signalButtonClicked);

        this->setStyleSheet("QPushButton{color:rgb(63,81,181);background:rgb(159,168,218);border:1px solid rgb(121,134,203);border-top-left-radius:3px;border-top-right-radius:3px;}\
                            QPushButton:pressed{padding-left:2px;padding-top:2px;}\
                            QPushButton:checked{qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(220, 240, 240, 200),\
										stop:0.3 rgba(220, 240, 240, 255), stop:1 rgba(200, 230, 230, 255));\
										color:rgb(40, 120, 100);background:red;font-weight:bold;}");
    }

    // ���tab��ť;
    void addTabButton(QString strButtonText, int buttonWidth)
    {
        QPushButton* tabButton = new QPushButton(strButtonText);
        tabButton->setFixedSize(QSize(buttonWidth, 20));
        tabButton->setCheckable(true);
        m_tabButtonGroup->addButton(tabButton, m_tabButtonGroup->buttons().count());
    }

    // ���tab���֮����ô˷��������ڰ�ť����;
    void addTabButtonFinished()
    {
        QHBoxLayout* hMainLayout = new QHBoxLayout(this);
        for (int i = 0; i < m_tabButtonGroup->buttons().count(); i++)
        {
            hMainLayout->addWidget(m_tabButtonGroup->button(i));
        }

        hMainLayout->addStretch();
        hMainLayout->setSpacing(5);
        hMainLayout->setContentsMargins(5, 5, 0, 0);
    }

signals:
    // tab��ť���;
    void signalButtonClicked(int buttonId);

private:
    // �߿����;
    void paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);
        painter.fillRect(this->rect(), QColor(185, 225, 210));
    }

private:
    QButtonGroup* m_tabButtonGroup;

};

/*****Ȩ�޹�����********/

class RoleManageWidget : public BaseWidget
{
    Q_OBJECT

public :
    RoleManageWidget(QWidget* parent, WheelUserType loginUserType)
        : BaseWidget(parent, BaseWidgetType::PopupWindow)
        , m_loginUserType(loginUserType)
    {
        initWidget();
        initTableWidget();
        this->setTitleContent("Ȩ�޹���");
        this->setShowCloseButton();
        this->setFixedSize(QSize(400, 400));
        this->setWindowModality(Qt::ApplicationModal);

        connect(this, &BaseWidget::signalCloseButtonClicked, this, [=] {
            m_userNameWidget->setLineEditText("");
            m_userPswWidget->setLineEditText("");
            m_userRoleWidget->setComboBoxCurrentIndex(0);
            this->hide();
        });

        WHEEL_BACK_TO_CORE_SOCKET.wheelRobotAddUserConfigStatus.connect(boost::bind(&RoleManageWidget::signalAddNewUserCallBack, this, _1, _2));
        WHEEL_BACK_TO_CORE_SOCKET.wheelRobotDeleteUserConfigStatus.connect(boost::bind(&RoleManageWidget::signalDeleteUserCallBack, this, _1, _2));
        
        connect(this, &RoleManageWidget::signalAddNewUserCallBack, this, [=](bool isSuccess, QString strMsg) {
            if (isSuccess)
            {
                m_userNameWidget->setLineEditText("");
                m_userPswWidget->setLineEditText("");
                m_userRoleWidget->setComboBoxCurrentIndex(0);
                m_userTelNumberWidget->setLineEditText("");
                initTableWidget();
            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "����", "���ʧ��", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        });

        connect(this, &RoleManageWidget::signalDeleteUserCallBack, this, [=](bool isSuccess, QString strMsg) {
            if (isSuccess)
            {
                initTableWidget();

            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "����", "ɾ��ʧ��", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        });
        

        this->setStyleSheet("QPushButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(145, 167, 255);border-radius:3px;}\
						QPushButton:pressed{padding-left:2px;padding-top:2px;}\
						QWidget#TitleBackWidget{border-top:3px solid rgb(145, 167, 255); }");
    }

    // ��ʼ���ؼ�;
    void initWidget()
    {
        m_userNameWidget = new InputWidget(InputWidgetType::WheelLineEdit);
        m_userNameWidget->setTipText("�û���");

        m_userPswWidget = new InputWidget(InputWidgetType::WheelLineEdit);
        m_userPswWidget->setTipText("����");

        m_userRoleWidget = new InputWidget(InputWidgetType::WheelComboBox);
        m_userRoleWidget->setTipText("�û�Ȩ��");
        m_userRoleWidget->setTipLabelWidth(84);
        m_userRoleWidget->setFixedWidth(215);

        QStringList strRoleList = QStringList() << "��ͨ�û�" << "����Ա" << "��������Ա";
        m_userRoleWidget->setComboBoxContent(strRoleList);

        m_userTelNumberWidget = new InputWidget(InputWidgetType::WheelLineEdit);
        m_userTelNumberWidget->setTipText("����");

        QVBoxLayout* hUserInfoLayout = new QVBoxLayout;
        hUserInfoLayout->addWidget(m_userNameWidget);
        hUserInfoLayout->addWidget(m_userPswWidget);
        hUserInfoLayout->addWidget(m_userRoleWidget);
        hUserInfoLayout->addWidget(m_userTelNumberWidget);
        hUserInfoLayout->setSpacing(10);
        hUserInfoLayout->setMargin(0);

        QPushButton* pButtonAdd = new QPushButton("����");
        pButtonAdd->setFixedSize(QSize(60, 30));
        connect(pButtonAdd, &QPushButton::clicked, this, [=] {
            WheelUserConfig userData;
            userData.user_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
            userData.user_name = m_userNameWidget->getLineEditText();
            userData.user_password = m_userPswWidget->getLineEditText().toUtf8().toBase64();
            userData.user_role = getUserRole(m_userRoleWidget->getComboBoxCurrentContent());
            userData.user_telephone = m_userTelNumberWidget->getLineEditText();
            WHEEL_BACK_TO_CORE_SOCKET.robot_user_config_add_req(userData);
        });

        QPushButton* pButtonDelete = new QPushButton("ɾ��");
        pButtonDelete->setFixedSize(QSize(60, 30));
        connect(pButtonDelete, &QPushButton::clicked, this, [=] {
            int currentRow = m_userTableWidget->getTableWidget()->currentRow();
            if (currentRow >= 0)
            {
                QString strUid = m_userDataList[currentRow].user_uuid;
                WHEEL_BACK_TO_CORE_SOCKET.robot_user_config_delete_req(strUid);
            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "��ʾ", "��ѡ���û�����ɾ��", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        });

        QHBoxLayout* hButtonLayout = new QHBoxLayout;
        hButtonLayout->addStretch();
        hButtonLayout->addWidget(pButtonAdd);
        hButtonLayout->addWidget(pButtonDelete);
        hButtonLayout->setMargin(0);

        m_userTableWidget = new CustomTableWidget(3, false);
        m_userTableWidget->setHorizontalHeaderLabels(QStringList() << "���" << "�û���" << "Ȩ��");
        m_userTableWidget->setIsShowTurnPageWidget(false);
        QTableWidget* tableWidget = m_userTableWidget->getTableWidget();
        tableWidget->setStyleSheet("border:1px solid lightgray;");

        QVBoxLayout* vMainLayout = new QVBoxLayout(this->getCenterWidget());
        vMainLayout->addLayout(hUserInfoLayout);
        vMainLayout->addLayout(hButtonLayout);
        vMainLayout->addWidget(m_userTableWidget);
        vMainLayout->setSpacing(10);
        vMainLayout->setMargin(20);
    }

    // ��ʼ��table����;
    void initTableWidget()
    {
        m_userDataList = WHEEL_STATION_CONFIG.getWheelUserConfigData(m_loginUserType);
        
        QTableWidget* tableWidget = m_userTableWidget->getTableWidget();
        tableWidget->clearContents();
        tableWidget->setRowCount(0);

        for (int i = 0; i < m_userDataList.count(); i++)
        {
            tableWidget->insertRow(i);
            
            QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
            idItem->setTextAlignment(Qt::AlignCenter);
            tableWidget->setItem(i, 0, idItem);

            QTableWidgetItem* userNameItem = new QTableWidgetItem(m_userDataList[i].user_name);
            userNameItem->setTextAlignment(Qt::AlignCenter);
            tableWidget->setItem(i, 1, userNameItem);

            QTableWidgetItem* userRoleItem = new QTableWidgetItem(getUserRoleText(m_userDataList[i].user_role));
            userRoleItem->setTextAlignment(Qt::AlignCenter);
            tableWidget->setItem(i, 2, userRoleItem);

            tableWidget->setRowHeight(i, 40);
        }
    }

    // �������ƻ�ȡ��Ӧ�û�Ȩ��;
    WheelUserType getUserRole(QString userRole)
    {
        WheelUserType role;
        if (userRole == "��ͨ�û�")
        {
            role = WHEEL_USER_NORMAL;
        }
        else if (userRole == "����Ա")
        {
            role = WHEEL_USER_MANAGER;
        }
        else if (userRole == "��������Ա")
        {
            role = WHEEL_USER_SUPER_MANAGER;
        }

        return role;
    }

    // �����û�Ȩ�޻�ȡ����;
    QString getUserRoleText(WheelUserType userRoleType)
    {
        QString strRole;
        switch (userRoleType)
        {
        case WHEEL_USER_NORMAL:
            strRole = "��ͨ�û�";
            break;
        case WHEEL_USER_MANAGER:
            strRole = "����Ա";
            break;
        case WHEEL_USER_SUPER_MANAGER:
            strRole = "��������Ա";
            break;
        default:
            break;
        }

        return strRole;
    }

signals:
    // ����û�����ص�;
    void signalAddNewUserCallBack(bool, QString);
    // ɾ���û�����ص�;
    void signalDeleteUserCallBack(bool, QString);

private:
    CustomTableWidget* m_userTableWidget;

    InputWidget* m_userNameWidget;
    InputWidget* m_userPswWidget;
    InputWidget* m_userRoleWidget;
    InputWidget* m_userTelNumberWidget;

    WheelUserType m_loginUserType;

    QList<WheelUserConfig> m_userDataList;
};

/********Ȩ�޹���ҳ��**********/

class DLWheelPermissionManager : public QWidget
{
	Q_OBJECT

public:
    DLWheelPermissionManager(QWidget* parent = NULL);

    // ��ʼ���ؼ�;
    void initWidget(WheelUserType currentLoginRole);

private:
    // ��ʼ���ؼ�;
    void initTabButtonWidget();
    void initLeftStackedWidget();
    void initRightStackedWidget();
    void initAllocatePeopleTopWidget();
    void initAllocatePeopleBottomWidget();
    void initStackedWidget();

	// ���ڱ��������¼�;
	void paintEvent(QPaintEvent *event);

private:
    // ҳ���л����ؼ�;
    TabButtonWidget * m_firstLevelTabWidget;
    TabButtonWidget* m_secondLevelTabWidget;

    TabButtonWidget* m_leftThirdLevelTabWidget;
    
    TabButtonWidget* m_rightThirdLevelTabWidget;
    TabButtonWidget* m_rightForthLevelTabWidget;

    QWidget* m_tabButtonBackWidget;

    // ���widget;
    QStackedWidget* m_leftStackedWidget;
    QTreeWidget* m_permissionLevelTreeWidget;

    // �Ҳ�widget;
    QStackedWidget* m_rightStackedWidget;

    QWidget* m_allocatePeopleTopBackWidget;  
    QWidget* m_allocateRoleBackWidget;
    QLineEdit* m_staffNumberLineEdit;
    QLineEdit* m_staffNameLineEdit;
    CustomButtonListWidget* m_customButtonListWidget;
    CustomTableWidget* m_allocatePeopleTableWidget;

    TitleWidget* m_allocatePeopleBottomBackWidget;
    QLineEdit* m_userNumberLineEdit;
    QLineEdit* m_userNameLineEdit;
    CustomButtonListWidget* m_customButtonListBottomWidget;

    QWidget* m_stackedBackWidget;

    // �����Ƿ��ʼ��;
    bool m_isInitWidget;

    // Ȩ�޹�����;
    RoleManageWidget* m_roleManageWidget;
};

#endif // DL_WHEEL_PERMISSION_MANAGER_SET_H
