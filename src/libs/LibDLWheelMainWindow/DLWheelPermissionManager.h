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

/*********tab按钮控件*********/

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

    // 添加tab按钮;
    void addTabButton(QString strButtonText, int buttonWidth)
    {
        QPushButton* tabButton = new QPushButton(strButtonText);
        tabButton->setFixedSize(QSize(buttonWidth, 20));
        tabButton->setCheckable(true);
        m_tabButtonGroup->addButton(tabButton, m_tabButtonGroup->buttons().count());
    }

    // 添加tab完成之后调用此方法，用于按钮布局;
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
    // tab按钮点击;
    void signalButtonClicked(int buttonId);

private:
    // 边框绘制;
    void paintEvent(QPaintEvent *event)
    {
        QPainter painter(this);
        painter.fillRect(this->rect(), QColor(185, 225, 210));
    }

private:
    QButtonGroup* m_tabButtonGroup;

};

/*****权限管理窗口********/

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
        this->setTitleContent("权限管理");
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
                DLMessageBox::showDLMessageBox(NULL, "错误", "添加失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        });

        connect(this, &RoleManageWidget::signalDeleteUserCallBack, this, [=](bool isSuccess, QString strMsg) {
            if (isSuccess)
            {
                initTableWidget();

            }
            else
            {
                DLMessageBox::showDLMessageBox(NULL, "错误", "删除失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        });
        

        this->setStyleSheet("QPushButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(145, 167, 255);border-radius:3px;}\
						QPushButton:pressed{padding-left:2px;padding-top:2px;}\
						QWidget#TitleBackWidget{border-top:3px solid rgb(145, 167, 255); }");
    }

    // 初始化控件;
    void initWidget()
    {
        m_userNameWidget = new InputWidget(InputWidgetType::WheelLineEdit);
        m_userNameWidget->setTipText("用户名");

        m_userPswWidget = new InputWidget(InputWidgetType::WheelLineEdit);
        m_userPswWidget->setTipText("密码");

        m_userRoleWidget = new InputWidget(InputWidgetType::WheelComboBox);
        m_userRoleWidget->setTipText("用户权限");
        m_userRoleWidget->setTipLabelWidth(84);
        m_userRoleWidget->setFixedWidth(215);

        QStringList strRoleList = QStringList() << "普通用户" << "管理员" << "超级管理员";
        m_userRoleWidget->setComboBoxContent(strRoleList);

        m_userTelNumberWidget = new InputWidget(InputWidgetType::WheelLineEdit);
        m_userTelNumberWidget->setTipText("号码");

        QVBoxLayout* hUserInfoLayout = new QVBoxLayout;
        hUserInfoLayout->addWidget(m_userNameWidget);
        hUserInfoLayout->addWidget(m_userPswWidget);
        hUserInfoLayout->addWidget(m_userRoleWidget);
        hUserInfoLayout->addWidget(m_userTelNumberWidget);
        hUserInfoLayout->setSpacing(10);
        hUserInfoLayout->setMargin(0);

        QPushButton* pButtonAdd = new QPushButton("新增");
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

        QPushButton* pButtonDelete = new QPushButton("删除");
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
                DLMessageBox::showDLMessageBox(NULL, "提示", "请选择用户进行删除", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            }
        });

        QHBoxLayout* hButtonLayout = new QHBoxLayout;
        hButtonLayout->addStretch();
        hButtonLayout->addWidget(pButtonAdd);
        hButtonLayout->addWidget(pButtonDelete);
        hButtonLayout->setMargin(0);

        m_userTableWidget = new CustomTableWidget(3, false);
        m_userTableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "用户名" << "权限");
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

    // 初始化table数据;
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

    // 根据名称获取对应用户权限;
    WheelUserType getUserRole(QString userRole)
    {
        WheelUserType role;
        if (userRole == "普通用户")
        {
            role = WHEEL_USER_NORMAL;
        }
        else if (userRole == "管理员")
        {
            role = WHEEL_USER_MANAGER;
        }
        else if (userRole == "超级管理员")
        {
            role = WHEEL_USER_SUPER_MANAGER;
        }

        return role;
    }

    // 根据用户权限获取名称;
    QString getUserRoleText(WheelUserType userRoleType)
    {
        QString strRole;
        switch (userRoleType)
        {
        case WHEEL_USER_NORMAL:
            strRole = "普通用户";
            break;
        case WHEEL_USER_MANAGER:
            strRole = "管理员";
            break;
        case WHEEL_USER_SUPER_MANAGER:
            strRole = "超级管理员";
            break;
        default:
            break;
        }

        return strRole;
    }

signals:
    // 添加用户结果回调;
    void signalAddNewUserCallBack(bool, QString);
    // 删除用户结果回调;
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

/********权限管理页面**********/

class DLWheelPermissionManager : public QWidget
{
	Q_OBJECT

public:
    DLWheelPermissionManager(QWidget* parent = NULL);

    // 初始化控件;
    void initWidget(WheelUserType currentLoginRole);

private:
    // 初始化控件;
    void initTabButtonWidget();
    void initLeftStackedWidget();
    void initRightStackedWidget();
    void initAllocatePeopleTopWidget();
    void initAllocatePeopleBottomWidget();
    void initStackedWidget();

	// 窗口背景绘制事件;
	void paintEvent(QPaintEvent *event);

private:
    // 页面中基本控件;
    TabButtonWidget * m_firstLevelTabWidget;
    TabButtonWidget* m_secondLevelTabWidget;

    TabButtonWidget* m_leftThirdLevelTabWidget;
    
    TabButtonWidget* m_rightThirdLevelTabWidget;
    TabButtonWidget* m_rightForthLevelTabWidget;

    QWidget* m_tabButtonBackWidget;

    // 左侧widget;
    QStackedWidget* m_leftStackedWidget;
    QTreeWidget* m_permissionLevelTreeWidget;

    // 右侧widget;
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

    // 窗口是否初始化;
    bool m_isInitWidget;

    // 权限管理窗口;
    RoleManageWidget* m_roleManageWidget;
};

#endif // DL_WHEEL_PERMISSION_MANAGER_SET_H
