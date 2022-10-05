#include "DLWheelPermissionManager.h"
#include <QHBoxLayout>
#include <QPainter>
#include <QButtonGroup>
#include <QPainter>
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "DLWheelTaskCheck.h"

#define TITLE_WIDGET_HEIGHT 150

DLWheelPermissionManager::DLWheelPermissionManager(QWidget* parent)
    : QWidget(parent)
    , m_isInitWidget(false)
{
    this->setStyleSheet("QWidget#BorderWidget{border:1px solid lightGray;}");
}

void DLWheelPermissionManager::initTabButtonWidget()
{
    m_firstLevelTabWidget = new TabButtonWidget;
    m_firstLevelTabWidget->addTabButton("default", 80);
    m_firstLevelTabWidget->addTabButton("��֯Ȩ�޹���", 120);
    m_firstLevelTabWidget->addTabButtonFinished();

    m_secondLevelTabWidget = new TabButtonWidget;
    m_secondLevelTabWidget->addTabButton("��֯��Ա", 80);
    m_secondLevelTabWidget->addTabButton("��ɫ����", 80);
    m_secondLevelTabWidget->addTabButton("Ȩ�޹���", 80);
    m_secondLevelTabWidget->addTabButtonFinished();
    connect(m_secondLevelTabWidget, &TabButtonWidget::signalButtonClicked, this, [=](int buttonId) {
        switch (buttonId)
        {
        case 0:
            break;
        case 1:
            break;
        case 2:
        {
            m_roleManageWidget->show();
        }
            break;
        default:
            break;
        }
    });

    m_leftThirdLevelTabWidget = new TabButtonWidget;
    m_leftThirdLevelTabWidget->addTabButton("Ȩ�޲��", 80);
    m_leftThirdLevelTabWidget->addTabButtonFinished();
    m_leftThirdLevelTabWidget->setFixedWidth(600);

    m_rightThirdLevelTabWidget = new TabButtonWidget;
    m_rightThirdLevelTabWidget->addTabButton("�ѷ�����Ա - ��ǰ�豸�澯", 200);
    m_rightThirdLevelTabWidget->addTabButton("�ѷ����ɫ - ��ǰ�豸�澯", 200);
    m_rightThirdLevelTabWidget->addTabButtonFinished();

    QHBoxLayout* hThirdLevelLayout = new QHBoxLayout;
    hThirdLevelLayout->addWidget(m_leftThirdLevelTabWidget);
    hThirdLevelLayout->addWidget(m_rightThirdLevelTabWidget);
    hThirdLevelLayout->setSpacing(5);
    hThirdLevelLayout->setMargin(0);

    m_tabButtonBackWidget = new QWidget;
    QVBoxLayout* vTabButtonLayout = new QVBoxLayout(m_tabButtonBackWidget);
    vTabButtonLayout->addWidget(m_firstLevelTabWidget);
    vTabButtonLayout->addWidget(m_secondLevelTabWidget);
    vTabButtonLayout->addLayout(hThirdLevelLayout);
    vTabButtonLayout->setSpacing(3);
    vTabButtonLayout->setMargin(0);
}

void DLWheelPermissionManager::initLeftStackedWidget()
{
    m_leftStackedWidget = new QStackedWidget;
    m_leftStackedWidget->setFixedWidth(600);

    m_permissionLevelTreeWidget = new QTreeWidget;
    m_leftStackedWidget->insertWidget(0, m_permissionLevelTreeWidget);
}

void DLWheelPermissionManager::initRightStackedWidget()
{
    m_rightStackedWidget = new QStackedWidget;

    initAllocatePeopleTopWidget();
    initAllocatePeopleBottomWidget();
    m_allocateRoleBackWidget = new QWidget;

    QVBoxLayout* vLayout = new QVBoxLayout(m_allocateRoleBackWidget);
    vLayout->addWidget(m_allocatePeopleTopBackWidget);
    vLayout->addWidget(m_allocatePeopleBottomBackWidget);
    vLayout->setSpacing(10);
    vLayout->setMargin(0);

    m_rightStackedWidget->insertWidget(0, m_allocateRoleBackWidget);
}

void DLWheelPermissionManager::initAllocatePeopleTopWidget()
{
    m_allocatePeopleTopBackWidget = new QWidget;
    m_allocatePeopleTopBackWidget->setObjectName("BorderWidget");

    m_rightForthLevelTabWidget = new TabButtonWidget;
    m_rightForthLevelTabWidget->addTabButton("�����", 60);
    m_rightForthLevelTabWidget->addTabButton("�б�", 50);
    m_rightForthLevelTabWidget->addTabButtonFinished();

    QLabel* staffNumberLable = new QLabel("����:");

    m_staffNumberLineEdit = new QLineEdit;
    m_staffNumberLineEdit->setFixedWidth(180);

    m_staffNameLineEdit = new QLineEdit;
    m_staffNameLineEdit->setFixedWidth(180);

    QWidget* inputWidget = new QWidget;
    QHBoxLayout* hLayout = new QHBoxLayout(inputWidget);
    hLayout->addWidget(new QLabel("����:"));
    hLayout->addWidget(m_staffNumberLineEdit);
    hLayout->addWidget(new QLabel("����:"));
    hLayout->addWidget(m_staffNameLineEdit);
    hLayout->addStretch();
    hLayout->setSpacing(5);
    hLayout->setContentsMargins(5, 0, 0, 0);

    m_customButtonListWidget = new CustomButtonListWidget;
    m_customButtonListWidget->addWidget(inputWidget);
    m_customButtonListWidget->addToolButton(0, "��ѯ", ":/Resources/Common/image/Search.png");
    m_customButtonListWidget->addToolButton(1, "�ջ���Ա", "");
    m_customButtonListWidget->addToolButton(2, "����Excel", "");
    m_customButtonListWidget->addWidgetFinished();

    m_allocatePeopleTableWidget = new CustomTableWidget(5);
    m_allocatePeopleTableWidget->setHorizontalHeaderLabels(QStringList() << "�к�" << "�û�����" << "�û�����" << "��������" << "������λ");
    m_allocatePeopleTableWidget->setIsShowTurnPageWidget(false);

    QVBoxLayout* vLayout = new QVBoxLayout(m_allocatePeopleTopBackWidget);
    vLayout->addWidget(m_rightForthLevelTabWidget);
    vLayout->addWidget(m_customButtonListWidget);
    vLayout->addWidget(m_allocatePeopleTableWidget);
    vLayout->setSpacing(0);
    vLayout->setMargin(0);
}

void DLWheelPermissionManager::initAllocatePeopleBottomWidget()
{
    m_allocatePeopleBottomBackWidget = new TitleWidget;
    m_allocatePeopleBottomBackWidget->setObjectName("BorderWidget");
    m_allocatePeopleBottomBackWidget->setTitleText("�ɷ�����Ա");

    m_userNumberLineEdit = new QLineEdit;
    m_userNumberLineEdit->setFixedWidth(180);

    m_userNameLineEdit = new QLineEdit;
    m_userNameLineEdit->setFixedWidth(180);

    QWidget* inputWidget = new QWidget;
    QHBoxLayout* hLayout = new QHBoxLayout(inputWidget);
    hLayout->addWidget(new QLabel("�û�����"));
    hLayout->addWidget(m_userNumberLineEdit);
    hLayout->addWidget(new QLabel("�û�����"));
    hLayout->addWidget(m_userNameLineEdit);
    hLayout->addStretch();
    hLayout->setSpacing(5);
    hLayout->setContentsMargins(5, 0, 0, 0);

    m_customButtonListBottomWidget = new CustomButtonListWidget;
    m_customButtonListBottomWidget->addWidget(inputWidget);
    m_customButtonListBottomWidget->addToolButton(0, "��ѯ", ":/Resources/Common/image/Search.png");
    m_customButtonListBottomWidget->addWidgetFinished();

    QTreeWidget* treeWidget = new QTreeWidget;

    QVBoxLayout* vLayout = new QVBoxLayout(m_allocatePeopleBottomBackWidget->getCenterWidget());
    vLayout->addWidget(m_customButtonListBottomWidget);
    vLayout->addWidget(treeWidget);
    vLayout->setSpacing(0);
    vLayout->setMargin(0);
}

void DLWheelPermissionManager::initStackedWidget()
{
    initLeftStackedWidget();
    initRightStackedWidget();

    m_stackedBackWidget = new QWidget;

    QHBoxLayout* hStackedLayout = new QHBoxLayout(m_stackedBackWidget);
    hStackedLayout->addWidget(m_leftStackedWidget);
    hStackedLayout->addWidget(m_rightStackedWidget);
    hStackedLayout->setSpacing(5);
    hStackedLayout->setMargin(0);
}
    

void DLWheelPermissionManager::initWidget(WheelUserType currentLoginRole)
{
    if (!m_isInitWidget)
    {
        m_roleManageWidget = new RoleManageWidget(nullptr, currentLoginRole);
        m_isInitWidget = true;
        initTabButtonWidget();
        initStackedWidget();

        QVBoxLayout* vMainLayout = new QVBoxLayout(this);
        vMainLayout->addWidget(m_tabButtonBackWidget);
        vMainLayout->addWidget(m_stackedBackWidget);
        vMainLayout->setSpacing(3);
        vMainLayout->setContentsMargins(210, 0, 0, 0);
    }
}

void DLWheelPermissionManager::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.fillRect(this->rect(), Qt::white);
}
