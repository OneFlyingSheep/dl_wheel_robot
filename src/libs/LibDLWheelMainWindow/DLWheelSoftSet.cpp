#include "DLWheelSoftSet.h"
#include <QHBoxLayout>
#include "LibDLHangRailCommonTools/DLHangRailCommonTools.h"
#include <QPainter>
#include <QtWebEngineWidgets/QWebEngineView>
#include <QtWebChannel/QWebChannel>
#include <QApplication>
#include <QFile>

DLWheelSoftSet::DLWheelSoftSet(QWidget* parent /* = 0 */)
	: QWidget(parent)
	, m_isInitWidget(false)
{
	DLHangRailCommonTools::loadStyleSheet(this, ":/Resources/DLWheelSoftSet/DLWheelSoftSet.css");
}

void DLWheelSoftSet::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initTopWidget();
	initBottomBackWidget();
    initCenterWidget();

	QVBoxLayout* vRightLayout = new QVBoxLayout;
	vRightLayout->addWidget(m_topBackWidget);
	vRightLayout->addWidget(m_centerBackWidget);
	vRightLayout->addWidget(m_bottomBackWidget);
	vRightLayout->setSpacing(0);
	vRightLayout->setMargin(0);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addSpacing(210);
	hMainLayout->addLayout(vRightLayout);
	hMainLayout->setSpacing(0);
	hMainLayout->setMargin(0);
}

void DLWheelSoftSet::initTopWidget()
{
	m_topBackWidget = new QWidget;
	m_topBackWidget->setObjectName("TopBackWidget");
	m_topBackWidget->setFixedHeight(35);

	m_pButtonPrint = new QToolButton;
	m_pButtonPrint->setText("打印");
	m_pButtonPrint->setFixedSize(QSize(65, 30));
	m_pButtonPrint->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonPrint->setIcon(QIcon(":/Resources/DLWheelSoftSet/PrintButton.png"));
	m_pButtonPrint->setIconSize(QSize(QSize(30, 30)));

	m_pButtonBackUp = new QToolButton;
	m_pButtonBackUp->setText("后退");
	m_pButtonBackUp->setFixedSize(QSize(60, 30));
	m_pButtonBackUp->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonBackUp->setIcon(QIcon(":/Resources/DLWheelSoftSet/BackUpButton.png"));
	m_pButtonBackUp->setIconSize(QSize(QSize(20, 20)));

	m_pButtonGoForward = new QToolButton;
	m_pButtonGoForward->setText("前进");
	m_pButtonGoForward->setFixedSize(QSize(60, 30));
	m_pButtonGoForward->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonGoForward->setIcon(QIcon(":/Resources/DLWheelSoftSet/GoForwardButton.png"));
	m_pButtonGoForward->setIconSize(QSize(QSize(20, 20)));

    m_pButtonReload = new QToolButton;
    m_pButtonReload->setText("重新加载");
    m_pButtonReload->setFixedSize(QSize(80, 30));
    m_pButtonReload->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    m_pButtonReload->setIcon(QIcon(":/Resources/Common/image/Reset.png"));
    m_pButtonReload->setIconSize(QSize(QSize(20, 20)));
    connect(m_pButtonReload, &QToolButton::clicked, this, [=] {
        QString strFilePath = QApplication::applicationDirPath() + "/ConfigData/WheelRobotCfg/SoftSet.md";
        QFile defaultTextFile(strFilePath);
        defaultTextFile.open(QIODevice::ReadOnly);
        m_content.setText(QString::fromLocal8Bit(defaultTextFile.readAll()));
    });
    
	m_searchLineEdit = new QLineEdit;
	m_searchLineEdit->setFixedSize(QSize(300, 25));
	m_searchLineEdit->setPlaceholderText("请输入...");

	m_pButtonSearch = new QToolButton;
	m_pButtonSearch->setText("搜索");
	m_pButtonSearch->setObjectName("SearchButton");
	m_pButtonSearch->setFixedSize(QSize(90, 25));
	m_pButtonSearch->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonSearch->setIcon(QIcon(":/Resources/DLWheelSoftSet/SearchButton.png"));
	m_pButtonSearch->setIconSize(QSize(QSize(25, 25)));

	QHBoxLayout* hTopLayout = new QHBoxLayout(m_topBackWidget);
	hTopLayout->addWidget(m_pButtonPrint);
	hTopLayout->addWidget(m_pButtonBackUp);
	hTopLayout->addWidget(m_pButtonGoForward);
    hTopLayout->addWidget(m_pButtonReload);
	hTopLayout->addStretch();
	hTopLayout->addWidget(m_searchLineEdit);
	hTopLayout->addWidget(m_pButtonSearch);
	hTopLayout->setMargin(5);
	hTopLayout->setSpacing(5);
}

void DLWheelSoftSet::initBottomBackWidget()
{
	m_bottomBackWidget = new QWidget;
	m_bottomBackWidget->setFixedHeight(35);

	m_pButtonUserManual = new QPushButton;
	m_pButtonUserManual->setFixedSize(QSize(75, 25));
	m_pButtonUserManual->setText("使用手册");

	m_pButtonCommonProblem = new QPushButton;
	m_pButtonCommonProblem->setFixedSize(QSize(75, 25));
	m_pButtonCommonProblem->setText("常见问题");

	QHBoxLayout* hBottomLayout = new QHBoxLayout(m_bottomBackWidget);
	hBottomLayout->addStretch();
	hBottomLayout->addWidget(m_pButtonUserManual);
	hBottomLayout->addWidget(m_pButtonCommonProblem);
	hBottomLayout->setSpacing(5);
	hBottomLayout->setMargin(5);
}

void DLWheelSoftSet::initCenterWidget()
{
    m_centerBackWidget = new QWidget;
    m_centerBackWidget->setObjectName("CenterBackWidget");

    QWebEngineView* m_webEngineView = new QWebEngineView;
    m_webEngineView->setContextMenuPolicy(Qt::NoContextMenu);
    QWebEnginePage* page = new QWebEnginePage(this);
    m_webEngineView->setPage(page);

    QWebChannel *channel = new QWebChannel(this);
    channel->registerObject(QStringLiteral("content"), &m_content);
    page->setWebChannel(channel);

    m_webEngineView->setUrl(QUrl("qrc:/Resources/WebView/MarkDown/index.html"));

    QString strFilePath = QApplication::applicationDirPath() + "/ConfigData/WheelRobotCfg/SoftSet.md";
    QFile defaultTextFile(strFilePath);
    defaultTextFile.open(QIODevice::ReadOnly);
    m_content.setText(defaultTextFile.readAll());

    QHBoxLayout* hLayout = new QHBoxLayout(m_centerBackWidget);
    hLayout->addWidget(m_webEngineView);
    hLayout->addStretch();
    hLayout->setSpacing(0);
    hLayout->setMargin(5);
}

void DLWheelSoftSet::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), Qt::white);
}