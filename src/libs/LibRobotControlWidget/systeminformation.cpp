
/// 机器人控制界面程序

#include "systeminformation.h"

SystemInformation::SystemInformation(QWidget *parent)
	: QWidget(parent), m_totalDiskNum(0), m_surplusDiskNum(0)
{
	getDiskSizeFunction();

	qDebug() << "m_totalDiskNum = " << m_totalDiskNum << "\t m_surplusDiskNum = " << m_surplusDiskNum;

	initWidget();
}

SystemInformation::~SystemInformation()
{

}

void SystemInformation::initWidget()
{
	QVBoxLayout* nMainLayout = new QVBoxLayout;

	QLabel* labelSystemInfo = new QLabel("系统信息");
	labelSystemInfo->setAlignment(Qt::AlignCenter);
	
	QWidget* nInfoWidget = new QWidget;
	nInfoWidget->setStyleSheet(".QWidget{border: 2px solid rgb(0, 110, 110);border-radius:8px;padding-top:3px;}");
								//QLabel{padding-right:2px; width:200}");

	QLabel* nLabelNetWork = new QLabel("网络信息:");
	nLabelNetWork->setAlignment(Qt::AlignRight);
	QLabel* nLineEditNetWork = new QLabel;
	Signalsintensity nSignalsIntensity;
	nLineEditNetWork->setPixmap(nSignalsIntensity.GetPixmap(10));
	QHBoxLayout* nLayoutNerWork = new QHBoxLayout;
	nLayoutNerWork->addWidget(nLabelNetWork);
	nLayoutNerWork->addWidget(nLineEditNetWork);

	QLabel* nLabelDiskInfo = new QLabel("磁盘信息:");
	nLabelDiskInfo->setAlignment(Qt::AlignRight);
	DiskInfoItem* nLineEditDiskInfo = new DiskInfoItem;
	nLineEditDiskInfo->setValue(m_totalDiskNum, m_totalDiskNum -m_surplusDiskNum);
	QHBoxLayout* nLayoutDiskInfo = new QHBoxLayout;
	nLayoutDiskInfo->addWidget(nLabelDiskInfo);
	nLayoutDiskInfo->addWidget(nLineEditDiskInfo);

	//int value = m_totalDiskNum - m_surplusDiskNum;
	QLabel* nLabelDiskTotle = new QLabel("磁盘总空间(GB):");
	nLabelDiskTotle->setAlignment(Qt::AlignRight);
	QLabel* nLabelDiskTotleNum = new QLabel(QString::number(m_totalDiskNum));
	QHBoxLayout* nLayoutDiskNum = new QHBoxLayout;
	nLayoutDiskNum->addWidget(nLabelDiskTotle);
	nLayoutDiskNum->addWidget(nLabelDiskTotleNum);

	QLabel* nLabelDiskSurplus = new QLabel("磁盘剩余空间(GB):");
	nLabelDiskSurplus->setAlignment(Qt::AlignRight);
	QLabel* nLabelDiskSurplusNum = new QLabel(QString::number(m_surplusDiskNum));
	QHBoxLayout* nLayoutDiskSurplus = new QHBoxLayout;
	nLayoutDiskSurplus->addWidget(nLabelDiskSurplus);
	nLayoutDiskSurplus->addWidget(nLabelDiskSurplusNum);


	nMainLayout->addLayout(nLayoutNerWork);
	nMainLayout->addLayout(nLayoutDiskInfo);
	nMainLayout->addLayout(nLayoutDiskNum);
	nMainLayout->addLayout(nLayoutDiskSurplus);

	nInfoWidget->setLayout(nMainLayout);

	QVBoxLayout* nLayoutThis = new QVBoxLayout;
	nLayoutThis->addWidget(labelSystemInfo);
	nLayoutThis->addWidget(nInfoWidget);

	this->setLayout(nLayoutThis);
}

//获取磁盘容量
quint64 SystemInformation::getDiskSpace(QString iDriver, bool flag)
{
	ULARGE_INTEGER freeDiskSpaceAvailable, totalDiskSpace, totalFreeDiskSpace;

	//调用函数获取磁盘参数(单位为字节Byte),转化为GB，需要除以(1024*1024*1024)
	GetDiskFreeSpaceEx(iDriver.toStdString().c_str(), &freeDiskSpaceAvailable, &totalDiskSpace, &totalFreeDiskSpace);
	if (flag)
	{
		return (quint64)totalDiskSpace.QuadPart / GB;
	}
	else
	{
		return (quint64)totalFreeDiskSpace.QuadPart / GB;
	}
}

//获取电脑所有盘符名
QStringList SystemInformation::getDiskName()
{
	QFileInfoList list = QDir::drives();
	QStringList diskNameList(NULL);

	for (int i = 0; i < list.count(); i++)
	{

		QString str = list.at(i).absoluteDir().absolutePath();
		diskNameList.append(str);
	}
	return diskNameList;
}

void SystemInformation::getDiskSizeFunction()
{
	QStringList diskList = getDiskName(); 
	foreach(QString str, diskList)
	{
		if (str.isEmpty())
		{
			continue;
		}
		quint64 totalDiskSpace = getDiskSpace(str, true);
		quint64 freeDiskSpace  = getDiskSpace(str, false);
		//qDebug() << "盘符" + str + "【总容量:" << totalDiskSpace << "\t剩余容量:" << freeDiskSpace << "】";

		m_totalDiskNum	+= totalDiskSpace;
		m_surplusDiskNum += freeDiskSpace;
	}
}

