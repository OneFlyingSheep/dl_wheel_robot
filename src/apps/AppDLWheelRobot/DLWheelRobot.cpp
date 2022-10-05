 #include "common/DLHangRailRobotGlobalDef.hpp"
#include "LibDLWheelMainWindow/DLWheelMainWindow.h"
#include "LibDLHangRailCommonWidget/ConfigWindow.h"
#include "LibDLHangRailHardwareCtrl/LibDLHangRailHardwareCtrl.h"
#include <QApplication>
#include <QProxyStyle>
#include <DbgHelp.h>
#include <QSharedMemory>
#include <QDebug>
#include <QDir>
#include "LibDLWheelMainWindow/DLWheelRobotManager.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"

#define APP_NAME "WheelRobot"

// ���´���Ϊ�ر�cmd����;
//#pragma comment(linker,"/subsystem:windows /entry:mainCRTStartup")

static LONG ApplicationCrashHandler(EXCEPTION_POINTERS *pException)
{
	//And output crash information
	EXCEPTION_RECORD *record = pException->ExceptionRecord;
	QString errCode(QString::number(record->ExceptionCode, 16));
	QString errAddr(QString::number((uint)record->ExceptionAddress, 16));
	QString errFlag(QString::number(record->ExceptionFlags, 16));
	QString errPara(QString::number(record->NumberParameters, 16));
	qDebug() << "errCode: " << errCode;
	qDebug() << "errAddr: " << errAddr;
	qDebug() << "errFlag: " << errFlag;
	qDebug() << "errPara: " << errPara;
	//Create the dump file
    // ����dmp�ļ���;
    QString strFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/dmp/" + APP_NAME;
    QDir downloadDir;
    if (!downloadDir.exists(strFilePath))
    {
        downloadDir.mkpath(strFilePath);
    }
    QString fileName = strFilePath + "/_crash_" + QDateTime::currentDateTime().toString("yyyy-MM-dd hh-mm-ss") + ".dmp";
	HANDLE hDumpFile = CreateFileW((LPCWSTR)fileName.utf16(),
		GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hDumpFile != INVALID_HANDLE_VALUE) {
		qDebug() << "CreateFileW success" << fileName;
		MINIDUMP_EXCEPTION_INFORMATION dumpInfo;
		dumpInfo.ExceptionPointers = pException;
		dumpInfo.ThreadId = GetCurrentThreadId();
		dumpInfo.ClientPointers = TRUE;
		MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hDumpFile, MiniDumpWithFullMemory, &dumpInfo, NULL, NULL);
		CloseHandle(hDumpFile);
		return EXCEPTION_CONTINUE_SEARCH;
	}

	qDebug() << "CreateFileW error" << fileName;
	return EXCEPTION_EXECUTE_HANDLER;
}


//ȥ�����ڲ�����ϵͳ����߿�;
class MyProxyStyle : public QProxyStyle
{
public:
	virtual void drawPrimitive(PrimitiveElement element, const QStyleOption * option,
		QPainter * painter, const QWidget * widget = 0) const
	{
		if (PE_FrameFocusRect == element)
		{
			//���ﲻ���κβ�����QtĬ���ǻ��ƾ������߿�
		}
		else
		{
			QProxyStyle::drawPrimitive(element, option, painter, widget);
		}
	}
};

void initSingletons()
{
    WHEEL_ROBOT_BACKGROUND_CONFIG.loadFromFile("");
}

int main(int argc, char *argv[])
{
    // ������õ�һ�У���Ȼ��ȡ����exe·��;
    QApplication a(argc, argv);
    initSingletons();
	
	SetUnhandledExceptionFilter((LPTOP_LEVEL_EXCEPTION_FILTER)ApplicationCrashHandler);

	FLAGS_colorlogtostderr = true; //�����������Ļ����־��ʾ��Ӧ��ɫ
	FLAGS_logbufsecs = 0; //������־�����Ĭ��Ϊ30�룬�˴���Ϊ�������
	FLAGS_max_log_size = 100; //�����־��СΪ 100MB
	FLAGS_stop_logging_if_full_disk = true; //�����̱�д��ʱ��ֹͣ��־���

    // ����log�ļ���;
    QString strFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/log/" + APP_NAME;
    QDir downloadDir;
    if (!downloadDir.exists(strFilePath))
    {
        downloadDir.mkpath(strFilePath);
    }

	google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_INFO, QString("%1%2").arg(strFilePath).arg("/INFO_").toLocal8Bit().constData());
    google::SetLogDestination(google::GLOG_ERROR, QString("%1%2").arg(strFilePath).arg("/ERROR_").toLocal8Bit().constData()); //���� google::ERROR �������־�洢·�����ļ���ǰ׺
    google::SetLogDestination(google::GLOG_WARNING, QString("%1%2").arg(strFilePath).arg("/WARN_").toLocal8Bit().constData()); //���� google::WARNING �������־�洢·�����ļ���ǰ׺
    google::SetStderrLogging(google::GLOG_INFO);

    google::SetLogFilenameExtension(QString("%1%2").arg(APP_NAME).arg(".log").toLocal8Bit().constData());

	// ���������ڴ�, �ж��Ƿ��Ѿ����г���;
	QSharedMemory* shareMemory = new QSharedMemory(APP_NAME);
	if (!shareMemory->create(1)) {
		TipWindow * tipWindow = new TipWindow;
		tipWindow->exec();
		return 1;
	}

    // ��������ļ���RcfĿ¼�Ƿ�������ȷ;
    QString strRecClientPath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath;
    if (!downloadDir.exists(strRecClientPath))
    {
        bool isSuccess = downloadDir.mkpath(strRecClientPath);
        if (!isSuccess)
        {
            DLMessageBox::showDLMessageBox(NULL, "����", "Rcf_ClientĿ¼����ʧ��,���������ļ�", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return 0;
        }
    }

	qApp->setStyle(new MyProxyStyle);
	DLWheelMainWindow w;
	a.exec();
	return 0;
}
