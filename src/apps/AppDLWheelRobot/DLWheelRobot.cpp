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

// 以下代码为关闭cmd窗口;
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
    // 创建dmp文件夹;
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


//去除窗口部件的系统焦点边框;
class MyProxyStyle : public QProxyStyle
{
public:
	virtual void drawPrimitive(PrimitiveElement element, const QStyleOption * option,
		QPainter * painter, const QWidget * widget = 0) const
	{
		if (PE_FrameFocusRect == element)
		{
			//这里不做任何操作，Qt默认是绘制矩形虚线框
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
    // 必须放置第一行，不然获取不到exe路径;
    QApplication a(argc, argv);
    initSingletons();
	
	SetUnhandledExceptionFilter((LPTOP_LEVEL_EXCEPTION_FILTER)ApplicationCrashHandler);

	FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
	FLAGS_logbufsecs = 0; //缓冲日志输出，默认为30秒，此处改为立即输出
	FLAGS_max_log_size = 100; //最大日志大小为 100MB
	FLAGS_stop_logging_if_full_disk = true; //当磁盘被写满时，停止日志输出

    // 创建log文件夹;
    QString strFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/log/" + APP_NAME;
    QDir downloadDir;
    if (!downloadDir.exists(strFilePath))
    {
        downloadDir.mkpath(strFilePath);
    }

	google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_INFO, QString("%1%2").arg(strFilePath).arg("/INFO_").toLocal8Bit().constData());
    google::SetLogDestination(google::GLOG_ERROR, QString("%1%2").arg(strFilePath).arg("/ERROR_").toLocal8Bit().constData()); //设置 google::ERROR 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::GLOG_WARNING, QString("%1%2").arg(strFilePath).arg("/WARN_").toLocal8Bit().constData()); //设置 google::WARNING 级别的日志存储路径和文件名前缀
    google::SetStderrLogging(google::GLOG_INFO);

    google::SetLogFilenameExtension(QString("%1%2").arg(APP_NAME).arg(".log").toLocal8Bit().constData());

	// 创建共享内存, 判断是否已经运行程序;
	QSharedMemory* shareMemory = new QSharedMemory(APP_NAME);
	if (!shareMemory->create(1)) {
		TipWindow * tipWindow = new TipWindow;
		tipWindow->exec();
		return 1;
	}

    // 检测配置文件中Rcf目录是否配置正确;
    QString strRecClientPath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath;
    if (!downloadDir.exists(strRecClientPath))
    {
        bool isSuccess = downloadDir.mkpath(strRecClientPath);
        if (!isSuccess)
        {
            DLMessageBox::showDLMessageBox(NULL, "错误", "Rcf_Client目录创建失败,请检查配置文件", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return 0;
        }
    }

	qApp->setStyle(new MyProxyStyle);
	DLWheelMainWindow w;
	a.exec();
	return 0;
}
