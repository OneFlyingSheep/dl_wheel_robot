#include "common/DLWheelRobotGlobalDef.hpp"
#include "common/DLWheelRobotFSM.hpp"
#include <LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreServer.h>
#include <LibDLWheelRobotTask/LibDLWheelRobotTask.h>
#include <QCoreApplication>
#include <QSharedMemory>
#include <QDateTime>
#include <DbgHelp.h>
#include "LibProtoServer/ProtoServer.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include "common/DLWheelRobotSimpleFSM.hpp"
#include "boost/filesystem.hpp"
#include "LibConvertSymbolsToLua/LibConvertSymbolsToLua.h"
#include <QSharedMemory>
#include "LibDLWheelIntelligentMainWiindow/DLWheelIntelligentMainWiindow.h"
#include "LibDLWheelRobotBimCore/LibDLWheelRobotBimCore.h"


#define APP_WHEEL_ROBOT_CORE_NAME "WheelRailRobotCore"

static LONG ApplicationCrashHandler(EXCEPTION_POINTERS *pException)
{
    //And output crash information
    EXCEPTION_RECORD *record = pException->ExceptionRecord;
    QString errCode(QString::number(record->ExceptionCode, 16));
    QString errAddr(QString::number((uint)record->ExceptionAddress, 16));
    QString errFlag(QString::number(record->ExceptionFlags, 16));
    QString errPara(QString::number(record->NumberParameters, 16));
    //Create the dump file
    QString root_path = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath.replace("/", "\\") + "\\dmp\\" + APP_WHEEL_ROBOT_CORE_NAME + "\\";

    boost::filesystem::create_directories(boost::filesystem::path(std::string(root_path.replace("\\", "/").toLocal8Bit())));


    QString fileName = root_path + "crash_" + QDateTime::currentDateTime().toString("yyyy-MM-dd hh-mm-ss") + ".dmp";
    HANDLE hDumpFile = CreateFileW((LPCWSTR)fileName.utf16(),
        GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hDumpFile != INVALID_HANDLE_VALUE) {
        MINIDUMP_EXCEPTION_INFORMATION dumpInfo;
        dumpInfo.ExceptionPointers = pException;
        dumpInfo.ThreadId = GetCurrentThreadId();
        dumpInfo.ClientPointers = TRUE;
        MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hDumpFile, MiniDumpWithFullMemory, &dumpInfo, NULL, NULL);
        CloseHandle(hDumpFile);
    }
    return EXCEPTION_EXECUTE_HANDLER;
}

void initSingletons()
{
    FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
    FLAGS_logbufsecs = 0; //缓冲日志输出，默认为30秒，此处改为立即输出
    FLAGS_max_log_size = 100; //最大日志大小为 100MB
    FLAGS_stop_logging_if_full_disk = true; //当磁盘被写满时，停止日志输出

    QString root_path;
    std::string info_log_path;
    std::string err_log_path;
    std::string warn_log_path;

    root_path = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath.replace("/", "\\") + "\\log\\" + APP_WHEEL_ROBOT_CORE_NAME + "\\";
    info_log_path = root_path.toLocal8Bit() + "\\INFO_";
    err_log_path  = root_path.toLocal8Bit() + "\\ERROR_";
    warn_log_path = root_path.toLocal8Bit() + "\\WARN_";
    std::string extension_name = QString("_" + QString(APP_WHEEL_ROBOT_CORE_NAME).toLocal8Bit() + ".log").toLocal8Bit();

    boost::filesystem::create_directories(boost::filesystem::path(std::string(root_path.replace("\\", "/").toLocal8Bit())));

    google::InitGoogleLogging("Wheel_Core");
    google::SetLogDestination(google::GLOG_INFO, info_log_path.c_str());
    google::SetLogDestination(google::GLOG_ERROR, err_log_path.c_str()); //设置 google::ERROR 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::GLOG_WARNING, warn_log_path.c_str()); //设置 google::WARNING 级别的日志存储路径和文件名前缀
    google::SetStderrLogging(google::GLOG_ERROR);
    google::SetLogFilenameExtension(extension_name.c_str());

    WHEEL_ROBOT_DB.openDb(WHEEL_ROBOT_CORE_CONFIG.getCfg().databaseLocal,
        WHEEL_ROBOT_CORE_CONFIG.getCfg().databaseName,
        WHEEL_ROBOT_CORE_CONFIG.getCfg().databaseUsername,
        WHEEL_ROBOT_CORE_CONFIG.getCfg().databasePassword/*.toUtf8()*/,
        WHEEL_ROBOT_CORE_CONFIG.getCfg().databasePort);
}

int main(int argc, char *argv[])
{
    // 创建共享内存, 判断是否已经运行程序;
    QSharedMemory* shareMemory = new QSharedMemory(APP_WHEEL_ROBOT_CORE_NAME);
    if (!shareMemory->create(1)) {
        ROS_ERROR("已有进程在执行，请先结束");
        return 1;
    }

    SetUnhandledExceptionFilter((LPTOP_LEVEL_EXCEPTION_FILTER)ApplicationCrashHandler);

//    QApplication a(argc, argv);
    QCoreApplication a(argc, argv);

    initSingletons();

    HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
    DWORD prev_mode;
    GetConsoleMode(hStdin, &prev_mode);
    SetConsoleMode(hStdin, prev_mode & ~ENABLE_QUICK_EDIT_MODE);

    RCF::RcfInitDeinit rcfInit;
	RCF::setDefaultMaxMessageLength(64*1024*1024);
    FileTranseferServer server;

    try
    {
        server.initServer(WHEEL_ROBOT_CORE_CONFIG.getCfg().coreServerIp.toStdString(), WHEEL_ROBOT_CORE_CONFIG.getCfg().rcfServerPort);
        server.setRootDir(std::string(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath.toLocal8Bit()));
        server.bindDownloadProcess();
        server.bindUploadProcess();
        server.start();
    }
    catch (const RCF::Exception & e)
    {
		ROS_ERROR("rcf server init failed, err: %s", e.getError().getErrorString().c_str());
        return 1;
    }


    LibDLWheelRobotTask m_myTask;
	LibDLWheelRobotBimCore m_bim_core;
	m_bim_core.RunMessageLoop(WHEEL_ROBOT_CORE_CONFIG.getCfg().temporaryDoorPort);

	m_myTask.signal_inspect_notify.connect(boost::bind(&LibDLWheelRobotBimCore::slot_inspectResultNotify, &m_bim_core, _1));
	//m_myTask.signal_taskCollectFinish.connect(boost::bind(&LibDLWheelRobotTask::slot_taskCollectFinish, &m_myTask, _1));
	//m_myTask.signal_taskCollectFinish.connect(boost::bind(&LibDLWheelRobotTask::slot_taskCollectFinish, &m_myTask, _1));
	WHEEL_CORE_SERVER.robotBodyInfoSignal.connect(boost::bind(&LibDLWheelRobotBimCore::slot_robotBodyInfo, &m_bim_core, _1, _2));
	WHEEL_CORE_SERVER.robotEnvInfoSignal.connect(boost::bind(&LibDLWheelRobotBimCore::slot_robotEnvInfo, &m_bim_core, _1, _2));
	WHEEL_CORE_SERVER.robotTaskTrajSignal.connect(boost::bind(&LibDLWheelRobotBimCore::slot_robotTaskTrajInfo, &m_bim_core, _1, _2));
	WHEEL_CORE_SERVER.robotPosSignal.connect(boost::bind(&LibDLWheelRobotBimCore::slot_robotPosInfo, &m_bim_core, _1, _2));
	WHEEL_CORE_SERVER.robotPatrolStatusSignal.connect(boost::bind(&LibDLWheelRobotBimCore::slot_robotStatusInfo, &m_bim_core, _1, _2));

    WHEEL_CORE_SERVER.signal_taskCollectFinish.connect(boost::bind(&LibDLWheelRobotTask::slot_taskCollectFinish, &m_myTask, _1));
    WHEEL_CORE_SERVER.signal_updateTimedTask.connect(boost::bind(&LibDLWheelRobotTask::slot_updateTimedTask, &m_myTask));
    WHEEL_CORE_SERVER.signal_deviceUploadFinish.connect(boost::bind(&LibDLWheelRobotTask::slot_deviceUploadFinish, &m_myTask, _1));

    WHEEL_CORE_SERVER.wheelRobotTaskCallback.connect(boost::bind(&LibDLWheelRobotTask::slot_setCurrentTask, &m_myTask, _1));
    WHEEL_CORE_SERVER.wheelRobotCurrentDeviceCallback.connect(boost::bind(&LibDLWheelRobotTask::slot_setCurrentDevice, &m_myTask, _1, _2, _3));
	
    WHEEL_CORE_SERVER.countTodyTaskCallback.connect(boost::bind(&LibDLWheelRobotTask::slot_countTodayTasks, &m_myTask));
    
    WHEEL_CORE_SERVER.startServer();

    a.exec();
    google::ShutdownGoogleLogging();
    return 0;
}
