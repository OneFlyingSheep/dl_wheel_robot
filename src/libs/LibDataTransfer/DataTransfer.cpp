#include "Datatransfer.h"
//#include <QDebug>
#include "LibProtoClient/ProtoClient.h"
#include "LibProtoServer/ProtoServer.h"
#include <iostream>
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"

DataTransfer::DataTransfer(QObject* parent)
{
}


void DataTransfer::run()
{
    Sleep(3000);
	switch (pro_.cmd_type_)
	{
	case DOWNLOAD_FILE:
	{
		//qDebug() << "+++++++++++++download:" << pro_.download_path_.c_str();
		int ret = GFILE_TRANSFER_CLIENT.downloadFile(pro_.dst_relative_path_, pro_.file_name_, pro_.download_path_);
        emit sig_finished(pro_.cmd_type_, ret, QString::fromLocal8Bit(pro_.file_name_.c_str()));
        if (m_iCheck == 1)
        {
            emit sig_finished_infrared(QString::fromStdString(pro_.download_path_), QString::fromStdString(pro_.file_name_));
        }
        break;
	}
	case UPLOAD_FILE:
	{
		int ret = GFILE_TRANSFER_CLIENT.uploadFile(pro_.src_file_path_, pro_.dst_relative_path_);
		emit sig_finished(pro_.cmd_type_, ret, QString::fromLocal8Bit(pro_.file_name_.c_str()));
		break;
	}
	default:
		break; 
	}
 
}


void DataTransfer::set_transfer_info(TransferPro pro)
{
	pro_ = pro;
}

void DataTransfer::set_infrared_type(int check)
{
    m_iCheck = check;
}


