#include "ProtoServer.h"
#include <iostream>
#include<direct.h>
#include "common/DLRobotCommonDef.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>


//回调函数
void onServerFileUpload(RCF::RcfSession &session, const RCF::FileUploadInfo &uploadInfo)
{
	const RCF::FileManifest &fileManifest = uploadInfo.mManifest;
	bool isCompeted = uploadInfo.mCompleted;
	boost::uint64_t currentFilePos = uploadInfo.mCurrentPos;
	ROS_INFO("rcfserver upload, currentFile:%d, currentFilePos:%d!", isCompeted, currentFilePos);
}


void onServerFileDown(RCF::RcfSession &session, const RCF::FileDownloadInfo &downloadInfo)
{
	const RCF::FileManifest &fileManifest = downloadInfo.mManifest;
	boost::uint32_t currentFile = downloadInfo.mCurrentFile;
	boost::uint64_t currentFilePos = downloadInfo.mCurrentPos;
	ROS_INFO("rcfserver downlaod, currentFile:%d, currentFilePos:%d!", currentFile, currentFilePos);
} 


/////////////////////////////
FileTranseferImpl::FileTranseferImpl()
{
	
}

FileTranseferImpl::~FileTranseferImpl()
{
}


bool FileTranseferImpl::createPath(const std::string &relative_path)
{
	bool ret = false;
	boost::filesystem::path file_path;
	try
	{
		file_path = boost::filesystem::path(root_path_ + "/" + relative_path, boost::filesystem::native);
		if (!boost::filesystem::exists(file_path)) {
			ret = boost::filesystem::create_directories(file_path);
			return ret;
		}
		else {
			return true;
		}
	}
	catch (const std::exception& e)
	{
		ROS_ERROR("createPath:%s, err:%s!!", file_path.string().c_str(), e.what());
		return ret;
	}
	
}


bool FileTranseferImpl::CopyDirectory(const std::string &strSourceDir, const std::string &strDestDir)
{
	boost::filesystem::recursive_directory_iterator end;
	boost::system::error_code ec;

	if (!boost::filesystem::exists(strDestDir)) {
		boost::filesystem::create_directories(strDestDir);
	}

	for (boost::filesystem::recursive_directory_iterator pos(strSourceDir); pos != end; ++pos)
	{
		std::string strAppPath = boost::filesystem::path(*pos).string();
		std::string strRestorePath;

		boost::replace_first_copy(std::back_inserter(strRestorePath), strAppPath, strSourceDir, strDestDir);

		if (boost::filesystem::is_directory(strAppPath)) {
			boost::filesystem::create_directories(boost::filesystem::path(strRestorePath), ec);
		}
		else {
			if (!boost::filesystem::exists(strRestorePath)) {
			//	boost::filesystem::copy_file(strAppPath, strRestorePath, boost::filesystem::copy_option::overwrite_if_exists, ec);
				boost::filesystem::rename(strAppPath, strRestorePath);
				continue;
			}
			if (boost::filesystem::exists(strRestorePath) && boost::filesystem::last_write_time(strAppPath) != boost::filesystem::last_write_time(strRestorePath)) {
			//	boost::filesystem::copy_file(strAppPath, strRestorePath, boost::filesystem::copy_option::overwrite_if_exists, ec);
				boost::filesystem::rename(strAppPath, strRestorePath);
			}
		}
	}
	if (ec)
	{
		std::string mm = ec.message().c_str();
		ROS_INFO("upload dir:%s", ec.message().c_str());
		return false;
	}
	return true;
}


int FileTranseferImpl::dumpDirpath(const std::string & relative_path, const std::string &filePath)
{
	//创建相对路径及目录
	if (!createPath(relative_path)) {
		return -1;
	}

	boost::filesystem::path temp_file_path(filePath);
	std::string parent_path = temp_file_path.parent_path().string();
	std::string child_path = temp_file_path.filename().string();
	std::string dst_file = root_path_ + "/" + relative_path + "/" + child_path;

	//拷贝源文件目的目录
	if (boost::filesystem::is_directory(filePath)) 
	{
		CopyDirectory(filePath, dst_file);
		ROS_INFO("rcfserver upload dir-> %s: %s, successfully!", child_path.c_str(), dst_file.c_str());
	}
	else
	{
	//	boost::filesystem::copy_file(filePath, dst_file, boost::filesystem::copy_option::overwrite_if_exists);
		boost::filesystem::rename(filePath, dst_file);
		ROS_INFO("rcfserver upload file-> %s: %s, successfully!", child_path.c_str(), dst_file.c_str());
	}

	//删除临时文件夹
	boost::system::error_code error;
	boost::filesystem::path temp_path(parent_path);
	boost::filesystem::remove_all(temp_path, error);


	return 0;
}


void FileTranseferImpl::set_root_path(const std::string &root_path)
{
	root_path_ = root_path;
}


int FileTranseferImpl::UploadFile(const std::string &save_relative_path, const std::string &file_name, RCF::FileUpload fileUpload)
{
	std::string uploadPath;
	uploadPath = fileUpload.getLocalPath();
//	boost::thread thread(boost::bind(&FileTranseferImpl::dumpDirpath, this, save_relative_path, uploadPath));
	dumpDirpath(save_relative_path, uploadPath);
	return 0;
}


int FileTranseferImpl::UploadDir(const std::string &save_relative_path, const std::string &file_name, RCF::FileUpload fileUpload)
{
	std::string uploadPath;
	uploadPath = fileUpload.getLocalPath();
//	boost::thread thread(boost::bind(&FileTranseferImpl::dumpDirpath, this, save_relative_path, uploadPath));
	dumpDirpath(save_relative_path, uploadPath);
	return 0;
}


int FileTranseferImpl::DownloadFile(const std::string & relative_path, const std::string &file_name, RCF::FileDownload fileDownload)
{

	std::string src_file_path = root_path_ + "/" + relative_path + "/" + file_name;
	std::string file_type = relative_path.substr(0, relative_path.find_first_of("/\\"));

	fileDownload = RCF::FileDownload(src_file_path.c_str());
	ROS_INFO("rcfserver downlaod file-> %s: %s, successfully!", file_type.c_str(), file_name.c_str());
	return 0;

}


int FileTranseferImpl::DownloadDir(const std::string & relative_path, const std::string &file_name, RCF::FileDownload fileDownload)
{
	std::string src_file_path = root_path_ + "/" + relative_path + "/" + file_name;
	std::string file_type = relative_path.substr(0, relative_path.find_first_of("/\\"));

	ROS_INFO("rcfserver downlaod dir-> %s: %s, successfully!", file_type.c_str(), file_name.c_str());
	fileDownload = RCF::FileDownload(src_file_path.c_str());
	return 0;
}


////////////////////////////////////////////////////////////


FileTranseferServer::FileTranseferServer()
	: root_path_("D:/RCF_Server")
{

}


void FileTranseferServer::setRootDir(const std::string &root_path)
{
	root_path_ = root_path;
	fileTranseferImpl_.set_root_path(root_path_);
	server_.setFileUploadDirectory(root_path_ + "/uploadTemp");

	//´´½¨¸ùÄ¿Â¼
	if (!boost::filesystem::exists(root_path_))
	{
		boost::filesystem::create_directories(root_path_);
	}
	ROS_INFO("rcfserver set root path: %s", root_path_.c_str());
}


bool FileTranseferServer::initServer(const std::string &ip, int port)
{
	try
	{
		fileTranseferImpl_.set_root_path(root_path_);
		server_.addEndpoint(RCF::TcpEndpoint(ip, port));
		server_.bind<FileService>(fileTranseferImpl_);
		server_.setSessionTimeoutMs(-1);
		server_.setFileUploadDirectory(root_path_+"/uploadTemp");
		ROS_INFO("rcfserver init successfully!");
		return true;
	}
	catch (const RCF::Exception & e)
	{
		std::cout << "Caught exception:\n";
		std::cout << e.getError().getErrorString() << std::endl;
		ROS_ERROR("rcfserver init failed, err:%s", e.getError().getErrorString().c_str());
	}
}


void FileTranseferServer::setTimeOutMs(int ms)
{
	server_.setSessionTimeoutMs(ms);
	ROS_INFO("rcfserver setTimeout: %d", ms);
}


void FileTranseferServer::start()
{
	server_.start();
}


void FileTranseferServer::bindUploadProcess()
{
	server_.setOnFileUploadProgress(&onServerFileUpload);
}


void FileTranseferServer::bindDownloadProcess()
{
	server_.setOnFileDownloadProgress(&onServerFileDown);
}






