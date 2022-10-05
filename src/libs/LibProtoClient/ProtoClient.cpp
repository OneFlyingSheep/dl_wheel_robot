#include "ProtoClient.h"
#include <iostream>
#include <direct.h>
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include "common/DLRobotCommonDef.h"
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>


static int percentage_ = 0;
boost::signals2::signal<void(int)> signal_percentage_;

//回调函数
void onFileTransferProcessCallback(const RCF::FileTransferProgress &process)
{
	percentage_ = ((double)process.mBytesTransferredSoFar) / process.mBytesTotalToTransfer * 100;
	signal_percentage_(percentage_);
}


FileTransferClient::FileTransferClient()
	: root_path_("D:\\RCF_Server")
{
	//创建根目录
	if (!boost::filesystem::exists(root_path_))
	{
		boost::filesystem::create_directories(root_path_);
		ROS_INFO("rcfclient create root path: %s", root_path_.c_str());
	}
	signal_percentage_.connect(boost::bind(&FileTransferClient::trans_percentage, this, _1));

}


void FileTransferClient::set_root_path(const std::string &root_path)
{
	root_path_ = root_path;
	//创建根目录
	if (!boost::filesystem::exists(root_path_))
	{
		boost::filesystem::create_directories(root_path_);
		ROS_INFO("rcfclient create root path: %s", root_path_.c_str());
	}
	ROS_INFO("rcfclient set root path: %s", root_path_.c_str());
}


void FileTransferClient::init(const std::string &ip, int port)
{
	ip_ = ip;
	port_ = port;
	ROS_INFO("rcfclient init ip:%s, port:%d", ip_.c_str(), port_);
}


int FileTransferClient::getTransferPercentage()
{
	signal_tran_percentage(percentage_);
	return percentage_;
}


void FileTransferClient::trans_percentage(int percentage)
{
	signal_tran_percentage(percentage);
}


int FileTransferClient::uploadFile(const std::string &src_file_path, const std::string &dst_relative_path)
{
	try
	{
		RCF::RcfInitDeinit rcfInit;
		RcfClient<FileService> file_client(RCF::TcpEndpoint(ip_, port_));
		file_client.getClientStub().setFileProgressCallback(&onFileTransferProcessCallback);
		RCF::FileUpload fileUpload(src_file_path);
		std::string file_name = src_file_path.substr(src_file_path.find_last_of("/\\") + 1);


		//上传文件夹，否则是上传普通文件
		boost::filesystem::path _path(src_file_path);
		int ret = -1;
		if (boost::filesystem::is_directory(_path))
		{
			ret = file_client.UploadDir(dst_relative_path, file_name, fileUpload);
			ROS_INFO("rcfclient upload dir->%s,dst_path:%s, successfully!", file_name.c_str(), dst_relative_path.c_str());
			return ret;
		}
		else 
		{
			ret = file_client.UploadFile(dst_relative_path, file_name, fileUpload);
			ROS_INFO("rcfclient upload file->%s, dst_path:%s, successfully!", file_name.c_str(), dst_relative_path.c_str());
			if (percentage_ == 100) {
				percentage_ = 0;
			}
			return ret;
		}
	}
	catch (const RCF::Exception & e)
	{
		percentage_ = 0;
		ROS_ERROR("rcfclient upload -> %s failed, err:%s", src_file_path.c_str(), e.getError().getErrorString().c_str());
		return -1;
	}
}


int FileTransferClient::downloadFile(const std::string &dst_relative_path, const std::string &file_name, const std::string &download_path)
{
	try
	{
		RCF::RcfInitDeinit rcfInit;
		RcfClient<FileService> file_client(RCF::TcpEndpoint(ip_, port_));
		file_client.getClientStub().setFileProgressCallback(&onFileTransferProcessCallback);

		//创建下载临时路径
		std::string temp_download_path =root_path_ + "/downloadTemp";
		if (!boost::filesystem::exists(temp_download_path)) {
			if (!boost::filesystem::create_directories(temp_download_path)) {
				ROS_INFO("rcfclient create temp download path failed, please check config file!");
			}
		}
	
		RCF::FileDownload fileDownload;
		fileDownload.setDownloadToPath(temp_download_path);
		std::string dst_file_path;
		
		if (!boost::filesystem::exists(download_path)) {
			bool ret = boost::filesystem::create_directories(download_path);
			if (!ret) {
				boost::filesystem::create_directories(root_path_ + "/downloadTemp");
				dst_file_path = root_path_ + "/downloadTemp";
			}
			else {
				dst_file_path = download_path;
			}
		}
		else {
			dst_file_path = download_path;
		}

		//下载文件到临时文价夹
		percentage_ = 0;
		int ret = -1;
		ret = file_client.DownloadFile(dst_relative_path, file_name, fileDownload);
		if (percentage_ == 100) {
			percentage_ = 0;
		}

		//将临时下载文件夹的文件拷贝到目标文件夹
        std::string  temp_file_name = file_name;
        if (file_name.empty())
        {
            temp_file_name = dst_relative_path.substr(dst_relative_path.find_last_of("/\\") + 1);
        }
        
		std::string src_temp_file = temp_download_path + "/" + temp_file_name;
		std::string dst_file = dst_file_path + "/" + temp_file_name;
		boost::filesystem::path _path(dst_file);
		
		if (!boost::filesystem::exists(src_temp_file)) {
			ROS_ERROR("rcfclient download faild!");
			return -1;
		}
		else {
			ROS_INFO("rcfclient download file->%s: %s successfully!", dst_relative_path.c_str(), dst_file_path.c_str());
		}


		bool is_dir = boost::filesystem::is_directory(src_temp_file);
		if (is_dir) {
			//查找目录下是否有同名文件，有则删除，无则直接下载
            CopyDirectory(src_temp_file, dst_file);
		}
		else {
			//查找目录下是否有同名文件，有则删除，无则直接下载
			if (boost::filesystem::exists(_path)) {
				boost::filesystem::remove(_path);
			}
			else {
			//	boost::filesystem::copy_file(src_temp_file, dst_file);
                if (!boost::filesystem::exists(dst_file_path))
                {
                    boost::filesystem::create_directory(dst_file_path);
                }
				boost::filesystem::rename(src_temp_file, dst_file);
			}
		}
// 		try 
// 		{
// 		//
//             boost::filesystem::remove_all(boost::filesystem::path(src_temp_file));
// 		}
// 		catch (const std::exception& e) 
// 		{
// 			std::string err = e.what();
// 			ROS_ERROR("rcfclient delete template faild, err:%s!", e.what());
// 		}
		return ret;
	}
	catch (const RCF::Exception & e)
	{
		ROS_ERROR("rcfclient dowload -> %s failed, err:%s", file_name.c_str(), e.getError().getErrorString().c_str());
		return -1;
	}
}


bool FileTransferClient::CopyDirectory(const std::string &strSourceDir, const std::string &strDestDir)
{
	boost::filesystem::recursive_directory_iterator end; //设置遍历结束标志，用recursive_directory_iterator即可循环的遍历目录
	boost::system::error_code ec;
	for (boost::filesystem::recursive_directory_iterator pos(strSourceDir); pos != end; ++pos)
	{
		std::string strAppPath = boost::filesystem::path(*pos).string();
		std::string strRestorePath;
		//replace_first_copy在algorithm/string头文件中，在strAppPath中查找strSourceDir字符串，找到则用strDestDir替换，替换后的字符串保存在一个输出迭代器中
		boost::replace_first_copy(std::back_inserter(strRestorePath), strAppPath, strSourceDir, strDestDir);
		if (!boost::filesystem::exists(boost::filesystem::path(strRestorePath).parent_path()))
		{
			boost::filesystem::create_directories(boost::filesystem::path(strRestorePath).parent_path(), ec);
		}

		if (boost::filesystem::is_directory(strAppPath)) {
			boost::filesystem::create_directory(boost::filesystem::path(strRestorePath), ec);
		}
		else {
		//	boost::filesystem::copy_file(strAppPath, strRestorePath, boost::filesystem::copy_option::overwrite_if_exists, ec);
			boost::filesystem::rename(strAppPath, strRestorePath);
		}
	}
	if (ec)
	{
		return false;
	}
	return true;
}






