#ifndef PROTO_SERVER_ALEXWEI_2018_0501_H
#define PROTO_SERVER_ALEXWEI_2018_0501_H

#include <string>
#include <RCF/RCF.hpp>
#include <RCF/FileUpload.hpp>
#include <RCF/FileDownload.hpp>


RCF_BEGIN(FileService, "FileService")
	RCF_METHOD_R3(int, UploadFile, const std::string &, const std::string &, RCF::FileUpload)
	RCF_METHOD_R3(int, DownloadFile, const std::string &, const std::string &, RCF::FileDownload)
	RCF_METHOD_R3(int, UploadDir, const std::string &, const std::string &, RCF::FileUpload)
	RCF_METHOD_R3(int, DownloadDir, const std::string &, const std::string &, RCF::FileDownload)
RCF_END(FileService)


class FileTranseferImpl
{
public:
	FileTranseferImpl();
	~FileTranseferImpl();
	int UploadFile(const std::string &save_relative_path, const std::string &file_name,  RCF::FileUpload);
	int UploadDir(const std::string &save_relative_path, const std::string &file_name, RCF::FileUpload);

	int DownloadFile(const std::string &save_relative_path, const std::string &s, RCF::FileDownload fileDownload);
	int DownloadDir(const std::string &relative_path, const std::string &s, RCF::FileDownload fileDownload);
	void set_root_path(const std::string &root_path);

private:
	int dumpDirpath(const std::string & type, const std::string &path);
	bool createPath(const std::string &relative_path);
	bool CopyDirectory(const std::string &strSourceDir, const std::string &strDestDir);
	 
private:
	std::string root_path_;
};



class FileTranseferServer
{

public:
	FileTranseferServer();

	bool initServer(const std::string &ip, int port);
	void setTimeOutMs(int ms = -1);
	void setRootDir(const std::string &name = "D:/RCF_Server");
	void start();
	void bindUploadProcess();
	void bindDownloadProcess();

private:
	std::string root_path_;
	RCF::RcfServer server_;
	FileTranseferImpl fileTranseferImpl_;
};






#endif // ! PROTO_SERVER_ALEXWEI_2018_0501_H





