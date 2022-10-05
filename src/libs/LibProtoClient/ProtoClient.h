#ifndef PROTO_CLIENT_ALEXWEI_2018_0501_H
#define PROTO_CLIENT_ALEXWEI_2018_0501_H

#include <common/Singleton.hpp>
#include "LibProtoServer/ProtoServer.h"
#include <boost/signals2.hpp>


class FileTransferClient : public Singleton<FileTransferClient>
{

public:
	FileTransferClient();
	void init(const std::string &ip = "192.168.1.38", int port = 50001);

	int uploadFile(const std::string &src_file_path, const std::string &dst_relative_path);
	int downloadFile(const std::string &dst_relative_path, const std::string &file_name, const std::string &download_path = "");
	bool CopyDirectory(const std::string &strSourceDir, const std::string &strDestDir);
	void set_root_path(const std::string &root_path);
	void trans_percentage(int);
	int getTransferPercentage();

public:
	boost::signals2::signal<void(int)> signal_tran_percentage;

private:
	std::string root_path_;
	std::string ip_;
	int port_;

};


#define GFILE_TRANSFER_CLIENT FileTransferClient::GetSingleton()



#endif // ! PROTO_SERVER_ALEXWEI_2018_0501_H





