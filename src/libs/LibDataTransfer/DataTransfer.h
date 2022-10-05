#ifndef DATATRANSFER_ALEXWEI_20180503_H
#define DATATRANSFER_ALEXWEI_20180503_H

#include <QThread>

struct TransferPro
{
	int cmd_type_;
	std::string src_file_path_;
	std::string download_path_;
	std::string dst_relative_path_;
	std::string file_name_;

	TransferPro() {
		cmd_type_ = -1;
		src_file_path_ = "";
		download_path_ = "D:/";
		dst_relative_path_ = "";
		file_name_ = "";
	}
};


class DataTransfer : public QThread
{
	Q_OBJECT
		
public:
	DataTransfer(QObject *parent = 0);
	~DataTransfer(){}
	
	void run();
	void set_transfer_info(TransferPro pro);

    void set_infrared_type(int check);
signals:
	void sig_finished(int action_type, int execCode, QString exeParam);
    void sig_finished_infrared(QString path, QString name);

private:
	TransferPro pro_;
    int m_iCheck = 0;

public:
    const static int DOWNLOAD_FILE	= 0;
    const static int UPLOAD_FILE	= 1;
	const static int FILE_SMAP		= 0;
	const static int FILE_2D		= 1;
    
};


#endif