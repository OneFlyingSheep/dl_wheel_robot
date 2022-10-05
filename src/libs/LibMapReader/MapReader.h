#ifndef MAPREADER_ALEXWEI_20180424_H
#define MAPREADER_ALEXWEI_20180424_H

#include <string>
#include <QThread>
#include <QReadWriteLock>
#include "MapData.h"

class MapReader : public QThread
{
	Q_OBJECT

public:
	enum THREAD_TYPE
	{
		READ_THREAD_TYPE,						//������
		WRITE_THREAD_TYPE,						//д����
		TYPE_NUM
	};
	MapReader(QObject *parent = 0);
	~MapReader();
	void SetReadPro(const std::string& file, int type, bool isOnlyLoadLm = false);
	void SetMapData(const std::string& file, MapData map_data);
	MapData &GetData();
	void ClearMapData();
signals:
	void FinishedSignals(int type, bool isOnlyLoadLm);

protected:
	void run();

private:
	void Read(const std::string& file);
	void CreateMap(const std::string& fileName);

private:
	bool m_bIsOnlyLoadLm;
	QList<int> m_lstTypes;					//���治ͬ������
	std::string m_strSmapFileName;				//smap�ļ���
	MapData m_stMapData;
	QReadWriteLock m_lockReadWrite;
};


#endif //MAPREADER_ALEXWEI_20180424_H