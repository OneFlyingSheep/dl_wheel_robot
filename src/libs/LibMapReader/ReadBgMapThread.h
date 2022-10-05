#ifndef _READ_BG_MAP_THREAD_H
#define _READ_BG_MAP_THREAD_H
#pragma execution_character_set("utf-8")

#include <QThread>
#include <QMutex>

struct AreaInfo;

class ReadBgMapThread : public QThread
{
	Q_OBJECT

public:
	ReadBgMapThread(QObject *parent = 0);
	~ReadBgMapThread();
	void SetFilePath(const QString &strFileName);			//设置文件路径
	void Start();
	void Stop();
	void Clear();
	QVector<QVector<AreaInfo>> GetRectAreaPoints();

signals:
	void SendAreaPointsSignal(QPointF ptMinPoint, QPointF ptMaxPoint);

protected:
	virtual void run();

private:
	void ReadFile(const QString &strFileName);
	void CalcAreaRangle();
	void CalcAreaPoints();


private:
	bool						m_bIsRun;
	QString						m_strFileName;
	QMutex						m_mutex;
	QVector<QPointF>			m_vBgNormalPoints;
	QVector<QVector<AreaInfo>>	m_vRectAreaPoints;
	QPointF						m_ptMinPoint;
	QPointF						m_ptMaxPoint;
};


#endif //_READ_BG_MAP_THREAD_H