#include <fstream>
#include <iostream>
#include <string>
#include <QDebug>
#include "ReadBgMapThread.h"
#include "MapData.h"

#define CELL_WIDTH (100.0)
#define CELL_HEIGHT (100.0)

ReadBgMapThread::ReadBgMapThread(QObject* parent)
	:m_bIsRun(false)
{
	m_vBgNormalPoints.clear();
}

ReadBgMapThread::~ReadBgMapThread()
{
	Stop();
}

void ReadBgMapThread::SetFilePath(const QString &strFileName)
{
	m_mutex.lock();
	m_strFileName = strFileName;
	m_mutex.unlock();
}

void ReadBgMapThread::Start()
{
	m_mutex.lock();
	m_bIsRun = true;
	m_mutex.unlock();
	start();
}

void ReadBgMapThread::Stop()
{
	m_mutex.lock();
	m_bIsRun = false;
	m_mutex.unlock();
	quit();
	wait();
}

void ReadBgMapThread::run()
{
	m_mutex.lock();
	bool bIsRun = m_bIsRun;
	m_mutex.unlock();
	if (bIsRun)
	{
		ReadFile(m_strFileName);
		CalcAreaRangle();
		CalcAreaPoints();
		emit SendAreaPointsSignal(m_ptMinPoint, m_ptMaxPoint);
	}
	
}

void ReadBgMapThread::ReadFile(const QString &strFileName)
{
	std::ifstream heightMapfile(strFileName.toLocal8Bit().data());
	if (heightMapfile)
	{
		char buf[256];
		double dMinX, dMaxX, dMinY, dMaxY;
		dMinX = 1e10;
		dMinY = 1e10;
		dMaxX = -1e10;
		dMaxY = -1e10;
		while (heightMapfile)
		{
			double x, y;
			heightMapfile.getline(buf, 100);
			if (heightMapfile.fail())
				break;
			if (buf[0] == '\n' || buf[0] == '\r')
			{
				continue;
			}
			sscanf(buf, "%lf %lf", &x, &y);
			x /= 100;
			y /= 100.;
			y = 0 - y;
			if (dMinX > x)
			{
				dMinX = x;
			}

			if (dMinY > y)
			{
				dMinY = y;
			}

			if (dMaxX < x)
			{
				dMaxX = x;
			}

			if (dMaxY < y)
			{
				dMaxY = y;
			}
			QPointF temP(x, y);

			m_vBgNormalPoints.push_back(temP);
		}
		m_ptMinPoint = QPointF(dMinX, dMinY);
		m_ptMaxPoint = QPointF(dMaxX, dMaxY);
		heightMapfile.close();

	}
	else 
	{
		std::cout << "unable  to open heightMapfile" << endl;
	}
}

void ReadBgMapThread::CalcAreaRangle()
{
	m_vRectAreaPoints.clear();
	int iXCellNum = (int)(m_ptMaxPoint.x() - m_ptMinPoint.x()) / CELL_WIDTH;		//计算列数
	int iYCellNum = (int)(m_ptMaxPoint.y() - m_ptMinPoint.y()) / CELL_HEIGHT;		//计算行数

	++iXCellNum;
	++iYCellNum;

	for (int iRow = 0; iRow < iYCellNum; ++iRow)
	{
		QVector<AreaInfo> vAreaInfos;

		double dMinY = (double)iRow * CELL_HEIGHT;
		double dMaxY = (double)(iRow+1) * CELL_HEIGHT;
		dMinY += m_ptMinPoint.y();
		dMaxY += m_ptMinPoint.y();


		for (int iColumn = 0; iColumn < iXCellNum; ++iColumn)
		{
			AreaInfo stAreaInfo;
			stAreaInfo.dMinX = (double)iColumn * CELL_WIDTH;
			stAreaInfo.dMinX += m_ptMinPoint.x();
			stAreaInfo.dMinY = dMinY;
			stAreaInfo.dMaxX = (double)(iColumn + 1) * CELL_WIDTH;
			stAreaInfo.dMaxX += m_ptMinPoint.x();
			stAreaInfo.dMaxY = dMaxY;
			vAreaInfos.push_back(stAreaInfo);
		}
		m_vRectAreaPoints.push_back(vAreaInfos);
	}

}

void ReadBgMapThread::CalcAreaPoints()
{
	for (int i = 0; i < (int)m_vBgNormalPoints.size(); i++)
	{
		QPointF stPoint = m_vBgNormalPoints[i];
		int iColumn = (stPoint.x() - m_ptMinPoint.x()) / CELL_WIDTH;			//计算第几列
		int iRow = (stPoint.y() - m_ptMinPoint.y()) / CELL_HEIGHT;			//计算第几行
		//qDebug() << "============计算出的row:" << iRow << ";column:" << iColumn;
		m_vRectAreaPoints[iRow][iColumn].vPoints.push_back(m_vBgNormalPoints[i]);
	}
}

void ReadBgMapThread::Clear()
{
	m_bIsRun = false;
	m_vBgNormalPoints.clear();
	m_vRectAreaPoints.clear();
}

QVector<QVector<AreaInfo>> ReadBgMapThread::GetRectAreaPoints()
{
	return m_vRectAreaPoints;
}
