#pragma once

/// 系统信息显示界面

#include <stdio.h>
#include <atlbase.h>
#include <atlcom.h>
#include <sapi.h>
#include <QWidget>
#include <QString>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>
#include <QStringList>
#include <QtWidgets/QApplication>
#include <QDebug>
#include <QDir>
#include <QFileInfoList>
#include <Windows.h>
#include <QSlider>

#include "diskInfoItem.h"
#include "Signalsintensity.h"

#define GB (1024*1024*1024)

#pragma execution_character_set("utf-8")

class SystemInformation :
	public QWidget
{
	Q_OBJECT
public:
	SystemInformation(QWidget *parent = NULL);
	~SystemInformation();

	void getDiskSizeFunction(); 

	QStringList getDiskName();
	quint64 getDiskSpace(QString iDriver, bool flag);
private:
	void initWidget();

	qint64 m_totalDiskNum, m_surplusDiskNum;
};

