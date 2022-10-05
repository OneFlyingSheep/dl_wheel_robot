#include "SearchRecordCreateExcel.h"
#include <QApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <QFileDialog>
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
// #include <boost/uuid/uuid_generators.hpp>  
// #include <boost/uuid/uuid_io.hpp>  
// #include <boost/uuid/uuid.hpp>
// #include <boost/lexical_cast.hpp>
// #include <boost/algorithm/string.hpp>

#define EXCEL_TEMPLATE_TO_LOAD		 "/ExcelTemplate/demo.xlsx"
#define EXCEL_SET_ROW_HEIGHT         80
#define EXCEL_SET_COLUMN_WIDTH		 15
#define EXCEL_PICTURE_LEFT           139
#define EXCEL_WRITE_PICTURE_WIDTH    13.5
#define EXCEL_WRITE_PICTURE_HEIGHT   79.5

SearchRecordCreateExcel::SearchRecordCreateExcel(QObject* parent)
{
	is_link = false;
	m_rowHeight = 0.0;
}

SearchRecordCreateExcel::~SearchRecordCreateExcel()
{
	if(is_link)
		delete m_excelC;
}

void SearchRecordCreateExcel::CreateNewExcelForList(QList<QStringList> m_searchResult, QString m_SavePath, WheelRobotExcelChoose cho)
{
	if (m_searchResult.size() == 0 || m_searchResult.size() == 1)
	{
		return;
	}
	switch (cho)
	{
	case EXCEL_DEVICE_ALARM_SEARCH_AUDIT:
		createExcelDeviceAlarmSearchAudit(m_searchResult, m_SavePath);
		break;
	case EXCEL_PATROL_RESULT_BROWSE:
		createExcelPatrolResultBrowse(m_searchResult, m_SavePath);
		break;
	case EXCEL_COMPARE_ANALYZE:
		createExcelCompareAnalyze(m_searchResult, m_SavePath);
		break;
	case EXCEL_CREATE_REPORT:
		createExcelCreateReport(m_searchResult, m_SavePath);
		break;
	case EXCEL_STANDARD_POINT:
		createExcelStandardPoint(m_searchResult, m_SavePath);
		break;
	case EXCEL_RECOGNITION_UNUSUAL_POINT_SEARCH:
		createExcelRecognitionUnusualPointSearch(m_searchResult, m_SavePath);
		break;
	case EXCEL_PATROL_POINT_SET:
		createExcelPatrolPointSet(m_searchResult, m_SavePath);
		break;
	default:
		break;
	}
}

void SearchRecordCreateExcel::createExcelDeviceAlarmSearchAudit(QList<QStringList> m_searchResult, QString m_SavePath)
{
	QExcel * qExcel = new QExcel;

	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_picture;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;
	rowNow = 0;

	qExcel->OpenExcel(m_SavePath);
	qExcel->selectSheet("Sheet1");

	qExcel->setColumnWidth(1, 15);
	qExcel->setColumnWidth(2, 20);
	qExcel->setColumnWidth(3, 40);
	qExcel->setColumnWidth(4, 15);
	qExcel->setColumnWidth(5, 15);
	qExcel->setColumnWidth(6, 30);
	qExcel->setColumnWidth(7, 15);
	qExcel->setColumnWidth(8, 15);

	m_headQV.append("序号");
	for (int i = 0; i < m_searchResult[0].size(); i++)
	{
		m_headQV.append(m_searchResult[0][i]);
	}

	for (int i = 1; i < m_searchResult.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i);
		for (int k = 0; k < 5; k++)
		{
			m_variant.append(m_searchResult[i].at(k));
		}
		m_value.append(m_variant);

		m_choo.clear();

		if (m_searchResult[i][5] != "")
		{
			m_choo.append(m_searchResult[i][5]);
		}
		if (m_searchResult[i][6] != "")
		{
			m_choo.append(m_searchResult[i][6]);
		}
		m_picture.append(m_choo);
	}

	rowNow++;
	qExcel->setCellString(QString("A%1:H%2").arg(rowNow).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:F%2").arg(rowNow + 1).arg(rowNow + m_searchResult.size() - 1), m_var);
	/*
	m_rowHeight = (rowNow)* EXCEL_WRITE_PICTURE_WIDTH;

	for (int i = 0; i < m_searchResult.size() - 1; i++)
	{
		QList<QString>::ConstIterator ci;
		int column = 7;
		qExcel->setRowHeight(rowNow + 1, EXCEL_SET_ROW_HEIGHT); // 设置行高

		for (ci = m_picture[i].constBegin(); ci != m_picture[i].constEnd(); ci++)
		{

			if ((*ci).mid(0, 3) == "D:\\")
			{
				if (column == 7)
				{
					qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
				if (column == 8)
				{
					qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
			}
			else
			{
			}
			column++;
		}
		if (i == m_searchResult.size() - 2)
		{
			m_rowHeight = (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight;
		}
		rowNow++;
		//	qDebug() << "excel插入第" << i + 1 << "条";
	}
	*/
	qExcel->save();
	qExcel->close();
	delete qExcel;
}

void SearchRecordCreateExcel::createExcelPatrolResultBrowse(QList<QStringList> m_searchResult, QString m_SavePath)
{
	QExcel * qExcel = new QExcel;

	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_picture;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;
	rowNow = 0;

	qExcel->OpenExcel(m_SavePath);
	qExcel->selectSheet("Sheet1");

	qExcel->setColumnWidth(1, 15);
	qExcel->setColumnWidth(2, 40);
	qExcel->setColumnWidth(3, 40);
	qExcel->setColumnWidth(4, 30);
	qExcel->setColumnWidth(5, 15);
	qExcel->setColumnWidth(6, 15);
 	qExcel->setColumnWidth(7, 15);


	m_headQV.append("序号");
	for (int i = 0; i < m_searchResult[0].size(); i++)
	{
		m_headQV.append(m_searchResult[0][i]);
	}

	for (int i = 1; i < m_searchResult.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i);
		for (int k = 0; k < 4; k++)
		{
			m_variant.append(m_searchResult[i].at(k));
		}
		m_value.append(m_variant);

		m_choo.clear();

		if (m_searchResult[i].size() > 4)
		{
			if (m_searchResult[i][4] != "")
			{
				m_choo.append(m_searchResult[i][4]);
			}
		}
		if (m_searchResult[i].size() > 5)
		{
			if (m_searchResult[i][5] != "")
			{
				m_choo.append(m_searchResult[i][5]);
			}
		}
		m_picture.append(m_choo);
	}

	rowNow++;
	qExcel->setCellString(QString("A%1:G%2").arg(rowNow).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:E%2").arg(rowNow + 1).arg(rowNow + m_searchResult.size() - 1), m_var);
	/*
	m_rowHeight = (rowNow)* EXCEL_WRITE_PICTURE_WIDTH;

	for (int i = 0; i < m_searchResult.size() - 1; i++)
	{
		QList<QString>::ConstIterator ci;
		int column = 7;
		qExcel->setRowHeight(rowNow + 1, EXCEL_SET_ROW_HEIGHT); // 设置行高

		for (ci = m_picture[i].constBegin(); ci != m_picture[i].constEnd(); ci++)
		{

			if ((*ci).mid(0, 3) == "D:\\")
			{
				if (column == 7)
				{
				//	qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
					qExcel->setCellPicture(143.5 * (7 - 1), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
				if (column == 8)
				{
				//	qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
					qExcel->setCellPicture(143.5 * (7 - 1) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
			}
			else
			{
			}
			column++;
		}
		if (i == m_searchResult.size() - 2)
		{
			m_rowHeight = (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight;
		}
		rowNow++;
		//	qDebug() << "excel插入第" << i + 1 << "条";
	}
	*/
	qExcel->save();
	qExcel->close();
	delete qExcel;
}

void SearchRecordCreateExcel::createExcelCompareAnalyze(QList<QStringList> m_searchResult, QString m_SavePath)
{
	QExcel * qExcel = new QExcel;

	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_picture;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;
	rowNow = 0;

	qExcel->OpenExcel(m_SavePath);
	qExcel->selectSheet("Sheet1");


	qExcel->setColumnWidth(1, 15);
	qExcel->setColumnWidth(2, 40);
	qExcel->setColumnWidth(3, 40);
	qExcel->setColumnWidth(4, 30);
	qExcel->setColumnWidth(5, 15);
	qExcel->setColumnWidth(6, 15);


	m_headQV.append("序号");
	for (int i = 0; i < m_searchResult[0].size(); i++)
	{
		m_headQV.append(m_searchResult[0][i]);
	}

	for (int i = 1; i < m_searchResult.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i);
		for (int k = 0; k < 3; k++)
		{
			m_variant.append(m_searchResult[i].at(k));
		}
		m_value.append(m_variant);

		m_choo.clear();

		if (m_searchResult[i].size() > 3)
		{
			if (m_searchResult[i][3] != "")
			{
				m_choo.append(m_searchResult[i][3]);
			}
		}

		if (m_searchResult[i].size() > 4)
		{
			if (m_searchResult[i][4] != "")
			{
				m_choo.append(m_searchResult[i][4]);
			}
		}
		m_picture.append(m_choo);
	}

	rowNow++;
	qExcel->setCellString(QString("A%1:F%2").arg(rowNow).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:D%2").arg(rowNow + 1).arg(rowNow + m_searchResult.size() - 1), m_var);
	/*
	m_rowHeight = (rowNow)* EXCEL_WRITE_PICTURE_WIDTH;

	for (int i = 0; i < m_searchResult.size() - 1; i++)
	{
		QList<QString>::ConstIterator ci;
		int column = 7;
		qExcel->setRowHeight(rowNow + 1, EXCEL_SET_ROW_HEIGHT); // 设置行高

		for (ci = m_picture[i].constBegin(); ci != m_picture[i].constEnd(); ci++)
		{

			if ((*ci).mid(0, 3) == "D:\\")
			{
				if (column == 7)
				{
					qExcel->setCellPicture(127.7 * (7 - 1), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
				if (column == 8)
				{
					qExcel->setCellPicture(127.7 * (7 - 1) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
			}
			else
			{
			}
			column++;
		}
		if (i == m_searchResult.size() - 2)
		{
			m_rowHeight = (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight;
		}
		rowNow++;
	}
	*/
	qExcel->save();
	qExcel->close();
	delete qExcel;
}

void SearchRecordCreateExcel::createExcelCreateReport(QList<QStringList> m_searchResult, QString m_SavePath)
{
	QExcel * qExcel = new QExcel;
	bool bPic = false;

	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_picture;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;
	rowNow = 0;

	qExcel->OpenExcel(m_SavePath);
	qExcel->selectSheet("Sheet1");

	int dataSize = 0; 
	m_headQV.append("序号");
	for (int i = 0; i < m_searchResult[0].size(); i++)
	{
		m_headQV.append(m_searchResult[0][i]);
		if (m_searchResult[0][i] == "采集信息")
		{
			qExcel->setColumnWidth(i + 2, 15);
			bPic = true;
		}
	}
	if (bPic)
	{
		dataSize = m_searchResult[0].size() - 1;
	}
	else
	{
		dataSize = m_searchResult[0].size() + 1;
	}

	for (int i = 1; i < m_searchResult.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i);
		
		for (int k = 0; k < dataSize - 1; k++)
		{
			m_variant.append(m_searchResult[i].at(k));
		}
		m_value.append(m_variant);

		if (bPic)
		{
			m_choo.clear();

			if (m_searchResult[i].size() > dataSize - 1)
			{
				if (m_searchResult[i][dataSize - 2] != "")
				{
					m_choo.append(m_searchResult[i][dataSize - 1]);
				}
			}

			if (m_searchResult[i].size() > dataSize)
			{
				if (m_searchResult[i][dataSize - 1] != "")
				{
					m_choo.append(m_searchResult[i][dataSize]);
				}
			}
			m_picture.append(m_choo);
		}
	}
	char letter = 65;
	rowNow++;
	qExcel->setCellString(QString("A%1:%2%3").arg(rowNow).arg(QString(letter + m_searchResult[0].size())).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:%2%3").arg(rowNow + 1).arg(QString(letter + dataSize - 1)).arg(rowNow + m_searchResult.size() - 1), m_var);
	

	
	if (bPic)
	{
		qExcel->mergeCells(QString("A%1:%2%3").arg(rowNow + m_searchResult.size()).arg(QString(letter + dataSize + 1)).arg(rowNow + m_searchResult.size()));
		qExcel->setCellString(rowNow + m_searchResult.size(), 1, "查询点位共  个，正常点位  个，告警点位  个，识别异常点位  个");
		/*
		m_rowHeight = (rowNow)* EXCEL_WRITE_PICTURE_WIDTH;
		for (int i = 0; i < m_searchResult.size() - 1; i++)
		{
			QList<QString>::ConstIterator ci;
			int column = 7;
			qExcel->setRowHeight(rowNow + 1, EXCEL_SET_ROW_HEIGHT); // 设置行高

			for (ci = m_picture[i].constBegin(); ci != m_picture[i].constEnd(); ci++)
			{

				if ((*ci).mid(0, 3) == "D:\\")
				{
					if (column == 7)
					{
						qExcel->setCellPicture(54 * (dataSize), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
					}
					if (column == 8)
					{
						qExcel->setCellPicture(54 * (dataSize) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
					}
				}
				else
				{
				}
				column++;
			}
			if (i == m_searchResult.size() - 2)
			{
				m_rowHeight = (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight;
			}
			rowNow++;
		}
		*/
	}
	else
	{
		qExcel->mergeCells(QString("A%1:%2%3").arg(rowNow + m_searchResult.size()).arg(QString(letter + dataSize - 1)).arg(rowNow + m_searchResult.size()));
		qExcel->setCellString(rowNow + m_searchResult.size(), 1, "查询点位共  个，正常点位  个，告警点位  个，识别异常点位  个");
	}

	qExcel->save();
	qExcel->close();
	delete qExcel;
}

void SearchRecordCreateExcel::createExcelStandardPoint(QList<QStringList> m_searchResult, QString m_SavePath)
{
	QExcel * qExcel = new QExcel;

	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_picture;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;
	rowNow = 0;

	qExcel->OpenExcel(m_SavePath);
	qExcel->selectSheet("Sheet1");


	qExcel->setColumnWidth(1, 10);
	qExcel->setColumnWidth(2, 20);
	qExcel->setColumnWidth(3, 40);
	qExcel->setColumnWidth(4, 40);
	qExcel->setColumnWidth(5, 30);
	qExcel->setColumnWidth(6, 15);
	qExcel->setColumnWidth(7, 15);
	qExcel->setColumnWidth(8, 15);

	m_headQV.append("序号");
	for (int i = 0; i < m_searchResult[0].size(); i++)
	{
		m_headQV.append(m_searchResult[0][i]);
	}

	for (int i = 1; i < m_searchResult.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i);
		for (int k = 0; k < 7; k++)
		{
			m_variant.append(m_searchResult[i].at(k));
		}
		m_value.append(m_variant);

	}

	rowNow++;
	qExcel->setCellString(QString("A%1:H%2").arg(rowNow).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:H%2").arg(rowNow + 1).arg(rowNow + m_searchResult.size() - 1), m_var);

	qExcel->save();
	qExcel->close();
	delete qExcel;
}

void SearchRecordCreateExcel::createExcelRecognitionUnusualPointSearch(QList<QStringList> m_searchResult, QString m_SavePath)
{
	QExcel * qExcel = new QExcel;

	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_picture;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;
	rowNow = 0;

	qExcel->OpenExcel(m_SavePath);
	qExcel->selectSheet("Sheet1");

	qExcel->setColumnWidth(1, 15);
	qExcel->setColumnWidth(2, 40);
	qExcel->setColumnWidth(3, 40);
	qExcel->setColumnWidth(4, 15);
	qExcel->setColumnWidth(5, 15);
	qExcel->setColumnWidth(6, 15);
	qExcel->setColumnWidth(7, 15);


	m_headQV.append("序号");
	for (int i = 0; i < m_searchResult[0].size(); i++)
	{
		m_headQV.append(m_searchResult[0][i]);
	}

	for (int i = 1; i < m_searchResult.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i);
		for (int k = 0; k < 4; k++)
		{
			m_variant.append(m_searchResult[i].at(k));
		}
		m_value.append(m_variant);

		m_choo.clear();

		if (m_searchResult[i].size() > 4)
		{
			if (m_searchResult[i][4] != "")
			{
				m_choo.append(m_searchResult[i][4]);
			}
		}
		if (m_searchResult[i].size() > 5)
		{
			if (m_searchResult[i][5] != "")
			{
				m_choo.append(m_searchResult[i][5]);
			}
		}
		m_picture.append(m_choo);
	}

	rowNow++;
	qExcel->setCellString(QString("A%1:G%2").arg(rowNow).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:E%2").arg(rowNow + 1).arg(rowNow + m_searchResult.size() - 1), m_var);
	/*
	m_rowHeight = (rowNow)* EXCEL_WRITE_PICTURE_WIDTH;

	for (int i = 0; i < m_searchResult.size() - 1; i++)
	{
		QList<QString>::ConstIterator ci;
		int column = 7;
		qExcel->setRowHeight(rowNow + 1, EXCEL_SET_ROW_HEIGHT); // 设置行高

		for (ci = m_picture[i].constBegin(); ci != m_picture[i].constEnd(); ci++)
		{

			if ((*ci).mid(0, 3) == "D:\\")
			{
				if (column == 7)
				{
					//	qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
					qExcel->setCellPicture(128.5 * (7 - 1), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
				if (column == 8)
				{
					//	qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
					qExcel->setCellPicture(128.5 * (7 - 1) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
			}
			else
			{
			}
			column++;
		}
		if (i == m_searchResult.size() - 2)
		{
			m_rowHeight = (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight;
		}
		rowNow++;
		//	qDebug() << "excel插入第" << i + 1 << "条";
	}
	*/
	qExcel->save();
	qExcel->close();
	delete qExcel;
}

void SearchRecordCreateExcel::createExcelPatrolPointSet(QList<QStringList> m_searchResult, QString m_SavePath)
{
	QExcel * qExcel = new QExcel;

	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_picture;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;
	rowNow = 0;

	qExcel->OpenExcel(m_SavePath);
	qExcel->selectSheet("Sheet1");


	qExcel->setColumnWidth(1, 10);
	qExcel->setColumnWidth(2, 20);
	qExcel->setColumnWidth(3, 40);
	qExcel->setColumnWidth(4, 40);
	qExcel->setColumnWidth(5, 30);
	qExcel->setColumnWidth(6, 15);
	qExcel->setColumnWidth(7, 15);
	qExcel->setColumnWidth(8, 15);
	qExcel->setColumnWidth(9, 15);
	qExcel->setColumnWidth(10, 15);
	qExcel->setColumnWidth(11, 15);

	m_headQV.append("序号");
	for (int i = 0; i < m_searchResult[0].size(); i++)
	{
		m_headQV.append(m_searchResult[0][i]);
	}

	for (int i = 1; i < m_searchResult.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i);
		for (int k = 0; k < 10; k++)
		{
			m_variant.append(m_searchResult[i].at(k));
		}
		m_value.append(m_variant);

	}

	rowNow++;
	qExcel->setCellString(QString("A%1:K%2").arg(rowNow).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:K%2").arg(rowNow + 1).arg(rowNow + m_searchResult.size() - 1), m_var);

	qExcel->save();
	qExcel->close();
	delete qExcel;
}

bool SearchRecordCreateExcel::CreateNewExcelForTask(QString m_TaskUUid, QString m_SavePath)
{
	QString strFilePath = QApplication::applicationDirPath() + EXCEL_TEMPLATE_TO_LOAD;
	QExcel * qExcel = new QExcel;
	rowNow = 1;
	QString fileName = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/report";
	QDir dir;
	if (!dir.exists(fileName))
	{
		dir.mkpath(fileName);
	}
	QString path = QString("%1/%2.xlsx").arg(fileName).arg(m_SavePath);
	bool bl = qExcel->OpenExcel(path);
	if (!bl)
		return bl;
	qExcel->selectSheet("Sheet1");

	qExcel->setColumnWidth(1, 15);
	qExcel->setColumnWidth(2, 20);
	qExcel->setColumnWidth(3, 40);
	qExcel->setColumnWidth(4, 15);
	qExcel->setColumnWidth(5, 15);
	qExcel->setColumnWidth(6, 30);
	qExcel->setColumnWidth(7, 15);
	qExcel->setColumnWidth(8, 15);
	
// 	QElapsedTimer timer;
// 	timer.start();
// 	qDebug() << "write start:" << timer.elapsed() << "ms"; timer.restart();
	SetReportHeadMessage(m_TaskUUid, qExcel);
	SetAlarmPoint(m_TaskUUid, qExcel);
	SetUnusualPoint(m_TaskUUid, qExcel);
	SetNormalPoint(m_TaskUUid, qExcel);
//	qDebug() << "write end:" << timer.elapsed() << "ms"; timer.restart();
	qExcel->save();
	qExcel->close();
	delete qExcel;
	return bl;
}

void SearchRecordCreateExcel::SetReportHeadMessage(QString m_TaskUUid, QExcel *qExcel)
{
	WheelReportHeadMessage m_ReportHeadMessage;
	WHEEL_ROBOT_DB.getWheelReportHeadMessageDB(m_TaskUUid, m_ReportHeadMessage);
	QString StartTime = m_ReportHeadMessage.TaskStartTime.replace("T", " ");
	QString StopTime = m_ReportHeadMessage.TaskStopTime.replace("T", " ");
	QString OverallTime = DateCalculate(StartTime, StopTime);
	QString TaskRunTime = QString("开始时间:%1 结束时间:%2 总时常:%3").arg(StartTime).arg(StopTime).arg(OverallTime);
	QString TaskParrolPoint = QString("本次任务共巡检%1点位，正常点位%2个，告警点位%3个，识别异常点位%4个")
		.arg(m_ReportHeadMessage.PatrolPointNum.AllPatrolPointNum)
		.arg(m_ReportHeadMessage.PatrolPointNum.NormalPatrolPointNum)
		.arg(m_ReportHeadMessage.PatrolPointNum.AlarmPatrolPointNum)
		.arg(m_ReportHeadMessage.PatrolPointNum.UnusualPatrolPointNum);
	QString TaskEnvironment = QString("环境温度:%1 环境湿度:%2 环境PM2.5%3 环境风向:%4 环境风速%5")
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_temperature)
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_humidity)
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_pm_2_5)
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_wind_direct)
		.arg(m_ReportHeadMessage.EnvironmentMessage.envi_wind_speed);

	qExcel->mergeCells(rowNow, 1, rowNow, 8); // 合并单元格
	qExcel->setCellString(rowNow, 1, "巡检报告");
	qExcel->setCellTextCenter(rowNow, 1);
	rowNow++;
	qExcel->mergeCells(rowNow, 2, rowNow, 8);
	qExcel->setCellString(rowNow, 1, "任务名称");
	qExcel->setCellString(rowNow, 2, m_ReportHeadMessage.TaskName);

	rowNow++;
	qExcel->mergeCells(rowNow, 2, rowNow, 8);
	qExcel->setCellString(rowNow, 1, "任务时间");
	qExcel->setCellString(rowNow, 2, TaskRunTime);

	rowNow++;
	qExcel->mergeCells(rowNow, 2, rowNow, 8);
	qExcel->setCellString(rowNow, 1, "任务状态");
	qExcel->setCellString(rowNow, 2, m_ReportHeadMessage.TaskStatus);

	rowNow++;
	qExcel->mergeCells(rowNow, 2, rowNow, 8);
	qExcel->setCellString(rowNow, 1, "巡检点");
	qExcel->setCellString(rowNow, 2, TaskParrolPoint);

	rowNow++;
	qExcel->mergeCells(rowNow, 2, rowNow, 8);
	qExcel->setCellString(rowNow, 1, "环境信息");
	qExcel->setCellString(rowNow, 2, TaskEnvironment);
	m_rowHeight = (rowNow)* EXCEL_WRITE_PICTURE_WIDTH;
}

void SearchRecordCreateExcel::SetAlarmPoint(QString m_TaskUUid, QExcel *qExcel)
{
	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_searchResult;
	QList<AlarmUnusualNormalPoint> m_AlarmPoint;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;
	WHEEL_ROBOT_DB.getWheelAlarmPointDB(m_TaskUUid, m_AlarmPoint);
	if (m_AlarmPoint.size() == 0)
	{
		return;
	}
	m_headQL << "序号" << "识别类型" << "点位名称" << "识别结果" << "告警等级" << "识别时间" << "采集信息" << "采集信息";
	for (int i = 0; i < m_headQL.size(); i++)
	{
		m_headQV.append(m_headQL[i]);
	}

	for (int i = 0; i < m_AlarmPoint.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i + 1);
		m_variant.append(m_AlarmPoint[i].DiscernType);
		m_variant.append(m_AlarmPoint[i].PointName);
		m_variant.append(m_AlarmPoint[i].DiscernResult);
		m_variant.append(m_AlarmPoint[i].AlarmGradeOrAuditResult);
		m_variant.append(m_AlarmPoint[i].DiscernTime);
		m_value.append(m_variant);

		m_choo.clear();
		if (!m_AlarmPoint[i].CollectMessage.VisibleLightPath.isEmpty())
		{
			m_choo.append(m_AlarmPoint[i].CollectMessage.VisibleLightPath);
		}
		if (!m_AlarmPoint[i].CollectMessage.InfraredLightPath.isEmpty())
		{
			m_choo.append(m_AlarmPoint[i].CollectMessage.InfraredLightPath);
		}
		m_searchResult.append(m_choo);

	}
	castListListVariant2Variant(m_value, m_var);

	rowNow++;
	qExcel->mergeCells(rowNow, 1, rowNow, 8);
	qExcel->setCellString(rowNow, 1, "告警点位");
	qExcel->setCellTextCenter(rowNow, 1);

	rowNow++;
	qExcel->setCellString(QString("A%1:H%2").arg(rowNow).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:F%2").arg(rowNow+1).arg(rowNow+ m_AlarmPoint.size()), m_var);

	rowNow++;
	/*
	m_rowHeight = m_rowHeight + (2) * EXCEL_WRITE_PICTURE_WIDTH;

	for (int i = 0; i < m_searchResult.size(); i++)
	{
		QList<QString>::ConstIterator ci;
		int column = 7;
		qExcel->setRowHeight(rowNow + 1, EXCEL_SET_ROW_HEIGHT); // 设置行高

		for (ci = m_searchResult[i].constBegin(); ci != m_searchResult[i].constEnd(); ci++)
		{

			if ((*ci).mid(0, 3) == "D:\\")
			{
				if (column == 7)
				{
					qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
				if (column == 8)
				{
					qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
			}
			else
			{
			}
			column++;
		}
		if (i == m_searchResult.size() - 1)
		{
			m_rowHeight = (EXCEL_WRITE_PICTURE_HEIGHT * (i + 1)) + m_rowHeight;
		}
		rowNow++;
	//	qDebug() << "excel插入第" << i + 1 << "条";
	}
	*/
}

void SearchRecordCreateExcel::SetUnusualPoint(QString m_TaskUUid, QExcel *qExcel)
{
	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_searchResult;
	QList<AlarmUnusualNormalPoint> m_UnusualPoint;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;

	WHEEL_ROBOT_DB.getUnusualPointDB(m_TaskUUid, m_UnusualPoint);
	if (m_UnusualPoint.size() == 0)
	{
		return;
	}
	m_headQL << "序号" << "识别类型" << "点位名称" << "识别结果" << "审核结果" << "识别时间" << "采集信息" << "采集信息";
	for (int i = 0; i < m_headQL.size(); i++)
	{
		m_headQV.append(m_headQL[i]);
	}

	for (int i = 0; i < m_UnusualPoint.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i + 1);
		m_variant.append(m_UnusualPoint[i].DiscernType);
		m_variant.append(m_UnusualPoint[i].PointName);
		m_variant.append(m_UnusualPoint[i].DiscernResult);
		m_variant.append(m_UnusualPoint[i].AlarmGradeOrAuditResult);
		m_variant.append(m_UnusualPoint[i].DiscernTime);
		m_value.append(m_variant);

		m_choo.clear();
		if (!m_UnusualPoint[i].CollectMessage.InfraredLightPath.isEmpty())
		{
			m_choo.append(m_UnusualPoint[i].CollectMessage.VisibleLightPath);
		}
		if (!m_UnusualPoint[i].CollectMessage.VisibleLightPath.isEmpty())
		{
			m_choo.append(m_UnusualPoint[i].CollectMessage.InfraredLightPath);
		}
		m_searchResult.append(m_choo);

	}
	castListListVariant2Variant(m_value, m_var);

	rowNow++;
	qExcel->mergeCells(rowNow, 1, rowNow, 8);
	qExcel->setCellString(rowNow, 1, "识别异常点位");
	qExcel->setCellTextCenter(rowNow, 1);

	rowNow++;
	qExcel->setCellString(QString("A%1:H%2").arg(rowNow).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:F%2").arg(rowNow + 1).arg(rowNow + m_UnusualPoint.size()), m_var);

	rowNow++;
	/*
	m_rowHeight = m_rowHeight + (2)* EXCEL_WRITE_PICTURE_WIDTH;

	for (int i = 0; i < m_searchResult.size(); i++)
	{
		QList<QString>::ConstIterator ci;
		int column = 7;
		qExcel->setRowHeight(rowNow + 1, EXCEL_SET_ROW_HEIGHT); // 设置行高

		for (ci = m_searchResult[i].constBegin(); ci != m_searchResult[i].constEnd(); ci++)
		{

			if ((*ci).mid(0, 3) == "D:\\")
			{
				if (column == 7)
				{
					qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
				if (column == 8)
				{
					qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
			}
			else
			{
			}
			column++;
		}
		if (i == m_searchResult.size() - 1)
		{
			m_rowHeight = (EXCEL_WRITE_PICTURE_HEIGHT * (i + 1)) + m_rowHeight;
		}
		rowNow++;
	//	qDebug() << "excel插入第" << i + 1 << "条";
	}
	*/
}
void SearchRecordCreateExcel::SetNormalPoint(QString m_TaskUUid, QExcel *qExcel)
{
	QStringList m_headQL;
	QStringList m_choo;
	QList<QStringList> m_searchResult;
	QList<AlarmUnusualNormalPoint> m_NormalPoint;

	QList<QVariant> m_headQV;
	QVariant m_var;
	QList<QVariant> m_variant;
	QList<QList<QVariant>> m_value;
	WHEEL_ROBOT_DB.getWheelNormalPointDB(m_TaskUUid, m_NormalPoint);
	if (m_NormalPoint.size() == 0)
	{
		return;
	}
	m_headQL << "序号" << "识别类型" << "点位名称" << "识别结果" << "告警等级" << "识别时间" << "采集信息" << "采集信息";
	for (int i = 0; i < m_headQL.size(); i++)
	{
		m_headQV.append(m_headQL[i]);
	}

	for (int i = 0; i < m_NormalPoint.size(); i++)
	{
		m_variant.clear();
		m_variant.append(i + 1);
		m_variant.append(m_NormalPoint[i].DiscernType);
		m_variant.append(m_NormalPoint[i].PointName);
		m_variant.append(m_NormalPoint[i].DiscernResult);
		m_variant.append(m_NormalPoint[i].AlarmGradeOrAuditResult);
		m_variant.append(m_NormalPoint[i].DiscernTime);
		m_value.append(m_variant);

		m_choo.clear();
		if (!m_NormalPoint[i].CollectMessage.VisibleLightPath.isEmpty())
		{
			m_choo.append(m_NormalPoint[i].CollectMessage.VisibleLightPath);
		}
		if (!m_NormalPoint[i].CollectMessage.InfraredLightPath.isEmpty())
		{
			m_choo.append(m_NormalPoint[i].CollectMessage.InfraredLightPath);
		}
		m_searchResult.append(m_choo);

	}
	castListListVariant2Variant(m_value, m_var);

	rowNow++;
	qExcel->mergeCells(rowNow, 1, rowNow, 8);
	qExcel->setCellString(rowNow, 1, "正常点位");
	qExcel->setCellTextCenter(rowNow, 1);

	rowNow++;
	qExcel->setCellString(QString("A%1:H%2").arg(rowNow).arg(rowNow), m_headQV);

	castListListVariant2Variant(m_value, m_var);
	qExcel->setCellStringC(QString("A%1:F%2").arg(rowNow + 1).arg(rowNow + m_NormalPoint.size()), m_var);

	rowNow++;
	/*
	m_rowHeight = m_rowHeight + (2)* EXCEL_WRITE_PICTURE_WIDTH;

	for (int i = 0; i < m_searchResult.size(); i++)
	{
		QList<QString>::ConstIterator ci;
		int column = 7;
		qExcel->setRowHeight(rowNow + 1, EXCEL_SET_ROW_HEIGHT); // 设置行高

		for (ci = m_searchResult[i].constBegin(); ci != m_searchResult[i].constEnd(); ci++)
		{

			if ((*ci).mid(0, 3) == "D:\\")
			{
				if (column == 7)
				{
					qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1), (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
				if (column == 8)
				{
					qExcel->setCellPicture(EXCEL_PICTURE_LEFT * (7 - 1) + 94, (EXCEL_WRITE_PICTURE_HEIGHT * i) + m_rowHeight, (*ci));//27
				}
			}
			else
			{
			}
			column++;
		}
		if (i == m_searchResult.size() - 1)
		{
			m_rowHeight = (EXCEL_WRITE_PICTURE_HEIGHT * (i + 1)) + m_rowHeight;
		}
		rowNow++;
	//	qDebug() << "excel插入第" << i + 1 << "条";
	}
	*/
}

void SearchRecordCreateExcel::insertUUidIntoExcel()
{
// 	qExcel->OpenExcel("D:/vvvv.xls");
// 	qExcel->selectSheet("Sheet5");
// 	for (int i = 0; i < 5465; i++)
// 	{
// 		
// 		boost::uuids::random_generator rgen;//随机生成器  
// 		boost::uuids::uuid ssid = rgen();//生成一个随机的UUID
// 		std::string tmp = boost::lexical_cast<std::string>(ssid);
// 		boost::erase_all(tmp, "-");
// 		QString qsid = QString::fromStdString(tmp);
// 
// 		qExcel->setCellString(i+2, 1, qsid); // 设置单元格的值
// 		
// 	}
// 	qExcel->save();
// 	qExcel->close();
}

QStringList SearchRecordCreateExcel::LoadExcelAndGetData(QString m_OpenPath)
{
// 	QExcel * c_excel = new QExcel;
// 	QStringList sheetName;
// 	c_excel->OpenExcel(m_OpenPath);
// 	int c_sheetCount =  c_excel->getSheetsCount();
// 	for (int i = 0; i < c_sheetCount; i++)
// 	{
// 		sheetName.append(c_excel->getSheetName(i + 1));
// 	}
// 	setExcelClassObject(c_excel);
// 	is_link = true;


	m_excelC = new QExcel;
	QStringList sheetName;
	m_excelC->OpenExcel(m_OpenPath);
	int c_sheetCount = m_excelC->getSheetsCount();
	for (int i = 0; i < c_sheetCount; i++)
	{
		sheetName.append(m_excelC->getSheetName(i + 1));
	}
//	setExcelClassObject(c_excel);
	is_link = true;

//	delete c_excel;
	return sheetName;
}

QList<QStringList> SearchRecordCreateExcel::chooseSheetName(QString shtN)
{
	int c_i, c_j, c_k, c_l;
	m_excelC->selectSheet(shtN);
	m_excelC->getUsedRange(&c_i, &c_j, &c_k, &c_l);
	qDebug() << c_i << c_j << c_k << c_l;
	char c_r = 65;
	char c_t = 65;
	c_r = c_r + c_j - 1;
	c_t = c_t + c_l - 1;
	QList<int> count;
	count.append(c_i);count.append(c_j);count.append(c_k);count.append(c_l);

	QVariant var = m_excelC->getCellValue(QString("%1%2:H%3%4").arg(c_r).arg(c_i).arg(c_t).arg(c_k));
	QList<QStringList> c_data;
	castVariant2ListListVariant(var, c_data, count);

	return c_data;
}

void SearchRecordCreateExcel::setExcelClassObject(QExcel *q)
{
	m_excelC = q;
}

QExcel* SearchRecordCreateExcel::getExcelClassObject()
{
	return m_excelC;
}

QString SearchRecordCreateExcel::DateCalculate(QString StartTime, QString StopTime)
{
	int startYear = StartTime.mid(0, 4).toInt();
	int startMonth = StartTime.mid(5, 2).toInt();
	int startDay = StartTime.mid(8, 2).toInt();
	int startHour = StartTime.mid(11, 2).toInt();
	int startMinute = StartTime.mid(14, 2).toInt();
	int startSecond = StartTime.mid(17, 2).toInt();

	int stopYear = StopTime.mid(0, 4).toInt();
	int stopMonth = StopTime.mid(5, 2).toInt();
	int stopDay = StopTime.mid(8, 2).toInt();
	int stopHour = StopTime.mid(11, 2).toInt();
	int stopMinute = StopTime.mid(14, 2).toInt();
	int stopSecond = StopTime.mid(17, 2).toInt();

	int OverallSecond = stopSecond - startSecond;
	int OverallMinute = stopMinute - startMinute;
	int OverallHour = stopHour - startHour;
	int OverallDay = stopDay - startDay;
	int OverallMonth = stopMonth - startMonth;
	int OverallYear = stopYear - startYear;

	if (OverallSecond < 0)
	{
		OverallMinute = OverallMinute - 1;
		OverallSecond = OverallSecond + 60;
	}
	if (OverallMinute < 0)
	{
		OverallHour = OverallHour - 1;
		OverallMinute = OverallMinute + 60;
	}
	if (OverallHour < 0)
	{
		OverallDay = OverallDay - 1;
		OverallHour = OverallHour + 24;
	}
	if (OverallDay < 0)
	{
		OverallMonth = OverallMonth - 1;
		OverallDay = OverallDay + 30;
	}
	if (OverallMonth < 0)
	{
		OverallYear = OverallYear - 1;
		OverallMonth = OverallMonth + 12;
	}
	QString OverallTime = QString("%1年%2月%3日%4时%5分%6秒")
		.arg(OverallYear).arg(OverallMonth).arg(OverallDay)
		.arg(OverallHour).arg(OverallMinute).arg(OverallSecond);
	return OverallTime;
}

void SearchRecordCreateExcel::castVariant2ListListVariant(const QVariant &var, QList<QStringList> &res, QList<int> cot)
{
	QVariantList varRows = var.toList();
	if (varRows.isEmpty())
	{
		return;
	}
	const int rowCount = varRows.size();
	QVariantList rowData;
	for (int i = 1; i < rowCount; ++i)
	{
		QStringList c_data;
		rowData = varRows[i].toList();
		for (int j = 1; j < cot[3]; j++)
		{
			c_data.append(rowData[j].toString());
		}
		res.append(c_data);
	}
}

void SearchRecordCreateExcel::castListListVariant2Variant(const QList<QList<QVariant>> &cells, QVariant &res)
{
	QVariantList vars;
	const int rows = cells.size();
	for (int i = 0; i < rows; ++i)
	{
		vars.append(QVariant(cells[i]));
	}
	res = QVariant(vars);
}

void SearchRecordCreateExcel::setChooseListCreateExcel(QList<QStringList> m_searchResult, QString m_SavePath)
{
//	CreateNewExcelForList(m_searchResult, m_SavePath);
}
void SearchRecordCreateExcel::setChooseTaskCreateExcel(QString m_TaskUUid, QString m_SavePath)
{
	CreateNewExcelForTask(m_TaskUUid, m_SavePath);
}

void SearchRecordCreateExcel::testNewPic()
{
	QExcel * qExcel = new QExcel;
	qExcel->OpenExcel("D:/abcdef.xlsx");
	qExcel->selectSheet("Sheet1");

	qExcel->setCellPictureNew();

	qExcel->save();
	qExcel->close();
	delete qExcel;
}