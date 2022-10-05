#pragma once
#include <QList>
#include <QStringList>
#include <QDebug>
#include "qexcel.h"
#include <QThread>
#include "common/Singleton.hpp"
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"

class SearchRecordCreateExcel : public QObject
{
	Q_OBJECT
public:
	SearchRecordCreateExcel(QObject *parent = NULL);
	~SearchRecordCreateExcel();

public:
	void CreateNewExcelForList(QList<QStringList> m_searchResult, QString m_SavePath, WheelRobotExcelChoose cho);
	//�豸�澯��ѯȷ��
	void createExcelDeviceAlarmSearchAudit(QList<QStringList> m_searchResult, QString m_SavePath);
	//Ѳ�������
	void createExcelPatrolResultBrowse(QList<QStringList> m_searchResult, QString m_SavePath);
	//�Աȷ���
	void createExcelCompareAnalyze(QList<QStringList> m_searchResult, QString m_SavePath);
	//�Զ��屨��
	void createExcelCreateReport(QList<QStringList> m_searchResult, QString m_SavePath);
	//��׼��λ��
	void createExcelStandardPoint(QList<QStringList> m_searchResult, QString m_SavePath);
	//ʶ���쳣��λ��ѯ
	void createExcelRecognitionUnusualPointSearch(QList<QStringList> m_searchResult, QString m_SavePath);
	//Ѳ���λ����
	void createExcelPatrolPointSet(QList<QStringList> m_searchResult, QString m_SavePath);

	bool CreateNewExcelForTask(QString m_TaskUUid, QString m_SavePath);

	void SetReportHeadMessage(QString m_TaskUUid, QExcel * qExcel);
	void SetAlarmPoint(QString m_TaskUUid, QExcel * qExcel);
	void SetUnusualPoint(QString m_TaskUUid, QExcel * qExcel);
	void SetNormalPoint(QString m_TaskUUid, QExcel * qExcel);

	void insertUUidIntoExcel();

	//����excel
	QStringList LoadExcelAndGetData(QString m_OpenPath);
	//ѡ��sheet
	QList<QStringList> chooseSheetName(QString shtN);

	void setExcelClassObject(QExcel * q);
	QExcel * getExcelClassObject();

	QString DateCalculate(QString StartTime, QString StopTime);

	void castVariant2ListListVariant(const QVariant & var, QList<QStringList>& res, QList<int> cot);
	void castListListVariant2Variant(const QList<QList<QVariant>>& cells, QVariant & res);

	void testNewPic();

public slots:
	void setChooseListCreateExcel(QList<QStringList> m_searchResult, QString m_SavePath);
	void setChooseTaskCreateExcel(QString m_TaskUUid, QString m_SavePath);

private:
	int rowNow;
	float m_rowHeight;
	QExcel *m_excelC;
	bool is_link;
};

