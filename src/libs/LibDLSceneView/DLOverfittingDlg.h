#ifndef __DL_OVERFITING_DLG_H__
#define __DL_OVERFITING_DLG_H__

#pragma execution_character_set("utf-8")

#include <QDialog>


namespace Ui {
	class DLOverfittingDlg;
}

class DLCustomScene;

class  DLOverfittingDlg : public  QDialog
{
    Q_OBJECT

public:
    DLOverfittingDlg(QWidget *parent = NULL);
    ~DLOverfittingDlg();

	void SetScene(DLCustomScene *pCustomScene);

	void SetSelectLineID(const QString &strLineName);			//设置拟合线的内容	
	void SetSelectPointIDs(const QString &strPointName);		//设置拟合点的内容

	void ClearContent();

private slots:
	void SelectLineBtnSlot();//选中拟合标准线的槽函数
	void SelectPointsBtnSlot();//选中拟合点的槽函数
	void StartOverfittingBtnSlot();			//开始拟合
	void CancelBtnSlot();		//取消拟合

private:
	Ui::DLOverfittingDlg *ui;
	DLCustomScene *m_pCustomScene;
};

#endif //__DL_OVERFITING_DLG_H__