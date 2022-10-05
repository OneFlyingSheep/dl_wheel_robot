#pragma once

#include <QWidget>
#include <QLabel>
#include <QCheckBox>
#include "common/DLWheelRobotGlobalDef.hpp"

struct CheckBoxInfo
{
	QString checkBoxText;
	QString checkBoxId;
	bool isEnable;
	bool isChecked;
	CheckBoxInfo(QString text, QString id = "1", bool isEnable = true, bool isChecked = false)
	{
		this->checkBoxText = text;
		this->checkBoxId = id;
		this->isEnable = isEnable;
		this->isChecked = isChecked;
	}
};

class CheckBoxWidget : public QWidget
{
	Q_OBJECT

public:
	CheckBoxWidget(QWidget *parent = NULL);
	~CheckBoxWidget();

	// ���õ�ǰcheckBox����;
	void setCheckBoxType(WheelCheckBoxTypeEnum type);

	// ��ȡ��ǰcheckBox����;
	WheelCheckBoxTypeEnum getCheckBoxType();

	// ���checkBox;
	void addCheckBoxWidget(QString widgetText, QList<WheelPackageCheckBoxStruct> checkBoxDataList);

	void addCheckBoxWidget(QString widgetText, QList<CheckBoxInfo> checkBoxInfoList);

	// ��ȡ��ǰѡ��checkBox�� ID list;
	QStringList getCheckedIdList();

    // ����checkBox;
    void resetCheckBox();

signals:
	// checkBox���������Ϣ;
	void signalCheckBoxClicked(QString checkBoxId, bool isChecked);

	private slots:
	// checkBoxȫ�����;
	void onChooseAllCheckBoxClicked(int checkBoxState);

	// checkBox��ʾ������;
	void onShowMoreCheckBoxClicked(int checkBoxState);

	// checkBox���;
	void onCheckBoxItemClicked(int checkBoxState);

private:
	// ���浱ǰ�����е�checkBox;
	QList<QCheckBox*> m_checkBoxList;

    // �����һ��ȫ��ѡ��checkBox;
    QList<QCheckBox*> m_checkBoxChooseAllList;

	// ���浱ǰ�����е�checkBox����Ϣ;
	QList<CheckBoxInfo> m_checkBoxInfoList;

    // CheckBox��Ϣ;
	QList<WheelPackageCheckBoxStruct> m_checkBoxDataList;
	// checkBox֮��ļ��;
	int m_checkBoxInterval;

	WheelCheckBoxTypeEnum m_checkBoxType;
};
