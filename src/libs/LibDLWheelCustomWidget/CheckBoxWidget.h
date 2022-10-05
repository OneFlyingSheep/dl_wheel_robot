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

	// 设置当前checkBox类型;
	void setCheckBoxType(WheelCheckBoxTypeEnum type);

	// 获取当前checkBox类型;
	WheelCheckBoxTypeEnum getCheckBoxType();

	// 添加checkBox;
	void addCheckBoxWidget(QString widgetText, QList<WheelPackageCheckBoxStruct> checkBoxDataList);

	void addCheckBoxWidget(QString widgetText, QList<CheckBoxInfo> checkBoxInfoList);

	// 获取当前选中checkBox的 ID list;
	QStringList getCheckedIdList();

    // 重置checkBox;
    void resetCheckBox();

signals:
	// checkBox点击触发信息;
	void signalCheckBoxClicked(QString checkBoxId, bool isChecked);

	private slots:
	// checkBox全部点击;
	void onChooseAllCheckBoxClicked(int checkBoxState);

	// checkBox显示更多点击;
	void onShowMoreCheckBoxClicked(int checkBoxState);

	// checkBox点击;
	void onCheckBoxItemClicked(int checkBoxState);

private:
	// 保存当前项所有的checkBox;
	QList<QCheckBox*> m_checkBoxList;

    // 保存第一个全部选项checkBox;
    QList<QCheckBox*> m_checkBoxChooseAllList;

	// 保存当前项所有的checkBox的信息;
	QList<CheckBoxInfo> m_checkBoxInfoList;

    // CheckBox信息;
	QList<WheelPackageCheckBoxStruct> m_checkBoxDataList;
	// checkBox之间的间距;
	int m_checkBoxInterval;

	WheelCheckBoxTypeEnum m_checkBoxType;
};
