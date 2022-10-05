#pragma once

#include <QWidget>

namespace Ui {
	class CollectStandardizationWidget;
}


class CollectStandardizationWidget : public QWidget
{
	Q_OBJECT

public:
	explicit CollectStandardizationWidget(QWidget *parent = 0);
	~CollectStandardizationWidget();

	//计算新值
	void CalculateZoomValue();
	//切换zoom值
	void CutZoomTxt();
	//是否使用默认值
	void DefaultTxt();
signals:
	void startCollectPhotoSignals();

private slots:

private:
	Ui::CollectStandardizationWidget *ui;
};
