#ifndef __DL_WHEEL_ROBOT_COLLECT_MAP_H__
#define __DL_WHEEL_ROBOT_COLLECT_MAP_H__

#include <QWidget>

class BaseWidget;
class DLMapEditorWidget;
class DLWheelCollectMapWidget : public QWidget
{
	Q_OBJECT

public:
	DLWheelCollectMapWidget(bool isDevelopProgram = true, QWidget *parent = NULL);
	~DLWheelCollectMapWidget();

	void ClearMap();							//清空map编辑界面并返回保存路径

private:
    // 初始化控件;
	void initWidget();

signals:
    // 发送巡检点信号;
    void signalSendCollectPatrolPoint();
    // 发送打开地图信号;
    void signalSendOpenSmap(QString);
	void OpenSmapFileSignal(QString);					//发给主界面，打开实时监控的smap

private:
	BaseWidget* m_pCollectMapWidget;
    bool m_isDevelopProgram;
	DLMapEditorWidget *m_pMapEditorWidget;
};

#endif