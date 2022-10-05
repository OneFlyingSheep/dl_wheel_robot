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

	void ClearMap();							//���map�༭���沢���ر���·��

private:
    // ��ʼ���ؼ�;
	void initWidget();

signals:
    // ����Ѳ����ź�;
    void signalSendCollectPatrolPoint();
    // ���ʹ򿪵�ͼ�ź�;
    void signalSendOpenSmap(QString);
	void OpenSmapFileSignal(QString);					//���������棬��ʵʱ��ص�smap

private:
	BaseWidget* m_pCollectMapWidget;
    bool m_isDevelopProgram;
	DLMapEditorWidget *m_pMapEditorWidget;
};

#endif