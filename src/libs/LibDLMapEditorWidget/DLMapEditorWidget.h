#ifndef DLMAPEDITORWIDGET_ALEXWEI_20180520_H
#define DLMAPEDITORWIDGET_ALEXWEI_20180520_H

#include <QWidget>
#include <boost/bind.hpp>
#include "common/DLWheelRobotGlobalDef.hpp"
class DLCustomScene;
class DLView;
class DLFrameView;
class DataTransfer;
class DLStatusBar;
class LaserParamWidget;
class SmapBuilder;
class QListWidget;
class QLineEdit;
class QPushButton;
class QLabel;
class QListWidgetItem;
class QTabWidget;
class QButtonGroup;
class DLMapListWidget;
class DLCollectMapView;
class QGroupBox;

//////////////////////////////////////////

class InputWidget;
class QGraphicsItem;
class DLMapEditorWidget : public QWidget
{
	Q_OBJECT

	enum COLLECT_BTN_TYPE
	{
		OPEN_SMAP_TYPE,					//�򿪵����Ϣ�ļ�
		SAVE_SMAP_TYPE,					//��������Ϣ�ļ�
		SAVEAS_SMAP_TYPE,				//�������Ϣ�ļ�
		DOWNLOAD_2D_MAP_TYPE,			//����2D��ͼ
		UPLOAD_SMAP_TYPE,				//�ϴ�Smap
		UPLOAD_MAPINFO_TYPE,			//�ϴ������Ϣ�ļ�
		SELECT_TYPE,					//ѡ��
		OVERFITTING_TYPE,				//���	
		ADD_PATH_TYPE,					//���·��
		ADD_ADVANCED_AREA_TYPE,			//�߼�����
		RELOCATION_TYPE,				//�ض�λ
		KEY_MODE_TYPE,					//����ģʽ
		TASK_MODE_TYPE,					//����ģʽ
		HAND_MODE_TYPE,					//�ֱ�ģʽ
		URGENCY_RELOCATION_TYPE			//������λģʽ
	};

public:
	DLMapEditorWidget(QWidget *parent = 0);
	~DLMapEditorWidget();

	void ClearMap();						//��յ�ͼ
	
	//core��ͼ�ϴ���ϵĻص�
	void smapUploadCompleted(bool retcode, QString desc);

protected:
	virtual void closeEvent(QCloseEvent *event);

signals:
    // core�ص�;    
	void signalRobotModeStateCallBack(WheelRobotSwitchRunningStatus);

	void OpenSmapFileSignal(QString strSmapFile);							//���ļ�

	
private slots:
    void slot_on_open_smap(QString file);
	//file
	void slot_on_set_build_map_param();					//����smap��ͼ �Ĳۺ���
	void slot_on_download_2dmap();						//����2D��ͼ �Ĳۺ���
	void slot_on_download_smap();						//����smap��ͼ �Ĳۺ���
	void slot_on_upload_smap();							//�ϴ�smap �Ĳۺ���
	void slot_on_load_smap();							//��smap��ͼ �Ĳۺ���
	void slot_on_save_smap();							//����smap��ͼ �Ĳۺ���
	void slot_on_load_backstage_map();					//�򿪺�̨��ͼ �Ĳۺ���
	void slot_on_save_backstage_map();					//�����̨��ͼ �Ĳۺ���
	void slot_on_unload_smap();							//ж�ص�ͼ �Ĳۺ���
	void slot_on_quit();								//�˳� �Ĳۺ���
		
	void slot_on_load_backstagelandmark();				//����Ѳ���Ĳۺ���
	void slot_on_connect_backstagelandmark();			//����·���Ĳۺ���


	//edit
	void slot_on_undo();
	void slot_on_redo();
	void slot_on_cut();
	void slot_on_copy();
	void slot_on_paste();
	void slot_on_delete();
	void slot_on_selectall();
	void slot_on_find();
	void CutOutAreaBtnSlot();							//��ȡ��ť�Ĳۺ���
	void SetBgPixmapBtnSlot();							//���ñ���ͼƬ��ť�Ĳۺ���

	//operate
	void slot_on_rotate();
	void slot_on_select();
	void slot_on_landmark();
	void slot_on_bezier();
	void slot_on_line();
	void slot_on_virtual_line();
	void slot_on_forbbidon_line();
	void slot_on_advanced_area();
	void slot_on_picture();
	void slot_on_device_area();
	void slot_on_station_area();
	void slot_on_relocation();
	void slot_on_add_landmark();


	void slot_on_build_map();
    void slot_on_choose_map(QString download_path, QString filename, int file_type);
	void slot_on_transfer_finished(int command_type, int execCode, QString exeParam);
	void slot_on_build_process(float process);
	void slot_on_build_finished();

    void slot_on_start_slam();
    void slot_on_end_slam();

	void SelectedChangedSlot();						//ѡ��Ľڵ㷢���ı�Ĳۺ���
	void ChangeTypeSlot(const QString &);			//scene���ͷ����ı�Ĳۺ���
	void SMAPChangedSlot();

	void ReadFinishedSlot(bool bIsRunning);			//��дsmap�ļ������Ĳۺ���

	void BtnClickeSlot(int);  //��ťclicked�Ĳۺ���


private:
	void initWidget();								//��ʼ������
	void initSceneView();							//��ʼ��scene��view
	void initStatusBar();							//��ʼ��״̬��
	void initCoreFunction();						//��ʼ��core
	void SaveSMAPFile();							//����smap�ļ��ۺ���
	void SaveAsSMAPFile(QString &strSaveAsFile);							//���Ϊsmap�ļ��ۺ���
	void SaveBSMAPFile(QString strFilePath = "");							//����bsmap��ͼ
	void UploadSMAPFile();							//�ϴ�smap��ͼ
	
	QPushButton *CreateToolBtn(QWidget *parentWgt, const QString strName, const QString &strObjectName, const QString &strIconPath);  //������ť��


private:
    QPushButton *m_pCollectStartSlamBtn;					//��ʼɨ��
    QPushButton *m_pCollectEndSlamBtn;						//����ɨ��
    QPushButton *m_pDownload2DBtn;							//����2D��ͼ	
	QPushButton *m_pDownloadSMAPBtn;						//����SMAP��ͼ		
    QPushButton *m_pUploadSMAPBtn;							//�ϴ�smap��ͼ
	QPushButton *m_pCreateSMAPBtn;							//����smap��ͼ

	//QTabWidget *m_pMapEditeTabWgt;							//tabwgt
	QPushButton *m_pOpenSMAPBtn;							//��smap��ͼ
	QPushButton *m_pSaveSMAPBtn;							//����smap��ͼ

// 	QPushButton *m_pLoadBackstagelandmarkBtn;				//����Ѳ��㰴ť
// 	QPushButton *file_connect_landmark_button_;				//����·����ť
// 	QPushButton *m_pOpenBackstagemapBtn;					//�򿪺�̨��ͼ��ť
	QPushButton *m_pUnloadMapBtn;							//ж�ص�ͼ��ť
	QPushButton *m_pQuitBtn;								//�˳���ť


	QPushButton *edit_undo_button_;							//����
	QPushButton *edit_redo_button_;							//����
	QPushButton *edit_cut_button_;							//����
	QPushButton *edit_copy_button_;							//����
	QPushButton *edit_paste_button_;						//ճ��
	QPushButton *edit_delete_button_;						//ɾ��
	QPushButton *edit_selectall_button_;					//ȫѡ
	QPushButton *edit_find_button_;							//����
	QPushButton *m_pCutOutAreaBtn;							//����
	QPushButton *m_pSetBgPixmapBtn;

	QPushButton *operate_select_button_;
	QPushButton *operate_coordinate_transform_button_;
	QPushButton *operate_landmark_button_;
	QPushButton *operate_bezier_button_;
	QPushButton *operate_advanced_area_button_;
	QPushButton *operate_normal_line_button_;
	QPushButton *operate_virtual_line_button_;
	QPushButton *operate_forbidden_line_button_;
	QPushButton *operate_picture_button_;
	QPushButton *operate_device_area_button_;
	QPushButton *operate_station_button_;
	QPushButton *operate_relocation_button_;
	QPushButton *operate_add_relocation_button_;


private:
	DLStatusBar *m_pStatusBar;
	DLCustomScene *m_pScene;
	DLCollectMapView *m_pView;
	//DLFrameView *m_pView;
	DataTransfer *m_pThreadDataTransfer;						//�߳������ϴ�smap��������
	LaserParamWidget *laserParamWidget_;
	SmapBuilder *smapBuilder_;
  //  DLMapListWidget *mapListWidget_;
    // ��ǰģʽ;
    InputWidget* m_pCurrentModeLabel;
	QList<QGraphicsItem *> m_lstSelectedsItems;
	bool m_bSMAPIsChanged;

	QString m_strFileName;										//���浱ǰ���ļ���
	bool m_bIsUploadSmap;										//��¼�Ƿ��ϴ�������

	QButtonGroup *m_pFileBtnGroup{nullptr};				//��ʼɨ��/����ɨ��/����2d��ͼ/�ϴ�smap/����smap/��������Ϣ
	QButtonGroup *m_pOperatorBtnGroup{nullptr};				//ѡ��/�Զ��ӵ�/�ֶ��ӵ�/���·��/�߼�����/�ض�λ
	//QButtonGroup *m_pModelBtnGroup{nullptr};				//����ģʽ/����ģʽ/�ֱ�ģʽ/������λģʽ
	QString m_strOpenMapPath{""};
	DLMapListWidget *m_pMapListWgt{nullptr};		//���ص�ͼ

	QGroupBox *m_pFileGroupBox{ nullptr };
	QGroupBox *m_pOperatorGroupBox{ nullptr };
};


#endif //DLCONTROLWIDGET_ALEXWEI_20180426_H

