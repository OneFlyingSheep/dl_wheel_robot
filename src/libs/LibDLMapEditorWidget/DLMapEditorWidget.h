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
		OPEN_SMAP_TYPE,					//打开点边信息文件
		SAVE_SMAP_TYPE,					//保存点边信息文件
		SAVEAS_SMAP_TYPE,				//另存点边信息文件
		DOWNLOAD_2D_MAP_TYPE,			//下载2D地图
		UPLOAD_SMAP_TYPE,				//上传Smap
		UPLOAD_MAPINFO_TYPE,			//上传点边信息文件
		SELECT_TYPE,					//选择
		OVERFITTING_TYPE,				//拟合	
		ADD_PATH_TYPE,					//添加路径
		ADD_ADVANCED_AREA_TYPE,			//高级区域
		RELOCATION_TYPE,				//重定位
		KEY_MODE_TYPE,					//键盘模式
		TASK_MODE_TYPE,					//任务模式
		HAND_MODE_TYPE,					//手柄模式
		URGENCY_RELOCATION_TYPE			//紧急定位模式
	};

public:
	DLMapEditorWidget(QWidget *parent = 0);
	~DLMapEditorWidget();

	void ClearMap();						//清空地图
	
	//core地图上传完毕的回调
	void smapUploadCompleted(bool retcode, QString desc);

protected:
	virtual void closeEvent(QCloseEvent *event);

signals:
    // core回调;    
	void signalRobotModeStateCallBack(WheelRobotSwitchRunningStatus);

	void OpenSmapFileSignal(QString strSmapFile);							//打开文件

	
private slots:
    void slot_on_open_smap(QString file);
	//file
	void slot_on_set_build_map_param();					//构建smap地图 的槽函数
	void slot_on_download_2dmap();						//下载2D地图 的槽函数
	void slot_on_download_smap();						//下载smap地图 的槽函数
	void slot_on_upload_smap();							//上传smap 的槽函数
	void slot_on_load_smap();							//打开smap地图 的槽函数
	void slot_on_save_smap();							//保存smap地图 的槽函数
	void slot_on_load_backstage_map();					//打开后台地图 的槽函数
	void slot_on_save_backstage_map();					//保存后台地图 的槽函数
	void slot_on_unload_smap();							//卸载地图 的槽函数
	void slot_on_quit();								//退出 的槽函数
		
	void slot_on_load_backstagelandmark();				//加载巡检点的槽函数
	void slot_on_connect_backstagelandmark();			//生成路劲的槽函数


	//edit
	void slot_on_undo();
	void slot_on_redo();
	void slot_on_cut();
	void slot_on_copy();
	void slot_on_paste();
	void slot_on_delete();
	void slot_on_selectall();
	void slot_on_find();
	void CutOutAreaBtnSlot();							//截取按钮的槽函数
	void SetBgPixmapBtnSlot();							//设置背景图片按钮的槽函数

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

	void SelectedChangedSlot();						//选择的节点发生改变的槽函数
	void ChangeTypeSlot(const QString &);			//scene类型发生改变的槽函数
	void SMAPChangedSlot();

	void ReadFinishedSlot(bool bIsRunning);			//读写smap文件结束的槽函数

	void BtnClickeSlot(int);  //按钮clicked的槽函数


private:
	void initWidget();								//初始化界面
	void initSceneView();							//初始化scene和view
	void initStatusBar();							//初始化状态栏
	void initCoreFunction();						//初始话core
	void SaveSMAPFile();							//保存smap文件槽函数
	void SaveAsSMAPFile(QString &strSaveAsFile);							//另存为smap文件槽函数
	void SaveBSMAPFile(QString strFilePath = "");							//保存bsmap地图
	void UploadSMAPFile();							//上传smap地图
	
	QPushButton *CreateToolBtn(QWidget *parentWgt, const QString strName, const QString &strObjectName, const QString &strIconPath);  //创建按钮的


private:
    QPushButton *m_pCollectStartSlamBtn;					//开始扫描
    QPushButton *m_pCollectEndSlamBtn;						//结束扫描
    QPushButton *m_pDownload2DBtn;							//下载2D地图	
	QPushButton *m_pDownloadSMAPBtn;						//下载SMAP地图		
    QPushButton *m_pUploadSMAPBtn;							//上传smap地图
	QPushButton *m_pCreateSMAPBtn;							//构建smap地图

	//QTabWidget *m_pMapEditeTabWgt;							//tabwgt
	QPushButton *m_pOpenSMAPBtn;							//打开smap地图
	QPushButton *m_pSaveSMAPBtn;							//保存smap地图

// 	QPushButton *m_pLoadBackstagelandmarkBtn;				//加载巡检点按钮
// 	QPushButton *file_connect_landmark_button_;				//生成路径按钮
// 	QPushButton *m_pOpenBackstagemapBtn;					//打开后台地图按钮
	QPushButton *m_pUnloadMapBtn;							//卸载地图按钮
	QPushButton *m_pQuitBtn;								//退出按钮


	QPushButton *edit_undo_button_;							//撤销
	QPushButton *edit_redo_button_;							//重做
	QPushButton *edit_cut_button_;							//剪切
	QPushButton *edit_copy_button_;							//复制
	QPushButton *edit_paste_button_;						//粘贴
	QPushButton *edit_delete_button_;						//删除
	QPushButton *edit_selectall_button_;					//全选
	QPushButton *edit_find_button_;							//查找
	QPushButton *m_pCutOutAreaBtn;							//截屏
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
	DataTransfer *m_pThreadDataTransfer;						//线程用来上传smap到机器人
	LaserParamWidget *laserParamWidget_;
	SmapBuilder *smapBuilder_;
  //  DLMapListWidget *mapListWidget_;
    // 当前模式;
    InputWidget* m_pCurrentModeLabel;
	QList<QGraphicsItem *> m_lstSelectedsItems;
	bool m_bSMAPIsChanged;

	QString m_strFileName;										//保存当前的文件名
	bool m_bIsUploadSmap;										//记录是否上传机器人

	QButtonGroup *m_pFileBtnGroup{nullptr};				//开始扫描/结束扫描/下载2d地图/上传smap/下载smap/保存点边信息
	QButtonGroup *m_pOperatorBtnGroup{nullptr};				//选择/自动加点/手动加点/添加路径/高级区域/重定位
	//QButtonGroup *m_pModelBtnGroup{nullptr};				//任务模式/键盘模式/手柄模式/紧急定位模式
	QString m_strOpenMapPath{""};
	DLMapListWidget *m_pMapListWgt{nullptr};		//下载地图

	QGroupBox *m_pFileGroupBox{ nullptr };
	QGroupBox *m_pOperatorGroupBox{ nullptr };
};


#endif //DLCONTROLWIDGET_ALEXWEI_20180426_H

