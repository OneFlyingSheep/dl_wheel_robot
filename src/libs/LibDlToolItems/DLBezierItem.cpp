#include <QtWidgets>
#include "DLBezierItem.h"
#include "DLLandmarkItem.h"
#include "DLControlTransformItem.h"
#include "LibDLSceneView/DLCustomScene.h"

#define PI    3.1415

using namespace HShape;

#define DEFAULT_VAL "0.8"
#define MEDIAN_VAL "0.5"
#define MIN_VAL "0.3"

BezierPropertyWidget::BezierPropertyWidget(QWidget *parent)
	:QDialog(parent)
	,m_bIsBatchSet(false)
	, m_bIsPressed(false)
	, m_pPositiveCheckBox(NULL)
	, m_pNagetiveCheckBox(NULL)
{
	bezier_groupBox_ = new QGroupBox("路径设置");

	QVBoxLayout *beizer_base_layout = new QVBoxLayout;
	QGridLayout *beizer_base_pro_layout = new QGridLayout;
	bezier_base_pro_groupBox_ = new QGroupBox;
	start_index_id_label_ = new QLabel("起点ID");
	start_index_id_lineEdit_ = new QLineEdit;
	end_index_id_label_ = new QLabel("终点ID");
	end_index_id_lineEdit_ = new QLineEdit;
		
	start_index_id_lineEdit_->setFocusPolicy(Qt::NoFocus);
	end_index_id_lineEdit_->setFocusPolicy(Qt::NoFocus);

	direction_type_label_ = new QLabel("方向");
	direction_type_combobox_ = new QComboBox;
	widget_label_ = new QLabel("权重");
	weight_lineEdit_ = new QLineEdit;
	direction_type_combobox_->addItem("双向");
	direction_type_combobox_->addItem("单向");
	
	weight_lineEdit_->setFocusPolicy(Qt::NoFocus);

	path_passing_checkBox_ = new QCheckBox("是否通行");
	path_passing_checkBox_->setCheckable(true);
	path_passing_checkBox_->setCheckState(Qt::Checked);

	m_rotate_checkBox = new QCheckBox("是否可以旋转");
	m_rotate_checkBox->setCheckable(true);
	m_rotate_checkBox->setCheckState(Qt::Checked);

	m_pUltrasonicChx = new QCheckBox("超声波");
	m_pUltrasonicChx->setCheckable(true);
	m_pUltrasonicChx->setCheckState(Qt::Checked);

	m_pFallArrestChx = new QCheckBox("防跌落");
	m_pFallArrestChx->setCheckable(true);
	m_pFallArrestChx->setCheckState(Qt::Checked);

	QHBoxLayout *pRotateLayout = new QHBoxLayout;
	pRotateLayout->addWidget(m_rotate_checkBox);
	pRotateLayout->addWidget(m_pUltrasonicChx);

	beizer_base_pro_layout->addWidget(start_index_id_label_, 0, 0);
	beizer_base_pro_layout->addWidget(start_index_id_lineEdit_, 0, 1);
	beizer_base_pro_layout->addWidget(end_index_id_label_, 0, 2);
	beizer_base_pro_layout->addWidget(end_index_id_lineEdit_, 0, 3);
	beizer_base_pro_layout->addWidget(direction_type_label_, 1, 0);
	beizer_base_pro_layout->addWidget(direction_type_combobox_, 1, 1);
	beizer_base_pro_layout->addWidget(widget_label_, 1, 2);
	beizer_base_pro_layout->addWidget(weight_lineEdit_, 1, 3);

	QHBoxLayout* path_passing_checkBox_loayout = new QHBoxLayout;
//  path_passing_checkBox_loayout->addWidget(path_passing_checkBox_);
// 	path_passing_checkBox_loayout->addWidget(m_rotate_checkBox);
// 	path_passing_checkBox_loayout->addStretch();

	//添加三个按钮初始化数据
	m_pDefaultValBtn = new QPushButton(bezier_base_pro_groupBox_);
	m_pDefaultValBtn->setText("默认值");
	//m_pDefaultValBtn->setFixedWidth(40);
	connect(m_pDefaultValBtn, SIGNAL(clicked()), this, SLOT(DefaultValBtnSlot()));
	path_passing_checkBox_loayout->addWidget(m_pDefaultValBtn);

	m_pMedianValBtn = new QPushButton(bezier_base_pro_groupBox_);
	m_pMedianValBtn->setText("中间值");
	//m_pMedianValBtn->setFixedWidth(40);
	connect(m_pMedianValBtn, SIGNAL(clicked()), this, SLOT(MedianValBtnSlot()));
	path_passing_checkBox_loayout->addWidget(m_pMedianValBtn);

	m_pMinValBtn = new QPushButton(bezier_base_pro_groupBox_);
	m_pMinValBtn->setText("最小值");
	//m_pMinValBtn->setFixedWidth(40);
	connect(m_pMinValBtn, SIGNAL(clicked()), this, SLOT(MinValBtnSlot()));
	path_passing_checkBox_loayout->addWidget(m_pMinValBtn);

//	beizer_base_layout->addLayout(beizer_base_pro_layout);
// 	beizer_base_layout->addLayout(path_passing_checkBox_loayout);
// 	beizer_base_layout->addWidget(m_rotate_checkBox);
// 	bezier_base_pro_groupBox_->setLayout(beizer_base_layout);
	

	QLabel* nLabelObstacleRange = new QLabel("提前停障距离(m)");
	LineEditObstacleRange = new QLineEdit;
	LineEditObstacleRange->setText("1.5");
	QHBoxLayout* nLayoutObstacleRange = new QHBoxLayout;
	nLayoutObstacleRange->addWidget(nLabelObstacleRange);
	nLayoutObstacleRange->addWidget(LineEditObstacleRange);

	QHBoxLayout* positive_direction_layout = new QHBoxLayout;
	QGridLayout* positive_param_input_layout = new QGridLayout;
	positive_direction_gropBox_ = new QGroupBox("0 -> 1");
	QLabel *positive_max_speed_label_ = new QLabel("最大速度");
	QLabel *positive_max_acc_speed_label_ = new QLabel("最大加速度");
	QLabel *positive_max_angular_speed_label_ = new QLabel("最大角速度");
	QLabel *positive_max_acc_angular_speed_label_ = new QLabel("最大角加速度");
	QLabel *positive_block_distance_label_ = new QLabel("阻挡距离");
	positive_max_speed_lineEdit_ = new QLineEdit;
	positive_max_acc_speed_lineEdit_ = new QLineEdit;
	positive_max_angular_speed_lineEdit_ = new QLineEdit;
	positive_max_acc_angular_speed_lineEdit_ = new QLineEdit;
	positive_block_distance_lineEdit_ = new QLineEdit;
	QLabel *positive_max_speed_unit_label_ = new QLabel("m/s");
	QLabel *positive_max_acc_speed_unit_label_ = new QLabel("m/s²");
	QLabel *positive_max_angular_speed_unit_label_ = new QLabel("rad/s");
	QLabel *positive_max_acc_angular_speed_unit_label_ = new QLabel("rad/s²");
	QLabel *positive_block_distance_unit_label_ = new QLabel("m");
	positive_param_input_layout->addWidget(positive_max_speed_label_, 0, 0);
	positive_param_input_layout->addWidget(positive_max_speed_lineEdit_, 0, 1);
	positive_param_input_layout->addWidget(positive_max_speed_unit_label_, 0, 2);
	positive_param_input_layout->addWidget(positive_max_acc_speed_label_, 1, 0);
	positive_param_input_layout->addWidget(positive_max_acc_speed_lineEdit_,1, 1);
	positive_param_input_layout->addWidget(positive_max_acc_speed_unit_label_, 1, 2);
	positive_param_input_layout->addWidget(positive_max_angular_speed_label_, 2, 0);
	positive_param_input_layout->addWidget(positive_max_angular_speed_lineEdit_, 2, 1);
	positive_param_input_layout->addWidget(positive_max_angular_speed_unit_label_, 2, 2);
	positive_param_input_layout->addWidget(positive_max_acc_angular_speed_label_, 3, 0);
	positive_param_input_layout->addWidget(positive_max_acc_angular_speed_lineEdit_, 3, 1);
	positive_param_input_layout->addWidget(positive_max_acc_angular_speed_unit_label_, 3, 2);
	positive_param_input_layout->addWidget(positive_block_distance_label_, 4, 0);
	positive_param_input_layout->addWidget(positive_block_distance_lineEdit_, 4, 1);
	positive_param_input_layout->addWidget(positive_block_distance_unit_label_, 4, 2);
	
	positive_direction_layout->setContentsMargins(30, 5, 5, 5);
	positive_direction_layout->addLayout(positive_param_input_layout);
	positive_direction_gropBox_->setLayout(positive_direction_layout);




	QHBoxLayout* nagetive_direction_layout = new QHBoxLayout;
	QGridLayout* nagetive_param_input_layout = new QGridLayout;
	nagetive_direction_gropBox_ = new QGroupBox("1 -> 0");
	QLabel *nagetive_max_speed_label_ = new QLabel("最大速度");
	QLabel *nagetive_max_acc_speed_label_ = new QLabel("最大加速度");
	QLabel *nagetive_max_angular_speed_label_ = new QLabel("最大角速度");
	QLabel *nagetive_max_acc_angular_speed_label_ = new QLabel("最大角加速度");
	QLabel *nagetive_block_distance_label_ = new QLabel("阻挡距离");
	nagetive_max_speed_lineEdit_ = new QLineEdit;
	nagetive_max_acc_speed_lineEdit_ = new QLineEdit;
	nagetive_max_angular_speed_lineEdit_ = new QLineEdit;
	nagetive_max_acc_angular_speed_lineEdit_ = new QLineEdit;
	nagetive_block_distance_lineEdit_ = new QLineEdit;
	QLabel *nagetive_max_speed_unit_label_ = new QLabel("m/s");
	QLabel *nagetive_max_acc_speed_unit_label_ = new QLabel("m/s²");
	QLabel *nagetive_max_angular_speed_unit_label_ = new QLabel("rad/s");
	QLabel *nagetive_max_acc_angular_speed_unit_label_ = new QLabel("rad/s²");
	QLabel *nagetive_block_distance_unit_label_ = new QLabel("m");
	nagetive_param_input_layout->addWidget(nagetive_max_speed_label_, 0, 0);
	nagetive_param_input_layout->addWidget(nagetive_max_speed_lineEdit_, 0, 1);
	nagetive_param_input_layout->addWidget(nagetive_max_speed_unit_label_, 0, 2);
	nagetive_param_input_layout->addWidget(nagetive_max_acc_speed_label_, 1, 0);
	nagetive_param_input_layout->addWidget(nagetive_max_acc_speed_lineEdit_, 1, 1);
	nagetive_param_input_layout->addWidget(nagetive_max_acc_speed_unit_label_, 1, 2);
	nagetive_param_input_layout->addWidget(nagetive_max_angular_speed_label_, 2, 0);
	nagetive_param_input_layout->addWidget(nagetive_max_angular_speed_lineEdit_, 2, 1);
	nagetive_param_input_layout->addWidget(nagetive_max_angular_speed_unit_label_, 2, 2);
	nagetive_param_input_layout->addWidget(nagetive_max_acc_angular_speed_label_, 3, 0);
	nagetive_param_input_layout->addWidget(nagetive_max_acc_angular_speed_lineEdit_, 3, 1);
	nagetive_param_input_layout->addWidget(nagetive_max_acc_angular_speed_unit_label_, 3, 2);
	nagetive_param_input_layout->addWidget(nagetive_block_distance_label_, 4, 0);
	nagetive_param_input_layout->addWidget(nagetive_block_distance_lineEdit_, 4, 1);
	nagetive_param_input_layout->addWidget(nagetive_block_distance_unit_label_, 4, 2);

	nagetive_direction_layout->setContentsMargins(30, 5, 5, 5);
	nagetive_direction_layout->addLayout(nagetive_param_input_layout);
	nagetive_direction_gropBox_->setLayout(nagetive_direction_layout);

	
	QPushButton *pCancelBtn = new QPushButton("取消");
	save_info_buttom_ = new QPushButton("确认");

	QHBoxLayout *pBtnLayout = new QHBoxLayout;
	pBtnLayout->addWidget(pCancelBtn);
	pBtnLayout->addWidget(save_info_buttom_);

	// 	beizer_base_layout->addWidget(m_rotate_checkBox);

	QVBoxLayout *bezier_groupBox_layout = new QVBoxLayout;
	//bezier_groupBox_layout->addWidget(bezier_base_pro_groupBox_);
	bezier_groupBox_layout->addLayout(path_passing_checkBox_loayout); 
	bezier_groupBox_layout->addWidget(path_passing_checkBox_);
	bezier_groupBox_layout->addLayout(pRotateLayout);
	bezier_groupBox_layout->addWidget(m_pFallArrestChx);
	/// 20210616 ADD
	bezier_groupBox_layout->addLayout(nLayoutObstacleRange);
	//bezier_groupBox_layout->addWidget(m_rotate_checkBox);
	bezier_groupBox_layout->addWidget(positive_direction_gropBox_);
	bezier_groupBox_layout->addWidget(nagetive_direction_gropBox_);
	//bezier_groupBox_layout->addWidget(save_info_buttom_);
	bezier_groupBox_layout->addLayout(pBtnLayout);
	bezier_groupBox_->setLayout(bezier_groupBox_layout);

	QHBoxLayout *main_layout = new QHBoxLayout;
	main_layout->addWidget(bezier_groupBox_);
	this->setLayout(main_layout);
	this->setFixedWidth(300);
	setWindowFlags(Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);

	connect(save_info_buttom_, SIGNAL(clicked()), this, SLOT(slot_on_set_bezier_info()));
	connect(pCancelBtn, SIGNAL(clicked()), this, SLOT(close()));
	setWindowModality(Qt::ApplicationModal);				//设置窗口模态

	m_pPositiveCheckBox = new QCheckBox(this);
	m_pNagetiveCheckBox = new QCheckBox(this);
	connect(m_pPositiveCheckBox, SIGNAL(stateChanged(int)), this, SLOT(PositiveStateChangeSlot(int)));
	connect(m_pNagetiveCheckBox, SIGNAL(stateChanged(int)), this, SLOT(NagetiveStateChangeSlot(int)));

	//设置默认勾选
	m_pPositiveCheckBox->setChecked(true);
	m_pNagetiveCheckBox->setChecked(true);
	positive_direction_gropBox_->setEnabled(true);
	nagetive_direction_gropBox_->setEnabled(true);

	DefaultValBtnSlot();
}


BezierPropertyWidget::~BezierPropertyWidget()
{
}




void BezierPropertyWidget::slot_on_set_bezier_info()
{
	if (!m_bIsBatchSet)
	{
//		bool dir_pro;
		
//		if (direction_type_combobox_->currentIndex() == 0) {
//			dir_pro = true;
//		}
// 		else {
// 			dir_pro = false;
// 		}
		//////////判断是否可以通过
// 		if (path_passing_checkBox_->checkState() != Qt::Unchecked) {
// 			parent_->setPassing(true);
// 			pro_passing.value = "true";
// 		}
// 		else {
// 			parent_->setPassing(false);
// 			pro_passing.value = "false";
// 		}
//		parent_->set_dir_pro(dir_pro);
		

		parent_->set_dir_pro(true);

		std::string weight = weight_lineEdit_->text().toLocal8Bit();
		std::string maxspeed = positive_max_speed_lineEdit_->text().toLocal8Bit();
		std::string maxacc = positive_max_acc_speed_lineEdit_->text().toLocal8Bit();
		std::string maxrot = positive_max_angular_speed_lineEdit_->text().toLocal8Bit();
		std::string maxrotacc = positive_max_acc_angular_speed_lineEdit_->text().toLocal8Bit();
		std::string blockdist = positive_block_distance_lineEdit_->text().toLocal8Bit();

		std::string _maxspeed = nagetive_max_speed_lineEdit_->text().toLocal8Bit();
		std::string _maxacc = nagetive_max_acc_speed_lineEdit_->text().toLocal8Bit();
		std::string _maxrot = nagetive_max_angular_speed_lineEdit_->text().toLocal8Bit();
		std::string _maxrotacc = nagetive_max_acc_angular_speed_lineEdit_->text().toLocal8Bit();
		std::string _blockdist = nagetive_block_distance_lineEdit_->text().toLocal8Bit();
		std::string _obstacle_range = LineEditObstacleRange->text().toLocal8Bit();
		Property pro_allowspin("allowspin", "bool");
		if (m_rotate_checkBox->isChecked())
		{
			pro_allowspin.value = "true";
		}
		else
		{
			pro_allowspin.value = "false";
		}

		Property proUltrasonic("ultrasonic", "bool");
		if (m_pUltrasonicChx->isChecked())
		{//超声波
			proUltrasonic.value = "true";
		}
		else
		{
			proUltrasonic.value = "false";
		}

		Property proFallArrest("fallarrest", "bool");
		if (m_pFallArrestChx->isChecked())
		{//防跌落
			proFallArrest.value = "true";
		}
		else
		{
			proFallArrest.value = "false";
		}

		

		std::vector<Property> PropertyVec, nagetive_propertyVec;

		Property pro_widget("weight", "double", weight);
		Property pro_maxspeed("maxspeed", "double", maxspeed);
		Property pro_maxacc("maxacc", "double", maxacc);
		Property pro_maxrot("maxrot", "double", maxrot);
		Property pro_maxrotacc("maxrotacc", "double", maxrotacc);
		Property pro_blockdist("blockdist", "double", blockdist);
		Property pro_passing("passing", "bool");
		Property pro_obstacle_range("obstacle_range", "double", _obstacle_range);

		PropertyVec.push_back(pro_widget);
		PropertyVec.push_back(pro_maxspeed);
		PropertyVec.push_back(pro_maxacc);
		PropertyVec.push_back(pro_maxrot);
		PropertyVec.push_back(pro_maxrotacc);
		PropertyVec.push_back(pro_blockdist);
		if (m_pPositiveCheckBox->isChecked())
		{
			pro_passing.value = "true";
			PropertyVec.push_back(pro_passing);
		}
		else
		{
			pro_passing.value = "false";
			PropertyVec.push_back(pro_passing);
		}
		PropertyVec.push_back(pro_obstacle_range);
		PropertyVec.push_back(pro_allowspin);
		PropertyVec.push_back(proUltrasonic);
		PropertyVec.push_back(proFallArrest);

		parent_->setPropertyList(PropertyVec);

		Property _pro_weighgt("weight", "double", weight);
		Property _pro_maxspeed("maxspeed", "double", _maxspeed);
		Property _pro_maxacc("maxacc", "double", _maxacc);
		Property _pro_maxrot("maxrot", "double", _maxrot);
		Property _pro_maxrotacc("maxrotacc", "double", _maxrotacc);
		Property _pro_blockdist("blockdist", "double", _blockdist);
		Property _pro_passing("passing", "bool");
		nagetive_propertyVec.push_back(_pro_weighgt);
		nagetive_propertyVec.push_back(_pro_maxspeed);
		nagetive_propertyVec.push_back(_pro_maxacc);
		nagetive_propertyVec.push_back(_pro_maxrot);
		nagetive_propertyVec.push_back(_pro_maxrotacc);
		nagetive_propertyVec.push_back(_pro_blockdist);
		if (m_pNagetiveCheckBox->isChecked())
		{
			//parent_->setPassing(true);
			_pro_passing.value = "true";
			nagetive_propertyVec.push_back(_pro_passing);
		}
		else
		{
			//parent_->setPassing(false);
			_pro_passing.value = "false";
			nagetive_propertyVec.push_back(_pro_passing);
		}
		nagetive_propertyVec.push_back(pro_allowspin);
		nagetive_propertyVec.push_back(proUltrasonic);
		nagetive_propertyVec.push_back(proFallArrest);
		parent_->setNagetivePropertyList(nagetive_propertyVec);
		parent_->update();
	}
	else                          //批量处理
	{
// 		bool dir_pro;
// 		if (direction_type_combobox_->currentIndex() == 0) {
// 			dir_pro = true;
// 		}
// 		else {
// 			dir_pro = false;
// 		}
		Property pro_passing("passing", "bool");
		if (path_passing_checkBox_->checkState() != Qt::Unchecked) {
			pro_passing.value = "true";
		}
		else {
			pro_passing.value = "false";
		}

		Property pro_allowspin("allowspin", "bool");
		if (m_rotate_checkBox->isChecked())
		{
			pro_allowspin.value = "true";
		}
		else
		{
			pro_allowspin.value = "false";
		}

		Property proUltrasonic("ultrasonic", "bool");
		if (m_pUltrasonicChx->isChecked())
		{//超声波
			proUltrasonic.value = "true";
		}
		else
		{
			proUltrasonic.value = "false";
		}

		Property proFallArrest("fallarrest", "bool");
		if (m_pFallArrestChx->isChecked())
		{//防跌落
			proFallArrest.value = "true";
		}
		else
		{
			proFallArrest.value = "false";
		}

		std::string maxspeed = positive_max_speed_lineEdit_->text().toLocal8Bit();
		std::string maxacc = positive_max_acc_speed_lineEdit_->text().toLocal8Bit();
		std::string maxrot = positive_max_angular_speed_lineEdit_->text().toLocal8Bit();
		std::string maxrotacc = positive_max_acc_angular_speed_lineEdit_->text().toLocal8Bit();

		foreach(DLBezierItem *pParentItem, m_lstParentItems)
		{
			if (pParentItem)
			{
				std::vector<Property> PropertyVev = pParentItem->getPropertyList();
				std::vector<Property> nagetive_propertyVev = pParentItem->getNagetivePropertyList();

				QString blockdist = positive_block_distance_lineEdit_->text();

				if (PropertyVev.size() > DLBezierItem::FALL_ARREST)
				{
					PropertyVev[DLBezierItem::MAXSPEED_PRO].value = maxspeed;
					PropertyVev[DLBezierItem::MAXACC_PRO].value = maxacc;
					PropertyVev[DLBezierItem::MAXROT_PRO].value = maxrot;
					PropertyVev[DLBezierItem::MAXROTACC_PRO].value = maxrotacc;
					if (!blockdist.isEmpty())
					{
						PropertyVev[DLBezierItem::BLOCKDIST_PRO].value = blockdist.toLocal8Bit();
					}
					PropertyVev[DLBezierItem::PASSING_PRO].value = pro_passing.value;
					PropertyVev[DLBezierItem::ALLOWSPIN_PRO].value = pro_allowspin.value;
					PropertyVev[DLBezierItem::ULTRASONIC].value = proUltrasonic.value;
					PropertyVev[DLBezierItem::FALL_ARREST].value = proFallArrest.value;
				}
				if (nagetive_propertyVev.size() > DLBezierItem::FALL_ARREST)
				{

					nagetive_propertyVev[DLBezierItem::MAXSPEED_PRO].value = maxspeed;
					nagetive_propertyVev[DLBezierItem::MAXACC_PRO].value = maxacc;
					nagetive_propertyVev[DLBezierItem::MAXROT_PRO].value = maxrot;
					nagetive_propertyVev[DLBezierItem::MAXROTACC_PRO].value = maxrotacc;
					if (!blockdist.isEmpty())
					{
						nagetive_propertyVev[DLBezierItem::BLOCKDIST_PRO].value = blockdist.toLocal8Bit();
					}
					nagetive_propertyVev[DLBezierItem::PASSING_PRO].value = pro_passing.value;
					nagetive_propertyVev[DLBezierItem::ALLOWSPIN_PRO].value = pro_allowspin.value;
					nagetive_propertyVev[DLBezierItem::ULTRASONIC].value = proUltrasonic.value;		//超声波
					nagetive_propertyVev[DLBezierItem::FALL_ARREST].value = proFallArrest.value;	//防跌落

// 					nagetive_propertyVev[1].value = maxspeed;
// 					nagetive_propertyVev[2].value = maxacc;
// 					nagetive_propertyVev[3].value = maxrot;
// 					nagetive_propertyVev[4].value = maxrotacc;
// 					if (!blockdist.isEmpty())
// 					{
// 						nagetive_propertyVev[5].value = blockdist.toLocal8Bit();
// 					}
// 					nagetive_propertyVev[6].value = pro_passing.value;
// 					nagetive_propertyVev[7].value = pro_allowspin.value;
				}

// 				if (pro_passing.value.c_str() == "false")
// 				{
// 					//pParentItem->setPassing(false);
// 				}
// 				else
// 				{
// 					pParentItem->setPassing(true);
// 				}
				//pParentItem->set_dir_pro(dir_pro);
				pParentItem->set_dir_pro(true);
				pParentItem->setPropertyList(PropertyVev);
				pParentItem->setNagetivePropertyList(nagetive_propertyVev);
				pParentItem->update();
			}
		}

	}
	this->hide();
}


void BezierPropertyWidget::DefaultValBtnSlot()
{
	positive_max_speed_lineEdit_->setText(DEFAULT_VAL);
	positive_max_acc_speed_lineEdit_->setText(DEFAULT_VAL);
	positive_max_angular_speed_lineEdit_->setText(DEFAULT_VAL);
	positive_max_acc_angular_speed_lineEdit_->setText(DEFAULT_VAL);

	nagetive_max_speed_lineEdit_->setText(DEFAULT_VAL);
	nagetive_max_acc_speed_lineEdit_->setText(DEFAULT_VAL);
	nagetive_max_angular_speed_lineEdit_->setText(DEFAULT_VAL);
	nagetive_max_acc_angular_speed_lineEdit_->setText(DEFAULT_VAL);

}

void BezierPropertyWidget::MedianValBtnSlot()
{
	positive_max_speed_lineEdit_->setText(MEDIAN_VAL);
	positive_max_acc_speed_lineEdit_->setText(MEDIAN_VAL);
	positive_max_angular_speed_lineEdit_->setText(MEDIAN_VAL);
	positive_max_acc_angular_speed_lineEdit_->setText(MEDIAN_VAL);

	nagetive_max_speed_lineEdit_->setText(MEDIAN_VAL);
	nagetive_max_acc_speed_lineEdit_->setText(MEDIAN_VAL);
	nagetive_max_angular_speed_lineEdit_->setText(MEDIAN_VAL);
	nagetive_max_acc_angular_speed_lineEdit_->setText(MEDIAN_VAL);
}

void BezierPropertyWidget::MinValBtnSlot()
{
	positive_max_speed_lineEdit_->setText(MIN_VAL);
	positive_max_acc_speed_lineEdit_->setText(MIN_VAL);
	positive_max_angular_speed_lineEdit_->setText(MIN_VAL);
	positive_max_acc_angular_speed_lineEdit_->setText(MIN_VAL);

	nagetive_max_speed_lineEdit_->setText(MIN_VAL);
	nagetive_max_acc_speed_lineEdit_->setText(MIN_VAL);
	nagetive_max_angular_speed_lineEdit_->setText(MIN_VAL);
	nagetive_max_acc_angular_speed_lineEdit_->setText(MIN_VAL);
}

void BezierPropertyWidget::PositiveStateChangeSlot(int iState)
{
	if (iState == Qt::Checked)
	{
		positive_direction_gropBox_->setEnabled(true);
	}
	else
	{
		positive_direction_gropBox_->setEnabled(false);
	}
}

void BezierPropertyWidget::NagetiveStateChangeSlot(int iState)
{
	if (iState == Qt::Checked)
	{
		nagetive_direction_gropBox_->setEnabled(true);
	}
	else
	{
		nagetive_direction_gropBox_->setEnabled(false);
	}
}

void BezierPropertyWidget::ClearContent()
{
	start_index_id_lineEdit_->clear();
	end_index_id_lineEdit_->clear();

	direction_type_combobox_->setCurrentIndex(0);
	weight_lineEdit_->clear();
	path_passing_checkBox_->setChecked(true);

	positive_max_speed_lineEdit_->clear();
	positive_max_acc_speed_lineEdit_->clear();
	positive_max_angular_speed_lineEdit_->clear();
	positive_max_acc_angular_speed_lineEdit_->clear();

	nagetive_max_speed_lineEdit_->clear();
	nagetive_max_acc_speed_lineEdit_->clear();
	nagetive_max_angular_speed_lineEdit_->clear();
	nagetive_max_acc_angular_speed_lineEdit_->clear();

	positive_block_distance_lineEdit_->clear();
	nagetive_block_distance_lineEdit_->clear();
}

void BezierPropertyWidget::keyPressEvent(QKeyEvent *event)
{
	if (Qt::Key_Return == event->key())
	{
		slot_on_set_bezier_info();
		this->hide();
	}
}


void BezierPropertyWidget::setBezierProperty()
{
	int start_id = parent_->start_id();
	int end_id = parent_->end_id();
//	bool dir_pro = parent_->dir_pro();
	bool dir_pro = true;
//	bool isPassing = parent_->isPassing();
	double weight = parent_->path_weight();
	std::vector<Property> PropertyVev = parent_->getPropertyList();
	std::vector<Property> nagetive_propertyVev = parent_->getNagetivePropertyList();
	weight_lineEdit_->setText(QString::number(weight));
// 	if (isPassing) {
// 		path_passing_checkBox_->setCheckState(Qt::Checked);
// 	}
// 	else {
// 		path_passing_checkBox_->setCheckState(Qt::Unchecked);
// 	}
	start_index_id_lineEdit_->setText(QString::number(start_id));
	end_index_id_lineEdit_->setText(QString::number(end_id));

	if (dir_pro) {
	//	direction_type_combobox_->setCurrentIndex(0);
		nagetive_direction_gropBox_->show();
		positive_direction_gropBox_->setTitle(QString::number(start_id) +" -> " + QString::number(end_id));
		nagetive_direction_gropBox_->setTitle(QString::number(end_id) + " -> " + QString::number(start_id));
		if (PropertyVev.size() != 0) {
			positive_max_speed_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[1].value.c_str()));
			positive_max_acc_speed_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[2].value.c_str()));
			positive_max_angular_speed_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[3].value.c_str()));
			positive_max_acc_angular_speed_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[4].value.c_str()));
			positive_block_distance_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[5].value.c_str()));

			if (PropertyVev[6].value == "true") {
				//path_passing_checkBox_->setCheckState(Qt::Checked);
				m_pPositiveCheckBox->setChecked(true);
			}
			else {
				//path_passing_checkBox_->setCheckState(Qt::Unchecked);
				m_pPositiveCheckBox->setChecked(false);
			}

			if (PropertyVev[7].value == "true") {
				m_rotate_checkBox->setChecked(true);
			}
			else {
				m_rotate_checkBox->setChecked(false);
			}

			if (PropertyVev[DLBezierItem::ULTRASONIC].value == "true") {
				m_pUltrasonicChx->setChecked(true);
			}
			else {
				m_pUltrasonicChx->setChecked(false);
			}

			if (PropertyVev[DLBezierItem::FALL_ARREST].value == "true") {
				m_pFallArrestChx->setChecked(true);
			}
			else {
				m_pFallArrestChx->setChecked(false);
			}
		}
		if (nagetive_propertyVev.size() != 0) {
			nagetive_max_speed_lineEdit_->setText(QString::fromLocal8Bit(nagetive_propertyVev[1].value.c_str()));
			nagetive_max_acc_speed_lineEdit_->setText(QString::fromLocal8Bit(nagetive_propertyVev[2].value.c_str()));
			nagetive_max_angular_speed_lineEdit_->setText(QString::fromLocal8Bit(nagetive_propertyVev[3].value.c_str()));
			nagetive_max_acc_angular_speed_lineEdit_->setText(QString::fromLocal8Bit(nagetive_propertyVev[4].value.c_str()));
			nagetive_block_distance_lineEdit_->setText(QString::fromLocal8Bit(nagetive_propertyVev[5].value.c_str()));
			if (nagetive_propertyVev[6].value == "true") {
				m_pNagetiveCheckBox->setChecked(true);
			}
			else {
				m_pNagetiveCheckBox->setChecked(false);
			}

			if (PropertyVev[DLBezierItem::ULTRASONIC].value == "true") {
				m_pUltrasonicChx->setChecked(true);
			}
			else {
				m_pUltrasonicChx->setChecked(false);
			}

			if (PropertyVev[DLBezierItem::FALL_ARREST].value == "true") {
				m_pFallArrestChx->setChecked(true);
			}
			else {
				m_pFallArrestChx->setChecked(false);
			}
		}
	}
	else {
		direction_type_combobox_->setCurrentIndex(1);
		nagetive_direction_gropBox_->hide();
		positive_direction_gropBox_->setTitle(QString::number(start_id) + " -> " + QString::number(end_id));
		if (PropertyVev.size() > 6) {
			positive_max_speed_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[1].value.c_str()));
			positive_max_acc_speed_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[2].value.c_str()));
			positive_max_angular_speed_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[3].value.c_str()));
			positive_max_acc_angular_speed_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[4].value.c_str()));
			positive_block_distance_lineEdit_->setText(QString::fromLocal8Bit(PropertyVev[5].value.c_str()));
		}
	}

}



void BezierPropertyWidget::setParent(DLBezierItem *item)
{
	parent_ = item;
}




void BezierPropertyWidget::SetBatchSet(bool bIsBatchSet)
{
	m_bIsBatchSet = bIsBatchSet;
}

void BezierPropertyWidget::SetBatchSetParents(QList<DLBezierItem *> lstParentItems)
{
	m_lstParentItems = lstParentItems;
}

void BezierPropertyWidget::InitBatchProperty()
{
	ClearContent();
	DefaultValBtnSlot();
}

void BezierPropertyWidget::showEvent(QShowEvent *event)
{
	if (m_bIsBatchSet)
	{
		weight_lineEdit_->setEnabled(false);
		start_index_id_lineEdit_->setEnabled(false);
		end_index_id_lineEdit_->setEnabled(false);
		nagetive_direction_gropBox_->hide();
		m_pPositiveCheckBox->hide();
		m_pNagetiveCheckBox->hide();
		positive_direction_gropBox_->setTitle("");
		path_passing_checkBox_->setVisible(true);
		this->setFixedHeight(300);
	}
	else
	{
		m_pPositiveCheckBox->show();
		m_pNagetiveCheckBox->show();
		weight_lineEdit_->setEnabled(true);
		start_index_id_lineEdit_->setEnabled(true);
		end_index_id_lineEdit_->setEnabled(true);
		nagetive_direction_gropBox_->show();
		path_passing_checkBox_->setVisible(false);
		this->setFixedHeight(483);
	}

	if (NULL != positive_direction_gropBox_ && NULL != m_pPositiveCheckBox)
	{
		m_pPositiveCheckBox->move(positive_direction_gropBox_->pos() + QPoint(5, 2));
	}
	if (NULL != nagetive_direction_gropBox_ && NULL != m_pNagetiveCheckBox)
	{
		m_pNagetiveCheckBox->move(nagetive_direction_gropBox_->pos() + QPoint(5, 2));
	}
}


void BezierPropertyWidget::mousePressEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		m_bIsPressed = true;
		m_ptStartPos = QCursor::pos();
	}
}

void BezierPropertyWidget::mouseReleaseEvent(QMouseEvent *event)
{
	if (event->buttons() & Qt::LeftButton && m_bIsPressed)
	{
		m_bIsPressed = false;
	}
}

void BezierPropertyWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (event->buttons() & Qt::LeftButton && m_bIsPressed)
	{
		QPoint ptEndPos = QCursor::pos();
		this->move(this->pos() + (ptEndPos - m_ptStartPos));
		m_ptStartPos = ptEndPos;
	}
}

/////////////////////////////////////////////////////////////




DLBezierItem::DLBezierItem(DLShapeItem *start,DLShapeItem *end,QPointF c1,QPointF c2,int id,std::string className, std::string instanceName)
	:id_(id),className_(className),instanceName_(instanceName)
{
	create_item(start,end,c1,c2,id,className);
	
}

DLBezierItem::DLBezierItem(DLShapeItem *start,DLShapeItem *end,QPointF c1,QPointF c2,int id,std::string className)
:id_(id),className_(className)
{
	create_item(start,end,c1,c2,id,className);

}


DLBezierItem::DLBezierItem(DLShapeItem *start,DLShapeItem *end,int id,std::string name):id_(id),className_(name)
{
	Bezier pro;	
	pro.start_.x_=start->pos().x();
	pro.start_.y_=start->pos().y();
	pro.end_.x_=end->pos().x();
	pro.end_.y_=end->pos().y();

	double dx=pro.end_.x_-pro.start_.x_;
	double dy=pro.end_.y_-pro.start_.y_;

	double c1_x=pro.start_.x_+1.0*dx/3.0;
	double c1_y=pro.start_.y_+1.0*dy/3.0;
	double c2_x=pro.start_.x_+2.0*dx/3.0;
	double c2_y=pro.start_.y_+2.0*dy/3.0;


	QPointF c1(c1_x,c1_y);
	QPointF c2(c2_x,c2_y);
	create_item(start,end,c1,c2,id,name);


}


void DLBezierItem::create_item(DLShapeItem *start,DLShapeItem *end,QPointF c1,QPointF c2,int id,std::string name)
{
	m_bIsPassing = true;
	m_bIsShowAuxLine = false;
	double_dir_ = true;
	childe_selectable_ = true;
	className_ = "BezierPath";
	start_landmark_=start;
	end_landmark_=end;
	start_id_=start->id();
	end_id_=end->id();

	((DLLandmarkItem*)start_landmark_)->add_edge(this);
	((DLLandmarkItem*)end_landmark_)->add_edge(this);

	Bezier pro;	
	pro.start_.x_=start->pos().x();
	pro.start_.y_=start->pos().y();
	pro.end_.x_=end->pos().x();
	pro.end_.y_=end->pos().y();


	pro.c1_.x_=c1.x();
	pro.c1_.y_=c1.y();
	pro.c2_.x_=c2.x();
	pro.c2_.y_=c2.y();


	////
	set_solid(false);

	start_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
	start_item_->setParentItem(this);
	start_item_->setVisible(false);

				 
	c1_item_ = new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE, this);
	c1_item_->setParentItem(this);
	c1_item_->setVisible(false);


	c2_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE, this);
	c2_item_->setParentItem(this);
	c2_item_->setVisible(false);

	end_item_=new DLControlTransformItem(DLControlTransformItem::ALIGN_FREE);
	end_item_->setParentItem(this);
	end_item_->setVisible(false);

	setFlag(QGraphicsItem::ItemIsSelectable, true);
	setFlag(QGraphicsItem::ItemIsMovable, false);
	setFlag(QGraphicsItem::ItemSendsGeometryChanges);
	setAcceptedMouseButtons(Qt::LeftButton);

	
	set_bezier_pro(pro);
	set_control_pro(pro);

	double weight = sqrt( (pro.start_.x_-pro.end_.x_)*(pro.start_.x_ - pro.end_.x_) + (pro.start_.y_ - pro.end_.y_)*(pro.start_.y_ - pro.end_.y_) );



	//去的属性列表
	Property pro_widget("weight", "double", std::to_string(weight));
	Property pro_maxspeed("maxspeed", "double", "0.8");
	Property pro_maxacc("maxacc", "double", "0.8");
	Property pro_maxrot("maxrot", "double", "0.8");
	Property pro_maxrotacc("maxrotacc", "double", "0.8");
	Property pro_blockdist("blockdist", "double", "0.7");
	Property pro_passing("passing", "bool", "true");
	Property pro_allowspin("allowspin", "bool", "true");
	Property proUltrasonic("ultrasonic", "bool", "true");  //超声波
	Property proFallArrest("fallarrest", "bool", "true");  //防跌落
	Property proAdvanceArea("advancearea", "bool", "false");

	m_vGoPropertys.push_back(pro_widget);
	m_vGoPropertys.push_back(pro_maxspeed);
	m_vGoPropertys.push_back(pro_maxacc);
	m_vGoPropertys.push_back(pro_maxrot);
	m_vGoPropertys.push_back(pro_maxrotacc);
	m_vGoPropertys.push_back(pro_blockdist);
	m_vGoPropertys.push_back(pro_passing);
	m_vGoPropertys.push_back(pro_allowspin);
	m_vGoPropertys.push_back(proUltrasonic);
	m_vGoPropertys.push_back(proFallArrest);
	m_vGoPropertys.push_back(proAdvanceArea);

	////////////////////////////////////////

	//回的属性列表
	Property _pro_weighgt = pro_widget;
	Property _pro_maxspeed("maxspeed", "double", "0.8");
	Property _pro_maxacc("maxacc", "double", "0.8");
	Property _pro_maxrot("maxrot", "double", "0.8");
	Property _pro_maxrotacc("maxrotacc", "double", "0.8");
	Property _pro_blockdist("blockdist", "double", "0.7");
	Property _pro_passing = pro_passing;
	Property _pro_allowspin = pro_allowspin;

	m_vBackPropertys.push_back(_pro_weighgt);
	m_vBackPropertys.push_back(_pro_maxspeed);
	m_vBackPropertys.push_back(_pro_maxacc);
	m_vBackPropertys.push_back(_pro_maxrot);
	m_vBackPropertys.push_back(_pro_maxrotacc);
	m_vBackPropertys.push_back(_pro_blockdist);
	m_vBackPropertys.push_back(_pro_passing);
	m_vBackPropertys.push_back(_pro_allowspin);
	m_vBackPropertys.push_back(proUltrasonic);
	m_vBackPropertys.push_back(proFallArrest);
	m_vBackPropertys.push_back(proAdvanceArea);
}


DLBezierItem::~DLBezierItem()
{


	if(c1_item_!=NULL)
	{
		delete c1_item_;
		c1_item_=NULL;
	}

	if(c2_item_!=NULL)
	{
		delete c2_item_;
		c2_item_=NULL;
	}	



}


QRectF DLBezierItem::boundingRect() const
{

	//QPainterPath path=shape();
	//return path.boundingRect();
	QLineF line_1, line_2;
	line_1.setP1(QPointF(pro_.start_.x_, pro_.start_.y_));
	line_1.setP2(QPointF(pro_.c1_.x_, pro_.c1_.y_));
	QLineF temp_line_1 = line_1;
	temp_line_1.setAngle(line_1.angle() + 180);

	line_2.setP1(QPointF(pro_.end_.x_, pro_.end_.y_));
	line_2.setP2(QPointF(pro_.c2_.x_, pro_.c2_.y_));
	QLineF temp_line_2 = line_2;
	temp_line_2.setAngle(line_2.angle() + 180);

	QPolygonF polygon;
	polygon.append(temp_line_1.p2());
	polygon.append(QPointF(pro_.c1_.x_, pro_.c1_.y_));
	polygon.append(QPointF(pro_.c2_.x_, pro_.c2_.y_));
	polygon.append(temp_line_2.p2());

	QRectF rect = polygon.boundingRect();
	if (abs(rect.topLeft().y() - rect.bottomRight().y()) < 5) {
		rect.setTopLeft( QPointF(rect.topLeft().x(), rect.topLeft().y()-5) );
		rect.setBottomRight(QPointF(rect.bottomRight().x(), rect.bottomRight().y()+5));
	}
	else if (abs(rect.topLeft().x() - rect.bottomRight().x()) < 5) {
		rect.setTopLeft( QPointF(rect.topLeft().x() - 5, rect.topLeft().y()) );
		rect.setBottomRight( QPointF(rect.bottomRight().x() + 5, rect.bottomRight().y()) );
	}
	return rect;
}

QPainterPath DLBezierItem::shape() const
{

	QPainterPath path;
	path.moveTo(QPointF(pro_.start_.x_,pro_.start_.y_));
	//path.cubicTo(QPointF(pro_.c1_.x_,pro_.c1_.y_),QPointF(pro_.c2_.x_,pro_.c2_.y_),QPointF(pro_.end_.x_,pro_.end_.y_));
	path.lineTo(QPointF(pro_.end_.x_, pro_.end_.y_));
	
	if(is_solid_==true)
	{
		return path;
	}
	else
	{
		QPainterPathStroker path_stroker;
		path_stroker.setWidth(15);
		return path_stroker.createStroke(path);
	}

}

LinePassingState DLBezierItem::getLinePassingState()
{
	LinePassingState state;
	if (m_vGoPropertys.size() != 0)
	{
		if (m_vGoPropertys[6].value == "true")
		{
			if (m_vBackPropertys.size() != 0)
			{
				if (m_vBackPropertys[6].value == "true")
				{
					return AllowPassing;
				}
				else
				{
					return StartToStopPassing;
				}
			}
			else 
			{
				return Unnormal;
			}
		}
		else
		{
			if (m_vBackPropertys.size() != 0)
			{
				if (m_vBackPropertys[6].value == "true")
				{
					return StopToStartPassing;
				}
				else
				{
					return ForbidPassing;
				}
			}
			else
			{
				return Unnormal;
			}
		}
	}
	else
	{
		return Unnormal;
	}
	return Unnormal;
}

void DLBezierItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	QPen pen = get_pen();
	QBrush brush=get_brush();
	if(isSelected()){
		pen.setWidth(pen.width()+1);
	}

	painter->save();
	QPainterPath path;						//绘制贝塞尔曲线
	path.moveTo(QPointF(pro_.start_.x_, pro_.start_.y_));
	//path.cubicTo(QPointF(pro_.c1_.x_, pro_.c1_.y_), QPointF(pro_.c2_.x_, pro_.c2_.y_), QPointF(pro_.end_.x_, pro_.end_.y_));
	path.lineTo(QPointF(pro_.end_.x_, pro_.end_.y_));

	if (!m_bIsAdvanceArea)
	{//不是高级区域 =  通行
// 		pen.setColor(QColor(0, 0, 0));
// 		painter->setPen(pen);
// 
// 		QPainterPathStroker path_stroker;
// 		path_stroker.setWidth(1);
// 		painter->drawPath(path_stroker.createStroke(path));
	

		switch (getLinePassingState())
		{
		case AllowPassing:
		{
			pen.setColor(QColor(0, 0, 0));
			painter->setPen(pen);

			QPainterPathStroker path_stroker;
			path_stroker.setWidth(1);
			painter->drawPath(path_stroker.createStroke(path));
			break;
		}
		case StartToStopPassing:
		{
			pen.setColor(QColor(138, 43, 226));
			painter->setPen(pen);

			QPointF point = path.pointAtPercent(0.5);
			QLineF line(start_pos(), end_pos());
			double dAngle = ::acos(line.dx() / line.length());
			if (line.dy() >= 0) { dAngle = PI * 2 - dAngle; }
			QPointF arrowPoint1 = point + QPointF(sin(dAngle - PI / 3) * 15, cos(dAngle - PI / 3) * 15);
			QPointF arrowPoint2 = point + QPointF(sin(dAngle - PI + PI / 3) * 15, cos(dAngle - PI + PI / 3) * 15);
			painter->drawLine(QLineF(point, arrowPoint1));
			painter->drawLine(QLineF(point, arrowPoint2));

			QPainterPathStroker path_stroker;
			path_stroker.setWidth(1);
			painter->drawPath(path_stroker.createStroke(path));
			break;
		}
		case StopToStartPassing:
		{
			pen.setColor(QColor(138, 43, 226));
			painter->setPen(pen);

			QPointF point = path.pointAtPercent(0.5);
			QLineF line(start_pos(), end_pos());
			double dAngle = ::acos(line.dx() / line.length());
			if (line.dy() >= 0) { dAngle = PI * 2 - dAngle; }
			QPointF arrowPoint1 = point - QPointF(sin(dAngle - PI / 3) * 15, cos(dAngle - PI / 3) * 15);
			QPointF arrowPoint2 = point - QPointF(sin(dAngle - PI + PI / 3) * 15, cos(dAngle - PI + PI / 3) * 15);
			painter->drawLine(QLineF(point, arrowPoint1));
			painter->drawLine(QLineF(point, arrowPoint2));

			QPainterPathStroker path_stroker;
			path_stroker.setWidth(1);
			painter->drawPath(path_stroker.createStroke(path));
			break;
		}
		case ForbidPassing:
		{
			pen.setColor(QColor(255, 0, 0));
			painter->setPen(pen);
			QPointF point = path.pointAtPercent(0.5);
			painter->drawLine(QLineF(point + QPointF(-5, 5), point + QPointF(5, -5)));
			painter->drawLine(QLineF(point + QPointF(-5, -5), point + QPointF(5, 5)));
			QPainterPathStroker path_stroker;
			path_stroker.setWidth(1);
			painter->drawPath(path_stroker.createStroke(path));
			break;
		}
		case Unnormal:
			return;
			break;
		default:
			return;
			break;
		}
	}
	else
	{//是高级区域 = 不通行
// 		pen.setColor(QColor(255, 0, 0));
// 		painter->setPen(pen);
// 		QPointF point = path.pointAtPercent(0.5);
// 		painter->drawLine(QLineF(point + QPointF(-5, 5), point + QPointF(5, -5)));
// 		painter->drawLine(QLineF(point + QPointF(-5, -5), point + QPointF(5, 5)));
// 		QPainterPathStroker path_stroker;
// 		path_stroker.setWidth(1);
// 		painter->drawPath(path_stroker.createStroke(path));
		pen.setColor(QColor(255, 0, 0));
		painter->setPen(pen);
		QPointF point = path.pointAtPercent(0.5);
		painter->drawLine(QLineF(point + QPointF(-5, 5), point + QPointF(5, -5)));
		painter->drawLine(QLineF(point + QPointF(-5, -5), point + QPointF(5, 5)));
		QPainterPathStroker path_stroker;
		path_stroker.setWidth(1);
		painter->drawPath(path_stroker.createStroke(path));
	}
	

	if (option->state & QStyle::State_Selected)
	{//如果是选中线
		painter->save();
		QPen pen;
		pen.setColor(Qt::green);
		pen.setStyle(Qt::DotLine);
		painter->setPen(pen);
		QPainterPathStroker path_stroker;
		path_stroker.setWidth(3);
		painter->drawPath(path_stroker.createStroke(path));
		painter->restore();
	}
	painter->restore();

	if (m_bIsShowAuxLine)
	{
		painter->save();
		painter->setPen(QPen(Qt::black, 1, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin));
		QLineF line_1, line_2;
		line_1.setP1(QPointF(pro_.start_.x_, pro_.start_.y_));
		line_1.setP2(QPointF(pro_.c1_.x_, pro_.c1_.y_));
		QLineF temp_line = line_1;
		temp_line.setAngle(line_1.angle()+180);
		line_1.setP1(QPointF(temp_line.p2()));
		line_1.setP2(QPointF(pro_.c1_.x_, pro_.c1_.y_));

		line_2.setP1(QPointF(pro_.end_.x_, pro_.end_.y_));
		line_2.setP2(QPointF(pro_.c2_.x_, pro_.c2_.y_));
		temp_line = line_2;
		temp_line.setAngle(line_2.angle() + 180);
		line_2.setP1(QPointF(temp_line.p2()));
		line_2.setP2(QPointF(pro_.c2_.x_, pro_.c2_.y_));

		painter->drawLine(line_1);
		painter->drawLine(line_2);
		painter->restore();
	}
}

void DLBezierItem::slot_pro( Bezier pro )
{
	set_bezier_pro(pro);
	set_control_pro(pro);
}

void DLBezierItem::set_bezier_pro( Bezier pro )
{
	pro_=pro;

	//通知形状更新
	prepareGeometryChange();

}

void DLBezierItem::set_control_pro(Bezier pro)
{
	start_item_->setPos(pro_.start_.x_,pro_.start_.y_);
	c1_item_->setPos(pro_.c1_.x_,pro_.c1_.y_);
	c2_item_->setPos(pro_.c2_.x_,pro_.c2_.y_);
	end_item_->setPos(pro_.end_.x_,pro_.end_.y_);

	start_landmark_->setPos(start_item_->scenePos());
	end_landmark_->setPos(end_item_->scenePos());

}


void DLBezierItem::setPassing(bool isPassing)
{
	if (m_vBackPropertys.size() >= 7 && m_vGoPropertys.size() >= 7) {
		if (isPassing)
		{//可以通行
			m_vBackPropertys[6].value = "true";
			m_vGoPropertys[6].value = "true";
		}
		else
		{//不能通行
			m_vBackPropertys[6].value = "false";
			m_vGoPropertys[6].value = "false";
		}
	}

	update();
}



void DLBezierItem::SetIsAdvanceArea(bool bIsAdvanceArea /*= false*/)
{//更新是否是高级区域属性
	m_bIsAdvanceArea = bIsAdvanceArea;
	if (m_vGoPropertys.size() > ADVANCE_ARE_PRO)
	{
		if (bIsAdvanceArea)
		{
			m_vGoPropertys[ADVANCE_ARE_PRO].value = "true";
		}
		else
		{
			m_vGoPropertys[ADVANCE_ARE_PRO].value = "false";
		}
	}

	if (m_vBackPropertys.size() > ADVANCE_ARE_PRO)
	{
		if (bIsAdvanceArea)
		{
			m_vBackPropertys[ADVANCE_ARE_PRO].value = "true";
		}
		else
		{
			m_vBackPropertys[ADVANCE_ARE_PRO].value = "false";
		}
	}
}

double DLBezierItem::path_weight()
{
	return sqrt((pro_.start_.x_ - pro_.end_.x_)*(pro_.start_.x_ - pro_.end_.x_) + (pro_.start_.y_ - pro_.end_.y_)*(pro_.start_.y_ - pro_.end_.y_));
}


void DLBezierItem::set_child_selectable(bool state)
{
	childe_selectable_ = state;
}


void DLBezierItem::update_shape()
{
	
	Bezier pro=cal_pro();
	set_bezier_pro(pro);
	set_control_pro(pro);

	double weight = sqrt((pro.start_.x_ - pro.end_.x_)*(pro.start_.x_ - pro.end_.x_) + (pro.start_.y_ - pro.end_.y_)*(pro.start_.y_ - pro.end_.y_));
	Property pro_widget("weight", "double", std::to_string(weight));
	m_vGoPropertys[0] = pro_widget;

	emit sig_pro(pro_);

}

void DLBezierItem::update_bezier()
{
	Bezier pro;	
	pro.start_.x_=this->mapFromItem(start_landmark_,QPointF(0,0)).x();
	pro.start_.y_=this->mapFromItem(start_landmark_,QPointF(0,0)).y();
	pro.c1_.x_=c1_item_->pos().x();
	pro.c1_.y_=c1_item_->pos().y();
	pro.c2_.x_=c2_item_->pos().x();
	pro.c2_.y_=c2_item_->pos().y();
	pro.end_.x_=this->mapFromItem(end_landmark_,QPointF(0,0)).x();
	pro.end_.y_=this->mapFromItem(end_landmark_,QPointF(0,0)).y();

	set_bezier_pro(pro);
	set_control_pro(pro);
}

void DLBezierItem::set_control_point_visible(bool visible)
{
	
	start_item_->setVisible(visible);
	c1_item_->setVisible(visible);
	c2_item_->setVisible(visible);
	end_item_->setVisible(visible);
	start_item_->update();
	c1_item_->update();
	c2_item_->update();
	end_item_->update();
// 	if (!visible)
// 	{
// 		showAuxLine_ = visible;
// 	}

}

void DLBezierItem::SetShowAuxLine(bool bIsShowAuxLine)
{
	m_bIsShowAuxLine = bIsShowAuxLine;
}

Bezier DLBezierItem::cal_pro()
{

	Bezier pro;	
	pro.start_.x_=start_item_->pos().x();
	pro.start_.y_=start_item_->pos().y();
	pro.c1_.x_=c1_item_->pos().x();
	pro.c1_.y_=c1_item_->pos().y();
	pro.c2_.x_=c2_item_->pos().x();
	pro.c2_.y_=c2_item_->pos().y();
	pro.end_.x_=end_item_->pos().x();
	pro.end_.y_ = end_item_->pos().y();


	return pro;

}

void DLBezierItem::hide_control_point()
{
	set_control_point_visible(false);
}

QPointF DLBezierItem::start_pos()
{
	return start_item_->scenePos();
}

QPointF DLBezierItem::c1_pos()
{
	return c1_item_->scenePos();
}


QPointF DLBezierItem::c2_pos()
{
	return c2_item_->scenePos();
}

QPointF DLBezierItem::end_pos()
{
	return end_item_->scenePos();
}

void DLBezierItem::setPropertyList(std::vector<Property> PropertyVec)
{
	if (PropertyVec.size() == 0)
	{
		return;
	}
// 	m_vGoPropertys.clear();
// 	std::vector<Property>::iterator it;
// 	for (it = PropertyVec.begin(); it != PropertyVec.end(); ++it)
// 	{
// 		Property tempPro;
// 		tempPro.key = it->key;
// 		tempPro.type = it->type;
// 		tempPro.value = it->value;
// 		m_vGoPropertys.push_back(tempPro);
// 	}

	m_vGoPropertys = PropertyVec;

	if (m_vGoPropertys.size() <= ADVANCE_ARE_PRO)
	{
		//特殊处理高级区域
		if (m_bIsAdvanceArea)
		{
			Property proAdvanceArea("advancearea", "bool", "true");
			m_vGoPropertys.push_back(proAdvanceArea);
		}
		else
		{
			Property proAdvanceArea("advancearea", "bool", "false");
			m_vGoPropertys.push_back(proAdvanceArea);
		}
	}

	

	DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene());
	if (NULL != pScene)
	{
		emit pScene->SMAPChangedSignal();
	}
}

void DLBezierItem::setNagetivePropertyList(std::vector<Property> PropertyVec)
{
	if (PropertyVec.size() == 0)
	{
		return;
	}
	m_vBackPropertys.clear();
	m_vBackPropertys = PropertyVec;
// 	std::vector<Property>::iterator it;
// 	for (it = PropertyVec.begin(); it != PropertyVec.end(); ++it)
// 	{
// 		Property tempPro;
// 		tempPro.key = it->key;
// 		tempPro.type = it->type;
// 		tempPro.value = it->value;
// 		m_vBackPropertys.push_back(tempPro);
// 	}


	if (m_vBackPropertys.size() <= ADVANCE_ARE_PRO)
	{
		//特殊处理高级区域
		if (m_bIsAdvanceArea)
		{
			Property proAdvanceArea("advancearea", "bool", "true");
			m_vBackPropertys.push_back(proAdvanceArea);
		}
		else
		{
			Property proAdvanceArea("advancearea", "bool", "false");
			m_vBackPropertys.push_back(proAdvanceArea);
		}
	}
	

	DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene());
	if (NULL != pScene)
	{
		emit pScene->SMAPChangedSignal();
	}

}
