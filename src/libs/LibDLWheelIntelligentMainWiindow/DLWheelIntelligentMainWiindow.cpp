#include "DLWheelIntelligentMainWiindow.h"
#include <QGridLayout>
#include <QDateTime>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include "LibDLIntelligentSocket/BJIntelligentData.hpp"

#pragma execution_character_set("utf-8")

DLWheelIntelligentMainWiindow::DLWheelIntelligentMainWiindow(QWidget* parent /*= NULL*/)
{
    initWidget();

    this->setStyleSheet("QPushButton{font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
						QPushButton:hover{background-color:rgb(44 , 137 , 255);}\
						QPushButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");
}

void DLWheelIntelligentMainWiindow::initSystemMsgButton()
{
    m_systemButtonBackWidget = new QGroupBox;
    m_systemButtonBackWidget->setTitle("System Msg");

    QLabel* ipLabel = new QLabel("IP");
    QLineEdit* ipLineEdit = new QLineEdit;
    ipLineEdit->setText("192.168.60.1");

    QLabel* portLabel = new QLabel("Port");
    QLineEdit* portLineEdit = new QLineEdit;
    portLineEdit->setText("8234");

    QLabel* LocalCodeLabel = new QLabel("Local_Code");
    QLineEdit* LocalCodeLineEdit = new QLineEdit;
    LocalCodeLineEdit->setText("Client01");

    QLabel* remoteCodeLabel = new QLabel("Remote_Code");
    QLineEdit* remoteCodeLineEdit = new QLineEdit;
    remoteCodeLineEdit->setText("Server01");

    QLabel* ftp_user_name_Label = new QLabel("ftp_username");
    QLineEdit* ftp_user_name_Edit = new QLineEdit;
    ftp_user_name_Edit->setText("robotftps");

    QLabel* ftp_passwd_Label = new QLabel("ftp_passwd");
    QLineEdit* ftp_passwd_Edit = new QLineEdit;
    ftp_passwd_Edit->setEchoMode(QLineEdit::Password);
    ftp_passwd_Edit->setText("robot123");

    QLabel* ftpsPortLabel = new QLabel("ftpsPort");
    QLineEdit* ftpsPortLineEdit = new QLineEdit;
    ftpsPortLineEdit->setText("21");

    QGridLayout* gLineLayout = new QGridLayout;
    gLineLayout->addWidget(ipLabel, 0, 0);
    gLineLayout->addWidget(ipLineEdit, 0, 1);
    gLineLayout->addWidget(portLabel, 0, 2);
    gLineLayout->addWidget(portLineEdit, 0, 3);
    gLineLayout->addWidget(LocalCodeLabel, 1, 0);
    gLineLayout->addWidget(LocalCodeLineEdit, 1, 1);
    gLineLayout->addWidget(remoteCodeLabel, 1, 2);
    gLineLayout->addWidget(remoteCodeLineEdit, 1, 3);
    gLineLayout->addWidget(ftp_user_name_Label, 2, 0);
    gLineLayout->addWidget(ftp_user_name_Edit, 2, 1);
    gLineLayout->addWidget(ftp_passwd_Label, 2, 2);
    gLineLayout->addWidget(ftp_passwd_Edit, 2, 3);
    gLineLayout->addWidget(ftpsPortLabel, 3, 0);
    gLineLayout->addWidget(ftpsPortLineEdit, 3, 1);


    QPushButton* pButtonConnect = new QPushButton("Connect");
    pButtonConnect->setFixedSize(QSize(150, 25));
    connect(pButtonConnect, &QPushButton::clicked, this, [=] {
        m_socketClent = boost::make_shared<LibDLIntelligentSocket>(LocalCodeLineEdit->text().toStdString(), remoteCodeLineEdit->text().toStdString());
        m_socketClent->connect(ipLineEdit->text().toStdString(), portLineEdit->text().toInt(),
            ftpsPortLineEdit->text().toInt(), ftp_user_name_Edit->text().toStdString(), ftp_passwd_Edit->text().toStdString());
    });

    // system msg
    QPushButton* pButtonRegisterMsg = new QPushButton("RegisterMsg");
    pButtonRegisterMsg->setFixedSize(QSize(150, 25));
    connect(pButtonRegisterMsg, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            m_socketClent->send_register_msg_req();
        }
    });

    QPushButton* pButtonHeartBeatMsg = new QPushButton("HeartBeatMsg");
    pButtonHeartBeatMsg->setFixedSize(QSize(150, 25));
    connect(pButtonHeartBeatMsg, &QPushButton::clicked, this, [=] {
     //   if (onCheckConnect())
        {
        //    m_socketClent->send_heart_beat_msg_req();
            boost::shared_ptr<XmlProtocolMsg> msg;
            m_socketClent->recv_model_sync_resp(msg);
        }
        
    });

    QPushButton* pButtonCommonResponseNoItem = new QPushButton("CommonResponseNoItem");
    pButtonCommonResponseNoItem->setFixedSize(QSize(150, 25));
    connect(pButtonCommonResponseNoItem, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            m_socketClent->send_common_response_no_item();
        }        
    });

    QPushButton* pButtonCommonResponseWithItem = new QPushButton("CommonResponseWithItem");
    pButtonCommonResponseWithItem->setFixedSize(QSize(150, 25));
    connect(pButtonCommonResponseWithItem, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            ROS_INFO("error ");
        }
    });

    QGridLayout* gSystemMsgLayout = new QGridLayout(m_systemButtonBackWidget);
    gSystemMsgLayout->addWidget(pButtonConnect, 0, 0);
    gSystemMsgLayout->addWidget(pButtonRegisterMsg, 1, 0);
    gSystemMsgLayout->addWidget(pButtonHeartBeatMsg, 1, 1);
    gSystemMsgLayout->addLayout(gLineLayout, 0, 2, 2, 1);
    gSystemMsgLayout->setSpacing(20);

    testTimer = new QTimer(this);
    connect(testTimer, &QTimer::timeout, this, [=] {
        if (onCheckConnect())
        {
            robotRunningStatus msg;
            msg.robot_code = "Robot1";
            msg.robot_name = "testRobot1";
            msg.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            tvu temp1;
            temp1.type = "1";
            temp1.value = "1.0";
            temp1.value_unit = "m/s";
            temp1.unit = "m/s";

            tvu temp2;
            temp2.type = "2";
            temp2.value = "100";
            temp2.value_unit = "m";
            temp2.unit = "m";

            tvu temp3;
            temp3.type = "3";
            temp3.value = "29.0";
            temp3.value_unit = "V";
            temp3.unit = "V";

            msg.data.push_back(temp1);
            msg.data.push_back(temp2);
            msg.data.push_back(temp3);

            m_socketClent->send_robot_running_status_msg_req(msg);
        }
    });
    
}

void DLWheelIntelligentMainWiindow::initMonitorMsgButton()
{
    m_monitorButtonBackWidget = new QWidget;

    // monitor msg
    QPushButton* pButtonRobotStatusReq = new QPushButton("RobotStatusReq");
    pButtonRobotStatusReq->setFixedSize(QSize(150, 25));
    connect(pButtonRobotStatusReq, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            robotStatus status;
            status.robot_name = "testRobot1";
            status.robot_code = "Robot1";
            status.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            tvu temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9;
            temp1.type = std::to_string(1);
            temp1.value = "22.8";
            temp1.value_unit = "22.8V";
            temp1.unit = "V";

            temp2.type = std::to_string(2);
            temp2.value = "1";
            temp2.value_unit = "1";
            temp2.unit = "";

            temp3.type = std::to_string(3);
            temp3.value = "1";
            temp3.value_unit = "1";
            temp3.unit = "";

            temp4.type = std::to_string(4);
            temp4.value = "0";
            temp4.value_unit = "0";
            temp4.unit = "";

            temp5.type = std::to_string(21);
            temp5.value = "0";
            temp5.value_unit = "0";
            temp5.unit = "0";

            temp6.type = std::to_string(41);
            temp6.value = "2";
            temp6.value_unit = "2";
            temp6.unit = "";

            temp7.type = std::to_string(61);
            temp7.value = "4";
            temp7.value_unit = "4";
            temp7.unit = "";

            temp8.type = std::to_string(81);
            temp8.value = "0";
            temp8.value_unit = "0";
            temp8.unit = "";

            temp9.type = std::to_string(101);
            temp9.value = "1";
            temp9.value_unit = "1";
            temp9.unit = "";

            status.data.push_back(temp1);
            status.data.push_back(temp2);
            status.data.push_back(temp3);
            status.data.push_back(temp4);
            status.data.push_back(temp5);
            status.data.push_back(temp6);
            status.data.push_back(temp7);
            status.data.push_back(temp8);
            status.data.push_back(temp9);

            m_socketClent->send_robot_status_msg_req(status);
        }
    });
    QPushButton* pButtonRobotStatusRsp = new QPushButton("RobotStatusRsp");
    pButtonRobotStatusRsp->setFixedSize(QSize(150, 25));
    connect(pButtonRobotStatusRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
			//m_socketClent->test_send_msg();
            testTimer->start(5000);
        }
    });

    QPushButton* pButtonRobotRunningStatusReq = new QPushButton("RobotRunningStatusReq");
    pButtonRobotRunningStatusReq->setFixedSize(QSize(150, 25));
    connect(pButtonRobotRunningStatusReq, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            robotRunningStatus msg;
            msg.robot_code = "Robot1";
            msg.robot_name = "testRobot1";
            msg.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            tvu temp1;
            temp1.type = "1";
            temp1.value = "1.0";
            temp1.value_unit = "m/s";
            temp1.unit = "m/s";

            tvu temp2;
            temp2.type = "2";
            temp2.value = "100";
            temp2.value_unit = "m";
            temp2.unit = "m";

            tvu temp3;
            temp3.type = "3";
            temp3.value = "29.0";
            temp3.value_unit = "V";
            temp3.unit = "V";

            msg.data.push_back(temp1);
            msg.data.push_back(temp2);
            msg.data.push_back(temp3);

            m_socketClent->send_robot_running_status_msg_req(msg);
        }
    });
    QPushButton* pButtonRobotRunningStatusRsp = new QPushButton("RobotRunningStatusRsp");
    pButtonRobotRunningStatusRsp->setFixedSize(QSize(150, 25));
    connect(pButtonRobotRunningStatusRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
        }
    });

    QPushButton* pButtonRobotCoordinateReq = new QPushButton("RobotCoordinateReq");
    pButtonRobotCoordinateReq->setFixedSize(QSize(150, 25));
    connect(pButtonRobotCoordinateReq, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            robotCoordinate data;
            data.robot_code = "Robot1";
            data.robot_name = "testRobot1";
            data.file_path = "D:\\Test\\File.jpg";
            data.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            data.coordinate_pixel = "1,2,3,4";
            data.coordinate_geography = "5,7,8,9";

            m_socketClent->send_robot_coordinate_msg_req(data);
        }
    });
    QPushButton* pButtonRobotCoordinateRsp = new QPushButton("RobotCoordinateRsp");
    pButtonRobotCoordinateRsp->setFixedSize(QSize(150, 25));
    connect(pButtonRobotCoordinateRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
        }
    });

    QPushButton* pButtonPatrolLineReq = new QPushButton("PatrolLineReq");
    pButtonPatrolLineReq->setFixedSize(QSize(150, 25));
    connect(pButtonPatrolLineReq, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
        }
    });
    QPushButton* pButtonPatrolLineRsp = new QPushButton("PatrolLineRsp");
    pButtonPatrolLineRsp->setFixedSize(QSize(150, 25));
    connect(pButtonPatrolLineRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
        }
    });

    QPushButton* pButtonRobotAlarmReq = new QPushButton("RobotAlarmReq");
    pButtonRobotAlarmReq->setFixedSize(QSize(150, 25));
    connect(pButtonRobotAlarmReq, &QPushButton::clicked, this, [=] {
        //if (onCheckConnect())
        {
            robotAlarmData data;
            data.robot_code = "Robot1";
            data.robot_name = "testRobot1";
            data.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            data.content = "robot test alarm";
            m_socketClent->send_robot_alarm_msg_req(data);
        }
    });
    QPushButton* pButtonRobotAlarmRsp = new QPushButton("RobotAlarmRsp");
    pButtonRobotAlarmRsp->setFixedSize(QSize(150, 25));
    connect(pButtonRobotAlarmRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            m_socketClent->ftp_send_test();
        }
    });

    QPushButton* pButtonRobotMicroWeatherReq = new QPushButton("RobotMicroWeatherReq");
    pButtonRobotMicroWeatherReq->setFixedSize(QSize(150, 25));
    connect(pButtonRobotMicroWeatherReq, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            robotMicroWeatherData msg;
            msg.robot_code = "Robot1";
            msg.robot_name = "testRobot1";
            msg.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();

            tvu temp1;
            temp1.type = "1";
            temp1.value = "28.0";
            temp1.value_unit = "℃";
            temp1.unit = "℃";

            tvu temp2;
            temp2.type = "2";
            temp2.value = "58";
            temp2.value_unit = "%";
            temp2.unit = "%";

            tvu temp3;
            temp3.type = "3";
            temp3.value = "1.0";
            temp3.value_unit = "m/s";
            temp3.unit = "m/s";

            msg.data.push_back(temp1);
            msg.data.push_back(temp2);
            msg.data.push_back(temp3);

            m_socketClent->send_robot_micro_weather_req(msg);
        }
    });
    QPushButton* pButtonRobotMicroWeatherRsp = new QPushButton("RobotMicroWeatherRsp");
    pButtonRobotMicroWeatherRsp->setFixedSize(QSize(150, 25));
    connect(pButtonRobotMicroWeatherRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
        }
    });

    QPushButton* pButtonRobotTaskDataReq = new QPushButton("RobotTaskDataReq");
    pButtonRobotTaskDataReq->setFixedSize(QSize(150, 25));
    connect(pButtonRobotTaskDataReq, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            robotTaskData data;
            data.task_patrolled_id = "123";
            data.task_name = "test Robot task";
            data.task_code = "task1";
            data.task_state = "1";
            data.plan_start_time = "2019-05-08 08:20:11";
            data.start_time = "2019-05-08 08:20:20";
            data.task_progress = "100%";
            data.task_estimated_time = "620";
            data.description = "this is a test task";

            m_socketClent->send_robot_task_data_msg_req(data);
        }
    });
    QPushButton* pButtonRobotTaskDataRsp = new QPushButton("RobotTaskDataRsp");
    pButtonRobotTaskDataRsp->setFixedSize(QSize(150, 25));
    connect(pButtonRobotTaskDataRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
        }
    });

    QPushButton* pButtonRobotPatrolResultReq = new QPushButton("RobotPatrolResultReq");
    pButtonRobotPatrolResultReq->setFixedSize(QSize(150, 25));
    connect(pButtonRobotPatrolResultReq, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            robotPatrolResult task;
            robotDeviceUploadType devRes1, devRes2, devRes3;
            task.robot_code = "Robot1";
            task.task_name = "test Robot task";
            task.task_code = "task1";
            task.task_patrolled_id = "task01";

            devRes1.device_name = "test dev 1 infrared";
            devRes1.device_id = "test id dev 1";
            devRes1.value = "29.0";
            devRes1.value_unit = "29.0℃";
            devRes1.unit = "℃";
            devRes1.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            devRes1.recognition_type = "4";
            devRes1.file_type = "1";
            devRes1.file_path = "D:\\test\\test1";
            devRes1.rectangle = "1,2,3,4";

            devRes2.device_name = "test dev 2 visible";
            devRes2.device_id = "testiddev2";
            devRes2.value = "21.1";
            devRes2.value_unit = "29.0V";
            devRes2.unit = "V";
            devRes2.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            devRes2.recognition_type = "1";
            devRes2.file_type = "2";
            devRes2.file_path = "D:\\test\\test2";
            devRes2.rectangle = "1,2,3,4";

            devRes3.device_name = "test dev 3 visible";
            devRes3.device_id = "test id dev 3";
            devRes3.value = "18.9";
            devRes3.value_unit = "18.9V";
            devRes3.unit = "V";
            devRes3.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            devRes3.recognition_type = "2";
            devRes3.file_type = "1";
            devRes3.file_path = "D:\\test\\test4";
            devRes3.rectangle = "1,2,3,4";

            task.data.push_back(devRes1);
            task.data.push_back(devRes2);
            task.data.push_back(devRes3);

            m_socketClent->send_robot_patrol_result_msg_req(task);
        }

    });
    QPushButton* pButtonRobotPatrolResultRsp = new QPushButton("RobotPatrolResultRsp");
    pButtonRobotPatrolResultRsp->setFixedSize(QSize(150, 25));
    connect(pButtonRobotPatrolResultRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
        }
    });

    QPushButton* pButtonRobotDevAlarmDataReq = new QPushButton("RobotDevAlarmDataReq");
    pButtonRobotDevAlarmDataReq->setFixedSize(QSize(150, 25));
    connect(pButtonRobotDevAlarmDataReq, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            robotDevAlarmData data;
            robotDeviceAlarmType alarmData1, alarmData2;
            data.robot_code = "Robot1";
            data.task_name = "test Robot task";
            data.task_code = "task1";
            data.task_patrolled_id = "task01";

            alarmData1.device_name = "test dev 1 visible";
            alarmData1.device_id = "taskdev1";
            alarmData1.alarm_level = "3";
            alarmData1.alarm_type = "6";
            alarmData1.recognition_type = "3";
            alarmData1.value = "";
            alarmData1.value_unit = "";
            alarmData1.unit = "";
            alarmData1.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            alarmData1.content = "visible test alarm";

            alarmData2.device_name = "test dev 2 visible";
            alarmData2.device_id = "taskdev2";
            alarmData2.alarm_level = "1";
            alarmData2.alarm_type = "1";
            alarmData2.recognition_type = "4";
            alarmData2.value = "158.0";
            alarmData2.value_unit = "℃";
            alarmData2.unit = "℃";
            alarmData2.time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss").toStdString();
            alarmData2.content = "infrared test alarm";

            data.data.push_back(alarmData1);
            data.data.push_back(alarmData2);

            m_socketClent->send_robot_dev_alarm_data_msg_req(data);
        }
    });
    QPushButton* pButtonRobotDevAlarmDataRsp = new QPushButton("RobotDevAlarmDataRsp");
    pButtonRobotDevAlarmDataRsp->setFixedSize(QSize(150, 25));
    connect(pButtonRobotDevAlarmDataRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
        }
    });

    QPushButton* pButtonRobotPatrolReportReq = new QPushButton("RobotPatrolReportReq");
    pButtonRobotPatrolReportReq->setFixedSize(QSize(150, 25));
    connect(pButtonRobotPatrolReportReq, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
            robotPatrolReportData res;
            res.task_patrolled_id = "task01";
            res.task_name = "test Robot task";
            res.task_code = "task1";
            res.plan_start_time = "2019-05-08 08:20:11";
            res.start_time = "2019-05-08 08:20:20";
            res.end_time = "2019-05-08 08:40:25";
            res.all_count = 100;
            res.normal_count = 80;
            res.all_count = 10;
            res.recognition_error_count = 5;
            res.miss_count = 5;
            res.temperature = "28.5℃";
            res.humidity = "58%";
            res.wind_speed = "0.6m/3";
            res.description = "task report test";
            res.road_file_path = "D:\\test1.map";
            res.task_finish_state = "1";

            m_socketClent->send_robot_patrol_report_msg_req(res);
        }
    });
    QPushButton* pButtonRobotPatrolReportRsp = new QPushButton("RobotPatrolReportRsp");
    pButtonRobotPatrolReportRsp->setFixedSize(QSize(150, 25));
    connect(pButtonRobotPatrolReportRsp, &QPushButton::clicked, this, [=] {
        if (onCheckConnect())
        {
        }
    });

    QGroupBox* groupLeft = new QGroupBox;
    groupLeft->setTitle("Req");
    QGridLayout* gMoniotorMsgLayoutLeft = new QGridLayout(groupLeft);
    gMoniotorMsgLayoutLeft->addWidget(pButtonRobotStatusReq, 0, 0);
    gMoniotorMsgLayoutLeft->addWidget(pButtonRobotMicroWeatherReq, 0, 1);
    gMoniotorMsgLayoutLeft->addWidget(pButtonRobotRunningStatusReq, 1, 0);
    gMoniotorMsgLayoutLeft->addWidget(pButtonRobotTaskDataReq, 1, 1);
    gMoniotorMsgLayoutLeft->addWidget(pButtonRobotCoordinateReq, 2, 0);
    gMoniotorMsgLayoutLeft->addWidget(pButtonRobotPatrolResultReq, 2, 1);
    gMoniotorMsgLayoutLeft->addWidget(pButtonPatrolLineReq, 3, 0);
    gMoniotorMsgLayoutLeft->addWidget(pButtonRobotDevAlarmDataReq, 3, 1);
    gMoniotorMsgLayoutLeft->addWidget(pButtonRobotAlarmReq, 4, 0);
    gMoniotorMsgLayoutLeft->addWidget(pButtonRobotPatrolReportReq, 4, 1);
    gMoniotorMsgLayoutLeft->setSpacing(20);

    QGroupBox* groupRight = new QGroupBox;
    groupRight->setTitle("Rsp");

    QGridLayout* gMoniotorMsgLayoutRight = new QGridLayout(groupRight);
    gMoniotorMsgLayoutRight->addWidget(pButtonRobotStatusRsp, 0, 0);
    gMoniotorMsgLayoutRight->addWidget(pButtonRobotMicroWeatherRsp, 0, 1);
    gMoniotorMsgLayoutRight->addWidget(pButtonRobotRunningStatusRsp, 1, 0);
    gMoniotorMsgLayoutRight->addWidget(pButtonRobotTaskDataRsp, 1, 1);
    gMoniotorMsgLayoutRight->addWidget(pButtonRobotCoordinateRsp, 2, 0);
    gMoniotorMsgLayoutRight->addWidget(pButtonRobotPatrolResultRsp, 2, 1);
    gMoniotorMsgLayoutRight->addWidget(pButtonPatrolLineRsp, 3, 0);
    gMoniotorMsgLayoutRight->addWidget(pButtonRobotDevAlarmDataRsp, 3, 1);
    gMoniotorMsgLayoutRight->addWidget(pButtonRobotAlarmRsp, 4, 0);
    gMoniotorMsgLayoutRight->addWidget(pButtonRobotPatrolReportRsp, 4, 1);
    gMoniotorMsgLayoutRight->setSpacing(20);

    QHBoxLayout* hMonitorLayout = new QHBoxLayout(m_monitorButtonBackWidget);
    hMonitorLayout->addWidget(groupLeft);
    hMonitorLayout->addWidget(groupRight);
    hMonitorLayout->setSpacing(20);
}

void DLWheelIntelligentMainWiindow::initWidget()
{
    initSystemMsgButton();
    initMonitorMsgButton();
   

    QVBoxLayout* vLayout = new QVBoxLayout;
    vLayout->addWidget(m_systemButtonBackWidget);
    vLayout->addWidget(m_monitorButtonBackWidget);
    vLayout->setSpacing(50);

    QHBoxLayout* hLayout = new QHBoxLayout(this);
    hLayout->addStretch();
    hLayout->addLayout(vLayout);
    hLayout->addStretch();

}

bool DLWheelIntelligentMainWiindow::onCheckConnect()
{
    if (m_socketClent == NULL || m_socketClent->bConnected() == false)
    {
        //QMessageBox::information(NULL, "Tip", "请先建立连接");
        ROS_INFO("connect lost ");
        return false;
    }

    return true;
}
