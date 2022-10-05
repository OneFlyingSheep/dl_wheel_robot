#include "AlarmMessageManager.h"

AlarmMessageManager *AlarmMessageManager::m_pAlarmMsgManager = nullptr;
AlarmMessageManager *AlarmMessageManager::GetInstance()
{
    if (nullptr == m_pAlarmMsgManager)
    {
        m_pAlarmMsgManager = new AlarmMessageManager;
    }
    return m_pAlarmMsgManager;
}

void AlarmMessageManager::GetAlarmMessage(int iId, AlarmMessage &alarmMessage)
{
    std::map<int, AlarmMessage>::iterator it = m_mapID2AlarmMessage.find(iId);
    if (it != m_mapID2AlarmMessage.end())
    {
        alarmMessage.iID = it->second.iID;
        alarmMessage.strSpeakAlarmMessage = it->second.strSpeakAlarmMessage;
        alarmMessage.strDisplayAlarmMessage = it->second.strDisplayAlarmMessage;
        alarmMessage.strErrorCode = it->second.strErrorCode;
    }
}

AlarmMessageManager::~AlarmMessageManager()
{

}

void AlarmMessageManager::Init()
{

    //��ǰ���ߵ��ͨѶ�쳣 1101
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_WALKING_MOTOR_COMMUNICATION, "��ǰ��ϵ����", "��ǰ���ߵ��ͨѶ�쳣");

    //��ǰ���ߵ���Ͽ����� 1102
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_WALKING_MOTOR_POWER, "��ǰ��ϵ����", "��ǰ���ߵ���Ͽ�����");

    //��ǰ���ߵ��״̬�쳣 1103
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_WALKING_MOTOR_STATUS, "��ǰ��ϵ����", "��ǰ���ߵ��״̬�쳣");

    //��ǰת����ͨѶ�쳣 1104
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_STEERING_MOTOR_COMMUNICATION, "��ǰ��ϵ����", "��ǰת����ͨѶ�쳣");

    //��ǰת�����Ͽ����� 1105
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_STEERING_MOTOR_POWER, "��ǰ��ϵ����", "��ǰת�����Ͽ�����");

    //��ǰת����״̬�쳣 1106
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_STEERING_MOTOR_STATUS, "��ǰ��ϵ����", "��ǰת����״̬�쳣");

    //������ߵ��ͨѶ�쳣  1201
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_WALKING_MOTOR_COMMUNICATION, "�����ϵ����", "������ߵ��ͨѶ�쳣");

    //������ߵ���Ͽ�����  1202
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_WALKING_MOTOR_POWER, "�����ϵ����", "������ߵ���Ͽ�����");

    //������ߵ��״̬�쳣 1203
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_WALKING_MOTOR_STATUS, "�����ϵ����", "������ߵ��״̬�쳣");

    //���ת����ͨѶ�쳣 1204
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_STEERING_MOTOR_COMMUNICATION, "�����ϵ����", "���ת����ͨѶ�쳣");

    //���ת�����Ͽ����� 1205
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_STEERING_MOTOR_POWER, "�����ϵ����", "���ת�����Ͽ�����");

    //���ת����״̬�쳣 1206
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_STEERING_MOTOR_STATUS, "�����ϵ����", "���ת����״̬�쳣");

    //��ǰ��ϵ����
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_WALKING_MOTOR_COMMUNICATION, "��ǰ��ϵ����", "��ǰ���ߵ��ͨѶ�쳣");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_WALKING_MOTOR_POWER, "��ǰ��ϵ����", "��ǰ���ߵ���Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_WALKING_MOTOR_STATUS, "��ǰ��ϵ����", "��ǰ���ߵ��״̬�쳣");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_STEERING_MOTOR_COMMUNICATION, "��ǰ��ϵ����", "��ǰת����ͨѶ�쳣");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_STEERING_MOTOR_POWER, "��ǰ��ϵ����", "��ǰת�����Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_STEERING_MOTOR_STATUS, "��ǰ��ϵ����", "��ǰת����״̬�쳣");
    //�Һ���ϵ����
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_WALKING_MOTOR_COMMUNICATION, "�Һ���ϵ����", "�Һ����ߵ��ͨѶ�쳣");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_WALKING_MOTOR_POWER, "�Һ���ϵ����", "�Һ����ߵ���Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_WALKING_MOTOR_STATUS, "�Һ���ϵ����", "�Һ����ߵ��״̬�쳣");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_STEERING_MOTOR_COMMUNICATION, "�Һ���ϵ����", "�Һ�ת����ͨѶ�쳣");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_STEERING_MOTOR_POWER, "�Һ���ϵ����", "�Һ�ת�����Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_STEERING_MOTOR_STATUS, "�Һ���ϵ����", "�Һ�ת����״̬�쳣");
    // ǰ��̽�⵽�ϰ���
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_SONAR, "ǰ��̽�⵽�ϰ���", "��������ǰ��̽�⵽�ϰ���");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_SONAR, "ǰ��̽�⵽�ϰ���", "��������ǰ��̽�⵽�ϰ���");
    InsertAlarmMessage(CLIENT_ALARM_LEFT_FRONT_ANTI_DROP, "̽�⵽̨��", "��������ǰ��̽�⵽̨��");
    InsertAlarmMessage(CLIENT_ALARM_LEFT_BACK_ANTI_DROP, "̽�⵽̨��", "���������̽�⵽̨��");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_FRONT_ANTI_DROP, "̽�⵽̨��", "��������ǰ��̽�⵽̨��");
    InsertAlarmMessage(CLIENT_ALARM_RIGHT_BACK_ANTI_DROP, "̽�⵽̨��", "�������Һ�̽�⵽̨��");
    InsertAlarmMessage(CLIENT_ALARM_LIDAR_COMMUNICATION, "�����״����", "�����״�ͨѶ�쳣");
    InsertAlarmMessage(CLIENT_ALARM_LIDAR_STATUS, "�����״����", "�����״�״̬�쳣");
    // ����������
    InsertAlarmMessage(CLIENT_ALARM_CONTROL_BOARD_CONNECT, "����������", "���ذ�Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_CONTROL_BOARD_TEMPERATURE, "�������¶ȹ���", "�������¶ȹ���");
    InsertAlarmMessage(CLIENT_ALARM_ROBOT_COMMUNICATION, "�����˶Ͽ�����", "�����˶Ͽ�����");
    // �����˵�������
    InsertAlarmMessage(CLIENT_ALARM_BATTERY_POWER, "�����˵�������", "�����˵�������");
    InsertAlarmMessage(CLIENT_ALARM_BATTERY_VOLTAGE, "�����˵�������", "�����˵�ѹ����");
    InsertAlarmMessage(CLIENT_ALARM_BATTERY_COMMUNICATION, "�����˵���쳣", "�����˵��ͨѶ�쳣");
    // ��̨����
    InsertAlarmMessage(CLIENT_ALARM_PTZ_CONNECT, "��̨����", "��̨�Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_HORIZONTAL_MOTOR_COMMUNICATIION, "��̨����", "��̨ˮƽ��ת���ͨѶ�쳣");
    InsertAlarmMessage(CLIENT_ALARM_HORIZONTAL_MOTOR_POWER, "��̨����", "��̨ˮƽ��ת����Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_HORIZONTAL_MOTOR_STATUS, "��̨����", "��̨ˮƽ��ת���״̬�쳣");
    InsertAlarmMessage(CLIENT_ALARM_PITCH_MOTOR_COMMUNICATION, "��̨����", "��̨�������ͨѶ�쳣");
    InsertAlarmMessage(CLIENT_ALARM_PITCH_MOTOR_POWER, "��̨����", "��̨��������Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_PITCH_MOTOR_STATUS, "��̨����", "��̨�������״̬�쳣");
    // �ɼ����������
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_CONNECT, "�ɼ����������", "�ɼ�������Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_SETFOCUS, "�ɼ����������", "�ɼ���������ý������ʧ��");
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_SETZOOM, "�ɼ����������", "�ɼ���������ñ���ʧ��");
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_CHECK_FOCUS_AND_ZOOM, "�ɼ����������", "�ɼ��������鱶�ʺͽ���ʧ��");
    InsertAlarmMessage(CLIENT_ALARM_CAMERA_CAPTURE, "�ɼ����������", "�ɼ����������ʧ��");
    // �����������
    InsertAlarmMessage(CLIENT_ALARM_INFRARED_CONNECT, "�����������", "��������Ͽ�����");
    InsertAlarmMessage(CLIENT_ALARM_INFRARED_FOCUS, "�����������", "��������Զ��Խ�ʧ��");
    InsertAlarmMessage(CLIENT_ALARM_INFRARED_CAPTURE, "�����������", "�����������ʧ��");
    // ��ͼ��Ϣ��ʧ
    InsertAlarmMessage(CLIENT_ALARM_MAP_FILE_LOST, "��ͼ��Ϣ��ʧ", "���ص�ͼ��ʧ");
    InsertAlarmMessage(CLIENT_ALARM_MAP_INFO_FILE_LOST, "��ͼ��Ϣ��ʧ", "����·�߶�ʧ");
    // �����˶�ʧ��λ
    InsertAlarmMessage(CLIENT_ALARM_LOCATION_RELIABILITY, "�����˶�ʧ��λ", "�����˶�ʧ��λ");
    // ������ƫ��·��
    InsertAlarmMessage(CLIENT_ALARM_ROBOT_DEVIATE_PATH, "������ƫ��·��", "������ƫ��·��");

    //������ �¶� �쳣
    InsertAlarmMessage(CLIENT_ALARM_CONTROL_T_ANOMALY, "������ �¶� �쳣", "�������¶��쳣��");

    //������ ʪ�� �쳣
    InsertAlarmMessage(CLIENT_ALARM_CONTROL_HUMIDTY_ANOMALY, "������ ʪ�� �쳣", "������ʪ���쳣��");

	//	İ����
	InsertAlarmMessage(CLIENT_CAP_STRANGER, "����İ����", "����İ����");

}

void AlarmMessageManager::InsertAlarmMessage(int iID, std::string strSpeakAlarmMessage, std::string strDisplayAlarmMessage, std::string strErrorCode)
{
    AlarmMessage stAlarmMessage;
    stAlarmMessage.iID = iID;
    stAlarmMessage.strSpeakAlarmMessage = strSpeakAlarmMessage;
    stAlarmMessage.strDisplayAlarmMessage = strDisplayAlarmMessage;
    stAlarmMessage.strErrorCode = strErrorCode;
    m_mapID2AlarmMessage.insert(std::make_pair(iID, stAlarmMessage));
}

AlarmMessageManager::AlarmMessageManager()
{
    Init();
}
