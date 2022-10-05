#include "LuaThreshold.h"
#include <QApplication>
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"

#define   LUA_FILE_PATH   "/hello.lua"

LuaThreshold::LuaThreshold()
{
}

LuaThreshold::~LuaThreshold()
{
}

DeviceAlarmLevel LuaThreshold::thresholdCheck(QString res, QString device_uuid, WheelRobotMeterType meter_type_id)
{
    ROS_INFO("thresholdCheck 1");
    QString threshold_filename;
    bool iDBRet;
    iDBRet = WHEEL_ROBOT_DB.getThresholdFileName(device_uuid, meter_type_id, threshold_filename);
    ROS_INFO("thresholdCheck 2. %s", threshold_filename.toStdString().c_str());
    if (iDBRet)
    {
        std::string filePath = std::string(QString(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/threshold/" + threshold_filename + ".lua").toLocal8Bit());
        ROS_INFO("thresholdCheck 3. %s", filePath.c_str());
        lua_State *L = luaL_newstate();
        if (L == NULL)
        {
            ROS_INFO("thresholdCheck 4");
            return Alarm_NONE;
        }
        //2.����Lua�ļ�  
        int bLuaRet = luaL_loadfile(L, filePath.c_str());
        if (bLuaRet)
        {
            const char *pErrorMsg = lua_tostring(L, -1);
            ROS_ERROR("luaL_loadfile error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
            lua_close(L);
            return Alarm_NONE;
        }
        //3.����Lua�ļ�  
        bLuaRet = lua_pcall(L, 0, 0, 0);
        if (bLuaRet)
        {
            const char *pErrorMsg = lua_tostring(L, -1);
            ROS_ERROR("lua_pcall 3 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
            lua_close(L);
            return Alarm_NONE;
        }

        //6.��ȡ����  
        lua_getglobal(L, "checkResult");        // ��ȡ������ѹ��ջ��  
        lua_pushnumber(L, res.toFloat());          // ѹ���һ������

        bLuaRet = lua_pcall(L, 1, 1, 0);// ���ú�������������Ժ󣬻Ὣ����ֵѹ��ջ�У�2��ʾ����������1��ʾ���ؽ��������  
        if (bLuaRet)                    // ���ó���  
        {
            const char *pErrorMsg = lua_tostring(L, -1);
            ROS_ERROR("lua_pcall 6 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
            lua_close(L);
            return Alarm_NONE;
        }

        if (lua_isnumber(L, -1))        //ȡֵ���  
        {
            
            int fValue = lua_tonumber(L, -1);
            ROS_INFO("thresholdCheck 5�� %d", fValue);
            lua_close(L);
            return (DeviceAlarmLevel)fValue;
        }
    }
    ROS_INFO("thresholdCheck 6");
    return Alarm_NONE;
}

DeviceAlarmLevel LuaThreshold::thresholdThreePhase(QString aPhase, QString bPhase, QString cPhase, float &max_temp_diff)
{
    return Alarm_NONE;
}

int LuaThreshold::checkLuaScript(QString luaPath, QString &errMsg)
{
    std::string filePath = std::string(luaPath.toLocal8Bit()); 

    lua_State *L = luaL_newstate();
    if (L == NULL)
    {
        errMsg = "load lua file err, file path:" + luaPath;
        return -1;
    }
    //2.����Lua�ļ�  
    int bLuaRet = luaL_loadfile(L, filePath.c_str());
    if (bLuaRet)
    {
        const char *pErrorMsg = lua_tostring(L, -1);
        errMsg = "luaL_loadfile error, errMsg:" + QString::fromLocal8Bit(pErrorMsg);
        lua_close(L);
        return bLuaRet;
    }
    //3.����Lua�ļ�  
    bLuaRet = lua_pcall(L, 0, 0, 0);
    if (bLuaRet)
    {
        const char *pErrorMsg = lua_tostring(L, -1);
        errMsg = "lua_pcall error, errMsg:" + QString::fromLocal8Bit(pErrorMsg);
        lua_close(L);
        return bLuaRet;
    }

    return bLuaRet;
}

DeviceAlarmLevel LuaThreshold::virtualLuaScript(QString res, QString virtual_uuid)
{
	QString threshold_filename;
	bool iDBRet;
	//��ʱʹ�ã����ڸ���
	iDBRet = WHEEL_ROBOT_DB.getThresholdFileNameForVirtual(virtual_uuid, threshold_filename);
	
	if (iDBRet)
	{
		std::string filePath = std::string(QString(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/threshold/" + threshold_filename + ".lua").toLocal8Bit());

		lua_State *L = luaL_newstate();
		if (L == NULL)
		{
			return Alarm_NONE;
		}
		//2.����Lua�ļ�  
		int bLuaRet = luaL_loadfile(L, filePath.c_str());
		if (bLuaRet)
		{
			const char *pErrorMsg = lua_tostring(L, -1);
			ROS_ERROR("luaL_loadfile error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
			lua_close(L);
			return Alarm_NONE;
		}
		//3.����Lua�ļ�  
		bLuaRet = lua_pcall(L, 0, 0, 0);
		if (bLuaRet)
		{
			const char *pErrorMsg = lua_tostring(L, -1);
			ROS_ERROR("lua_pcall 3 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
			lua_close(L);
			return Alarm_NONE;
		}

		//6.��ȡ����  
		lua_getglobal(L, "checkResult");        // ��ȡ������ѹ��ջ��  
		lua_pushnumber(L, res.toFloat());          // ѹ���һ������

		bLuaRet = lua_pcall(L, 1, 1, 0);// ���ú�������������Ժ󣬻Ὣ����ֵѹ��ջ�У�2��ʾ����������1��ʾ���ؽ��������  
		if (bLuaRet)                    // ���ó���  
		{
			const char *pErrorMsg = lua_tostring(L, -1);
			ROS_ERROR("lua_pcall 6 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
			lua_close(L);
			return Alarm_NONE;
		}

		if (lua_isnumber(L, -1))        //ȡֵ���  
		{
			int fValue = lua_tonumber(L, -1);
			lua_close(L);
			return (DeviceAlarmLevel)fValue;
		}
	}
	return Alarm_NONE;
}

QString LuaThreshold::phonyValueScript(QString uuid_)
{
	QString threshold_filename;

	std::string filePath = std::string(QString("D:/phonyvalue.lua").toLocal8Bit());

	lua_State *L = luaL_newstate();
	if (L == NULL)
	{
		return "-2";
	}
	//2.����Lua�ļ�  
	int bLuaRet = luaL_loadfile(L, filePath.c_str());
	if (bLuaRet)
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		ROS_ERROR("luaL_loadfile error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
		lua_close(L);
		return "-2";
	}
	//3.����Lua�ļ�  
	bLuaRet = lua_pcall(L, 0, 0, 0);
	if (bLuaRet)
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		ROS_ERROR("lua_pcall 3 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
		lua_close(L);
		return "-2";
	}

	//6.��ȡ����  
	lua_getglobal(L, "deviceResult");        // ��ȡ������ѹ��ջ��  
//	lua_pushnumber(L, res.toFloat());          // ѹ���һ������
	lua_pushstring(L, uuid_.toStdString().c_str());

	bLuaRet = lua_pcall(L, 1, 1, 0);// ���ú�������������Ժ󣬻Ὣ����ֵѹ��ջ�У�2��ʾ����������1��ʾ���ؽ��������  
	if (bLuaRet)                    // ���ó���  
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		ROS_ERROR("lua_pcall 6 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
		lua_close(L);
		return "-2";
	}

	if (lua_isnumber(L, -1))        //ȡֵ���  
	{
		std::string fValue = lua_tostring(L, -1);
		lua_close(L);
		return QString::fromStdString(fValue);
	}

	return "-2";
}

bool LuaThreshold::getLoginData(QStringList &_data)
{
	QString threshold_filename;
	
	//��ʱʹ�ã����ڸ���
	std::string filePath = QApplication::applicationDirPath().toStdString() + "/doLogin.lua";

	lua_State *L = luaL_newstate();
	if (L == NULL)
	{
		return false;
	}
	//2.����Lua�ļ�  
	int bLuaRet = luaL_loadfile(L, filePath.c_str());
	if (bLuaRet)
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		lua_close(L);
		return false;
	}
	//3.����Lua�ļ�  
	bLuaRet = lua_pcall(L, 0, 0, 0);
	if (bLuaRet)
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		lua_close(L);
		return false;
	}

	//6.��ȡ����  
	lua_getglobal(L, "LoginMsg");        // ��ȡ������ѹ��ջ��  

	bLuaRet = lua_pcall(L, 0, 2, 0);// ���ú�������������Ժ󣬻Ὣ����ֵѹ��ջ�У�2��ʾ����������1��ʾ���ؽ��������  
	if (bLuaRet)                    // ���ó���  
	{
		lua_close(L);
		return false;
	}

	if (lua_isstring(L, -2))        //ȡֵ���  
	{
		std::string username = lua_tostring(L, -2);
		std::string password = lua_tostring(L, -1);

		lua_close(L);

		_data.append(QString::fromStdString(username));
		_data.append(QString::fromStdString(password));
	}
	return true;
}