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
        //2.加载Lua文件  
        int bLuaRet = luaL_loadfile(L, filePath.c_str());
        if (bLuaRet)
        {
            const char *pErrorMsg = lua_tostring(L, -1);
            ROS_ERROR("luaL_loadfile error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
            lua_close(L);
            return Alarm_NONE;
        }
        //3.运行Lua文件  
        bLuaRet = lua_pcall(L, 0, 0, 0);
        if (bLuaRet)
        {
            const char *pErrorMsg = lua_tostring(L, -1);
            ROS_ERROR("lua_pcall 3 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
            lua_close(L);
            return Alarm_NONE;
        }

        //6.读取函数  
        lua_getglobal(L, "checkResult");        // 获取函数，压入栈中  
        lua_pushnumber(L, res.toFloat());          // 压入第一个参数

        bLuaRet = lua_pcall(L, 1, 1, 0);// 调用函数，调用完成以后，会将返回值压入栈中，2表示参数个数，1表示返回结果个数。  
        if (bLuaRet)                    // 调用出错  
        {
            const char *pErrorMsg = lua_tostring(L, -1);
            ROS_ERROR("lua_pcall 6 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
            lua_close(L);
            return Alarm_NONE;
        }

        if (lua_isnumber(L, -1))        //取值输出  
        {
            
            int fValue = lua_tonumber(L, -1);
            ROS_INFO("thresholdCheck 5， %d", fValue);
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
    //2.加载Lua文件  
    int bLuaRet = luaL_loadfile(L, filePath.c_str());
    if (bLuaRet)
    {
        const char *pErrorMsg = lua_tostring(L, -1);
        errMsg = "luaL_loadfile error, errMsg:" + QString::fromLocal8Bit(pErrorMsg);
        lua_close(L);
        return bLuaRet;
    }
    //3.运行Lua文件  
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
	//暂时使用，后期更改
	iDBRet = WHEEL_ROBOT_DB.getThresholdFileNameForVirtual(virtual_uuid, threshold_filename);
	
	if (iDBRet)
	{
		std::string filePath = std::string(QString(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/threshold/" + threshold_filename + ".lua").toLocal8Bit());

		lua_State *L = luaL_newstate();
		if (L == NULL)
		{
			return Alarm_NONE;
		}
		//2.加载Lua文件  
		int bLuaRet = luaL_loadfile(L, filePath.c_str());
		if (bLuaRet)
		{
			const char *pErrorMsg = lua_tostring(L, -1);
			ROS_ERROR("luaL_loadfile error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
			lua_close(L);
			return Alarm_NONE;
		}
		//3.运行Lua文件  
		bLuaRet = lua_pcall(L, 0, 0, 0);
		if (bLuaRet)
		{
			const char *pErrorMsg = lua_tostring(L, -1);
			ROS_ERROR("lua_pcall 3 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
			lua_close(L);
			return Alarm_NONE;
		}

		//6.读取函数  
		lua_getglobal(L, "checkResult");        // 获取函数，压入栈中  
		lua_pushnumber(L, res.toFloat());          // 压入第一个参数

		bLuaRet = lua_pcall(L, 1, 1, 0);// 调用函数，调用完成以后，会将返回值压入栈中，2表示参数个数，1表示返回结果个数。  
		if (bLuaRet)                    // 调用出错  
		{
			const char *pErrorMsg = lua_tostring(L, -1);
			ROS_ERROR("lua_pcall 6 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
			lua_close(L);
			return Alarm_NONE;
		}

		if (lua_isnumber(L, -1))        //取值输出  
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
	//2.加载Lua文件  
	int bLuaRet = luaL_loadfile(L, filePath.c_str());
	if (bLuaRet)
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		ROS_ERROR("luaL_loadfile error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
		lua_close(L);
		return "-2";
	}
	//3.运行Lua文件  
	bLuaRet = lua_pcall(L, 0, 0, 0);
	if (bLuaRet)
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		ROS_ERROR("lua_pcall 3 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
		lua_close(L);
		return "-2";
	}

	//6.读取函数  
	lua_getglobal(L, "deviceResult");        // 获取函数，压入栈中  
//	lua_pushnumber(L, res.toFloat());          // 压入第一个参数
	lua_pushstring(L, uuid_.toStdString().c_str());

	bLuaRet = lua_pcall(L, 1, 1, 0);// 调用函数，调用完成以后，会将返回值压入栈中，2表示参数个数，1表示返回结果个数。  
	if (bLuaRet)                    // 调用出错  
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		ROS_ERROR("lua_pcall 6 error, path:%s, errMsg:%s", filePath.c_str(), pErrorMsg);
		lua_close(L);
		return "-2";
	}

	if (lua_isnumber(L, -1))        //取值输出  
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
	
	//暂时使用，后期更改
	std::string filePath = QApplication::applicationDirPath().toStdString() + "/doLogin.lua";

	lua_State *L = luaL_newstate();
	if (L == NULL)
	{
		return false;
	}
	//2.加载Lua文件  
	int bLuaRet = luaL_loadfile(L, filePath.c_str());
	if (bLuaRet)
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		lua_close(L);
		return false;
	}
	//3.运行Lua文件  
	bLuaRet = lua_pcall(L, 0, 0, 0);
	if (bLuaRet)
	{
		const char *pErrorMsg = lua_tostring(L, -1);
		lua_close(L);
		return false;
	}

	//6.读取函数  
	lua_getglobal(L, "LoginMsg");        // 获取函数，压入栈中  

	bLuaRet = lua_pcall(L, 0, 2, 0);// 调用函数，调用完成以后，会将返回值压入栈中，2表示参数个数，1表示返回结果个数。  
	if (bLuaRet)                    // 调用出错  
	{
		lua_close(L);
		return false;
	}

	if (lua_isstring(L, -2))        //取值输出  
	{
		std::string username = lua_tostring(L, -2);
		std::string password = lua_tostring(L, -1);

		lua_close(L);

		_data.append(QString::fromStdString(username));
		_data.append(QString::fromStdString(password));
	}
	return true;
}