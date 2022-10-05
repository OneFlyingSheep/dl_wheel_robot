#include "HikCameraPointData.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"

HikPointData* HikPointData::m_HikObject = new HikPointData;

HikPointData::HikPointData()
{
}

HikPointData* HikPointData::getInitance()
{
	return m_HikObject;
}
void HikPointData::init()
{
	setZoomInfoMapDefault();
	setZoomValueList();
	QString filePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + QString("/ZoomCaptureImage/zoomValue/%1.txt").arg(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().strCheckZoomTxt);
	readZoomPtzValueFromTxt(filePath);
}
void HikPointData::readZoomPtzValueFromTxt(QString filePath)
{
	QFile file(filePath);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		m_zoomInfoMap.clear();
		while (!file.atEnd())
		{
			QStringList strInfo = QString(file.readLine()).split(",");
			zoomInfo info;
			info.zoomValue = strInfo[0].toInt();
			info.verticalAngle = strInfo[1].toFloat();
			info.levelAangle = strInfo[2].toFloat();
			setZoomInfoMap(info);
		}
		file.close();
		m_isUseDefaultZoomMap = false;
	}
	else
	{
		m_isUseDefaultZoomMap = true;
	}
}

zoomInfo HikPointData::getZoomInfoData(int zoomValue)
{
    if (m_isUseDefaultZoomMap)
    {
        if (zoomValue > 16384)
        {
            zoomInfo info;
            info.zoomValue = zoomValue;
            info.levelAangle = 12540 * exp(-0.0005 * zoomValue);
            info.verticalAngle = 7200 * exp(-0.0005 * zoomValue);
            return info;
        }
        return m_zoomInfoDefaultMap[zoomValue];
    }
    else
    {
        return m_zoomInfoMap[zoomValue];
    }
    return m_zoomInfoDefaultMap[zoomValue];
}

int HikPointData::getZoomValue(int zoomValue)
{
    if (zoomValue > 16384)
    {
        return zoomValue;
    }

	int _i = 0;
	for (auto _value : m_zoomValue)
	{
		if (_value == zoomValue)
		{
			return _value;
		}
		if (zoomValue < _value)
		{
			if (abs(zoomValue - _value) > abs(zoomValue - m_zoomValue[_i - 1]))
			{
				return m_zoomValue[_i - 1];
			}
			else
			{
				return _value;
			}
		}
		_i++;
	}
}

QList<int> HikPointData::getZoomValueList()
{
	return m_zoomValue;
}

void HikPointData::setZoomInfoMap(zoomInfo info)
{
	m_zoomInfoMap.insert(info.zoomValue, info);
}

void HikPointData::setIsUseDefaultsZoom(bool isCheck)
{
	m_isUseDefaultZoomMap = isCheck;
}

void HikPointData::setZoomInfoMapDefault()
{
	m_zoomInfoDefaultMap[0] = zoomInfo{ 0, 56.90f, 33.90f };
	m_zoomInfoDefaultMap[579] = zoomInfo{ 579, 54.50f, 32.30f };
	m_zoomInfoDefaultMap[1157] = zoomInfo{ 1157, 52.30f, 30.90f };
	m_zoomInfoDefaultMap[1736] = zoomInfo{ 1736, 49.30f, 29.00f };
	m_zoomInfoDefaultMap[2314] = zoomInfo{ 2314, 46.60f, 27.30f };
	m_zoomInfoDefaultMap[2893] = zoomInfo{ 2893, 44.20f, 25.70f };
	m_zoomInfoDefaultMap[3471] = zoomInfo{ 3471, 42.00f, 24.40f };
	m_zoomInfoDefaultMap[4050] = zoomInfo{ 4050, 39.40f, 22.80f };
	m_zoomInfoDefaultMap[4628] = zoomInfo{ 4628, 37.10f, 21.40f };
	m_zoomInfoDefaultMap[5207] = zoomInfo{ 5207, 34.50f, 19.80f };
	m_zoomInfoDefaultMap[5785] = zoomInfo{ 5785, 32.30f, 18.50f };
	m_zoomInfoDefaultMap[6045] = zoomInfo{ 6045, 31.50f, 18.00f };
	m_zoomInfoDefaultMap[6304] = zoomInfo{ 6304, 30.30f, 17.30f };
	m_zoomInfoDefaultMap[6564] = zoomInfo{ 6564, 28.90f, 16.50f };
	m_zoomInfoDefaultMap[6823] = zoomInfo{ 6823, 28.20f, 16.10f };
	m_zoomInfoDefaultMap[7083] = zoomInfo{ 7083, 27.30f, 15.60f };
	m_zoomInfoDefaultMap[7343] = zoomInfo{ 7343, 26.10f, 14.90f };
	m_zoomInfoDefaultMap[7602] = zoomInfo{ 7602, 25.30f, 14.40f };
	m_zoomInfoDefaultMap[7862] = zoomInfo{ 7862, 24.10f, 13.70f };
	m_zoomInfoDefaultMap[8121] = zoomInfo{ 8121, 23.40f, 13.30f };
	m_zoomInfoDefaultMap[8381] = zoomInfo{ 8381, 22.50f, 12.80f };
	m_zoomInfoDefaultMap[8537] = zoomInfo{ 8537, 21.90f, 12.40f };
	m_zoomInfoDefaultMap[8694] = zoomInfo{ 8694, 21.40f, 12.10f };
	m_zoomInfoDefaultMap[8850] = zoomInfo{ 8850, 21.20f, 12.00f };
	m_zoomInfoDefaultMap[9006] = zoomInfo{ 9006, 20.30f, 11.50f };
	m_zoomInfoDefaultMap[9163] = zoomInfo{ 9163, 19.80f, 11.20f };
	m_zoomInfoDefaultMap[9319] = zoomInfo{ 9319, 19.20f, 10.90f };
	m_zoomInfoDefaultMap[9475] = zoomInfo{ 9475, 18.70f, 10.60f };
	m_zoomInfoDefaultMap[9631] = zoomInfo{ 9631, 18.20f, 10.30f };
	m_zoomInfoDefaultMap[9788] = zoomInfo{ 9788, 17.80f, 10.10f };
	m_zoomInfoDefaultMap[9944] = zoomInfo{ 9944, 17.30f, 9.80f };
	m_zoomInfoDefaultMap[10056] = zoomInfo{ 10056, 16.90f, 9.60f };
	m_zoomInfoDefaultMap[10168] = zoomInfo{ 10168, 16.60f, 9.40f };
	m_zoomInfoDefaultMap[10281] = zoomInfo{ 10281, 16.20f, 9.20f };
	m_zoomInfoDefaultMap[10393] = zoomInfo{ 10393, 15.90f, 9.00f };
	m_zoomInfoDefaultMap[10505] = zoomInfo{ 10505, 15.50f, 8.80f };
	m_zoomInfoDefaultMap[10617] = zoomInfo{ 10617, 15.20f, 8.60f };
	m_zoomInfoDefaultMap[10729] = zoomInfo{ 10729, 14.90f, 8.40f };
	m_zoomInfoDefaultMap[10842] = zoomInfo{ 10842, 14.60f, 8.20f };
	m_zoomInfoDefaultMap[10954] = zoomInfo{ 10954, 14.20f, 8.00f };
	m_zoomInfoDefaultMap[11066] = zoomInfo{ 11066, 14.00f, 7.90f };
	m_zoomInfoDefaultMap[11149] = zoomInfo{ 11149, 13.80f, 7.80f };
	m_zoomInfoDefaultMap[11232] = zoomInfo{ 11232, 13.50f, 7.60f };
	m_zoomInfoDefaultMap[11315] = zoomInfo{ 11315, 13.30f, 7.50f };
	m_zoomInfoDefaultMap[11398] = zoomInfo{ 11398, 13.00f, 7.30f };
	m_zoomInfoDefaultMap[11482] = zoomInfo{ 11482, 12.80f, 7.20f };
	m_zoomInfoDefaultMap[11565] = zoomInfo{ 11565, 12.60f, 7.10f };
	m_zoomInfoDefaultMap[11648] = zoomInfo{ 11648, 12.40f, 7.00f };
	m_zoomInfoDefaultMap[11731] = zoomInfo{ 11731, 12.20f, 6.90f };
	m_zoomInfoDefaultMap[11814] = zoomInfo{ 11814, 11.90f, 6.70f };
	m_zoomInfoDefaultMap[11897] = zoomInfo{ 11897, 11.80f, 6.60f };
	m_zoomInfoDefaultMap[11966] = zoomInfo{ 11966, 11.60f, 6.50f };
	m_zoomInfoDefaultMap[12036] = zoomInfo{ 12036, 11.40f, 6.40f };
	m_zoomInfoDefaultMap[12105] = zoomInfo{ 12105, 11.30f, 6.40f };
	m_zoomInfoDefaultMap[12174] = zoomInfo{ 12174, 11.10f, 6.30f };
	m_zoomInfoDefaultMap[12244] = zoomInfo{ 12244, 10.90f, 6.20f };
	m_zoomInfoDefaultMap[12313] = zoomInfo{ 12313, 10.70f, 6.00f };
	m_zoomInfoDefaultMap[12382] = zoomInfo{ 12382, 10.60f, 6.00f };
	m_zoomInfoDefaultMap[12451] = zoomInfo{ 12451, 10.40f, 5.90f };
	m_zoomInfoDefaultMap[12521] = zoomInfo{ 12521, 10.30f, 5.80f };
	m_zoomInfoDefaultMap[12590] = zoomInfo{ 12590, 10.10f, 5.70f };
	m_zoomInfoDefaultMap[12648] = zoomInfo{ 12648, 10.00f, 5.60f };
	m_zoomInfoDefaultMap[12706] = zoomInfo{ 12706, 9.90f, 5.60f };
	m_zoomInfoDefaultMap[12764] = zoomInfo{ 12764, 9.70f, 5.50f };
	m_zoomInfoDefaultMap[12822] = zoomInfo{ 12822, 9.70f, 5.40f };
	m_zoomInfoDefaultMap[12880] = zoomInfo{ 12880, 9.50f, 5.40f };
	m_zoomInfoDefaultMap[12938] = zoomInfo{ 12938, 9.40f, 5.30f };
	m_zoomInfoDefaultMap[12996] = zoomInfo{ 12996, 9.30f, 5.20f };
	m_zoomInfoDefaultMap[13054] = zoomInfo{ 13054, 9.20f, 5.20f };
	m_zoomInfoDefaultMap[13112] = zoomInfo{ 13112, 9.00f, 5.10f };
	m_zoomInfoDefaultMap[13170] = zoomInfo{ 13170, 8.90f, 5.00f };
	m_zoomInfoDefaultMap[13220] = zoomInfo{ 13220, 8.80f, 5.00f };
	m_zoomInfoDefaultMap[13271] = zoomInfo{ 13271, 8.80f, 4.90f };
	m_zoomInfoDefaultMap[13321] = zoomInfo{ 13321, 8.60f, 4.90f };
	m_zoomInfoDefaultMap[13372] = zoomInfo{ 13372, 8.60f, 4.80f };
	m_zoomInfoDefaultMap[13422] = zoomInfo{ 13422, 8.40f, 4.80f };
	m_zoomInfoDefaultMap[13472] = zoomInfo{ 13472, 8.40f, 4.70f };
	m_zoomInfoDefaultMap[13523] = zoomInfo{ 13523, 8.30f, 4.70f };
	m_zoomInfoDefaultMap[13573] = zoomInfo{ 13573, 8.20f, 4.60f };
	m_zoomInfoDefaultMap[13624] = zoomInfo{ 13624, 8.10f, 4.60f };
	m_zoomInfoDefaultMap[13674] = zoomInfo{ 13674, 8.00f, 4.50f };
	m_zoomInfoDefaultMap[13717] = zoomInfo{ 13717, 7.90f, 4.50f };
	m_zoomInfoDefaultMap[13760] = zoomInfo{ 13760, 7.80f, 4.40f };
	m_zoomInfoDefaultMap[13803] = zoomInfo{ 13803, 7.80f, 4.40f };
	m_zoomInfoDefaultMap[13846] = zoomInfo{ 13846, 7.70f, 4.30f };
	m_zoomInfoDefaultMap[13889] = zoomInfo{ 13889, 7.60f, 4.30f };
	m_zoomInfoDefaultMap[13931] = zoomInfo{ 13931, 7.60f, 4.30f };
	m_zoomInfoDefaultMap[13974] = zoomInfo{ 13974, 7.50f, 4.20f };
	m_zoomInfoDefaultMap[14017] = zoomInfo{ 14017, 7.40f, 4.20f };
	m_zoomInfoDefaultMap[14060] = zoomInfo{ 14060, 7.30f, 4.10f };
	m_zoomInfoDefaultMap[14103] = zoomInfo{ 14103, 7.30f, 4.10f };
	m_zoomInfoDefaultMap[14140] = zoomInfo{ 14140, 7.20f, 4.10f };
	m_zoomInfoDefaultMap[14176] = zoomInfo{ 14176, 7.20f, 4.00f };
	m_zoomInfoDefaultMap[14213] = zoomInfo{ 14213, 7.10f, 4.00f };
	m_zoomInfoDefaultMap[14249] = zoomInfo{ 14249, 7.00f, 4.00f };
	m_zoomInfoDefaultMap[14286] = zoomInfo{ 14286, 7.00f, 3.90f };
	m_zoomInfoDefaultMap[14322] = zoomInfo{ 14322, 6.90f, 3.90f };
	m_zoomInfoDefaultMap[14359] = zoomInfo{ 14359, 6.90f, 3.90f };
	m_zoomInfoDefaultMap[14395] = zoomInfo{ 14395, 6.80f, 3.80f };
	m_zoomInfoDefaultMap[14432] = zoomInfo{ 14432, 6.70f, 3.80f };
	m_zoomInfoDefaultMap[14468] = zoomInfo{ 14468, 6.70f, 3.80f };
	m_zoomInfoDefaultMap[14500] = zoomInfo{ 14500, 6.60f, 3.70f };
	m_zoomInfoDefaultMap[14531] = zoomInfo{ 14531, 6.60f, 3.70f };
	m_zoomInfoDefaultMap[14563] = zoomInfo{ 14563, 6.50f, 3.70f };
	m_zoomInfoDefaultMap[14594] = zoomInfo{ 14594, 6.50f, 3.60f };
	m_zoomInfoDefaultMap[14626] = zoomInfo{ 14626, 6.40f, 3.60f };
	m_zoomInfoDefaultMap[14657] = zoomInfo{ 14657, 6.40f, 3.60f };
	m_zoomInfoDefaultMap[14689] = zoomInfo{ 14689, 6.30f, 3.60f };
	m_zoomInfoDefaultMap[14720] = zoomInfo{ 14720, 6.30f, 3.50f };
	m_zoomInfoDefaultMap[14752] = zoomInfo{ 14752, 6.20f, 3.50f };
	m_zoomInfoDefaultMap[14783] = zoomInfo{ 14783, 6.20f, 3.50f };
	m_zoomInfoDefaultMap[14808] = zoomInfo{ 14808, 6.10f, 3.50f };
	m_zoomInfoDefaultMap[14833] = zoomInfo{ 14833, 6.10f, 3.40f };
	m_zoomInfoDefaultMap[14859] = zoomInfo{ 14859, 6.10f, 3.40f };
	m_zoomInfoDefaultMap[14884] = zoomInfo{ 14884, 6.00f, 3.40f };
	m_zoomInfoDefaultMap[14909] = zoomInfo{ 14909, 6.00f, 3.40f };
	m_zoomInfoDefaultMap[14934] = zoomInfo{ 14934, 6.00f, 3.40f };
	m_zoomInfoDefaultMap[14959] = zoomInfo{ 14959, 5.90f, 3.30f };
	m_zoomInfoDefaultMap[14985] = zoomInfo{ 14985, 5.90f, 3.30f };
	m_zoomInfoDefaultMap[15010] = zoomInfo{ 15010, 5.80f, 3.30f };
	m_zoomInfoDefaultMap[15035] = zoomInfo{ 15035, 5.80f, 3.30f };
	m_zoomInfoDefaultMap[15058] = zoomInfo{ 15058, 5.80f, 3.20f };
	m_zoomInfoDefaultMap[15080] = zoomInfo{ 15080, 5.70f, 3.20f };
	m_zoomInfoDefaultMap[15103] = zoomInfo{ 15103, 5.70f, 3.20f };
	m_zoomInfoDefaultMap[15126] = zoomInfo{ 15126, 5.70f, 3.20f };
	m_zoomInfoDefaultMap[15149] = zoomInfo{ 15149, 5.60f, 3.20f };
	m_zoomInfoDefaultMap[15171] = zoomInfo{ 15171, 5.60f, 3.20f };
	m_zoomInfoDefaultMap[15194] = zoomInfo{ 15194, 5.60f, 3.10f };
	m_zoomInfoDefaultMap[15217] = zoomInfo{ 15217, 5.50f, 3.10f };
	m_zoomInfoDefaultMap[15239] = zoomInfo{ 15239, 5.50f, 3.10f };
	m_zoomInfoDefaultMap[15262] = zoomInfo{ 15262, 5.50f, 3.10f };
	m_zoomInfoDefaultMap[15280] = zoomInfo{ 15280, 5.40f, 3.10f };
	m_zoomInfoDefaultMap[15297] = zoomInfo{ 15297, 5.40f, 3.00f };
	m_zoomInfoDefaultMap[15315] = zoomInfo{ 15315, 5.40f, 3.00f };
	m_zoomInfoDefaultMap[15333] = zoomInfo{ 15333, 5.40f, 3.00f };
	m_zoomInfoDefaultMap[15351] = zoomInfo{ 15351, 5.30f, 3.00f };
	m_zoomInfoDefaultMap[15368] = zoomInfo{ 15368, 5.30f, 3.00f };
	m_zoomInfoDefaultMap[15386] = zoomInfo{ 15386, 5.30f, 3.00f };
	m_zoomInfoDefaultMap[15404] = zoomInfo{ 15404, 5.20f, 3.00f };
	m_zoomInfoDefaultMap[15421] = zoomInfo{ 15421, 5.20f, 2.90f };
	m_zoomInfoDefaultMap[15439] = zoomInfo{ 15439, 5.20f, 2.90f };
	m_zoomInfoDefaultMap[15454] = zoomInfo{ 15454, 5.20f, 2.90f };
	m_zoomInfoDefaultMap[15469] = zoomInfo{ 15469, 5.10f, 2.90f };
	m_zoomInfoDefaultMap[15484] = zoomInfo{ 15484, 5.10f, 2.90f };
	m_zoomInfoDefaultMap[15499] = zoomInfo{ 15499, 5.10f, 2.90f };
	m_zoomInfoDefaultMap[15515] = zoomInfo{ 15515, 5.10f, 2.90f };
	m_zoomInfoDefaultMap[15530] = zoomInfo{ 15530, 5.00f, 2.80f };
	m_zoomInfoDefaultMap[15545] = zoomInfo{ 15545, 5.00f, 2.80f };
	m_zoomInfoDefaultMap[15560] = zoomInfo{ 15560, 5.00f, 2.80f };
	m_zoomInfoDefaultMap[15575] = zoomInfo{ 15575, 5.00f, 2.80f };
	m_zoomInfoDefaultMap[15590] = zoomInfo{ 15590, 5.00f, 2.80f };
	m_zoomInfoDefaultMap[15604] = zoomInfo{ 15604, 4.90f, 2.80f };
	m_zoomInfoDefaultMap[15618] = zoomInfo{ 15618, 4.90f, 2.80f };
	m_zoomInfoDefaultMap[15632] = zoomInfo{ 15632, 4.90f, 2.70f };
	m_zoomInfoDefaultMap[15646] = zoomInfo{ 15646, 4.90f, 2.70f };
	m_zoomInfoDefaultMap[15660] = zoomInfo{ 15660, 4.80f, 2.70f };
	m_zoomInfoDefaultMap[15673] = zoomInfo{ 15673, 4.80f, 2.70f };
	m_zoomInfoDefaultMap[15687] = zoomInfo{ 15687, 4.80f, 2.70f };
	m_zoomInfoDefaultMap[15701] = zoomInfo{ 15701, 4.80f, 2.70f };
	m_zoomInfoDefaultMap[15715] = zoomInfo{ 15715, 4.80f, 2.70f };
	m_zoomInfoDefaultMap[15729] = zoomInfo{ 15729, 4.70f, 2.70f };
	m_zoomInfoDefaultMap[15739] = zoomInfo{ 15739, 4.70f, 2.60f };
	m_zoomInfoDefaultMap[15749] = zoomInfo{ 15749, 4.70f, 2.60f };
	m_zoomInfoDefaultMap[15759] = zoomInfo{ 15759, 4.70f, 2.60f };
	m_zoomInfoDefaultMap[15769] = zoomInfo{ 15769, 4.70f, 2.60f };
	m_zoomInfoDefaultMap[15779] = zoomInfo{ 15779, 4.70f, 2.60f };
	m_zoomInfoDefaultMap[15789] = zoomInfo{ 15789, 4.60f, 2.60f };
	m_zoomInfoDefaultMap[15799] = zoomInfo{ 15799, 4.60f, 2.60f };
	m_zoomInfoDefaultMap[15809] = zoomInfo{ 15809, 4.60f, 2.60f };
	m_zoomInfoDefaultMap[15819] = zoomInfo{ 15819, 4.60f, 2.60f };
	m_zoomInfoDefaultMap[15829] = zoomInfo{ 15829, 4.60f, 2.60f };
	m_zoomInfoDefaultMap[15839] = zoomInfo{ 15839, 4.50f, 2.60f };
	m_zoomInfoDefaultMap[15849] = zoomInfo{ 15849, 4.50f, 2.50f };
	m_zoomInfoDefaultMap[15859] = zoomInfo{ 15859, 4.50f, 2.50f };
	m_zoomInfoDefaultMap[15869] = zoomInfo{ 15869, 4.50f, 2.50f };
	m_zoomInfoDefaultMap[15880] = zoomInfo{ 15880, 4.50f, 2.50f };
	m_zoomInfoDefaultMap[15890] = zoomInfo{ 15890, 4.50f, 2.50f };
	m_zoomInfoDefaultMap[15900] = zoomInfo{ 15900, 4.50f, 2.50f };
	m_zoomInfoDefaultMap[15910] = zoomInfo{ 15910, 4.40f, 2.50f };
	m_zoomInfoDefaultMap[15920] = zoomInfo{ 15920, 4.40f, 2.50f };
	m_zoomInfoDefaultMap[15930] = zoomInfo{ 15930, 4.40f, 2.50f };
	m_zoomInfoDefaultMap[15936] = zoomInfo{ 15936, 4.40f, 2.50f };
	m_zoomInfoDefaultMap[15943] = zoomInfo{ 15943, 4.40f, 2.50f };
	m_zoomInfoDefaultMap[15949] = zoomInfo{ 15949, 4.40f, 2.50f };
	m_zoomInfoDefaultMap[15955] = zoomInfo{ 15955, 4.40f, 2.50f };
	m_zoomInfoDefaultMap[15962] = zoomInfo{ 15962, 4.30f, 2.40f };
	m_zoomInfoDefaultMap[15968] = zoomInfo{ 15968, 4.30f, 2.40f };
	m_zoomInfoDefaultMap[15974] = zoomInfo{ 15974, 4.30f, 2.40f };
	m_zoomInfoDefaultMap[15980] = zoomInfo{ 15980, 4.30f, 2.40f };
	m_zoomInfoDefaultMap[15987] = zoomInfo{ 15987, 4.30f, 2.40f };
	m_zoomInfoDefaultMap[15993] = zoomInfo{ 15993, 4.30f, 2.40f };
	m_zoomInfoDefaultMap[16001] = zoomInfo{ 16001, 4.30f, 2.40f };
	m_zoomInfoDefaultMap[16008] = zoomInfo{ 16008, 4.30f, 2.40f };
	m_zoomInfoDefaultMap[16016] = zoomInfo{ 16016, 4.20f, 2.40f };
	m_zoomInfoDefaultMap[16023] = zoomInfo{ 16023, 4.20f, 2.40f };
	m_zoomInfoDefaultMap[16031] = zoomInfo{ 16031, 4.20f, 2.40f };
	m_zoomInfoDefaultMap[16039] = zoomInfo{ 16039, 4.20f, 2.40f };
	m_zoomInfoDefaultMap[16046] = zoomInfo{ 16046, 4.20f, 2.40f };
	m_zoomInfoDefaultMap[16054] = zoomInfo{ 16054, 4.20f, 2.40f };
	m_zoomInfoDefaultMap[16061] = zoomInfo{ 16061, 4.20f, 2.30f };
	m_zoomInfoDefaultMap[16069] = zoomInfo{ 16069, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16074] = zoomInfo{ 16074, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16079] = zoomInfo{ 16079, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16084] = zoomInfo{ 16084, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16089] = zoomInfo{ 16089, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16094] = zoomInfo{ 16094, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16099] = zoomInfo{ 16099, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16104] = zoomInfo{ 16104, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16109] = zoomInfo{ 16109, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16114] = zoomInfo{ 16114, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16119] = zoomInfo{ 16119, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16123] = zoomInfo{ 16123, 4.10f, 2.30f };
	m_zoomInfoDefaultMap[16127] = zoomInfo{ 16127, 4.00f, 2.30f };
	m_zoomInfoDefaultMap[16130] = zoomInfo{ 16130, 4.00f, 2.30f };
	m_zoomInfoDefaultMap[16134] = zoomInfo{ 16134, 4.00f, 2.30f };
	m_zoomInfoDefaultMap[16138] = zoomInfo{ 16138, 4.00f, 2.30f };
	m_zoomInfoDefaultMap[16142] = zoomInfo{ 16142, 4.00f, 2.30f };
	m_zoomInfoDefaultMap[16146] = zoomInfo{ 16146, 4.00f, 2.30f };
	m_zoomInfoDefaultMap[16149] = zoomInfo{ 16149, 4.00f, 2.20f };
	m_zoomInfoDefaultMap[16153] = zoomInfo{ 16153, 4.00f, 2.20f };
	m_zoomInfoDefaultMap[16157] = zoomInfo{ 16157, 4.00f, 2.20f };
	m_zoomInfoDefaultMap[16161] = zoomInfo{ 16161, 4.00f, 2.20f };
	m_zoomInfoDefaultMap[16165] = zoomInfo{ 16165, 4.00f, 2.20f };
	m_zoomInfoDefaultMap[16168] = zoomInfo{ 16168, 4.00f, 2.20f };
	m_zoomInfoDefaultMap[16172] = zoomInfo{ 16172, 4.00f, 2.20f };
	m_zoomInfoDefaultMap[16176] = zoomInfo{ 16176, 4.00f, 2.20f };
	m_zoomInfoDefaultMap[16180] = zoomInfo{ 16180, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16184] = zoomInfo{ 16184, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16187] = zoomInfo{ 16187, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16191] = zoomInfo{ 16191, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16195] = zoomInfo{ 16195, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16199] = zoomInfo{ 16199, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16203] = zoomInfo{ 16203, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16206] = zoomInfo{ 16206, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16210] = zoomInfo{ 16210, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16214] = zoomInfo{ 16214, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16218] = zoomInfo{ 16218, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16222] = zoomInfo{ 16222, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16225] = zoomInfo{ 16225, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16229] = zoomInfo{ 16229, 3.90f, 2.20f };
	m_zoomInfoDefaultMap[16233] = zoomInfo{ 16233, 3.80f, 2.20f };
	m_zoomInfoDefaultMap[16237] = zoomInfo{ 16237, 3.80f, 2.20f };
	m_zoomInfoDefaultMap[16241] = zoomInfo{ 16241, 3.80f, 2.20f };
	m_zoomInfoDefaultMap[16244] = zoomInfo{ 16244, 3.80f, 2.20f };
	m_zoomInfoDefaultMap[16248] = zoomInfo{ 16248, 3.80f, 2.10f };
	m_zoomInfoDefaultMap[16252] = zoomInfo{ 16252, 3.80f, 2.10f };
	m_zoomInfoDefaultMap[16256] = zoomInfo{ 16256, 3.80f, 2.10f };
	m_zoomInfoDefaultMap[16260] = zoomInfo{ 16260, 3.80f, 2.10f };
	m_zoomInfoDefaultMap[16263] = zoomInfo{ 16263, 3.80f, 2.10f };
	m_zoomInfoDefaultMap[16267] = zoomInfo{ 16267, 3.80f, 2.10f };
	m_zoomInfoDefaultMap[16271] = zoomInfo{ 16271, 3.80f, 2.10f };
	m_zoomInfoDefaultMap[16275] = zoomInfo{ 16275, 3.80f, 2.10f };
	m_zoomInfoDefaultMap[16278] = zoomInfo{ 16278, 3.80f, 2.10f };
	m_zoomInfoDefaultMap[16282] = zoomInfo{ 16282, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16286] = zoomInfo{ 16286, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16290] = zoomInfo{ 16290, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16293] = zoomInfo{ 16293, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16297] = zoomInfo{ 16297, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16301] = zoomInfo{ 16301, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16304] = zoomInfo{ 16304, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16308] = zoomInfo{ 16308, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16312] = zoomInfo{ 16312, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16316] = zoomInfo{ 16316, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16319] = zoomInfo{ 16319, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16323] = zoomInfo{ 16323, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16327] = zoomInfo{ 16327, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16331] = zoomInfo{ 16331, 3.70f, 2.10f };
	m_zoomInfoDefaultMap[16335] = zoomInfo{ 16335, 3.60f, 2.10f };
	m_zoomInfoDefaultMap[16338] = zoomInfo{ 16338, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16342] = zoomInfo{ 16342, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16346] = zoomInfo{ 16346, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16349] = zoomInfo{ 16349, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16351] = zoomInfo{ 16351, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16354] = zoomInfo{ 16354, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16356] = zoomInfo{ 16356, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16359] = zoomInfo{ 16359, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16361] = zoomInfo{ 16361, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16364] = zoomInfo{ 16364, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16366] = zoomInfo{ 16366, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16369] = zoomInfo{ 16369, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16371] = zoomInfo{ 16371, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16372] = zoomInfo{ 16372, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16374] = zoomInfo{ 16374, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16375] = zoomInfo{ 16375, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16376] = zoomInfo{ 16376, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16378] = zoomInfo{ 16378, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16379] = zoomInfo{ 16379, 3.60f, 2.00f };
	m_zoomInfoDefaultMap[16380] = zoomInfo{ 16380, 3.50f, 2.00f };
	m_zoomInfoDefaultMap[16381] = zoomInfo{ 16381, 3.50f, 2.00f };
	m_zoomInfoDefaultMap[16383] = zoomInfo{ 16383, 3.50f, 2.00f };
	m_zoomInfoDefaultMap[16384] = zoomInfo{ 16384, 3.50f, 2.00f };
}

void HikPointData::setZoomValueList()
{
	m_zoomValue.append(0);
	m_zoomValue.append(579);
	m_zoomValue.append(1157);
	m_zoomValue.append(1736);
	m_zoomValue.append(2314);
	m_zoomValue.append(2893);
	m_zoomValue.append(3471);
	m_zoomValue.append(4050);
	m_zoomValue.append(4628);
	m_zoomValue.append(5207);
	m_zoomValue.append(5785);
	m_zoomValue.append(6045);
	m_zoomValue.append(6304);
	m_zoomValue.append(6564);
	m_zoomValue.append(6823);
	m_zoomValue.append(7083);
	m_zoomValue.append(7343);
	m_zoomValue.append(7602);
	m_zoomValue.append(7862);
	m_zoomValue.append(8121);
	m_zoomValue.append(8381);
	m_zoomValue.append(8537);
	m_zoomValue.append(8694);
	m_zoomValue.append(8850);
	m_zoomValue.append(9006);
	m_zoomValue.append(9163);
	m_zoomValue.append(9319);
	m_zoomValue.append(9475);
	m_zoomValue.append(9631);
	m_zoomValue.append(9788);
	m_zoomValue.append(9944);
	m_zoomValue.append(10056);
	m_zoomValue.append(10168);
	m_zoomValue.append(10281);
	m_zoomValue.append(10393);
	m_zoomValue.append(10505);
	m_zoomValue.append(10617);
	m_zoomValue.append(10729);
	m_zoomValue.append(10842);
	m_zoomValue.append(10954);
	m_zoomValue.append(11066);
	m_zoomValue.append(11149);
	m_zoomValue.append(11232);
	m_zoomValue.append(11315);
	m_zoomValue.append(11398);
	m_zoomValue.append(11482);
	m_zoomValue.append(11565);
	m_zoomValue.append(11648);
	m_zoomValue.append(11731);
	m_zoomValue.append(11814);
	m_zoomValue.append(11897);
	m_zoomValue.append(11966);
	m_zoomValue.append(12036);
	m_zoomValue.append(12105);
	m_zoomValue.append(12174);
	m_zoomValue.append(12244);
	m_zoomValue.append(12313);
	m_zoomValue.append(12382);
	m_zoomValue.append(12451);
	m_zoomValue.append(12521);
	m_zoomValue.append(12590);
	m_zoomValue.append(12648);
	m_zoomValue.append(12706);
	m_zoomValue.append(12764);
	m_zoomValue.append(12822);
	m_zoomValue.append(12880);
	m_zoomValue.append(12938);
	m_zoomValue.append(12996);
	m_zoomValue.append(13054);
	m_zoomValue.append(13112);
	m_zoomValue.append(13170);
	m_zoomValue.append(13220);
	m_zoomValue.append(13271);
	m_zoomValue.append(13321);
	m_zoomValue.append(13372);
	m_zoomValue.append(13422);
	m_zoomValue.append(13472);
	m_zoomValue.append(13523);
	m_zoomValue.append(13573);
	m_zoomValue.append(13624);
	m_zoomValue.append(13674);
	m_zoomValue.append(13717);
	m_zoomValue.append(13760);
	m_zoomValue.append(13803);
	m_zoomValue.append(13846);
	m_zoomValue.append(13889);
	m_zoomValue.append(13931);
	m_zoomValue.append(13974);
	m_zoomValue.append(14017);
	m_zoomValue.append(14060);
	m_zoomValue.append(14103);
	m_zoomValue.append(14140);
	m_zoomValue.append(14176);
	m_zoomValue.append(14213);
	m_zoomValue.append(14249);
	m_zoomValue.append(14286);
	m_zoomValue.append(14322);
	m_zoomValue.append(14359);
	m_zoomValue.append(14395);
	m_zoomValue.append(14432);
	m_zoomValue.append(14468);
	m_zoomValue.append(14500);
	m_zoomValue.append(14531);
	m_zoomValue.append(14563);
	m_zoomValue.append(14594);
	m_zoomValue.append(14626);
	m_zoomValue.append(14657);
	m_zoomValue.append(14689);
	m_zoomValue.append(14720);
	m_zoomValue.append(14752);
	m_zoomValue.append(14783);
	m_zoomValue.append(14808);
	m_zoomValue.append(14833);
	m_zoomValue.append(14859);
	m_zoomValue.append(14884);
	m_zoomValue.append(14909);
	m_zoomValue.append(14934);
	m_zoomValue.append(14959);
	m_zoomValue.append(14985);
	m_zoomValue.append(15010);
	m_zoomValue.append(15035);
	m_zoomValue.append(15058);
	m_zoomValue.append(15080);
	m_zoomValue.append(15103);
	m_zoomValue.append(15126);
	m_zoomValue.append(15149);
	m_zoomValue.append(15171);
	m_zoomValue.append(15194);
	m_zoomValue.append(15217);
	m_zoomValue.append(15239);
	m_zoomValue.append(15262);
	m_zoomValue.append(15280);
	m_zoomValue.append(15297);
	m_zoomValue.append(15315);
	m_zoomValue.append(15333);
	m_zoomValue.append(15351);
	m_zoomValue.append(15368);
	m_zoomValue.append(15386);
	m_zoomValue.append(15404);
	m_zoomValue.append(15421);
	m_zoomValue.append(15439);
	m_zoomValue.append(15454);
	m_zoomValue.append(15469);
	m_zoomValue.append(15484);
	m_zoomValue.append(15499);
	m_zoomValue.append(15515);
	m_zoomValue.append(15530);
	m_zoomValue.append(15545);
	m_zoomValue.append(15560);
	m_zoomValue.append(15575);
	m_zoomValue.append(15590);
	m_zoomValue.append(15604);
	m_zoomValue.append(15618);
	m_zoomValue.append(15632);
	m_zoomValue.append(15646);
	m_zoomValue.append(15660);
	m_zoomValue.append(15673);
	m_zoomValue.append(15687);
	m_zoomValue.append(15701);
	m_zoomValue.append(15715);
	m_zoomValue.append(15729);
	m_zoomValue.append(15739);
	m_zoomValue.append(15749);
	m_zoomValue.append(15759);
	m_zoomValue.append(15769);
	m_zoomValue.append(15779);
	m_zoomValue.append(15789);
	m_zoomValue.append(15799);
	m_zoomValue.append(15809);
	m_zoomValue.append(15819);
	m_zoomValue.append(15829);
	m_zoomValue.append(15839);
	m_zoomValue.append(15849);
	m_zoomValue.append(15859);
	m_zoomValue.append(15869);
	m_zoomValue.append(15880);
	m_zoomValue.append(15890);
	m_zoomValue.append(15900);
	m_zoomValue.append(15910);
	m_zoomValue.append(15920);
	m_zoomValue.append(15930);
	m_zoomValue.append(15936);
	m_zoomValue.append(15943);
	m_zoomValue.append(15949);
	m_zoomValue.append(15955);
	m_zoomValue.append(15962);
	m_zoomValue.append(15968);
	m_zoomValue.append(15974);
	m_zoomValue.append(15980);
	m_zoomValue.append(15987);
	m_zoomValue.append(15993);
	m_zoomValue.append(16001);
	m_zoomValue.append(16008);
	m_zoomValue.append(16016);
	m_zoomValue.append(16023);
	m_zoomValue.append(16031);
	m_zoomValue.append(16039);
	m_zoomValue.append(16046);
	m_zoomValue.append(16054);
	m_zoomValue.append(16061);
	m_zoomValue.append(16069);
	m_zoomValue.append(16074);
	m_zoomValue.append(16079);
	m_zoomValue.append(16084);
	m_zoomValue.append(16089);
	m_zoomValue.append(16094);
	m_zoomValue.append(16099);
	m_zoomValue.append(16104);
	m_zoomValue.append(16109);
	m_zoomValue.append(16114);
	m_zoomValue.append(16119);
	m_zoomValue.append(16123);
	m_zoomValue.append(16127);
	m_zoomValue.append(16130);
	m_zoomValue.append(16134);
	m_zoomValue.append(16138);
	m_zoomValue.append(16142);
	m_zoomValue.append(16146);
	m_zoomValue.append(16149);
	m_zoomValue.append(16153);
	m_zoomValue.append(16157);
	m_zoomValue.append(16161);
	m_zoomValue.append(16165);
	m_zoomValue.append(16168);
	m_zoomValue.append(16172);
	m_zoomValue.append(16176);
	m_zoomValue.append(16180);
	m_zoomValue.append(16184);
	m_zoomValue.append(16187);
	m_zoomValue.append(16191);
	m_zoomValue.append(16195);
	m_zoomValue.append(16199);
	m_zoomValue.append(16203);
	m_zoomValue.append(16206);
	m_zoomValue.append(16210);
	m_zoomValue.append(16214);
	m_zoomValue.append(16218);
	m_zoomValue.append(16222);
	m_zoomValue.append(16225);
	m_zoomValue.append(16229);
	m_zoomValue.append(16233);
	m_zoomValue.append(16237);
	m_zoomValue.append(16241);
	m_zoomValue.append(16244);
	m_zoomValue.append(16248);
	m_zoomValue.append(16252);
	m_zoomValue.append(16256);
	m_zoomValue.append(16260);
	m_zoomValue.append(16263);
	m_zoomValue.append(16267);
	m_zoomValue.append(16271);
	m_zoomValue.append(16275);
	m_zoomValue.append(16278);
	m_zoomValue.append(16282);
	m_zoomValue.append(16286);
	m_zoomValue.append(16290);
	m_zoomValue.append(16293);
	m_zoomValue.append(16297);
	m_zoomValue.append(16301);
	m_zoomValue.append(16304);
	m_zoomValue.append(16308);
	m_zoomValue.append(16312);
	m_zoomValue.append(16316);
	m_zoomValue.append(16319);
	m_zoomValue.append(16323);
	m_zoomValue.append(16327);
	m_zoomValue.append(16331);
	m_zoomValue.append(16335);
	m_zoomValue.append(16338);
	m_zoomValue.append(16342);
	m_zoomValue.append(16346);
	m_zoomValue.append(16349);
	m_zoomValue.append(16351);
	m_zoomValue.append(16354);
	m_zoomValue.append(16356);
	m_zoomValue.append(16359);
	m_zoomValue.append(16361);
	m_zoomValue.append(16364);
	m_zoomValue.append(16366);
	m_zoomValue.append(16369);
	m_zoomValue.append(16371);
	m_zoomValue.append(16372);
	m_zoomValue.append(16374);
	m_zoomValue.append(16375);
	m_zoomValue.append(16376);
	m_zoomValue.append(16378);
	m_zoomValue.append(16379);
	m_zoomValue.append(16380);
	m_zoomValue.append(16381);
	m_zoomValue.append(16383);
	m_zoomValue.append(16384);
}
