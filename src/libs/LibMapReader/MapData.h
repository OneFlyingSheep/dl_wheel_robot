#ifndef MAPDATA_ALEXWEI_20180424_H
#define MAPDATA_ALEXWEI_20180424_H

#include <string>
#include <vector>
#include <list>
#include <QPointF>
#include <QVector>

struct Property {
	std::string key;
	std::string type;
	std::string value;

	Property(std::string _key = "", std::string _type = "", std::string _value = "") :
		key(_key), type(_type), value(_value) {}
};

struct Add_IsFixed
{
	std::string key;
	std::string type;
	std::string value;

	Add_IsFixed(std::string _key = "", std::string _type = "", std::string _value = "") :
		key(_key), type(_type), value(_value) {}
};

struct AdvancedDefine {
	std::string type;
	std::string className;
};

struct NormalPosition {
	double pos_x;
	double pos_y;
	double pos_dir;
	std::string instanceName;
	std::string className;
};

struct Header {
	std::string mapType;
	std::string mapName;
	NormalPosition minPos;
	NormalPosition maxPos;
	double resolution;
	std::string version;
};

struct NormalPosArea
{
	NormalPosArea()
	{
		dNormalPosAngle = 0.0;
		dPosX = 0.0;
		dPosY = 0.0;
	}
	double dNormalPosAngle;
	double dPosX;
	double dPosY;
};

struct CutoutArea
{
	CutoutArea()
	{
		dStartX = 0.0;
		dStartY = 0.0;
		dEndX = 0.0;
		dEndY = 0.0;
		strBgPixmap = "";
	}
	double dStartX;						//剪切矩形的坐标
	double dStartY;
	double dEndX;
	double dEndY;
	std::string strBgPixmap;			//剪切矩形的背景图片
};

struct AdvancedPosition {
	NormalPosition pos;
	std::vector<Property> PropertyList;
	std::vector<Add_IsFixed> ISFixedList;
	std::string instanceName;
	std::string className;
};

struct NormalLine {
	NormalPosition startPos;
	NormalPosition endPos;
};

struct AdvancedLine {
	std::string instanceName;
	std::string className;
	NormalLine normalLine;
};

struct AdvancedCurve {
	std::string className;
	std::string instanceName;
	NormalPosition startPos;
	NormalPosition endPos;
	NormalPosition controlPos1;
	NormalPosition controlPos2;
	std::vector<Property> PropertyList;
};

struct AreaRect {
	std::string className;
	std::string instanceName;
	NormalPosition centerPos;
	double width;
	double height;
};

struct AdvancedArea {
	std::string className;
	std::string instanceName;
	std::vector<NormalPosition> posGroup;
	std::vector<Property> PropertyList;
};

struct PatrolRoutePath
{
	std::string patrol_path_name_;
	std::vector<std::string> patrol_station_vec_;
};

struct MapData 
{
	Header header;
	//CutoutArea stCutoutArea;									//剪切的区域
	NormalPosArea stNormalPosArea;								//普通点背景坐标和尺寸
	std::list< NormalPosition > MapNormalPosList;				//普通点（背景点）
	std::list< NormalLine > MapNormalLineList;
	std::list< AdvancedDefine > MapAdvancedObjectDefineList;
	std::list< AdvancedPosition > MapAdvancedPointList;			//巡检点
	std::list< AdvancedLine > MapAdvancedLineList;
	std::list< AdvancedCurve > MapAdvancedCurveList;			//贝塞尔曲线
	std::list< AdvancedArea > MapAdvanceAreaList;
	std::list< PatrolRoutePath > PatrolRoutePath;
	std::list< AreaRect> MapAreaRectList;						//整个区域的范围
};


struct BackStageMapData
{//记录地图的所有信息
	Header header;
	std::list< NormalLine > MapNormalLineList;			//普通的线
	std::list< AreaRect> MapAreaRectList;				//扇形
	std::list< PatrolRoutePath > PatrolRoutePath;
	std::list< AdvancedPosition > MapAdvancedPointList;		//基准点
	std::list< AdvancedCurve > MapAdvancedCurveList;		//曲线

};

struct AreaInfo
{
	double dMinX;
	double dMaxX;
	double dMinY;
	double dMaxY;
	QVector<QPointF> vPoints;
};


#endif // !MAPDATA_ALEXWEI_20180424_H
