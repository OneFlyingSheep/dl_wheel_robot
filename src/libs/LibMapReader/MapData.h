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
	double dStartX;						//���о��ε�����
	double dStartY;
	double dEndX;
	double dEndY;
	std::string strBgPixmap;			//���о��εı���ͼƬ
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
	//CutoutArea stCutoutArea;									//���е�����
	NormalPosArea stNormalPosArea;								//��ͨ�㱳������ͳߴ�
	std::list< NormalPosition > MapNormalPosList;				//��ͨ�㣨�����㣩
	std::list< NormalLine > MapNormalLineList;
	std::list< AdvancedDefine > MapAdvancedObjectDefineList;
	std::list< AdvancedPosition > MapAdvancedPointList;			//Ѳ���
	std::list< AdvancedLine > MapAdvancedLineList;
	std::list< AdvancedCurve > MapAdvancedCurveList;			//����������
	std::list< AdvancedArea > MapAdvanceAreaList;
	std::list< PatrolRoutePath > PatrolRoutePath;
	std::list< AreaRect> MapAreaRectList;						//��������ķ�Χ
};


struct BackStageMapData
{//��¼��ͼ��������Ϣ
	Header header;
	std::list< NormalLine > MapNormalLineList;			//��ͨ����
	std::list< AreaRect> MapAreaRectList;				//����
	std::list< PatrolRoutePath > PatrolRoutePath;
	std::list< AdvancedPosition > MapAdvancedPointList;		//��׼��
	std::list< AdvancedCurve > MapAdvancedCurveList;		//����

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
