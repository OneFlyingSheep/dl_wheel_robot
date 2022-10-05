#include <fstream>
#include <QDebug>
#include "MapReader.h"
#include "LibQJson/json.h"

MapReader::MapReader(QObject* parent)
{
	m_bIsOnlyLoadLm = false;
}

MapReader::~MapReader()
{

}

void MapReader::SetReadPro(const std::string& file, int type, bool isOnlyLoadLm)
{
	m_lockReadWrite.lockForWrite();
	m_strSmapFileName = file;
	m_lstTypes.push_back(type);
	m_bIsOnlyLoadLm = isOnlyLoadLm;
	m_lockReadWrite.unlock();
}

void MapReader::SetMapData(const std::string& file, MapData stMapData)
{
	m_lockReadWrite.lockForWrite();
	ClearMapData();
	m_lstTypes.push_back(WRITE_THREAD_TYPE);
	m_stMapData = stMapData;
	m_strSmapFileName = file;
	m_lockReadWrite.unlock();
}

void MapReader::run()
{
	while (m_lstTypes.size() > 0)
	{
		int iType = m_lstTypes.takeFirst();
		switch (iType)
		{
		case READ_THREAD_TYPE:
		{
			Read(m_strSmapFileName);
			emit FinishedSignals(READ_THREAD_TYPE, m_bIsOnlyLoadLm);
		}break;
		case WRITE_THREAD_TYPE:
		{
			CreateMap(m_strSmapFileName);
			emit FinishedSignals(WRITE_THREAD_TYPE, false);
		}break;
		default:
			break;
		}
	}

	
}


void MapReader::ClearMapData()
{
	m_stMapData.MapAdvanceAreaList.clear();
	m_stMapData.MapAdvancedCurveList.clear();
	m_stMapData.MapAdvancedLineList.clear();
	m_stMapData.MapAdvancedObjectDefineList.clear();
	m_stMapData.MapAdvancedPointList.clear();
	m_stMapData.MapNormalLineList.clear();
	m_stMapData.MapNormalPosList.clear();
	m_stMapData.PatrolRoutePath.clear();
}


void MapReader::Read(const std::string& file)
{
	ClearMapData();
	Json::Reader reader;
	Json::Value root;
	Json::Value pos_array;
	Json::Value line_array;
	Json::Value advanced_object_array;
	Json::Value advanced_point_array;
	Json::Value advanced_line_array;
	Json::Value advanced_curve_array;
	Json::Value advanced_area_array;
	Json::Value area_rect_array;
	Json::Value normalPosAngleArrays;

	std::string filename = file;
	std::ifstream is;
	is.open(filename.c_str(), std::ios::binary);
	
	try
	{
// 		if (m_bIsOnlyLoadLm) {
// 			if (reader.parse(is, root, false))
// 			{
// 				//-------------------advancedPointList part---------------;
// 				advanced_point_array = root["advancedPointList"];
// 				for (unsigned int i = 0; i < advanced_point_array.size(); ++i)
// 				{
// 					Json::Value advanced_point_array_property = root["advancedPointList"][i]["property"];
// 
// 					AdvancedPosition advancePosition;
// 					advancePosition.pos.pos_x = advanced_point_array[i]["pos"]["x"].asDouble();
// 					advancePosition.pos.pos_y = advanced_point_array[i]["pos"]["y"].asDouble();
// 					advancePosition.pos.pos_dir = advanced_point_array[i]["pos"]["dir"].asDouble();
// 					advancePosition.className = advanced_point_array[i]["className"].asString();
// 					advancePosition.instanceName = advanced_point_array[i]["instanceId"].asString();
// 
// 					for (unsigned int j = 0; j < advanced_point_array_property.size(); j++)
// 					{
// 						Property  _property;
// 						_property.key = advanced_point_array_property[j]["key"].asString();
// 						_property.type = advanced_point_array_property[j]["type"].asString();
// 
// 						if (advanced_point_array_property[j]["boolValue"].asBool() == true) {
// 							_property.value = "true";
// 						}
// 						else {
// 							_property.value = "false";
// 						}
// 						advancePosition.PropertyList.push_back(_property);
// 					}
// 					map_data_.MapAdvancedPointList.push_back(advancePosition);
// 				}
// 
// 				//-------------advancedCurveList part-----------------
// 				advanced_curve_array = root["advancedCurveList"];
// 				for (unsigned int i = 0; i < advanced_curve_array.size(); ++i)
// 				{
// 					Json::Value advanced_curve_array_property = root["advancedCurveList"][i]["property"];
// 					AdvancedCurve advancedCurve;
// 
// 					advancedCurve.className = advanced_curve_array[i]["className"].asString();
// 					//advancedCurve.startPos.className = advanced_curve_array[i]["startPos"]["className"].asString();
// 					advancedCurve.startPos.instanceName = advanced_curve_array[i]["startPos"]["instanceId"].asString();
// 					advancedCurve.startPos.pos_x = advanced_curve_array[i]["startPos"]["pos"]["x"].asDouble();
// 					advancedCurve.startPos.pos_y = advanced_curve_array[i]["startPos"]["pos"]["y"].asDouble();
// 					//advancedCurve.endPos.className = advanced_curve_array[i]["endPos"]["className"].asString();
// 					advancedCurve.endPos.instanceName = advanced_curve_array[i]["endPos"]["instanceId"].asString();
// 					advancedCurve.endPos.pos_x = advanced_curve_array[i]["endPos"]["pos"]["x"].asDouble();
// 					advancedCurve.endPos.pos_y = advanced_curve_array[i]["endPos"]["pos"]["y"].asDouble();
// 					advancedCurve.controlPos1.pos_x = advanced_curve_array[i]["controlPos1"]["x"].asDouble();
// 					advancedCurve.controlPos1.pos_y = advanced_curve_array[i]["controlPos1"]["y"].asDouble();
// 					advancedCurve.controlPos2.pos_x = advanced_curve_array[i]["controlPos2"]["x"].asDouble();
// 					advancedCurve.controlPos2.pos_y = advanced_curve_array[i]["controlPos2"]["y"].asDouble();
// 
// 					for (unsigned int j = 0; j < advanced_curve_array_property.size(); ++j)
// 					{
// 						Property  _property;
// 						_property.key = advanced_curve_array_property[j]["key"].asString();
// 						_property.type = advanced_curve_array_property[j]["type"].asString();
// 						if (_property.key == "weight") {
// 							double temp_value = advanced_curve_array_property[j]["doubleValue"].asDouble();
// 							_property.value = std::to_string(temp_value);
// 						}
// 						else if (_property.key == "passing") {
// 							if (advanced_curve_array_property[j]["boolValue"].asBool() == true) {
// 								_property.value = "true";
// 							}
// 							else {
// 								_property.value = "false";
// 							}
// 						}
// 						else {
// 							_property.value = advanced_curve_array_property[j]["strValue"].asString();
// 						}
// 						advancedCurve.PropertyList.push_back(_property);
// 					}
// 					map_data_.MapAdvancedCurveList.push_back(advancedCurve);
// 				}
// 
// 			}
// 			is.close();
// 		}
// 		else {
			if (reader.parse(is, root, false))
			{
				//-------------header part---------------
// 				m_stMapData.header.mapType = root["header"]["mapType"].asString();
// 				m_stMapData.header.mapName = root["header"]["mapName"].asString();
// 				m_stMapData.header.minPos.pos_x = root["header"]["minPos"]["x"].asDouble();
// 				m_stMapData.header.minPos.pos_y = root["header"]["minPos"]["y"].asDouble();
// 				m_stMapData.header.maxPos.pos_x = root["header"]["maxPos"]["x"].asDouble();
// 				m_stMapData.header.maxPos.pos_y = root["header"]["maxPos"]["y"].asDouble();
// 				m_stMapData.header.resolution = root["header"]["resolution"].asDouble();

				//-------------cut out area---------------
// 				m_stMapData.stCutoutArea.dStartX = root["cutoutarea"]["startx"].asDouble();
// 				m_stMapData.stCutoutArea.dStartY = root["cutoutarea"]["starty"].asDouble();
// 				m_stMapData.stCutoutArea.dEndX = root["cutoutarea"]["endx"].asDouble();
// 				m_stMapData.stCutoutArea.dEndY = root["cutoutarea"]["endy"].asDouble();
// 				m_stMapData.stCutoutArea.strBgPixmap = root["cutoutarea"]["bgpixmap"].asString();

				//-------------normal pos area---------------
				//m_stMapData.stNormalPosArea.dNormalPosAngle = root["NormalPosArea"]["angle"].asDouble();
				//m_stMapData.stNormalPosArea.dPosX = root["NormalPosArea"]["x"].asDouble();
			//	m_stMapData.stNormalPosArea.dPosY = root["NormalPosArea"]["y"].asDouble();

				//-----------------normalPosList part--------------------
// 				pos_array = root["normalPosList"];
// 				for (unsigned int i = 0; i < pos_array.size(); ++i)
// 				{
// 					NormalPosition normalPosition;
// 					normalPosition.pos_x = pos_array[i]["x"].asDouble();
// 					normalPosition.pos_y = pos_array[i]["y"].asDouble();
// 					m_stMapData.MapNormalPosList.push_back(normalPosition);
// 				}



				//------------------advancedObjectDefineList part---------------
				advanced_object_array = root["advancedObjectDefineList"];
				for (unsigned int i = 0; i < advanced_object_array.size(); ++i)
				{
					AdvancedDefine advancedObjDefine;
					advancedObjDefine.type = advanced_object_array[i]["type"].asString();
					advancedObjDefine.className = advanced_object_array[i]["className"].asString();
					m_stMapData.MapAdvancedObjectDefineList.push_back(advancedObjDefine);
				}


				//-------------------advancedPointList part---------------;
				advanced_point_array = root["advancedPointList"];
				for (unsigned int i = 0; i < advanced_point_array.size(); ++i)
				{
					Json::Value advanced_point_array_property = root["advancedPointList"][i]["property"];
					Json::Value advanced_point_array_isfixed = root["advancedPointList"][i]["fixed"];

					AdvancedPosition advancePosition;
					advancePosition.pos.pos_x = advanced_point_array[i]["pos"]["x"].asDouble();
					advancePosition.pos.pos_y = advanced_point_array[i]["pos"]["y"].asDouble();
					advancePosition.pos.pos_dir = advanced_point_array[i]["dir"].asDouble();
					advancePosition.className = advanced_point_array[i]["className"].asString();
					advancePosition.instanceName = advanced_point_array[i]["instanceId"].asString();

					for (unsigned int j = 0; j < advanced_point_array_property.size(); j++)
					{
						Property  _property;
						_property.key = advanced_point_array_property[j]["key"].asString();
						_property.type = advanced_point_array_property[j]["type"].asString();
						if (advanced_point_array_property[j]["boolValue"].asBool() == true) 
						{
							_property.value = "true";
						}
						else
						{
							_property.value = "false";
						}
						advancePosition.PropertyList.push_back(_property);
					}

					for (unsigned int iFixed = 0; iFixed < advanced_point_array_isfixed.size(); iFixed++)
					{
						Add_IsFixed _fixedTmp;
						_fixedTmp.key = advanced_point_array_isfixed[iFixed]["key"].asString();
						_fixedTmp.type = advanced_point_array_isfixed[iFixed]["type"].asString();
						if(advanced_point_array_isfixed[iFixed]["boolValue"].asBool() == true)
						{ 
							_fixedTmp.value = "true";
						}
						else
						{
							_fixedTmp.value = "false";
						}
						advancePosition.ISFixedList.push_back(_fixedTmp);
					}
					m_stMapData.MapAdvancedPointList.push_back(advancePosition);
				}

				//---------------------normalLineList part-----------------
				line_array = root["normalLineList"];
				for (unsigned int i = 0; i < line_array.size(); ++i)
				{
					NormalLine normalLine;
					normalLine.startPos.pos_x = line_array[i]["startPos"]["x"].asDouble();
					normalLine.startPos.pos_y = line_array[i]["startPos"]["y"].asDouble();
					normalLine.endPos.pos_x = line_array[i]["endPos"]["x"].asDouble();
					normalLine.endPos.pos_y = line_array[i]["endPos"]["y"].asDouble();
					m_stMapData.MapNormalLineList.push_back(normalLine);
				}

				//-----------------advancedLineList-------------------		
				advanced_line_array = root["advancedLineList"];
				for (unsigned int i = 0; i < advanced_line_array.size(); ++i)
				{
					AdvancedLine advancedLine;
					advancedLine.className = advanced_line_array[i]["className"].asString();
					advancedLine.instanceName = advanced_line_array[i]["instanceId"].asString();
					advancedLine.normalLine.startPos.pos_x = advanced_line_array[i]["line"]["startPos"]["x"].asDouble();
					advancedLine.normalLine.startPos.pos_y = advanced_line_array[i]["line"]["startPos"]["y"].asDouble();
					advancedLine.normalLine.endPos.pos_x = advanced_line_array[i]["line"]["endPos"]["x"].asDouble();
					advancedLine.normalLine.endPos.pos_y = advanced_line_array[i]["line"]["endPos"]["y"].asDouble();
					m_stMapData.MapAdvancedLineList.push_back(advancedLine);
				}

				//-------------advancedCurveList part-----------------
				advanced_curve_array = root["advancedCurveList"];
				for (unsigned int i = 0; i < advanced_curve_array.size(); ++i)
				{
					Json::Value advanced_curve_array_property = root["advancedCurveList"][i]["property"];
					AdvancedCurve advancedCurve;

					advancedCurve.className = advanced_curve_array[i]["className"].asString();
					//advancedCurve.startPos.className = advanced_curve_array[i]["startPos"]["className"].asString();
					advancedCurve.startPos.instanceName = advanced_curve_array[i]["startPos"]["instanceId"].asString();
					advancedCurve.startPos.pos_x = advanced_curve_array[i]["startPos"]["pos"]["x"].asDouble();
					advancedCurve.startPos.pos_y = advanced_curve_array[i]["startPos"]["pos"]["y"].asDouble();
					//advancedCurve.endPos.className = advanced_curve_array[i]["endPos"]["className"].asString();
					advancedCurve.endPos.instanceName = advanced_curve_array[i]["endPos"]["instanceId"].asString();
					advancedCurve.endPos.pos_x = advanced_curve_array[i]["endPos"]["pos"]["x"].asDouble();
					advancedCurve.endPos.pos_y = advanced_curve_array[i]["endPos"]["pos"]["y"].asDouble();
// 					advancedCurve.controlPos1.pos_x = advanced_curve_array[i]["controlPos1"]["x"].asDouble();
// 					advancedCurve.controlPos1.pos_y = advanced_curve_array[i]["controlPos1"]["y"].asDouble();
// 					advancedCurve.controlPos2.pos_x = advanced_curve_array[i]["controlPos2"]["x"].asDouble();
// 					advancedCurve.controlPos2.pos_y = advanced_curve_array[i]["controlPos2"]["y"].asDouble();

					for (unsigned int j = 0; j < advanced_curve_array_property.size(); ++j)
					{
						Property  _property;
						_property.key = advanced_curve_array_property[j]["key"].asString();
						_property.type = advanced_curve_array_property[j]["type"].asString();
						if (_property.key == "weight" || _property.key == "blockdist") {
							double temp_value = advanced_curve_array_property[j]["doubleValue"].asDouble();
							_property.value = std::to_string(temp_value);
						}
						else if (_property.key == "passing" || _property.key == "allowspin" || _property.key == "advancearea" || \
								 _property.key == "ultrasonic" || _property.key == "fallarrest")
						{
							if (advanced_curve_array_property[j]["boolValue"].asBool() == true) 
							{
								_property.value = "true";
							}
							else 
							{
								_property.value = "false";
							}
						}
						else 
						{
							_property.value = advanced_curve_array_property[j]["strValue"].asString();
						}
						advancedCurve.PropertyList.push_back(_property);
					}
					m_stMapData.MapAdvancedCurveList.push_back(advancedCurve);
				}

				//-----------------advanceAreaList--------------------
				advanced_area_array = root["advancedAreaList"];
				for (unsigned int i = 0; i < advanced_area_array.size(); ++i)
				{
					AdvancedArea advancedArea;
					Json::Value advanced_area_array_posGroup = root["advancedAreaList"][i]["posGroup"];
					Json::Value advanced_area_array_property = root["advancedAreaList"][i]["property"];

					advancedArea.className = advanced_area_array[i]["className"].asString();
					advancedArea.instanceName = advanced_area_array[i]["instanceId"].asString();

					for (unsigned int j = 0; j < advanced_area_array_posGroup.size(); ++j)
					{
						NormalPosition normalPosition;
						normalPosition.pos_x = advanced_area_array_posGroup[j]["x"].asDouble();
						normalPosition.pos_y = advanced_area_array_posGroup[j]["y"].asDouble();
						advancedArea.posGroup.push_back(normalPosition);
					}

					for (unsigned int k = 0; k < advanced_area_array_property.size(); ++k)
					{
						Property  _property;
						_property.key = advanced_area_array_property[k]["key"].asString();
						_property.type = advanced_area_array_property[k]["type"].asString();
						if (advanced_area_array_property[k]["boolValue"].asBool() == true) {
							_property.value = "true";
						}
						else {
							_property.value = "false";
						}
						advancedArea.PropertyList.push_back(_property);
					}
					m_stMapData.MapAdvanceAreaList.push_back(advancedArea);
				}

				//-----------------DeviceAreaList--------------------
				area_rect_array = root["DeviceAreaList"];
				for (unsigned int i = 0; i < area_rect_array.size(); ++i)
				{
					AreaRect areaRect;

					areaRect.className = area_rect_array[i]["className"].asString();
					areaRect.instanceName = area_rect_array[i]["instanceName"].asString();
					areaRect.centerPos.pos_x = area_rect_array[i]["centerPos"]["x"].asDouble();
					areaRect.centerPos.pos_y = area_rect_array[i]["centerPos"]["y"].asDouble();
					areaRect.width = area_rect_array[i]["width"].asDouble();
					areaRect.height = area_rect_array[i]["height"].asDouble();

					m_stMapData.MapAreaRectList.push_back(areaRect);
				}
			}
			is.close();
		//}
	}
	catch (const std::exception & err)
	{
		is.close();
	}
}

void MapReader::CreateMap(const std::string& fileName)
{

	Json::Value rootNew;

	//---------------header-----------------
// 	rootNew["header"]["mapType"] = m_stMapData.header.mapType;
// 	rootNew["header"]["mapName"] = m_stMapData.header.mapName;
// 	rootNew["header"]["minPos"]["x"] = m_stMapData.header.minPos.pos_x;
// 	rootNew["header"]["minPos"]["y"] = m_stMapData.header.minPos.pos_y;
// 	rootNew["header"]["maxPos"]["x"] = m_stMapData.header.maxPos.pos_x;
// 	rootNew["header"]["maxPos"]["y"] = m_stMapData.header.maxPos.pos_y;
// 	rootNew["header"]["resolution"] = m_stMapData.header.resolution;

	//---------------cut out area-----------------
// 	rootNew["cutoutarea"]["startx"] = m_stMapData.stCutoutArea.dStartX;
// 	rootNew["cutoutarea"]["starty"] = m_stMapData.stCutoutArea.dStartY;
// 	rootNew["cutoutarea"]["endx"] = m_stMapData.stCutoutArea.dEndX;
// 	rootNew["cutoutarea"]["endy"] = m_stMapData.stCutoutArea.dEndY;
// 	rootNew["cutoutarea"]["bgpixmap"] = m_stMapData.stCutoutArea.strBgPixmap;

	//---------------normal pos area-----------------
// 	rootNew["NormalPosArea"]["angle"] = m_stMapData.stNormalPosArea.dNormalPosAngle;
// 	rootNew["NormalPosArea"]["x"] = m_stMapData.stNormalPosArea.dPosX;
// 	rootNew["NormalPosArea"]["y"] = m_stMapData.stNormalPosArea.dPosY;

	//------------DeviceAreaList---------
	std::list<AreaRect>::iterator ARIt;
	for (ARIt = m_stMapData.MapAreaRectList.begin(); ARIt != m_stMapData.MapAreaRectList.end(); ++ARIt)
	{
		Json::Value advancedAreaItem;
		advancedAreaItem["className"] = ARIt->className;
		advancedAreaItem["instanceId"] = ARIt->instanceName;
		advancedAreaItem["centerPos"]["x"] = ARIt->centerPos.pos_x;
		advancedAreaItem["centerPos"]["y"] = ARIt->centerPos.pos_y; ;
		advancedAreaItem["width"] = ARIt->width;
		advancedAreaItem["height"] = ARIt->height;

		rootNew["DeviceAreaList"].append(advancedAreaItem);
	}

	//--------------normalPosList---------
	std::list<NormalPosition>::iterator NPIt;
	for (NPIt = m_stMapData.MapNormalPosList.begin(); NPIt != m_stMapData.MapNormalPosList.end(); ++NPIt)
	{
		Json::Value normalPosItem;
		normalPosItem["x"] = NPIt->pos_x;
		normalPosItem["y"] = NPIt->pos_y;
		rootNew["normalPosList"].append(normalPosItem);
	}

	//----------normalLineList---------
	std::list<NormalLine>::iterator NLIt;
	for (NLIt = m_stMapData.MapNormalLineList.begin(); NLIt != m_stMapData.MapNormalLineList.end(); ++NLIt)

	{
		Json::Value normalLineItem;
		normalLineItem["startPos"]["x"] = NLIt->startPos.pos_x;
		normalLineItem["startPos"]["y"] = NLIt->startPos.pos_y;
		normalLineItem["endPos"]["x"] = NLIt->endPos.pos_x;
		normalLineItem["endPos"]["y"] = NLIt->endPos.pos_y;
		rootNew["normalLineList"].append(normalLineItem);
	}


	//-----------advancedObjectDefineList---------
	std::list<AdvancedDefine>::iterator PIt;
	for (PIt = m_stMapData.MapAdvancedObjectDefineList.begin(); PIt != m_stMapData.MapAdvancedObjectDefineList.end(); ++PIt)
	{
		Json::Value advanceDefineItem;
		advanceDefineItem["type"] = PIt->type;
		advanceDefineItem["className"] = PIt->className;
		rootNew["advancedObjectDefineList"].append(advanceDefineItem);
	}


	//------------advancedPointList---------
	std::list<AdvancedPosition>::iterator APIt;
	std::vector<Property>::iterator APPIt;
	std::vector<Add_IsFixed>::iterator nIsFixedIt;
	for (APIt = m_stMapData.MapAdvancedPointList.begin(); APIt != m_stMapData.MapAdvancedPointList.end(); ++APIt)
	{
		Json::Value advancePointItem;
		advancePointItem["className"] = APIt->className;
		advancePointItem["instanceId"] = APIt->instanceName;
		advancePointItem["dir"] = APIt->pos.pos_dir;
		advancePointItem["pos"]["x"] = APIt->pos.pos_x;
		advancePointItem["pos"]["y"] = APIt->pos.pos_y;

		for (APPIt = APIt->PropertyList.begin(); APPIt != APIt->PropertyList.end(); ++APPIt)
		{
			Json::Value properItem;
			properItem["key"] = APPIt->key;
			properItem["type"] = APPIt->type;
			if (APPIt->value == "true") {
				properItem["boolValue"] = true;
			}
			else {
				properItem["boolValue"] = false;
			}
			advancePointItem["property"].append(properItem);
		}
		for(nIsFixedIt = APIt->ISFixedList.begin(); nIsFixedIt != APIt->ISFixedList.end(); nIsFixedIt ++)
		{
			Json::Value isFixedItem;
			isFixedItem["key"] = nIsFixedIt->key;
			isFixedItem["type"] = nIsFixedIt->type;
			if (nIsFixedIt->value == "true")
			{
				isFixedItem["boolValue"] = true;
			}
			else
			{
				isFixedItem["boolValue"] = false;
			}
			advancePointItem["fixed"].append(isFixedItem);
		}

		rootNew["advancedPointList"].append(advancePointItem);
	}


	//-----------advancedLineList---------
	std::list<AdvancedLine>::iterator ALIt;
	for (ALIt = m_stMapData.MapAdvancedLineList.begin(); ALIt != m_stMapData.MapAdvancedLineList.end(); ++ALIt)
	{
		Json::Value advancedLineItem;
		advancedLineItem["className"] = ALIt->className;
		advancedLineItem["instanceId"] = ALIt->instanceName;
		advancedLineItem["line"]["startPos"]["x"] = ALIt->normalLine.startPos.pos_x;
		advancedLineItem["line"]["startPos"]["y"] = ALIt->normalLine.startPos.pos_y;
		advancedLineItem["line"]["endPos"]["x"] = ALIt->normalLine.endPos.pos_x;
		advancedLineItem["line"]["endPos"]["y"] = ALIt->normalLine.endPos.pos_y;
		rootNew["advancedLineList"].append(advancedLineItem);
	}

	//----------------advancedCurveList---------
	std::list<AdvancedCurve>::iterator ACIt;
	std::vector<Property>::iterator ACPIt;
	for (ACIt = m_stMapData.MapAdvancedCurveList.begin(); ACIt != m_stMapData.MapAdvancedCurveList.end(); ++ACIt)
	{
		Json::Value advancedCurveItem;
		advancedCurveItem["className"] = ACIt->className;
		//advancedCurveItem["instanceId"] = ACIt->instanceName;
		//advancedCurveItem["startPos"]["className"] = ACIt->startPos.className;
		advancedCurveItem["startPos"]["instanceId"] = ACIt->startPos.instanceName;
		advancedCurveItem["startPos"]["pos"]["x"] = ACIt->startPos.pos_x;
		advancedCurveItem["startPos"]["pos"]["y"] = ACIt->startPos.pos_y;
		//advancedCurveItem["endPos"]["className"] = ACIt->endPos.className;
		advancedCurveItem["endPos"]["instanceId"] = ACIt->endPos.instanceName;
		advancedCurveItem["endPos"]["pos"]["x"] = ACIt->endPos.pos_x;
		advancedCurveItem["endPos"]["pos"]["y"] = ACIt->endPos.pos_y;
// 		advancedCurveItem["controlPos1"]["x"] = ACIt->controlPos1.pos_x;
// 		advancedCurveItem["controlPos1"]["y"] = ACIt->controlPos1.pos_y;
// 		advancedCurveItem["controlPos2"]["x"] = ACIt->controlPos2.pos_x;
// 		advancedCurveItem["controlPos2"]["y"] = ACIt->controlPos2.pos_y;

		for (ACPIt = ACIt->PropertyList.begin(); ACPIt != ACIt->PropertyList.end(); ++ACPIt)
		{
			Json::Value properItem;;
			properItem["key"] = ACPIt->key;
			properItem["type"] = ACPIt->type;
			
			if (ACPIt->key == "weight" || ACPIt->key == "blockdist") 
			{
				properItem["doubleValue"] = std::stod(ACPIt->value);
			}
			else if(ACPIt->key == "passing" || ACPIt->key == "allowspin" || ACPIt->key == "advancearea" || ACPIt->key == "ultrasonic" || ACPIt->key == "fallarrest"){
				if (ACPIt->value == "true") {
					properItem["boolValue"] = true;
				}
				else {
					properItem["boolValue"] = false;
				}
			}
			else {
				properItem["strValue"] = ACPIt->value;
			}
			advancedCurveItem["property"].append(properItem);
		}
		rootNew["advancedCurveList"].append(advancedCurveItem);
	}


	//------------advancedAreaList---------
	std::list<AdvancedArea>::iterator AAIt;
	std::vector<NormalPosition>::iterator AANIt;
	std::vector<Property>::iterator AAPIt;
	for (AAIt = m_stMapData.MapAdvanceAreaList.begin(); AAIt != m_stMapData.MapAdvanceAreaList.end(); ++AAIt)
	{
		Json::Value advancedAreaItem;
		advancedAreaItem["className"] = AAIt->className;
		advancedAreaItem["instanceId"] = AAIt->instanceName;
		for (AANIt = AAIt->posGroup.begin(); AANIt != AAIt->posGroup.end(); ++AANIt)
		{
			Json::Value posGroupItem;
			posGroupItem["x"] = AANIt->pos_x;
			posGroupItem["y"] = AANIt->pos_y;
			advancedAreaItem["posGroup"].append(posGroupItem);
		}
		for (AAPIt = AAIt->PropertyList.begin(); AAPIt != AAIt->PropertyList.end(); ++AAPIt)
		{
			Json::Value propertyItem;
			propertyItem["key"] = AAPIt->key;
			propertyItem["type"] = AAPIt->type;
			if (AAPIt->value == "true") {
				propertyItem["boolValue"] = true;
			}
			else {
				propertyItem["boolValue"] = false;
			}
			advancedAreaItem["property"].append(propertyItem);
		}
		rootNew["advancedAreaList"].append(advancedAreaItem);
	}

	//qDebug() << "create map:" << fileName.c_str() << __LINE__;

	std::ofstream fw;
	fw.open(fileName.c_str(), std::ios::ate | std::ios::out);
	if (!fw)
	{
		fw.close();
		return;
	}
// 	std::string tmp_string = rootNew.toStyledString();
// 	std::string str_value;
// 	for (unsigned int i = 0; i < tmp_string.size(); ++i)
// 	{
// 		if (tmp_string[i] == 0x0a || tmp_string[i] == 0x00)
// 		{
// 			continue;
// 		}
// 		str_value.push_back(tmp_string[i]);
// 	}
// 	fw << str_value;
	Json::StyledStreamWriter writer;
	writer.write(fw, rootNew);
	fw.close();
}


MapData& MapReader::GetData()
{
	return m_stMapData;
}


