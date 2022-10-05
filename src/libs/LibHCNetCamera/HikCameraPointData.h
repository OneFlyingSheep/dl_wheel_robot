#pragma once
#include <QMap>
#include <QList>

struct zoomInfo
{
	int zoomValue;
	float levelAangle;
	float verticalAngle;
};

class HikPointData
{
public:
	HikPointData();
	~HikPointData() {}

public:
	static HikPointData* getInitance();
	void init();
	//∂¡»°txt
	void readZoomPtzValueFromTxt(QString filePath);
	//∑µªÿ
    zoomInfo getZoomInfoData(int zoomValue);
	bool getIsUseDefaultZoomMap()
	{
		return m_isUseDefaultZoomMap;
	}
	int getZoomValue(int zoomValue);
	QList<int> getZoomValueList();
	void setZoomValueList();

	void setZoomInfoMap(zoomInfo info);
	void setIsUseDefaultsZoom(bool isCheck);
	void setZoomInfoMapDefault();

private:
	QMap<int, zoomInfo> m_zoomInfoMap;
	QMap<int, zoomInfo> m_zoomInfoDefaultMap;
 	QList<int> m_zoomValue;
	bool m_isUseDefaultZoomMap = true;

	static HikPointData* m_HikObject;
};