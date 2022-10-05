#ifndef HSHAPE_ALEXWEI_H
#define HSHAPE_ALEXWEI_H

#include <vector>
#include "LibMapReader/MapData.h"

struct Landmark {
	int i;
};

namespace HShape{

	enum ItemType
	{
		DEVICEITEM = 100,
		ARROWITEM = 101,
		BEZIERITEM = 102,					//巡检路径
		COORDINATEITEM = 103,
		CROSSITEM = 104,
		LANDMARKITEM = 105,					//巡检点
		RECTITEM = 106,
		MUTILRECTITEM = 107,
		MUTILPOINT = 108,
		PIXMAPITEM = 109,
		DEVICEAREAITEM = 110,				//绘制图片的
		SEGMENTITEM = 111,
		POINTITEM = 112,
		ROBOTITEM = 113,
		POLYGON = 114,
		ADVANCEDAREA = 115,
		BEZIERCONTROLPTITEM = 116				//贝塞尔曲线的控制点
	};


	struct Point{
	public:
		double x_;
		double y_;
	};


	struct Circle
	{
	public:
		Point center_;
		double radius_;

	};

	struct Rectangle
	{
	public:
		Point center_;
		double width_;
		double height_;
	};

	struct RectangleAffine
	{
	public:
		Rectangle rect_;
		double angle_;
		double slope_;
	};

	struct Ellipse{
	public:
		Point center_;
		double width_;
		double height_;
		double angle_;//degree
	};


	struct Segment
	{
	public:
		Point start_;
		Point end_;
	};

	struct MultiSegment{
	public:
		std::vector<Point> points_;//size must > 2
	};

	struct Line
	{
	public:
		Point center_;
		double angle_;//degree

	};

	struct Polygon{
	public:
		std::vector<Point> points_;
	};

	struct Bezier{
	public:
		Point start_;
		Point c1_;
		Point c2_;
		Point end_;
	};

	struct Arc{
	public:
		Point center_;
		double radius_;
		double start_angle_;
		double span_angle_;
	};


	struct EArc{
	public:
		Point center_;
		double width_;
		double height_;
		double start_angle_;
		double span_angle_;

	};

	struct ArcCaliper{
	public:
		Arc arc_;
		unsigned int caplier_number_;
		double search_length_;
		double projection_length_;
		bool direction_;

	};


	struct SegmentCaliper{
	public:
		Segment segment_;
		unsigned int caliper_number_;
		double serach_length_;
		double projection_length_;
		bool direction_;
	};

	struct CircleAnnulusSection{
	public:
		Circle circle_;
		double scale_;
		double start_angle_;
		double span_anle_;
	};

	struct EllipticalAnnulusSection{
		Ellipse ellipse_;
		double scale_;
		double start_angle_;
		double span_anle_;
	};

	struct Axis{
		Point orgin_;
		double axisx_;//x轴长度
		double axisy_;//y轴长度
		double arrow_size_;//箭头大小
	};

	struct Cross{
		Point center_;
		double width_;
		double height_;
	};

	struct Arrow{
		Point start_;
		Point end_;
		double width_;
		double arrow_size_;
	};


}

#endif


