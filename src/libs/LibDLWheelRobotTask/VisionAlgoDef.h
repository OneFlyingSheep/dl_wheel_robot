#pragma once

#include <QVector>
#include <QPoint>

enum VisionAlgoType
{
	Robot_meter_recog_req	 = 1000,
	Robot_face_recog_req	 = 1001,
	Robot_fire_recog_req	 = 1002,
	Robot_firehose_recog_req = 1003
};

struct CommonMsg
{
	int		state_;
	int		err_code_;
	std::string err_msg_;
};

struct MeterResultItem
{
	int		meter_id_;
	double	value_;
};

struct MeterResults
{
	QVector<MeterResultItem> results_;
	CommonMsg ret_msg_;
};

struct FaceResultItem
{
	int		face_id_;
	QPoint	top_left_;
	QPoint	bottom_right_;
	double	confidence_;
};

struct FaceResults
{
	QVector<FaceResultItem> results_;
	CommonMsg ret_msg_;
};

struct FireRecogParam
{
	double distance_a_;
	double distance_b_;
	double AngleA_;
	double AngleB_;
	double AngleA3_;
	double AngleB3_;
	double distance_;
	double AngleA2_;
	double AngleB2_;
	QString other_info_;
};

struct FireResults
{
	double pos_x_;
	double pos_y_;
	double pos_z_;
	CommonMsg ret_msg_;
};

struct FireHoseResultItem
{
	int		tag_id_;
	double	pos_x_;
	double	pos_y_;
	double	pos_z_;
	double	yaw_;
	double	pitch_;
	double	roll_;
};

struct FireHoseResults
{
	QVector<FireHoseResultItem> results_;
	CommonMsg ret_msg_;
};