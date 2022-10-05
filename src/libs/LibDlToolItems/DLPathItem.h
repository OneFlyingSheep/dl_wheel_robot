#ifndef DLPATHITEM_ALEXWEI_20180420_H
#define DLPATHITEM_ALEXWEI_20180420_H

#include <QGraphicsObject>
#include <map>
#include <vector>

class DLPathItem : public QGraphicsObject
{
	Q_OBJECT

public:
	DLPathItem();
	~DLPathItem() {}

	void initUpdateRect(const QRectF &rect);
	void initPath(std::map<int, QGraphicsItem*> &bezier_list);
	void UpdatePath(int index_1, int index_2, double percentage, int patrol_index);
	void SetWorkPath(std::vector<QPoint> work_paths, std::vector<int> patrol_points, bool is_work);				//设置工作的路径 是否开始工作

protected:
	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *, const QStyleOptionGraphicsItem *, QWidget *);


private:
	QPainterPath cal_bezier_path(QGraphicsItem *bezier_item, int start_index, int end_index);
	std::vector<QPoint> cal_unfinished_path(int start_index, int end_index);
	std::vector<QPoint> cal_finished_path(int start_index, int end_index);
	QGraphicsItem* find_bezier(int start_id, int end_id);

private:
	bool m_bIsWork;												//记录是否开始任务
	int patrol_point_index_;
	int path_width_;
	int currentPath_start_index_;
	int currentPath_end_index_;
	double percentage_;
	std::map<int, QGraphicsItem*> m_lstBeziers;					//记录所有的巡检路径
	
	std::vector<QPoint> m_vFinishedPath;						//完成的线
	std::vector<QPoint> m_vUnfinishedPath;						//未完成的线
	std::vector<QPoint> m_lstInvalidLines;						//记录巡检路径 开始的索引号和结束的索引号
	std::vector<QPoint> current_list_;
	std::vector<QPoint> m_lstWorkPath;							//记录任务巡检的顺序如：1-2， 2-3
	std::vector<int> m_vPatrolPoints;							//记录任务巡检点号 如：1，2，3
	
	QRectF update_rect_;
	QPainterPath finished_bezier_path_;
	QPainterPath unfinished_bezier_path_;
	QPainterPath invalid_bezier_path_;
	QPainterPath current_bezier_path_;
	QList<QPainterPath> m_lstFinishedBezierPaths;					//完成的path
	QList<QPainterPath> m_lstUnfinishedBezierPaths;					//未完成的path
	QList<QPainterPath> m_lstInvalidBezierPaths;					//全部的paths
	QList<QPainterPath> m_lstCurrentBezierPaths;					//当前的path

	int m_iBeginIndex{0};					//记录走到巡检点的位置
};


#endif