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
	void SetWorkPath(std::vector<QPoint> work_paths, std::vector<int> patrol_points, bool is_work);				//���ù�����·�� �Ƿ�ʼ����

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
	bool m_bIsWork;												//��¼�Ƿ�ʼ����
	int patrol_point_index_;
	int path_width_;
	int currentPath_start_index_;
	int currentPath_end_index_;
	double percentage_;
	std::map<int, QGraphicsItem*> m_lstBeziers;					//��¼���е�Ѳ��·��
	
	std::vector<QPoint> m_vFinishedPath;						//��ɵ���
	std::vector<QPoint> m_vUnfinishedPath;						//δ��ɵ���
	std::vector<QPoint> m_lstInvalidLines;						//��¼Ѳ��·�� ��ʼ�������źͽ�����������
	std::vector<QPoint> current_list_;
	std::vector<QPoint> m_lstWorkPath;							//��¼����Ѳ���˳���磺1-2�� 2-3
	std::vector<int> m_vPatrolPoints;							//��¼����Ѳ���� �磺1��2��3
	
	QRectF update_rect_;
	QPainterPath finished_bezier_path_;
	QPainterPath unfinished_bezier_path_;
	QPainterPath invalid_bezier_path_;
	QPainterPath current_bezier_path_;
	QList<QPainterPath> m_lstFinishedBezierPaths;					//��ɵ�path
	QList<QPainterPath> m_lstUnfinishedBezierPaths;					//δ��ɵ�path
	QList<QPainterPath> m_lstInvalidBezierPaths;					//ȫ����paths
	QList<QPainterPath> m_lstCurrentBezierPaths;					//��ǰ��path

	int m_iBeginIndex{0};					//��¼�ߵ�Ѳ����λ��
};


#endif