#if _MSC_VER >= 1600  
#pragma execution_character_set("utf-8")  
#endif 

#ifndef QWORD_H
#define QWORD_H

#include <QObject>
#include <QtWidgets/QApplication>
#include <QString>
#include <QVariant>
//#include <QAxObject>
#include<ActiveQt/QAxObject>
#include<ActiveQt/QAxWidget>
//#include <QAxWidget>

class QWord : public QObject
{
	Q_OBJECT
public:
	explicit QWord(QObject *parent = 0);
	~QWord();
public:
	QAxObject * getDocuments() { return m_documents; }
	QAxObject* getDocument() { return m_document; }
	QAxObject* getWordApp() { return m_word; }
public:
	/**************************************************************************/
	/* 文件   操作                                                            */
	/**************************************************************************/
	void save();				//保存操作内容
	void close();				//关闭 退出 析构时候也会自动调用一次
	void saveAs();				//新建word另存为
	bool createNewWord();		//创建一个新的word
	bool Open(QString file);
	void openWord();			//打开一个现有的word
	bool createNewWord(const QString& filePath);		//创建一个新的word

	void setTableAutoFitBehavior(int flag);				//表格自动拉伸列 0固定  1根据内容调整  2 根据窗口调整
	void setPageOrientation(int flag);					//设置页面0为纵向wdOrientPortrait  1为横向wdOrientLandscape
	void setWordPageView(int flag);						//设置页面视图，是web视图wdWebView 6还是页面视图wdPrintView 3 或者阅读视图wdReadingView 7
	void setFontSize(int fontsize);						//设置字体大小
	void setFontName(QString& fontName);				//设置字体	比如 “宋体”
	void setFontBold(bool flag);						//字体加粗
	void setParagraphAlignment(int flag);				//设置选中位置文字居中 0 ,居左 1,居右 2
	void setColumnWidth(int column, int width);			//设置列宽
	void setColumnWidth(int nTable, int column, int width);			//设置列宽
	void setCellString(int row, int column, const QString& text);
	void setCellString(int nTable, int row, int column, const QString& text);
	void setCellFontBold(int row, int column, bool isBold);			//设置内容粗体  isBold控制是否粗体
	void setCellFontBold(int nTable, int row, int column, bool isBold);
	void setCellFontSize(int row, int column, int size);			//设置文字大小
	void setOptionCheckSpell(bool flags);							//设置审阅的拼写检查  true开启检查  false 取消检查

	QString GetText();												//获取内容
	void getUsedRange(int *topLeftRow, int *topLeftColumn, int *bottomRightRow, int *bottomRightColumn);
	void setSelectionRange(int start, int end);						//"SetRange(1, 9)"第1个字符后开始，到第9个字符结束范围
	QVariant getCellValue(int row, int column);						//获取单元格内容
	int getTableCount();											//获取word中表格总数


	QString getStrErrorInfo() { return m_strError; }					//获取代码中出现的错误信息可以用QMessageBox::information打印 在cpp不用QMessageBox 是怕你们在线程中调导出报表
	void deleteSelectColumn(int column);							//删除指定的列
	void moveForEnd();												//移动选定对象到文档末尾
	void insertCellPic(int row, int column, const QString& picPath);	//单元格插入图片
	void insertCellPic(int nTable, int row, int column, const QString& picPath);

	void intsertTable(int row, int column);							//插入一个几行几列表格
	void insertMoveDown();											//插入回车
	void insertText(const QString& text);							//插入文字

																	//插入巡检单位  巡检计划 巡检时间 巡 检 人  顺序传参
	void insertTitle(const QString& str1, const QString& str2, const QString& str3, const QString& str4);
	void MergeCells(int tableIndex, int nStartRow, int nStartCol, int nEndRow, int nEndCol);//合并单元格	
private:
	QAxObject * m_word;
	QAxObject* m_documents;
	QAxObject* m_document;
	QString  m_fileName;
	QString  m_saveName;
	QString  m_strError;
};

#endif // QWORD_H
