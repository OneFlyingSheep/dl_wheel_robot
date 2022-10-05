#ifndef QEXCEL_H
#define QEXCEL_H

#include <ActiveQt/QAxObject>
#include <ActiveQt/QAxWidget>
#include <QString>
#include <QVariant>

class QAxObject;

class QExcel : public QObject
{
public:
    QExcel();
    ~QExcel();

public:
    QAxObject * getWorkBooks();
    QAxObject * getWorkBook();
    QAxObject * getWorkSheets();
    QAxObject * getWorkSheet();

	bool OpenExcel(QString xlsFilePath);

	void NewExcel(const QString &fileName);

public:
    /**************************************************************************/
    /* 工作表                                                                 */
    /**************************************************************************/
    //sheetIndex 起始于 1
    void selectSheet(int sheetIndex); // 获取表单
    void deleteSheet(int sheetIndex); // 删除表单
    void selectSheet(const QString& sheetName);
    void deleteSheet(const QString& sheetName);
    void insertSheet(QString sheetName); // 插入表单
    int getSheetsCount(); // 获取表单的个数
    //在 selectSheet() 之后才可调用
    QString getSheetName(); // 获取当前表单名称
    QString getSheetName(int sheetIndex);

    /**************************************************************************/
    /* 单元格                                                                 */
    /**************************************************************************/
    void setCellString(int row, int column, const QString& value); // 设置单元格的值
    void setCellString(int row, int column, const int value); // 设置单元格的值
    void setCellString(int row, int column, const double value); // 设置单元格的值
	void setCellPicture(int row, int column, const QString& strPath);
	void setCellPictureNew();

	void setCellString(QString path, QList<QVariant> value);
	void setCellStringC(QString path, QVariant value);
    //cell 例如 "A7"
    void setCellString(const QString& cell, const QString& value); // 设置单元格的值
    //range 例如 "A5:C7"
    void mergeCells(const QString& range); // 合并单元格
    void mergeCells(int topLeftRow, int topLeftColumn, int bottomRightRow, int bottomRightColumn); // 合并单元格
    QVariant getCellValue(int row, int column); // 获取单元格的值
	QVariant getCellValue(QString path);        // 获取单元格的值
    void clearCell(int row, int column); // 清除单元格内容
    void clearCell(const QString& cell); // 清除单元格内容

    /**************************************************************************/
    /* 布局格式                                                               */
    /**************************************************************************/
    int getUsedRowsCount(); // 获取单元格应用的列数
    void getUsedRange(int *topLeftRow, int *topLeftColumn, int *bottomRightRow, int *bottomRightColumn); // 获取当前活动的单元格范围
    void setColumnWidth(int column, int width); // 设置列宽
    void setRowHeight(int row, int height); // 设置行高
    void setAutoFitRow(int row); // 将行高设置适中
    void mergeSerialSameCellsInAColumn(int column, int topRow); // 将相同的内容进行合并单元格操作，并清空被合并单元格内容
    void setCellTextCenter(int row, int column); // 设置居中
    void setCellTextCenter(const QString& cell); // 设置居中
    void setCellTextWrap(int row, int column, bool isWrap); // 设置自动换行
    void setCellTextWrap(const QString& cell, bool isWrap); // 设置自动换行
    void setCellFontBold(int row, int column, bool isBold); // 设置单元格字体为粗体
    void setCellFontBold(const QString& cell, bool isBold); // 设置单元格字体为粗体
    void setCellFontSize(int row, int column, int size); // 设置单元格字体字号
    void setCellFontSize(const QString& cell, int size); // 设置单元格字体字号
    void setCellBorders(int row,int column,QColor color = QColor(0,0,0)); // 设置单元格边框
    void setCellBorders(const QString& cell,QColor color = QColor(0,0,0)); // 设置单个单元格边框
    void setCellBorders(int topLeftRow,int topLeftColumn,int bottomRightRow,int bottomRightColumn,QColor color = QColor(0,0,0)); // 设置某区域边框颜色
    void setAllCellBorders(QColor color = QColor(0,0,0)); // 设置所有边框为颜色
    void setColumnTextFormat(int column,int type = 0); // 设置某列的单元格的格式
    void setCellTextFormat(int row,int column, int type = 0); // 设置单元格格式
    void setCellTextFormat(const QString& cell,int type = 0); // 设置单元格格式
    void setCellTextFormat(int topLeftRow,int topLeftColumn,int bottomRightRow,int bottomRightColumn,int type = 0); // 设置某区域单元格格式

    /**************************************************************************/
    /* 文件                                                                   */
    /**************************************************************************/
    void save(); // 保存
	void saveAs(QString NewPath);
    QString saveExcelPath(QString &fileName);
    void close(); // 关闭

private:
    QAxObject * excel;
    QAxObject * workBooks;
    QAxObject * workBook;
    QAxObject * sheets;
    QAxObject * sheet;

    QAxObject* m_word;
    QAxObject* m_documents;
    QAxObject* m_document;

    QString strName;
    QString  m_saveName;
    QString  m_strError;

	QAxObject *c_range;

	int c_c;
};

#endif // QEXCEL_H
