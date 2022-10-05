#include <QDir>
#include <QFile>
#include <QColor>
#include <QStringList>
#include <QDebug>

#include <QDateTime>
#include <QFileDialog>
#include <QTextStream>

#include "qexcel.h"

QExcel::QExcel()
{
	c_c = 0;
}

QExcel::~QExcel()
{
    close();
}

QAxObject *QExcel::getWorkBooks()
{
    return workBooks;
}

QAxObject *QExcel::getWorkBook()
{
    return workBook;
}

QAxObject *QExcel::getWorkSheets()
{
    return sheets;
}

QAxObject *QExcel::getWorkSheet()
{
    return sheet;
}

bool QExcel::OpenExcel(QString xlsFilePath)
{
	excel = 0;
	workBooks = 0;
	workBook = 0;
	sheets = 0;
	sheet = 0;

	excel = new QAxObject();
	bool bl = excel->setControl("Excel.Application");
	if (!bl)
		return bl;
	workBooks = excel->querySubObject("Workbooks");
	QFile file(xlsFilePath);
	strName = xlsFilePath;
	if (file.exists())
	{
		workBooks->dynamicCall("Open(const QString&)", xlsFilePath);
	}
	else
	{
		workBooks->dynamicCall("Add");
	}
	workBook = excel->querySubObject("ActiveWorkBook");
	sheets = workBook->querySubObject("WorkSheets");
	return bl;
}

void QExcel::NewExcel(const QString &fileName)
{
	excel = 0;
	workBooks = 0;
	workBook = 0;
	sheets = 0;
	sheet = 0;

	excel = new QAxObject();
	excel->setControl("Excel.Application");
	excel->dynamicCall("SetVisible(bool)", false);
	excel->setProperty("DisplayAlerts", false);
	workBooks = excel->querySubObject("Workbooks");

	QFile file(fileName);
	if (!file.exists())
	{
		workBooks->dynamicCall("Add");
		workBook = excel->querySubObject("ActiveWorkBook");
		sheets = workBook->querySubObject("WorkSheets");
		workBook->dynamicCall("SaveAs(const QString&)", fileName);
	}
}
// 表单操作
void QExcel::insertSheet(QString sheetName)
{
    sheets->querySubObject("Add()");
    QAxObject * a = sheets->querySubObject("Item(int)", 1);
    a->setProperty("Name", sheetName);
}

void QExcel::selectSheet(int sheetIndex)
{
    sheet = sheets->querySubObject("Item(int)", sheetIndex);
}

void QExcel::selectSheet(const QString& sheetName)
{
    sheet = sheets->querySubObject("Item(const QString&)", sheetName);
}

int QExcel::getSheetsCount()
{
    return sheets->property("Count").toInt();
}

QString QExcel::getSheetName()
{
    return sheet->property("Name").toString();
}

QString QExcel::getSheetName(int sheetIndex)
{
    QAxObject * a = sheets->querySubObject("Item(int)", sheetIndex);
    return a->property("Name").toString();
}

// 单元格操作
void QExcel::clearCell(int row, int column)
{
    QString cell;
    cell.append(QChar(column - 1 + 'A'));
    cell.append(QString::number(row));

    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->dynamicCall("ClearContents()");
}

void QExcel::clearCell(const QString& cell)
{
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->dynamicCall("ClearContents()");
}

void QExcel::deleteSheet(int sheetIndex)
{
    QAxObject * a = sheets->querySubObject("Item(int)", sheetIndex);
    a->dynamicCall("delete");
}

void QExcel::deleteSheet(const QString& sheetName)
{
    QAxObject * a = sheets->querySubObject("Item(const QString&)", sheetName);
    a->dynamicCall("delete");
}

void QExcel::setCellString(int row, int column, const QString& value)
{
    QAxObject *range = sheet->querySubObject("Cells(int,int)", row, column);
    range->dynamicCall("SetValue(const QString&)", value);
	delete range;
}
void QExcel::setCellString(int row, int column, const int value)
{
    QAxObject *range = sheet->querySubObject("Cells(int,int)", row, column);
    range->dynamicCall("SetValue(const QString&)", value);
}

void QExcel::setCellString(int row, int column, const double value)
{
    QAxObject *range = sheet->querySubObject("Cells(int,int)", row, column);
    range->dynamicCall("SetValue(const QString&)", value);
}

void QExcel::setCellString(QString path, QList<QVariant> value)
{
	QAxObject *range = sheet->querySubObject("Range(QString)", path);
	range->setProperty("Value", value);
}

void QExcel::setCellStringC(QString path, QVariant value)
{
	QAxObject *range = sheet->querySubObject("Range(QString)", path);
	range->setProperty("Value", value);
}

void QExcel::setCellPicture(int row, int column, const QString& strPath)
{
	QAxObject *range = sheet->querySubObject("Shapes");
	range->dynamicCall("AddPicture( QString&, bool, bool, double, double, double, double)", strPath, true, true, row, column, 91, 77);
}

void QExcel::setCellPictureNew()
{
	QAxObject *range = sheet->querySubObject("Cells(int, int) = 123", 1, 1);
	//	range->dynamicCall("Sheet1.Shapes.AddPicture(\"D:\\test\\cc.jpg\", True, True, 10, 10, 50, 50).Name = \"s\"");
	//	range->dynamicCall("Sheet1.Shapes.AddPicture(\"D:\\test\\cc.jpg\", True, True, 10, 10, 50, 50).Name = \"s\"");

// 	sheet->dynamicCall("Cells(int, int) = 123", 1, 1);
// 	sheets->dynamicCall("Cells(int, int) = 123", 1, 1);
// 	workBook->dynamicCall("Cells(int, int) = 123", 1, 1);
// 	workBooks->dynamicCall("Cells(int, int) = 123", 1, 1);
// 	excel->dynamicCall("Cells(int, int) = 123", 1, 1);

	//workBook = excel->querySubObject("ActiveWorkBook");
	//sheets
// 	QString A;
// 	for (int i = 0; i < 2000; i++)
// 	{
// 		range->dynamicCall("AddPicture( QString&, bool, bool, double, double, double, double)", "D:\\test\\cc.jpg", true, true, 10, 10+i*20, 50, 50);
// 	}
// 	QString B;
}

void QExcel::setCellString(const QString& cell, const QString& value)
{
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->dynamicCall("SetValue(const QString&)", value);
}

QVariant QExcel::getCellValue(int row, int column)
{
    QAxObject *range = sheet->querySubObject("Cells(int,int)", row, column);
//    QVariant cellValue = range->property("Value");
    return range->property("Value");
}

QVariant QExcel::getCellValue(QString path)
{
	QAxObject *range = sheet->querySubObject("Range(QString)", path);
	return range->property("Value");
}

void QExcel::mergeCells(const QString& cell)
{
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->setProperty("VerticalAlignment", -4108);//xlCenter
    range->setProperty("WrapText", true);
    range->setProperty("MergeCells", true);
}

void QExcel::mergeCells(int topLeftRow, int topLeftColumn, int bottomRightRow, int bottomRightColumn)
{
    QString cell;
    cell.append(QChar(topLeftColumn - 1 + 'A'));
    cell.append(QString::number(topLeftRow));
    cell.append(":");
    cell.append(QChar(bottomRightColumn - 1 + 'A'));
    cell.append(QString::number(bottomRightRow));

    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->setProperty("VerticalAlignment", -4108);//xlCenter
    range->setProperty("WrapText", true);
    range->setProperty("MergeCells", true);
}

void QExcel::getUsedRange(int *topLeftRow, int *topLeftColumn, int *bottomRightRow, int *bottomRightColumn)
{
    QAxObject *usedRange = sheet->querySubObject("UsedRange");
    *topLeftRow = usedRange->property("Row").toInt();
    *topLeftColumn = usedRange->property("Column").toInt();

    QAxObject *rows = usedRange->querySubObject("Rows");
    *bottomRightRow = *topLeftRow + rows->property("Count").toInt() - 1;

    QAxObject *columns = usedRange->querySubObject("Columns");
    *bottomRightColumn = *topLeftColumn + columns->property("Count").toInt() - 1;
}

int QExcel::getUsedRowsCount()
{
    QAxObject *usedRange = sheet->querySubObject("UsedRange");
    int topRow = usedRange->property("Row").toInt();
    QAxObject *rows = usedRange->querySubObject("Rows");
    int bottomRow = topRow + rows->property("Count").toInt() - 1;
    return bottomRow;
}
// 单元格样式设置
void QExcel::setAutoFitRow(int row)
{
    QString rowsName;
    rowsName.append(QString::number(row));
    rowsName.append(":");
    rowsName.append(QString::number(row));

    QAxObject * rows = sheet->querySubObject("Rows(const QString &)", rowsName);
    rows->dynamicCall("AutoFit()");
}

void QExcel::setRowHeight(int row, int height)
{
    QString rowsName;
    rowsName.append(QString::number(row));
    rowsName.append(":");
    rowsName.append(QString::number(row));

    QAxObject * r = sheet->querySubObject("Rows(const QString &)", rowsName);
    r->setProperty("RowHeight", height);
}

void QExcel::setColumnWidth(int column, int width)
{
    QString columnName;
    columnName.append(QChar(column - 1 + 'A'));
    columnName.append(":");
    columnName.append(QChar(column - 1 + 'A'));

    QAxObject * col = sheet->querySubObject("Columns(const QString&)", columnName);
    col->setProperty("ColumnWidth", width);
}

void QExcel::setCellFontBold(int row, int column, bool isBold)
{
    QString cell;
    cell.append(QChar(column - 1 + 'A'));
    cell.append(QString::number(row));

    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range = range->querySubObject("Font");
    range->setProperty("Bold", isBold);
}

void QExcel::setCellFontBold(const QString &cell, bool isBold)
{
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range = range->querySubObject("Font");
    range->setProperty("Bold", isBold);
}

void QExcel::setCellFontSize(int row, int column, int size)
{
    QString cell;
    cell.append(QChar(column - 1 + 'A'));
    cell.append(QString::number(row));

    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range = range->querySubObject("Font");
    range->setProperty("Size", size);
}

void QExcel::setCellFontSize(const QString &cell, int size)
{
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range = range->querySubObject("Font");
    range->setProperty("Size", size);
}

void QExcel::setCellTextCenter(int row, int column)
{
    QString cell;
    cell.append(QChar(column - 1 + 'A'));
    cell.append(QString::number(row));

    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->setProperty("HorizontalAlignment", -4108);//xlCenter
}

void QExcel::setCellTextCenter(const QString &cell)
{
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->setProperty("HorizontalAlignment", -4108);//xlCenter
}

void QExcel::setCellTextWrap(int row, int column, bool isWrap)
{
    QString cell;
    cell.append(QChar(column - 1 + 'A'));
    cell.append(QString::number(row));

    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->setProperty("WrapText", isWrap);
}

void QExcel::setCellTextWrap(const QString &cell, bool isWrap)
{
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->setProperty("WrapText", isWrap);
}

void QExcel::setCellBorders(int row, int column,QColor color)
{
    QString cell;
    cell.append(QChar(column - 1 + 'A'));
    cell.append(QString::number(row));
    QAxObject *range = sheet->querySubObject("Range(const QString*)",cell);
    QAxObject *border = range->querySubObject("Borders");
    border->setProperty("Color",color);
}

void QExcel::setCellBorders(const QString &cell,QColor color)
{
    QAxObject *range = sheet->querySubObject("Range(const QString*)",cell);
    QAxObject *border = range->querySubObject("Borders");
    border->setProperty("Color",color);
}

void QExcel::setCellBorders(int topLeftRow, int topLeftColumn, int bottomRightRow, int bottomRightColumn, QColor color)
{
    QString cell;
    cell.append(QChar(topLeftColumn - 1 + 'A'));
    cell.append(QString::number(topLeftRow));
    cell.append(":");
    cell.append(QChar(bottomRightColumn - 1 + 'A'));
    cell.append(QString::number(bottomRightRow));
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    QAxObject *border = range->querySubObject("Borders");
    border->setProperty("Color",color);
}

void QExcel::setAllCellBorders(QColor color)
{
    int a,b,c,rowsCount;
    getUsedRange(&a, &b, &rowsCount, &c);
    for(int i=a;i<=rowsCount;i++)
    {
        for(int k=b;k<=c;k++)
        {
            setCellBorders(i,k,color);
        }
    }
}

void QExcel::setColumnTextFormat(int column, int type)
{
    QString cell;
    cell.append(QChar(column - 1 + 'A'));
    QString strType;
    switch (type) {
    case 0:
        strType = "@"; // 文本
        break;
    case 1:
        strType = "General"; // 常规
        break;
    case 2:
        strType = "0"; // 数字
        break;
    case 3:
        strType = "0.00"; // 数字(带小数点)
        break;
    case 4:
        strType = "#,##0;-#,##0";  // 数字(类似1,000)
        break;
    case 5:
        strType = "0%"; // 数字(百分比)
        break;
    case 6:
        strType = "yyyy'年'm'月'd'日'"; // 日期(年月日)
        break;
    case 7:
        strType = "h:mm:ss AM/PM"; // 时间(12点制)
        break;
    default:
        strType = "@";
        break;
    }
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->dynamicCall("NumberFormat",strType);
}

void QExcel::setCellTextFormat(int row, int column, int type)
{
    QString cell;
    cell.append(QChar(column - 1 + 'A'));
    cell.append(QString::number(row));
    QString strType;
    switch (type) {
    case 0:
        strType = "@"; // 文本
        break;
    case 1:
        strType = "General"; // 常规
        break;
    case 2:
        strType = "0"; // 数字
        break;
    case 3:
        strType = "0.00"; // 数字(带小数点)
        break;
    case 4:
        strType = "#,##0;-#,##0";  // 数字(类似1,000)
        break;
    case 5:
        strType = "0%"; // 数字(百分比)
        break;
    case 6:
        strType = "yyyy'年'm'月'd'日'"; // 日期(年月日)
        break;
    case 7:
        strType = "h:mm:ss AM/PM"; // 时间(12点制)
        break;
    default:
        strType = "@";
        break;
    }

    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->dynamicCall("NumberFormat",strType);
}

void QExcel::setCellTextFormat(const QString &cell, int type)
{
    QString strType;
    switch (type) {
    case 0:
        strType = "@"; // 文本
        break;
    case 1:
        strType = "General"; // 常规
        break;
    case 2:
        strType = "0"; // 数字
        break;
    case 3:
        strType = "0.00"; // 数字(带小数点)
        break;
    case 4:
        strType = "#,##0;-#,##0";  // 数字(类似1,000)
        break;
    case 5:
        strType = "0%"; // 数字(百分比)
        break;
    case 6:
        strType = "yyyy'年'm'月'd'日'"; // 日期(年月日)
        break;
    case 7:
        strType = "h:mm:ss AM/PM"; // 时间(12点制)
        break;
    default:
        strType = "@";
        break;
    }
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->dynamicCall("NumberFormat",strType);
}

void QExcel::setCellTextFormat(int topLeftRow, int topLeftColumn, int bottomRightRow, int bottomRightColumn, int type)
{
    QString cell;
    cell.append(QChar(topLeftColumn - 1 + 'A'));
    cell.append(QString::number(topLeftRow));
    cell.append(":");
    cell.append(QChar(bottomRightColumn - 1 + 'A'));
    cell.append(QString::number(bottomRightRow));
    QString strType;
    switch (type) {
    case 0:
        strType = "@"; // 文本
        break;
    case 1:
        strType = "General"; // 常规
        break;
    case 2:
        strType = "0"; // 数字
        break;
    case 3:
        strType = "0.00"; // 数字(带小数点)
        break;
    case 4:
        strType = "#,##0;-#,##0";  // 数字(类似1,000)
        break;
    case 5:
        strType = "0%"; // 数字(百分比)
        break;
    case 6:
        strType = "yyyy'年'm'月'd'日'"; // 日期(年月日)
        break;
    case 7:
        strType = "h:mm:ss AM/PM"; // 时间(12点制)
        break;
    default:
        strType = "@";
        break;
    }
    QAxObject *range = sheet->querySubObject("Range(const QString&)", cell);
    range->dynamicCall("NumberFormat",strType);
}

void QExcel::mergeSerialSameCellsInAColumn(int column, int topRow)
{
    int a,b,c,rowsCount;
    getUsedRange(&a, &b, &rowsCount, &c);

    int aMergeStart = topRow, aMergeEnd = topRow + 1;

    QString value;
    while(aMergeEnd <= rowsCount)
    {
        value = getCellValue(aMergeStart, column).toString();
        while(value == getCellValue(aMergeEnd, column).toString())
        {
            clearCell(aMergeEnd, column);
            aMergeEnd++;
        }
        aMergeEnd--;
        mergeCells(aMergeStart, column, aMergeEnd, column);

        aMergeStart = aMergeEnd + 1;
        aMergeEnd = aMergeStart + 1;
    }
}

void QExcel::save()
{
    if(!QFile::exists(strName))
        workBook->dynamicCall("SaveAs (const QString&)",QDir::toNativeSeparators(strName));
    else
        workBook->dynamicCall("Save()");
}

void QExcel::saveAs(QString NewPath)
{
	if (!QFile::exists(NewPath))
		workBook->dynamicCall("SaveAs (const QString&)", QDir::toNativeSeparators(NewPath));
	else
		workBook->dynamicCall("Save()");
}

QString QExcel::saveExcelPath(QString &fileName)
{
    strName = fileName;
    return strName;
}

void QExcel::close()
{
    if(excel!=NULL)
    {
        excel->dynamicCall("Quit(void)");
        delete sheet;
        delete sheets;
        delete workBook;
        delete workBooks;
        delete excel;

        excel = NULL;
        workBooks = NULL;
        workBook = NULL;
        sheets = NULL;
        sheet = NULL;
    }
}