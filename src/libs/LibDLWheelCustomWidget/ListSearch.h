#ifndef LISTSEARCH_H
#define LISTSEARCH_H

#include <QListWidget>

class ListSearch
{
public:
    static void SearchItem(QListWidget *pListWgt, const QString &strKeyWord);     //搜索函数

private:
    ListSearch();
    ~ListSearch();
	static void SetIsAllItemShow(bool bIsShow);						//设置所有节点是否显示
	static void FuzzySearch(QString &);								//模糊搜索
	static void PreciseSearch(const QString &strKeyWord);			//精确搜索
	static void GetPinYin(const QString &strHanZi, QString &strInitial, QString &strQuanPin);	//将汉字转为拼音（首字母以及全拼）
	static void GetWString(const std::string &strIn, std::wstring &wStrOut);//将string转为wstring
private:
    static QListWidget *m_pCurListWgt;
};

#endif // LISTSEARCH_H
