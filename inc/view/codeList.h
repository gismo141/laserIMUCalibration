#ifndef CODE_LIST_H
#define CODE_LIST_H

#include <QMainWindow>
#include <QVBoxLayout>
#include <QTreeWidget>
#include <QLabel>
#include <QPushButton>
#include <QString>

#include "../control/codeManager.h"

class codeList : public QWidget {
    Q_OBJECT
  private:
    QTreeWidgetItem*         addCodeToList(code* code);
    QTreeWidgetItem*         findCodeInList(const QString& name);
  protected:
    QMainWindow*             thisParent;
    codeManager*             myCodeManager;

    QVBoxLayout*             codeLayout;
    QTreeWidget*             codeTree;
    QLabel*                  colorLabel;
    QPushButton*             add;
    QPushButton*             setCode;
  public:
    codeList(QMainWindow* parent = 0);
    std::vector<QString>    getCodeList(void);
  public slots:
    void                    addCode(const QString& name, const QString& definition, const QString& anker, QColor color, const QString& headcode);
    code*                    getCode(QTreeWidgetItem* code);
    void                    updateCodeTree(void);
    void                    removeCode(const QString& codeToRemove);
    void                    setCodeinSelection(void);
};

#endif // CODE_LIST_H