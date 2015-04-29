#ifndef CODE_MANAGER_H
#define CODE_MANAGER_H

#include <vector>
#include <QWidget>

#include "model/code.h"

class codeManager : public QWidget {
    Q_OBJECT
  private:
    std::vector<code*> m_codes;
  protected:

  public:
    codeManager();
  public slots:
    code*               addCode(const QString& name
                                , const QString& definition
                                , const QString& anker
                                , QColor color
                                , const QString& headcode);
    void                removeCode(code* codeToRemove);
    std::vector<code*>  getCodes(void);
    code*               getCode(const QString& name);
};

#endif // CODE_MANAGER_H