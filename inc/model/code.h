#ifndef CODE_H
#define CODE_H

#include <QString>
#include <QColor>

class code {
  private:
    QString m_name;
    QString m_definition;
    QString m_anchor;
    QColor  m_color;
    code*   m_headcode;
  public:
    code(const QString& name
         , const QString& definition
         , const QString& anchor
         , QColor color
         , code* headcode);
    void    setName(const QString& name);
    QString getName(void);
    void    setDefinition(const QString& definition);
    QString getDefinition(void);
    void    setAnchor(const QString& anchor);
    QString getAnchor(void);
    void    setColor(QColor color);
    QColor  getColor(void);
    void    setHeadcode(code* headcode);
    code*   getHeadcode(void);
};

#endif // CODE_H