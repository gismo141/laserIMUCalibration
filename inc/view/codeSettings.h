#ifndef CODE_SETTINGS_H
#define CODE_SETTINGS_H

#include <QDialog>
#include <QWidget>
#include <QPlainTextEdit>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QColor>
#include <QTreeWidgetItem>

class codeSettings : public QDialog {
    Q_OBJECT
  private:
    QWidget*		 thisParent;
    QLabel*			 l_name;
    QLabel*			 l_definition;
    QLabel*			 l_anchor;
    QLabel*			 l_headcode;
    QPlainTextEdit*	m_name;
    QPlainTextEdit*	m_definition;
    QPlainTextEdit*	m_anchor;
    QColor 			m_color;
    QComboBox*		 m_headcode;
    QPushButton*	 b_save;
    QPushButton*	 b_cancel;
    QPushButton*	 b_remove;
    QPushButton*	 b_changeColor;
    void			setHeight(QPlainTextEdit* edit, int nRows);
    void 			updateCodeList(void);
  public:
    codeSettings(QWidget* parent = 0);
  public slots:
    void 			setCodeColor(void);
    void 			saveCode(void);
    void 			removeCode(void);
    void 			changeParameters(QTreeWidgetItem*, int);
};

#endif // CODE_SETTINGS_H