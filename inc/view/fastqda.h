#ifndef FASTQDA_H
#define FASTQDA_H

#include <QMainWindow>
#include <QDockWidget>

#include <QMenuBar>
#include <QMenu>
#include <QFileDialog>
#include <QMessageBox>
#include <QString>
#include <QTreeWidgetItem>

#include "view/documentList.h"
#include "view/mainWidget.h"
#include "view/codeList.h"

/**
 * @brief This class represents the main-window and its menubar.
 * @details Every action triggered via the menubar is processed here. Also it is responsible for all the systemcalls like quit.
 */
class fastQDA : public QMainWindow {
    Q_OBJECT
  private:
    /**
     * @brief This function creates the menubar and its entries.
     * @details All global functions of the program should be accessable from here.
     */
    QMenu*                   setupFileMenu(void);
    QMenu*                   setupEditMenu(void);
    QMenu*                   setupViewMenu(void);
    QMenu*                   setupHelpMenu(void);

    void                    setupDocumentBrowser(void);
    void                    setupMainWidget(void);
    void                    setupCodeBrowser(void);
  protected:
    QMenuBar*                myMenu;
    QFileDialog*             myFileDialog;
    QFileDialog*             myProjectDialog;
    mainWidget*              myMainWidget;
    QDockWidget*             documentBrowser;
    QDockWidget*             codeBrowser;
  public:
    /**
     * @brief std. ctor
     */
    fastQDA(void);
    /**
     * @brief std. dtor
     */
    ~fastQDA(void) {}

  public slots:
    /**
     * @brief This function imports a file into the local database.
     * @details If the selected file is a *.pdf its text is extracted with `pdftotext`, after that the file is saved in the local database and its content will be sent to the mainWidget's coding-pane.
     *
     * @param name The pointer to the selected file to import.
     */
    void importFile(const QString& name);
    /**
     * @brief This function shows the program in fullscreen-mode.
     * @details No explicit information atm.
     */
    void showFullScreen(void) {
        this->showFullScreen();
    }
    /**
     * @brief This function shows the program in the standard-size-window.
     * @details No explicit information atm.
     */
    void showNormal(void) {
        this->showNormal();
    }
    /**
     * @brief This function shows a messagebox.
     * @details At the moment there is only a `about`-box implemented.
     */
    void messageBox(void) {
        QMessageBox::about(this, "fastQDA", "fastQDA is a fast and simple alternative to MaxQDA.\n\n\t(C) by Michael Riedel, 2014");
        //myDialog->exec();
    }
    void                    setCodeinSelection(QTreeWidgetItem* currentItem);
    void                    saveProject(void);
};

#endif // FASTQDA_H