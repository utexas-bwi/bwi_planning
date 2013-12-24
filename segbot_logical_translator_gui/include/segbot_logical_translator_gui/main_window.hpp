/**
 * @file /include/segbot_logical_translator_gui/main_window.hpp
 *
 * @brief Qt based gui for segbot_logical_translator_gui.
 *
 * @date November 2010
 **/
#ifndef segbot_logical_translator_gui_MAIN_WINDOW_H
#define segbot_logical_translator_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace segbot_logical_translator_gui {

  /*****************************************************************************
   ** Interface [MainWindow]
   *****************************************************************************/
  /**
   * @brief Qt central, all operations relating to the view part here.
   */
  class MainWindow : public QMainWindow {
    Q_OBJECT

    public:
      MainWindow(int argc, char** argv, QWidget *parent = 0);
      ~MainWindow();

    public Q_SLOTS:

      void updateFrameInfo();
      void newTextAvailable();

    private:
      Ui::MainWindow ui;
      QNode qnode;
  };

}  // namespace segbot_logical_translator_gui

#endif // segbot_logical_translator_gui_MAIN_WINDOW_H
