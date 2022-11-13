#include "MainWindow.hpp"
#include "QTOgreWindow.hpp"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  ogreWindow = new QTOgreWindow();
  QWidget *renderingContainer = QWidget::createWindowContainer(ogreWindow);

  this->setCentralWidget(renderingContainer);
}

MainWindow::~MainWindow() {
  delete ogreWindow;
  delete ui;
}
