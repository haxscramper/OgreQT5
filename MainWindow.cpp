#include "MainWindow.hpp"
#include "QTOgreWindow.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    ogreWindow                  = new QTOgreWindow();
    QWidget* renderingContainer = QWidget::createWindowContainer(
        ogreWindow);

    resize(600, 600);

    this->setCentralWidget(renderingContainer);
}

MainWindow::~MainWindow() { delete ogreWindow; }
