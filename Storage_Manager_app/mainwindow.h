/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
  Ui::MainWindow* ui;
};

#endif // MAINWINDOW_H

