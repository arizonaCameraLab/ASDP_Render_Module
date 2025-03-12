/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"  // Include the generated header

// Define the version number
const QString VERSION_NUMBER = "1.0.0";

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
  // Setup UI and other initialization
  ui->setupUi(this);  // Set up the UI

  // Set the window title with the version number
  setWindowTitle(QString("Storage Manager v%1").arg(VERSION_NUMBER));
}

MainWindow::~MainWindow()
{
  delete ui;
}
