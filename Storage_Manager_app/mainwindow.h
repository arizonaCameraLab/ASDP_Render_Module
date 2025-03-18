/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>
#include <QMainWindow>
#include <ASDP_Core_API.h>
using namespace asdp;

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    /// @brief Network Interface Card to listen on has been selected.
    void SelectNIC(const QString& nicName); 

    /// @brief Server to connect to has been selected.
    void SelectServer(const QString& coreURL);

signals:
    /// @brief Signal to show or hide list of servers.
    void ShowServers(bool show);

    /// @brief Signal to show or hide the controls.
    void ShowControls(bool show);

    /// @brief Report the serial number of the server.
    void SetSerialNumber(QString serialNumber);

private:
  Ui::MainWindow* ui;

  std::shared_ptr<CoreClient> m_client;
  std::shared_ptr<Receiver> m_receiver;

  void ResetServer();
  void ResetNIC();
};

#endif // MAINWINDOW_H
