/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>
#include <QMainWindow>
#include <QTimer>
#include <ASDP_Core_API.h>
#include <Display.h>
using namespace asdp;
using namespace asdp::render;

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

    /// @brief Slot for the periodic task that shows information.
    void PeriodicTask(); 

    /// @brief Start recording a stream.
    void StartRecording();

    /// @brief Stop recording a stream.
    void StopRecording();

    /// @brief Start replaying the specified stream.
    void StartReplay(const QString& streamID);

    /// @brief Delete the stream.
    void DeleteStream(const QString& streamID);

    /// @brief View the specified camera.
    void ViewCamera(const QString& cameraID);

signals:
    /// @brief Signal to show or hide list of servers.
    void ShowServers(bool show);

    /// @brief Signal to show or hide the controls.
    void ShowControls(bool show);

    /// @brief Report the serial number of the server.
    void SetSerialNumber(QString serialNumber);

    /// @brief Set the information about the Core.
    void SetInfo(QString coreInfo);

private:
  Ui::MainWindow* ui;

  void ResetServer();
  void ResetNIC();

  // Timer for a periodic task that polls the server for messages and updates info.
  std::shared_ptr<QTimer> m_timer;

  std::shared_ptr<CoreClient> m_client;
  std::shared_ptr<Receiver> m_receiver;

  std::vector<FeatureID> m_features;
  std::vector<CameraInfo> m_cameras;
  std::vector<CameraInfo> m_lastCameras;
  std::vector<uint32_t> m_streams;
  std::vector<uint32_t> m_lastStreams;
  std::string m_streamInfo;

  // Variables and functions for displaying video from a camera.
  std::shared_ptr<DisplayTexture> m_displayTexture;
  std::vector< std::shared_ptr<CameraRenderInfo> > m_visibleCameras;
  std::shared_ptr<CompositeCameras> m_composite;
  std::shared_ptr<Display> m_display;
};

#endif // MAINWINDOW_H
