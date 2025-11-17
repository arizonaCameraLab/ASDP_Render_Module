/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>
#include <thread>
#include <QMainWindow>
#include <QTimer>
#include <ASDP_Core_API.h>
#include <Display.h>
#include <ToneMap.h>
#include <ImageStatistics.h>
#include <CUDABufferPool.h>
#include <cuda_runtime.h>

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

    /// @brief The skip interval has changed.
    void SkipIntervalChanged();

    /// @brief Do recording at startup
    void RecordAtStartup();

    /// @brief Do not record at startup
    void NoRecordAtStartup();

    /// @brief Use an IR camera (as opposed to a visible light camera).
    void UseIRCamera(bool isIR);

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
  void ResetStreaming();

  // Timer for a periodic task that polls the server for messages and updates info.
  std::shared_ptr<QTimer> m_timer;

  std::shared_ptr<CoreClient> m_client;
  std::shared_ptr<Receiver> m_receiver;
  std::shared_ptr<ReceiverUDP> m_receiverCam;
  std::string m_hostname;
  QString m_coreURL;
  bool m_triggersConfigured = false;

  std::vector<FeatureID> m_features;
  std::vector<CameraInfo> m_cameras;
  std::vector<CameraInfo> m_lastCameras;
  std::vector<uint32_t> m_streams;
  std::vector<uint32_t> m_lastStreams;
  std::string m_streamInfo;

  // Image statistics.
  std::shared_ptr<asdp::render::imageStatistics::MeanStdGroup> m_meanStdGroup;

  // Variables and functions for displaying video from a camera.
  std::shared_ptr<Display> m_display;
  std::shared_ptr<DisplayTexture> m_displayTexture;
  std::vector< std::shared_ptr<CameraRenderInfo> > m_visibleCameras;
  GLuint m_toneMap = 0;
  std::atomic<bool> m_doneStreaming{ false };
  std::shared_ptr<CUDABufferPool> m_cpuPinnedImageBuffer;
  std::shared_ptr<CUDABufferPool> m_gpuImageBuffer;
  std::shared_ptr<cudaStream_t> m_stream;
  std::shared_ptr<std::thread> m_copyThread;
  std::shared_ptr<std::thread> m_receiveThread;
  std::vector<RenderTimingInfo::camera> m_emptyTimingInfo;
  StreamEndpoint m_endpoint;
  uint32_t m_streamingCameraID = 0;
  bool m_useIRCamera = true;
};

#endif // MAINWINDOW_H
