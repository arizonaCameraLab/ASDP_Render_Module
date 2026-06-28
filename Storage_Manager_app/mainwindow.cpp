/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <GL/glew.h>

#ifdef _WIN32
#include <winsock2.h>
#include <iphlpapi.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Iphlpapi.lib")
#else
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#endif

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include "mainwindow.h"
#include "ui_mainwindow.h"  // Include the generated header

#include <ASDP_SpinFreeQueue.hpp>
#include <CPUDataToTextureHandler.h>

// Define the version number
const QString VERSION_NUMBER = "1.9.0";

static std::vector<std::string> getIPAddresses()
{
  std::vector<std::string> ipAddresses;

#ifdef _WIN32
  // Add Localhost, which does not show up on the list on Windows.
  ipAddresses.push_back("localhost");

  // Get the list of all network interfaces on the system
  // Get the list of all network interfaces on the system
  ULONG bufferLength = 0;
  GetAdaptersInfo(NULL, &bufferLength);

  IP_ADAPTER_INFO* adapterInfo = (IP_ADAPTER_INFO*)malloc(bufferLength);
  if (GetAdaptersInfo(adapterInfo, &bufferLength) == NO_ERROR) {
    // Iterate over all network interfaces
    for (IP_ADAPTER_INFO* adapter = adapterInfo; adapter; adapter = adapter->Next) {
      // Iterate over all IP addresses for this network interface
      for (IP_ADDR_STRING* ipAddr = &adapter->IpAddressList; ipAddr; ipAddr = ipAddr->Next) {
        // Skip empty IP addresses
        if (ipAddr->IpAddress.String[0] != '0') {
          ipAddresses.push_back(ipAddr->IpAddress.String);
        }
      }
    }
  }

  free(adapterInfo);
#else
  struct ifaddrs* ifAddrStruct = NULL;
  struct ifaddrs* ifa = NULL;
  void* tmpAddrPtr = NULL;

  getifaddrs(&ifAddrStruct);

  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr) {
      continue;
    }
    // check it is IP4
    if (ifa->ifa_addr->sa_family == AF_INET) {
      tmpAddrPtr = &((struct sockaddr_in*)ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
      // Skip empty IP addresses
      if (addressBuffer[0] != '0') {
        ipAddresses.push_back(addressBuffer);
      }
    }
  }
  if (ifAddrStruct != NULL) {
    freeifaddrs(ifAddrStruct);
  }
#endif

  return ipAddresses;
}

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent), ui(new Ui::MainWindow), m_timer(std::make_shared<QTimer>(this))
{
  // Setup UI and other initialization
  ui->setupUi(this);  // Set up the UI

  // Set the window title with the version number
  setWindowTitle(QString("Storage Manager v%1").arg(VERSION_NUMBER + "-" + BUILD_TYPE + " using Core API "
    + asdp::Core::GetVersion().c_str()));

  // Fill in the entries for the skip combo box and set its default value.
  ui->comboBoxSkip->addItem("0");
  for (int i = 2; i <= 20; i += 2) {
    ui->comboBoxSkip->addItem(QString::number(i));
  }
  ui->comboBoxSkip->setCurrentIndex(5);  // Default to 10

  // Look up the network interfaces that are available and add them to the combo box.
  std::vector<std::string> ipAddresses = getIPAddresses();
  ui->comboBoxNIC->addItem("");
  for (const std::string& ipAddress : ipAddresses) {
    ui->comboBoxNIC->addItem(QString::fromStdString(ipAddress));
  }

  // Create a CUDA stream for us to use with an auto-deleting destructor.
  cudaStream_t* streamPtr = new cudaStream_t;
  cudaStreamCreate(streamPtr);
  m_stream = std::shared_ptr<cudaStream_t>(streamPtr,
    [](cudaStream_t* ptr) { cudaStreamDestroy(*ptr); delete ptr; }
  );

  // Hook up the timer to the periodic task.
  connect(m_timer.get(), &QTimer::timeout, this, &MainWindow::PeriodicTask);
}

MainWindow::~MainWindow()
{
  ResetServer();
  delete ui;
}

void MainWindow::SelectNIC(const QString& nicName)
{
  ResetNIC();
  m_hostname = nicName.toStdString();

  // Implement the logic for selecting the NIC here
  std::cout << "Selected NIC: " << nicName.toStdString() << std::endl;

  {
    // Find the list of servers on the selected NIC and add them to the
    // pull-down list.
    m_client = std::make_shared<CoreClient>(nicName.toStdString());
    if (m_client->GetConstructorStatus() != OKAY) {
      std::cerr << "Failed to open client: " << ErrorMessage(m_client->GetConstructorStatus()) << std::endl;
      return;
    }
    std::cout << "Listening for servers on " << nicName.toStdString() << std::endl;

    // Wait for two seconds to allow servers to send Discovery messages and then check the status of
    // the Discover thread and find the servers.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::map<uint32_t, std::string> servers;
    Status threadStatus;
    Status status = m_client->GetDiscoveryThreadStatus(threadStatus);
    if (status != OKAY) {
      std::cerr << "Failed to get discovery thread status: " << ErrorMessage(status) << std::endl;
      return;
    }
    if (threadStatus != OKAY) {
      std::cerr << "Discovery thread status: " << ErrorMessage(threadStatus) << std::endl;
      return;
    }
    status = m_client->IdentifiedServers(servers);
    if (status != OKAY) {
      std::cerr << "Failed to get identified servers: " << ErrorMessage(status) << std::endl;
      return;
    }
    std::cout << "Servers found: " << servers.size() << std::endl;
    for (const auto& server : servers) {
      ui->comboBoxCore->addItem(QString::fromStdString(server.second) + " (serial #" + QString::number(server.first) + ")");
      std::cout << "  " << server.second << " (serial #" << server.first << ")" << std::endl;
    }

    // Emit the signal to show the list of servers
    emit ShowServers(true);
  }
}

void MainWindow::ResetNIC()
{
  ui->comboBoxCore->clear();
  ui->comboBoxCore->addItem("");
  m_client.reset();
  emit ShowServers(false);

  // Reset the server
  ResetServer();
}

void MainWindow::ResetServer()
{
  ResetStreaming();
  ui->comboBoxCamera->setCurrentIndex(0);
  ui->comboBoxReplay->setCurrentIndex(0);
  m_receiver.reset();
  m_features.clear();
  m_cameras.clear();
  m_streams.clear();
  emit SetInfo("");
  emit ShowControls(false);
  emit SetSerialNumber("");
  m_timer->stop();
}

void MainWindow::ResetStreaming()
{
  // Stop any running threads.
  m_doneStreaming = true;
  if (m_copyThread) {
    m_copyThread->join();
    m_copyThread.reset();
  }
  if (m_receiveThread) {
    m_receiveThread->join();
    m_receiveThread.reset();
  }
  m_doneStreaming = false;

  // Request stop streaming on a camera if we have an endpoint.
  if (m_client && m_streamingCameraID) {
    m_client->SendCommandPacket(CommandPacketCancelSubregion(m_streamingCameraID, m_endpoint));
    m_streamingCameraID = 0;
    m_endpoint = StreamEndpoint();
  }

  // Reset the camera receiver.
  m_receiverCam.reset();

  // Grab the context and then clear the visible cameras and delete the tone map.
  if (m_displayTexture) {
    m_displayTexture->BorrowContext();
    m_visibleCameras.clear();
    glDeleteTextures(1, &m_toneMap);
    m_display.reset();
    std::cout << "Not viewing camera." << std::endl;
    m_displayTexture->ReturnContext();
  }
}

void MainWindow::SelectServer(const QString& coreURL)
{
  // Strip of a space any anything after the space.
  int spaceIndex = coreURL.indexOf(' ');
  m_coreURL = (spaceIndex >= 0) ? coreURL.left(spaceIndex) : coreURL;
  ResetServer();
  if (m_coreURL.isEmpty()) {
    Status status = m_client->DisconnectFromServer();
    if (status != OKAY) {
      std::cerr << "Failed to disconnect from server: " << ErrorMessage(status) << std::endl;
    }
    return;
  }

  std::cout << "Selected Core: " << coreURL.toStdString() << std::endl;

  // Connect to the server.
  std::cout << "Connecting to " << m_coreURL.toStdString() << std::endl;
  uint16_t major, minor, patch;
  Status status = m_client->ConnectToServer(m_coreURL.toStdString(), major, minor, patch);
  if (status != OKAY) {
    std::cerr << "Failed to connect to server: " << ErrorMessage(status) << std::endl;
    return;
  }
  std::cout << "  Connected to server version " << major << "." << minor << "." << patch << std::endl;

  // If the string contains "(serial #", then extract the serial number and tell which serial number we are connected to.
  QString serialValue;
  int start = coreURL.indexOf("(serial #");
  if (start != -1) {
    start += QString("(serial #").length();
    int end = coreURL.indexOf(')', start);
    if (end != -1) {
      serialValue = coreURL.mid(start, end - start).trimmed();
    }
  }
  if (!serialValue.isEmpty()) {
    uint32_t serialNumber = serialValue.toInt();
    std::cout << "  Connected to server with serial number " << serialNumber << std::endl;
    SetSerialNumber(std::to_string(serialNumber).c_str());
  }

  // Get the main stream receiver
  status = m_client->GetMainStreamReceiver(m_receiver);
  if (status != OKAY) {
    std::cerr << "Failed to get main stream receiver: " << ErrorMessage(status) << std::endl;
    return;
  }

  // Show the controls.
  emit ShowControls(true);

  // Mark triggers not configured.
  m_triggersConfigured = false;

  // Start the update timer, firing 10x/second.
  m_timer->start(100);  // 100 milliseconds
}

void MainWindow::PeriodicTask()
{
  if (m_receiver) {
    std::shared_ptr<StreamPacket> response;
    size_t offset = 0;
    Status status = m_receiver->ReceiveStreamPacket(0, response, offset);
    while (status == OKAY) {
      std::shared_ptr<Message> message;
      status = response->GetNextMessage(message);
      if (status != OKAY) {
        std::cerr << "Failed to get message from stream packet: " << ErrorMessage(status) << std::endl;
        return;
      }
      MessageID type;
      status = message->GetType(type);
      if (status != OKAY) {
        std::cerr << "Failed to get message type: " << ErrorMessage(status) << std::endl;
        return;
      }
      switch (type) {
      case STATE:
        {
          // Cast the message into a state message and then extract the information and print it.
          MessageState state(*message);
          if (state.GetConstructorStatus() != OKAY) {
            std::cerr << "Failed to construct state message: " << ErrorMessage(state.GetConstructorStatus()) << std::endl;
            return;
          }

          std::string coreInfo = "State:\n";

          uint8_t recordOnReset;
          status = state.GetRecordOnReset(recordOnReset);
          if (status != OKAY) {
            std::cerr << "Failed to get record on reset: " << ErrorMessage(status) << std::endl;
            return;
          }
          coreInfo += "  Record on Reset: " + std::to_string(recordOnReset) + "\n";

          uint8_t streaming;
          status = state.GetCamerasStreaming(streaming);
          if (status != OKAY) {
            std::cerr << "Failed to get cameras streaming: " << ErrorMessage(status) << std::endl;
            return;
          }
          coreInfo += "  Cameras Streaming: " + std::to_string(streaming) + "\n";

          status = state.GetCameras(m_cameras);
          if (status != OKAY) {
            std::cerr << "Failed to get cameras: " << ErrorMessage(status) << std::endl;
            return;
          }
          coreInfo += "  Cameras: " + std::to_string(m_cameras.size()) + "\n";
          if (m_cameras != m_lastCameras) {
            m_lastCameras = m_cameras;
            ui->comboBoxCamera->clear();
            ui->comboBoxCamera->addItem("");
            for (int i = 1; i <= m_cameras.size(); ++i) {
              ui->comboBoxCamera->addItem(QString::number(i));
            }
          }

          uint64_t totalDiskSpace, remainingDiskSpace;
          status = state.GetTotalDiskSpace(totalDiskSpace);
          if (status != OKAY) {
            std::cerr << "Failed to get total disk space: " << ErrorMessage(status) << std::endl;
            return;
          }
          status = state.GetRemainingDiskSpace(remainingDiskSpace);
          if (status != OKAY) {
            std::cerr << "Failed to get remaining disk space: " << ErrorMessage(status) << std::endl;
            return;
          }
          coreInfo += "  Disk Space: " + std::to_string(totalDiskSpace / 1000000000) + "GB total, "
            + std::to_string(remainingDiskSpace / 1000000000) + "GB remaining\n";

          uint8_t storing;
          status = state.GetStoring(storing);
          if (status != OKAY) {
            std::cerr << "Failed to get storing: " << ErrorMessage(status) << std::endl;
            return;
          }
          coreInfo += "  Recording: " + std::to_string(storing) + "\n";

          uint8_t replaying;
          status = state.GetReplaying(replaying);
          if (status != OKAY) {
            std::cerr << "Failed to get replaying: " << ErrorMessage(status) << std::endl;
            return;
          }
          coreInfo += "  Replaying: " + std::to_string(replaying) + "\n";

          uint8_t replayAtEnd;
          status = state.GetReplayAtEnd(replayAtEnd);
          if (status != OKAY) {
            std::cerr << "Failed to get replay at end: " << ErrorMessage(status) << std::endl;
            return;
          }
          coreInfo += "  Replay at end: " + std::to_string(replayAtEnd) + "\n";

          emit SetInfo(QString::fromStdString(coreInfo + "\n" + m_streamInfo));

          // Store the available features.
          status = state.GetFeatures(m_features);
          if (status != OKAY) {
            std::cerr << "Failed to get features: " << ErrorMessage(status) << std::endl;
            return;
          }

          // If the triggers have not been configured, then set up the triggers to run the cameras
          // at their maximum frame rate.
          if (!m_triggersConfigured && m_cameras.size() > 0) {
            double cameraFPS = 1.0 / m_cameras[0].minTriggerPeriod;

            for (size_t i = 0; i < m_cameras.size(); i++) {
              CameraInfo& camera = m_cameras[i];
              TriggerInfo ti;
              ti.ID = camera.trigger;
              ti.mode = 3;
              ti.period = 1 / cameraFPS;
              ti.offset = 0;
              ti.trackingFactor = 0.005;
              ti.externalID = camera.trigger;
              status = m_client->SendCommandPacket(CommandPacketConfigureTrigger(ti));
              if (status != OKAY) {
                std::cerr << "Failed to configure trigger: " << ErrorMessage(status) << std::endl;
              }
              std::cout << "  Configured trigger for camera " << i+1 << " with period " << ti.period << " seconds" << std::endl;
            }
            m_triggersConfigured = true;
          }
        }
        break;

      case STORED_STREAMS:
        {
          // Cast the message into a stored streams message and then extract the information and print it.
          MessageStoredStreamList storedStreams(*message);
          if (storedStreams.GetConstructorStatus() != OKAY) {
            std::cerr << "Failed to construct stored streams message: " << ErrorMessage(storedStreams.GetConstructorStatus()) << std::endl;
            return;
          }
          status = storedStreams.GetIDs(m_streams);
          if (status != OKAY) {
            std::cerr << "Failed to get stored stream IDs: " << ErrorMessage(status) << std::endl;
            return;
          }
          m_streamInfo = "Streams:\n";
          for (const uint32_t& stream : m_streams) {
            m_streamInfo += "  " + std::to_string(stream) + "\n";
          }
          if (m_streams != m_lastStreams) {
            m_lastStreams = m_streams;

            ui->comboBoxReplay->clear();
            ui->comboBoxReplay->addItem("");
            ui->comboBoxDelete->clear();
            ui->comboBoxDelete->addItem("");
            for (const uint32_t& stream : m_streams) {
              ui->comboBoxReplay->addItem(QString::number(stream));
              ui->comboBoxDelete->addItem(QString::number(stream));
            }
          }
        }
        break;

      default:
        // Ignore other message types.
        {};
      } // switch based on type

      status = m_receiver->ReceiveStreamPacket(0, response, offset);
    }
    if (status != TIMEOUT) {
      std::cerr << "Failed to receive stream packet: " << ErrorMessage(status) << std::endl;
    }

    // Request a list of stored streams if the storage API is available and we have a client.
    if (std::find(m_features.begin(), m_features.end(), STORAGE_API_AVAILABLE) != m_features.end()) {\
      if (m_client) {
        status = m_client->SendCommandPacket(CommandPacketListStoredStreams());
        if (status != OKAY) {
          std::cerr << "Failed to list streams: " << ErrorMessage(status) << std::endl;
          return;
        }
      }
    }

  } // if (m_receiver)

  // Stop streaming and delete the display if it is no longer valid, and reset the selection.
  if (m_display) {
    if (m_display->GetStatus() != "") {
      ResetStreaming();
      ui->comboBoxCamera->setCurrentIndex(0);
    }
  }
}

void MainWindow::StartRecording()
{
  if (m_client) {
    Status status = m_client->SendCommandPacket(CommandPacketStartRecording());
    if (status != OKAY) {
      std::cerr << "Failed to start recording: " << ErrorMessage(status) << std::endl;
      return;
    }
  }
}

void MainWindow::StopRecording()
{
  if (m_client) {
    Status status = m_client->SendCommandPacket(CommandPacketStopRecording());
    if (status != OKAY) {
      std::cerr << "Failed to stop recording: " << ErrorMessage(status) << std::endl;
      return;
    }
  }
}

void MainWindow::StartReplay(const QString& streamID)
{
  if (m_client) {
    if (streamID.isEmpty()) {
      Status status = m_client->SendCommandPacket(CommandPacketStopReplay());
      if (status != OKAY) {
        std::cerr << "Failed to stop replay: " << ErrorMessage(status) << std::endl;
        return;
      }
    } else {
      Status status = m_client->SendCommandPacket(CommandPacketStartReplay(streamID.toUInt(), Time(1, 0)));
      if (status != OKAY) {
        std::cerr << "Failed to start replay: " << ErrorMessage(status) << std::endl;
        return;
      }
    }
  }
}

void MainWindow::DeleteStream(const QString& streamID)
{
  if (m_client) {
    if (!streamID.isEmpty()) {
      std::cout << "Deleting stream: " << streamID.toStdString() << std::endl;
      Status status = m_client->SendCommandPacket(CommandPacketEraseStoredStream(streamID.toUInt()));
      if (status != OKAY) {
        std::cerr << "Failed to delete stream: " << ErrorMessage(status) << std::endl;
        return;
      }
    }
  }
}

void MainWindow::RecordAtStartup()
{
  if (m_client) {
    Status status = m_client->SendCommandPacket(CommandPacketSetStartUpRecordingState(1));
    if (status != OKAY) {
      std::cerr << "Failed to set record at startup: " << ErrorMessage(status) << std::endl;
      return;
    }
  }
}

void MainWindow::NoRecordAtStartup()
{
  if (m_client) {
    Status status = m_client->SendCommandPacket(CommandPacketSetStartUpRecordingState(0));
    if (status != OKAY) {
      std::cerr << "Failed to set no record at startup: " << ErrorMessage(status) << std::endl;
      return;
    }
  }
}

void MainWindow::UseIRCamera(bool isIR)
{
  m_useIRCamera = isIR;
  if (m_streamingCameraID != 0) {
    std::cout << "Launching camera, IR =" << isIR << std::endl;
    ViewCamera(QString::number(m_streamingCameraID));
  }
}

void MainWindow::SkipIntervalChanged()
{
  if (m_streamingCameraID != 0) {
    std::cout << "Changing skip interval to " << ui->comboBoxSkip->currentText().toStdString() << std::endl;
    // Stop the current streaming and then restart it with the new skip interval.
    ViewCamera(QString::number(m_streamingCameraID));
  }
}

void MainWindow::ViewCamera(const QString& cameraID)
{
  // Remove any existing streaming and display. We can only have one display at a time.
  ResetStreaming();

  if (!m_client || cameraID.isEmpty()) {
    return;
  }
  std::cout << "Viewing Camera: " << cameraID.toStdString() << std::endl;

  size_t index = cameraID.toUInt() - 1;
  if (index >= m_cameras.size()) {
    std::cerr << "Invalid camera index: " << index << std::endl;
    return;
  }
  uint16_t width = m_cameras[index].width;
  uint16_t height = m_cameras[index].height;

  // Get the IP address of the NIC we are using to talk with the server from looking at the TCP receiver.
  SenderReceiverTCP* tcpReceiver = dynamic_cast<SenderReceiverTCP*>(m_receiver.get());
  if (!tcpReceiver) {
    std::cerr << "No TCP receiver to get NIC address from." << std::endl;
    return;
  }
  uint32_t ip;
  if (OKAY != m_client->GetMyIP(ip)) {
    std::cerr << "Failed to get IP address from TCP receiver." << std::endl;
    return;
  }
  ip = GetLocalIPForRemote(ip);
  if (ip == 0) {
    std::cerr << "Failed to get local IP address for remote IP." << std::endl;
    return;
  }

  // Create a UDP receiver for the camera.
  m_receiverCam = std::make_shared<ReceiverUDP>(StreamEndpoint(ip,0));
  if (m_receiverCam->GetConstructorStatus() != OKAY) {
    std::cerr << "Error constructing ReceiverUDP: " << ErrorMessage(m_receiverCam->GetConstructorStatus()) << std::endl;
    m_receiverCam.reset();
    return;
  }

  // Construct a DisplayTexture object to handle textures.  It will be the base object that all others will use
  // to share contexts.
  m_displayTexture = std::make_shared<DisplayTexture>();

  // Construct information about the camera to be displayed. Add it as the only entry in the vector.
  std::shared_ptr<Distortion> dist(new DistortionNone);
  std::shared_ptr<Vignette> vig(new VignetteNone);
  std::shared_ptr<asdp::render::CameraRenderInfo> info =
    std::make_shared<CameraRenderInfo>(cameraID.toUShort(),
      std::array<double, 3>{0.0, 0.0, 0.0}, std::array<double, 3>{ 0.0, 0.0, 0.0 },
      std::array<uint16_t, 2>{width, height}, std::array<double, 2>{40.0, 32.5},
      dist, vig, std::make_shared<asdp::render::ImageQueue>(), -1.0f);

  // Fill in three textures for this camera, all gray and at time zero.
  // We must borrow the context from the displayTexture so that we can create the textures.
  if (!m_displayTexture->BorrowContext()) {
    std::cerr << "Error borrowing context from displayTexture." << std::endl;
    return;
  }

  std::vector<uint16_t> image(size_t(width) * size_t(height), 32767);

  // Create the textures for the camera. Make two for each Composite to pull when it is looking
  // for the next image to render, one for the texture thread to write to, and one to lie fallow.
  for (size_t i = 0; i < 2 + 2 + 1; i++) {
    std::shared_ptr<ImageData> imageData = std::make_shared<ImageData>();

    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    // Set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // Set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Load image into the texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16, width, height, 0, GL_RED, GL_UNSIGNED_SHORT, image.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    imageData->texture = texture;
    info->m_imageQueue->InsertImage(imageData);
  }

  // Handle construction of a tonemap texture, removing the previous if there is one.
  glDeleteTextures(1, &m_toneMap);
  glGenTextures(1, &m_toneMap);
  m_toneMap = ToneMap().GenerateTexture();
  if (!m_useIRCamera) {
    // Visible-light cameras only have values in the range 0-1023
    m_toneMap = ToneMap({ {0.0, 0.0,0.0,0.0}, {1023.0f/65535, 1.0,1.0,1.0} }).GenerateTexture();
  }

  if (!m_displayTexture->ReturnContext()) {
    std::cerr << "Error returning context to displayTexture." << std::endl;
    return;
  }

  // Construct a vector with a single camera in it to display.
  m_visibleCameras.push_back(info);

  // Make a range estimator that does the whole range.
  std::shared_ptr<RangeEstimator> rangeEstimator = std::make_shared<RangeEstimatorFixed>();

  // Configure an event (empty) structure to handle callbacks for the display windows.
  std::shared_ptr<EventHandlers> handlers = std::make_shared<EventHandlers>();

  // Create a PoseAdjuster that does not adjust based on helicopter motion and disables latency compensation.
  PoseAdjusterCoordinates poseAdjusterCoordinates = INITIAL_ORIENTATION;
  std::shared_ptr<PoseAdjuster> poseAdjuster = std::make_shared<PoseAdjuster>(2000, poseAdjusterCoordinates, true);

  // If we are using an IR camera, create a range estimator based on its mean and standard deviation.
  if (m_useIRCamera) {
    // Make a display object that shares textures with the others.
    std::shared_ptr<Display> display = std::make_shared<DisplayTexture>(m_displayTexture.get());
    m_meanStdGroup = std::make_shared<asdp::render::imageStatistics::MeanStdGroup>(m_visibleCameras,
      display, m_cameras[0].minTriggerPeriod);
    rangeEstimator = std::make_shared<RangeEstimatorStdRanges>(m_meanStdGroup, 1.5, 1.5);
  }

  // Construct a composite object to render the visible cameras.
  std::shared_ptr<CompositeCameras> composite = std::make_shared<CompositeCameras>(
    m_visibleCameras, m_toneMap, poseAdjuster, Time(1 / 60.0),
    0,
    Time(0, 1000000 / 60.0), nullptr,
    rangeEstimator);

  // Construct a DisplayWindow to show the camera data.
  std::string name = "Camera " + cameraID.toStdString();
  std::array<float, 3> viewpointOffset = { 0.0f, 0.0f, 0.0f };
  std::array<float, 3> viewpointRotation = { 0.0f, 0.0f, 0.0f };
  m_display = std::make_shared<DisplayWindow>(name, composite, m_client, 0, 0, 0,
    viewpointOffset, viewpointRotation, 60, 2500,
    width, height, 40.0, "", m_displayTexture.get());

  // Construct shared pointers to the data structures that we'll need to do rendering, with
  // custom destructors that will clean up when the shared_ptr is destroyed.
  try {
    m_cpuPinnedImageBuffer = std::make_shared<CUDABufferPool>(width * height * sizeof(uint16_t), 5, true);
    m_gpuImageBuffer = std::make_shared<CUDABufferPool>(width * height * sizeof(uint16_t), 5, false);
  } catch (const std::exception& e) {
    std::cerr << "Error constructing buffers: " << e.what() << std::endl;
    return;
  }

  // Make the queues to pass data between the receiver and texture threads.
  std::shared_ptr< SpinFreeQueue < std::shared_ptr<DataToSendToGPU> > > dataQueue =
    std::make_shared< SpinFreeQueue< std::shared_ptr<DataToSendToGPU> > >();

  // Launch the threads to receive and then copy data to the GPU. Send all of the data to the GPU in one
  // batch at the end of the frame.
  m_doneStreaming = false;
  m_copyThread = std::make_shared<std::thread>(CopyDataToTextures, width, height, std::ref(m_doneStreaming),
    dataQueue, size_t(height), m_displayTexture, std::ref(m_emptyTimingInfo));
  m_receiveThread = std::make_shared<std::thread>(std::thread(ReceiveDataThread, std::ref(*m_receiverCam), 9000,
    std::ref(m_doneStreaming), m_cpuPinnedImageBuffer, m_gpuImageBuffer, m_stream, m_visibleCameras.back()->m_imageQueue,
    dataQueue, nullptr, nullptr, nullptr));

  // Request the camera to start sending data, showing every 10th frame.
  if (m_client && m_receiverCam) {
    uint16_t port;
    m_receiverCam->GetPort(port);
    m_endpoint = StreamEndpoint(ip, port);
    m_streamingCameraID = cameraID.toUInt();
    SubregionDescription region;
    region.cameraID = m_streamingCameraID;
    region.skipFrames = ui->comboBoxSkip->currentText().toUInt();
    region.startTimeSeconds = 0;
    region.startTimeMicroseconds = 0;
    region.left = 0;
    region.top = 0;
    region.right = width - 1;
    region.bottom = height - 1;
    Status status = m_client->SendCommandPacket(CommandPacketStreamSubregion(m_endpoint, region));
    if (status != OKAY) {
      std::cerr << "Failed to start streaming: " << ErrorMessage(status) << std::endl;
      return;
    }
  }
}
