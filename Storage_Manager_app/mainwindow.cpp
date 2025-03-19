/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

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

// Define the version number
const QString VERSION_NUMBER = "0.9.0";

static std::vector<std::string> getIPAddresses()
{
  std::vector<std::string> ipAddresses;

  // Add Localhost.
  ipAddresses.push_back("localhost");

#ifdef _WIN32
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
      if (ipAddr->IpAddress.String[0] != '0') {
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
  setWindowTitle(QString("Storage Manager v%1").arg(VERSION_NUMBER));

  // Look up the network interfaces that are available and add them to the combo box.
  std::vector<std::string> ipAddresses = getIPAddresses();
  ui->comboBoxNIC->addItem("");
  for (const std::string& ipAddress : ipAddresses) {
    ui->comboBoxNIC->addItem(QString::fromStdString(ipAddress));
  }

  // Hook up the timer to the periodic task.
  connect(m_timer.get(), &QTimer::timeout, this, &MainWindow::PeriodicTask);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::SelectNIC(const QString& nicName)
{
  ResetNIC();

  // Implement the logic for selecting the NIC here
  std::cout << "Selected NIC: " << nicName.toStdString() << std::endl;

  if (!nicName.isEmpty()) {
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
    std::vector<std::string> servers;
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
    for (const std::string& server : servers) {
      ui->comboBoxCore->addItem(QString::fromStdString(server));
      std::cout << "  " << server << std::endl;
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
  m_receiver.reset();
  m_features.clear();
  m_cameras.clear();
  m_streams.clear();
  emit ShowControls(false);
  emit SetSerialNumber("");
  m_timer->stop();
}

void MainWindow::SelectServer(const QString& coreURL)
{
  ResetServer();

  // Implement the logic for selecting the core here
  std::cout << "Selected Core: " << coreURL.toStdString() << std::endl;

  if (!coreURL.isEmpty()) {
    // Connect to the server.
    std::cout << "Connecting to " << coreURL.toStdString() << std::endl;
    uint16_t major, minor, patch;
    Status status = m_client->ConnectToServer(coreURL.toStdString(), major, minor, patch);
    if (status != OKAY) {
      std::cerr << "Failed to connect to server: " << ErrorMessage(status) << std::endl;
      return;
    }
    std::cout << "  Connected to server version " << major << "." << minor << "." << patch << std::endl;
    uint32_t serialNumber;
    status = m_client->GetServerSerialNumber(serialNumber);
    if (status != OKAY) {
      std::cerr << "Failed to get server serial number: " << ErrorMessage(status) << std::endl;
      return;
    }
    std::cout << "  Connected to server with serial number " << serialNumber << std::endl;
    SetSerialNumber(std::to_string(serialNumber).c_str());

    // Get the main stream receiver
    status = m_client->GetMainStreamReceiver(m_receiver);
    if (status != OKAY) {
      std::cerr << "Failed to get main stream receiver: " << ErrorMessage(status) << std::endl;
      return;
    }
  }

  // Show the controls.
  emit ShowControls(true);

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

  // Delete the display if it is no longer valid.
  if (m_display) {
    if (m_display->GetStatus() != "") {
      m_display.reset();
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
      Status status = m_client->SendCommandPacket(CommandPacketEraseStoredStream(streamID.toUInt()));
      if (status != OKAY) {
        std::cerr << "Failed to delete stream: " << ErrorMessage(status) << std::endl;
        return;
      }
    }
  }
}

void MainWindow::ViewCamera(const QString& cameraID)
{
  std::cout << "Viewing Camera: " << cameraID.toStdString() << std::endl;

  std::shared_ptr<DisplayTexture> displayTexture;
  std::vector< std::shared_ptr<CameraRenderInfo> > visibleCameras;
  std::shared_ptr<CompositeCameras> composite;

  /// @todo
}
