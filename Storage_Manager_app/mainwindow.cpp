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
#include "mainwindow.h"
#include "ui_mainwindow.h"  // Include the generated header

// Define the version number
const QString VERSION_NUMBER = "1.0.0";

static std::vector<std::string> getIPAddresses()
{
  std::vector<std::string> ipAddresses;

  // Add Localhost.
  ipAddresses.push_back("localhost");

#ifdef _WIN32
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
    : QMainWindow(parent), ui(new Ui::MainWindow)
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

}

void MainWindow::SelectNIC(const QString& nicName)
{
  // Implement the logic for selecting the NIC here
  std::cout << "Selected NIC:" << nicName.toStdString() << std::endl;
  ui->comboBoxCore->clear();
  ui->comboBoxCore->addItem("");
  m_client.reset();

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
    }
    if (threadStatus != OKAY) {
      std::cerr << "Discovery thread status: " << ErrorMessage(threadStatus) << std::endl;
    }
    status = m_client->IdentifiedServers(servers);
    if (status != OKAY) {
      std::cerr << "Failed to get identified servers: " << ErrorMessage(status) << std::endl;
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

MainWindow::~MainWindow()
{
  delete ui;
}
