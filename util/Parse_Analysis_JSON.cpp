/*
 * Copyright (C) 2026: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <iostream>
#include <vector>
#include <string>
#include "Analysis.h"

static void usage(const char* progName)
{
  std::cerr << "Usage: " << progName << " <JSONFile>" << std::endl;
}

int main(int argc, char** argv)
{
  std::string JSONFileName;
  if (argc != 2) {
    usage(argv[0]);
    exit(1);
  }
  JSONFileName = argv[1];

  // Run inside a block so that objects are cleaned up before exiting.
  try {
    // Open a JSON reader for the specified file.
    std::shared_ptr<asdp::JSONStringReceiver> jsonReader;
    std::string url = "file://" + JSONFileName;
    asdp::Status status = asdp::JSONStringReceiver::Create(url, jsonReader);
    if (status != asdp::Status::OKAY) {
      std::cerr << "Error opening JSON file: " << asdp::ErrorMessage(status) << std::endl;
      return 2;
    }

    // Read each entry and parse it as an AnalysisReport, then print some info about it.
    std::string jsonString;
    do {
      status = jsonReader->Receive(1.0, jsonString);
      if (status == asdp::Status::OKAY) {
        try {
          asdp::analysis::AnalysisReport report(jsonString);
          std::cout << "CamID: " << report.CamID
                    << ", Timestamp: " << report.Timestamp.seconds << "." << report.Timestamp.microseconds
                    << ", Name: " << report.Name;
          if (report.Loc) {
            std::cout << ", Loc: [" << (*(report.Loc))[0] << ", " << (*(report.Loc))[1] << "]";
          }
          if (report.Rect) {
            std::cout << ", Rect: [" << (*(report.Rect))[0] << ", " << (*(report.Rect))[1] << "]";
          }
          if (report.Vel) {
            std::cout << ", Vel: [" << (*(report.Vel))[0] << ", " << (*(report.Vel))[1] << "]";
          }
          if (!report.Class.empty()) {
            std::cout << ", Classes: ";
            for (const auto& cls : report.Class) {
              std::cout << "[";
              if (cls.Type) {
                std::cout << "Type: " << *(cls.Type) << " ";
              }
              if (cls.Chance) {
                std::cout << "Chance: " << *(cls.Chance) << " ";
              }
              if (cls.IFF) {
                std::cout << "IFF: " << *(cls.IFF) << " ";
              }
              std::cout << "] ";
            }
          }
          std::cout << std::endl;
        } catch (const std::exception& e) {
          std::cerr << "Error parsing AnalysisReport: " << e.what() << std::endl;
        }
      } else if (status != asdp::Status::TIMEOUT) {
        std::cerr << "Error reading JSON entry: " << asdp::ErrorMessage(status) << std::endl;
        return 3;
      }
    } while (status == asdp::Status::OKAY);


  } catch (const std::exception& e) {
    std::cerr << "Error parsing file: " << e.what() << std::endl;
    return 1;
  }
  
  // Done
  return 0;
}
