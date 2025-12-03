/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "Analysis.h"
#include <stdexcept>
#include <set>
#include <nlohmann/json.hpp>

using namespace asdp::analysis;
using json = nlohmann::json;

AnalysisReport::AnalysisReport(std::string jsonString)
{
  // Convert the incoming string to a JSON object.
  json jsonObject;
  try {
    jsonObject = json::parse(jsonString);
  } catch (json::parse_error& e) {
    throw std::invalid_argument("Malformed JSON string: " + std::string(e.what()));
  }

  // Extract required fields
  try {
    CamID = jsonObject.at("CamID").get<uint32_t>();
    json timeJson = jsonObject.at("Timestamp");
    Timestamp.seconds = timeJson[0];
    Timestamp.microseconds = timeJson[1];
    Name = jsonObject.at("Name").get<std::string>();
  } catch (json::out_of_range& e) {
    throw std::invalid_argument("Missing required field: " + std::string(e.what()));
  }

  // Extract optional fields
  try {
    if (jsonObject.contains("Loc")) {
      Loc = std::make_shared<std::array<float, 2>>(jsonObject.at("Loc").get<std::array<float, 2>>());
    }
  } catch (...) { }

  try {
    if (jsonObject.contains("Rect")) {
      Rect = std::make_shared<std::array<float, 2>>(jsonObject.at("Rect").get<std::array<float, 2>>());
    }
  } catch (...) {}

  try {
    if (jsonObject.contains("Vel")) {
      Vel = std::make_shared<std::vector<std::array<float, 3>>>(jsonObject.at("Vel").get<std::vector<std::array<float, 3>>>());
    }
  } catch (...) {}

  try {
    if (jsonObject.contains("Class")) {
      for (const auto& classJson : jsonObject.at("Class")) {
        Classification c;
        if (classJson.contains("Type")) {
          c.Type = std::make_shared<std::string>(classJson.at("Type").get<std::string>());
        }
        if (classJson.contains("Chance")) {
          c.Chance = std::make_shared<float>(classJson.at("Chance").get<float>());
        }
        if (classJson.contains("IFF")) {
          c.IFF =  std::make_shared<std::string>(classJson.at("IFF").get<std::string>());
        }

        Class.push_back(c);
      }
    }
  } catch (...) {}
}

asdp::render::CompositeCameras::Annotation AnalysisReport::ConvertToAnnotation(float chanceThreshold, float alpha)
{
  asdp::render::CompositeCameras::Annotation annotation;

  // If we have classifications and all of the classifications have Chance fields and all of the filled in
  // Chance fields are below threshold, return an empty annotation.
  if (!Class.empty()) {
    // We have classifications, check their Chance fields.
    bool allBelowThreshold = true;
    for (const auto& c : Class) {
      // If we don't have a chance field, or if the chance is above threshold, we keep the annotation.
      if (!c.Chance || *c.Chance >= chanceThreshold) {
        allBelowThreshold = false;
        break;
      }
    }
    if (allBelowThreshold) {
      return annotation; // Return empty annotation
    }
  }

  // Set the camera ID.
  annotation.cameraID = CamID;

  // Determine the color based on the values in the IFF fields (if there are any). Use a set to determine which
  // values are present. If the set has no value or more than one value, use white. If there is a single value,
  // then use the corresponding color: red for "foe", green for "friend", yellow for "neutral".
  std::set<std::string> iffValues;
  for (const auto& c : Class) {
    if (c.IFF) {
      iffValues.insert(*c.IFF);
    }
  }
  annotation.color = { 1.0f, 1.0f, 1.0f, alpha }; // White
  if (iffValues.size() == 1) {
    std::string iff = *iffValues.begin();
    if (iff == "friend") {
      annotation.color = { 0.0f, 1.0f, 0.0f, alpha }; // Green
    } else if (iff == "foe") {
      annotation.color = { 1.0f, 0.0f, 0.0f, alpha }; // Red
    } else if (iff == "neutral") {
      annotation.color = { 1.0f, 1.0f, 0.0f, alpha }; // Yellow
    }
  }

  // Set default UV coordinates to the center of the image.  If we have a location, use it.
  annotation.uv = { 0.5f, 0.5f };
  if (Loc) {
    annotation.uv = { (*Loc)[0], (*Loc)[1] };
  }

  // If we have a rectangle, set the bounding box.
  if (Rect) {
    annotation.bbox = std::make_shared< std::array<float, 2> >(*Rect);
  }

  // Set the label, starting with the base name.
  annotation.label = Name;

  // Sort the classifications by chance, highest first.
  auto sortedClassifications = Class;
  std::sort(sortedClassifications.begin(), sortedClassifications.end(),
    [](const Classification& a, const Classification& b) {
      float chanceA = a.Chance ? *a.Chance : 1.0f;
      float chanceB = b.Chance ? *b.Chance : 1.0f;
      return chanceA > chanceB;
    });

  // If there is just one classification, add it on a new line with two spaces before it.
  // Otherwise, add each on a new line with two spaces before and a question mark at the end.
  if (sortedClassifications.size() == 1) {
    const auto& c = sortedClassifications[0];
    if (c.Type) {
      annotation.label += "\n  " + *c.Type;
    }
  } else {
    for (const auto& c : sortedClassifications) {
      if (c.Type) {
        annotation.label += "\n  " + *c.Type + " ?";
      }
    }
  }

  return annotation;
}

std::string asdp::analysis::AnalysisReport::Test()
{
  try {
    // Create a sample JSON string
    std::string sampleJson = R"({
      "CamID": 1,
      "Timestamp": [1625078400, 500000],
      "Name": "TestAnalysis",
      "Loc": [34.05, -118.25],
      "Rect": [100.0, 200.0],
      "Vel": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
      "Class": [
        {"Type": "Car", "Chance": 0.95, "IFF": "friend"},
        {"Type": "Person", "Chance": 0.85}
      ]
    })";

    // Parse the JSON string
    AnalysisReport report(sampleJson);

    // Validate parsed data
    if (report.CamID != 1) return "CamID mismatch";
    if (report.Timestamp.seconds != 1625078400 || report.Timestamp.microseconds != 500000) return "Timestamp mismatch";
    if (report.Name != "TestAnalysis") return "Name mismatch";
    if (!report.Loc || (*report.Loc)[0] != 34.05f || (*report.Loc)[1] != -118.25f) return "Loc mismatch";
    if (!report.Rect || (*report.Rect)[0] != 100.0f || (*report.Rect)[1] != 200.0f) return "Rect mismatch";
    if (!report.Vel || report.Vel->size() != 2) return "Vel size mismatch";
    if (report.Class.size() != 2) return "Class size mismatch";
    if (!report.Class[0].Type || *report.Class[0].Type != "Car") return "Class[0] Type mismatch";
    if (!report.Class[0].Chance || *report.Class[0].Chance != 0.95f) return "Class[0] Chance mismatch";
    if (!report.Class[0].IFF || *report.Class[0].IFF != "friend") return "Class[0] IFF mismatch";
    if (!report.Class[1].Type || *report.Class[1].Type != "Person") return "Class[1] Type mismatch";
    if (!report.Class[1].Chance || *report.Class[1].Chance != 0.85f) return "Class[1] Chance mismatch";
    if (report.Class[1].IFF != nullptr) return "Class[1] IFF should be nullptr";

    // Make another sample JSON with no optional fields.
    std::string minimalJson = R"({
      "CamID": 2,
      "Timestamp": [1625078500, 0],
      "Name": "MinimalAnalysis"
    })";

    // Parse the minimal JSON string
    AnalysisReport minimalReport(minimalJson);

    // Validate minimal parsed data
    if (minimalReport.CamID != 2) return "Minimal CamID mismatch";
    if (minimalReport.Timestamp.seconds != 1625078500 || minimalReport.Timestamp.microseconds != 0) return "Minimal Timestamp mismatch";
    if (minimalReport.Name != "MinimalAnalysis") return "Minimal Name mismatch";
    if (minimalReport.Loc != nullptr) return "Minimal Loc should be nullptr";
    if (minimalReport.Rect != nullptr) return "Minimal Rect should be nullptr";
    if (minimalReport.Vel != nullptr) return "Minimal Vel should be nullptr";
    if (!minimalReport.Class.empty()) return "Minimal Class should be empty";

    // Make a sample JSON with some required fields missing to test error handling.
    std::string missingFieldsJson = R"({
      "CamID": 3,
      "Name": "MissingTimestampAnalysis"
    })";
    try {
      AnalysisReport missingFieldsReport(missingFieldsJson);
      return "Missing required fields did not throw an exception";
    } catch (const std::invalid_argument&) {
      // Expected exception
    }

    // Make a malformed JSON string to test error handling.
    std::string malformedJson = R"({
      "CamID": 3,
      "Timestamp": [1625078600, 0],
      "Name": "MalformedAnalysis",
      "Loc": [34.05, -118.25
    })"; // Missing closing bracket for Loc array
    try {
      AnalysisReport malformedReport(malformedJson);
      return "Malformed JSON did not throw an exception";
    } catch (const std::invalid_argument&) {
      // Expected exception
    }

    // Convert the first report to an annotation and validate.
    auto annotation = report.ConvertToAnnotation(0.9f, 0.8f);
    if (annotation.cameraID != 1) return "Annotation cameraID mismatch";
    if (annotation.color[0] != 0.0f || annotation.color[1] != 1.0f || annotation.color[2] != 0.0f ||
        annotation.color[3] != 0.8f) {
      return "Annotation color mismatch";
    }
    if (annotation.uv[0] != 34.05f || annotation.uv[1] != -118.25f) return "Annotation uv mismatch";
    if (!annotation.bbox || (*annotation.bbox)[0] != 100.0f || (*annotation.bbox)[1] != 200.0f) return "Annotation bbox mismatch";
    if (annotation.label.find("TestAnalysis") == std::string::npos) return "Annotation label missing base name";
    if (annotation.label.find("Car") == std::string::npos) return "Annotation label missing classification";
    if (annotation.label.find("Person") == std::string::npos) return "Annotation label missing classification";

    // Retry the conversion with a higher chance threshold to exclude all classifications.
    auto annotationFiltered = report.ConvertToAnnotation(1.0f);
    if (annotationFiltered.label != "") return "Annotation should be empty due to chance threshold";

    // Make a new report whose classifications have no Chance fields and verify that it is converted.
    std::string noChanceJson = R"({
      "CamID": 4,
      "Timestamp": [1625078700, 0],
      "Name": "NoChanceAnalysis",
      "Class": [
        {"Type": "Bicycle", "IFF": "neutral"},
        {"Type": "Dog"}
      ]
    })";
    AnalysisReport noChanceReport(noChanceJson);
    auto annotationNoChance = noChanceReport.ConvertToAnnotation(0.5f);
    if (annotationNoChance.label.find("NoChanceAnalysis") == std::string::npos) return "NoChance Annotation label missing base name";
    if (annotationNoChance.label.find("Bicycle") == std::string::npos) return "NoChance Annotation label missing classification";
    if (annotationNoChance.label.find("Dog") == std::string::npos) return "NoChance Annotation label missing classification";
    if (annotationNoChance.color[0] != 1.0f || annotationNoChance.color[1] != 1.0f ||
        annotationNoChance.color[2] != 0.0f || annotationNoChance.color[3] != 1.0f) {
      return "NoChance Annotation color mismatch";
    }

  } catch (const std::exception& e) {
    return std::string("Exception during test: ") + e.what();
  }

  return ""; // Success
}
