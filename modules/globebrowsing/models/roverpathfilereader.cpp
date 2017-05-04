/*****************************************************************************************
*                                                                                       *
* OpenSpace                                                                             *
*                                                                                       *
* Copyright (c) 2014-2017                                                               *
*                                                                                       *
* Permission is hereby granted, free of charge, to any person obtaining a copy of this  *
* software and associated documentation files (the "Software"), to deal in the Software *
* without restriction, including without limitation the rights to use, copy, modify,    *
* merge, publish, distribute, sublicense, and/or sell copies of the Software, and to    *
* permit persons to whom the Software is furnished to do so, subject to the following   *
* conditions:                                                                           *
*                                                                                       *
* The above copyright notice and this permission notice shall be included in all copies *
* or substantial portions of the Software.                                              *
*                                                                                       *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,   *
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A         *
* PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT    *
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF  *
* CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE  *
* OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                         *
****************************************************************************************/

#include <modules/globebrowsing/models/roverpathfilereader.h>
#include <ghoul/logging/logmanager.h>
#include <ghoul/filesystem/filesystem.h>

#include <fstream>
#include <gdal_priv.h>
#include "ogrsf_frmts.h"

namespace {
	const std::string _loggerCat		= "RoverPathFileReader";
	const char* keyRoverLocationPath	= "RoverLocationPath";
	const char* keyModelPath			= "ModelPath";
}

namespace openspace {
namespace globebrowsing {
	
std::vector<SubSite> RoverPathFileReader::extractAllSubsites(const ghoul::Dictionary dictionary) {
	std::string roverLocationFilePath;
	if (!dictionary.getValue(keyRoverLocationPath, roverLocationFilePath))
		throw ghoul::RuntimeError(std::string(keyRoverLocationPath) + " must be specified!");

	std::fstream in(roverLocationFilePath.c_str());

	if (!in.is_open())
		throw ghoul::FileNotFoundError(roverLocationFilePath);

	GDALDataset *poDS;
	poDS = (GDALDataset*)GDALOpenEx(roverLocationFilePath.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
	if (poDS == NULL) {
		LERROR("Could not open .shp file");
	}

	OGRLayer *poLayer = poDS->GetLayerByName("rover_locations");

	OGRFeature *poFeature;
	poLayer->ResetReading();

	int currentSite = 0;
	double siteLat;
	double siteLon;
	std::vector<SubSite> subSites;

	while ((poFeature = poLayer->GetNextFeature()) != NULL) {

		// Extract coordinates from OGR
		std::string frame = poFeature->GetFieldAsString("frame");
		std::string site = poFeature->GetFieldAsString("site");
		std::string drive = poFeature->GetFieldAsString("drive");
		double lat = poFeature->GetFieldAsDouble("plcl");
		double lon = poFeature->GetFieldAsDouble("longitude");

		// Saves all coordinates for rendering the path and only site coordinates for rendering sites.
		// GetFieldAsDouble resturns zero (0) if field is empty.
		if (lat != 0 && lon != 0) {
			if (frame == "SITE") {
				siteLat = lat;
				siteLon = lon;
			}

			std::string type = "site";
			SubSite subSite;
			subSite.site = convertString(site, type);
			type = "drive";
			subSite.drive = convertString(drive, type);
			subSite.lat = lat;
			subSite.lon = lon;
			subSite.frame = frame;
			subSite.siteLat = siteLat;
			subSite.siteLon = siteLon;

			subSites.push_back(subSite);
		}
		OGRFeature::DestroyFeature(poFeature);
	}
	GDALClose(poDS);

	return subSites;
}

std::vector<SubSite> RoverPathFileReader::extractSubsitesWithModels(const ghoul::Dictionary dictionary) {
	std::string roverLocationFilePath;
	if (!dictionary.getValue(keyRoverLocationPath, roverLocationFilePath))
		throw ghoul::RuntimeError(std::string(keyRoverLocationPath) + " must be specified!");

	std::string surfaceModelFilePath;
	if (!dictionary.getValue(keyModelPath, surfaceModelFilePath))
		throw ghoul::RuntimeError(std::string(keyModelPath) + " must be specified!");

	ghoul::Dictionary tempDictionary;
	tempDictionary.setValue(keyRoverLocationPath, roverLocationFilePath);
	std::vector<SubSite> allSubsites = extractAllSubsites(tempDictionary);

	std::vector<SubSite> subSitesWithModels;

	int counter = 0;
	for (auto subSite : allSubsites) {
		std::string pathToDriveFolder;
		std::string site = convertString(subSite.site, "site");
		std::string drive = convertString(subSite.drive, "drive");
		pathToDriveFolder = surfaceModelFilePath + "site" + site + "/" + "drive" + drive;
		bool exists = FileSys.directoryExists(pathToDriveFolder);
		if(exists) {
			subSitesWithModels.push_back(subSite);
		}
	}

	return subSitesWithModels;
}

std::string RoverPathFileReader::convertString(const std::string sitenr, const std::string type) {
	int k = std::stoi(sitenr);

	std::string temp;
	if (type == "site") {
		if (k < 10) {
			temp = "00" + std::to_string(k);
		}
		else if (k < 100) {
			temp = "0" + std::to_string(k);
		}
	}
	else if (type == "drive") {
		if (k < 10) {
			temp = "000" + std::to_string(k);
		}
		else if (k < 100) {
			temp = "00" + std::to_string(k);
		}
		else if (k < 1000) {
			temp = "0" + std::to_string(k);
		}
		else {
			temp = std::to_string(k);
		}
	}
	return temp;
}

} // namespace globebrowsing
} // namespace openspace
