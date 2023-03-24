#ifndef UASRISK_CLI_CLI_CPP_
#define UASRISK_CLI_CLI_CPP_

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <filesystem>
#include <gdal_priv.h>
#include <cpl_conv.h>
#include <png++/png.hpp>
#include <colormap/colormap.hpp>

#include <uasgroundrisk/risk_analysis/aircraft/AircraftModel.h>
#include "uasgroundrisk/risk_analysis/weather/WeatherMap.h"
#include "uasgroundrisk/map_gen/TemporalPopulationMap.h"
#include "uasgroundrisk/risk_analysis/obstacles/ObstacleMap.h"
#include "uasrisk/ground/GroundRiskVoxelGrid.h"

namespace kuba
{
	extern "C"
	{
#include <zip/zip.h>
	}
}

void help()
{
	std::cout
		<< "uasrisk CLI. Version 0.1. A.Pilko <a.pilko@soton.ac.uk>\n"
		<< "Command line interface for UAS risk analysis and map generation\n"
		<< "See riskmap.template.yml for further instructions and configuration.\n\n"
		<< "Usage:\n"
		<< "\t./uasrisk-cli <path-to-config-yaml>"
		<< std::endl;
}

const std::vector<std::string> OUTPUT_LAYERS{
	"Ground Strike Risk",
	"Ground Fatality Risk"
};

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-result"
/**
 * A quick and dirty utility to translate YAML config files to risk map generation configs.
 * For all your trying-not-to-kill-people needs.
 */
int main(int argc, char* argv[])
{
	// Don't need a complex arg parsing lib here

	if (argc != 2)
	{
		std::cerr << "Incorrect number of args." << std::endl;
		help();
		exit(1);
	}
	const auto configPath = argv[1];
	if (!std::filesystem::exists(configPath))
	{
		std::cerr << "File " << configPath << " does not exist!";
		exit(128);
	}
	const auto configRoot = YAML::LoadFile(configPath);

	GDALAllRegister();
	CPLPushErrorHandler(CPLQuietErrorHandler);
	auto* gtiffDriver = GetGDALDriverManager()->GetDriverByName("GTiff");

	// Wrap in a big try-catch to handle missing/broken stuff in the config
	try
	{
		const auto configName = configRoot["name"].as<std::string>();
		const auto outPath = std::filesystem::path(configRoot["output_path"].as<std::string>()) / configName;
		try
		{
			std::filesystem::create_directories(outPath);
		}
		catch (std::filesystem::filesystem_error& e)
		{
			std::cerr << "Could not create output directory " << outPath << std::endl;
			exit(1);
		}
		const auto tlos = configRoot["tlos"].as<float>();

		std::filesystem::create_directory(outPath / "output");
		const auto kmlDataPath = outPath / "output";
		std::ofstream kmlFile(kmlDataPath / "output.kml");
		kmlFile << "<?xml version='1.0' encoding='UTF-8'?>\n";
		kmlFile << "<kml xmlns='http://www.opengis.net/kml/2.2' xmlns:gx='http://www.google.com/kml/ext/2.2'>\n";
		kmlFile << "<Document>\n";

		// Parse map
		// Bounds
		const auto boundsNode = configRoot["riskmap"]["bounds"];
		const std::array<float, 4> xyBounds{
			boundsNode["south"].as<float>(),
			boundsNode["west"].as<float>(),
			boundsNode["north"].as<float>(),
			boundsNode["east"].as<float>()
		};
		const std::array<float, 6> xyzBounds{
			boundsNode["south"].as<float>(),
			boundsNode["west"].as<float>(),
			boundsNode["floor"].as<float>(),
			boundsNode["north"].as<float>(),
			boundsNode["east"].as<float>(),
			boundsNode["ceiling"].as<float>()
		};

		//Resolutions
		const auto resolutionsNode = configRoot["riskmap"]["resolution"];
		const auto xyRes = resolutionsNode["lateral"].as<float>();
		const auto zRes = resolutionsNode["vertical"].as<float>();


		//Aircraft
		const auto aircraftNode = configRoot["aircraft"];
		ugr::risk::AircraftModel aircraftModel;
		aircraftModel.mass = aircraftNode["mass"].as<double>();
		aircraftModel.length = aircraftNode["length"].as<double>();
		aircraftModel.width = aircraftNode["width"].as<double>();
		aircraftModel.state.velocity << aircraftNode["speed"].as<double>(), 0, 0.1;
		aircraftModel.state.position << 0, 0, 0; // This gets pushed correct values from the risk layering in GRVG
		aircraftModel.failureProb = aircraftNode["failure_prob"].as<double>();
		//Iterate possibly defined descents
		for (const auto descentNode : aircraftNode["descents"])
		{
			if (descentNode.first.as<std::string>() == "parachute")
			{
				aircraftModel.addDescentModel<ugr::risk::ParachuteDescentModel>(
					aircraftNode["descents"]["parachute"]["drag_coeff"].as<double>(),
					aircraftNode["descents"]["parachute"]["chute_area"].as<double>(),
					aircraftNode["descents"]["parachute"]["chute_deploy_time"].as<double>()
				);
			}
			else if (descentNode.first.as<std::string>() == "ballistic")
			{
				aircraftModel.addDescentModel<ugr::risk::BallisticDescentModel>(
					aircraftNode["descents"]["ballistic"]["frontal_area"].as<double>(),
					aircraftNode["descents"]["ballistic"]["drag_coeff"].as<double>()
				);
			}
			else if (descentNode.first.as<std::string>() == "uncontrolled_glide")
			{
				aircraftModel.addDescentModel<ugr::risk::GlideDescentModel>(
					aircraftNode["descents"]["uncontrolled_glide"]["best_glide_airspeed"].as<double>(),
					aircraftNode["descents"]["uncontrolled_glide"]["glide_ratio"].as<double>()
				);
			}
		}

		//Environment
		//Weather
		ugr::risk::WeatherMap weather(xyBounds, xyRes);
		const auto weatherNode = configRoot["environment"]["weather"];
		for (const auto windNode : weatherNode)
		{
			if (windNode.first.as<std::string>() == "wind")
			{
				weather.addConstantWind(
					weatherNode["wind"]["speed"].as<float>(),
					weatherNode["wind"]["bearing"].as<float>()
				);
			}
		}
		//Population
		auto hourOfDay = configRoot["environment"]["population"]["hour"].as<short>();
		//Obstacles
		ugr::risk::ObstacleMap obstacles(xyBounds, xyRes);

		const auto obstaclesNode = configRoot["environment"]["obstacles"];
		for (const auto obstacleNode : obstaclesNode)
		{
			if (obstacleNode.first.as<std::string>() == "buildings")
			{
				obstacles.addBuildingHeights();
			}
		}

		std::cout << "Successfully parsed config file! Starting generation..." << std::endl;
		std::cout << "Evaluating Weather map...\n";
		weather.eval();
		std::cout << "Evaluating Obstacle map...\n";
		obstacles.eval();
		std::cout << "Preparing Population map...\n";
		std::cout << "Downloading OSM data...\n";
		ugr::mapping::TemporalPopulationMap population(xyBounds, xyRes);
		population.setHourOfDay(hourOfDay);
		std::cout << "Evaluating Population map...\n";
		population.eval();

		std::cout << "Preparing Risk map...\n";
		ur::GroundRiskVoxelGrid grvg(xyzBounds, xyRes, zRes, &population, &aircraftModel, &obstacles, &weather);
		std::cout << "Evaluating Risk map ...\n";
		std::cout << "Statistics for each altitude layer will be reported:\n";
		grvg.eval();

		// Work out geotransform for geotiffs later
		const auto vgSize = grvg.getSize();
		double geoTransform[6];
		const auto nsPixelSize = std::abs(xyBounds[2] - xyBounds[0]) / vgSize.y();
		const auto ewPixelSize = std::abs(xyBounds[1] - xyBounds[3]) / vgSize.x();
		geoTransform[0] = xyBounds[1], geoTransform[1] = ewPixelSize, geoTransform[2] = 0, geoTransform[3] =
			xyBounds[2], geoTransform[4] = 0, geoTransform[5] = nsPixelSize;

		// The riskmap is generated by here, so output it in slices
		using MatrixSliceType = Eigen::Matrix<ur::FPScalar, Eigen::Dynamic, Eigen::Dynamic>;

		for (const auto& outputLayer : OUTPUT_LAYERS)
		{
			for (int zIndex = 0; zIndex < grvg.getSize().z(); ++zIndex)
			{
				// Get out the tensor layer and the altitude value for the z index
				const auto tensorLayer = grvg.get(outputLayer);
				const auto layerAltitude = grvg.local2World(0, 0, zIndex).z();
				// Create an identifying filename
				std::ostringstream filenameSS;
				filenameSS << outputLayer << "_z" << std::fixed << std::setprecision(1) << layerAltitude << "m";
				const std::string fileName = filenameSS.str();

				// Slice the tensor for the z layer
				const Eigen::Tensor<ur::FPScalar, 2> tensorChipR2 = tensorLayer.chip(zIndex, 2);

				// Convert to a Matrix type and output as csv
				MatrixSliceType layer = Eigen::Map<const MatrixSliceType>(tensorChipR2.data(), vgSize.x(), vgSize.y());
				const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
				std::ofstream file(outPath / (fileName + ".csv"));
				if (file.is_open())
				{
					file << layer.format(CSVFormat);
					file.close();
				}
				// Reorient it to geotiff write order
				layer = layer.transpose().colwise().reverse().eval();
				// Output as a geotiff
				auto* gtiffDataset =
					gtiffDriver->Create((outPath / (fileName + ".tif")).c_str(),
						layer.cols(),
						layer.rows(),
						1,
						GDT_Float32,
						nullptr);
				gtiffDataset->SetProjection("EPSG:4326");
				gtiffDataset->SetGeoTransform(geoTransform);
				auto* rowBuffer = static_cast<float*>(CPLMalloc(layer.cols() * sizeof(float)));
				for (int row = 0; row < layer.rows(); ++row)
				{
					for (int col = 0; col < layer.cols(); ++col)
					{
						rowBuffer[col] = layer(row, col);
					}
					gtiffDataset->GetRasterBand(1)->RasterIO(GF_Write,
						0,
						row,
						layer.cols(),
						1,
						rowBuffer,
						layer.cols(),
						1,
						GDT_Float32,
						0,
						0);
				}
				GDALClose(gtiffDataset);

				// Convert to a png file then embed it in a kml file and zip up to a kmz archive

				const auto cmap = colormap::palettes.at("reds").rescale(0, tlos);
				png::image<png::rgb_pixel> pngImage(layer.cols(), layer.rows());
				// Create the PNG image data
				for (int r = 0; r < layer.rows(); r++)
				{
					for (int c = 0; c < layer.cols(); c++)
					{
						const float value = layer(r, c);
						const auto cval = cmap(value);
						pngImage.set_pixel(c, r, png::rgb_pixel(
							cval.getRed().getValue(),
							cval.getGreen().getValue(),
							cval.getBlue().getValue()
						));
					}
				}
				pngImage.write((kmlDataPath / (fileName + ".png")).c_str());
				kmlFile << "  <GroundOverlay>\n";
				kmlFile << "    <name>" << outputLayer << " " << std::fixed << std::setprecision(1) << layerAltitude
						<< std::setprecision(9) << "m" << "</name>\n";
				kmlFile << "    <Icon>\n";
				kmlFile << "      <href>" << fileName << ".png" << "</href>\n";
				kmlFile << "    </Icon>\n";
				kmlFile << "    <altitude>" << layerAltitude << "</altitude>\n";
				kmlFile << "    <gx:altitudeMode>absolute</gx:altitudeMode>\n";
				kmlFile << "    <color>44ffffff</color>\n";
				kmlFile << "    <LatLonBox>\n";
				kmlFile << "      <north>" << xyBounds[2] << "</north>\n";
				kmlFile << "      <south>" << xyBounds[0] << "</south>\n";
				kmlFile << "      <east>" << xyBounds[3] << "</east>\n";
				kmlFile << "      <west>" << xyBounds[1] << "</west>\n";
				kmlFile << "    </LatLonBox>\n";
				kmlFile << "  </GroundOverlay>\n";
			}
		}
		kmlFile << "</Document>\n";
		kmlFile << "</kml>\n";
		kmlFile.close();

		struct kuba::zip_t
			* kmzZip = kuba::zip_open((outPath / "output.kmz").c_str(), ZIP_DEFAULT_COMPRESSION_LEVEL, 'w');
		for (const auto& file : std::filesystem::directory_iterator(kmlDataPath))
		{
			if (!file.is_directory())
			{
				kuba::zip_entry_open(kmzZip, file.path().filename().c_str());
				kuba::zip_entry_fwrite(kmzZip, file.path().c_str());
				kuba::zip_entry_close(kmzZip);
			}
		}
		kuba::zip_close(kmzZip);

		grvg.writeToNetCDF(outPath / "ground_risk_voxel_grid.nc");
	}
	catch (std::invalid_argument& e)
	{
		std::cerr << "Config file failed to parse: " << e.what() << std::endl;
		exit(1);
	}
	catch (YAML::InvalidNode& e)
	{
		std::cerr << "Config file failed to parse: " << e.what() << std::endl;
		exit(1);
	}

	GDALDestroyDriver(gtiffDriver);

	return 0;

}
#pragma clang diagnostic pop

#endif //UASRISK_CLI_CLI_CPP_
