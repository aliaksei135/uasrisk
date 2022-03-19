#include <array>
#include <vector>
#include <gtest/gtest.h>

#include <uasrisk/environment/VoxelGrid.h>
#include <uasrisk/environment/VoxelGridBuilder.h>
#include <uasrisk/air/io/openaip/OpenAIPReader.h>
#include <uasrisk/air/io/opensky/OpenSkyCsvReader.h>

#include "uasgroundrisk/map_gen/PopulationMap.h"
#include "uasgroundrisk/risk_analysis/RiskMap.h"
#include "uasgroundrisk/risk_analysis/aircraft/AircraftDescentModel.h"
#include "uasgroundrisk/risk_analysis/aircraft/AircraftStateModel.h"
#include "uasgroundrisk/risk_analysis/weather/WeatherMap.h"

#include <omp.h>

#include "uasgroundrisk/map_gen/TemporalPopulationMap.h"


TEST(GroundRiskIntegrationTests, FullTest)
{
    constexpr int xyRes = 60;
    constexpr int zRes = 20;
    const std::array<float, 4> xyBounds{
        50.9065510f, -1.4500237f, 50.9517765f,
        -1.3419628f
    };
    const std::array<float, 6> xyzBounds{
        xyBounds[0], xyBounds[1], 50, xyBounds[2],
        xyBounds[3], 250
    };

    ugr::risk::AircraftModel aircraft;
    // State set inside sweep loop

    aircraft.mass = 50;
    aircraft.length = 5;
    aircraft.width = 5;

    aircraft.addDescentModel<ugr::risk::GlideDescentModel>(21, 15);
    aircraft.addDescentModel<ugr::risk::BallisticDescentModel>(0.6 * 0.6, 0.8);
    // aircraft.descents.emplace_back(
    //     std::unique_ptr<ugr::risk::ParachuteDescentModel>(
    //         new ugr::risk::ParachuteDescentModel(90, 2.8, 3.2, 1.2, 12.5, 2)));

    ugr::risk::WeatherMap weather(xyBounds, xyRes);
    // weather.addGribs();
    weather.addConstantWind(5, 220);
    // weather.addWeatherAvoidanceZone(badWeatherGridIter.begin(), badWeatherGridIter.end());
    weather.eval();

    ugr::mapping::TemporalPopulationMap population(xyBounds, xyRes);
    population.setHourOfDay(12);
    population.eval();

    ugr::risk::ObstacleMap obstacleMap(xyBounds, xyRes);
    obstacleMap.addBuildingHeights();
    obstacleMap.eval();

    auto* vg = new ur::VoxelGrid(xyzBounds, xyRes, zRes);
    ur::VoxelGridBuilder vgb(vg);

    const ur::io::OpenSkyCsvReader osReader;
    const auto trajs = osReader.readFile("states.csv");

    const ur::io::OpenAIPReader oaReader;
    const auto airspaces = oaReader.readFile("openaip_airspace_uk.aip");

    vgb.handleTrajectory(trajs);
    vgb.handleBlockingPolygon(airspaces);

    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

    std::ofstream file(
        "population_map.csv");
    if (file.is_open())
    {
        file << population.get("Population Density").format(CSVFormat);
        file.close();
    }

    const auto& vgSize = vg->getSize();
    std::vector<ur::FPScalar> alts;
    alts.reserve(vgSize[2]);
    for (int i = 0; i < vgSize[2]; ++i)
        alts.emplace_back(vg->local2World(0, 0, i)[2]);

    std::vector<ur::FPScalar> lons(vgSize[0]);
    std::vector<ur::FPScalar> lats(vgSize[1]);
    for (int i = 0; i < vgSize[0]; ++i)
        lons[i] = vg->local2World(0, i, 0)[0];

    for (int i = 0; i < vgSize[1]; ++i)
        lats[i] = vg->local2World(i, 0, 0)[1];

    for (int k = 0; k < alts.size(); ++k)
    {
        aircraft.state.position << 0, 0, alts[k];
        aircraft.state.velocity << 20, 20, 0;

        ugr::risk::RiskMap rm(population, aircraft, obstacleMap, weather);
        rm.SetAnyHeading(true);
        rm.generateMap({ugr::risk::RiskType::FATALITY});
        // const auto& gsr = rm.get("Glide Strike Risk");
        // auto& bsr = rm.get("Ballistic Strike Risk");
        // std::cout << "Risk for altitude: " << alts[k] << "\n";
        // std::cout << "Glide Risk max: " << gsr.maxCoeff() << "\n";
        // std::cout << "Ballistic Risk max: " << bsr.maxCoeff() << "\n";
        // std::cout << "Glide Risk mean: " << gsr.mean() << "\n";
        // std::cout << "Ballistic Risk mean: " << bsr.mean() << "\n";
        const auto& fatalityRisk = rm.get("Fatality Risk");


        // std::cout << fatalityRisk << " max " << std::scientific << fatalityRisk.maxCoeff()
        // << std::endl;

        std::ofstream file("fatality_alt_" + std::to_string(alts[k]) + ".csv");
        if (file.is_open())
        {
            file << fatalityRisk.format(CSVFormat);
            file.close();
        }


        assert(vgSize[0] == rm.getSize()[0]);
        assert(vgSize[1] == rm.getSize()[1]);
#pragma omp parallel for collapse(2)
        for (int i = 0; i < vgSize[0]; ++i)
        {
            for (int j = 0; j < vgSize[1]; ++j)
            {
                auto val = fatalityRisk(i, j);
#pragma omp critical
                vg->at("Ground Risk", {vgSize[0] - i, j, k}) = val;

                // const auto& worldCoord = vg->local2World(i, j, k);
                // const ugr::gridmap::Position pos(worldCoord[0], worldCoord[1]);
                // const ugr::gridmap::Index& rmIdx = rm.world2Local(pos);
                // if (rm.isInBounds(rmIdx))
                // {
                // 	auto& brisk = rm.at("Ballistic Strike Risk", rmIdx);
                // 	if (isnan(brisk))
                // 		brisk = 0;
                // 	const auto& grisk = rm.at("Glide Strike Risk", rmIdx);
                // 	// std::cout << "B: " << brisk << " G: " << grisk << "\n";
                // 	// #pragma omp critical
                // 	vg->at("Ground Risk", {i, j, k}) = brisk + grisk;
                // }
            }
        }
    }

    std::cout << "All Finished!\n";

    const auto& mar = vg->get("Air Risk").maximum();
    std::cout << "Max Air Risk: " << mar << "\n";
    const auto& mgr = vg->get("Ground Risk").maximum();
    std::cout << "Max Ground Risk: " << mgr << "\n";
    const auto& aar = vg->get("Air Risk").mean();
    std::cout << "Mean Air Risk: " << aar << "\n";
    const auto& agr = vg->get("Ground Risk").mean();
    std::cout << "Mean Ground Risk: " << agr << "\n";

    vg->writeToNetCDF("CombinedVoxelGrid.nc");

    std::cout << "Written to netCDF...\n";
}
