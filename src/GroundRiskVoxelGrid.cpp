#include "uasrisk/ground/GroundRiskVoxelGrid.h"


ur::GroundRiskVoxelGrid::GroundRiskVoxelGrid(const std::array<ur::FPScalar, 6>& bounds, const ur::FPScalar xyRes,
                                             const ur::FPScalar zRes, const char* const worldSrs,
                                             ugr::mapping::PopulationMap* populationMap,
                                             ugr::risk::AircraftModel* aircraftModel,
                                             ugr::risk::ObstacleMap* obstacleMap,
                                             ugr::risk::WeatherMap* weather):
    VoxelGrid(bounds, xyRes, zRes, worldSrs), populationMap(populationMap), aircraftModel(aircraftModel),
    obstacleMap(obstacleMap), weatherMap(weather)
{
}

void ur::GroundRiskVoxelGrid::eval()
{
    add("Ground Strike Risk", 0);
    add("Ground Fatality Risk", 0);


    auto groundStrikeRiskTensor = get("Ground Strike Risk");
    auto groundFatalityRiskTensor = get("Ground Fatality Risk");

    for (int i = 0; i < sizeZ; ++i)
    {
        // Get the altitude value for this layer
        const auto altitude = local2World(0, 0, i)[2];

        // Create a thread local aircraft model as we modify the altitude on it
        // ugr::risk::AircraftModel localAircraftModel(aircraftModel);

        // Create a riskmap object 
        ugr::risk::RiskMap altRiskMap(*populationMap, *aircraftModel, *obstacleMap, *weatherMap);
        // Set the state altitude to the altitude of this layer
        aircraftModel->state.position(2) = altitude;
        //TODO Allow propogation of heading
        // Set any heading for now
        altRiskMap.SetAnyHeading(true);
        // Evaluate the ground risk map for this altitude
        altRiskMap.eval();

        // At this point, altRiskMap should have 2 layers we want to propogate to the voxel grid: "Strike Risk" and "Fatality Risk"
        const auto strikeRiskMat = altRiskMap.get("Strike Risk");
        const auto fatalityRiskMat = altRiskMap.get("Fatality Risk");

        // auto strikeRisk2Tensor = Eigen::TensorMap<const Eigen::Tensor<const ur::FPScalar, 2>>(strikeRiskMat.data(), sizeX, sizeY);

        // Make sure the shapes are compatible otherwise inserting without runtime size checks won't work
        assert(strikeRiskMat.cols() == sizeX,
               "Ground Risk RiskMap and GroundRiskVoxelGrid have incompatible layer shapes");
        assert(strikeRiskMat.rows() == sizeY,
               "Ground Risk RiskMap and GroundRiskVoxelGrid have incompatible layer shapes");

        // Insert the layers into the tensor
        groundStrikeRiskTensor.chip(i, 2) = Eigen::TensorMap<const Eigen::Tensor<const ur::FPScalar, 2>>(
            strikeRiskMat.data(), sizeX, sizeY);
        groundFatalityRiskTensor.chip(i, 2) = Eigen::TensorMap<const Eigen::Tensor<const ur::FPScalar, 2>>(
            fatalityRiskMat.data(), sizeX, sizeY);

        std::cout<< strikeRiskMat.sum();
    }

    add("Ground Risk", groundFatalityRiskTensor);
}
