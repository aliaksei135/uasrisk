#include "uasrisk/ground/GroundRiskVoxelGrid.h"
#include <cassert>


ur::GroundRiskVoxelGrid::GroundRiskVoxelGrid(const std::array<ur::FPScalar, 6>& bounds, const ur::FPScalar xyRes,
                                             const ur::FPScalar zRes,
                                             ugr::mapping::PopulationMap* populationMap,
                                             ugr::risk::AircraftModel* aircraftModel,
                                             ugr::risk::ObstacleMap* obstacleMap,
                                             ugr::risk::WeatherMap* weather):
	VoxelGrid(bounds, xyRes, zRes, "EPSG:4326"), populationMap(populationMap), aircraftModel(aircraftModel),
	obstacleMap(obstacleMap), weatherMap(weather)
{
}

void ur::GroundRiskVoxelGrid::eval()
{
	add("Ground Strike Risk", 0);
	add("Ground Fatality Risk", 0);
	add("Ground Fatality Risk", 0);


	auto& groundStrikeRiskTensor = get("Ground Strike Risk");
	auto& groundFatalityRiskTensor = get("Ground Fatality Risk");

	for (int z = 0; z < sizeZ; ++z)
	{
		// Get the altitude value for this layer
		const auto altitude = local2World(0, 0, z)[2];

		// Set the state altitude to the altitude of this layer
		aircraftModel->state.position(2) = altitude;

		// Create a thread local aircraft model as we modify the altitude on it
		// ugr::risk::AircraftModel localAircraftModel(aircraftModel);

		// Create a riskmap object 
		ugr::risk::RiskMap altRiskMap(*populationMap, *aircraftModel, *obstacleMap, *weatherMap);

		//TODO Allow propogation of heading
		// Set any heading for now
		altRiskMap.SetAnyHeading(true);

		// Evaluate the ground risk map for this altitude
		altRiskMap.eval();

		// At this point, altRiskMap should have 2 layers we want to propogate to the voxel grid: "Strike Risk" and "Fatality Risk"
		const auto& strikeRiskMat = altRiskMap.get("Strike Risk");
		const auto& fatalityRiskMat = altRiskMap.get("Fatality Risk");

		// auto strikeRisk2Tensor = Eigen::TensorMap<const Eigen::Tensor<const ur::FPScalar, 2>>(strikeRiskMat.data(), sizeX, sizeY);

		// Make sure the shapes are compatible otherwise inserting without runtime size checks won't work
		assert(strikeRiskMat.cols() == sizeX);
		assert(strikeRiskMat.rows() == sizeY);

		// Insert the layers into the tensor
		// groundStrikeRiskTensor.chip(z, 2) = Eigen::TensorMap<const Eigen::Tensor<const ur::FPScalar, 2>>(
		//     strikeRiskMat.data(), sizeX, sizeY);
		// groundFatalityRiskTensor.chip(z, 2) = Eigen::TensorMap<const Eigen::Tensor<const ur::FPScalar, 2>>(
		//     fatalityRiskMat.data(), sizeX, sizeY);

		for (int x = 0; x < sizeX; ++x)
		{
			for (int y = 0; y < sizeY; ++y)
			{
				const auto sr = strikeRiskMat(x, y);
				const auto fr = fatalityRiskMat(x, y);
				if (!isnan(sr))
					groundStrikeRiskTensor(x, y, z) = sr;
				if (!isnan(fr))
					groundFatalityRiskTensor(x, y, z) = fr;
				// const auto psr = groundStrikeRiskTensor(x, y, z);
				// const auto pfr = groundFatalityRiskTensor(x, y, z);
				// if (psr != sr || pfr != fr)
				// {
				//     std::cout << "NE\n";
				// }
			}
		}

		std::cout << "Mat Sum: " << strikeRiskMat.array().isNaN().select(0, strikeRiskMat).sum();
		std::cout << " Mat Avg: " << strikeRiskMat.array().isNaN().select(0, strikeRiskMat).mean();
		std::cout << " Strike Tensor Sum: " << groundStrikeRiskTensor.sum();
		std::cout << " Strike Tensor Avg: " << groundStrikeRiskTensor.mean();
		std::cout << " Fatality Tensor Sum: " << groundFatalityRiskTensor.sum();
		std::cout << " Fatality Tensor Avg: " << groundFatalityRiskTensor.mean();
		std::cout << "--------------------------\n";
	}

	add("Ground Risk", 0);
	get("Ground Risk") = groundFatalityRiskTensor;
}
