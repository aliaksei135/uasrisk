import numpy as np

import pyuasrisk
from pyuasrisk import VoxelGrid, GridMap, GeospatialGridMap, AircraftStateModel, AircraftModel, \
    PopulationMap, TemporalPopulationMap, ObstacleMap, WeatherMap, FullGroundRiskVoxelGrid, GroundRiskVoxelGrid, \
    IncrementalGroundRiskVoxelGrid, AirRiskVoxelGrid, RiskVoxelGrid

# SWNE
xy_bounds = np.array([50.9, -1.5, 50.95, -1.4])
xyz_bounds = np.array([50.9, -1.5, 0, 50.95, -1.4, 100])
bounds_centre = np.array([-1.45, 50.925, 40])

xy_res = 200  # Just testing, so coarse is fine
z_res = 40


def test_bindings():
    assert pyuasrisk.__version__
    assert pyuasrisk.__doc__


def test_voxel_grid():
    voxel_grid = VoxelGrid(xyz_bounds, xy_res, z_res)
    assert voxel_grid is not None
    assert np.array_equal(voxel_grid.size, np.array([55, 44, 2.0]))
    # Test default layers are there
    assert voxel_grid.layers


def test_voxel_grid_layer_crud():
    voxel_grid = VoxelGrid(xyz_bounds, xy_res, z_res)
    assert voxel_grid is not None
    voxel_grid.add("test_layer", np.ones(voxel_grid.size, dtype=np.float32, order="F"))
    assert voxel_grid.layers
    assert np.array_equal(voxel_grid.get("test_layer"), np.ones(voxel_grid.size))
    assert voxel_grid.layers[0] == "test_layer"
    assert voxel_grid.at("test_layer", np.array([0, 0, 0])) == 1
    assert voxel_grid.at("test_layer", voxel_grid.size - 1) == 1
    assert voxel_grid.atPosition("test_layer", np.array([-1.45, 50.925, 1])) == 1


def test_voxel_grid_bounds():
    voxel_grid = VoxelGrid(xyz_bounds, xy_res, z_res)
    assert voxel_grid is not None
    assert not voxel_grid.isInBounds(voxel_grid.size + 200)
    assert voxel_grid.isInBounds(voxel_grid.size - 1)


def test_voxel_grid_projection():
    voxel_grid = VoxelGrid(xyz_bounds, xy_res, z_res)
    assert voxel_grid is not None
    local_idx = voxel_grid.world2Local(bounds_centre)
    assert voxel_grid.isInBounds(local_idx)
    world_idx = voxel_grid.local2World(local_idx)
    assert np.allclose(world_idx, bounds_centre, atol=0.005)


def test_grid_map():
    grid_map = GridMap()
    assert grid_map is not None


def test_geospatial_grid_map_layer_crud():
    geospatial_grid_map = GeospatialGridMap(xy_bounds, xy_res)
    assert geospatial_grid_map is not None
    geospatial_grid_map.add("test_layer", np.ones(geospatial_grid_map.size, dtype=np.float32, order="F"))
    assert geospatial_grid_map.layers
    assert np.array_equal(geospatial_grid_map.get("test_layer"), np.ones(geospatial_grid_map.size))
    assert geospatial_grid_map.layers[0] == "test_layer"
    assert geospatial_grid_map.at("test_layer", np.array([0, 0])) == 1
    assert geospatial_grid_map.at("test_layer", geospatial_grid_map.size - 1) == 1
    assert geospatial_grid_map.atPosition("test_layer", bounds_centre[:2]) == 1


def test_geospatial_grid_map_bounds():
    geospatial_grid_map = GeospatialGridMap(xy_bounds, xy_res)
    assert geospatial_grid_map is not None
    assert not geospatial_grid_map.isInBounds(geospatial_grid_map.size + 200)
    assert geospatial_grid_map.isInBounds(geospatial_grid_map.size - 1)


def test_geospatial_grid_map_projection():
    geospatial_grid_map = GeospatialGridMap(xy_bounds, xy_res)
    assert geospatial_grid_map is not None
    local_idx = geospatial_grid_map.world2Local(bounds_centre[:2])
    assert geospatial_grid_map.isInBounds(local_idx)
    world_idx = geospatial_grid_map.local2World(local_idx)
    assert np.allclose(world_idx, bounds_centre[:2], atol=0.005)


def test_aircraft_state_model():
    asm = AircraftStateModel()
    asm.position = np.array([0, 0, 40])
    asm.velocity = np.array([20, 0, 0])
    assert asm
    assert np.array_equal(asm.position, np.array([0, 0, 40]))
    assert np.array_equal(asm.velocity, np.array([20, 0, 0]))


def test_aircraft_model():
    asm = AircraftStateModel()
    asm.position = np.array([0, 0, 40])
    asm.velocity = np.array([20, 0, 0])
    am = AircraftModel(25, 6, 5, 5e-3)
    am.state = asm
    assert am is not None
    assert am.state is not None
    assert am.mass == 25
    assert am.width == 6
    assert am.length == 5
    assert am.failureProb == 5e-3

    am.addGlideDescentModel(22, 15)
    am.addBallisticDescentModel(5, 0.8)
    am.addParachuteDescentModel(1.1, 6.0, 1.3)

    assert am.descentNames


def test_population_map():
    population_map = PopulationMap(xy_bounds, xy_res)
    assert population_map is not None
    assert not population_map.layers
    population_map.eval()
    assert population_map.layers
    assert population_map.get("Population Density") is not None
    assert np.array_equal(population_map.get("Population Density"), np.zeros(population_map.size))


def test_temporal_population_map():
    temporal_population_map = TemporalPopulationMap(xy_bounds, xy_res)
    assert temporal_population_map is not None
    temporal_population_map.setHourOfDay(12)
    temporal_population_map.eval()
    assert temporal_population_map.layers
    assert temporal_population_map.get("Population Density") is not None
    assert temporal_population_map.get("Population Density").mean() > 0


def test_obstacle_map():
    obstacle_map = ObstacleMap(xy_bounds, xy_res)
    assert obstacle_map is not None
    assert not obstacle_map.layers
    obstacle_map.addBuildingHeights()
    obstacle_map.eval()
    assert obstacle_map.layers
    assert obstacle_map.get("Building Height") is not None
    assert obstacle_map.get("Building Height").mean() > 0


def test_weather_map():
    weather_map = WeatherMap(xy_bounds, xy_res)
    assert weather_map is not None
    weather_map.addConstantWind(5, 135)
    weather_map.eval()
    assert weather_map.layers
    assert weather_map.get("Wind VelX") is not None
    assert weather_map.get("Wind VelY") is not None
    assert weather_map.get("Wind VelX").mean() - 3.5355 < 1e-3
    assert weather_map.get("Wind VelY").mean() - 3.5355 < 1e-3


def test_full_ground_risk_voxel_grid():
    population_map = TemporalPopulationMap(xy_bounds, xy_res)
    population_map.setHourOfDay(12)
    population_map.eval()
    assert population_map.get("Population Density").mean() > 0

    asm = AircraftStateModel()
    asm.position = np.array([0, 0, 40])
    asm.velocity = np.array([20, 0, 0])
    am = AircraftModel(25, 6, 5, 5e-3)
    am.state = asm
    am.addGlideDescentModel(22, 15)
    am.addBallisticDescentModel(5, 0.8)

    obstacle_map = ObstacleMap(xy_bounds, xy_res)
    obstacle_map.addBuildingHeights()
    obstacle_map.eval()

    weather_map = WeatherMap(xy_bounds, xy_res)
    weather_map.addConstantWind(5, 135)
    weather_map.eval()

    full_ground_risk_voxel_grid = FullGroundRiskVoxelGrid(xyz_bounds, xy_res, z_res, population_map, am,
                                                          obstacle_map, weather_map)
    assert full_ground_risk_voxel_grid is not None
    full_ground_risk_voxel_grid.eval()
    assert full_ground_risk_voxel_grid.at("Ground Risk", np.array([10, 10, 1])) > 0
    assert full_ground_risk_voxel_grid.atPosition("Ground Risk", bounds_centre) > 0
    # assert full_ground_risk_voxel_grid.at("Ground Fatality Risk", np.array([10, 10, 1])) > 0
    # assert full_ground_risk_voxel_grid.get("Ground Strike Risk").mean() > 0
    # assert full_ground_risk_voxel_grid.get("Ground Fatality Risk").mean() > 0


def test_ground_risk_voxel_grid():
    asm = AircraftStateModel()
    asm.position = np.array([0, 0, 40])
    asm.velocity = np.array([20, 0, 0])
    am = AircraftModel(25, 6, 5, 5e-3)
    am.state = asm
    am.addGlideDescentModel(22, 15)
    am.addBallisticDescentModel(5, 0.8)
    ground_risk_voxel_grid = GroundRiskVoxelGrid(xyz_bounds, xy_res, z_res, am)
    assert ground_risk_voxel_grid is not None
    ground_risk_voxel_grid.eval()
    assert ground_risk_voxel_grid.at("Ground Risk", np.array([10, 10, 1])) > 0
    assert ground_risk_voxel_grid.atPosition("Ground Risk", bounds_centre) > 0
    # assert ground_risk_voxel_grid.at("Ground Fatality Risk", np.array([10, 10, 1])) > 0
    # assert ground_risk_voxel_grid.get("Ground Strike Risk").mean() > 0
    # assert ground_risk_voxel_grid.get("Ground Fatality Risk").mean() > 0


def test_incremental_ground_risk_voxel_grid():
    asm = AircraftStateModel()
    asm.position = np.array([0, 0, 40])
    asm.velocity = np.array([20, 0, 0])
    am = AircraftModel(25, 6, 5, 5e-3)
    am.state = asm
    am.addGlideDescentModel(22, 15)
    am.addBallisticDescentModel(5, 0.8)

    incremental_ground_risk_voxel_grid = IncrementalGroundRiskVoxelGrid(xyz_bounds, xy_res, z_res, am)
    assert incremental_ground_risk_voxel_grid is not None
    index_centre = np.array([10, 10])

    positionStrikeRisk = incremental_ground_risk_voxel_grid.getPositionPointStrikeProbability(bounds_centre, 270)
    positionFatalityRisk = incremental_ground_risk_voxel_grid.getPositionPointFatalityProbability(bounds_centre, 270)
    assert positionStrikeRisk > 0
    assert positionFatalityRisk > 0
    assert positionStrikeRisk > positionFatalityRisk

    indexStrikeRisk = incremental_ground_risk_voxel_grid.getIndexPointStrikeProbability(index_centre, 50, 270)
    indexFatalityRisk = incremental_ground_risk_voxel_grid.getIndexPointFatalityProbability(index_centre, 50, 270)
    assert indexStrikeRisk > 0
    assert indexFatalityRisk > 0
    assert indexStrikeRisk > indexFatalityRisk

# def test_air_risk_voxel_grid():
#     air_risk_voxel_grid = AirRiskVoxelGrid()
#     assert air_risk_voxel_grid is not None
