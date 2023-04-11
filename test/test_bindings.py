import numpy as np
import pyuasrisk
from pyuasrisk import VoxelGrid, GridMap, GeospatialGridMap, AircraftStateModel, AircraftModel, \
    PopulationMap, TemporalPopulationMap, ObstacleMap, WeatherMap, FullGroundRiskVoxelGrid, GroundRiskVoxelGrid, \
    IncrementalGroundRiskVoxelGrid, AirRiskVoxelGrid, RiskVoxelGrid

# SWNE
xy_bounds = np.array([50.8, -1.6, 50.85, -1.5])
xyz_bounds = np.array([50.8, -1.6, 0, 50.85, -1.5, 100])
xy_res = 100
z_res = 40


def test_bindings():
    assert pyuasrisk.__version__ == "0.0.1"
    assert pyuasrisk.__doc__


def test_voxel_grid():
    voxel_grid = VoxelGrid(xyz_bounds, xy_res, z_res)
    assert voxel_grid is not None


def test_grid_map():
    grid_map = GridMap()
    assert grid_map is not None


def test_geospatial_grid_map():
    geospatial_grid_map = GeospatialGridMap(xy_bounds, xy_res)
    assert geospatial_grid_map is not None


def test_aircraft_state_model():
    aircraft_state_model = AircraftStateModel()
    assert aircraft_state_model is not None


def test_aircraft_model():
    aircraft_model = AircraftModel(25, 5, 5, 5e-3)
    assert aircraft_model is not None


def test_population_map():
    population_map = PopulationMap(xy_bounds, xy_res)
    assert population_map is not None


def test_temporal_population_map():
    temporal_population_map = TemporalPopulationMap(xy_bounds, xy_res)
    assert temporal_population_map is not None


def test_obstacle_map():
    obstacle_map = ObstacleMap(xy_bounds, xy_res)
    assert obstacle_map is not None


def test_weather_map():
    weather_map = WeatherMap(xy_bounds, xy_res)
    assert weather_map is not None


def test_full_ground_risk_voxel_grid():
    population_map = TemporalPopulationMap(xy_bounds, xy_res)
    aircraft_model = AircraftModel(25, 5, 5, 5e-3)
    aircraft_model.state = AircraftStateModel()
    obstacle_map = ObstacleMap(xy_bounds, xy_res)
    weather_map = WeatherMap(xy_bounds, xy_res)
    full_ground_risk_voxel_grid = FullGroundRiskVoxelGrid(xyz_bounds, xy_res, z_res, population_map, aircraft_model,
                                                          obstacle_map, weather_map)
    assert full_ground_risk_voxel_grid is not None


def test_ground_risk_voxel_grid():
    aircraft_model = AircraftModel(25, 5, 5, 5e-3)
    aircraft_model.state = AircraftStateModel()
    ground_risk_voxel_grid = GroundRiskVoxelGrid(xyz_bounds, xy_res, z_res, aircraft_model)
    assert ground_risk_voxel_grid is not None


def test_incremental_ground_risk_voxel_grid():
    aircraft_model = AircraftModel(25, 5, 5, 5e-3)
    aircraft_model.state = AircraftStateModel()
    incremental_ground_risk_voxel_grid = IncrementalGroundRiskVoxelGrid(xyz_bounds, xy_res, z_res, aircraft_model)
    assert incremental_ground_risk_voxel_grid is not None


# def test_air_risk_voxel_grid():
#     air_risk_voxel_grid = AirRiskVoxelGrid()
#     assert air_risk_voxel_grid is not None
