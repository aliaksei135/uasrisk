from ctypes import cdll
import os
from pathlib import Path

current_dir = Path(os.path.dirname(os.path.realpath(__file__)))

if os.path.exists(current_dir / 'lib'):
    lib_dir = current_dir / 'lib'
elif os.path.exists(current_dir / 'lib64'):
    lib_dir = current_dir / 'lib64'

cdll.LoadLibrary(lib_dir / 'libuasgroundrisk.so')
cdll.LoadLibrary(lib_dir / 'libuasrisk.so')

from ._pyuasrisk import __version__, __doc__, VoxelGrid, GridMap, GeospatialGridMap, AircraftStateModel, AircraftModel, \
    PopulationMap, TemporalPopulationMap, ObstacleMap, WeatherMap, FullGroundRiskVoxelGrid, GroundRiskVoxelGrid, \
    IncrementalGroundRiskVoxelGrid, AirRiskVoxelGrid, RiskVoxelGrid

__all__ = ["VoxelGrid", "GridMap", "GeospatialGridMap", "AircraftStateModel", "AircraftModel", "PopulationMap",
           "TemporalPopulationMap", "ObstacleMap", "WeatherMap", "FullGroundRiskVoxelGrid", "GroundRiskVoxelGrid",
           "IncrementalGroundRiskVoxelGrid", "AirRiskVoxelGrid", "RiskVoxelGrid"]
