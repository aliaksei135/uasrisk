from ctypes import cdll
import os
from pathlib import Path

current_dir = Path(os.path.dirname(os.path.realpath(__file__)))

if os.path.exists(current_dir / 'lib'):
    lib_dir = current_dir / 'lib'
elif os.path.exists(current_dir / 'lib64'):
    lib_dir = current_dir / 'lib64'

for lib in lib_dir.glob('*'):
    cdll.LoadLibrary(lib)

from ._pyuasrisk import __version__, __doc__, VoxelGrid, GroundRiskVoxelGrid, AirRiskVoxelGrid, RiskVoxelGrid, AircraftModel, AircraftStateModel, PopulationMap, TemporalPopulationMap

__all__ = ["VoxelGrid", "GroundRiskVoxelGrid", "AirRiskVoxelGrid", "RiskVoxelGrid", "AircraftModel", "AircraftStateModel", "PopulationMap", "TemporalPopulationMap"]