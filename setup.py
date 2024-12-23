import sys
import os
import shutil
import sysconfig
import platform

from skbuild import setup
from setuptools import find_packages


# Remove previous build folder
def remove_readonly(func, path, excinfo):
    os.chmod(path, stat.S_IWRITE)
    os.remove(path)


if os.path.exists('_skbuild'):
    shutil.rmtree('_skbuild', onerror=remove_readonly)

# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
package_data_dir = sysconfig.get_paths()['purelib'] + '/pyuasrisk/data'
setup(
    name="pyuasrisk",
    version="0.0.1",
    author="Aliaksei Pilko",
    author_email="a.pilko@soton.ac.uk",
    description="Risk calculation for uncrewed aerial systems",
    long_description="",
    # license="Proprietary",
    packages=['pyuasrisk', 'pyuasrisk.data'],
    cmake_install_dir="python/pyuasrisk",
    cmake_args=[f"-DUGR_DATA_DIR={package_data_dir}", "-DUR_BUILD_CLI=OFF", "-DUR_BUILD_PYTHON_BINDINGS=ON"],
    include_package_data=True,
    package_data={"pyuasrisk": ["data/*.*"]},
    package_dir={"": "python"},
    python_requires=">=3.7",
)
