import sys
import sysconfig

from skbuild import setup
from setuptools import find_packages

# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
package_data_dir= sysconfig.get_paths()['purelib'] + '/pyuasrisk/data'
print(package_data_dir)
setup(
    name="pyuasrisk",
    version="0.0.1",
    author="Aliaksei Pilko",
    author_email="a.pilko@soton.ac.uk",
    description="Risk calculation for uncrewed aerial systems",
    long_description="",
    license="Proprietary",
    packages=find_packages(where='python'),
    cmake_install_dir="python/pyuasrisk",
    cmake_args=[f"-DUGR_DATA_DIR={package_data_dir}"],
    include_package_data=True,
    package_data={"pyuasrisk": ["data/*.*"]},
    package_dir={"": "python"},
    python_requires=">=3.6",
)
