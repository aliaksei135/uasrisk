from conan import ConanFile


class UasgroundriskConan(ConanFile):
    name = "uasrisk"
    version = "0.1"
    author = "Aliaksei Pilko <A.Pilko@soton.ac.uk>"
    license = "Proprietary"
    settings = "os", "compiler", "arch", "build_type"
    generators = "CMakeToolchain", "CMakeDeps"
    exports_sources = "CMakeLists.txt", "src/*"

    def requirements(self):
        self.requires("eigen/[>3.3.9]")
        self.requires("boost/1.81.0")
        self.requires("proj/9.5.0")
        self.requires("fast-cpp-csv-parser/cci.20211104")
        self.requires("rapidjson/cci.20220822")
        self.requires("pugixml/1.15")
        self.requires("spdlog/[>=1.10.0]")
        # uasgroundrisk deps
        self.requires("cpr/1.10.5")
        self.requires("expat/2.7.1")
        self.requires("geos/3.13.0")
        self.requires("shapelib/1.5.0")
