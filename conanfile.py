from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout


class ORLConan(ConanFile):
    name = "orl"
    version = "0.1"
    package_type = "application"

    settings = "os", "compiler", "build_type", "arch"

    requires = (
        "catch2/[>=3.0 <4.0]",
        "cglm/[>=0.9 <1.0]",
        "eigen/[>=3.4 <4.0]",
    )

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        toolchain = CMakeToolchain(self)
        toolchain.generate()
