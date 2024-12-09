import re
import sys
from setuptools import setup, find_packages, Extension

from Cython.Build import cythonize

if sys.platform == "win32":
    extra_compile_args = ["/std:c++17"]
else:
    extra_compile_args = ["-std=c++17"]

ext_modules = [
    Extension(
        name="w9_pathfinding.cpf",
        sources=["w9_pathfinding/wrapper.pyx"],
        include_dirs=["w9_pathfinding/src/"],
        language="c++",
        extra_compile_args=extra_compile_args,
    )
]


def read_version():
    with open("w9_pathfinding/__init__.py") as f:
        match = re.search(r'__version__ = ["\'](.*)["\']', f.read())
        if match:
            return match.group(1)
        raise RuntimeError("Unable to find version string.")


with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    # Information
    name="w9-pathfinding",
    version=read_version(),
    author="w9PcJLyb",
    description="Implementation of some pathfinding algorithms",
    url="https://github.com/w9PcJLyb/pathfinding",
    project_urls={
        "Bug Tracker": "https://github.com/w9PcJLyb/pathfinding/issues",
    },
    license="Apache-2.0",
    keywords="pathfinding mapf",
    long_description=long_description,
    long_description_content_type="text/markdown",
    # Build instructions
    ext_modules=cythonize(ext_modules),
    packages=find_packages(),
    install_requires=["setuptools", "cython"],
    extras_require={
        "dev": ["pytest", "pytest-subtests", "twine", "numpy", "matplotlib"],
    },
    python_requires=">=3.10",
)
