import re
import sys
from setuptools import setup, find_packages, Extension

from Cython.Build import cythonize

if sys.platform == "win32":
    extra_compile_args = ["/std:c++17"]
else:
    extra_compile_args = ["-std=c++17"]


def sources(*files):
    return [f"w9_pathfinding/src/{f}.cpp" for f in files]


include_dirs = ["w9_pathfinding/src/include"]


ext_modules = [
    Extension(
        name="w9_pathfinding.bindings.envs",
        sources=[
            "w9_pathfinding/bindings/envs.pyx",
            *sources(
                "env", "graph", "grid", "grid_3d", "hex_grid", "components"
            ),
        ],
        include_dirs=include_dirs,
        language="c++",
        extra_compile_args=extra_compile_args,
    ),
    Extension(
        name="w9_pathfinding.bindings.pf",
        sources=[
            "w9_pathfinding/bindings/pf.pyx",
            *sources(
                "env",
                "dfs",
                "bfs",
                "bi_bfs",
                "dijkstra",
                "bi_dijkstra",
                "a_star",
                "bi_a_star",
                "gbs",
                "ida_star",
                "resumable_search",
            ),
        ],
        include_dirs=include_dirs,
        language="c++",
        extra_compile_args=extra_compile_args,
    ),
    Extension(
        name="w9_pathfinding.bindings.mapf",
        sources=[
            "w9_pathfinding/bindings/mapf.pyx",
            *sources(
                "env",
                "reservation_table",
                "resumable_search",
                "space_time_a_star",
                "hc_a_star",
                "whc_a_star",
                "cbs",
                "icts",
                "multi_agent_a_star",
            ),
        ],
        include_dirs=include_dirs,
        language="c++",
        extra_compile_args=extra_compile_args,
    ),
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
    url="https://github.com/w9PcJLyb/w9-pathfinding",
    project_urls={
        "Bug Tracker": "https://github.com/w9PcJLyb/w9-pathfinding/issues",
        "Documentation": "https://w9-pathfinding.readthedocs.io/stable",
        "Release Notes": "https://w9-pathfinding.readthedocs.io/stable/changelog.html",
    },
    license="Apache-2.0",
    keywords="pathfinding mapf",
    long_description=long_description,
    long_description_content_type="text/markdown",
    # Build instructions
    ext_modules=cythonize(ext_modules, language_level="3", force=False),
    packages=find_packages(),
    install_requires=["setuptools", "cython"],
    extras_require={
        "dev": [
            "pytest",
            "pytest-subtests",
            "twine",
            "numpy",
            "matplotlib",
            "pydantic>=2.11.0",
        ],
    },
    python_requires=">=3.10",
)
