Installation
============

Requirements
---------------------------

- **Python 3.10 or higher**

- **C++ Toolchain**

  Currently, w9-pathfinding is distributed as **source code**.  
  Because it includes C++ and Cython components, your system must be able
  to compile native extensions.

  You will need a working C++ toolchain:

  - **On Linux**: Install the required build tools:

    .. code-block:: bash

      sudo apt install --reinstall build-essential gcc g++

  - **On Windows**: Install Microsoft Visual C++ Build Tools (MSVC), either via Visual Studio or the standalone installer.

  - **On macOS**: Install Xcode command line tools:

    .. code-block:: bash

      xcode-select --install

    *(Note: I haven't tested this library on macOS)*


PyPI
---------------------------

The package is available on pypi, so you can install it with `pip`:

.. code-block:: bash

   pip install w9-pathfinding


For development purposes  
---------------------------

Alternatively, you can install it manually:

1. Clone the library:

   .. code-block:: bash

     git clone https://github.com/w9PcJLyb/pathfinding.git

2. Setup virtual environment (optional but recommended)

3. Install Cython:

   .. code-block:: bash

     pip install cython

4. Build the Cython extensions:

   .. code-block:: bash

     python setup.py build_ext --inplace

5. Finally, install the package:

   .. code-block:: bash

     pip install -e .
