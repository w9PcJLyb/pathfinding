# Documentation for w9-pathfinding

This folder contains the source files for the **w9-pathfinding** documentation.

The documentation is built with **Sphinx** and hosted on **Read the Docs**: https://w9-pathfinding.readthedocs.io

## Building the documentation locally

1. Install documentation dependencies

```bash
pip install -r docs/requirements.txt
```

2. Install graphviz

```bash
sudo apt install graphviz
```

3. Compile cython extensions

```bash
pip install cython
python setup.py build_ext --inplace
```

4. Build the docs

From the docs directory, run:

```bash
make html
```

Open `_build/html/index.html` in your browser.
