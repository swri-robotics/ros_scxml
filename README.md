# ros_scxml
[![Build Status](https://travis-ci.com/swri-robotics/ros_scxml.svg?branch=master)](https://travis-ci.com/swri-robotics/ros_scxml)
[![Github Issues](https://img.shields.io/github/issues/swri-robotics/ros_scxml.svg)](http://github.com/swri-robotics/ros_scxml/issues)

[![license - bsd 2 clause](https://img.shields.io/:license-BSD%202--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)

Lightweight finite state machine library that uses the [SCXML](https://commons.apache.org/proper/commons-scxml/guide/scxml-documents.html) standard

---
## Prerequisites
### QT 5
The `QScxml` module is available from `Qt` version 5.7 and higher and is currently only distributed on Ubuntu 20.04. If your distribution or version of `Qt` does not have the `QScxml` module there are several options for getting it:

#### Qt Modules PPA (Recommended)
[This PPA](https://launchpad.net/~skycoder42/+archive/ubuntu/qt-modules) provides binary distributions of the `QScxml` module for Ubuntu 17.10 and 18.04.
These binaries install to the standard install directory and should be the same sub-version as the other `Qt` modules for the distribution.
This the most straightforward solution for Ubuntu 17.10/18.04.
To install, run:

```bash
sudo add-apt-repository ppa:beineri/ppa:skycoder42/qt-modules
sudo apt update
sudo apt install libqt5scxml-dev
```

#### `/opt` Directory Qt Install (PPA)
[This PPA](https://launchpad.net/~beineri) provides binary distributions of various versions of Qt for various operating systems.
These binaries install to the `/opt` directory, which is not a standard search path for `cmake`.
As such, several build and run environment variables need to be modified in order to use this distribution of `Qt`.
Installing a version of `Qt` from this PPA alongside the system installation can also cause version tagging issues when compiling code that depends on `Qt`.
To install, run:

```bash
sudo add-apt-repository ppa:beineri/opt-qt-5.12.10-bionic
sudo apt-get update
sudo apt install qt512scxml
```
> Note: Edit command above for your desired version of `Qt`

In order to make this installation find-able to `cmake`, several environment variables must be set.
Locate your Qt installation directory in the `/opt` directory and set the environment variables as follows:

```bash
export CMAKE_PREFIX_PATH=/opt/qt<version>/lib/cmake:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/qt<version>/lib:/opt/qt<version>/plugins:$LD_LIBRARY_PATH
```

#### Alternative Download (Qt Installer)
The library can be downloaded from [here](http://download.qt.io/official_releases/qt/).  Run the installation script with root access and follow the on screen instructions.
