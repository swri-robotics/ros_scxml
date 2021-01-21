# ros_scxml
[![Build Status](https://travis-ci.com/swri-robotics/ros_scxml.svg?branch=master)](https://travis-ci.com/swri-robotics/ros_scxml)
[![Github Issues](https://img.shields.io/github/issues/swri-robotics/ros_scxml.svg)](http://github.com/swri-robotics/ros_scxml/issues)

[![license - bsd 2 clause](https://img.shields.io/:license-BSD%202--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)

Lightweight finite state machine library that uses the [SCXML](https://commons.apache.org/proper/commons-scxml/guide/scxml-documents.html) standard

---
## Prerequisites
### QT 5
The `QScxml` module is only available from `Qt` version 5.7 and higher. If your distributed version of `Qt` does not have the `QScxml` module there are several options for getting it:

#### Recommended Download (PPA)
- [This PPA](https://launchpad.net/~beineri) provides binary distributions of various versions of Qt for various operating systems
    - On the PPA site, select the desired `Qt` version for your system
- Add the PPA
    ```bash
    sudo add-apt-repository ppa:beineri/opt-qt-5.13.2-bionic
    sudo apt-get update
    ```
    > Note: Edit command above to match the version of qt that you'd like to install
- Install the full Qt library 
    ```
    sudo apt install qt513-meta-full
    ```
    > Note: Edit command above to match the version of qt that you'd like to install

#### Alternative Download (Qt Installer)
- The library can be downloaded from [here](http://download.qt.io/official_releases/qt/).  Run the installation script with root access and follow the on screen instructions.

#### Environment variables
In order to make this library accessible to cmake the `CMAKE_PREFIX_PATH` and `LD_LIBRARY_PATH` environment variables must be set.  Locate your Qt installation directory (usually in the `/opt` directory) and set the environment variables as follows:

```bash
export CMAKE_PREFIX_PATH=<path>/<to>/<qt>/lib/cmake:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=<path>/<to>/<qt>/lib:<path>/<to>/<qt>/plugins:$LD_LIBRARY_PATH
```
