#! /bin/bash
add-apt-repository -y ppa:skycoder42/qt-modules
apt-get update -qq
apt-get install -y --no-install-recommends libqt5scxml-dev
