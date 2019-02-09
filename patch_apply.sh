#!/bin/bash
cd ../OkapiLib
make template || exit 1
prosv5 c p okapilib@3.3.9 || exit 1
prosv5 c f okapilib@3.3.9.zip || exit 1
cd ../Elliot2
prosv5 c i okapilib@3.3.9 || exit 1

