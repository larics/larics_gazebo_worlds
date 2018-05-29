#!/bin/bash

xsltproc X3dToVrml97.xslt $1.x3d > $1.vrml
./binvox $1.vrml
binvox2bt $1.binvox