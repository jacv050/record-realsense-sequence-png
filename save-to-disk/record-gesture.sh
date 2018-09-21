#!/bin/bash

#dir_dataset/labels/samples/channels/frames.png

mkdir $1
mkdir $1/Depth $1/Color $1/OpticalFlow
./rs-save-to-disk $1
#cp rs-save-to-disk $1
#cd $1 && ./rs-save-to-disk
#rm rs-save-to-disk
