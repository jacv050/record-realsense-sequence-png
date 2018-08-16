#!/bin/bash

mkdir $1
cp rs-save-to-disk $1
cd $1 && ./rs-save-to-disk
rm rs-save-to-disk
