#!/bin/bash

ARCH="X86"
VERSION="4.5.0"
FILENAME="VRmUsbCamDevKitForLinux${ARCH}_${VERSION}"

URL="http://www.vrmagic.com/fileadmin/downloads/imaging/Software/linux/${FILENAME}.zip"
EXTRACT_TO="vrmusb.zip"
TEMP_DIR=/tmp/foo #$(mktemp -d)

pushd $TEMP_DIR
wget -O $EXTRACT_TO $URL 
unzip $EXTRACT_TO
cd $FILENAME
ls
popd