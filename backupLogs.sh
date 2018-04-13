#!/bin/bash

logFilesCmd="adb shell 'ls -l sdcard/robotControllerLog.txt* 2>&1 | grep -iv \"no such file\"' | wc -l"
if [ $(eval $logFilesCmd) != "0" ]; then
	echo "Copying files..."
	adb shell 'ls sdcard/robotControllerLog.txt*' | tr -d '\r' | xargs -n1 adb pull
	echo "Deleting files..."
	adb shell 'rm sdcard/robotControllerLog.txt*'
fi
