#!/bin/sh

sleep $3

shift # The sleep time is droped

roslaunch $@
