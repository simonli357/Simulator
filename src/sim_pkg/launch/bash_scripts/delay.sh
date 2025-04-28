#!/bin/sh

sleep $1

shift # The sleep time is droped

roslaunch $@
