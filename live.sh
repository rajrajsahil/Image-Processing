#!/bin/bash
if [ -p mypipe ]
then
	rm mypipe
fi
mkfifo mypipe
nc 10.42.0.152 9696 > mypipe
