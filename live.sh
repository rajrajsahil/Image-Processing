#!/bin/bash
if [ -p myfifo ]
then
	rm myfifo
fi
mkfifo myfifo
nc 10.42.0.152 9696 > myfifo
