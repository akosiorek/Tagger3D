#!/bin/bash

cd src

for file in $(find $PWD); do
	if [[ -d $file ]]; then
		echo $file
		echo ""
		echo ""
	else 
		cat $file
		echo ""
		echo ""
	fi
done
