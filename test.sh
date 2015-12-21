#/bin/bash

loop=1

while(($loop<=200))
do 
	if ./bootloader == 0
	then
		echo "success $loop" >> ./log.txt 
	else
		echo "fail $loop" >> ./log.txt
	fi
	let "loop++"
done
