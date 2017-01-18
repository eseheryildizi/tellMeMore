#!/bin/bash

data=( "$@" )

echo "Enter a test file name:"
read name

echo ${data[@]} >> $name.txt

sshpass -p "luttraccc11" scp $name.txt altan@172.24.6.56:/usr/local/MATLAB/R2015b/bin
sshpass -p "luttraccc11" ssh -t altan@172.24.6.56 "cd /usr/local/MATLAB/R2015b/bin ; bash ./matlab -r 'matlabknn2 $name.txt ; exit;' "  
