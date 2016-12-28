#!/bin/bash
x=0
data=( "$@" )

colors=(red green blue yellow brown white)
sizes=(small medium big)
shapes=(cube sphere cylindre)

contains () {
  local e
  for e in "${@:2}"; do [[ "$e" == "$1" ]] && return 1; done
  return 0
}


echo "What color is it?"
read color


while contains "$color" "${colors[@]}" 
do
	echo "No such color!"
	echo "What color is it?"
	read color
done

echo "What's its shape?"
read shape

while contains $shape "${shapes[@]}"
do
	echo "No such shape!"
	echo "What's its shape?"
	read shape
done

echo "What's its size?"
read size

while contains $size "${sizes[@]}"
do
	echo "No such size!"
	echo "What's its size?"
	read size
done

for i in "${!colors[@]}"; do
   if [[ "${colors[$i]}" = "${color}" ]]; then
	break
   fi
done

for j in "${!shapes[@]}"; do
   if [[ "${shapes[$j]}" = "${shape}" ]]; then
	break
   fi
done

for k in "${!sizes[@]}"; do
   if [[ "${sizes[$k]}" = "${size}" ]]; then
	break
   fi
done

while [ -f "$size$color$shape$x.txt" ]
do
((x++))
done
echo ${data[@]} $i $j $k >> $size$color$shape$x.txt

sshpass -p "luttraccc11" scp $size$color$shape$x.txt altan@172.20.34.94:/usr/local/MATLAB/R2015b/bin/files

kill $PPID