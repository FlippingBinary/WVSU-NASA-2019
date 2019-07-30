#!/bin/bash

name=/rocksatx/launch/00001
i=0
while [[ -d $name ]] ; do
  let i++
  printf -v name "/rocksatx/launch/%05d" $i
done

echo $name

mkdir -p $name
cd $name

rm /rocksatx/rocksatx
gcc -o /rocksatx/rocksatx /rocksatx/rocksatx.c 2>&1 >> gcc.log

/rocksatx/rocksatx.sh &>>rocksatx.log &
/rocksatx/sbsvideo.sh &>>sbsvideo.log &
