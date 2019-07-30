#!/bin/bash

while true ; do
  name=video-00001.h264
  j=1
  while [[ -f $name ]] ; do
    let j++
    printf -v name "video-%05d.h264" $j
  done
  
  echo $name
  
  /usr/bin/raspivid -3d sbs -vf -n -w 1792 -h 540 -fps 25 -g 25 -hf -t 0 -b 10000000 -o $name -ih
  sleep 1
done

# convert video to mp4 after event.
#MP4Box -fps 25 -add video.h264 video.mp4
