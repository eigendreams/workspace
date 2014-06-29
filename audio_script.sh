#!/bin/sh -e
sshpass -p 'finder' ssh "arecord -D plughw:1,0 -f dat" | aplay -f dat
