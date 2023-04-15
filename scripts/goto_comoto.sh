#!/bin/bash
if [ -z "$1" ]
  then
    echo "No argument supplied"
    exit 1
fi
cd $1
julia mpc_server.jl $i &
