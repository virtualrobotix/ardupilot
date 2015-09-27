#!/bin/bash

export PATH=$PATH:/bin:/usr/bin

export TMPDIR=$PWD/build.tmp.$$
echo $TMPDIR
rm -rf $TMPDIR
echo "Building in $TMPDIR"

date
#git remote update
#git checkout -B for_merge remotes/origin/for_merge
#githash=$(git rev-parse HEAD)

hdate=$(date +"%Y-%m/%Y-%m-%d-%H-%m")
mkdir -p binaries/$hdate
binaries=$PWD/../buildlogs/binaries
BASEDIR=$PWD

error_count=0

echo "TMPDIR = $TMPDIR"
echo "binaries = $binaries"
echo "BASEDIR = $BASEDIR"

if [ "$#" = 0 ]
then
    #Any parameters are passed: I have to build all
    echo "Build all version for all vehicle"
    for build in beta stable latest; do
        echo "Build $build for Plane"  #build_arduplane $build
        echo "Build $build for Copter" #build_arducopter $build
        echo "Build $build for Rover"  #build_rover $build
    done
else
    if [ "$1" = "Copter" ] || [ "$1" = "Plane" ] || [ "$1" = "Rover" ]
    then
        #First parameter is the type of vehicle
        if [ "$2" = "" ]
        then
            #Second parameter is empty: I have to build all version for the vehicle
            echo "Build all version for $1"
            for build in beta stable latest; do
                if [ "$1" = "Plane" ]
                then
                    echo "Build $build for Plane"  #build_arduplane $build
                elif [ "$1" = "Copter" ]
                then
                    echo "Build $build for Copter" #build_arducopter $build
                elif [ "$1" = "Rover" ]
                then
                    echo "Build $build for Rover"  #build_rover $build
                fi
            done
        else
            echo "Build version $2 for vehicle $1"
            if [ "$1" = "Plane" ]
            then
                echo "Build $2 for $1" #build_arduplane $2
            elif [ "$1" = "Copter" ]
            then
                echo "Build $2 for $1" #build_arducopter $2
            elif [ "$1" = "Rover" ]
            then
                echo "Build $2 for $1" #build_rover $2
            fi
        fi
    else
        if [ "$1" = "beta" ] || [ "$1" = "stable" ] || [ "$1" = "latest" ]
        then
            #First parameter is the version: I have to build this version for all vehicle
            echo "Build version $1 for all vehicle"
            echo "Build $1 for Plane"  #build_arduplane $build
            echo "Build $1 for Copter" #build_arducopter $build
            echo "Build $1 for Rover"  #build_rover $build
        else
            echo "Error"
        fi
    fi
fi

rm -rf $TMPDIR

exit $error_count
