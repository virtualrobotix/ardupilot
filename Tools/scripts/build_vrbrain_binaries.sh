#!/bin/bash

export PATH=$PATH:/bin:/usr/bin

export TMPDIR=$PWD/build.tmp.$$
echo $TMPDIR
rm -rf $TMPDIR
echo "Building in $TMPDIR"

date
git remote update
git checkout -B for_merge remotes/origin/for_merge
githash=$(git rev-parse HEAD)

hdate=$(date +"%Y-%m/%Y-%m-%d-%H-%m")
mkdir -p binaries/$hdate
binaries=$PWD/../buildlogs/binaries
VRNUTTX=$PWD/../VRNuttX
BASEDIR=$PWD

error_count=0

# Print some information ##########################################
# echo "PATH     = $PATH"
# echo "PWD      = $PWD"
# echo "TMPDIR   = $TMPDIR"
# echo "GIT HASH = $githash"
# echo "DATE     = $hdate"
# echo "DIR      = binaries/$hdate"
# echo "binaries = $binaries"
# echo "BASEDIR = $BASEDIR"
###################################################################

# checkout the right version of the tree
checkout() {
    vehicle="$1"
    tag="$2"
    board="$3"
    git stash
    if [ "$tag" = "latest" ]; then
        vtag="for_merge"
        vtag2="for_merge"
    else
        vtag="$vehicle-$tag-$board"
        vtag2="$vehicle-$tag"
    fi

    echo "Checkout for $vehicle for $board with tag $tag"

    # git checkout -B "$vtag2" remotes/origin/"$vtag2" || return 1
    git checkout -f -B "$vtag" remotes/origin/"$vtag" || git checkout -f -B "$vtag2" remotes/origin/"$vtag2" || return 1
    git log -1

    pushd $VRNUTTX
    # git checkout -B "$vtag2" remotes/origin/"$vtag2" || git checkout -B for_merge remotes/origin/for_merge || {
    git checkout -f -B "$vtag" remotes/origin/"$vtag" || git checkout -f -B "$vtag2" remotes/origin/"$vtag2" || git checkout -f -B for_merge remotes/origin/for_merge || {
        popd
        return 1
    }
    git log -1
    popd


    return 0
}

# check if we should skip this build because we have already
# built this version
skip_build() {
    [ "$FORCE_BUILD" = "1" ] && return 1
    tag="$1"
    ddir="$2"
    bname=$(basename $ddir)
    ldir=$(dirname $(dirname $(dirname $ddir)))/$tag/$bname
    [ -d "$ldir" ] || {
        echo "$ldir doesn't exist - building"
        return 1
    }
    oldversion=$(cat "$ldir/git-version.txt" | head -1)
    newversion=$(git log -1 | head -1)
    [ "$oldversion" = "$newversion" ] && {
        echo "Skipping build - version match $newversion"
        return 0
    }
    echo "$ldir needs rebuild"
    return 1
}

addfwversion() {
    destdir="$1"
    git log -1 > "$destdir/git-version.txt"
    [ -f APM_Config.h ] && {
        shopt -s nullglob
        version=$(grep 'define.THISFIRMWARE' *.pde *.h 2> /dev/null | cut -d'"' -f2)
        echo >> "$destdir/git-version.txt"
        echo "APMVERSION: $version" >> "$destdir/git-version.txt"
    }    
}

# copy the built firmware to the right directory
copyit() {
    file="$1"
    dir="$2"
    tag="$3"
    bname=$(basename $dir)
    tdir=$(dirname $(dirname $(dirname $dir)))/$tag/$bname
    if [ "$tag" = "latest" ]; then
        mkdir -p "$dir"
        /bin/cp "$file" "$dir"
        addfwversion "$dir"
    fi
    echo "Copying $file to $tdir"
    mkdir -p "$tdir"
    addfwversion "$tdir"

    rsync "$file" "$tdir"
}

# build plane binaries
build_arduplane() {
    tag="$1"
    echo "Building ArduPlane $tag binaries from $(pwd)"
    pushd ArduPlane
    echo "Building ArduPlane VRBRAIN binaries"
    ddir=$binaries/Plane/$hdate/VRX
    checkout Plane $tag "" || {
        echo "Failed checkout of ArduPlane VRBRAIN $tag"
        error_count=$((error_count+1))
        checkout Plane "latest" ""
        popd
        return
    }
   skip_build $tag $ddir || {
       make vrbrain-clean && make vrbrain || {
           echo "Failed build of ArduPlane VRX $tag"
           error_count=$((error_count+1))
           checkout Plane "latest" ""
           popd
           return
       }
       copyit ArduPlane-vrbrain-v45.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v45.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v45.hex $ddir $tag &&
       copyit ArduPlane-vrbrain-v45P.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v45P.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v45P.hex $ddir $tag &&
       copyit ArduPlane-vrbrain-v51.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v51.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v51.hex $ddir $tag &&
       copyit ArduPlane-vrbrain-v51P.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v51P.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v51P.hex $ddir $tag &&
       copyit ArduPlane-vrbrain-v51Pro.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v51Pro.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v51Pro.hex $ddir $tag &&
       copyit ArduPlane-vrbrain-v51ProP.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v51ProP.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v51ProP.hex $ddir $tag &&
       copyit ArduPlane-vrbrain-v52.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v52.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v52.hex $ddir $tag &&
       copyit ArduPlane-vrbrain-v52P.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v52P.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v52P.hex $ddir $tag &&
       copyit ArduPlane-vrbrain-v52Pro.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v52Pro.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v52Pro.hex $ddir $tag &&
       copyit ArduPlane-vrbrain-v52ProP.vrx $ddir $tag && 
       copyit ArduPlane-vrbrain-v52ProP.bin $ddir $tag && 
       copyit ArduPlane-vrbrain-v52ProP.hex $ddir $tag &&
       copyit ArduPlane-vrubrain-v51.vrx $ddir $tag && 
       copyit ArduPlane-vrubrain-v51.bin $ddir $tag && 
       copyit ArduPlane-vrubrain-v51.hex $ddir $tag && 
       copyit ArduPlane-vrubrain-v51P.vrx $ddir $tag && 
       copyit ArduPlane-vrubrain-v51P.bin $ddir $tag && 
       copyit ArduPlane-vrubrain-v51P.hex $ddir $tag &&
       copyit ArduPlane-vrubrain-v52.vrx $ddir $tag && 
       copyit ArduPlane-vrubrain-v52.bin $ddir $tag && 
       copyit ArduPlane-vrubrain-v52.hex $ddir $tag
   }
    checkout Plane "latest" ""
    popd
}

# build copter binaries
build_arducopter() {
    tag="$1"
    echo "Building ArduCopter $tag binaries from $(pwd)"
    pushd ArduCopter
    frames="quad tri hexa y6 octa octa-quad heli"
    for f in $frames; do
        checkout Copter $tag $f || {
            echo "Failed checkout of ArduCopter VRX $tag $f"
            error_count=$((error_count+1))
            checkout Copter "latest" ""
            continue
        }
        echo "Building ArduCopter VRBRAIN-$f binaries"
        ddir="$binaries/Copter/$hdate/VRX-$f"
       skip_build $tag $ddir && continue
       make vrbrain-clean && make vrbrain-$f || {
           echo "Failed build of ArduCopter VRBRAIN $tag"
           error_count=$((error_count+1))
           continue
       }
        copyit ArduCopter-vrbrain-v45.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v45.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v45.hex $ddir $tag &&
        copyit ArduCopter-vrbrain-v45P.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v45P.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v45P.hex $ddir $tag &&
        copyit ArduCopter-vrbrain-v51.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v51.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v51.hex $ddir $tag &&
        copyit ArduCopter-vrbrain-v51P.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v51P.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v51P.hex $ddir $tag &&
        copyit ArduCopter-vrbrain-v51Pro.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v51Pro.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v51Pro.hex $ddir $tag &&
        copyit ArduCopter-vrbrain-v51ProP.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v51ProP.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v51ProP.hex $ddir $tag &&
        copyit ArduCopter-vrbrain-v52.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v52.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v52.hex $ddir $tag &&
        copyit ArduCopter-vrbrain-v52P.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v52P.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v52P.hex $ddir $tag &&
        copyit ArduCopter-vrbrain-v52Pro.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v52Pro.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v52Pro.hex $ddir $tag &&
        copyit ArduCopter-vrbrain-v52ProP.vrx $ddir $tag && 
        copyit ArduCopter-vrbrain-v52ProP.bin $ddir $tag && 
        copyit ArduCopter-vrbrain-v52ProP.hex $ddir $tag &&
        copyit ArduCopter-vrubrain-v51.vrx $ddir $tag && 
        copyit ArduCopter-vrubrain-v51.bin $ddir $tag && 
        copyit ArduCopter-vrubrain-v51.hex $ddir $tag && 
        copyit ArduCopter-vrubrain-v51P.vrx $ddir $tag && 
        copyit ArduCopter-vrubrain-v51P.bin $ddir $tag && 
        copyit ArduCopter-vrubrain-v51P.hex $ddir $tag &&
        copyit ArduCopter-vrubrain-v52.vrx $ddir $tag && 
        copyit ArduCopter-vrubrain-v52.bin $ddir $tag && 
        copyit ArduCopter-vrubrain-v52.hex $ddir $tag
    done
    checkout Copter "latest" ""
    popd
}

# build rover binaries
build_rover() {
    tag="$1"
    echo "Building APMrover2 $tag binaries from $(pwd)"
    pushd APMrover2
    echo "Building APMrover2 VRX binaries"
    ddir=$binaries/Rover/$hdate/VRX
    checkout Rover $tag "" || {
        checkout Rover "latest" ""
        popd
        return
    }
    skip_build $tag $ddir || {
        make vrbrain-clean && make vrbrain || {
            echo "Failed build of APMrover2 VRX $tag"
            error_count=$((error_count+1))
            checkout Rover "latest" ""
            popd
            return
        }
        copyit APMrover2-vrbrain-v45.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v45.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v45.hex $ddir $tag &&
        copyit APMrover2-vrbrain-v45P.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v45P.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v45P.hex $ddir $tag &&
        copyit APMrover2-vrbrain-v51.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v51.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v51.hex $ddir $tag &&
        copyit APMrover2-vrbrain-v51P.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v51P.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v51P.hex $ddir $tag &&
        copyit APMrover2-vrbrain-v51Pro.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v51Pro.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v51Pro.hex $ddir $tag &&
        copyit APMrover2-vrbrain-v51ProP.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v51ProP.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v51ProP.hex $ddir $tag &&
        copyit APMrover2-vrbrain-v52.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v52.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v52.hex $ddir $tag &&
        copyit APMrover2-vrbrain-v52P.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v52P.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v52P.hex $ddir $tag &&
        copyit APMrover2-vrbrain-v52Pro.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v52Pro.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v52Pro.hex $ddir $tag &&
        copyit APMrover2-vrbrain-v52ProP.vrx $ddir $tag && 
        copyit APMrover2-vrbrain-v52ProP.bin $ddir $tag && 
        copyit APMrover2-vrbrain-v52ProP.hex $ddir $tag &&
        copyit APMrover2-vrubrain-v51.vrx $ddir $tag && 
        copyit APMrover2-vrubrain-v51.bin $ddir $tag && 
        copyit APMrover2-vrubrain-v51.hex $ddir $tag && 
        copyit APMrover2-vrubrain-v51P.vrx $ddir $tag && 
        copyit APMrover2-vrubrain-v51P.bin $ddir $tag && 
        copyit APMrover2-vrubrain-v51P.hex $ddir $tag &&
        copyit APMrover2-vrubrain-v52.vrx $ddir $tag && 
        copyit APMrover2-vrubrain-v52.bin $ddir $tag && 
        copyit APMrover2-vrubrain-v52.hex $ddir $tag
    }
    checkout Rover "latest" ""
    popd
}

if [ "$#" = 0 ]
then
    #Any parameters are passed: I have to build all
    echo "Build all version for all vehicle"
    for build in beta stable latest; do
        echo "Build $build for Plane"
        build_arduplane $build
        echo "Build $build for Copter"
        build_arducopter $build
        echo "Build $build for Rover"
        build_rover $build
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
                    echo "Build $build for Plane"
                    build_arduplane $build
                elif [ "$1" = "Copter" ]
                then
                    echo "Build $build for Copter"
                    build_arducopter $build
                elif [ "$1" = "Rover" ]
                then
                    echo "Build $build for Rover"
                    build_rover $build
                fi
            done
        else
            echo "Build version $2 for vehicle $1"
            if [ "$1" = "Plane" ]
            then
                echo "Build $2 for $1"
                build_arduplane $2
            elif [ "$1" = "Copter" ]
            then
                echo "Build $2 for $1"
                build_arducopter $2
            elif [ "$1" = "Rover" ]
            then
                echo "Build $2 for $1"
                build_rover $2
            fi
        fi
    else
        if [ "$1" = "beta" ] || [ "$1" = "stable" ] || [ "$1" = "latest" ]
        then
            #First parameter is the version: I have to build this version for all vehicle
            echo "Build version $1 for all vehicle"
            echo "Build $1 for Plane"
            build_arduplane $build
            echo "Build $1 for Copter"
            build_arducopter $build
            echo "Build $1 for Rover"
            build_rover $build
        else
            echo "Error"
        fi
    fi
fi

rm -rf $TMPDIR

exit $error_count
