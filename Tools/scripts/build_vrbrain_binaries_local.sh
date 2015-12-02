#!/bin/bash

export PATH=$PATH:/bin:/usr/bin

export TMPDIR=$PWD/build.tmp.$$
echo $TMPDIR
rm -rf $TMPDIR
echo "Building in $TMPDIR"

date




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
    withgit="$2"
    board="$3"

    echo "Building ArduPlane $tag binaries from $(pwd)"
    pushd ArduPlane
    echo "Building ArduPlane $board binaries"
    ddir=$binaries/Plane/$hdate/VRX







    skip_build $tag $ddir || {
        make vrbrain-clean && make $board || {
            echo "Failed build of ArduPlane VRX $tag"
            error_count=$((error_count+1))

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

    popd
}

# build copter binaries
build_arducopter() {
    tag="$1"
    withgit="$2"
    board="$3"

    echo "Building ArduCopter $tag binaries from $(pwd)"
    pushd ArduCopter
    frames="quad tri hexa y6 octa octa-quad heli"
    for f in $frames; do






        echo "Building ArduCopter $board-$f binaries"
        ddir="$binaries/Copter/$hdate/VRX-$f"
        skip_build $tag $ddir && continue
        make vrbrain-clean && make $board-$f || {
            echo "Failed build of ArduCopter VRX $tag"
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

    popd
}

# build rover binaries
build_rover() {
    tag="$1"
    withgit="$2"
    board="$3"

    echo "Building APMrover2 $tag binaries from $(pwd)"
    pushd APMrover2
    echo "Building APMrover2 $board binaries"
    ddir=$binaries/Rover/$hdate/VRX





    skip_build $tag $ddir || {
        make vrbrain-clean && make $board || {
            echo "Failed build of APMrover2 VRX $tag"
            error_count=$((error_count+1))

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

    popd
}

build_vehicle() {
    vehicle="$1"
    release="$2"
    board="$3"

    withgit='1'
    if [ "$release" = "local" ]; then
        withgit='0'
    fi

    echo "Build $release for $vehicle with git $withgit for board $board"

    case "$vehicle" in
        Copter) build_arducopter $release $withgit $board ;;
        Plane) build_arduplane $release $withgit $board ;;
        Rover) build_rover $release $withgit $board ;;
        *) error "Unexpected vehicle $vehicle" ;;
    esac
}

# Parameters:
# -v: vehicle (nothing (all), Copter, Plane, Rover)
# -r: release (nothing (all), stable, beta, latest, local (without git checkout))

rflag='local'
vflag='Plane Copter Rover'
bflag='vrbrain'

while getopts 'v:b:' flag; do
    case "${flag}" in

        v) vflag="${OPTARG}" ;;
        b) bflag="${OPTARG}" ;;
        *) error "Unexpected option ${flag}" ;;
    esac
done

echo "Parameters - release: $rflag; vehicle: $vflag; boards: $bflag"

for v in $vflag; do
    for r in $rflag; do
        build_vehicle $v $r $bflag
    done
done

rm -rf $TMPDIR

exit $error_count
