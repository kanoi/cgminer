#!/usr/bin/bash
##  ===========================================================================
##  File    :   build-cgminer-gekko.sh
##  Notes   :   This script will build cgminer for GekkoScience Devices.
##          :   Originally developed to build cgminer on Raspberry Pi 5.
##  Who     :   David Means <w1t3h4t@gmail.com>
##  ===========================================================================

##  ===========================================================================
##  Changes these paths to match your envrionment
##  ===========================================================================
PROJECT_HOME=$HOME/Projects
CGMINER_DIR=$PROJECT_HOME/cgminer-Raspi
PATCH_DIR=$PROJECT_HOME/patches-cgminer
GIT_PROJECT=https://github.com/W1T3H4T/cgminer-Raspi.git
AUTOGEN_PARAMS="--enable-gekko --enable-icarus"
COPY_PATCHES=0
#
##  ===========================================================================
##  Do some reporting
##  ===========================================================================
function loginfo() {
    echo "[Info] $(date +'%d-%b %T %Y'): $@"
}

function logwarn() { 
    echo "[Warn] $(date +'%d-%b %T %Y'): $@"
}

function logerror() { 
    echo "[Error] $(date +'%d-%b %T %Y'): $@"
}

##  ===========================================================================
##  Do builds for patch testing
##  ===========================================================================
function do_patches() { 
    if [[ -z "$CGMINER_DIR" ]]; then
        echo
        logerror "CGMINER_DIR variable is (null)"
        echo
        exit 1
    fi
    if [[ -d $CGMINER_DIR ]]; then
        rm -rf $CGMINER_DIR
        git_clone
        cp -r $PATCH_DIR/* $CGMINER_DIR
        build_cgminer
    else
        logerror "Directory Not found: '$CGMINER_DIR'"
        exit 1
    fi
}

##  ===========================================================================
##  Give some platform info
##  ===========================================================================
function report_platform() {
    if [[ -r /etc/os-release ]]; then
        source /etc/os-release
        loginfo "Machine $(uname -m)"
        loginfo "OS is $ID $VERSION"
    else
        loginfo "Unknown platform"
    fi
}

##  ===========================================================================
##  Update the system
##  ===========================================================================
function apt_update() {
    echo 
    loginfo "Updating System"
    echo 
    sudo apt-get update
    sudo apt-get upgrade -y
}

##  ===========================================================================
##  Update our source
##  ===========================================================================
function git_clone() {
    if [[ -e ${CGMINER_DIR}/.git ]]; then
        echo 
        loginfo "Getting latest /kanoi/cgminer changes"
        echo
        cd $CGMINER_DIR && git pull
    else
        echo 
        loginfo "Cloning $GIT_PROJECT into ${CGMINER_DIR}"
        echo 
        cd ${PROJECT_HOME} && git clone $GIT_PROJECT
    fi
}

##  ===========================================================================
##  The default is _FORTIFY_SOURCE=1, but levels 2 and 3 are permissable.
##  More information about _FORTIFY_SOURCE: 
##  https://developers.redhat.com/articles/2022/09/17/gccs-new-fortification-level
##  ===========================================================================
function build_cgminer() {
    report_platform
    cd $CGMINER_DIR
    echo
    loginfo "Building solution in $(pwd)"
    echo
    CFLAGS="-O2 -march=native -fcommon -D_FORTIFY_SOURCE=1" ./autogen.sh $AUTOGEN_PARAMS
    [ $COPY_PATCHES ] && cp -r $PATCH_DIR .
    make clean
    make
}

##  ===========================================================================
##  Show the help
##  ===========================================================================
function show_help() { 
cat<<_EOF

Usage: $(basename $0) [--build] [--apt-update] [--git-update] [--patches] [--help|-h]

Perform system updates and build the solution, example:

user@host: \$ $(basename $0)

This script builds cgminer for Raspberry Pi 5, running Debian 12.5.

    --build         Do a build after updating the system and pulling/updating source.
    --apt           apt update and upgrade, and stop.
    --git           clone or pull updates from GitHub, and stop.
    --patches       Development only.  Clone from GitHub, install development changes, build.

Make changes to these script variables, as needed:
-> PROJECT_HOME=$PROJECT_HOME
-> CGMINER_DIR=$CGMINER_DIR
-> PATCH_DIR=$PATCH_DIR
-> GIT_PROJECT=$GIT_PROJECT

_EOF
}

##  ===========================================================================
##  Process ye'ol switches 
##  ===========================================================================
case "$@" in 
    --patches) do_patches
        ;;

    --git) git_clone
        ;;

    --apt) apt_update
        ;;

    -h|--help)
        show_help
        ;;

    --build)
        apt_update
        git_clone
        build_cgminer
        ;;

    "")
        show_help
        ;;
esac

