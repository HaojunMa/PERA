#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/haojun/Desktop/pera/src/pera/pera_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/haojun/Desktop/pera/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/haojun/Desktop/pera/install/lib/python2.7/dist-packages:/home/haojun/Desktop/pera/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/haojun/Desktop/pera/build" \
    "/usr/bin/python2" \
    "/home/haojun/Desktop/pera/src/pera/pera_control/setup.py" \
    build --build-base "/home/haojun/Desktop/pera/build/pera/pera_control" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/haojun/Desktop/pera/install" --install-scripts="/home/haojun/Desktop/pera/install/bin"
