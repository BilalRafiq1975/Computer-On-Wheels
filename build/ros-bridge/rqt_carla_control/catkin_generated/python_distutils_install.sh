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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/maanz-ai/ahmed-workspace/src/ros-bridge/rqt_carla_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/maanz-ai/ahmed-workspace/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/maanz-ai/ahmed-workspace/install/lib/python3/dist-packages:/home/maanz-ai/ahmed-workspace/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/maanz-ai/ahmed-workspace/build" \
    "/usr/bin/python3" \
    "/home/maanz-ai/ahmed-workspace/src/ros-bridge/rqt_carla_control/setup.py" \
     \
    build --build-base "/home/maanz-ai/ahmed-workspace/build/ros-bridge/rqt_carla_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/maanz-ai/ahmed-workspace/install" --install-scripts="/home/maanz-ai/ahmed-workspace/install/bin"
