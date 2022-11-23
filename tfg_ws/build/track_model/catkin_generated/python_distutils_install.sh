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

echo_and_run cd "/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/src/track_model"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/install/lib/python3/dist-packages:/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/build" \
    "/usr/bin/python3" \
    "/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/src/track_model/setup.py" \
     \
    build --build-base "/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/build/track_model" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/install" --install-scripts="/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/install/bin"
