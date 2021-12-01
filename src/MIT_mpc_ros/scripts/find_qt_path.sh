#/bin/bash

QT_VER="$(qtchooser -print-env |tail -1| cut -d/ -f2-6)"

printf "/${QT_VER}"

