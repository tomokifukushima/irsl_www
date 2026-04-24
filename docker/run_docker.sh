#!/bin/bash

abs_dir=$(dirname "$(readlink -f "$0")")

while [[ $# -gt 0 ]]; do
    case $1 in
        --ros-setup)
            _ROS_SETUP="$2"
            shift
            shift
            ;;
        --ros-ip)
            export ROS_IP="$2"
            shift
            shift
            ;;
        --ros-hostname)
            export ROS_HOSTNAME="$2"
            shift
            shift
            ;;
        --ros-master-uri)
            export ROS_MASTER_URI="$2"
            shift
            shift
            ;;
        --help)
            echo "run_docker.sh [--ros-setup ROS_SETUP] [--ros-ip ROS_IP] [--ros-hostname ROS_HOSTNAME] [--ros-master-uri ROS_MASTER_URI] "
            exit 0
            ;;
        *)
            shift
            ;;
    esac
done

if [[ -z "${ROS_MASTER_URI}" ]]; then
    if [[ -n "${ROS_HOSTNAME}" ]]; then
        export ROS_MASTER_URI="http://${ROS_HOSTNAME}:11311"
    elif [[ -n "${ROS_IP}" ]]; then
        export ROS_MASTER_URI="http://${ROS_IP}:11311"
    fi
fi

sig_hdl () {
    echo "catch signal $1"
    kill -INT ${COMP_PID}
    exit 0
}
trap "sig_hdl SIGTERM" SIGTERM
trap "sig_hdl SIGINT"  SIGINT
trap "sig_hdl SIGHUP"  SIGHUP
trap "sig_hdl SIGKILL" SIGKILL

set -x
ROOT_DIR=${abs_dir} docker compose -f ${abs_dir}/www-compose-linux-ssl.yaml up &

COMP_PID="$!"

wait ${COMP_PID}

exit 0
