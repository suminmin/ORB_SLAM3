#!/bin/bash

# docker run --rm -it -e "DISPLAY=host.docker.internal:0.0" -e "QT_X11_NO_MITSHM=1" -v C:/workspace/raspi_Public/raspi_stream:/root/raspi_stream -v C:/workspace/docker/orbslam3/ORB_SLAM3:/ORB_SLAM3 -it orbslam3:latest-cuda /bin/bash

docker run --rm -it -e "DISPLAY=host.docker.internal:0.0" -e "QT_X11_NO_MITSHM=1" -v /mnt/c/workspace/raspi_Public:/root/raspi_Public -v /mnt/c/workspace/sumi_repository/vslam/ORB_SLAM3:/ORB_SLAM3 -it orbslam3:latest-cuda /bin/bash

# docker run -it -e "DISPLAY=host.docker.internal:0.0" -e "QT_X11_NO_MITSHM=1" -v /mnt/c/workspace/sumi_repository/vslam/ORB_SLAM3:/ORB_SLAM3 -it orbslam3:latest-cuda /bin/bash
