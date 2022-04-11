#!/bin/bash

# cd Examples && sh euroc_eval_examples.sh

pathDataset="/root/raspi_stream/record/imu-calib-timestamp"

./Monocular/mono_own ../Vocabulary/ORBvoc.txt ./Monocular/own-right.yaml $pathDataset/record_vidacc_stream-0000.mp4 $pathDataset/record_vidacc_stream-0000-accTime.txt dataset-ownTest_monoi
