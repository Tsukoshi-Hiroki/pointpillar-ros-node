#!/bin/bash

# スクリプトをエラーで停止させる
set -e

# 保存先ディレクトリを設定
SAVE_DIR_1=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array
SAVE_DIR_2=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array_size
SAVE_DIR_3=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array_label
SAVE_DIR_4=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array_ped_size

# 保存先ディレクトリが存在しない場合は作成
if [ ! -d "$SAVE_DIR_1" ]; then
    echo "Creating directory: $SAVE_DIR_1"
    mkdir -p "$SAVE_DIR_1"
fi

if [ ! -d "$SAVE_DIR_2" ]; then
    echo "Creating directory: $SAVE_DIR_2"
    mkdir -p "$SAVE_DIR_2"
fi

if [ ! -d "$SAVE_DIR_3" ]; then
    echo "Creating directory: $SAVE_DIR_3"
    mkdir -p "$SAVE_DIR_3"
fi

if [ ! -d "$SAVE_DIR_4" ]; then
    echo "Creating directory: $SAVE_DIR_4"
    mkdir -p "$SAVE_DIR_4"
fi

# roslaunch をバックグラウンドで実行
echo "Starting roslaunch in the background..."
roslaunch pointpillar_ros main.launch &

# 少し待機して rostopic を実行
sleep 5
echo "Recording /detections topic to $SAVE_DIR_1/bbox_array.csv..."
echo "Recording /detections topic to $SAVE_DIR_2/bbox_array_size.csv..."

rostopic echo -p /detections > "$SAVE_DIR_1/bbox_array.csv" &
rostopic echo -p /detections/array_size > "$SAVE_DIR_2/bbox_array_size.csv" &
rostopic echo -p /label_count > "$SAVE_DIR_3/bbox_array_label.csv" &
rostopic echo -p /detections/ped_size > "$SAVE_DIR_4/bbox_array_ped_size.csv" &

echo "Script execution complete. Data saved to $SAVE_DIR_1/bbox_array.csv."
echo "Script execution complete. Data saved to $SAVE_DIR_2/bbox_array_size.csv."

sleep 30  
rosnode kill /record_bag &

sleep 1
echo "Kill all ros nodes!"
ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
# exit 0
