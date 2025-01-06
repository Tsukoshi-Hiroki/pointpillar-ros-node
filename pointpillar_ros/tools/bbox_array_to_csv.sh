#!/bin/bash

# スクリプトをエラーで停止させる
set -e

# 保存先ディレクトリを設定
SAVE_DIR_1=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array
SAVE_DIR_2=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array_size

# 保存先ディレクトリが存在しない場合は作成
if [ ! -d "$SAVE_DIR_1" ]; then
    echo "Creating directory: $SAVE_DIR_1"
    mkdir -p "$SAVE_DIR_1"
fi

if [ ! -d "$SAVE_DIR_2" ]; then
    echo "Creating directory: $SAVE_DIR_2"
    mkdir -p "$SAVE_DIR_2"
fi

# roslaunch をバックグラウンドで実行
echo "Starting roslaunch in the background..."
roslaunch pointpillar_ros bag_play_for_multi.launch &

# 少し待機して rostopic を実行
sleep 5
echo "Recording /detections topic to $SAVE_DIR_1/bbox_array.csv..."
echo "Recording /detections topic to $SAVE_DIR_2/bbox_array_size.csv..."
rostopic echo -p /detections > "$SAVE_DIR_1/bbox_array.csv" &
rostopic echo -p /detections/array_size > "$SAVE_DIR_2/bbox_array_size.csv" &

echo "Script execution complete. Data saved to $SAVE_DIR_1/bbox_array.csv."
echo "Script execution complete. Data saved to $SAVE_DIR_2/bbox_array_size.csv."

sleep 100 
echo "Kill all ros nodes!"
ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
# exit 0
