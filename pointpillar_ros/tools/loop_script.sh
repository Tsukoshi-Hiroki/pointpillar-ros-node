
#!/bin/bash

# ループ回数
LOOP_COUNT=30

# ベーススクリプトのパス
SCRIPT_PATH="./submodule/bbox_array_to_csv.sh"

# ループ実行
for i in $(seq 1 $LOOP_COUNT); do
    echo "Executing loop $i..."

    # SAVE_DIRのパスを一時的に変更し、ファイル名をインクリメント
    export SAVE_DIR_1=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array
    export SAVE_DIR_2=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array_size
    export SAVE_DIR_3=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array_label
    export SAVE_DIR_4=~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/bbox_array_ped_size
    export FILE_NAME_1="bbox_array_$i.csv"
    export FILE_NAME_2="bbox_array_size_$i.csv"
    export FILE_NAME_3="bbox_array_label_$i.csv"
    export FILE_NAME_4="bbox_array_ped_size_$i.csv"

    # ベーススクリプトを実行
    bash "$SCRIPT_PATH"

    # データファイルをリネーム（念のため上書き防止）
    mv "$SAVE_DIR_1/bbox_array.csv" "$SAVE_DIR_1/$FILE_NAME_1"
    mv "$SAVE_DIR_2/bbox_array_size.csv" "$SAVE_DIR_2/$FILE_NAME_2"
    mv "$SAVE_DIR_3/bbox_array_label.csv" "$SAVE_DIR_3/$FILE_NAME_3"
    mv "$SAVE_DIR_4/bbox_array_ped_size.csv" "$SAVE_DIR_4/$FILE_NAME_4"

    echo "Loop $i completed. Data saved to $SAVE_DIR_1/$FILE_NAME_1."
    echo "Loop $i completed. Data saved to $SAVE_DIR_2/$FILE_NAME_2."
    echo "Loop $i completed. Data saved to $SAVE_DIR_3/$FILE_NAME_3."
    echo "Loop $i completed. Data saved to $SAVE_DIR_4/$FILE_NAME_4."

    sleep 5
done

echo "All loops completed."
