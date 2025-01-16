#!/bin/bash

# 入力ディレクトリと出力ファイル名
input_file="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/average_array_size.csv"
output_file="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/average_average_array_size.csv"

# 入力チェック
if [[ -z "$input_file" || -z "$output_file" ]]; then
    echo "使用方法: $0 <入力ディレクトリ> <出力ファイル名>"
    exit 1
fi

# 入力ディレクトリと出力ファイルの展開
input_file=$(eval echo "$input_file")
output_file=$(eval echo "$output_file")

# 出力ファイルが存在しない場合は作成
if [[ ! -f "$output_file" ]]; then
    touch "$output_file"
fi

# 一行目（ヘッダー）を除外し、1列目の数値の平均を計算
average=$(tail -n +2 "$input_file" | awk -F',' '{sum += $1; count++} END {if (count > 0) print sum / count}')

# 出力CSVファイルに平均値を保存
echo "Average_Average_Size" > "$output_file"
echo "$average" >> "$output_file"

echo "平均値が $output_file に出力されました。"
