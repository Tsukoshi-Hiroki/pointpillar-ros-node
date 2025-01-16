#!/bin/bash

# 入力と出力ファイルを指定
input_file="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/average_value.csv"
output_file="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/average_average_value.csv"

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

# 一行目を除外し、1列目でグループ化して2列目の平均を計算
awk -F, 'NR > 1 { sum[$1] += $2; count[$1]++ } 
         END { 
           print "Label,Average_Average_Value" > "'$output_file'"
           for (key in sum) {
             print key "," sum[key]/count[key] >> "'$output_file'"
           }
         }' "$input_file"

echo "計算が完了しました。結果は '$output_file' に保存されました。"
