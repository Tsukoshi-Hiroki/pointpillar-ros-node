#!/bin/bash

# 入力ディレクトリと出力ファイル名
input_dir="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/bbox_array_size"
output_file="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/average_array_size.csv"

echo "Directory is $1"

# 入力チェック
if [[ -z "$input_dir" || -z "$output_file" ]]; then
    echo "使用方法: $0 <入力ディレクトリ> <出力ファイル名>"
    exit 1
fi

# 入力ディレクトリと出力ファイルの展開
input_dir=$(eval echo "$input_dir")
output_file=$(eval echo "$output_file")

# 出力ファイルが存在しない場合は作成
if [[ ! -f "$output_file" ]]; then
    touch "$output_file"
fi

# 書き出し先のヘッダーを作成
echo "Average_Array_Size" > "$output_file"

# 指定ディレクトリ内のCSVファイルを順に処理
for csv_file in "$input_dir"/bbox_array_size_*.csv; do
    # ファイルが存在しない場合はスキップ
    if [[ ! -f "$csv_file" ]]; then
        continue
    fi

    # B列の数値を取得し、1行目をスキップして平均を計算
    average=$(tail -n +2 "$csv_file" | cut -d',' -f2 | awk '{if ($1 ~ /^[0-9.]+$/) {sum+=$1; count++}} END {if (count > 0) print sum / count}')

    # 平均値を出力ファイルに追記
    echo "$average" >> "$output_file"
done

# 終了メッセージ
echo "計算完了。結果は $output_file に保存されました。"
