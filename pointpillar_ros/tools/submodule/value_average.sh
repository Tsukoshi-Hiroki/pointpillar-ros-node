#!/bin/bash

# 入力ディレクトリと出力ファイル名
input_dir="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/bbox_array"
output_file="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/average_value.csv"

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
echo "Label, Average_Value" > "$output_file"

# 指定ディレクトリ内のCSVファイルを順に処理
for csv_file in "$input_dir"/bbox_array_*.csv; do
    # ファイルが存在しない場合はスキップ
    if [[ ! -f "$csv_file" ]]; then
        continue
    fi

    # グループ化と平均計算
    tail -n +2 "$csv_file" | awk -F',' '
    {
        for (n = 0; n <= 9 ; n++) {
            group_index = 4 + (15 * n)
            value_index = 3 + (15 * n)
            if (group_index > NF || value_index > NF) break
            
            group_value = $group_index
            value = $value_index
            
            if (group_value ~ /^[0-9.]+$/ && value ~ /^[0-9.]+$/) {
                group_sums[group_value] += value
                group_counts[group_value]++
            }
        }
    }
    END {
        for (group in group_sums) {
            average = group_sums[group] / group_counts[group]
            print group "," average
        }
    }' >> "$output_file"
done

# 終了メッセージ
echo "計算完了。結果は $output_file に保存されました。"
