#!/bin/bash

# 入力ディレクトリと出力ファイル名
input_dir="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/bbox_array_label"
output_file="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/average_detection_ratio.csv"

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

# 初期化
sum_pedestrian=0
sum_cyclist=0
sum_car=0
sum_none=0
file_count=0

# 入力ディレクトリ内の各CSVファイルを処理
for file in "$input_dir"/*.csv; do
  if [ -f "$file" ]; then
    # 最終行を取得
    last_line=$(tail -n 1 "$file")

    # 最終行の3,4,5,6列目を抽出
    pedestrian=$(echo "$last_line" | awk -F"," '{print $3}')
    cyclist=$(echo "$last_line" | awk -F"," '{print $4}')
    car=$(echo "$last_line" | awk -F"," '{print $5}')
    none=$(echo "$last_line" | awk -F"," '{print $6}')

    # 数値として扱える場合のみ加算
    if [[ $pedestrian =~ ^[0-9]+(\.[0-9]+)?$ && $cyclist =~ ^[0-9]+(\.[0-9]+)?$ && $car =~ ^[0-9]+(\.[0-9]+)?$ && $none =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
      sum_pedestrian=$(echo "$sum_pedestrian + $pedestrian" | bc)
      sum_cyclist=$(echo "$sum_cyclist + $cyclist" | bc)
      sum_car=$(echo "$sum_car + $car" | bc)
      sum_none=$(echo "$sum_none + $none" | bc)
      file_count=$((file_count + 1))
    fi
  fi
done

# 平均値を計算
if [ $file_count -gt 0 ]; then
  avg_pedestrian=$(echo "$sum_pedestrian / $file_count" | bc -l)
  avg_cyclist=$(echo "$sum_cyclist / $file_count" | bc -l)
  avg_car=$(echo "$sum_car / $file_count" | bc -l)
  avg_none=$(echo "$sum_none / $file_count" | bc -l)

  avg_sum_detection=$(echo "$avg_pedestrian + $avg_cyclist + $avg_car + $avg_none" | bc)
  ratio_pedestrian=$(echo "100 * $avg_pedestrian / $avg_sum_detection" | bc -l)
  ratio_cyclist=$(echo "100 * $avg_cyclist / $avg_sum_detection" | bc -l)
  ratio_car=$(echo "100 * $avg_car / $avg_sum_detection" | bc -l)
  ratio_none=$(echo "100 * $avg_none / $avg_sum_detection" | bc -l)
  sum_avg_ratio=$(echo "$ratio_pedestrian + $ratio_cyclist + $ratio_car + $ratio_none" | bc)

  # 出力CSVファイルを作成
  echo "Pedestrian,Cyclist,Car,None Detection,Sum_Detection,ratio_Pedestrian,ratio_Cyclist,ratio_Car,ratio_None,Sum_Average_ratio" > "$output_file"
  echo "$avg_pedestrian,$avg_cyclist,$avg_car,$avg_none,$avg_sum_detection,$ratio_pedestrian,$ratio_cyclist,$ratio_car,$ratio_none,$sum_avg_ratio" >> "$output_file"

  echo "処理が完了しました。結果は $output_file に出力されました。"
else
  echo "有効なCSVファイルが見つかりませんでした。"
fi
