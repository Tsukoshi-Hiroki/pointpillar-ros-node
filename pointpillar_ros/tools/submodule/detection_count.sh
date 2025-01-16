#!/bin/bash

# 入力ディレクトリと出力ファイル名
input_dir="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/bbox_array_label"
output_file="~/ws/src/pointpillar-ros-node/pointpillar_ros/csv/$1/average_detection_count.csv"

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

# 出力ファイルを初期化（ヘッダー行を追加）
echo "TotalDetectionCount,NoneDetection,MisDetection,1Ped,2Ped,3Ped,4Ped,5Ped,6Ped,7Ped,8Ped,9Ped,10Ped,11Ped,12Ped,13Ped,14Ped,15Ped,16Ped" > "$output_file"

# 各列の合計を保持する配列
declare -a sums
# CSVファイル数のカウンタ
file_count=0

# 入力ディレクトリ内のCSVファイルをループ処理
for csv_file in "$input_dir"/*.csv; do
  if [[ -f "$csv_file" ]]; then
    # CSVファイルの最終行を取得
    last_line=$(tail -n 1 "$csv_file")

    # 最終行をカンマで分割して配列に格納
    IFS=',' read -r -a columns <<< "$last_line"

    # 3列目以降の数値を処理
    for ((i=2; i<${#columns[@]}; i++)); do
      # 数値を取得（数値以外の場合は0に変換）
      value=$(echo "${columns[$i]}" | grep -Eo '[0-9]+([.][0-9]+)?' || echo "0")

      # sums配列に値を加算
      if [[ -z "${sums[$i]}" ]]; then
        sums[$i]="$value"
      else
        sums[$i]=$(echo "${sums[$i]} + $value" | bc)
      fi
    done

    # 処理したCSVファイルのカウントを増加
    file_count=$((file_count + 1))
  fi
done

# 平均値を計算して出力ファイルに書き込む
output_line=""
for ((i=2; i<${#sums[@]}; i++)); do
  # 平均値を計算
  average=$(echo "scale=5; ${sums[$i]} / $file_count" | bc)

  # ラベルを決定
  if [[ $i -eq 2 ]]; then
    label="Total Detection Count"
  elif [[ $i -eq 3 ]]; then
    label="None Detection"
  elif [[ $i -eq 4 ]]; then
    label="Mis Detection"
  else
    label="$((i-4)) Pedestrian"
  fi

  # 平均値を出力ラインに追加
  if [[ -z "$output_line" ]]; then
    output_line="$average"
  else
    output_line+=",$average"
  fi

done

echo "$output_line" >> "$output_file"
