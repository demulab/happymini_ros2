# bash_launcher
## 概要
プログラムを一気に起動するbashのlauncher

## 設定
ファイルに権限付与
```
chmod +x ~/main_ws/src/happymini_ros2/bash_launcher/executions/fmm2023_launch.bash
chmod +x ~/main_ws/src/happymini_ros2/bash_launcher/bashes/*
```
</br>

ターミナルのプロファイル追加
> ターミナル「Preferences」の「Profiles」からbash_launcherというプロファイルを追加します
</br>

.bashrcに下記を追記
```
alias fmm2023_launch=~/main_ws/src/happymini_ros2/bash_launcher/executions/fmm2023_launch.bash
```
</br>

## 実行
ターミナルを開いて以下コマンドを実行
```
fmm2023_launch
```
プログラムが走らない場合は、`bash_launcher/executions/fmm2023_launch.bash`の`sleep`を調整
