# attribute_recognition
ロボカップ2023＠ホーム世界大会のfind my mates用の人属性認識のプログラム。

# 依存関係
```bash  
pip3 install -r requirements.txt
```  

# 実行方法
```bash
ros2 run attribute_recognition attribute_recog_node
```

# トピック
## /image_raw : sensor_msgs/Image
購読するトピック。認識したい対象画像を入力する。今後はactionlibもしくはserviceに実装を変更予定。

## /findmymates/attribute_sentence : std_msgs/String    
出力内容の人物の属性を紹介する文章。  
発話する場合は\nで一行ずつ発話させるのがおすすめ。  
