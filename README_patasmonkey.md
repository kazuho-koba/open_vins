# Patasmonkey向け OpenVINS 運用メモ

この文書はPatasmonkeyのOAK-D S2を入力にしたOpenVINS検証のための補足です。
upstreamのOpenVINS一般説明は[ReadMe.md](ReadMe.md)を参照してください。

## 使用設定

Patasmonkey launchは`pm_config/config/oak_d_s2/estimator_config1.yaml`を使います。camera–IMU
校正は、接続したOAKのMX IDに対応するYAMLを`pm_config`から指定します。別個体の
校正値を流用すると、VIOのscale・姿勢・初期化安定性を損なうため避けてください。

## 停止中の静的初期化＋起動時ZUPT

Patasmonkey側の上記YAMLでは`try_zupt: true`と`zupt_only_at_beginning: true`で、
jerkを加えず停止中に初期化できる。画像とIMUを約3秒以上取得し、視差と
加速度変動の静止判定が通るまで車体を停止させる。従来のjerk待ちへ戻す場合は
`try_zupt: false`にする。`init_dyn_use: false`と正の`init_imu_thresh`は維持する。
OpenVINS内の`config/oakd_s2/`コピーではなく、実際に渡した`config_path`を確認する。

ZUPTが成功したときにも`timelastupdate`を進め、初期化済みの静止状態をROS出力の
対象にする。upstreamでは通常の特徴更新前に`initialized()`がfalseのままとなり、
停止中の初期化成功ログが出てもpose/odometry/native stateが公開されなかった。
本変更はZUPTの推定式や受入れ条件を変えず、成功した状態更新の公開を可能にする。
実画像入力とsimulation入力の両方で同じ扱いにする。

停止中のpose/pathはZUPT更新、`odomimu`はIMU時刻への短時間予測から生成される。
この段階では通常の特徴更新がなく、特徴点群は空でも正常である。
`timing.txt`は通常の特徴更新を計測するため、ZUPTだけの区間には行が増えない。
初期yawは任意の基準であり、停止初期化は地球基準headingを与えない。

`zupt_only_at_beginning`の解除状態は`has_moved_since_zupt`で管理する。
ZUPTの公開開始ではこの値を変更せず、通常のVIO処理へ移行した後のZUPT再投入を
既存ロジックで抑止する。静止判定が誤ると低速移動を抑制し得るため、起動時は停止する。

## `track_frequency`の意味

`track_frequency`は「入力画像topicの期待周波数」ではなく、OpenVINS frontendが
特徴追跡へ投入する最大周波数です。ROS2Visualizerは、前回採用画像から
`1 / track_frequency`より短い間隔の画像を意図的に捨てます。

したがって20 Hz画像入力に`track_frequency: 20.0`を設定しても、timestamp丸め・
僅かなjitterにより実効更新は20 Hz未満になり得ます。値を上げると間引きは減ります
が、frontend/back-end負荷、キュー滞留、処理遅延を必ず同時に測定してください。

## 記録する情報

### IMU積分区間の異常診断

`propagate_and_clone`の積分時間チェックが失敗する場合、consoleに
`[PROP_INTERVAL]`を出力する。`time0/time1`はIMU時刻系の要求端点（秒）、
`dt_summed`は実際に選択した隣接IMU区間の合計である。
`buffer_*`は選択時点の元buffer、`selected_*`は補間・外挿後の区間を示す。
件数0の端点0は欠測を表す便宜値であり、実測時刻ではない。
異常時の `[PROP_CLEAN_REQUEST]` は直近64回のbuffer削除要求である。
`source` は通常IMU投入、camera ZUPT、simulation ZUPTを区別し、
`threshold` 未満の時刻を削除対象とする。実際の削除件数を表すものではなく、
64回より古い要求は残らない。正常時は履歴をメモリ保持するだけで出力しない。
`offset_old/new`とcamera/state時刻も同時に記録し、開始不足・終了不足・
時間補正変化を切り分ける。正常時には出力せず、既存assertionは維持する。

`pm_bag_global_localization.launch.py`は通常のOpenVINS出力に加え、以下を同一試行の
bagディレクトリ`openvins/`へ保存します。

- `state_estimate.txt`：状態、速度、IMU bias、オンライン校正状態
- `state_deviation.txt`：各状態の標準偏差
- `timing.txt`：更新処理時間（有効時）
- `console.log`：初期化、リセット、更新に関する標準出力
- 実効パラメータ、config snapshot、Git revision/diff

rosbagには`/ov_msckf/trackhist`、`/ov_msckf/points_msckf`、
`/ov_msckf/points_slam`も記録します。前二者は特徴追跡・採択点の後解析に使えます。

## VIO不調時の扱い

OpenVINS出力を直接global poseとして扱いません。Patasmonkey側のVIO速度・高さ
ゲートが、有限性、速度、急変、z不連続を検査してからlocalizerへ渡します。VIOが
異常なら、水平はwheel + Wit `wz`で継続し、高さは最後に検証済みのzで保持します。
これはVIOを正常化する機能ではなく、VIO破綻が全体のodometryを破壊しないための
安全策です。

## 検証手順

### IMU bufferの並行アクセス

PropagatorとZUPT updaterは、IMU callbackによる追加と画像更新threadによる
削除・区間抽出を同じmutexで保護します。public cleanerは自身でロックを取得し、
ロック済みのfeedは内部helperを使って二重ロックを避けます。必要なIMU区間を
別vectorへ取り出した後はロックを解放し、EKF計算中のIMU受信を妨げません。
保持時間、ZUPT判定閾値、積分assertionは変更していません。並行アクセスの
修正だけで入力欠測や過剰な履歴削除が解決するとは限らないため、異常時の
区間診断と実機再検証は引き続き必要です。

1. OAK-DのMX IDとUSB 3.x接続を`depthai_driver`の診断で確認する。
2. 停止状態でOpenVINSを起動し、初期化成功と出力開始を確認する。jerk待ち設定を
   選んだ場合だけ、静止区間を確保してから初期化に必要なセンサ運動を与える。
3. bagの画像・IMU・`trackhist`・OAK診断・OpenVINS native logを同一時刻で比較する。
4. VIO速度/高さゲートの診断を確認し、隔離の契機を記録する。

### IMU受信履歴の比較診断

非同期画像更新threadに渡すIMU到達時刻は値キャプチャします。callbackのローカル
`message`を参照キャプチャすると、detach後に破棄済みの変数へアクセスし得ます。
この修正は変数寿命の不具合を除きますが、DDS欠測や長い欠測区間の伝播問題を
解消するものではありません。購読QoS・画像間引き・EKF設定は変更しません。

画像queueのmutexは観測の検査・取り出しだけに使用し、VIO更新・可視化中は
解放します。観測のcv::Matは取り出したローカル変数が所有します。camera更新は
既存の単一更新threadで直列処理し、画像callbackのqueue投入待ちによってIMU受信
callbackまで止まることを避けます。queue容量や欠測区間の補間規則は変えません。

`subscribe.launch.py`の`imu_history_depth`は既定5件です。125 Hzでは約40 ms分の
履歴に相当するため、batch配送・callback遅延の検証時は64件（約512 ms）との
比較ができます。これは約254 msのpublish待ち実測値の約2倍を確保する試験値であり、
欠測の解消を保証する値ではありません。BEST_EFFORTは維持し、画像QoSは変更しません。

`imu_input_diagnostics:=true`では、受信callbackでheader stamp間隔が40 ms超または
逆行した場合だけ`[IMU_INPUT_GAP]`を出力します。`receipt`はROS時刻、他の値は
header時刻の秒単位です。bagの同区間と比較し、publisher側の欠測と購読側の欠測を
区別してください。これは全受信履歴ではなく、40 ms以下の欠測は検出しません。
同じ診断ON時には、画像callbackのcamera queue mutex待ちが40 msを超えると
`[CAMERA_QUEUE_WAIT]`を出力します。待ち時間はsteady clockで測ったms、stampは
画像headerの秒です。重い画像更新による受信callback停滞の切り分けに使います。
深さは1〜1024件に制限し、既定挙動は変更していません。

VIOリセット後の自動再利用は、Patasmonkey側の速度ゲートが一定期間の数値的健全性を
確認してから行います。これは意味論的な正常性まで保証しないため、画像特徴とnative
state logを必ず併用して評価してください。

### 欠測区間の更新前検査

ZUPTと通常propagationは、抽出IMU系列の先頭・末尾が要求区間の両端に一致し、
時刻が有限かつ単調増加することを`ImuIntervalCheck.h`で検査します。
許容差は1 usで、秒単位の丸め誤差用です。開始端点が欠けた系列をZUPTが受理して
状態時刻を進めると、cleanupが後続propagationに必要なIMUまで削除してしまうため、
不完全な区間は状態・共分散・cloneを変更する前に見送ります。
ZUPTのcamera–IMU offsetも、受理できた更新に対応する値だけを確定します。

見送り時は`[ZUPT_IMU_INTERVAL_REJECT]`／`[PROP_IMU_INTERVAL_REJECT]`を記録します。
これは欠測の修復・自動リセットではなく、不完全な積分によるabortや誤更新を避ける
保護です。一時的な端点不足では推定を見送ります。保持bufferの先頭まで要求開始時刻を
越えた復元不能な場合は`[VIO_IMU_RESET_REQUIRED]`を出し、非復帰ラッチを立てます。
以後の画像更新・IMU蓄積・高レートodom公開を停止し、停止中のbuffer肥大化や
古い状態に新しいtimestampだけを付ける出力を防ぎます。ノードの再起動・再初期化が
必要であり、自動リセットは実装していません。下流はVIO出力の鮮度で隔離してください。
端点間の長い欠測・既存の補間／外挿の妥当性は別途検証が必要です。
積分後の既存assertionは残しています。

端点欠落・逆順・NaN・実ログの失敗時刻を含む単体試験は、repository rootで次のように
実行できます。これはROS不要の純C++試験で、ROSパッケージのbuildはFoxy環境で行います。

```bash
g++ -std=c++14 -Wall -Wextra -pedantic -Iov_msckf/src \
  ov_msckf/src/test/test_imu_interval_check.cpp -o /tmp/imu_interval_test
/tmp/imu_interval_test
```

## VIO非復帰停止時の追加診断

`VIO_IMU_RESET_REQUIRED`が出た場合だけ、`VIO_IMU_RESET_BUFFER`に区間選択時の
実IMU buffer件数・先頭/末尾時刻[秒]を残し、`VIO_IMU_RESET_CLEAN`に既存の
有界履歴（最大64件）の削除要求元と閾値を出します。入力未到達と過去データ削除を
切り分ける目的です。正常時のログ量、buffer保持期間、積分方法、安全停止条件は変更しません。
プロセス生存やZUPT logだけで健全性を判断せず、odomimuの継続出力も確認してください。

`imu_input_diagnostics:=true`では欠測時の`IMU_GAP_CONTEXT`にcallback開始間隔と
直前のIMU feed／高レートodometry処理時間[ms]も残します。
headerが連続でもcallback開始間隔が40 msを超えた場合は`IMU_CALLBACK_DELAY`を
出し、購読履歴で吸収された受信停滞も確認できます。
`VIO_IMU_CALLBACK_SLOW`はその処理が20 msを超えた場合、`VIO_CALLBACK_SLOW`は
画像callback全体が20 msを超えた場合だけ出ます。変換・コピー・mask生成などを含み、
queue待ちだけでは捉えられない停滞を調べます。
steady_clockのwall時間であり、CPU実行時間やDDS内部受信時刻ではありません。
callbackに入る前の待ちやOSによる停止も別に検証する必要があります。
診断OFFでは新しい時計計測・ログは無効です。購読数、QoS、callback並列性、
積分方法と欠測時の安全停止条件は変更しません。

## 空のcamera queueでは更新threadを作らない

IMU callbackはfeedと高レートodometry公開の後、camera queueとworker起動flagを
同じmutexで確認する。画像がなければ戻り、画像がある場合だけ更新threadを起動する。
IMU全件の蓄積・odom公開を間引く変更ではない。後からqueueに入った画像は次のIMU
callbackで従来の到達時刻検査を通して処理する。既存workerのcamera更新は引き続き直列。
画像到着直後に空workerが偶然拾う動作はなくなるが、入力headerや更新時刻規約は維持する。
画像より高周波のIMUごとに空threadを作る無駄を減らす改修であり、DDSの欠測・OS待ちを
根治する保証ではない。callback groupやQoSは変更しない。
