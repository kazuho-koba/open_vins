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

1. OAK-DのMX IDとUSB 3.x接続を`depthai_driver`の診断で確認する。
2. 停止状態でOpenVINSを起動し、初期化成功と出力開始を確認する。jerk待ち設定を
   選んだ場合だけ、静止区間を確保してから初期化に必要なセンサ運動を与える。
3. bagの画像・IMU・`trackhist`・OAK診断・OpenVINS native logを同一時刻で比較する。
4. VIO速度/高さゲートの診断を確認し、隔離の契機を記録する。

VIOリセット後の自動再利用は、Patasmonkey側の速度ゲートが一定期間の数値的健全性を
確認してから行います。これは意味論的な正常性まで保証しないため、画像特徴とnative
state logを必ず併用して評価してください。
