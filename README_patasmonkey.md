# Patasmonkey向け OpenVINS 運用メモ

この文書はPatasmonkeyのOAK-D S2を入力にしたOpenVINS検証のための補足です。
upstreamのOpenVINS一般説明は[ReadMe.md](ReadMe.md)を参照してください。

## 使用設定

Patasmonkey launchは`config/oakd_s2/estimator_config1.yaml`を使います。camera–IMU
校正は、接続したOAKのMX IDに対応するYAMLを`pm_config`から指定します。別個体の
校正値を流用すると、VIOのscale・姿勢・初期化安定性を損なうため避けてください。

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
2. 停止状態でOpenVINSを起動し、初期化に必要な車体運動を安全に与える。
3. bagの画像・IMU・`trackhist`・OAK診断・OpenVINS native logを同一時刻で比較する。
4. VIO速度/高さゲートの診断を確認し、隔離の契機を記録する。

VIOリセット後の自動再利用は、Patasmonkey側の速度ゲートが一定期間の数値的健全性を
確認してから行います。これは意味論的な正常性まで保証しないため、画像特徴とnative
state logを必ず併用して評価してください。
