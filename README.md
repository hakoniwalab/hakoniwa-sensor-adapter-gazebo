# hakoniwa-sensor-adapter-gazebo

Gazebo の [gz-sensors](https://github.com/gazebosim/gz-sensors) を、[hakoniwa-sensor-adapter](https://github.com/hakoniwalab/hakoniwa-sensor-adapter) の口（`IImuSensorAdapter`）の裏に包むアダプタです。
物理の計算は使いません。ホストが渡す真値に、gz-sensors の誤差のモデル（ノイズ、ずれ、ゆっくり変わるずれ、分解能、測る周期）を当てはめます。

対応しているセンサは、箱庭ドローン PRO の 5 つのセンサです。

| 口 | gz-sensors | 作る関数 |
|---|---|---|
| `IImuSensorAdapter`（ジャイロと加速度） | `ImuSensor` | `create_gz_imu_sensor_adapter` |
| `IMagSensorAdapter`（地磁気） | `MagnetometerSensor` | `create_gz_mag_sensor_adapter` |
| `IBaroSensorAdapter`（気圧） | `AirPressureSensor` | `create_gz_baro_sensor_adapter` |
| `IGpsSensorAdapter`（GPS） | `NavSatSensor` | `create_gz_gps_sensor_adapter` |

## 使い方

```cpp
#include "hakoniwa/sensor_adapter/gazebo/gz_imu_sensor_adapter.hpp"

hakoniwa::sensor_adapter::ImuAdapterSetup setup{};
setup.config_path = "config/imu-noise.sdf";
setup.physics_step_ns = 1000000;  // 1 ms
std::string error;
auto imu = hakoniwa::sensor_adapter::gazebo::create_gz_imu_sensor_adapter(setup, error);
```

- 設定は SDFormat のファイルです。最初の `<model>` のリンクにある、その種類の最初の `<sensor>`（`imu`、`magnetometer`、`air_pressure`、`navsat`）を使います。例は `config/` にあります。
- `<pose>` は 0 にします。取り付け位置姿勢はホストが当てはめます。0 でなければ、作るときにエラーにします。
- 測る周期は `<update_rate>`（Hz）です。時刻 0 から、その周期の境目で測ります。

## 座標

口の座標は箱庭の決まりです。世界は NWU（x 北、y 西、z 上）、機体とセンサは FLU（x 前、y 左、z 上）です。
Gazebo の世界は ENU です。IMU と地磁気の計算（世界の量をセンサの軸に回す）は世界の軸の向きによらず、センサの軸は同じ FLU なので、変換はしません。GPS の速度だけは、`NavSatSensor` が ENU で持つので、NWU と ENU の間で直します。

## gz-sensors との対応

### IMU

| 口 | gz-sensors の `ImuSensor` |
|---|---|
| 角速度 | `SetAngularVelocity` |
| 比力 f | 重力を含まない加速度 a = f + R·g にして `SetLinearAcceleration`。gz-sensors が中で R·g を引くので、f に戻る |
| 世界からセンサへの回転、世界の重力 | `SetWorldPose`、`SetGravity` |
| 測った値 | `AngularVelocity()`、`LinearAcceleration()` |

### 地磁気

| 口 | gz-sensors の `MagnetometerSensor` |
|---|---|
| 世界の磁場（nT、NWU）、世界からセンサへの回転 | テスラに直して `SetWorldMagneticField`、`SetWorldPose`。gz-sensors がセンサの軸へ回す |
| 測った値（nT） | `MagneticField()` をナノテスラに直す |

SDF のノイズの値（`<magnetometer>` の `<x>` `<y>` `<z>`）は**テスラ**で書きます（10 nT なら `1e-8`）。

### 気圧

| 口 | gz-sensors の `AirPressureSensor` |
|---|---|
| 高さ（海面から、m） | 基準の高さを 0 にし、`SetPose` の z に入れる |
| 海面の天気 | 使わない（海面 101325 Pa、288.15 K の標準大気に固定） |
| 測った気圧（Pa） | `AirPressureSensor` は気圧を読む関数を持たず、値を gz-transport のメッセージでだけ出す。アダプタは自分の配信を購読し、測った時刻のメッセージが届くのを待って値にする |

- 大気の式は、対流圏の標準大気（ジオポテンシャル高度、気温減率 0.0065 K/m）です。
- 気温と気圧高度は出しません（0 を返し、能力の表で宣言します）。
- gz-transport は、同じプロセスの購読者にも自分のスレッドから届けます。そのため、測るたびにスレッドを 1 回またぎます。

### GPS

| 口 | gz-sensors の `NavSatSensor` |
|---|---|
| アンテナの緯度・経度・高度 | `SetPosition` |
| 速度（NWU） | ENU（東・北・上）に直して `SetVelocity`。測った値は NWU に戻す |
| 測った位置と速度 | `Latitude()`、`Longitude()`、`Altitude()`、`Velocity()` |
| 測位の種類 | いつも 3（3D）。`NavSatSensor` は測位を失う受信機を表さない |
| 対地速度、進行方向、精度の推定、衛星の数 | 出さない（0 を返し、能力の表で宣言する） |

SDF のノイズの値は、**水平の位置が度**（緯度・経度にそのまま足す。1e-5 度で緯度方向に約 1.1 m）、垂直の位置がメートル、速度が m/s です。

### すべてのセンサに共通

| 口 | gz-sensors |
|---|---|
| 1 刻み進める | `Sensor::Update(時刻, false)`。測ったときだけ true を返す |
| 測らなかった刻み | アダプタが前に測った値を返す（gz-sensors の getter は、最後に渡した入力やノイズを足す前の値を返すことがあるため） |
| リセット | gz-sensors にリセットがないため、センサを作り直す |
| 乱数の種 | `gz::math::Rand::Seed`（プロセス全体で 1 つ） |
| 配信の口 | インスタンスごとに別の topic にする（同じプロセスの中で混ざらないように） |

能力の表で「ない」と宣言するもの：

- どのセンサにもない：センサの値の遅れ、頭打ち、ログの列（gz-sensors が、使っているずれの値を外に出さないため）、インスタンスごとの乱数
- IMU：IMU のフィルタ、ジャイロと加速度で別々の周期、較正の値
- 地磁気：硬鉄・軟鉄
- 気圧：海面の天気、気温、気圧高度
- GPS：進行方向、受信機の状態（精度・衛星の数）

## ビルド（pixi、macOS・Windows・Linux）

依存（gz-sensors 10.2.0 とそのライブラリ、cmake、コンパイラ）は、conda-forge から [pixi](https://pixi.sh/) で入れます。
入る場所は、このリポジトリの中の `.pixi/` だけです。システムのパッケージ（Homebrew、apt）には触りません。

hakoniwa-sensor-adapter をこのリポジトリと同じ階層に置きます。2 つのリポジトリの親のディレクトリで、上から実行します。

```bash
git clone https://github.com/hakoniwalab/hakoniwa-sensor-adapter.git
git clone https://github.com/hakoniwalab/hakoniwa-sensor-adapter-gazebo.git
cd hakoniwa-sensor-adapter-gazebo
pixi run test
```

`pixi run test` は、依存を入れ、構成し、ビルドし、試験を流します。Windows では、Visual Studio 2022 の C++ ビルドツールが要ります。

- 環境の大きさは約 1.8 GB です（macOS arm64、245 パッケージ）。
- macOS では、ビルドした実行ファイルを初めて開くときだけ、十数秒待つことがあります。
- 乱数の並びは OS（C++ の標準ライブラリ）によって違います。同じ種で同じ値になるのは、同じ OS・同じ環境の中だけです。

## ビルド（Docker、Ubuntu 24.04 と Gazebo Jetty の apt）

hakoniwa-sensor-adapter をこのリポジトリと同じ階層に置きます。2 つのリポジトリの親のディレクトリで、上から実行します。

```bash
git clone https://github.com/hakoniwalab/hakoniwa-sensor-adapter.git
git clone https://github.com/hakoniwalab/hakoniwa-sensor-adapter-gazebo.git
docker build -t hako-gz-sensors:jetty hakoniwa-sensor-adapter-gazebo/docker
docker run --rm -v "$PWD:/work" hako-gz-sensors:jetty bash -c \
  "cmake -S hakoniwa-sensor-adapter-gazebo -B /tmp/build -DHAKONIWA_SENSOR_ADAPTER_GAZEBO_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release && cmake --build /tmp/build -j && ctest --test-dir /tmp/build --output-on-failure"
```

### 依存

gz-sensors 10.2.0（Ubuntu 24.04、arm64）で測った値です。

| 何を | 結果 |
|---|---|
| apt で入るパッケージ | 562 個（gz-sensors の開発パッケージと、その依存） |
| 実行に要るライブラリ | 37 個、約 29 MB（libc と libstdc++ を除く。IMU の試験で測定） |
| 実行に要らないもの | gz-rendering、OGRE などの描画のライブラリ |

実行に要る主なライブラリは次のとおりです。

- gz-sensors と IMU のコンポーネント、gz-common、gz-math、gz-msgs、gz-transport、gz-utils、sdformat
- protobuf、ZeroMQ、Zenoh、tinyxml2、urdfdom、spdlog、fmt

ビルドのときは、次の 2 つを足します（`docker/Dockerfile` に入っています）。

- `libgz-common7-profiler-dev`：gz-sensors の CMake の設定が要求しますが、gz-sensors の開発パッケージは連れてきません。
- `libgl-dev`：gz-sensors の CMake の設定は、IMU だけを使う場合でも gz-rendering を探し、gz-rendering が OpenGL の開発ファイルを要求します。

### 通信

gz-sensors は、センサを読み込むときに gz-transport の配信の口を作ります。このとき、プロセスに受信と配信のスレッドが立ち、マルチキャストで相手を探す処理が始まります。
同じ PC のほかの Gazebo と混ざらないようにするには、環境変数 `GZ_PARTITION` を分けます。

## ライセンス

MIT。gz-sensors とその依存は、それぞれのライセンス（gz-sensors は Apache-2.0）に従います。
