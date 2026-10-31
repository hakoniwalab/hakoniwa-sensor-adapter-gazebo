# hakoniwa-sensor-adapter-gazebo

Gazebo の [gz-sensors](https://github.com/gazebosim/gz-sensors) を、[hakoniwa-sensor-adapter](https://github.com/hakoniwalab/hakoniwa-sensor-adapter) の口（`IImuSensorAdapter`）の裏に包むアダプタです。
物理の計算は使いません。ホストが渡す真値に、gz-sensors の誤差のモデル（ノイズ、ずれ、ゆっくり変わるずれ、分解能、測る周期）を当てはめます。

対応しているセンサは IMU（ジャイロと加速度）です。

## 使い方

```cpp
#include "hakoniwa/sensor_adapter/gazebo/gz_imu_sensor_adapter.hpp"

hakoniwa::sensor_adapter::ImuAdapterSetup setup{};
setup.config_path = "config/imu-noise.sdf";
setup.physics_step_ns = 1000000;  // 1 ms
std::string error;
auto imu = hakoniwa::sensor_adapter::gazebo::create_gz_imu_sensor_adapter(setup, error);
```

- 設定は SDFormat のファイルです。最初の `<model>` のリンクにある、最初の `<sensor type="imu">` を使います。例は `config/` にあります。
- `<pose>` は 0 にします。取り付け位置姿勢はホストが当てはめます。0 でなければ、作るときにエラーにします。
- 測る周期は `<update_rate>`（Hz）です。時刻 0 から、その周期の境目で測ります。

## 座標

口の座標は箱庭の決まりです。世界は NWU（x 北、y 西、z 上）、機体とセンサは FLU（x 前、y 左、z 上）です。
Gazebo の世界は ENU ですが、IMU の計算（重力をセンサの軸に回して引く）は世界の軸の向きによらず、センサの軸は同じ FLU なので、変換はしません。

## gz-sensors との対応

| 口 | gz-sensors の `ImuSensor` |
|---|---|
| 角速度 | `SetAngularVelocity` |
| 比力 f | 重力を含まない加速度 a = f + R·g にして `SetLinearAcceleration`。gz-sensors が中で R·g を引くので、f に戻る |
| 世界からセンサへの回転、世界の重力 | `SetWorldPose`、`SetGravity` |
| 1 刻み進める | `Sensor::Update(時刻, false)`。測ったときだけ true を返す |
| 測った値 | `AngularVelocity()`、`LinearAcceleration()`。測らなかった刻みは、アダプタが前に測った値を返す |
| リセット | gz-sensors にリセットがないため、センサを作り直す |
| 乱数の種 | `gz::math::Rand::Seed`（プロセス全体で 1 つ） |

能力の表では、次を「ない」と宣言します。

- センサの値の遅れ、IMU のフィルタ、頭打ち
- ジャイロと加速度で別々の周期
- 較正の値、ログの列（gz-sensors が、使っているずれの値を外に出さないため）
- インスタンスごとの乱数

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
| 実行に要るライブラリ | 37 個、約 29 MB（libc と libstdc++ を除く） |
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
