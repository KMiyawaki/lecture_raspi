# lecture_raspi

`Raspberry Pi 4 or 3`に`Docker`をインストールし、`ROS`を動作させる。

## Raspbian OS

- [Raspberry Pi OS](https://www.raspberrypi.com/software/)から`Raspberry Pi Imager`をダウンロードして`Raspbian`をインストールする。

## Dockerのインストール

```shell
sudo apt update && sudo apt upgrade -y
sudo pip3 install --upgrade pip
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker ${USER}
sudo pip3 install docker-compose
```

`docker-compose`インストール時にエラーが発生した場合は以下を実行する。

```shell
sudo apt purge python3-openssl -y
sudo pip3 uninstall pyopenssl
sudo pip3 install pyopenssl
sudo pip3 install rust
sudo pip3 install --upgrade requests
sudo pip3 install docker-compose
```

`Emacs`テキストエディタもインストールしておく。

```shell
sudo apt install emacs
```

## 不要なDockerコンテナを必要に応じて破棄する

基本的には環境構築時に一度だけ実行すればよい。

- 停止しているコンテナを全て破棄する場合

```shell
docker system prune
WARNING! This will remove:
  - all stopped containers
  - all networks not used by at least one container
  - all dangling images
  - all dangling build cache

Are you sure you want to continue? [y/N] y # yを入力してエンター
```

- コンテナを選択して破棄する場合

```shell
docker ps -a --format "{{.ID}} {{.Names}}"
bcc31298065b ros1_noetic # 先頭の半角英数字文字列がコンテナID
docke rm bcc31298065b # 指定したIDのコンテナが破棄される
```

## （補足）不要なDockerイメージを破棄したい場合

```shell
docker images
REPOSITORY                        TAG       IMAGE ID       CREATED          SIZE
ros                               noetic    c1ca36166d90   2 months ago     932MB
# 削除したいIDを指定して以下のコマンドを実行する
docker rmi c1ca36166d90
Untagged: ros:noetic
```

## 古いROSのワークスペースを削除する

環境構築時に一度だけ実行すればよい。

```shell
cd
sudo rm -fr catkin_ws
```

## 仮想環境の構築（Dockerコンテナの作成）

ホスト側で以下を実行して`Docker`コンテナを作成する。

```shell
cd 
rm -fr docker_ros_noetic
git clone https://github.com/KMiyawaki/docker_ros_noetic.git
cd docker_ros_noetic
./attach.sh
ros1_noetic is not running. Try to start.
Making /home/pi/catkin_ws/src
kmiyawaki20/ros1_noetic_user_pi does not exists. 
Input password for user 'pi' > # Linuxのパスワードを入力してエンターを押す
'XXXXXX' Is that a correct password ?(Y/N) y # 正しければyを押す。間違っていればnを押して終了させる。
OK. Start to build image.
#1 [internal] load build definition from Dockerfile.useradd1
Creating ros1_noetic ... done
To run a command as administrator (user "root"), use "sudo <command>".
See "man sudo_root" for details.

pi@raspberrypi:~$ # この時点でコンテナ（仮想環境に入っている）
pi@raspberrypi:~$ exit # このコマンドで仮想環境から抜ける
exit 
pi@raspberrypi:~/docker_ros_noetic $ 
```

以降は以下を実行して、コンテナ内で作業する。

```shell
cd ~/docker_ros_noetic
./attach.sh
```

### コンテナの初期設定

次のコマンドでGUIをテストする。以下いずれのコマンドも`./attach.sh`後の**コンテナ内で**実行すること。

```shell
xeyes
```

次のように、目玉のマークが表示されれば成功。`Ctrl＋C`キーで終了させる。

![2023-04-27_192138.png](./images/2023-04-27_192138.png)

`ROS`のワークスペースを設定し、必要なパッケージをインストールする。
以下は、`./attach.sh`実行後の**コンテナ内**での作業であることに注意。

```shell
cd
cd setup_robot_programming
./init_workspace.sh
**Making workspace. Target ros-noetic**
Creating symlink "/home/pi/catkin_ws/src/CMakeLists.txt" pointing to "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake"
# ・・・省略・・・
./make_beginner_tutorials.sh
Created file beginner_tutorials/package.xml
Created file beginner_tutorials/CMakeLists.txt
Created folder beginner_tutorials/include/beginner_tutorials
# ・・・省略・・・
./upgrade_packages.sh
Get:1 http://ports.ubuntu.com/ubuntu-ports focal InRelease [265 kB]  
# ・・・省略・・・
exit # 一旦コンテナを抜ける。他にもコンテナに入っているターミナルがあれば全てexitする。
```

`roscore`を起動する。`./attach.sh`したコンテナ内で実行すること。

```shell
roscore
... logging to /root/.ros/log/ce235914-e4e5-11ed-8e14-e45f013e9eb3/roslaunch-raspberrypi-9023.log
# 省略・・・
started core service [/rosout]
```

もう一つ別のターミナルで`./attach.sh`し、`RViz`を起動する。

```shell
rviz
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
# 省略・・・
```

![2023-04-27_192900.png](./images/2023-04-27_192900.png)

### YDLidar X4を動かす

**まず、ホスト（Raspberry Pi）側**で作業する。

```shell
# ホスト側
cd 
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK/startup
sudo sh initenv.sh
ls /dev|grep yd
ydlidar # これが出力されればOK.
```

次に**コンテナに**`./attach.sh`して以下を実行する。

```shell
cd ~/catkin_ws/src
git clone https://github.com/KMiyawaki/oit_navigation_minibot_light_01.git
cd oit_navigation_minibot_light_01
./install.sh
ls /dev|grep yd
ydlidar # コンテナ側でもこれが出力されればOK.
roslaunch ydlidar_ros_driver X4.launch
# YDLidarが回転する
```

新しい端末で`attach.sh`し、コンテナ内で`RViz`を起動する。

```shell
rviz -f laser_frame
```

`Add`->`By topic`->`/scan`を選択して追加すると`YDLidar`のデータが表示される。

![2023-04-27_194549.png](./images/2023-04-27_194549.png)

### docker-compose.ymlの補足

`docker-compose`を使わない場合のコンテナの作成は以下で行える。

```shell
docker run -it --privileged ros:noetic
# または
docker run -it --security-opt seccomp:unconfined ros:noetic
```

オプションをつけない場合、以下のエラーが出る。

```shell
Fatal Python error: pyinit_main: can't initialize time
Python runtime state: core initialized
PermissionError: [Errno 1] Operation not permitted

Current thread 0xb6feb460 (most recent call first):
<no Python frame>
```

- 参考[Can you install Noetic with Docker on a Raspberry Pi?](https://answers.ros.org/question/359069/can-you-install-noetic-with-docker-on-a-raspberry-pi/)

## コンテナ内で追加インストール

- RVizを起動したときのエラー`[rospack] Error: no such package media_export`に対し、`sudo apt install ros-noetic-media-export`を行う。

## reTerminal固有の事項

`apt upgrade`後にモニタが映らなくなることがある。

- [reTerminalを使ってみる！](https://pocketgriffon.hatenablog.com/entry/2022/03/13/010200)
- [reTerminal E10-1 instillation failed](https://forum.seeedstudio.com/t/reterminal-e10-1-instillation-failed/270468/3)
- [reTerminalのCompute Module 4を入れ替える](https://lab.seeed.co.jp/entry/2021/07/13/120000)の「ドライバーをインストールする」

```shell
echo arm_64bit=0 | sudo tee -a /boot/config.txt # 一回だけ
git clone --depth 1 https://github.com/Seeed-Studio/seeed-linux-dtoverlays
cd seeed-linux-dtoverlays
sudo ./scripts/reTerminal.sh
sudo reboot
```

## 参考文献

1. [Raspberry Pi 4でDockerのインストール方法 2023年度版](https://raspida.com/rpi4-docker-install)
2. [Dockerを用いてRaspberryPi3上でROSを使ってみる](https://qiita.com/Spritaro/items/92e504023c2653595e79)
3. [Docker コンテナ内のGUIアプリを起動してホスト側に表示する](https://zukucode.com/2019/07/docker-gui-show.html)
