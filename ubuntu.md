# Raspberry PiにUbuntuを入れる

最終的には`Raspbian`+`Docker`に落ち着いたが、メモとして残しておく。

## OSのインストール

- [Raspberry Pi OS](https://www.raspberrypi.com/software/)から`Raspberry Pi Imager`をダウンロードして`Ubuntu 20`のサーバをインストールする。
- LANのインタフェースが最初は使えない場合があるので、[ノートPC（LetsNote）を有線LANのルータ化する方法](https://attrise.blog/ceo/archives/10214)を参照し、有線LANケーブルでノートPCと接続して作業する。
- ただし、上記設定をするとモバイルホットスポットができなくなるので、後で必ず元に戻すこと。
- `Raspberry Pi`のIPは頻繁に変わるが、都度`Windows`側から`arp -a`で確認可能。
  - 管理者権限でコマンドターミナルを起動して、`arp -d`でキャッシュクリアしたほうが良い。

## ディスプレイ設定

[RaspberryPi3でHDMI出力の解像度を変更する方法](https://corgi-lab.com/raspberrypi/raspberrypi3-hdmi/)を参照し、`/boot/firmware/usercfg.txt`を編集して、`HDMI`の設定を行う。

```text
framebuffer_width=1024
framebuffer_height=600

hdmi_force_hotplug=1
disable_overscan=1

hdmi_group=2
hdmi_mode=87
hdmi_cvt=1024 600 60 5 0 0 0
```

## GUIのインストール

```shell
sudo apt install lxde
# ディスプレイマネージャは lightdm を選択。
```

## ネットワークの設定

- [【Ubuntu 20.04.3 LTS】ターミナルでwifiを設定する方法 netplan “aliases are not supported”エラーへの対処法](https://itc.tokyo/linux/settings-wifi-on-terminal/)
- [Raspberry Pi 3 + Ubuntu 21.10 で Wi-Fi を有効にする](https://zenn.dev/superbrothers/scraps/cfdf74a844272b)

`netplan`を設定する。おそらく、非表示のSSIDには接続できない。  
とりあえずは`Windows`モバイルホットスポットを有効にして、そこにつなぐ。

## Swap の作成

- [Raspberry PiのUbuntuにスワップ領域を設定する](https://qiita.com/zrock/items/71e3874cb83ed12ec405)

```shell
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab
```

## GUIのON/OFF

GUIをオフにする。

```shell
sudo systemctl set-default multi-user.target  
```

戻す場合は以下。

```shell
sudo systemctl set-default graphical.target  
```

## 参考文献

[ラズパイ3にUbuntu 20.04LTSをインストール](https://takaken.tokyo/dev/raspberrypi/raspi_ubuntu_setup2004/)
[Raspberry PiでのUbuntu Serverセットアップ手順](https://zenn.dev/uchidaryo/articles/setup-raspi-ubuntu-server)
