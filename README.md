# STM32F4Discovery_Eclipse

## このリポジトリについて
STM32F4DiscoveryをEclipseとOpenOCDで開発するプロジェクトです。フリーの環境のみを使用して組み込み機器の開発ができます。</br>
<b>Eclipseのあちらこちらを調べてビルド、デバッグができるように設定する手間をかけず、
Eclipseと関連ツールを1度用意し終えたら、本リポジトリをPullして、そのままビルド、デバッグが可能状態になることを目的とします。</b></br>

Mster Branchは、必要最低限のアプリケーション動作のままにしておきます。Main関数に到達したら、コンソールに"Hello ARM World!"と出力し、
以降はLED LD4が1秒周期、Duty75%で点滅するプログラムです。
(これは、Eclipseのプロジェクト新規作成で、STM32F4xx C/C++ Projectを選択して作成すると自動生成されるサンプルプログラムを少し改造したものです。)
STM32F4Discoveryで新たに何か開発したければ、Master Branchを起点にすればよいという算段です。

STM32F4Discoveryを使ったさまざまなアプリケーションは、他ブランチで作成していこうと思っています。(思っているだけかもよ?)

<b>このリポジトリをそのまま使用可能にするためには、Windows 10 64bit環境で、以下の手順で開発環境を整える必要があります。</b>

## 用意するもの<br>
* Eclipse IDE for C/C++ Developers(Pleiades) Version: 2019-09 R (4.13.0) Build id: 20190917-1200
https://mergedoc.osdn.jp/
* Build Tool gnu-mcu-eclipse-windows-build-tools-2.12-20190422-1053-win64
https://github.com/gnu-mcu-eclipse/windows-build-tools/releases/v2.12-20190422/
* GNU Arm Embedded Toolchain gcc-arm-none-eabi-5_4-2016q3-20160926-win32
https://launchpad.net/gcc-arm-embedded/+download
* OpenOCD 0.10.0-12.1
https://github.com/ilg-archived/openocd/releases

※1 バージョンが異なるものを使用する場合は自己責任で。<br>
※2 当然ですがSTM32F4Discoveryの基板も要ります。

上記の各ファイルをダウンロード、展開した後、Cドライブ直下に以下のフォルダ構成でコピーします。
```
C:\
└─ eclipse
   ├─ BuildTools
   |  ├─ bin
   |  └─ gnu-ncu-eclipse
   |
   ├─ OpenOCD
   |  ├─ bin
   |  ├─ contrib
   |  ├─ doc
   |  ├─ gnu-mcu-eclipse
   |  ├─ OpenULINK
   |  └─ scripts
   |
   ├─ pleiades
   |  └─ eclipse
   |
   └─ Toolchain
      ├─ arm-non-eabi
      ├─ bin
      ├─ lib
      └─ share
```

## リポジトリをCloneまたはダウンロード~プロジェクトの取り込み
1. 本リポジトリをCloneまたはダウンロード
1. Eclipseを起動し、ワークスペースディレクトリに、Cloneまたはダウンロードしたフォルダの直下を指定して起動
1. メニュー→ファイル→ファイル・システムからプロジェクトを開くをクリック
1. 「ディレクトリー」ボタンをクリック
1. .projectファイルのあるディレクトリを選択。正しく選択されていた場合、「フォルダー」の欄にプロジェクト名が表示される
1. 「完了」をクリック
1. 「ようこそ」のタブが出ている場合は「×」で消す

3~7は、Cloneして初めに一回行えば、次からは起動時にプロジェクトが読み込まれています。

## ビルド~デバッグ開始
1. Eclipseのメニューの「プロジェクト」→「すべてビルド」をクリック
1. STM32F4DISCOVERYのUSB-Mini B端子とPCをUSBケーブルでつなぐ(Windows 10なら、ドライバは不要なはず。PCからマスストレージに見えるが気にしない。)
1. Eclipseのメニューの「実行」→「デバッグの構成」をクリック
1. 左のGDB OpenOCD Debuggingのところだけプルダウンできるはずなので(「>」がある)、これをクリック
1. 「>」の下に「STM32F4DISCOVERY Debug」があるので選択
1. 右下の「デバッグ」が有効になるのでクリック
1. デバッグが始まったら、Eclipseのメニューの「実行」→「再開」をクリック(または、ツールバー内にも再開ボタンがあるのでそちらでもよい)
成功すればLEDが点滅しだします。

## ライセンス
ライセンスは、各ソースコード内に記載されたライセンス条文に従います。

## その他

### HALドライバ
STMのHALドライバ(STM32F4DISCOVERY\system\include\stm32f4-hal以下、およびSTM32F4DISCOVERY\system\src\stm32f4-hal以下)は、
色々使えるように削除せずに残してありますが、Eclipseでは、フォルダ以下にあるソースファイルはすべてビルド対象になってしまいます。
必要に応じて削除して下さい。また、ドライバはプロジェクト作成時点のバージョンですので、必要に応じてCubeMX等で作成してアップデートしてください。
