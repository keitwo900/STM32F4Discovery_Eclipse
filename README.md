# STM32F4Discovery_Eclipse

## このリポジトリについて
STM32F4DiscoveryをEclipseとOpenOCDで開発するプロジェクトです。フリーの環境のみを使用して組み込み機器の開発ができます。</br>
<b>Eclipseのあちらこちらを調べてビルド、デバッグができるように設定する手間をかけず、
Eclipseと関連ツールを1度用意し終えたら、本リポジトリをPullして、そのままビルド、デバッグが可能状態になることを目的とします。</b></br>

develop_cdc_cdv Branchは、USB-CDCのデバイスクラスのサンプルアプリケーションコードです。
USB-CDC初期化後、受信処理が回り続け、データを受信すると、そのデータをエコーバックします。
USB-CDCのミドルウェア等は、CubeMxから作成したものです。


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
* GNU ARM Eclipse plug-ins v3.4.1.201704251808 https://github.com/eclipse-embed-cdt/eclipse-plugins/releases/tag/v3.4.1-201704251808 </br>
  ilg.gnuarmeclipse.repository-3.4.1-201704251808.zip をダウンロード、展開しておく(展開場所は任意)

※1 バージョンが異なるものを使用する場合は自己責任で。<br>
※2 当然ですがSTM32F4Discoveryの基板も要ります。

上記の各ファイルをダウンロード、展開した後、Cドライブ直下に以下のフォルダ構成でコピーします。
```
C:\
└─ eclipse
   ├─ BuildTools
   |  ├─ bin
   |  └─ gnu-mcu-eclipse
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

## クロスコンパイル用プラグインのインストール
<del>1. メニュー→ヘルプ→新規ソフトウェアのインストールをクリック</del>

<del>1. 「作業対象」に、「 http://gnuarmeclipse.sourceforge.net/updates 」と入力</del>

<del>1. 入力すると、下に「GNU ARM C/C++ Cross Development Tools」と出てくるので、選択して「次へ」または「完了」をクリックながらインストールを進める。</del> リンク先が消えたのでこの方法は使えなくなった。

1. メニュー→ヘルプ→新規ソフトウェアのインストールをクリック
1. 「作業対象」の右の「追加」をクリック
1. 「ローカル」をクリック
1. 先にダウンロードしたilg.gnuarmeclipse.repository-3.4.1-201704251808.zip を展開したフォルダを指定し、「追加」をクリック
1. 「GNU ARM C/C++ Cross Development Tools」と出てくるので、チェックボックスを選択して「次へ」または「完了」をクリックしながらインストールを進める。

## BuildTools、Toolchainパスの設定
1. メニュー→プロジェクト→プロパティーをクリック
1. C/C++ビルド→Tools Pathsを選択
1. Build tools folder:の欄に、BuildToolsのbinフォルダパスを指定(上記「用意するもの」項の通り環境を構築した場合は、「C:\eclipse\BuildTools\bin」)
1. Toolchain folder: の欄に、Toolchainのbinフォルダパスを指定(上記「用意するもの」項の通り環境を構築した場合は、「C:\eclipse\Toolchain\bin」)

尚、パス指定は、PCの環境変数から指定してもOKです。

上記作業まで行うと、ビルドできるようになると思います。

## ST-Linkドライバのインストール
ST社のページから、ST-Linkのドライバを入手してインストールしてください。20/06/15時点では、リンクは以下です。(ダウンロードにはST社サイトのアカウント登録が必要)<br>
https://www.st.com/ja/development-tools/stsw-link009.html

インストール後、STM32F4DISCOVERYのUSB MiniB端子とPCを接続し、LED LD1が点灯していることを確認してください。(点滅の場合は認識されていません)


## ビルド~デバッグ開始
1. Eclipseのメニューの「プロジェクト」→「すべてビルド」をクリック
1. STM32F4DISCOVERYのUSB-Mini B端子とPCをUSBケーブルでつなぐ(PCからマスストレージに見えるが気にしない。)
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
STMのHALドライバ(STM32F4DISCOVERY\lib\include\stm32f4-hal以下、およびSTM32F4DISCOVERY\lib\src\stm32f4-hal以下)は、
色々使えるように削除せずに残してありますが、Eclipseでは、フォルダ以下にあるソースファイルはすべてビルド対象になってしまいます。
必要に応じて削除して下さい。また、ドライバはプロジェクト作成時点のバージョンですので、必要に応じてCubeMX等で作成してアップデートしてください。

### Doxygen
STM32F4DISCOVERY\Doxyfileを使うと、DoxygenでHTMLを作ってソースコードブラウジングができるようになります。環境をお持ちの方はお試しください。<br>
<b>Dotを有効にしています。Graphviz等のDot言語の実行ファイルパス(C:\Program Files (x86)\Graphviz2.38\bin 等)をDOT_PATHに指定してください。インストールされていない場合は空欄にしてください。<br>
ここを設定しないとdoxygenがエラーで止まります。</b>
