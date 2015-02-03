# hsd2tms
Data Converter, from "Himawari Standard Data" to 256x256 png (Tile Map Service).

------
##目的
「ひまわり標準データ」から複数枚のメルカトル図法の縦横256ドットのPNGに変換し、主要な地図サービスから使えるようにするためのアプリケーションです。

##ビルド
###依存関係
libpngが必要です。

###コマンド例
例えば、

    git clone https://github.com/Torisugari/hsd2tms.git
    cd hsd2tms
    mkdir build
    cd build
    cmake ..
    make release
    make

のような順番でコマンドを実行するとビルドできます。

##使用方法

実行ファイルの名前はhsd2tmsで、引数にはひまわり標準データの書くセグメントファイル名のみをとります。拡張子は.DATです。気象観測センターからダウンロードできます。そのままだとbzip2で圧縮した上でzipに固めてあるので、全て解凍した上で、次のようなコマンドを実行してください。

    ./hsd2tms *.DAT

ダウンロード、解凍、PNGの生成までを一括で行いたい場合は、

     make test

を実行してください。ただし、生成されるPNGファイルはズームレベル7で300MBほどですが、元データはzipの状態で500MB、解凍するとDATの状態で1.5GBくらいあるので、"make test"を行う前にストレージの容量を確認してください。3GBでは少し足りないかもしれません。
