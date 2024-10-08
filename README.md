MZ-2000/80B Emulator for Raspberry Pi Pico
---
![screenshot](/pictures/screenshot00.jpg)
SHARP MZ-2000/80B のエミュレータです。
以下の機能を実装しています。

- メイン RAM(64KB)
- VRAM
- テープ
- Beep

速度は調整していませんが、単純にCPU の速度は実機より数割増し、描画は少し遅めなのでほぼトントンと思われます。
(Pico2 でもビルドできるようにしましたが速すぎると思います)

MZ-2000(カラーモニタ),MZ-2000(グリーンモニタ),MZ-80B(グリーンモニタ)の切り替えができます。
2000 と 80B を切り替えたときには IPL Reset をかけてください。
なお、80B モードは充分にテストされていませんので、注意してください。

---
# 配線など

BML3 や FM-7 エミュレータと同じです

- GPIO0 VGA:H-SYNC
- GPIO1 VGA:V-SYNC
- GPIO2 VGA:Blue
- GPIO3 VGA:Red
- GPIO4 VGA:Green
- GPIO6 Audio

VGA の RGB 信号には 220~330ohm 程度の抵抗を直列に入れます。
VGA、Audio の　GND に Pico の　GND を接続してください。

---
# キーボード

Pico の USB 端子に、OTG ケーブルなどを介して USB キーボードを接続します。
USB キーボードに存在しないキーは以下のように割り当てています。

- カナ → カタカナ・ひらがな
- GRAPH → ALT
- INS/DEL → INS,DEL
- HOME/CLR → Home

また F12 でメニュー画面に移ります。
MZT イメージの操作ができます。

---
# テープ

UART 入出力に対応していません。
LittleFS 上の MZT 形式のファイルをロード・セーブに用います。

---
# ROM など

いつものように純正ROM が必要です。
`mzrom_dummy.h` を `mzrom.h` にコピーしたのち、
`mzipl` `mzipl0` `mzfont` にそれぞれ、IPL(MZ-2000)、IPL(MZ-80B)、フォントのデータを入れてください。

---
# 制限事項

- 最初に起動する際に LittleFS のフォーマットで固まることがあります。(リセットでOK)
- キーボード割り込みが未実装です(使っているソフトって Hu-Basic くらい？)

---
# ライセンスなど

このエミュレータは以下のライブラリを使用しています。

- [Z80](https://github.com/redcode/Z80/tree/master)
- [Zeta](https://github.com/redcode/Zeta)
- [VGA ライブラリ(一部改変)](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics)
- [LittleFS](https://github.com/littlefs-project/littlefs)

---
# Gallary

![Hat color](/pictures/screenshot01.jpg)
![Hat Green](/pictures/screenshot02.jpg)