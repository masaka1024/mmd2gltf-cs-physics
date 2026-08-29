# divhunt — 実機の発散をヘッドレスで再現する

Unity 上の往復をやめ、手元で何度でも再現・二分できるようにするための一式。

## 手順

1. **モーションを取り出す** (どちらか)
   - **A. Unity から吸い出す**: `MmdPhysicsBehaviour` の `DumpBonePoseCsv` を ON
     (`DumpFrames` / `DumpSkipFrames` で範囲指定)。物理が読むのと同じタイミング・
     同じ境界変換 (`BoneWorldOrNull`) を通るので**座標系の取り違えが起きない**。
   - **B. `.glb` のアニメーションから生成する**: `gen.py`。Unity 不要で長時間ぶん作れる。
     ★座標変換は **A で採った CSV を基準に校正する**(推測しない)。校正手順は下記。
2. **再現する**: `DivHunt.exe`
   ```
   MMD_TEST_GLB=<glb>  BONECSV=<csv>  [FRAMES=n] [ITERS=n] [SUBSTEPS=n]
   [VMAX=200] [WMAX=5000] [PMAX=500]      … 逸脱と見なす閾値
   [TELEPORT=3] [TELEFRAC=0.25]           … テレポート検出 (0で無効)
   [MASSCLAMP=x]                          … 動的剛体の質量に上限 (異常値の切り分け)
   [NOCONTACT=1]                          … 接触を止める (ジョイント/接触の二分)
   ```
   「最初に閾値を超えた frame と剛体」を名指しする。

## 座標変換の校正 (gen.py の定数)

`.glb` のノード階層からワールド姿勢を組み、次で engine 座標へ移す:

```
pos  = (x, y, -z) * 12.5 + offset      # 12.5 = 1/UnitScale(0.08)
quat = (-x, -y, z, w)                  # Z反転に伴う共役
t    = (Unity側CSVのframe - 1) / 60    # frame0 は「アニメ適用前のバインド姿勢」
```

★この符号と時刻対応は**総当たりで当てて、Unity 実測 CSV との RMS で選んだ**もの。
位置 RMS **0.012**、回転 平均 **0.32°** で一致する。`offset` はシーン上のモデル配置
(ルート Transform / UnitScale) 由来なので**シーンを動かしたら取り直すこと**。
取り直し方: 動いている区間の CSV フレームを1つ選び、符号8通り × 時刻を掃引して RMS 最小を採る。

## 注意

- 生成 CSV は **BoneFollow 剛体が参照するボーンだけ**あればよい (このモデルで20本)。
- `.glb` のアニメは 30fps キーの LINEAR 補間。60Hz で再サンプルしている。
