# -*- coding: utf-8 -*-
"""滑走ベイク用の静止ダミーVMD。physics_probe/make_static_vmd.py の書き出しを
   このモデルのボーン名 (全ての親) で使う。

なぜ要るか: PmxEditor の物理ベイクは「VMD の最終フレームまで」を焼くので、
  駆動が無いモデルでも **長さを決めるだけの空モーション** が要る。

使い方: python make_slide_vmd.py [出力.vmd] [フレーム数]
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "PmxEditor_0273", "physics_probe"))
sys.path.insert(0, r"C:\mytask2\PmxEditor_0273\physics_probe")
import make_static_vmd as M

out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "fric_slide_static.vmd")
frames = int(sys.argv[2]) if len(sys.argv) > 2 else 180
M.BONE = "全ての親"          # このモデルに実在するボーン
M.MODEL_NAME = "fric_L_t030"
n, kf = M.write_vmd(out, frames)
print("静止ダミーVMD: %s (%d バイト) / ボーン %s / キー %s / %d F = %.1f 秒 @30fps"
      % (out, n, M.BONE, kf, frames, frames / 30.0))
print("検証:", "OK" if M.verify(out, kf) else "★NG")
