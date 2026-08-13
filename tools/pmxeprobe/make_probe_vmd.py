# -*- coding: utf-8 -*-
# E9_multi の各島の親ボーン(キネマティック)を揺さぶる VMD を作る。
#
# 狙い: [Jointロック内部演算] は静的平衡では一切効果が出なかった(11構成で完全一致)。
#       readme.txt:1051 の「ズレが気になる場合は」という言い回しから、動いているときにだけ
#       意味を持つ補正の可能性がある。そこで親を動かしてから長く静止させ、
#       「揺さぶった結果どこに落ち着くか」を比べる。
#
# 設計:
#   PmxEditor のモーション再生は任意フレームへ移動できず、常にループ再生される
#   (readme.txt:3731)。よって「動いている最中を狙って保存」は不可能。
#   代わりに 揺さぶり(0-60F) → 長い静止(60-600F) の構成にし、静止区間で保存する。
#   ループしても静止区間が18秒あるので狙いやすい。
#
# VMD 仕様: ヘッダ30byte + モデル名20byte + ボーンキー数u32 +
#           (名前15byte, フレームu32, 位置3f, 回転4f(x,y,z,w), 補間64byte) * n +
#           モーフ/カメラ/照明/セルフ影/IK表示 の各カウント(いずれも0)

import math
import os
import struct

# VMD 標準の補間パラメータ(線形相当)
DEFAULT_INTERP = bytes([
    20, 20, 0, 0, 20, 20, 20, 20, 107, 107, 107, 107, 107, 107, 107, 107,
    20, 20, 0, 0, 20, 20, 20, 20, 107, 107, 107, 107, 107, 107, 107, 107,
    20, 0, 0, 20, 20, 20, 20, 107, 107, 107, 107, 107, 107, 107, 107, 0,
    0, 0, 20, 20, 20, 20, 107, 107, 107, 107, 107, 107, 107, 107, 0, 0,
])


def quat_x(deg):
    """X 軸まわりの回転クォータニオン (x, y, z, w)"""
    h = math.radians(deg) * 0.5
    return (math.sin(h), 0.0, 0.0, math.cos(h))


def sjis15(name):
    b = name.encode("shift_jis")
    if len(b) > 15:
        raise ValueError(f"ボーン名が VMD の15バイト制限を超える: {name} ({len(b)})")
    return b + b"\x00" * (15 - len(b))


def write_vmd(path, model_name, keys):
    """keys: [(bone_name, frame, (px,py,pz), (qx,qy,qz,qw)), ...]"""
    out = bytearray()
    out += b"Vocaloid Motion Data 0002" + b"\x00" * 5      # 30 byte
    mn = model_name.encode("shift_jis")[:20]
    out += mn + b"\x00" * (20 - len(mn))                    # 20 byte

    out += struct.pack("<I", len(keys))
    for (name, frame, pos, quat) in keys:
        out += sjis15(name)
        out += struct.pack("<I", frame)
        out += struct.pack("<3f", *pos)
        out += struct.pack("<4f", *quat)
        out += DEFAULT_INTERP

    for _ in range(5):   # モーフ/カメラ/照明/セルフ影/IK表示
        out += struct.pack("<I", 0)

    with open(path, "wb") as fp:
        fp.write(bytes(out))
    return path


# E9_multi の島タグ (make_probe_pmx.build_multi と一致させること)
ISLAND_TAGS = ["base", "offMid", "offChild", "len1", "len4", "lim30", "lim0",
               "chain2", "chain3", "chain2L", "linFree", "mode2"]

SWING_DEG = 30.0
HOLD_END = 600      # ループ長。30fps で 20 秒


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(here, "models")
    os.makedirs(outdir, exist_ok=True)

    # (frame, 角度) 揺さぶってから静止
    timeline = [(0, 0.0), (20, SWING_DEG), (40, -SWING_DEG), (60, 0.0), (HOLD_END, 0.0)]

    keys = []
    for tag in ISLAND_TAGS:
        bone = f"{tag}_root"
        for (f, d) in timeline:
            keys.append((bone, f, (0.0, 0.0, 0.0), quat_x(d)))

    p = write_vmd(os.path.join(outdir, "E9_shake.vmd"), "E9_multi", keys)
    print(f"{os.path.getsize(p):6d}  {os.path.basename(p)}")
    print(f"  ボーン {len(ISLAND_TAGS)} 本 × キー {len(timeline)} = {len(keys)} キー")
    print(f"  0-60F で ±{SWING_DEG:g}° 揺さぶり、60-{HOLD_END}F は静止（30fps で 18 秒）")


if __name__ == "__main__":
    main()
