# -*- coding: utf-8 -*-
"""ベイクした VMD から box の滑走曲線を取り出し、当エンジン / 純Bullet と並べる。

なぜ VMD を直接読むか: PmxEditor の物理ベイクは **物理で動いたボーンの移動キー**を
書き出す。box ボーンの移動キーがそのまま「MMD 側の滑走曲線」になるので、
CSV への変換工程を挟まずに済む。

使い方:
    python compare_slide.py <ベイク済み.vmd> [engineOutDir] [bulletOutDir]
      既定の比較先は traj_engine_fm1.csv / bul_net_fric_L_tan030
"""
import csv, io, os, struct, sys, math

def read_vmd(path):
    """bone -> {frame: (x,y,z)}  移動キーだけ拾う"""
    d = open(path, "rb").read()
    o = 30 + 20                      # シグネチャ30 + モデル名20
    n = struct.unpack("<I", d[o:o+4])[0]; o += 4
    out = {}
    for _ in range(n):
        raw = d[o:o+15]; o += 15
        name = raw.split(b"\x00")[0].decode("cp932", "replace")
        f = struct.unpack("<I", d[o:o+4])[0]; o += 4
        x, y, z = struct.unpack("<3f", d[o:o+12]); o += 12
        o += 16 + 64                 # 回転 + 補間
        out.setdefault(name, {})[f] = (x, y, z)
    return out

def engine_curve(p):
    """FricProbe の TRAJ_OUT を読む。★NetDump は使えない (ジョイント0本だと箱を落とす)。"""
    if not os.path.exists(p): return None
    out = {}
    for r in csv.DictReader(io.open(p, encoding="utf-8")):
        if r["name"] != "box": continue
        out[int(r["frame"])] = (float(r["px"]), float(r["py"]), float(r["pz"]))
    return out

def bullet_curve(d):
    p = os.path.join(d, "net_bullet_state.csv")
    if not os.path.exists(p): return None
    out = {}
    for r in csv.DictReader(io.open(p, encoding="utf-8")):
        if r["sub"] != "-1" or r["name"] != "box": continue
        out[int(r["frame"])] = (float(r["px"]), float(r["py"]), float(r["pz"]))
    return out

def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    vmd = sys.argv[1]
    edir = sys.argv[2] if len(sys.argv) > 2 else "traj_engine_fm1.csv"
    bdir = sys.argv[3] if len(sys.argv) > 3 else "bul_net_fric_L_tan030"
    keys = read_vmd(vmd)
    box = None
    for k in keys:
        if "box" in k.lower(): box = keys[k]
    if box is None:
        print("★box ボーンの移動キーが無い。物理ベイクが焼かれていない可能性がある。")
        print("   VMD に入っていたボーン:", list(keys)); return 1
    E = engine_curve(edir); B = bullet_curve(bdir)
    print("=" * 78)
    print("滑走曲線の比較 (box の初期位置からの移動量)")
    print("  MMD/PMXe は 30fps のベイク。当エンジン / 純Bullet は 60fps なので 2 倍で拾う。")
    print("=" * 78)
    print("  %-8s %-10s %12s %12s %12s" % ("F(30fps)", "秒", "MMD/PMXe", "当エンジン", "純Bullet"))
    f0 = min(box)
    o = box[f0]
    for f in sorted(box):
        if f % 15 and f != max(box): continue
        m = math.dist(box[f], o)
        e = math.dist(E[f * 2], E[0]) if E and f * 2 in E else float("nan")
        b = math.dist(B[f * 2], B[0]) if B and f * 2 in B else float("nan")
        print("  %-8d %-10.2f %12.4f %12.4f %12.4f" % (f, f / 30.0, m, e, b))
    print()
    print("  ★読み方: MMD が当エンジン/純Bullet より **遅い** なら、MMD には μ 以外の")
    print("    抵抗 (減衰など) がある = もみあげ 2.24x の由来はそこ。")
    print("    3つが揃うなら、滑走そのものは合っていて、もみあげの差は接触の形状/鎖の側にある。")
    return 0

sys.exit(main())
