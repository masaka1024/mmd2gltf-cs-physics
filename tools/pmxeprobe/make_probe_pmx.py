# -*- coding: utf-8 -*-
# pmxeprobe: PmxEditor の補正層([ボーン位置合わせ再計算]/[Jointロック内部演算])を
# ブラックボックス同定するための最小 PMX を生成する。
#
# 設計方針 (2026-08-13):
#   - 「軌跡」でなく「静的平衡(固定点)」を測る。単一の動的剛体を重力で回転リミットへ
#     押し付け、釣り合った状態を読む。カオス発散も別ベイクの軌道分岐も起きない。
#   - 接触を完全に排除する(非衝突グループフラグ=0x0000 = どのグループとも衝突しない)。
#     残る力は 重力 + Joint のみ。PGS の鎖漏れも無い(リンク1本)。
#   - 移動ロック(min=max=0)にすることで [Jointロック内部演算] の対象になる。
#     readme.txt:1051「移動ロック指定のJointは内部的に補正処理を行っています」
#
# 仕様出典: PmxEditor 同梱 Lib/PMX仕様/PMX仕様.txt (●PMXヘッダ/●頂点/●ボーン/●剛体/●Joint)
# PMX 2.0 で出力する(2.1 拡張は不要)。

import math
import os
import struct

# ---------------- 低レベル書き出し ----------------

ENC = "utf-16-le"  # ヘッダ [0]=0

def f32(v):     return struct.pack("<f", v)
def f3(v):      return struct.pack("<3f", *v)
def i32(v):     return struct.pack("<i", v)
def u16(v):     return struct.pack("<H", v)
def u8(v):      return struct.pack("<B", v)
def i8(v):      return struct.pack("<b", v)

def text(s):
    b = s.encode(ENC)
    return i32(len(b)) + b

def rad(deg):
    return deg * math.pi / 180.0


class ProbeModel:
    """最小 PMX。Index サイズは全て 1byte 固定(要素数が 127 未満である前提)。"""

    def __init__(self, name):
        self.name = name
        self.bones = []       # (name, pos, parent, tail_offset)
        self.bodies = []      # dict
        self.joints = []      # dict
        self.verts = []       # (pos, bone)  BDEF1 のみ
        self.faces = []       # (a, b, c)
        self.materials = []   # (name, rgba, 面数) 省略時は従来どおり単一材質

    # --- 構築 API ---

    def add_measure_triad(self, bone, origin, scale=1.0):
        """回転観測用の四面体。原点 O と、そこから X/Y/Z へ scale だけ伸ばした3点を
        指定ボーンに BDEF1 で剛体ウェイト付けする。

        [現在の形状で保存] は頂点をスキニング済みの座標で書き出すので、保存後の
        (v_X - v_O, v_Y - v_O, v_Z - v_O) がそのままボーンの回転行列の各列になる。
        PMX のボーンレコードは位置しか持たず回転が残らないので、回転側を観測する
        唯一の手段がこれ。面に含めないと保存時に落ちる可能性があるので3面張る。
        """
        o = len(self.verts)
        self.verts.append((origin, bone))
        for d in ((scale, 0.0, 0.0), (0.0, scale, 0.0), (0.0, 0.0, scale)):
            self.verts.append(((origin[0] + d[0], origin[1] + d[1], origin[2] + d[2]), bone))
        self.faces += [(o, o + 1, o + 2), (o, o + 2, o + 3), (o, o + 3, o + 1)]
        return o

    def add_box_mesh(self, bone, center, half, rot_z=0.0, name=None, rgba=(0.8, 0.8, 0.8, 1.0)):
        """見える箱を1つ足す。**MMD は物理演算中のボーンマーカーを描かない**ので、
        剛体の動きを目視するには形状が要る。面ごとに頂点を複製して平坦シェーディングにする。
        rot_z は Z 軸まわりの傾き [rad]。ボーンは回転を持てないので頂点側へ焼き込む。"""
        c, s_ = math.cos(rot_z), math.sin(rot_z)
        hx, hy, hz = half
        base = len(self.verts)
        corner = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    x, y, z = sx * hx, sy * hy, sz * hz
                    corner.append((center[0] + x * c - y * s_,
                                   center[1] + x * s_ + y * c,
                                   center[2] + z))
        # 8 隅の並びは (sx,sy,sz) の辞書順: 0=(-,-,-) 1=(-,-,+) 2=(-,+,-) 3=(-,+,+)
        #                                   4=(+,-,-) 5=(+,-,+) 6=(+,+,-) 7=(+,+,+)
        quads = [(0, 1, 3, 2), (4, 6, 7, 5),    # -X, +X
                 (0, 4, 5, 1), (2, 3, 7, 6),    # -Y, +Y
                 (0, 2, 6, 4), (1, 5, 7, 3)]    # -Z, +Z
        nfaces = 0
        for q in quads:
            for tri in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
                a, bb, cc = (corner[i] for i in tri)
                u = (bb[0] - a[0], bb[1] - a[1], bb[2] - a[2])
                v = (cc[0] - a[0], cc[1] - a[1], cc[2] - a[2])
                nx = u[1] * v[2] - u[2] * v[1]
                ny = u[2] * v[0] - u[0] * v[2]
                nz = u[0] * v[1] - u[1] * v[0]
                ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
                nrm = (nx / ln, ny / ln, nz / ln)
                i0 = len(self.verts)
                for pt in (a, bb, cc):
                    self.verts.append((pt, bone, nrm))
                self.faces.append((i0, i0 + 1, i0 + 2))
                nfaces += 1
        self.materials.append((name or ("mat%d" % len(self.materials)), rgba, nfaces))
        return base

    def add_bone(self, name, pos, parent=-1, tail=(0.0, -1.0, 0.0)):
        self.bones.append((name, pos, parent, tail))
        return len(self.bones) - 1

    def add_body(self, name, bone, pos, mass, mode, shape=0, size=(0.5, 0.5, 0.5),
                 rot=(0.0, 0.0, 0.0), lin_damp=0.0, ang_damp=0.0,
                 restitution=0.0, friction=0.0, group=0, mask=0x0000):
        """mode: 0=ボーン追従 1=物理演算 2=物理+ボーン位置合わせ
        mask: 非衝突グループフラグ。実体は collide-with マスク。0x0000 = 一切衝突しない"""
        self.bodies.append(dict(
            name=name, bone=bone, group=group, mask=mask, shape=shape, size=size,
            pos=pos, rot=rot, mass=mass, lin_damp=lin_damp, ang_damp=ang_damp,
            restitution=restitution, friction=friction, mode=mode))
        return len(self.bodies) - 1

    def add_joint(self, name, a, b, pos, rot=(0.0, 0.0, 0.0),
                  lin_min=(0.0, 0.0, 0.0), lin_max=(0.0, 0.0, 0.0),
                  ang_min=(0.0, 0.0, 0.0), ang_max=(0.0, 0.0, 0.0),
                  spring_pos=(0.0, 0.0, 0.0), spring_rot=(0.0, 0.0, 0.0)):
        self.joints.append(dict(
            name=name, a=a, b=b, pos=pos, rot=rot,
            lin_min=lin_min, lin_max=lin_max, ang_min=ang_min, ang_max=ang_max,
            spring_pos=spring_pos, spring_rot=spring_rot))
        return len(self.joints) - 1

    # --- 出力 ---

    def to_bytes(self, comment=""):
        out = bytearray()

        # ●PMXヘッダ
        out += b"PMX "
        out += f32(2.0)
        out += u8(8)
        out += bytes([0,   # エンコード方式 0:UTF16
                      0,   # 追加UV数
                      1,   # 頂点Indexサイズ
                      1,   # テクスチャIndexサイズ
                      1,   # 材質Indexサイズ
                      1,   # ボーンIndexサイズ
                      1,   # モーフIndexサイズ
                      1])  # 剛体Indexサイズ

        # ●モデル情報
        out += text(self.name)
        out += text(self.name)
        out += text(comment)
        out += text(comment)

        # ●頂点 : 既定はダミー三角形1枚(ボーン0)。measure_triad があればそれも含む
        verts = self.verts if self.verts else [
            ((0.0, 0.0, 0.0), 0), ((1.0, 0.0, 0.0), 0), ((0.0, 1.0, 0.0), 0)]
        faces = self.faces if self.faces else [(0, 1, 2)]

        out += i32(len(verts))
        for vrec in verts:
            p, bone = vrec[0], vrec[1]
            nrm = vrec[2] if len(vrec) > 2 else (0.0, 0.0, -1.0)
            out += f3(p)              # 位置
            out += f3(nrm)            # 法線
            out += struct.pack("<2f", 0.0, 0.0)  # UV
            out += u8(0)              # BDEF1
            out += i8(bone)
            out += f32(1.0)           # エッジ倍率

        # ●面
        out += i32(len(faces) * 3)
        for f in faces:
            for v in f:
                out += u8(v)          # 頂点Indexサイズ=1, 頂点は符号なし

        # ●テクスチャ
        out += i32(0)

        # ●材質 : materials が空なら従来どおり全面を1材質でくるむ
        mats = self.materials if self.materials else [("mat", (1.0, 1.0, 1.0, 1.0), len(faces))]
        out += i32(len(mats))
        for (mname, rgba, nf) in mats:
            out += text(mname) + text(mname)
            out += struct.pack("<4f", *rgba)                # Diffuse
            out += f3((0.1, 0.1, 0.1)) + f32(5.0)           # Specular + 係数
            out += f3((rgba[0] * 0.5, rgba[1] * 0.5, rgba[2] * 0.5))   # Ambient
            out += u8(0x01)                                 # 描画フラグ: 両面描画
            out += struct.pack("<4f", 0.0, 0.0, 0.0, 1.0)   # エッジ色
            out += f32(1.0)                                 # エッジサイズ
            out += i8(-1)                                   # 通常テクスチャ
            out += i8(-1)                                   # スフィア
            out += u8(0)                                    # スフィアモード 無効
            out += u8(1)                                    # 共有Toonフラグ
            out += u8(0)                                    # 共有Toon[0]
            out += text("")                                 # メモ
            out += i32(nf * 3)                              # 面(頂点)数

        # ●ボーン
        out += i32(len(self.bones))
        for (name, pos, parent, tail) in self.bones:
            out += text(name) + text(name)
            out += f3(pos)
            out += i8(parent)
            out += i32(0)                               # 変形階層
            # 接続先=0(オフセット) / 回転可能 / 移動可能 / 表示 / 操作可
            # ★移動可能(0x0004)は必須。物理で動いたボーンは平行移動しているので、
            #   [Fixモーションの非物理化保存] が移動キーを書けずに失敗する。
            out += u16(0x0002 | 0x0004 | 0x0008 | 0x0010)
            out += f3(tail)                             # 座標オフセット

        # ●モーフ
        out += i32(0)

        # ●表示枠 : Root と 表情 の2つの特殊枠。
        # 仕様書●表示枠「PMXの初期状態では 表示枠:0 -> "Root", 表示枠:1 -> "表情"(いずれも特殊枠)」
        # 「編集時に誤って削除しないように注意」= エディタ側が両方の存在を前提にしている。
        # Root だけだと [現在の形状で保存] が通らないケースがあるため両方入れる。
        out += i32(2)

        out += text("Root") + text("Root")
        out += u8(1)                                    # 特殊枠
        out += i32(1)
        out += u8(0)                                    # 要素対象:ボーン
        out += i8(0)

        out += text("表情") + text("Exp")
        out += u8(1)                                    # 特殊枠
        out += i32(0)                                   # 枠内要素なし

        # ●剛体
        out += i32(len(self.bodies))
        for b in self.bodies:
            out += text(b["name"]) + text(b["name"])
            out += i8(b["bone"])
            out += u8(b["group"])
            out += u16(b["mask"])
            out += u8(b["shape"])
            out += f3(b["size"])
            out += f3(b["pos"])
            out += f3(b["rot"])
            out += f32(b["mass"])
            out += f32(b["lin_damp"])
            out += f32(b["ang_damp"])
            out += f32(b["restitution"])
            out += f32(b["friction"])
            out += u8(b["mode"])

        # ●Joint
        out += i32(len(self.joints))
        for j in self.joints:
            out += text(j["name"]) + text(j["name"])
            out += u8(0)                                # スプリング6DOF
            out += i8(j["a"])
            out += i8(j["b"])
            out += f3(j["pos"])
            out += f3(j["rot"])
            out += f3(j["lin_min"])
            out += f3(j["lin_max"])
            out += f3(j["ang_min"])
            out += f3(j["ang_max"])
            out += f3(j["spring_pos"])
            out += f3(j["spring_rot"])

        return bytes(out)

    def save(self, path, comment=""):
        with open(path, "wb") as fp:
            fp.write(self.to_bytes(comment))
        return path


# ---------------- 実験モデル ----------------

ROOT_Y = 10.0     # 親ボーン(固定点)の高さ
LINK = 2.0        # 親→子のリンク長(PMX単位)
R = 0.5           # 剛体半径(球。カプセルはマージン挙動が絡むので避ける)


def build_pendulum(mass, ang_min_deg, ang_max_deg, child_mode=1,
                   joint_at_parent=True, damp=0.9, triad=False, name="probe"):
    """親=ボーン追従(固定) / 子=物理演算 の1リンク振り子。
    接触なし(mask=0)・ばね0。Joint は移動ロック。

    ★腕は水平(-Z 方向)。重力(-Y)と直交させることで X 軸まわりのトルク m*g*L が最大になり、
      子が回転リミットへ押し付けられて静止する。鉛直に吊ると初期姿勢が既に平衡で
      リミットに一度も当たらず、何も測れない。
    ★減衰を入れて振動を殺す。静的平衡の「位置」は減衰に依存しない(速度0で減衰項も0)ので
      同定対象には影響しない。収束を速くするためだけのもの。

    平衡姿勢を4条件(補正2トグル ON/OFF)で読み比べる。
    """
    m = ProbeModel(name)

    arm = (0.0, 0.0, -LINK)  # PMX は -Z が正面。水平に張り出す
    child_pos = (0.0, ROOT_Y, -LINK)

    b_root = m.add_bone("親", (0.0, ROOT_Y, 0.0), -1, arm)
    b_child = m.add_bone("子", child_pos, b_root, arm)

    rb_root = m.add_body("親剛体", b_root, (0.0, ROOT_Y, 0.0),
                         mass=0.0, mode=0, shape=0, size=(R, 0.0, 0.0))
    rb_child = m.add_body("子剛体", b_child, child_pos,
                          mass=mass, mode=child_mode, shape=0, size=(R, 0.0, 0.0),
                          lin_damp=damp, ang_damp=damp)

    if triad:
        # 子ボーンに回転観測用の四面体。ダミー三角形(ボーン0)も残す
        m.verts += [((0.0, 0.0, 0.0), b_root), ((1.0, 0.0, 0.0), b_root),
                    ((0.0, 1.0, 0.0), b_root)]
        m.faces += [(0, 1, 2)]
        m.add_measure_triad(b_child, child_pos, scale=1.0)

    jpos = (0.0, ROOT_Y, 0.0) if joint_at_parent else child_pos

    # 回転は X 軸まわりのみ許可。Y/Z はロック(min=max=0)。
    m.add_joint("J0", rb_root, rb_child, jpos,
                lin_min=(0.0, 0.0, 0.0), lin_max=(0.0, 0.0, 0.0),
                ang_min=(rad(ang_min_deg), 0.0, 0.0),
                ang_max=(rad(ang_max_deg), 0.0, 0.0))
    return m


def build_gravity_meter(mass=1.0, k=100.0, name="E6_gravity_meter"):
    """重力計。親(ボーン追従)から子(動的)を Y 方向のばねで吊る。
    静的平衡のたわみは x = m*g/k で重力に正比例するので、
    「重力設定が本当に効いているか」を数値で確認できる。

    Y の移動制限を広く開け、Y のばね定数だけ立てる。回転は全ロック(min=max=0)。
    """
    m = ProbeModel(name)

    b_root = m.add_bone("親", (0.0, ROOT_Y, 0.0), -1, (0.0, -LINK, 0.0))
    b_child = m.add_bone("子", (0.0, ROOT_Y - LINK, 0.0), b_root, (0.0, -LINK, 0.0))

    rb_root = m.add_body("親剛体", b_root, (0.0, ROOT_Y, 0.0),
                         mass=0.0, mode=0, shape=0, size=(R, 0.0, 0.0))
    rb_child = m.add_body("子剛体", b_child, (0.0, ROOT_Y - LINK, 0.0),
                          mass=mass, mode=1, shape=0, size=(R, 0.0, 0.0),
                          lin_damp=0.9, ang_damp=0.9)

    # ジョイントは子の位置に置く(=たわみ0が初期姿勢)。Y だけ自由+ばね。
    m.add_joint("J0", rb_root, rb_child, (0.0, ROOT_Y - LINK, 0.0),
                lin_min=(0.0, -20.0, 0.0), lin_max=(0.0, 20.0, 0.0),
                ang_min=(0.0, 0.0, 0.0), ang_max=(0.0, 0.0, 0.0),
                spring_pos=(0.0, k, 0.0))
    return m



# ---------------- 複合プローブ (1モデルに独立実験を並べる) ----------------

ISLAND_PITCH = 8.0   # 島どうしの X 間隔。剛体マスク0なので干渉しないが視認のため離す


def add_island(m, x, tag, nlinks=1, link=2.0, limit_deg=5.0, joint_at="parent",
               lin_locked=True, child_mode=1, damp=0.9, mass=1.0):
    """独立した振り子の島を1つ追加する。

    非衝突グループフラグ0x0000 で島どうしは一切干渉しないので、
    1モデルに何個並べても各島が勝手に自分の固定点へ落ちる。
    = GUI 1回の保存で島の数だけデータ点が取れる。

    joint_at: "parent"(親剛体の位置) / "mid"(中点) / "child"(子剛体の位置)
              → Joint フレームのオフセット量を振る用
    lin_locked: False にすると移動制限を開ける(補正の対象外になるはずの対照)
    """
    root = m.add_bone(f"{tag}_root", (x, ROOT_Y, 0.0), -1, (0.0, 0.0, -link))
    m.add_body(f"{tag}_b0", root, (x, ROOT_Y, 0.0),
               mass=0.0, mode=0, shape=0, size=(R, 0.0, 0.0))

    prev_bone, prev_body, prev_pos = root, len(m.bodies) - 1, (x, ROOT_Y, 0.0)
    for i in range(1, nlinks + 1):
        pos = (x, ROOT_Y, -link * i)
        bone = m.add_bone(f"{tag}_bone{i}", pos, prev_bone, (0.0, 0.0, -link))
        m.add_body(f"{tag}_b{i}", bone, pos, mass=mass, mode=child_mode,
                   shape=0, size=(R, 0.0, 0.0), lin_damp=damp, ang_damp=damp)
        body = len(m.bodies) - 1

        if joint_at == "parent":
            jpos = prev_pos
        elif joint_at == "mid":
            jpos = (x, ROOT_Y, (prev_pos[2] + pos[2]) * 0.5)
        else:
            jpos = pos

        lmin = (0.0, 0.0, 0.0) if lin_locked else (-5.0, -5.0, -5.0)
        lmax = (0.0, 0.0, 0.0) if lin_locked else (5.0, 5.0, 5.0)

        m.add_joint(f"{tag}_J{i}", prev_body, body, jpos,
                    lin_min=lmin, lin_max=lmax,
                    ang_min=(rad(-limit_deg), 0.0, 0.0),
                    ang_max=(rad(limit_deg), 0.0, 0.0))
        prev_bone, prev_body, prev_pos = bone, body, pos


def build_multi(name="E9_multi"):
    """[Jointロック内部演算] のトリガー探し + オフセット/荷重スイープを1モデルに集約。

    トグル条件だけは分離が必要だが、それ以外のパラメータは全部同居できる。
    どれか1島だけ値が変われば、その構成が発動条件。
    """
    m = ProbeModel(name)
    islands = [
        # (tag,          nlinks, link, limit, joint_at, lin_locked, mode)
        ("base",              1, 2.0,  5.0, "parent", True,  1),  # E2 と同一 = 基準
        ("offMid",            1, 2.0,  5.0, "mid",    True,  1),  # フレームオフセット 1
        ("offChild",          1, 2.0,  5.0, "child",  True,  1),  # フレームオフセット 0
        ("len1",              1, 1.0,  5.0, "parent", True,  1),  # 荷重 τ=m*g*L を L で振る
        ("len4",              1, 4.0,  5.0, "parent", True,  1),  # 収束しない既知の島(参考)
        ("lim30",             1, 2.0, 30.0, "parent", True,  1),  # リミット角
        ("lim0",              1, 2.0,  0.0, "parent", True,  1),  # 回転も完全ロック
        # 鎖はリンク長2.0だと収束しなかった(同条件の再撮でばらつく)。
        # 腕が短い島は完全に決定的だったので、負荷を下げて固定点に落とす。
        ("chain2",            2, 1.0,  5.0, "parent", True,  1),  # ★動的×動的の移動ロック
        ("chain3",            3, 1.0,  5.0, "parent", True,  1),  # ★さらに長い鎖
        ("chain2L",           2, 0.5,  5.0, "parent", True,  1),  # さらに軽い鎖(保険)
        ("linFree",           1, 2.0,  5.0, "parent", False, 1),  # 移動ロックなし = 対照
        ("mode2",             1, 2.0,  5.0, "parent", True,  2),  # A との相互作用
    ]
    for i, (tag, n, link, lim, jat, lock, mode) in enumerate(islands):
        add_island(m, i * ISLAND_PITCH, tag, nlinks=n, link=link, limit_deg=lim,
                   joint_at=jat, lin_locked=lock, child_mode=mode)
    return m


def main():
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
    os.makedirs(outdir, exist_ok=True)
    made = []

    # --- E0: Joint なし自由落下。ハーネス検証用(補正は無関係のはず) ---
    m = ProbeModel("E0_freefall")
    b = m.add_bone("親", (0.0, ROOT_Y, 0.0), -1)
    m.add_body("落下剛体", b, (0.0, ROOT_Y, 0.0), mass=1.0, mode=1,
               shape=0, size=(R, 0.0, 0.0))
    made.append(m.save(os.path.join(outdir, "E0_freefall.pmx"),
                       "Joint なし。重力のみ。更新1回あたりの進み方の確認用"))

    # --- E1: 回転自由(±180°)。位置側=[ボーン位置合わせ再計算]の同定 ---
    m = build_pendulum(mass=1.0, ang_min_deg=-180.0, ang_max_deg=180.0,
                       name="E1_linlock_angfree")
    made.append(m.save(os.path.join(outdir, "E1_linlock_angfree.pmx"),
                       "移動ロック+回転自由。アンカー誤差が0へ潰れるかを見る"))

    # --- E2: 回転リミット±5°。質量スイープで超過量の入出力対応を取る ---
    # δ(超過量) が荷重に依存しなければ hard clamp、比例すれば部分緩和。
    for mass in (0.1, 0.5, 1.0, 2.0, 5.0, 10.0):
        tag = f"{mass:g}".replace(".", "p")
        m = build_pendulum(mass=mass, ang_min_deg=-5.0, ang_max_deg=5.0,
                           name=f"E2_lim5_m{tag}")
        made.append(m.save(os.path.join(outdir, f"E2_lim5_m{tag}.pmx"),
                           f"回転リミット±5° 質量{mass}。リミット超過量の荷重依存を見る"))

    # --- E4: 剛体タイプ 2(物理+ボーン位置合わせ) との切り分け ---
    m = build_pendulum(mass=1.0, ang_min_deg=-5.0, ang_max_deg=5.0, child_mode=2,
                       name="E4_mode2_lim5")
    made.append(m.save(os.path.join(outdir, "E4_mode2_lim5.pmx"),
                       "E2 の子剛体を mode2 へ。PMX側モード と ツール側トグル の切り分け"))

    # --- E7/E8: 回転観測用の四面体つき。E2/E4 と同条件で回転行列を読む ---
    for mode, tag in ((1, "E7_triad_mode1"), (2, "E8_triad_mode2")):
        m = build_pendulum(mass=1.0, ang_min_deg=-5.0, ang_max_deg=5.0,
                           child_mode=mode, triad=True, name=tag)
        made.append(m.save(os.path.join(outdir, f"{tag}.pmx"),
                           "子ボーンに測定用四面体。保存後の頂点からボーンの回転行列を復元する"))

    # --- E9: 複合プローブ。GUI 1回で11島ぶんのデータが取れる ---
    m = build_multi()
    made.append(m.save(os.path.join(outdir, "E9_multi.pmx"),
                       "独立した振り子を11島。トグル以外のパラメータを1モデルに集約"))

    # --- E6: 重力計。重力設定が実際に効いているかの計測系検証 ---
    m = build_gravity_meter()
    made.append(m.save(os.path.join(outdir, "E6_gravity_meter.pmx"),
                       "Yばね k=100 で吊る。たわみ x = m*g/k が重力に正比例する"))

    for p in made:
        print(f"{os.path.getsize(p):6d}  {os.path.relpath(p, outdir)}")
    print(f"\n{len(made)} models -> {outdir}")


if __name__ == "__main__":
    main()
