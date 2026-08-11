# -*- coding: utf-8 -*-
# 補正ON版 vs 補正OFF版 の対照実験(タスク1) + OFF版CSV作成(タスク2)。
# compose.py の関数を再利用。既存CSVは非上書き(別名出力)。
import sys, math, os, time, io
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import numpy as np
import compose as C

# データはリポジトリに含めない (再配布回避)。場所は環境変数で指定する。
#   MMD_TESTDATA : モデル/モーションの置き場 (既定: このリポジトリの Assets/testdata)
#   MMD_REFCSV   : ベイク済みCSVの置き場     (既定: MMD_TESTDATA と同じ)
#   MMD_MODEL    : モデルのファイル名 (拡張子なし。既定: modelA)
#   MMD_MOTION   : モーションのファイル名 (拡張子なし。既定: motionA)
DATA   = os.environ.get("MMD_TESTDATA",
                        os.path.join(_HERE, "..", "..", "Assets", "testdata"))
REFDIR = os.environ.get("MMD_REFCSV", DATA)
MODEL  = os.environ.get("MMD_MODEL", "modelA")
MOTION = os.environ.get("MMD_MOTION", "motionA")

PMX     = os.path.join(DATA, MODEL + ".pmx")
VMD_ON  = os.path.join(DATA, MOTION + "_fix.vmd")
VMD_OFF = os.path.join(DATA, MOTION + "_off.vmd")
REF     = os.path.join(REFDIR, MODEL + "_bone_world_pose.csv")       # ON版 43ボーン(既存)
OUT43   = os.path.join(REFDIR, MODEL + "_bone_world_pose_off_43_check.csv")
OUTHAIR = os.path.join(REFDIR, MODEL + "_bone_world_pose_hair_off.csv")  # OFF版 108(43+髪65)
NF = 7001
g = '%.7g'

def a(s): return s.encode('ascii','backslashreplace').decode()
def catof(nm):
    if any(k in nm for k in ('髪','ツインテ','もみあげ','前髪','モミアゲ')): return 'hair'
    if 'スカート' in nm: return 'skirt'
    return 'other'

bones, rbmap = C.parse_pmx(PMX)
order, depth = C.build_order(bones)
nidx = {b['name']:i for i,b in enumerate(bones)}
print(f"PMX bones={len(bones)} 剛体={len(rbmap)}")

keys_on  = C.parse_vmd(VMD_ON)
keys_off = C.parse_vmd(VMD_OFF)
print(f"[VMD] ON keys(bones)={len(keys_on)}  OFF keys(bones)={len(keys_off)}")

# ===== タスク1: 生キーの一致確認 (駆動=物理でないボーン は完全一致すべき) =====
# 物理ボーン(髪/スカート)は差があるべき。
def keyhash(kl):
    # frame列 + 全キーの丸め(1e-6)ハッシュ
    return tuple((f, tuple(round(x,6) for x in p), tuple(round(x,6) for x in q)) for f,p,q in kl)
same_other=0; diff_other=[]; same_phys=0; diff_phys=0; missing=0
for b in bones:
    nm=b['name']; c=catof(nm)
    ko=keys_on.get(nm); kf=keys_off.get(nm)
    if ko is None and kf is None: continue
    if ko is None or kf is None: missing+=1; continue
    eq = keyhash(ko)==keyhash(kf)
    if c=='other':
        if eq: same_other+=1
        else: diff_other.append(nm)
    else:
        if eq: same_phys+=1
        else: diff_phys+=1
print(f"[タスク1 生キー] 非物理(駆動/その他): 一致={same_other} 不一致={len(diff_other)}  物理(髪/スカート): 一致={same_phys} 不一致={diff_phys}  片側欠損={missing}")
if diff_other: print(f"   ★非物理で不一致(想定外): {[a(n) for n in diff_other[:20]]}")

# ===== FK合成 (両版) =====
t0=time.time(); Wp_on,Wq_on   = C.compose_all(bones,order,keys_on ,NF)
Wp_off,Wq_off = C.compose_all(bones,order,keys_off,NF); print(f"compose x2 in {time.time()-t0:.1f}s")

def qang(Q0,Q1):  # per-frame angle deg (NF,)
    d=np.clip(np.abs((Q0*Q1).sum(axis=1)),-1,1); return np.degrees(2*np.arccos(d))
def stt(x):
    x=np.sort(np.asarray(x,float)); return (x[len(x)//2], x[int(len(x)*0.9)] if len(x)>1 else x[-1], x.max())

# ===== タスク1: FKワールド差 をボーン種別ごとに =====
cats={'other':[], 'skirt':[], 'hair':[]}
for i,b in enumerate(bones):
    c=catof(b['name'])
    dp=np.linalg.norm(Wp_on[i]-Wp_off[i],axis=1)   # (NF,)
    dq=qang(Wq_on[i],Wq_off[i])
    cats[c].append((b['name'], float(dp.max()), float(np.median(dp)), float(dq.max()), float(np.median(dq))))
print("[タスク1 FKワールド差 ON vs OFF] 種別ごと per-bone(max/median over frames)の集計:")
for c in ('other','skirt','hair'):
    arr=cats[c]
    if not arr: continue
    pm=[x[1] for x in arr]  # per-bone max pos diff
    qm=[x[3] for x in arr]  # per-bone max rot diff
    md,p9,mx=stt(pm); qmd,qp9,qmx=stt(qm)
    print(f"  {c:5s} n={len(arr):3d}: 位置差 中央={md:.4f} p90={p9:.4f} 最大={mx:.4f} | 回転差 中央={qmd:.2f} p90={qp9:.2f} 最大={qmx:.2f}°")
    top=sorted(arr,key=lambda x:-x[1])[:3]
    print(f"        位置差 上位: {[(a(n),round(v,3)) for n,v,_,_,_ in top]}")

# ===== タスク2: OFF版 CSV 出力 (43-check + 108 hair) =====
order43=[]
with open(REF,encoding='utf-8') as f:
    next(f)
    for line in f:
        c=line.split(',');
        if c[0]!='0': break
        order43.append(c[1])
idx43=[nidx[n] for n in order43]
c43=[catof(n) for n in order43]
from collections import Counter
print(f"[43ボーン種別] {dict(Counter(c43))}")

with open(OUT43,'w',encoding='utf-8',newline='') as f:
    f.write("frame,boneName,posX,posY,posZ,quatX,quatY,quatZ,quatW\n")
    for fr in range(NF):
        for n,i in zip(order43,idx43):
            p=Wp_off[i][fr]; q=Wq_off[i][fr]
            f.write(f"{fr},{n},{g%p[0]},{g%p[1]},{g%p[2]},{g%q[0]},{g%q[1]},{g%q[2]},{g%q[3]}\n")
print(f"[OFF 43CSV] {OUT43} ({os.path.getsize(OUT43)}B)")

hair_idx=[i for i,b in enumerate(bones) if catof(b['name'])=='hair']
hair_names=[bones[i]['name'] for i in hair_idx]
order108=list(zip(order43,idx43))+list(zip(hair_names,hair_idx))
with open(OUTHAIR,'w',encoding='utf-8',newline='') as f:
    f.write("frame,boneName,posX,posY,posZ,quatX,quatY,quatZ,quatW\n")
    for fr in range(NF):
        for n,i in order108:
            p=Wp_off[i][fr]; q=Wq_off[i][fr]
            f.write(f"{fr},{n},{g%p[0]},{g%p[1]},{g%p[2]},{g%q[0]},{g%q[1]},{g%q[2]},{g%q[3]}\n")
print(f"[OFF hairCSV] {OUTHAIR} ({os.path.getsize(OUTHAIR)}B, {len(order108)}ボーン×{NF}f)")

# 再現性: OFF43の駆動(other)部が ON REF の同ボーンと一致するか (=入力同一の裏取り)
# ONのREFから other ボーンだけ読み、OFF43のother行と値比較。
ref_other={}
with open(REF,encoding='utf-8') as f:
    next(f)
    for line in f:
        cc=line.rstrip('\n').split(',')
        if catof(cc[1])=='other':
            ref_other[(cc[0],cc[1])]=cc[2:]
maxpos=0.0; maxq=0.0; cmp=0
with open(OUT43,encoding='utf-8') as f:
    next(f)
    for line in f:
        cc=line.rstrip('\n').split(',')
        if catof(cc[1])!='other': continue
        r=ref_other.get((cc[0],cc[1]))
        if r is None: continue
        cmp+=1
        for j in range(3): maxpos=max(maxpos,abs(float(cc[2+j])-float(r[j])))
        qo=[float(cc[5+k]) for k in range(4)]; qr=[float(r[3+k]) for k in range(4)]
        if sum(x*y for x,y in zip(qo,qr))<0: qo=[-x for x in qo]
        for x,y in zip(qo,qr): maxq=max(maxq,abs(x-y))
print(f"[再現性] OFF43 の other(駆動)部 vs ON REF: 比較{cmp}行 max位置差={maxpos:.3e} max回転差={maxq:.3e} (期待~0=入力同一)")
