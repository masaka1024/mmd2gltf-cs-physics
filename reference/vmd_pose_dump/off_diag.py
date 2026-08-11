# -*- coding: utf-8 -*-
# 180°回転差の切り分け(規約 vs 破綻)。compose.py を再利用。
# 1) 時間依存(frame 0/10/100/1000/7000) 2) クォータニオン符号(abs)+行列経由の突き合わせ 3) 回転軸の揃い + OFFスカート傾き中央値
import sys, math, os, time, io
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')  # cp932文字化け回避
import numpy as np
import compose as C

# データの場所は環境変数で指定する (off_compare.py の冒頭コメント参照)。
DATA   = os.environ.get("MMD_TESTDATA",
                        os.path.join(_HERE, "..", "..", "Assets", "testdata"))
MODEL  = os.environ.get("MMD_MODEL", "modelA")
MOTION = os.environ.get("MMD_MOTION", "motionA")

PMX     = os.path.join(DATA, MODEL + ".pmx")
VMD_ON  = os.path.join(DATA, MOTION + "_fix.vmd")
VMD_OFF = os.path.join(DATA, MOTION + "_off.vmd")
NF = 7001

def cat(nm):
    if any(k in nm for k in ('髪','ツインテ','もみあげ','前髪','モミアゲ')): return 'hair'
    if 'スカート' in nm: return 'skirt'
    return 'other'

bones, rbmap = C.parse_pmx(PMX)
order, depth = C.build_order(bones)
keys_on  = C.parse_vmd(VMD_ON)
keys_off = C.parse_vmd(VMD_OFF)
t0=time.time()
Wp_on,Wq_on   = C.compose_all(bones,order,keys_on ,NF)
Wp_off,Wq_off = C.compose_all(bones,order,keys_off,NF)
Wp_on=np.asarray(Wp_on); Wq_on=np.asarray(Wq_on); Wp_off=np.asarray(Wp_off); Wq_off=np.asarray(Wq_off)
print(f"compose x2 {time.time()-t0:.1f}s  bones={len(bones)}  Wq shape={Wq_on.shape}")

phys = [i for i,b in enumerate(bones) if cat(b['name']) in ('hair','skirt')]
skirt= [i for i,b in enumerate(bones) if cat(b['name'])=='skirt']
hair = [i for i,b in enumerate(bones) if cat(b['name'])=='hair']

def qdot(A,B): return (A*B).sum(axis=-1)
def ang_abs(A,B):  # 符号を揃えた(abs)角度 deg
    return np.degrees(2*np.arccos(np.clip(np.abs(qdot(A,B)),-1,1)))
def ang_nosign(A,B):  # abs無し(符号未処理)
    return np.degrees(2*np.arccos(np.clip(qdot(A,B),-1,1)))

def quat_to_mat(q):  # q=(x,y,z,w) -> 3x3
    x,y,z,w=q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
def mat_angle(Ra,Rb):  # 行列経由の相対回転角 deg
    R=Ra.T@Rb; c=(np.trace(R)-1)/2; return math.degrees(math.acos(max(-1,min(1,c))))

# ===== 1) 時間依存 =====
print("\n[1] 時間依存: ON vs OFF 回転差(abs) 物理ボーンの中央/最大 [deg]")
for f in (0,10,100,1000,7000):
    d = ang_abs(Wq_on[phys,f,:], Wq_off[phys,f,:])
    print(f"   frame {f:5d}: 中央={np.median(d):7.2f} 最大={d.max():7.2f}  (skirt中央={np.median(ang_abs(Wq_on[skirt,f,:],Wq_off[skirt,f,:])):.1f} hair中央={np.median(ang_abs(Wq_on[hair,f,:],Wq_off[hair,f,:])):.1f})")

# ===== 2) 符号処理の確認 + 行列突き合わせ =====
print("\n[2] クォータニオン符号: abs有り vs abs無し, および行列経由の一致 (frame1000, 物理先頭5本)")
fchk=1000
for i in phys[:5]:
    qa=Wq_on[i,fchk]; qb=Wq_off[i,fchk]
    a_abs=ang_abs(qa[None,:],qb[None,:])[0]
    a_nos=ang_nosign(qa[None,:],qb[None,:])[0]
    a_mat=mat_angle(quat_to_mat(qa),quat_to_mat(qb))
    print(f"   {bones[i]['name']:8s}: abs={a_abs:7.2f}  無符号={a_nos:7.2f}  行列={a_mat:7.2f}  dot={qdot(qa,qb):+.4f}")

# ===== 3) 回転軸の揃い + OFFスカート傾き =====
print("\n[3] 相対回転軸の揃い (frame1000, 物理ボーン, 角度>90°のもの)")
axes=[]
for i in phys:
    qa=Wq_on[i,fchk]; qb=Wq_off[i,fchk]
    # 相対 q_rel = qa^-1 * qb  (qa共役=(-x,-y,-z,w))
    ci=np.array([-qa[0],-qa[1],-qa[2],qa[3]])
    x1,y1,z1,w1=ci; x2,y2,z2,w2=qb
    qr=np.array([
        w1*x2+x1*w2+y1*z2-z1*y2,
        w1*y2-x1*z2+y1*w2+z1*x2,
        w1*z2+x1*y2-y1*x2+z1*w2,
        w1*w2-x1*x2-y1*y2-z1*z2])
    if qr[3]<0: qr=-qr
    ang=math.degrees(2*math.acos(max(-1,min(1,qr[3]))))
    if ang<90: continue
    ax=qr[:3]; n=np.linalg.norm(ax)
    if n>1e-6: axes.append(ax/n)
axes=np.array(axes)
if len(axes)>0:
    mean_ax=axes.mean(axis=0); mean_ax/=np.linalg.norm(mean_ax)
    dots=np.abs(axes@mean_ax)  # |各軸・平均軸| (1=揃い, 0=直交)
    print(f"   n(>90°)={len(axes)}  平均軸=({mean_ax[0]:+.3f},{mean_ax[1]:+.3f},{mean_ax[2]:+.3f})")
    print(f"   |軸・平均軸| 中央={np.median(dots):.3f} 最小={dots.min():.3f}  (1に近い=揃い=規約 / ばらばら=破綻)")
else:
    print("   >90°の物理ボーンなし")

# OFFスカート傾き(ボーン親子相対の角度)中央値: ON も同法で並記
def tilt_median(Wq, idxs):
    vals=[]
    for i in idxs:
        p=bones[i]['parent']
        if p is None or p<0: continue
        d=ang_abs(Wq[i], Wq[p])  # (NF,)
        vals.append(d)
    v=np.concatenate(vals); return np.median(v), np.percentile(v,90), v.max()
sm_on =tilt_median(Wq_on, skirt); sm_off=tilt_median(Wq_off,skirt)
print(f"\n[3b] スカート傾き(ボーン親子相対角) 中央/p90/最大: ON={sm_on[0]:.2f}/{sm_on[1]:.2f}/{sm_on[2]:.2f}  OFF={sm_off[0]:.2f}/{sm_off[1]:.2f}/{sm_off[2]:.2f}")
print(f"     (11°前後=常識的で回転そのまま比較可 / 180°級=規約の問題確定)")

# ===== 移動ロックJoint親子相対位置の保持度 (補正層の正体) =====
# スカート/髪の子ボーンについて、(子-親)の世界相対位置が bind相対からどれだけズレるか。
# 移動ロックなら理想0。ON(補正)=小さく保持 / OFF(純Bullet)=大きくズレ、が予想。
print("\n[4] 移動ロック相対位置の保持度 (子-親 の bind相対からのズレ, 中央/p90/最大)")
bindpos=np.array([b['pos'] for b in bones])
def rel_drift(Wp, idxs):
    vals=[]
    for i in idxs:
        p=bones[i]['parent']
        if p is None or p<0: continue
        rel = Wp[i]-Wp[p]                      # (NF,3)
        bind= bindpos[i]-bindpos[p]            # (3,)
        d = np.linalg.norm(rel-bind[None,:],axis=1)
        vals.append(d)
    v=np.concatenate(vals); return np.median(v), np.percentile(v,90), v.max()
for nm,idxs in (('skirt',skirt),('hair',hair)):
    on=rel_drift(Wp_on,idxs); off=rel_drift(Wp_off,idxs)
    print(f"   {nm:5s}: ON 中央={on[0]:.3f}/p90={on[1]:.3f}/最大={on[2]:.3f}  OFF 中央={off[0]:.3f}/p90={off[1]:.3f}/最大={off[2]:.3f}")

# 参考: 髪tip の bind からの世界移動量(OFFが実際に飛んでいるか)
print("\n[参考] 髪FR5 の frame別 ON/OFF 世界位置 (飛んでいるか)")
i5=[i for i,b in enumerate(bones) if b['name']=='髪FR5']
if i5:
    i5=i5[0]
    for f in (0,100,1000,7000):
        po=Wp_on[i5,f]; pf=Wp_off[i5,f]
        print(f"   f{f:5d}: ON=({po[0]:.2f},{po[1]:.2f},{po[2]:.2f}) OFF=({pf[0]:.2f},{pf[1]:.2f},{pf[2]:.2f}) 差={np.linalg.norm(po-pf):.2f}")
