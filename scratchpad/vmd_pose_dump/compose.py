# -*- coding: utf-8 -*-
# vmd_pose_dump (rewrite, 2026-08-09): PMX + VMD -> ボーンのワールド絶対姿勢 CSV。
# 旧ツール(pmx_min/vmd_min/compose/task4_export)の再実装。
# 仕様/落とし穴は本セッションの記録どおり:
#   - CSV列 frame,boneName,posX..quatW, PMXネイティブ(スケール/Z反転なし), ワールド絶対, %.7g
#   - 合成: Wq=親Wq⊗animRot, Wp=親Wp+rotate(親Wq, offset+animPos), 処理順=layer昇順→depth昇順
#   - IKは実装しない(足IKはベイク済/OFF指定, 二重適用回避)。付与親も適用しない(髪/スカートに無い)。
#   - 移動キーを落とさない(animPos必須)。欠損フレーム: pos線形/rot slerp。
import struct, sys, math

# ---------- quaternion (x,y,z,w) ----------
def qnorm(q):
    x,y,z,w=q; n=math.sqrt(x*x+y*y+z*z+w*w) or 1.0
    return (x/n,y/n,z/n,w/n)
def qmul(a,b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,
            aw*bw-ax*bx-ay*by-az*bz)
def qrot(q,v):
    x,y,z,w=q; vx,vy,vz=v
    # t = 2*cross(q.xyz, v); v' = v + w*t + cross(q.xyz, t)
    tx=2*(y*vz-z*vy); ty=2*(z*vx-x*vz); tz=2*(x*vy-y*vx)
    return (vx+w*tx+(y*tz-z*ty), vy+w*ty+(z*tx-x*tz), vz+w*tz+(x*ty-y*tx))
def qslerp(a,b,t):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    d=ax*bx+ay*by+az*bz+aw*bw
    if d<0: b=(-bx,-by,-bz,-bw); d=-d
    if d>0.9995:
        r=(ax+t*(b[0]-ax),ay+t*(b[1]-ay),az+t*(b[2]-az),aw+t*(b[3]-aw)); return qnorm(r)
    th=math.acos(max(-1,min(1,d))); s=math.sin(th)
    wa=math.sin((1-t)*th)/s; wb=math.sin(t*th)/s
    return (wa*ax+wb*b[0],wa*ay+wb*b[1],wa*az+wb*b[2],wa*aw+wb*b[3])

# ---------- PMX ----------
def parse_pmx(path):
    d=open(path,'rb').read(); g=d[8]; gg=d[9:9+g]
    enc=gg[0]; adduv=gg[1]; vidx=gg[2]; tidx=gg[3]; midx=gg[4]; bidx=gg[5]; mo=gg[6]; ridx=gg[7]
    ENC='utf-16-le' if enc==0 else 'utf-8'; o=9+g
    def rt(o):
        ln=struct.unpack('<I',d[o:o+4])[0]; o+=4; return d[o:o+ln].decode(ENC,'replace'),o+ln
    for _ in range(4): _,o=rt(o)
    vc=struct.unpack('<I',d[o:o+4])[0]; o+=4
    for _ in range(vc):
        o+=32+adduv*16; wt=d[o]; o+=1; o+=[vidx,vidx*2+4,vidx*4+16,vidx*2+40,vidx*4+16][wt]; o+=4
    fc=struct.unpack('<I',d[o:o+4])[0]; o+=4; o+=fc*vidx
    tc=struct.unpack('<I',d[o:o+4])[0]; o+=4
    for _ in range(tc): _,o=rt(o)
    mc=struct.unpack('<I',d[o:o+4])[0]; o+=4
    for _ in range(mc):
        _,o=rt(o); _,o=rt(o); o+=16+12+4+12+1+16+4+tidx+tidx+1; sh=d[o]; o+=1; o+=(1 if sh else tidx); _,o=rt(o); o+=4
    bc=struct.unpack('<I',d[o:o+4])[0]; o+=4
    bones=[]
    def ridx_read(o,size):
        v=struct.unpack('<b' if size==1 else '<h' if size==2 else '<i',d[o:o+size])[0]; return v,o+size
    for _ in range(bc):
        name,o=rt(o); _,o=rt(o)
        pos=struct.unpack('<3f',d[o:o+12]); o+=12
        parent,o=ridx_read(o,bidx)
        layer=struct.unpack('<i',d[o:o+4])[0]; o+=4
        fl=struct.unpack('<H',d[o:o+2])[0]; o+=2
        append=None
        o+= bidx if fl&1 else 12
        if fl&0x0100 or fl&0x0200:
            ap,o=ridx_read(o,bidx); o+=4; append=ap
        if fl&0x0400: o+=12
        if fl&0x0800: o+=24
        if fl&0x2000: o+=4
        if fl&0x0020:
            o+=bidx+8; lc=struct.unpack('<I',d[o:o+4])[0]; o+=4
            for _ in range(lc):
                o+=bidx; lim=d[o]; o+=1; o+= 24 if lim else 0
        bones.append(dict(name=name,pos=pos,parent=parent,layer=layer,append=append,flags=fl))
    # --- 続けて 剛体まで歩き、剛体→ボーン対応を返す(パーサ一本化) ---
    mc2=struct.unpack('<I',d[o:o+4])[0]; o+=4
    per={0:mo+4,1:vidx+12,2:bidx+28,3:vidx+16,4:vidx+16,5:vidx+16,6:vidx+16,7:vidx+16,8:midx+57,9:mo+4,10:ridx+25}
    for _ in range(mc2):
        _,o=rt(o); _,o=rt(o); o+=1; t=d[o]; o+=1; cnt=struct.unpack('<I',d[o:o+4])[0]; o+=4; o+=cnt*per[t]
    dc=struct.unpack('<I',d[o:o+4])[0]; o+=4
    for _ in range(dc):
        _,o=rt(o); _,o=rt(o); o+=1; ec=struct.unpack('<I',d[o:o+4])[0]; o+=4
        for _ in range(ec):
            tt=d[o]; o+=1; o+= bidx if tt==0 else mo
    rc=struct.unpack('<I',d[o:o+4])[0]; o+=4
    rb_bonemap=[]  # (rbname, boneIndex)
    for _ in range(rc):
        nm,o=rt(o); _,o=rt(o); bn,o=ridx_read(o,bidx); o+=1+2+1+12+12+12+20+1
        rb_bonemap.append((nm,bn))
    return bones, rb_bonemap

# ---------- VMD ----------
def parse_vmd(path):
    d=open(path,'rb').read(); off=30+20
    nb=struct.unpack('<I',d[off:off+4])[0]; off+=4
    keys={}
    for _ in range(nb):
        rec=d[off:off+111]; off+=111
        name=rec[:15].split(b'\x00')[0]
        try: name=name.decode('cp932')
        except: name=name.decode('latin1','replace')
        frame=struct.unpack('<I',rec[15:19])[0]
        px,py,pz=struct.unpack('<3f',rec[19:31])
        rx,ry,rz,rw=struct.unpack('<4f',rec[31:47])
        keys.setdefault(name,[]).append((frame,(px,py,pz),qnorm((rx,ry,rz,rw))))
    for k in keys: keys[k].sort(key=lambda t:t[0])
    return keys

def sample(keylist, f):
    # keylist sorted by frame. return (pos, quat) at frame f (pos線形/rot slerp)。
    import bisect
    frames=[k[0] for k in keylist]
    i=bisect.bisect_left(frames,f)
    if i<len(frames) and frames[i]==f: return keylist[i][1],keylist[i][2]
    if i==0: return keylist[0][1],keylist[0][2]
    if i>=len(frames): return keylist[-1][1],keylist[-1][2]
    f0,p0,q0=keylist[i-1]; f1,p1,q1=keylist[i]; t=(f-f0)/(f1-f0)
    p=(p0[0]+t*(p1[0]-p0[0]),p0[1]+t*(p1[1]-p0[1]),p0[2]+t*(p1[2]-p0[2]))
    return p,qslerp(q0,q1,t)

def build_order(bones):
    # depth
    depth=[0]*len(bones)
    for i,b in enumerate(bones):
        d=0; p=b['parent']
        while p is not None and 0<=p<len(bones) and d<1000: d+=1; p=bones[p]['parent']
        depth[i]=d
    order=sorted(range(len(bones)), key=lambda i:(bones[i]['layer'], depth[i], i))
    return order, depth

def fk_frame(bones, order, anim):
    # anim: dict idx->(pos,quat). returns list of (Wp, Wq) per bone index.
    Wp=[None]*len(bones); Wq=[None]*len(bones)
    for i in order:
        b=bones[i]; ar_p,ar_q = anim.get(i,((0,0,0),(0,0,0,1)))
        p=b['parent']
        if p is None or p<0 or p>=len(bones):
            pWq=(0,0,0,1); pWp=(0,0,0); off=b['pos']
        else:
            pWq=Wq[p]; pWp=Wp[p]
            off=(b['pos'][0]-bones[p]['pos'][0], b['pos'][1]-bones[p]['pos'][1], b['pos'][2]-bones[p]['pos'][2])
        wq=qmul(pWq,ar_q)
        loc=(off[0]+ar_p[0],off[1]+ar_p[1],off[2]+ar_p[2])
        rp=qrot(pWq,loc)
        Wp[i]=(pWp[0]+rp[0],pWp[1]+rp[1],pWp[2]+rp[2]); Wq[i]=wq
    return Wp,Wq

import numpy as np
def qmul_b(A,B):
    ax,ay,az,aw=A[:,0],A[:,1],A[:,2],A[:,3]; bx,by,bz,bw=B[:,0],B[:,1],B[:,2],B[:,3]
    return np.stack([aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz],axis=1)
def qrot_b(Q,V):
    x,y,z,w=Q[:,0],Q[:,1],Q[:,2],Q[:,3]; vx,vy,vz=V[:,0],V[:,1],V[:,2]
    tx=2*(y*vz-z*vy); ty=2*(z*vx-x*vz); tz=2*(x*vy-y*vx)
    return np.stack([vx+w*tx+(y*tz-z*ty), vy+w*ty+(z*tx-x*tz), vz+w*tz+(x*ty-y*tx)],axis=1)

def anim_arrays(bones, keys, NF):
    # 各ボーンの (rot(NF,4), pos(NF,3)) float32。密キーは直接、疎/欠損は補間/恒等。
    R=np.tile(np.array([0,0,0,1],dtype=np.float64),(len(bones),NF,1))
    T=np.zeros((len(bones),NF,3),dtype=np.float64)
    for i,b in enumerate(bones):
        kl=keys.get(b['name'])
        if not kl: continue
        frames=[k[0] for k in kl]
        if len(kl)==NF and frames[0]==0 and frames[-1]==NF-1:  # 密・連続
            for f,(p,q) in zip(frames,[(k[1],k[2]) for k in kl]):
                T[i,f]=p; R[i,f]=q
        else:
            for f in range(NF):
                p,q=sample(kl,f); T[i,f]=p; R[i,f]=q
    return R,T

def compose_all(bones, order, keys, NF):
    R,T=anim_arrays(bones,keys,NF)
    Wq=[None]*len(bones); Wp=[None]*len(bones)
    for i in order:
        b=bones[i]; p=b['parent']
        if p is None or p<0 or p>=len(bones):
            pWq=np.tile(np.array([0,0,0,1],dtype=np.float64),(NF,1)); pWp=np.zeros((NF,3),dtype=np.float64)
            off=np.array(b['pos'],dtype=np.float64)
        else:
            pWq=Wq[p]; pWp=Wp[p]
            off=np.array([b['pos'][0]-bones[p]['pos'][0],b['pos'][1]-bones[p]['pos'][1],b['pos'][2]-bones[p]['pos'][2]],dtype=np.float64)
        Wq[i]=qmul_b(pWq,R[i])
        Wp[i]=pWp+qrot_b(pWq, off[None,:]+T[i])
    return Wp,Wq

if __name__=='__main__':
    PMX=r"C:/mytask2/unity-bullet-physics/Assets/testdata/IA.pmx"
    VMD=r"C:/mytask2/unity-bullet-physics/Assets/testdata/IA_Conqueror_full_key_version_fix.vmd"
    REF=r"C:/mytask2/_external_testdata/IA_bone_world_pose.csv"
    OUT=r"C:/mytask2/_external_testdata/IA_bone_world_pose_43_check.csv"
    bones,rbmap=parse_pmx(PMX); order,depth=build_order(bones)
    nidx={b['name']:i for i,b in enumerate(bones)}
    keys=parse_vmd(VMD); NF=7001
    print(f"PMX bones={len(bones)}, 剛体={len(rbmap)}")
    # bind test
    Wp0,Wq0=fk_frame(bones,order,{}); me=max(max(abs(Wp0[i][j]-bones[i]['pos'][j]) for j in range(3)) for i in range(len(bones)))
    print(f"[bind test] 全{len(bones)}ボーン max位置誤差={me:.3e} 期待0")
    # hair translation max + which bone
    hair=[b['name'] for b in bones if any(k in b['name'] for k in ('髪','ツインテ','もみあげ','前髪','モミアゲ'))]
    hm=[(nm,max(math.sqrt(p[0]**2+p[1]**2+p[2]**2) for _,p,_ in keys[nm])) for nm in hair if nm in keys]
    hm.sort(key=lambda x:-x[1]); a=lambda s:s.encode('ascii','backslashreplace').decode()
    print(f"[hair移動] 最大 {hm[0][1]:.3f} @ {a(hm[0][0])} / 上位5: {[(a(n),round(v,2)) for n,v in hm[:5]]}")
    # hair bone <-> rigidbody (本体パーサ一本化)
    from collections import Counter
    hb_rb=Counter()
    for rn,bi in rbmap:
        if 0<=bi<len(bones) and any(k in bones[bi]['name'] for k in ('髪','ツインテ','もみあげ','前髪','モミアゲ')): hb_rb[bones[bi]['name']]+=1
    print(f"[hair-rb対応] 剛体紐づく髪ボーン={len(hb_rb)}, 剛体数分布={dict(Counter(hb_rb.values()))}, 複数剛体ボーン={[a(b) for b,c in hb_rb.items() if c>1]}")
    # 43-bone order from REF
    order43=[]
    with open(REF,encoding='utf-8') as f:
        next(f)
        for line in f:
            c=line.split(',')
            if c[0]!='0': break
            order43.append(c[1])
    idx43=[nidx[n] for n in order43]
    import time; t0=time.time()
    Wp,Wq=compose_all(bones,order,keys,NF); print(f"compose {NF}f in {time.time()-t0:.1f}s")
    # output 43-bone CSV (%.7g)
    g='%.7g'
    with open(OUT,'w',encoding='utf-8',newline='') as f:
        f.write("frame,boneName,posX,posY,posZ,quatX,quatY,quatZ,quatW\n")
        for fr in range(NF):
            for n,i in zip(order43,idx43):
                p=Wp[i][fr]; q=Wq[i][fr]
                f.write(f"{fr},{n},{g%p[0]},{g%p[1]},{g%p[2]},{g%q[0]},{g%q[1]},{g%q[2]},{g%q[3]}\n")
    import os
    ob=os.path.getsize(OUT); rb=os.path.getsize(REF)
    print(f"[byte一致] OUT={ob} REF={rb} 期待27556907 {'一致' if ob==rb else '不一致'}")
    # value diff vs REF
    maxpos=0; maxq=0
    with open(OUT,encoding='utf-8') as fo, open(REF,encoding='utf-8') as fr2:
        next(fo); next(fr2)
        for lo,lr in zip(fo,fr2):
            co=lo.split(','); cr=lr.split(',')
            for j in (2,3,4): maxpos=max(maxpos,abs(float(co[j])-float(cr[j])))
            qo=[float(co[k]) for k in (5,6,7,8)]; qr=[float(cr[k]) for k in (5,6,7,8)]
            if sum(a*b for a,b in zip(qo,qr))<0: qo=[-x for x in qo]
            for a2,b2 in zip(qo,qr): maxq=max(maxq,abs(a2-b2))
    print(f"[値差] max位置={maxpos:.3e} max回転成分={maxq:.3e}")
