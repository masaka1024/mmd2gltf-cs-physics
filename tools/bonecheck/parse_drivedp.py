# -*- coding: utf-8 -*-
"""drivedp の出力を読んで {条件ラベル: {部位: (中央, p90)}} と参照値を返す。"""
import io, re

def parse(path):
    labels = []; ref = {}; cond = {}
    for l in io.open(path, encoding='utf-8'):
        if '|' not in l:
            continue
        seg = [s for s in l.rstrip().split('|')]
        # 条件ラベル行: 先頭セルが空白のみで、後続に OFF/ON/自前 を含む
        body = [s.strip() for s in seg[1:] if s.strip()]
        if body and all(('OFF' in s or 'ON' in s or s == '自前') for s in body) and not re.search(r'\d', seg[0]):
            labels = body; continue
        m = re.match(r'\s{2}(\S+)\s+(\d+)\s*$', seg[0])
        if not m:
            continue
        part = m.group(1)
        nums = [s.split() for s in seg[1:] if s.strip()]
        ref[part] = (float(nums[0][0]), float(nums[0][1]))
        for i, t in enumerate(nums[1:]):
            cond.setdefault(labels[i] if i < len(labels) else str(i), {})[part] = (float(t[0]), float(t[2]))
    return ref, cond
