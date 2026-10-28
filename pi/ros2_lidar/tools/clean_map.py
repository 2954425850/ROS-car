#!/usr/bin/env python3
"""Remove transient "people" specks from a saved SLAM map.

A 2D lidar records whatever was in front of it, so anyone walking through
while mapping leaves a spray of occupied cells. Those cells are never
re-observed as free (nobody walks that exact spot again), so they stay in
the map and later become fake obstacles for nav2.

Heuristic used here (deliberately conservative):

  * occupied cells are grouped into 8-connected components;
  * components with area >= LARGE cells are the real structure (walls,
    desks, cabinets) -- never touched;
  * a small component is deleted only if it is BOTH
        area < SMALL cells  AND
        farther than KEEP_DIST from every large component.

The distance test is what preserves chair/table legs: they are small, but
they sit right next to a desk or wall. A person in the aisle produces
isolated specks with nothing around them.

    python3 clean_map.py in.pgm out_basename [--small 5] [--large 20]
                                                      [--keep-dist 12]

Writes out_basename.pgm/.yaml/.png plus out_basename_diff.png (removed
cells painted red) so the removal can be eyeballed before using it.
"""
import argparse
import collections
from collections import deque

import numpy as np
from PIL import Image

EIGHT = ((1, 0), (-1, 0), (0, 1), (0, -1),
         (1, 1), (1, -1), (-1, 1), (-1, -1))


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', 'expected binary PGM (P5)'
        w, h = map(int, f.readline().split())
        f.readline()  # maxval
        data = np.frombuffer(f.read(), dtype=np.uint8)
    assert data.size == w * h, 'PGM size mismatch'
    return data.reshape(h, w)


def write_pgm(path, arr):
    h, w = arr.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(arr.tobytes())


def components(occ):
    h, w = occ.shape
    labels = np.full((h, w), -1, dtype=np.int32)
    comps = []
    for y in range(h):
        for x in range(w):
            if not occ[y, x] or labels[y, x] != -1:
                continue
            idx = len(comps)
            q = deque([(y, x)])
            labels[y, x] = idx
            cells = []
            while q:
                cy, cx = q.popleft()
                cells.append((cy, cx))
                for dy, dx in EIGHT:
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < h and 0 <= nx < w and occ[ny, nx] \
                            and labels[ny, nx] == -1:
                        labels[ny, nx] = idx
                        q.append((ny, nx))
            comps.append(cells)
    return labels, comps


def distance_to_large(large_mask, limit):
    """BFS distance (in cells, 8-connectivity) from any large-component cell."""
    h, w = large_mask.shape
    dist = np.full((h, w), limit + 1, dtype=np.int16)
    q = deque()
    for y, x in zip(*np.nonzero(large_mask)):
        dist[y, x] = 0
        q.append((y, x))
    while q:
        cy, cx = q.popleft()
        d = dist[cy, cx]
        if d >= limit:
            continue
        for dy, dx in EIGHT:
            ny, nx = cy + dy, cx + dx
            if 0 <= ny < h and 0 <= nx < w and dist[ny, nx] > d + 1:
                dist[ny, nx] = d + 1
                q.append((ny, nx))
    return dist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('out')
    ap.add_argument('--small', type=int, default=5,
                    help='components with fewer cells are candidates '
                         '(default 5 = 125 cm^2 at 5 cm/cell)')
    ap.add_argument('--large', type=int, default=20,
                    help='components at least this big count as structure '
                         '(default 20)')
    ap.add_argument('--keep-dist', type=int, default=12,
                    help='keep small components within this many cells '
                         '(default 12 = 60 cm) of structure')
    a = ap.parse_args()

    d = read_pgm(a.src)
    h, w = d.shape
    occ = d == 0
    labels, comps = components(occ)
    large = np.zeros_like(occ)
    small_ids = []
    for i, cells in enumerate(comps):
        if len(cells) >= a.large:
            for y, x in cells:
                large[y, x] = True
        else:
            small_ids.append(i)

    dist = distance_to_large(large, a.keep_dist)
    removed = np.zeros_like(occ)
    removed_comps = 0
    removed_cells = 0
    for i in small_ids:
        cells = comps[i]
        near = min(dist[y, x] for y, x in cells)
        drop = len(cells) < a.small and near > a.keep_dist
        if len(cells) == 1:
            drop = True          # single-pixel noise, always
        if drop:
            removed_comps += 1
            removed_cells += len(cells)
            for y, x in cells:
                removed[y, x] = True

    # What to write into the removed cells: free if the cell clearly sits in
    # open, already-observed space (>= 4 of its 8 neighbours are free), else
    # unknown. Marking an aisle speck as free lets nav2 drive through where a
    # person once stood; marking a speck out in unexplored space as free would
    # invent floor that was never seen.
    out = d.copy()
    free_mask = d == 255
    for y, x in zip(*np.nonzero(removed)):
        nfree = 0
        for dy, dx in EIGHT:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free_mask[ny, nx]:
                nfree += 1
        out[y, x] = 255 if nfree >= 4 else 254
    write_pgm(a.out + '.pgm', out)

    # Copy everything from the source map's yaml except the image name --
    # origin/resolution must match the pgm exactly or the map sits in the
    # wrong place for localization.
    src_yaml = a.src.rsplit('.', 1)[0] + '.yaml'
    meta = {}
    try:
        with open(src_yaml) as f:
            for line in f:
                if ':' in line:
                    k, v = line.split(':', 1)
                    meta[k.strip()] = v.strip()
    except OSError:
        meta = {}
    with open(a.out + '.yaml', 'w') as f:
        f.write('image: %s.pgm\n' % a.out.rsplit('/', 1)[-1])
        f.write('resolution: %s\n' % meta.get('resolution', '0.050000'))
        f.write('origin: %s\n' % meta.get('origin', '[0.0, 0.0, 0.0]'))
        f.write('negate: %s\n' % meta.get('negate', '0'))
        f.write('occupied_thresh: %s\n' % meta.get('occupied_thresh', '0.65'))
        f.write('free_thresh: %s\n' % meta.get('free_thresh', '0.196'))

    Image.frombytes('L', (out.shape[1], out.shape[0]),
                    out.tobytes()).save(a.out + '.png')

    diff = np.stack([out, out, out], axis=-1)
    diff[removed] = (255, 0, 0)
    Image.fromarray(diff).save(a.out + '_diff.png')

    print('components=%d  large=%d  small=%d' %
          (len(comps), len(comps) - len(small_ids), len(small_ids)))
    print('removed %d components / %d cells (%.2f m2)' %
          (removed_comps, removed_cells, removed_cells * 0.0025))


if __name__ == '__main__':
    main()
