from __future__ import annotations

import argparse
import json
import math
import colorsys
from collections import deque
from pathlib import Path

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "benchmark_results" / "iron_region_graph"
MAP_CONFIGS = {
    "iron": ROOT / "example_problems" / "iron_harvest.domain" / "maps" / "scene_mp_2p_01.map",
    "orz": ROOT / "example_problems" / "game.domain" / "maps" / "orz900d.map",
}
MAP_PATH = MAP_CONFIGS["iron"]
MAP_KEY = "iron"
PREFIX = "iron_regions"


BLOCK = 16
PASSABLE_RATIO = 0.30
MIN_BARRIER_CELLS = 4
NEAR_SPAN_GAP = 2
WALL_EXTENSION_DEPTH = 6
WALL_EXTENSION_THICKNESS = 5
MAX_ISOLATED_PASSABLE_ISLAND = 3
TARGET_REGIONS = 90
MIN_REGION_CELLS = 8
BOTTLENECK_CLEARANCE = 2
BOTTLENECK_WALL_PRESSURE = 5
V1_MIN_REGION_CELLS = 18
V1_TARGET_REGIONS = 34
V2_MIN_OBSTACLE_CELLS = 6
V2_MAX_OBSTACLES = 90
V2_NEAREST_OBSTACLES = 3
V2_MAX_BRIDGE_DISTANCE = 18
V2_MIN_BRIDGE_DISTANCE = 2
V2_MIN_REGION_CELLS = 14
V2_AMBIGUOUS_MERGE_ABSOLUTE = 24
V2_AMBIGUOUS_MERGE_RATIO = 0.28
V2_AMBIGUOUS_MERGE_PASSES = 3


PALETTE = [
    (230, 57, 70),
    (29, 53, 87),
    (69, 123, 157),
    (42, 157, 143),
    (233, 196, 106),
    (244, 162, 97),
    (131, 56, 236),
    (255, 0, 110),
    (58, 134, 255),
    (6, 214, 160),
    (255, 209, 102),
    (17, 138, 178),
    (239, 71, 111),
    (115, 210, 222),
    (7, 59, 76),
    (156, 197, 161),
]


def read_map(path: Path):
    lines = path.read_text().splitlines()
    width = height = None
    grid_start = None
    for idx, line in enumerate(lines):
        lower = line.lower()
        if lower.startswith("height"):
            height = int(line.split()[-1])
        elif lower.startswith("width"):
            width = int(line.split()[-1])
        elif lower == "map":
            grid_start = idx + 1
            break
    if width is None or height is None or grid_start is None:
        raise ValueError(f"Cannot parse map header: {path}")
    grid = lines[grid_start:grid_start + height]
    return width, height, grid


def is_free(ch: str) -> bool:
    return ch in ".G"


def count_free_and_connected(width: int, height: int, grid, bx: int, by: int):
    """Return whether one coarse cell is safe to collapse into a passable node.

    A coarse cell is an approximate obstacle when it is too sparse, contains a
    wall-like obstacle component spanning the cell, or has split free space.
    """
    x0 = bx * BLOCK
    x1 = min(width, x0 + BLOCK)
    y0 = by * BLOCK
    y1 = min(height, y0 + BLOCK)
    total = (y1 - y0) * (x1 - x0)
    if total <= 0:
        return 0, False, 0.0

    local_w = x1 - x0
    local_h = y1 - y0
    free_map = [[False] * local_w for _ in range(local_h)]
    obstacle_map = [[False] * local_w for _ in range(local_h)]
    free_cells = []

    for ly, row_y in enumerate(range(y0, y1)):
        row = grid[row_y]
        for lx, col_x in enumerate(range(x0, x1)):
            if is_free(row[col_x]):
                free_map[ly][lx] = True
                free_cells.append((ly, lx))
            else:
                obstacle_map[ly][lx] = True

    free_count = len(free_cells)
    free_ratio = free_count / total
    if free_count == 0:
        return 0, False, 0.0
    if free_ratio < PASSABLE_RATIO:
        return free_count, False, free_ratio

    # A long obstacle stripe inside a coarse cell should not be represented as
    # passable just because the remaining free strip can still touch two edges.
    visited_obstacle = [[False] * local_w for _ in range(local_h)]
    dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    for sy in range(local_h):
        for sx in range(local_w):
            if not obstacle_map[sy][sx] or visited_obstacle[sy][sx]:
                continue
            q = deque([(sy, sx)])
            visited_obstacle[sy][sx] = True
            size = 0
            component_cells = []
            touches_top = touches_bottom = False
            touches_left = touches_right = False
            while q:
                cy, cx = q.popleft()
                size += 1
                component_cells.append((cy, cx))
                touches_top = touches_top or cy == 0
                touches_bottom = touches_bottom or cy == local_h - 1
                touches_left = touches_left or cx == 0
                touches_right = touches_right or cx == local_w - 1
                for dy, dx in dirs:
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < local_h and 0 <= nx < local_w:
                        if obstacle_map[ny][nx] and not visited_obstacle[ny][nx]:
                            visited_obstacle[ny][nx] = True
                            q.append((ny, nx))
            spans_vertically = touches_top and touches_bottom
            spans_horizontally = touches_left and touches_right
            # In iron, many meaningful walls cross a block boundary and leave a
            # tiny side gap inside the next block. Collapsing that block into a
            # normal region node hides the wall, so treat near-spanning obstacle
            # components as approximate obstacles too.
            xs = [lx for _, lx in component_cells]
            ys = [ly for ly, _ in component_cells]
            near_spans_horizontally = (
                (touches_left and local_w - 1 - max(xs) <= NEAR_SPAN_GAP) or
                (touches_right and min(xs) <= NEAR_SPAN_GAP)
            ) if xs else False
            near_spans_vertically = (
                (touches_top and local_h - 1 - max(ys) <= NEAR_SPAN_GAP) or
                (touches_bottom and min(ys) <= NEAR_SPAN_GAP)
            ) if ys else False
            if size >= MIN_BARRIER_CELLS and (
                spans_vertically or spans_horizontally or
                near_spans_vertically or near_spans_horizontally
            ):
                return free_count, False, free_ratio

    # BFS to check if all free cells form ONE connected component
    visited = [[False] * local_w for _ in range(local_h)]

    start = free_cells[0]
    q = deque([start])
    visited[start[0]][start[1]] = True
    component_cells = [start]

    while q:
        cy, cx = q.popleft()
        for dy, dx in dirs:
            ny, nx = cy + dy, cx + dx
            if 0 <= ny < local_h and 0 <= nx < local_w:
                if free_map[ny][nx] and not visited[ny][nx]:
                    visited[ny][nx] = True
                    q.append((ny, nx))
                    component_cells.append((ny, nx))

    # Check: are ALL free cells in this one component?
    if len(component_cells) != free_count:
        # Free cells are divided into multiple disconnected groups
        return free_count, False, free_ratio

    # Check edge connectivity: which edges can the free component reach?
    # Edge = top (ly==0), bottom (ly==local_h-1), left (lx==0), right (lx==local_w-1)
    edges_reached = set()
    for ly, lx in component_cells:
        if ly == 0: edges_reached.add('top')
        if ly == local_h - 1: edges_reached.add('bottom')
        if lx == 0: edges_reached.add('left')
        if lx == local_w - 1: edges_reached.add('right')

    # Must be able to reach at least 2 edges, including opposite pairs
    # e.g. {top, bottom} = vertical passage, {left, right} = horizontal passage
    # {top, left} = corner only (not enough)
    is_opposite_pair = (
        ('top' in edges_reached and 'bottom' in edges_reached) or
        ('left' in edges_reached and 'right' in edges_reached)
    )

    is_passable = is_opposite_pair
    return free_count, is_passable, free_ratio


def build_coarse_graph(width: int, height: int, grid):
    cw = math.ceil(width / BLOCK)
    ch = math.ceil(height / BLOCK)
    passable = [False] * (cw * ch)
    free_ratio = [0.0] * (cw * ch)

    for by in range(ch):
        for bx in range(cw):
            idx = by * cw + bx
            free_count, is_pass, ratio = count_free_and_connected(width, height, grid, bx, by)
            passable[idx] = is_pass
            free_ratio[idx] = ratio
    mark_wall_extension_cells(width, height, grid, cw, ch, passable, free_ratio)
    prune_small_passable_islands(cw, ch, passable)
    prune_isolated_passable_fragments(cw, ch, passable, free_ratio)
    return cw, ch, passable, free_ratio


def obstacle_components_in_block(width: int, height: int, grid, bx: int, by: int):
    x0 = bx * BLOCK
    x1 = min(width, x0 + BLOCK)
    y0 = by * BLOCK
    y1 = min(height, y0 + BLOCK)
    local_w = x1 - x0
    local_h = y1 - y0
    obstacle_map = [[False] * local_w for _ in range(local_h)]
    for ly, row_y in enumerate(range(y0, y1)):
        row = grid[row_y]
        for lx, col_x in enumerate(range(x0, x1)):
            obstacle_map[ly][lx] = not is_free(row[col_x])

    visited = [[False] * local_w for _ in range(local_h)]
    comps = []
    for sy in range(local_h):
        for sx in range(local_w):
            if not obstacle_map[sy][sx] or visited[sy][sx]:
                continue
            q = deque([(sy, sx)])
            visited[sy][sx] = True
            cells = []
            while q:
                cy, cx = q.popleft()
                cells.append((cy, cx))
                for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < local_h and 0 <= nx < local_w:
                        if obstacle_map[ny][nx] and not visited[ny][nx]:
                            visited[ny][nx] = True
                            q.append((ny, nx))
            comps.append((cells, local_w, local_h))
    return comps


def mark_wall_extension_cells(width: int, height: int, grid, cw: int, ch: int, passable, free_ratio):
    original_passable = list(passable)
    for idx, ok in enumerate(original_passable):
        if not ok or free_ratio[idx] >= 0.90:
            continue
        bx = idx % cw
        by = idx // cw
        for cells, local_w, local_h in obstacle_components_in_block(width, height, grid, bx, by):
            if len(cells) < MIN_BARRIER_CELLS:
                continue
            xs = [x for _, x in cells]
            ys = [y for y, _ in cells]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            touches_top = min_y == 0
            touches_bottom = max_y == local_h - 1
            touches_left = min_x == 0
            touches_right = max_x == local_w - 1

            extended_from_blocked_neighbor = (
                (touches_top and by > 0 and not original_passable[idx - cw] and
                 max_y - min_y + 1 >= WALL_EXTENSION_DEPTH and max_x - min_x + 1 >= WALL_EXTENSION_THICKNESS) or
                (touches_bottom and by + 1 < ch and not original_passable[idx + cw] and
                 max_y - min_y + 1 >= WALL_EXTENSION_DEPTH and max_x - min_x + 1 >= WALL_EXTENSION_THICKNESS) or
                (touches_left and bx > 0 and not original_passable[idx - 1] and
                 max_x - min_x + 1 >= WALL_EXTENSION_DEPTH and max_y - min_y + 1 >= WALL_EXTENSION_THICKNESS) or
                (touches_right and bx + 1 < cw and not original_passable[idx + 1] and
                 max_x - min_x + 1 >= WALL_EXTENSION_DEPTH and max_y - min_y + 1 >= WALL_EXTENSION_THICKNESS)
            )
            if extended_from_blocked_neighbor:
                passable[idx] = False
                break


def prune_small_passable_islands(cw: int, ch: int, passable):
    visited = [False] * (cw * ch)
    for idx, ok in enumerate(list(passable)):
        if not ok or visited[idx]:
            continue
        cells = []
        visited[idx] = True
        q = deque([idx])
        while q:
            cur = q.popleft()
            cells.append(cur)
            for nb in neighbors(cur, cw, ch):
                if passable[nb] and not visited[nb]:
                    visited[nb] = True
                    q.append(nb)
        if len(cells) <= MAX_ISOLATED_PASSABLE_ISLAND:
            for cell in cells:
                passable[cell] = False


def prune_isolated_passable_fragments(cw: int, ch: int, passable, free_ratio):
    for idx, ok in enumerate(list(passable)):
        if not ok:
            continue
        cardinal_open = sum(1 for nb in neighbors(idx, cw, ch) if passable[nb])
        if cardinal_open == 0 and free_ratio[idx] < 0.75:
            passable[idx] = False


def neighbors(idx: int, cw: int, ch: int):
    x = idx % cw
    y = idx // cw
    if y > 0:
        yield idx - cw
    if y + 1 < ch:
        yield idx + cw
    if x > 0:
        yield idx - 1
    if x + 1 < cw:
        yield idx + 1


def distance_to_blocked(cw: int, ch: int, passable):
    dist = [10**9] * (cw * ch)
    q = deque()
    for i, ok in enumerate(passable):
        if not ok:
            dist[i] = 0
            q.append(i)
    while q:
        cur = q.popleft()
        for nb in neighbors(cur, cw, ch):
            if dist[nb] > dist[cur] + 1:
                dist[nb] = dist[cur] + 1
                q.append(nb)
    return dist


def connected_passable_components(cw: int, ch: int, passable):
    comp = [-1] * (cw * ch)
    comps = []
    for i, ok in enumerate(passable):
        if not ok or comp[i] != -1:
            continue
        cid = len(comps)
        cells = []
        comp[i] = cid
        q = deque([i])
        while q:
            cur = q.popleft()
            cells.append(cur)
            for nb in neighbors(cur, cw, ch):
                if passable[nb] and comp[nb] == -1:
                    comp[nb] = cid
                    q.append(nb)
        comps.append(cells)
    return comp, comps


def connected_obstacle_components(cw: int, ch: int, passable):
    comp = [-1] * (cw * ch)
    comps = []
    for i, ok in enumerate(passable):
        if ok or comp[i] != -1:
            continue
        cid = len(comps)
        cells = []
        comp[i] = cid
        q = deque([i])
        while q:
            cur = q.popleft()
            cells.append(cur)
            for nb in neighbors(cur, cw, ch):
                if not passable[nb] and comp[nb] == -1:
                    comp[nb] = cid
                    q.append(nb)
        comps.append(cells)
    return comp, comps


def obstacle_boundary_cells(cw: int, ch: int, passable, cells):
    boundary = []
    for cell in cells:
        if any(passable[nb] for nb in neighbors(cell, cw, ch)):
            boundary.append(cell)
    return boundary


def component_bbox(cells, cw):
    xs = [c % cw for c in cells]
    ys = [c // cw for c in cells]
    return min(xs), min(ys), max(xs), max(ys)


def sample_cells(cells, limit=80):
    if len(cells) <= limit:
        return cells
    step = max(1, len(cells) // limit)
    return cells[::step][:limit]


def nearest_component_bridge(a_cells, b_cells, cw):
    best = None
    for a in sample_cells(a_cells):
        ax, ay = a % cw, a // cw
        for b in sample_cells(b_cells):
            bx, by = b % cw, b // cw
            dist = abs(ax - bx) + abs(ay - by)
            if best is None or dist < best[0]:
                best = (dist, a, b)
    return best


def bbox_manhattan_gap(a_bbox, b_bbox):
    ax0, ay0, ax1, ay1 = a_bbox
    bx0, by0, bx1, by1 = b_bbox
    dx = 0
    if ax1 < bx0:
        dx = bx0 - ax1
    elif bx1 < ax0:
        dx = ax0 - bx1
    dy = 0
    if ay1 < by0:
        dy = by0 - ay1
    elif by1 < ay0:
        dy = ay0 - by1
    return dx + dy


def bbox_center(bbox):
    x0, y0, x1, y1 = bbox
    return (x0 + x1) / 2.0, (y0 + y1) / 2.0


def line_cells(a: int, b: int, cw: int):
    x0, y0 = a % cw, a // cw
    x1, y1 = b % cw, b // cw
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    out = []
    while True:
        out.append(y * cw + x)
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return out


def build_v2_obstacle_partitions(cw: int, ch: int, passable):
    _, obstacle_comps = connected_obstacle_components(cw, ch, passable)
    obstacles = []
    for cid, cells in enumerate(obstacle_comps):
        boundary = obstacle_boundary_cells(cw, ch, passable, cells)
        if len(cells) < V2_MIN_OBSTACLE_CELLS or not boundary:
            continue
        x0, y0, x1, y1 = component_bbox(cells, cw)
        # Keep large objects and long wall-like objects. Tiny clutter is noise.
        span = max(x1 - x0 + 1, y1 - y0 + 1)
        score = len(cells) + len(boundary) * 0.7 + span * 2.0
        obstacles.append({
            "id": cid,
            "cells": cells,
            "boundary": boundary,
            "bbox": [x0, y0, x1, y1],
            "score": score,
        })
    obstacles.sort(key=lambda item: item["score"], reverse=True)
    obstacles = obstacles[:V2_MAX_OBSTACLES]

    bridges = {}
    for i, obs in enumerate(obstacles):
        candidates = []
        ox, oy = bbox_center(obs["bbox"])
        ranked = []
        for j, other in enumerate(obstacles):
            if i == j:
                continue
            gap = bbox_manhattan_gap(obs["bbox"], other["bbox"])
            if gap > V2_MAX_BRIDGE_DISTANCE + 4:
                continue
            tx, ty = bbox_center(other["bbox"])
            ranked.append((gap, abs(ox - tx) + abs(oy - ty), j, other))
        ranked.sort(key=lambda item: (item[0], item[1]))
        for gap, _, j, other in ranked[:12]:
            bridge = nearest_component_bridge(obs["boundary"], other["boundary"], cw)
            if bridge is None:
                continue
            dist, a, b = bridge
            if V2_MIN_BRIDGE_DISTANCE <= dist <= V2_MAX_BRIDGE_DISTANCE:
                candidates.append((dist, i, j, a, b))
        candidates.sort(key=lambda item: item[0])
        for dist, i, j, a, b in candidates[:V2_NEAREST_OBSTACLES]:
            key = tuple(sorted((i, j)))
            if key not in bridges or dist < bridges[key]["dist"]:
                bridges[key] = {"dist": dist, "a": a, "b": b, "obs": key}

    virtual_walls = set()
    for bridge in bridges.values():
        cells = line_cells(bridge["a"], bridge["b"], cw)
        passable_on_line = [c for c in cells if 0 <= c < len(passable) and passable[c]]
        if len(passable_on_line) <= 1:
            continue
        portal_cell = passable_on_line[len(passable_on_line) // 2]
        bridge["portal"] = portal_cell
        for cell in passable_on_line:
            virtual_walls.add(cell)

    region = [-1] * (cw * ch)
    region_id = 0
    for idx, ok in enumerate(passable):
        if not ok or idx in virtual_walls or region[idx] != -1:
            continue
        region[idx] = region_id
        q = deque([idx])
        while q:
            cur = q.popleft()
            for nb in neighbors(cur, cw, ch):
                if passable[nb] and nb not in virtual_walls and region[nb] == -1:
                    region[nb] = region_id
                    q.append(nb)
        region_id += 1

    merge_virtual_wall_fragments(cw, ch, passable, region, virtual_walls)
    merge_tiny_v2_regions(cw, ch, passable, region)
    split_disconnected_regions(cw, ch, passable, region)
    bridge_list = list(bridges.values())
    resolve_ambiguous_portal_regions(cw, ch, passable, region, bridge_list)
    split_disconnected_regions(cw, ch, passable, region)
    merge_leaf_fragment_regions(cw, ch, passable, region)
    split_disconnected_regions(cw, ch, passable, region)
    filtered_bridges = classify_bridge_portals(cw, ch, passable, region, bridge_list)
    return region, obstacles, filtered_bridges, virtual_walls


def nearby_regions_for_cell(cell: int, cw: int, ch: int, passable, region):
    regs = set()
    if 0 <= cell < len(region) and passable[cell] and region[cell] >= 0:
        regs.add(region[cell])
    for nb in neighbors(cell, cw, ch):
        if passable[nb] and region[nb] >= 0:
            regs.add(region[nb])
    return regs


def region_cell_counts(region):
    counts = {}
    for rid in region:
        if rid >= 0:
            counts[rid] = counts.get(rid, 0) + 1
    return counts


def region_contact_counts(cw: int, ch: int, passable, region, source_region):
    contacts = {}
    for idx, rid in enumerate(region):
        if rid != source_region:
            continue
        for nb in neighbors(idx, cw, ch):
            nr = region[nb]
            if passable[nb] and nr >= 0 and nr != source_region:
                contacts[nr] = contacts.get(nr, 0) + 1
    return contacts


def merge_region_into(cw: int, ch: int, passable, region, source_region, target_region):
    for idx, rid in enumerate(region):
        if rid == source_region:
            region[idx] = target_region


def resolve_ambiguous_portal_regions(cw: int, ch: int, passable, region, bridges):
    for _ in range(V2_AMBIGUOUS_MERGE_PASSES):
        changed = False
        counts = region_cell_counts(region)
        for bridge in bridges:
            portal = bridge.get("portal")
            if portal is None:
                continue
            regs = nearby_regions_for_cell(portal, cw, ch, passable, region)
            if len(regs) <= 2:
                continue
            ordered = sorted(regs, key=lambda rid: counts.get(rid, 0))
            smallest = ordered[0]
            second = ordered[1]
            small_size = counts.get(smallest, 0)
            second_size = counts.get(second, 1)
            is_fragment = (
                small_size <= V2_AMBIGUOUS_MERGE_ABSOLUTE or
                small_size <= second_size * V2_AMBIGUOUS_MERGE_RATIO
            )
            if not is_fragment:
                continue
            contacts = region_contact_counts(cw, ch, passable, region, smallest)
            candidates = {rid: contacts.get(rid, 0) for rid in regs if rid != smallest}
            if not candidates:
                continue
            target = max(candidates.items(), key=lambda item: (item[1], counts.get(item[0], 0)))[0]
            merge_region_into(cw, ch, passable, region, smallest, target)
            changed = True
        if not changed:
            break


def classify_bridge_portals(cw: int, ch: int, passable, region, bridges):
    for bridge in bridges:
        portal = bridge.get("portal")
        if portal is None:
            bridge["portalRegions"] = []
            bridge["portalStatus"] = "missing"
            continue
        regs = nearby_regions_for_cell(portal, cw, ch, passable, region)
        bridge["portalRegions"] = sorted(regs)
        if len(regs) == 2:
            bridge["portalStatus"] = "ok"
        elif len(regs) > 2:
            bridge["portalStatus"] = "junction"
        else:
            bridge["portalStatus"] = "same_region"
            bridge["portal"] = None
    return bridges


def merge_virtual_wall_fragments(cw: int, ch: int, passable, region, virtual_walls):
    # Assign wall cells to a neighboring region for coloring only. The bridge
    # metadata still marks where the conceptual separator is.
    for cell in virtual_walls:
        votes = {}
        for nb in neighbors(cell, cw, ch):
            rid = region[nb]
            if passable[nb] and rid >= 0:
                votes[rid] = votes.get(rid, 0) + 1
        if votes:
            region[cell] = max(votes.items(), key=lambda kv: kv[1])[0]


def merge_tiny_v2_regions(cw: int, ch: int, passable, region):
    for _ in range(3):
        counts = {}
        for rid in region:
            if rid >= 0:
                counts[rid] = counts.get(rid, 0) + 1
        tiny = {rid for rid, count in counts.items() if count < V2_MIN_REGION_CELLS}
        if not tiny:
            return
        changed = False
        for idx, rid in enumerate(list(region)):
            if rid not in tiny:
                continue
            votes = {}
            for nb in neighbors(idx, cw, ch):
                nr = region[nb]
                if passable[nb] and nr >= 0 and nr != rid:
                    votes[nr] = votes.get(nr, 0) + 1
            if votes:
                region[idx] = max(votes.items(), key=lambda kv: kv[1])[0]
                changed = True
        if not changed:
            return


def merge_leaf_fragment_regions(cw: int, ch: int, passable, region):
    """Merge tiny final regions that have only one neighboring region.

    These are usually artifacts created by virtual separators and later
    connectedness splitting. Keeping them creates meaningless one-edge regions.
    """
    for _ in range(4):
        counts = region_cell_counts(region)
        changed = False
        for rid, count in sorted(counts.items(), key=lambda item: item[1]):
            if count > V2_MIN_REGION_CELLS:
                continue
            contacts = region_contact_counts(cw, ch, passable, region, rid)
            if len(contacts) != 1:
                continue
            target = next(iter(contacts))
            merge_region_into(cw, ch, passable, region, rid, target)
            changed = True
        if not changed:
            return


def choose_seeds(cells, clearance, target_count):
    if not cells:
        return []
    target_count = max(1, min(target_count, len(cells)))
    first = max(cells, key=lambda c: clearance[c])
    seeds = [first]
    best = {}
    for c in cells:
        best[c] = manhattan_coarse(c, first)

    while len(seeds) < target_count:
        nxt = max(cells, key=lambda c: best[c] + clearance[c] * 1.5)
        if nxt in seeds:
            break
        seeds.append(nxt)
        for c in cells:
            d = manhattan_coarse(c, nxt)
            if d < best[c]:
                best[c] = d
    return seeds


def manhattan_coarse(a: int, b: int) -> int:
    # Width cancels out for ordering poorly if unknown; this function is only
    # used after idx values are projected via global cw below.
    ax, ay = CURRENT_CW and a % CURRENT_CW, CURRENT_CW and a // CURRENT_CW
    bx, by = CURRENT_CW and b % CURRENT_CW, CURRENT_CW and b // CURRENT_CW
    return abs(ax - bx) + abs(ay - by)


CURRENT_CW = 1


def grow_regions(cw: int, ch: int, passable, comps, clearance):
    global CURRENT_CW
    CURRENT_CW = cw
    region = [-1] * (cw * ch)
    seed_region = []

    total_passable = sum(1 for ok in passable if ok)
    next_region = 0
    for cells in comps:
        target = max(1, round(TARGET_REGIONS * len(cells) / max(1, total_passable)))
        seeds = choose_seeds(cells, clearance, target)
        q = deque()
        for seed in seeds:
            rid = next_region
            next_region += 1
            region[seed] = rid
            seed_region.append(seed)
            q.append(seed)
        while q:
            cur = q.popleft()
            for nb in neighbors(cur, cw, ch):
                if passable[nb] and region[nb] == -1:
                    region[nb] = region[cur]
                    q.append(nb)

    merge_small_regions(cw, ch, passable, region)
    return region


def blocked_neighbor_count(idx: int, cw: int, ch: int, passable):
    x = idx % cw
    y = idx // cw
    count = 0
    if y == 0 or not passable[idx - cw]:
        count += 1
    if y + 1 >= ch or not passable[idx + cw]:
        count += 1
    if x == 0 or not passable[idx - 1]:
        count += 1
    if x + 1 >= cw or not passable[idx + 1]:
        count += 1
    return count


def is_bottleneck_edge(a: int, b: int, cw: int, ch: int, passable, clearance, wall_pressure):
    if not passable[a] or not passable[b]:
        return False
    low_clearance = min(clearance[a], clearance[b]) <= BOTTLENECK_CLEARANCE
    high_wall_pressure = wall_pressure[a] + wall_pressure[b] >= BOTTLENECK_WALL_PRESSURE
    return low_clearance and high_wall_pressure


def build_v1_regions(cw: int, ch: int, passable, clearance):
    _, comps = connected_passable_components(cw, ch, passable)
    global CURRENT_CW
    CURRENT_CW = cw
    region = [-1] * (cw * ch)
    total_passable = sum(1 for ok in passable if ok)
    next_region = 0
    for cells in comps:
        target = max(1, round(V1_TARGET_REGIONS * len(cells) / max(1, total_passable)))
        seeds = choose_seeds(cells, clearance, target)
        q = deque()
        for seed in seeds:
            rid = next_region
            next_region += 1
            region[seed] = rid
            q.append(seed)
        while q:
            cur = q.popleft()
            for nb in neighbors(cur, cw, ch):
                if passable[nb] and region[nb] == -1:
                    region[nb] = region[cur]
                    q.append(nb)

    cut_edges = set()
    merge_v1_small_regions(cw, ch, passable, region, cut_edges)
    split_disconnected_regions(cw, ch, passable, region)
    return region, cut_edges


def merge_v1_small_regions(cw: int, ch: int, passable, region, cut_edges):
    changed = True
    while changed:
        changed = False
        counts = {}
        for rid in region:
            if rid >= 0:
                counts[rid] = counts.get(rid, 0) + 1
        small_regions = {rid for rid, count in counts.items() if count < V1_MIN_REGION_CELLS}
        if not small_regions:
            break
        for idx, rid in enumerate(list(region)):
            if rid not in small_regions:
                continue
            votes = {}
            for nb in neighbors(idx, cw, ch):
                nr = region[nb]
                if not passable[nb] or nr < 0 or nr == rid:
                    continue
                # Small fragments should be absorbed even across weak bottlenecks.
                votes[nr] = votes.get(nr, 0) + (2 if tuple(sorted((idx, nb))) not in cut_edges else 1)
            if votes:
                region[idx] = max(votes.items(), key=lambda kv: kv[1])[0]
                changed = True


def split_disconnected_regions(cw: int, ch: int, passable, region):
    next_region = max((rid for rid in region if rid >= 0), default=-1) + 1
    visited = [False] * len(region)
    for idx, rid in enumerate(list(region)):
        if rid < 0 or visited[idx]:
            continue
        cells = []
        visited[idx] = True
        q = deque([idx])
        while q:
            cur = q.popleft()
            cells.append(cur)
            for nb in neighbors(cur, cw, ch):
                if passable[nb] and not visited[nb] and region[nb] == rid:
                    visited[nb] = True
                    q.append(nb)
        # Any later component with the same original rid gets a fresh region id.
        for later in range(idx + 1, len(region)):
            if region[later] == rid and not visited[later]:
                new_id = next_region
                next_region += 1
                visited[later] = True
                q = deque([later])
                region[later] = new_id
                while q:
                    cur = q.popleft()
                    for nb in neighbors(cur, cw, ch):
                        if passable[nb] and not visited[nb] and region[nb] == rid:
                            visited[nb] = True
                            region[nb] = new_id
                            q.append(nb)
                break


def merge_small_regions(cw: int, ch: int, passable, region):
    counts = {}
    for rid in region:
        if rid >= 0:
            counts[rid] = counts.get(rid, 0) + 1
    small = {rid for rid, count in counts.items() if count < MIN_REGION_CELLS}
    if not small:
        return
    for idx, rid in enumerate(list(region)):
        if rid not in small:
            continue
        votes = {}
        for nb in neighbors(idx, cw, ch):
            nr = region[nb]
            if passable[nb] and nr >= 0 and nr != rid:
                votes[nr] = votes.get(nr, 0) + 1
        if votes:
            region[idx] = max(votes.items(), key=lambda kv: kv[1])[0]


def extract_portals(cw: int, ch: int, passable, region, cut_edges=None):
    edges = {}
    for idx, ok in enumerate(passable):
        if not ok or region[idx] < 0:
            continue
        x = idx % cw
        y = idx // cw
        for nb in (idx + 1, idx + cw):
            if nb >= len(passable):
                continue
            if nb == idx + 1 and x + 1 >= cw:
                continue
            if nb == idx + cw and y + 1 >= ch:
                continue
            if passable[nb] and region[nb] >= 0 and region[nb] != region[idx]:
                a, b = sorted((region[idx], region[nb]))
                edges.setdefault((a, b), []).append((idx, nb))

    portals = []
    for (a, b), cells in edges.items():
        # Collapse long boundaries into a few representative portals.
        stride = max(1, len(cells) // 4)
        for idx, (c1, c2) in enumerate(cells[::stride][:4]):
            x1, y1 = c1 % cw, c1 // cw
            x2, y2 = c2 % cw, c2 // cw
            portals.append({
                "regions": [a, b],
                "x": (x1 + x2 + 1) * BLOCK / 2,
                "y": (y1 + y2 + 1) * BLOCK / 2,
            })
    return edges, portals


def color_for_region(rid: int):
    hue = (rid * 0.618033988749895) % 1.0
    sat = 0.42 + ((rid * 17) % 20) / 100.0
    val = 0.92
    r, g, b = colorsys.hsv_to_rgb(hue, sat, val)
    return int(r * 255), int(g * 255), int(b * 255)


def render(width: int, height: int, grid, cw: int, ch: int, passable, region, portals):
    base = Image.new("RGBA", (width, height), (245, 245, 240, 255))
    pix = base.load()
    for y, row in enumerate(grid):
        for x, chv in enumerate(row):
            if not is_free(chv):
                pix[x, y] = (31, 36, 45, 255)

    overlay = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    for by in range(ch):
        y0 = by * BLOCK
        y1 = min(height, y0 + BLOCK)
        for bx in range(cw):
            idx = by * cw + bx
            rid = region[idx]
            if passable[idx] and rid >= 0:
                color = color_for_region(rid)
                draw.rectangle((bx * BLOCK, y0, min(width, (bx + 1) * BLOCK), y1),
                               fill=(*color, 78))

    # Region borders.
    border = ImageDraw.Draw(overlay)
    for by in range(ch):
        for bx in range(cw):
            idx = by * cw + bx
            rid = region[idx]
            if rid < 0:
                continue
            x0, y0 = bx * BLOCK, by * BLOCK
            x1, y1 = min(width, x0 + BLOCK), min(height, y0 + BLOCK)
            if bx + 1 < cw and region[idx + 1] != rid:
                border.line((x1, y0, x1, y1), fill=(20, 20, 20, 160), width=1)
            if by + 1 < ch and region[idx + cw] != rid:
                border.line((x0, y1, x1, y1), fill=(20, 20, 20, 160), width=1)

    # Portal points.
    for p in portals:
        x, y = p["x"], p["y"]
        border.ellipse((x - 4, y - 4, x + 4, y + 4), fill=(255, 255, 255, 235), outline=(10, 10, 10, 230), width=2)

    return Image.alpha_composite(base, overlay).convert("RGB")


def render_v2(width: int, height: int, grid, cw: int, ch: int, passable, region, portals, bridges, virtual_walls):
    img = render(width, height, grid, cw, ch, passable, region, portals).convert("RGBA")
    draw = ImageDraw.Draw(img)
    for bridge in bridges:
        a = bridge["a"]
        b = bridge["b"]
        ax, ay = (a % cw + 0.5) * BLOCK, (a // cw + 0.5) * BLOCK
        bx, by = (b % cw + 0.5) * BLOCK, (b // cw + 0.5) * BLOCK
        draw.line((ax, ay, bx, by), fill=(32, 42, 54, 210), width=3)
        portal = bridge.get("portal")
        if portal is not None:
            px, py = (portal % cw + 0.5) * BLOCK, (portal // cw + 0.5) * BLOCK
            draw.ellipse((px - 7, py - 7, px + 7, py + 7), fill=(255, 255, 255, 245), outline=(180, 30, 60, 255), width=3)
    return img.convert("RGB")


def write_html(summary):
    html = f"""<!doctype html>
<html lang=\"zh-CN\">
<head>
  <meta charset=\"utf-8\" />
  <title>{summary['title']} Region Graph Prototype</title>
  <style>
    body {{ margin: 0; font-family: system-ui, sans-serif; background: #101418; color: #eef2f7; }}
    header {{ position: sticky; top: 0; z-index: 2; background: rgba(16,20,24,.92); padding: 12px 18px; border-bottom: 1px solid #2a313a; }}
    h1 {{ font-size: 18px; margin: 0 0 6px; font-weight: 650; }}
    .meta {{ color: #a9b4c0; font-size: 13px; display: flex; gap: 18px; flex-wrap: wrap; }}
    .wrap {{ padding: 18px; }}
    img {{ display: block; max-width: none; image-rendering: auto; box-shadow: 0 0 0 1px #303946; }}
    .hint {{ color: #a9b4c0; margin: 0 0 12px; font-size: 13px; }}
  </style>
</head>
<body>
  <header>
    <h1>{summary['title']} Region Graph Prototype</h1>
    <div class=\"meta\">
      <span>map: {summary['map_name']}</span>
      <span>size: {summary['width']} x {summary['height']}</span>
      <span>block: {summary['block']}</span>
      <span>regions: {summary['regions']}</span>
      <span>region edges: {summary['region_edges']}</span>
      <span>portal points: {summary['portals']}</span>
    </div>
  </header>
  <div class=\"wrap\">
    <p class=\"hint\">滚动查看完整 iron 地图。底图黑色为障碍，浅色为可通行区域，彩色为图论区域，白色小圆点为区域连接 portal 候选。</p>
    <img src=\"{summary['image']}\" width=\"{summary['width']}\" height=\"{summary['height']}\" />
  </div>
</body>
</html>
"""
    (OUT / summary["html"]).write_text(html, encoding="utf-8")


def obstacle_runs_for_canvas(grid):
    rows = []
    for row in grid:
        runs = []
        start = None
        for x, chv in enumerate(row):
            blocked = not is_free(chv)
            if blocked and start is None:
                start = x
            elif not blocked and start is not None:
                runs.append([start, x])
                start = None
        if start is not None:
            runs.append([start, len(row)])
        rows.append(runs)
    return rows


def compact_obstacles(obstacles):
    out = []
    for idx, obs in enumerate(obstacles):
        out.append({
            "id": idx,
            "sourceId": obs["id"],
            "bbox": obs["bbox"],
            "score": round(obs["score"], 2),
            "cells": len(obs["cells"]),
            "boundary": len(obs["boundary"]),
        })
    return out


def region_stats(cw, ch, region, bridges):
    stats = {}
    for idx, rid in enumerate(region):
        if rid < 0:
            continue
        item = stats.setdefault(rid, {
            "id": rid,
            "cells": 0,
            "portals": 0,
            "neighbors": set(),
            "bbox": [10**9, 10**9, -1, -1],
        })
        x = idx % cw
        y = idx // cw
        item["cells"] += 1
        item["bbox"][0] = min(item["bbox"][0], x)
        item["bbox"][1] = min(item["bbox"][1], y)
        item["bbox"][2] = max(item["bbox"][2], x)
        item["bbox"][3] = max(item["bbox"][3], y)

    for bridge in bridges:
        portal = bridge.get("portal")
        if portal is None:
            continue
        touched = set()
        for nb in neighbors(portal, cw, ch):
            rid = region[nb]
            if rid >= 0:
                touched.add(rid)
        for rid in touched:
            if rid in stats:
                stats[rid]["portals"] += 1
                stats[rid]["neighbors"].update(other for other in touched if other != rid)

    result = []
    for rid in sorted(stats):
        item = stats[rid]
        item["neighbors"] = sorted(item["neighbors"])
        result.append(item)
    return result


def adjacent_region_boundaries(cw, ch, passable, region):
    edges = {}
    for idx, rid in enumerate(region):
        if rid < 0 or not passable[idx]:
            continue
        x = idx % cw
        y = idx // cw
        for nb in (idx + 1, idx + cw):
            if nb >= len(region):
                continue
            if nb == idx + 1 and x + 1 >= cw:
                continue
            if nb == idx + cw and y + 1 >= ch:
                continue
            nr = region[nb]
            if nr >= 0 and passable[nb] and nr != rid:
                a, b = sorted((rid, nr))
                edges.setdefault((a, b), []).append((idx, nb))
    return edges


def add_adjacency_portals(cw, ch, passable, region, portal_items, portal_cells):
    covered = {
        tuple(sorted(item["regions"]))
        for item in portal_items
        if len(item.get("regions", [])) == 2
    }
    added = []
    for pair, boundary in adjacent_region_boundaries(cw, ch, passable, region).items():
        if pair in covered or not boundary:
            continue
        mid = boundary[len(boundary) // 2]
        cell = mid[0]
        item = {
            "bridge": None,
            "cell": cell,
            "status": "adjacent",
            "regions": list(pair),
        }
        portal_items.append(item)
        portal_cells.add(cell)
        added.append(item)
    return added


def block_free_cells(width, height, grid, bx, by):
    x0 = bx * BLOCK
    x1 = min(width, x0 + BLOCK)
    y0 = by * BLOCK
    y1 = min(height, y0 + BLOCK)
    cells = []
    for y in range(y0, y1):
        row = grid[y]
        for x in range(x0, x1):
            if is_free(row[x]):
                cells.append((y, x))
    return cells


def diagonal_blocks_really_connected(width, height, grid, ax, ay, bx, by):
    x0 = min(ax, bx) * BLOCK
    x1 = min(width, (max(ax, bx) + 1) * BLOCK)
    y0 = min(ay, by) * BLOCK
    y1 = min(height, (max(ay, by) + 1) * BLOCK)
    starts = set(block_free_cells(width, height, grid, ax, ay))
    targets = set(block_free_cells(width, height, grid, bx, by))
    if not starts or not targets:
        return False

    q = deque(starts)
    visited = set(starts)
    while q:
        cy, cx = q.popleft()
        if (cy, cx) in targets:
            return True
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            ny, nx = cy + dy, cx + dx
            if not (y0 <= ny < y1 and x0 <= nx < x1):
                continue
            if (ny, nx) in visited or not is_free(grid[ny][nx]):
                continue
            visited.add((ny, nx))
            q.append((ny, nx))
    return False


def add_diagonal_corner_portals(width, height, grid, cw, ch, passable, region, portal_items, portal_cells):
    """Add portals for diagonal region contacts separated by obstacle corners.

    This captures passages like:

        A  #
        #  B

    The regions are not 4-neighbor adjacent on the coarse grid, but the corner
    often represents a useful local transition in the original map geometry.
    """
    existing = {
        (tuple(sorted(item.get("regions", []))), item.get("cell"))
        for item in portal_items
        if len(item.get("regions", [])) == 2
    }
    added = []
    seen_contacts = set()
    for idx, rid in enumerate(region):
        if rid < 0 or not passable[idx]:
            continue
        x = idx % cw
        y = idx // cw
        for dx, dy in ((1, 1), (-1, 1)):
            nx = x + dx
            ny = y + dy
            if nx < 0 or ny < 0 or nx >= cw or ny >= ch:
                continue
            nb = ny * cw + nx
            nr = region[nb]
            if nr < 0 or not passable[nb] or nr == rid:
                continue
            orth_a = y * cw + nx
            orth_b = ny * cw + x
            if passable[orth_a] or passable[orth_b]:
                continue
            if not diagonal_blocks_really_connected(width, height, grid, x, y, nx, ny):
                continue
            pair = tuple(sorted((rid, nr)))
            corner_key = (pair, min(idx, nb), max(idx, nb))
            if corner_key in seen_contacts:
                continue
            seen_contacts.add(corner_key)
            cell = idx
            if (pair, cell) in existing:
                continue
            item = {
                "bridge": None,
                "cell": cell,
                "x": (x + nx + 1) * BLOCK / 2,
                "y": (y + ny + 1) * BLOCK / 2,
                "status": "diagonal",
                "regions": list(pair),
            }
            portal_items.append(item)
            portal_cells.add(cell)
            existing.add((pair, cell))
            added.append(item)
    return added


def add_blocked_connector_portals(width, height, grid, cw, ch, passable, free_ratio, region, portal_items, portal_cells):
    """Add portals carried by free space inside approximate-obstacle blocks."""
    added = []
    for idx, ok in enumerate(passable):
        if ok or free_ratio[idx] < 0.45:
            continue
        bx = idx % cw
        by = idx // cw
        x0 = bx * BLOCK
        x1 = min(width, x0 + BLOCK)
        y0 = by * BLOCK
        y1 = min(height, y0 + BLOCK)
        free_cells = []
        local_free = set()
        for y in range(y0, y1):
            row = grid[y]
            for x in range(x0, x1):
                if is_free(row[x]):
                    free_cells.append((y, x))
                    local_free.add((y, x))
        if not free_cells:
            continue

        visited = set()
        for start in free_cells:
            if start in visited:
                continue
            q = deque([start])
            visited.add(start)
            comp = []
            while q:
                cy, cx = q.popleft()
                comp.append((cy, cx))
                for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                    nxt = (cy + dy, cx + dx)
                    if nxt in local_free and nxt not in visited:
                        visited.add(nxt)
                        q.append(nxt)
            if len(comp) < 16:
                continue

            touched = {}
            for cy, cx in comp:
                checks = []
                if cy == y0 and by > 0:
                    checks.append((idx - cw, cy - 1, cx))
                if cy == y1 - 1 and by + 1 < ch:
                    checks.append((idx + cw, cy + 1, cx))
                if cx == x0 and bx > 0:
                    checks.append((idx - 1, cy, cx - 1))
                if cx == x1 - 1 and bx + 1 < cw:
                    checks.append((idx + 1, cy, cx + 1))
                for nb, gy, gx in checks:
                    rid = region[nb]
                    if passable[nb] and rid >= 0 and is_free(grid[gy][gx]):
                        touched[rid] = touched.get(rid, 0) + 1

            if len(touched) < 2:
                continue
            regs = sorted(touched)
            avg_x = sum(x for _, x in comp) / len(comp) + 0.5
            avg_y = sum(y for y, _ in comp) / len(comp) + 0.5
            item = {
                "bridge": None,
                "cell": idx,
                "x": avg_x,
                "y": avg_y,
                "status": "blocked_connector",
                "regions": regs,
            }
            portal_items.append(item)
            portal_cells.add(idx)
            added.append(item)
    return added


def region_stats_from_portals(cw, region, portal_items):
    stats = {}
    for idx, rid in enumerate(region):
        if rid < 0:
            continue
        item = stats.setdefault(rid, {
            "id": rid,
            "cells": 0,
            "portals": 0,
            "neighbors": set(),
            "bbox": [10**9, 10**9, -1, -1],
        })
        x = idx % cw
        y = idx // cw
        item["cells"] += 1
        item["bbox"][0] = min(item["bbox"][0], x)
        item["bbox"][1] = min(item["bbox"][1], y)
        item["bbox"][2] = max(item["bbox"][2], x)
        item["bbox"][3] = max(item["bbox"][3], y)

    for portal in portal_items:
        regs = [rid for rid in portal.get("regions", []) if rid in stats]
        for rid in regs:
            stats[rid]["portals"] += 1
            stats[rid]["neighbors"].update(other for other in regs if other != rid)

    result = []
    for rid in sorted(stats):
        item = stats[rid]
        item["neighbors"] = sorted(item["neighbors"])
        result.append(item)
    return result


def color_hex_for_region(rid):
    r, g, b = color_for_region(rid)
    return f"#{r:02x}{g:02x}{b:02x}"


def build_interactive_payload(width, height, grid, cw, ch, passable, free_ratio, region, bridges, virtual_walls, obstacles):
    bridge_items = []
    portal_cells = set()
    portal_items = []
    for idx, bridge in enumerate(bridges):
        portal = bridge.get("portal")
        if portal is not None:
            portal_cells.add(portal)
            portal_items.append({
                "bridge": idx,
                "cell": portal,
                "status": bridge.get("portalStatus", "unknown"),
                "regions": bridge.get("portalRegions", []),
            })
        bridge_items.append({
            "id": idx,
            "a": bridge["a"],
            "b": bridge["b"],
            "dist": bridge["dist"],
            "portal": portal,
            "obs": list(bridge["obs"]),
            "portalRegions": bridge.get("portalRegions", []),
            "portalStatus": bridge.get("portalStatus", "unknown"),
        })

    supplemental_portals = add_adjacency_portals(cw, ch, passable, region, portal_items, portal_cells)
    diagonal_portals = add_diagonal_corner_portals(width, height, grid, cw, ch, passable, region, portal_items, portal_cells)
    blocked_connector_portals = add_blocked_connector_portals(width, height, grid, cw, ch, passable, free_ratio, region, portal_items, portal_cells)
    region_ids = sorted({rid for rid in region if rid >= 0})
    return {
        "meta": {
            "map": str(MAP_PATH.relative_to(ROOT)),
            "width": width,
            "height": height,
            "block": BLOCK,
            "coarseWidth": cw,
            "coarseHeight": ch,
            "regions": len(region_ids),
            "obstacles": len(obstacles),
            "bridges": len(bridges),
            "adjacencyPortals": len(supplemental_portals),
            "diagonalPortals": len(diagonal_portals),
            "blockedConnectorPortals": len(blocked_connector_portals),
            "virtualWalls": len(virtual_walls),
            "title": MAP_KEY.upper(),
        },
        "obstacleRuns": obstacle_runs_for_canvas(grid),
        "passable": [1 if ok else 0 for ok in passable],
        "region": region,
        "regionColors": {str(rid): color_hex_for_region(rid) for rid in region_ids},
        "regionStats": region_stats_from_portals(cw, region, portal_items),
        "bridges": bridge_items,
        "virtualWalls": sorted(virtual_walls),
        "portalCells": sorted(portal_cells),
        "portalItems": portal_items,
        "obstacles": compact_obstacles(obstacles),
    }


def write_interactive_html(payload):
    data_json = json.dumps(payload, separators=(",", ":"))
    html = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>{payload['meta']['title']} Interactive Region Graph</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #101418;
      --panel: rgba(18, 23, 29, .94);
      --line: #2b3440;
      --text: #eef2f7;
      --muted: #a8b3bf;
      --accent: #ef476f;
    }}
    html, body {{ margin: 0; width: 100%; height: 100%; overflow: hidden; background: var(--bg); color: var(--text); font-family: system-ui, -apple-system, Segoe UI, sans-serif; }}
    canvas {{ display: block; width: 100vw; height: 100vh; cursor: grab; }}
    canvas.dragging {{ cursor: grabbing; }}
    .panel {{ position: fixed; z-index: 3; background: var(--panel); border: 1px solid var(--line); box-shadow: 0 10px 30px rgba(0,0,0,.28); backdrop-filter: blur(8px); }}
    #toolbar {{ left: 14px; top: 14px; padding: 12px 14px; border-radius: 8px; width: 310px; }}
    #info {{ right: 14px; top: 14px; padding: 12px 14px; border-radius: 8px; width: 330px; max-height: calc(100vh - 48px); overflow: auto; }}
    #route-comparison {{ left: 14px; bottom: 14px; width: 410px; padding: 12px; border-radius: 12px; }}
    h1 {{ font-size: 16px; margin: 0 0 8px; }}
    .meta, .hint, .row {{ color: var(--muted); font-size: 12px; line-height: 1.45; }}
    .toggles {{ display: grid; grid-template-columns: 1fr 1fr; gap: 7px 10px; margin: 10px 0; }}
    label {{ font-size: 13px; color: var(--text); user-select: none; }}
    input {{ vertical-align: -2px; }}
    button {{ background: #1d2630; color: var(--text); border: 1px solid #344150; border-radius: 6px; padding: 6px 8px; margin-right: 6px; }}
    button:hover {{ border-color: #617286; }}
    .kv {{ display: grid; grid-template-columns: 116px 1fr; gap: 3px 8px; font-size: 12px; margin-top: 8px; }}
    .kv div:nth-child(odd) {{ color: var(--muted); }}
    .badge {{ display: inline-block; padding: 2px 6px; border-radius: 999px; background: #202a35; color: var(--muted); margin-right: 4px; }}
    #legend {{ display: flex; flex-wrap: wrap; gap: 6px; margin-top: 8px; }}
    .swatch {{ width: 12px; height: 12px; display: inline-block; border-radius: 3px; margin-right: 4px; vertical-align: -1px; }}
    .route-head {{ display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 9px; }}
    .route-head strong {{ font-size: 13px; }}
    .route-actions {{ display: flex; align-items: center; gap: 10px; }}
    .route-actions label {{ color: var(--muted); font-size: 12px; }}
    .route-actions button {{ margin: 0; padding: 4px 8px; font-size: 12px; }}
    .route-cards {{ display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }}
    .route-card {{ min-height: 82px; padding: 10px; border: 1px solid #354250; border-radius: 9px; background: rgba(11, 16, 21, .76); }}
    .route-card--global {{ border-top: 3px solid #38bdf8; }}
    .route-card--portal {{ border-top: 3px solid #fbbf24; }}
    .route-card__title {{ color: var(--muted); font-size: 11px; letter-spacing: .04em; text-transform: uppercase; }}
    .route-card__value {{ margin: 4px 0 3px; color: #f8fafc; font: 700 22px/1.1 ui-monospace, SFMono-Regular, Consolas, monospace; }}
    .route-card__meta {{ color: var(--muted); font: 11px/1.45 ui-monospace, SFMono-Regular, Consolas, monospace; }}
    .route-card__error {{ color: #fda4af; font-size: 12px; line-height: 1.45; margin-top: 8px; }}
    .route-delta {{ min-height: 16px; margin-top: 8px; color: var(--muted); font: 11px/1.4 ui-monospace, SFMono-Regular, Consolas, monospace; }}
    @media (max-width: 980px) {{
      #route-comparison {{ width: 350px; }}
      #info {{ width: 280px; }}
    }}
  </style>
</head>
<body>
  <canvas id="view"></canvas>
  <section id="toolbar" class="panel">
    <h1>{payload['meta']['title']} Region Graph</h1>
    <div class="meta" id="meta"></div>
    <div class="toggles">
      <label><input type="checkbox" id="layerLaser" checked> 激光底图</label>
      <label><input type="checkbox" id="layerBase" checked> 真实障碍</label>
      <label><input type="checkbox" id="layerCoarseObstacles" checked> 近似障碍格</label>
      <label><input type="checkbox" id="layerRegions"> 区域颜色</label>
      <label><input type="checkbox" id="layerBorders" checked> 区域边界</label>
      <label><input type="checkbox" id="layerBridges" checked> 障碍连线</label>
      <label><input type="checkbox" id="layerPortals" checked> Portal 点</label>
      <label><input type="checkbox" id="layerWalls"> 划分线格</label>
      <label><input type="checkbox" id="layerObstacles"> 大障碍 bbox</label>
      <label><input type="checkbox" id="layerGrid"> coarse 网格</label>
    </div>
    <button id="reset">重置视图</button>
    <button id="fit">适配全图</button>
    <div class="hint">Ctrl+点击设置起点，Shift+点击设置终点。蓝线为全局粗格 A*，黄线为 Region/Portal 分层估计。</div>
  </section>
  <section id="info" class="panel">
    <h1>当前查看</h1>
    <div id="hover" class="row">移动鼠标查看区域信息。</div>
    <div class="kv" id="details"></div>
  </section>
  <section id="route-comparison" class="panel">
    <div class="route-head">
      <strong>粗格路径对比</strong>
      <div class="route-actions">
        <label><input type="checkbox" id="toggle-global-route" checked> 全局</label>
        <label><input type="checkbox" id="toggle-portal-route" checked> Portal</label>
        <button type="button" id="clear-route">清除</button>
      </div>
    </div>
    <div class="route-cards">
      <article class="route-card route-card--global" id="global-route-card"></article>
      <article class="route-card route-card--portal" id="portal-route-card"></article>
    </div>
    <div class="route-delta" id="route-delta">Ctrl+点击起点，Shift+点击终点。</div>
  </section>
  <script id="region-data" type="application/json">{data_json}</script>
  <script>
    const data = JSON.parse(document.getElementById('region-data').textContent);
    const canvas = document.getElementById('view');
    const ctx = canvas.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const meta = data.meta;
    const regionStats = new Map(data.regionStats.map(r => [r.id, r]));
    const portalSet = new Set(data.portalCells);
    const portalByCell = new Map();
    data.portalItems.forEach((p, index) => {{
      p.index = index;
      const list = portalByCell.get(p.cell) || [];
      list.push(p);
      portalByCell.set(p.cell, list);
    }});
    const wallSet = new Set(data.virtualWalls);
    const toggles = Object.fromEntries(['Laser','Base','CoarseObstacles','Regions','Borders','Bridges','Portals','Walls','Obstacles','Grid'].map(name => [name.toLowerCase(), document.getElementById('layer' + name)]));
    let scale = 0.42;
    let offsetX = 28;
    let offsetY = 24;
    let dragging = false;
    let lastX = 0;
    let lastY = 0;
    let hoverCell = -1;
    let selectedRegion = -1;
    let routeStart = null;
    let routeGoal = null;
    let globalRouteResult = null;
    let portalRouteResult = null;
    const portalDistanceCache = new Map();
    const showGlobalRoute = document.getElementById('toggle-global-route');
    const showPortalRoute = document.getElementById('toggle-portal-route');

    document.getElementById('meta').innerHTML = `${{meta.width}} x ${{meta.height}} cells · block ${{meta.block}} · regions ${{meta.regions}} · bridges ${{meta.bridges}} · adjacent portals ${{meta.adjacencyPortals}} · diagonal portals ${{meta.diagonalPortals}} · blocked connectors ${{meta.blockedConnectorPortals}}`;
    document.getElementById('reset').onclick = () => {{ scale = 0.42; offsetX = 28; offsetY = 24; selectedRegion = -1; clearRoutes(); draw(); }};
    document.getElementById('fit').onclick = fit;
    document.getElementById('clear-route').onclick = () => {{ clearRoutes(); draw(); }};
    showGlobalRoute.addEventListener('change', draw);
    showPortalRoute.addEventListener('change', draw);
    Object.values(toggles).forEach(el => el.addEventListener('change', draw));

    function clearRoutes() {{
      routeStart = null;
      routeGoal = null;
      globalRouteResult = null;
      portalRouteResult = null;
      updateRouteComparison(null, null);
    }}

    function resize() {{
      canvas.width = Math.floor(innerWidth * dpr);
      canvas.height = Math.floor(innerHeight * dpr);
      canvas.style.width = innerWidth + 'px';
      canvas.style.height = innerHeight + 'px';
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw();
    }}

    function fit() {{
      scale = Math.min(innerWidth / meta.width, innerHeight / meta.height) * 0.94;
      offsetX = (innerWidth - meta.width * scale) / 2;
      offsetY = (innerHeight - meta.height * scale) / 2;
      draw();
    }}

    function worldFromScreen(x, y) {{
      return {{ x: (x - offsetX) / scale, y: (y - offsetY) / scale }};
    }}

    function coarseIndexFromWorld(wx, wy) {{
      const bx = Math.floor(wx / meta.block);
      const by = Math.floor(wy / meta.block);
      if (bx < 0 || by < 0 || bx >= meta.coarseWidth || by >= meta.coarseHeight) return -1;
      return by * meta.coarseWidth + bx;
    }}

    canvas.addEventListener('mousedown', ev => {{
      dragging = true;
      lastX = ev.clientX;
      lastY = ev.clientY;
      canvas.classList.add('dragging');
    }});
    addEventListener('mouseup', () => {{ dragging = false; canvas.classList.remove('dragging'); }});
    canvas.addEventListener('mousemove', ev => {{
      if (dragging) {{
        offsetX += ev.clientX - lastX;
        offsetY += ev.clientY - lastY;
        lastX = ev.clientX;
        lastY = ev.clientY;
        draw();
        return;
      }}
      const w = worldFromScreen(ev.clientX, ev.clientY);
      hoverCell = coarseIndexFromWorld(w.x, w.y);
      updateInfo(w);
      draw();
    }});
    canvas.addEventListener('click', ev => {{
      const w = worldFromScreen(ev.clientX, ev.clientY);
      const idx = coarseIndexFromWorld(w.x, w.y);
      if (idx >= 0 && ev.ctrlKey) {{
        routeStart = {{ cell: idx, point: {{ x: w.x, y: w.y }} }};
        updateRoutes();
        updateInfo(w);
        draw();
        return;
      }}
      if (idx >= 0 && ev.shiftKey) {{
        routeGoal = {{ cell: idx, point: {{ x: w.x, y: w.y }} }};
        updateRoutes();
        updateInfo(w);
        draw();
        return;
      }}
      const rid = idx >= 0 ? data.region[idx] : -1;
      selectedRegion = selectedRegion === rid ? -1 : rid;
      updateInfo(w);
      draw();
    }});
    canvas.addEventListener('wheel', ev => {{
      ev.preventDefault();
      const before = worldFromScreen(ev.clientX, ev.clientY);
      const factor = ev.deltaY < 0 ? 1.18 : 1 / 1.18;
      scale = Math.max(0.12, Math.min(12, scale * factor));
      offsetX = ev.clientX - before.x * scale;
      offsetY = ev.clientY - before.y * scale;
      draw();
    }}, {{ passive: false }});

    function draw() {{
      ctx.clearRect(0, 0, innerWidth, innerHeight);
      ctx.save();
      ctx.translate(offsetX, offsetY);
      ctx.scale(scale, scale);
      ctx.fillStyle = toggles.laser.checked ? '#f7f8f5' : '#f5f5f0';
      ctx.fillRect(0, 0, meta.width, meta.height);

      if (toggles.regions.checked) drawRegions();
      if (toggles.base.checked) drawBaseObstacles();
      if (toggles.coarseobstacles.checked) drawCoarseObstacles();
      if (toggles.borders.checked) drawRegionBorders();
      if (toggles.walls.checked) drawVirtualWalls();
      if (toggles.bridges.checked) drawBridges();
      if (toggles.portals.checked) drawPortals();
      drawRoute();
      if (toggles.obstacles.checked) drawObstacleBoxes();
      if (toggles.grid.checked) drawGrid();
      drawHover();
      ctx.restore();
    }}

    function drawBaseObstacles() {{
      ctx.fillStyle = '#1f242d';
      if (toggles.laser.checked) ctx.fillStyle = '#101820';
      for (let y = 0; y < data.obstacleRuns.length; y++) {{
        for (const [x0, x1] of data.obstacleRuns[y]) ctx.fillRect(x0, y, x1 - x0, 1);
      }}
    }}

    function drawCoarseObstacles() {{
      ctx.save();
      ctx.globalAlpha = toggles.laser.checked ? 0.40 : 0.30;
      ctx.fillStyle = '#000000';
      for (let by = 0; by < meta.coarseHeight; by++) {{
        for (let bx = 0; bx < meta.coarseWidth; bx++) {{
          const idx = by * meta.coarseWidth + bx;
          if (data.passable[idx]) continue;
          ctx.fillRect(bx * meta.block, by * meta.block, meta.block, meta.block);
        }}
      }}
      ctx.globalAlpha = 1;
      ctx.fillStyle = '#ffffff';
      ctx.strokeStyle = '#000000';
      ctx.lineWidth = Math.max(1, 1 / scale);
      ctx.font = `${{Math.max(9, Math.min(18, meta.block * 0.82))}}px ui-monospace, SFMono-Regular, Consolas, monospace`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      for (let by = 0; by < meta.coarseHeight; by++) {{
        for (let bx = 0; bx < meta.coarseWidth; bx++) {{
          const idx = by * meta.coarseWidth + bx;
          if (data.passable[idx]) continue;
          const x = (bx + 0.5) * meta.block;
          const y = (by + 0.55) * meta.block;
          ctx.strokeText('*', x, y);
          ctx.fillText('*', x, y);
        }}
      }}
      ctx.restore();
    }}

    function drawRegions() {{
      ctx.globalAlpha = toggles.laser.checked ? 0.24 : 0.45;
      for (let by = 0; by < meta.coarseHeight; by++) {{
        for (let bx = 0; bx < meta.coarseWidth; bx++) {{
          const idx = by * meta.coarseWidth + bx;
          const rid = data.region[idx];
          if (rid < 0) continue;
          ctx.fillStyle = data.regionColors[rid] || '#cccccc';
          ctx.fillRect(bx * meta.block, by * meta.block, meta.block, meta.block);
        }}
      }}
      ctx.globalAlpha = 1;
      if (selectedRegion >= 0) {{
        ctx.globalAlpha = 0.36;
        ctx.fillStyle = '#ffffff';
        for (let i = 0; i < data.region.length; i++) {{
          if (data.region[i] !== selectedRegion && data.region[i] >= 0) {{
            ctx.fillRect((i % meta.coarseWidth) * meta.block, Math.floor(i / meta.coarseWidth) * meta.block, meta.block, meta.block);
          }}
        }}
        ctx.globalAlpha = 1;
      }}
    }}

    function drawRegionBorders() {{
      ctx.strokeStyle = toggles.laser.checked ? 'rgba(12,16,22,.78)' : 'rgba(12,16,22,.62)';
      ctx.lineWidth = Math.max(1, 1 / scale);
      ctx.beginPath();
      for (let by = 0; by < meta.coarseHeight; by++) {{
        for (let bx = 0; bx < meta.coarseWidth; bx++) {{
          const idx = by * meta.coarseWidth + bx;
          const rid = data.region[idx];
          if (rid < 0) continue;
          const x = bx * meta.block, y = by * meta.block;
          if (bx + 1 < meta.coarseWidth && data.region[idx + 1] !== rid) {{ ctx.moveTo(x + meta.block, y); ctx.lineTo(x + meta.block, y + meta.block); }}
          if (by + 1 < meta.coarseHeight && data.region[idx + meta.coarseWidth] !== rid) {{ ctx.moveTo(x, y + meta.block); ctx.lineTo(x + meta.block, y + meta.block); }}
        }}
      }}
      ctx.stroke();
    }}

    function cellCenter(idx) {{
      return {{ x: (idx % meta.coarseWidth + 0.5) * meta.block, y: (Math.floor(idx / meta.coarseWidth) + 0.5) * meta.block }};
    }}

    function portalPoint(portal) {{
      if (Number.isFinite(portal.x) && Number.isFinite(portal.y)) return {{ x: portal.x, y: portal.y }};
      return cellCenter(portal.cell);
    }}

    const portalsByRegion = new Map();
    data.portalItems.forEach((portal, idx) => {{
      for (const rid of portal.regions || []) {{
        const list = portalsByRegion.get(rid) || [];
        list.push(idx);
        portalsByRegion.set(rid, list);
      }}
    }});

    function pointDistance(a, b) {{
      return (Math.abs(a.x - b.x) + Math.abs(a.y - b.y)) / meta.block;
    }}

    const blockedFine = new Uint8Array(meta.width * meta.height);
    for (let y = 0; y < data.obstacleRuns.length; y++) {{
      for (const [x0, x1] of data.obstacleRuns[y]) {{
        blockedFine.fill(1, y * meta.width + x0, y * meta.width + x1);
      }}
    }}

    function fineIndex(x, y) {{
      return y * meta.width + x;
    }}

    function isFreeFine(x, y) {{
      return x >= 0 && y >= 0 && x < meta.width && y < meta.height && !blockedFine[fineIndex(x, y)];
    }}

    function nearestFreeFine(point, maxRadius = 48) {{
      let sx = Math.max(0, Math.min(meta.width - 1, Math.round(point.x)));
      let sy = Math.max(0, Math.min(meta.height - 1, Math.round(point.y)));
      if (isFreeFine(sx, sy)) return {{ x: sx, y: sy }};
      const visited = new Set([fineIndex(sx, sy)]);
      const q = [{{ x: sx, y: sy, d: 0 }}];
      for (let head = 0; head < q.length; head++) {{
        const cur = q[head];
        if (cur.d >= maxRadius) continue;
        for (const [dx, dy] of [[1,0],[-1,0],[0,1],[0,-1]]) {{
          const nx = cur.x + dx, ny = cur.y + dy;
          if (nx < 0 || ny < 0 || nx >= meta.width || ny >= meta.height) continue;
          const key = fineIndex(nx, ny);
          if (visited.has(key)) continue;
          if (isFreeFine(nx, ny)) return {{ x: nx, y: ny }};
          visited.add(key);
          q.push({{ x: nx, y: ny, d: cur.d + 1 }});
        }}
      }}
      return {{ x: sx, y: sy }};
    }}

    class MinHeap {{
      constructor() {{ this.items = []; }}
      push(item) {{
        this.items.push(item);
        for (let i = this.items.length - 1; i > 0;) {{
          const p = (i - 1) >> 1;
          if (this.items[p].f <= item.f) break;
          this.items[i] = this.items[p];
          i = p;
        }}
        this.items[this.items.length - 1] = item;
      }}
      pop() {{
        if (!this.items.length) return null;
        const out = this.items[0];
        const item = this.items.pop();
        if (this.items.length && item) {{
          let i = 0;
          while (true) {{
            let c = i * 2 + 1;
            if (c >= this.items.length) break;
            if (c + 1 < this.items.length && this.items[c + 1].f < this.items[c].f) c++;
            if (this.items[c].f >= item.f) break;
            this.items[i] = this.items[c];
            i = c;
          }}
          this.items[i] = item;
        }}
        return out;
      }}
      get length() {{ return this.items.length; }}
    }}

    function finePosFromState(key) {{
      const cell = Math.floor(key / 4);
      const dir = key - cell * 4;
      const y = Math.floor(cell / meta.width);
      const x = cell - y * meta.width;
      return {{ x, y, dir }};
    }}

    function reconstructFinePath(parent, goalKey) {{
      const out = [];
      let cur = goalKey;
      while (cur !== undefined) {{
        const pos = finePosFromState(cur);
        out.push({{ x: pos.x, y: pos.y, dir: pos.dir }});
        cur = parent.get(cur);
      }}
      out.reverse();
      return out;
    }}

    function simplifyFinePath(path) {{
      if (path.length <= 2) return path;
      const out = [path[0]];
      let lastDx = Math.sign(path[1].x - path[0].x);
      let lastDy = Math.sign(path[1].y - path[0].y);
      for (let i = 2; i < path.length; i++) {{
        const dx = Math.sign(path[i].x - path[i - 1].x);
        const dy = Math.sign(path[i].y - path[i - 1].y);
        if (dx !== lastDx || dy !== lastDy) {{
          out.push(path[i - 1]);
          lastDx = dx;
          lastDy = dy;
        }}
      }}
      out.push(path[path.length - 1]);
      return out;
    }}

    function astarFineSegment(a, b, startDir = null) {{
      const start = nearestFreeFine(a);
      const goal = nearestFreeFine(b);
      const startCell = fineIndex(start.x, start.y);
      const goalCell = fineIndex(goal.x, goal.y);
      if (startCell === goalCell) return {{ path: [{{ ...start, dir: startDir ?? 0 }}], endDir: startDir ?? 0, cost: 0 }};
      const dirs = [[0,-1],[1,0],[0,1],[-1,0]];

      for (const margin of [96, 192, 384, Math.max(meta.width, meta.height)]) {{
        const minX = Math.max(0, Math.min(start.x, goal.x) - margin);
        const maxX = Math.min(meta.width - 1, Math.max(start.x, goal.x) + margin);
        const minY = Math.max(0, Math.min(start.y, goal.y) - margin);
        const maxY = Math.min(meta.height - 1, Math.max(start.y, goal.y) + margin);
        const open = new MinHeap();
        const gScore = new Map();
        const parent = new Map();
        const closed = new Set();
        const startDirs = startDir === null ? [0, 1, 2, 3] : [startDir];
        for (const dir of startDirs) {{
          const key = startCell * 4 + dir;
          gScore.set(key, 0);
          parent.set(key, undefined);
          open.push({{ key, x: start.x, y: start.y, dir, f: Math.abs(start.x - goal.x) + Math.abs(start.y - goal.y) }});
        }}
        let expanded = 0;
        const maxExpanded = margin > 1000 ? 260000 : 90000;
        while (open.length && expanded++ < maxExpanded) {{
          const cur = open.pop();
          if (!cur || closed.has(cur.key)) continue;
          const curCell = Math.floor(cur.key / 4);
          if (curCell === goalCell) {{
            const path = simplifyFinePath(reconstructFinePath(parent, cur.key));
            return {{ path, endDir: cur.dir, cost: gScore.get(cur.key) ?? Infinity }};
          }}
          closed.add(cur.key);
          const baseG = gScore.get(cur.key) ?? Infinity;

          for (const ndir of [(cur.dir + 1) % 4, (cur.dir + 3) % 4]) {{
            const nk = curCell * 4 + ndir;
            const ng = baseG + 1;
            if (ng < (gScore.get(nk) ?? Infinity)) {{
              gScore.set(nk, ng);
              parent.set(nk, cur.key);
              open.push({{ key: nk, x: cur.x, y: cur.y, dir: ndir, f: ng + Math.abs(cur.x - goal.x) + Math.abs(cur.y - goal.y) }});
            }}
          }}

          const [dx, dy] = dirs[cur.dir];
          const nx = cur.x + dx, ny = cur.y + dy;
          if (nx < minX || nx > maxX || ny < minY || ny > maxY || !isFreeFine(nx, ny)) continue;
          const nk = fineIndex(nx, ny) * 4 + cur.dir;
          if (closed.has(nk)) continue;
          const ng = baseG + 1;
          if (ng < (gScore.get(nk) ?? Infinity)) {{
            gScore.set(nk, ng);
            parent.set(nk, cur.key);
            open.push({{ key: nk, x: nx, y: ny, dir: cur.dir, f: ng + Math.abs(nx - goal.x) + Math.abs(ny - goal.y) }});
          }}
        }}
      }}
      return {{ path: [a, b], endDir: startDir ?? 0, cost: Infinity }};
    }}

    function buildObstacleAwareRoute(result) {{
      if (!result || !result.ok || result.points.length < 2) return null;
      const out = [];
      let dir = null;
      for (let i = 1; i < result.points.length; i++) {{
        const seg = astarFineSegment(result.points[i - 1], result.points[i], dir);
        if (!seg.path.length) continue;
        dir = seg.endDir;
        const path = seg.path.map(p => ({{ x: p.x, y: p.y }}));
        if (out.length) path.shift();
        out.push(...path);
      }}
      return out.length ? out : result.points;
    }}

    function coarseNeighbors(cell) {{
      const x = cell % meta.coarseWidth;
      const y = Math.floor(cell / meta.coarseWidth);
      const out = [];
      if (x > 0) out.push(cell - 1);
      if (x + 1 < meta.coarseWidth) out.push(cell + 1);
      if (y > 0) out.push(cell - meta.coarseWidth);
      if (y + 1 < meta.coarseHeight) out.push(cell + meta.coarseWidth);
      return out;
    }}

    function coarseManhattan(a, b) {{
      return Math.abs(a % meta.coarseWidth - b % meta.coarseWidth)
        + Math.abs(Math.floor(a / meta.coarseWidth) - Math.floor(b / meta.coarseWidth));
    }}

    function reconstructCoarsePath(parent, goal) {{
      const cells = [];
      let cur = goal;
      while (cur !== undefined) {{
        cells.push(cur);
        cur = parent.get(cur);
      }}
      cells.reverse();
      return cells;
    }}

    function runCoarseAStar(startCell, goalCell, allowed) {{
      const started = performance.now();
      if (startCell < 0 || goalCell < 0) return {{ ok: false, cells: [], reason: '端点不在粗网格内', expanded: 0, runtimeMs: performance.now() - started }};
      const open = [startCell];
      const openSet = new Set(open);
      const closed = new Set();
      const g = new Map([[startCell, 0]]);
      const f = new Map([[startCell, coarseManhattan(startCell, goalCell)]]);
      const parent = new Map();
      const limit = meta.coarseWidth * meta.coarseHeight * 4;
      let expanded = 0;
      while (open.length && expanded < limit) {{
        let bestAt = 0;
        for (let i = 1; i < open.length; i++) {{
          if ((f.get(open[i]) ?? Infinity) < (f.get(open[bestAt]) ?? Infinity)) bestAt = i;
        }}
        const current = open.splice(bestAt, 1)[0];
        openSet.delete(current);
        if (closed.has(current)) continue;
        if (current === goalCell) {{
          const cells = reconstructCoarsePath(parent, current);
          const coarseCost = g.get(current) ?? 0;
          return {{ ok: true, cells, coarseCost, estimatedCost: coarseCost * meta.block, expanded, runtimeMs: performance.now() - started }};
        }}
        closed.add(current);
        expanded++;
        for (const next of coarseNeighbors(current)) {{
          if (closed.has(next) || !allowed(next, current)) continue;
          const nextG = (g.get(current) ?? Infinity) + 1;
          if (nextG >= (g.get(next) ?? Infinity)) continue;
          parent.set(next, current);
          g.set(next, nextG);
          f.set(next, nextG + coarseManhattan(next, goalCell));
          if (!openSet.has(next)) {{
            open.push(next);
            openSet.add(next);
          }}
        }}
      }}
      return {{ ok: false, cells: [], reason: open.length ? '搜索达到扩展上限' : '粗网格不可达', expanded, runtimeMs: performance.now() - started }};
    }}

    function runGlobalCoarseAStar(startCell, goalCell) {{
      return runCoarseAStar(startCell, goalCell, next => data.passable[next] || portalByCell.has(next) || next === goalCell);
    }}

    function runRegionRestrictedAStar(startCell, goalCell, regionId) {{
      return runCoarseAStar(startCell, goalCell, next =>
        data.region[next] === regionId || portalByCell.has(next) || next === startCell || next === goalCell
      );
    }}

    function portalCacheKey(regionId, portalA, portalB) {{
      const lo = Math.min(portalA, portalB);
      const hi = Math.max(portalA, portalB);
      return `${{regionId}}|${{lo}}|${{hi}}`;
    }}

    function reverseSegment(result) {{
      return {{ ...result, cells: [...result.cells].reverse() }};
    }}

    function getPortalSegment(regionId, portalA, portalB, stats) {{
      const key = portalCacheKey(regionId, portalA, portalB);
      const cached = portalDistanceCache.get(key);
      if (cached) {{
        stats.cacheHits++;
        return portalA <= portalB ? cached : reverseSegment(cached);
      }}
      stats.cacheMisses++;
      const lo = Math.min(portalA, portalB);
      const hi = Math.max(portalA, portalB);
      const result = runRegionRestrictedAStar(data.portalItems[lo].cell, data.portalItems[hi].cell, regionId);
      stats.localExpanded += result.expanded || 0;
      if (result.ok) portalDistanceCache.set(key, result);
      return portalA <= portalB ? result : reverseSegment(result);
    }}

    function appendCells(target, segment) {{
      for (const cell of segment || []) {{
        if (!target.length || target[target.length - 1] !== cell) target.push(cell);
      }}
    }}

    function stateKey(portalIndex, regionId) {{
      return portalIndex + '|' + regionId;
    }}

    function parseState(key) {{
      const parts = key.split('|');
      return {{ portal: Number(parts[0]), region: Number(parts[1]) }};
    }}

    function resolveByFineBfs(point, maxExpanded = 90000) {{
      const start = nearestFreeFine(point, 96);
      const startKey = fineIndex(start.x, start.y);
      const visited = new Set([startKey]);
      const q = [{{ x: start.x, y: start.y, dist: 0 }}];
      const found = [];
      let foundDist = Infinity;
      for (let head = 0; head < q.length && head < maxExpanded; head++) {{
        const cur = q[head];
        if (cur.dist > foundDist) break;
        const cell = coarseIndexFromWorld(cur.x, cur.y);
        if (cell >= 0) {{
          const rid = data.region[cell];
          if (rid >= 0) {{
            found.push({{ region: rid, cost: cur.dist / meta.block, point: {{ x: cur.x, y: cur.y }}, cell }});
            foundDist = cur.dist;
          }}
          const portals = portalByCell.get(cell) || [];
          for (const portal of portals) {{
            for (const pr of portal.regions || []) {{
              found.push({{ region: pr, cost: cur.dist / meta.block, point: portalPoint(portal), cell: portal.cell }});
              foundDist = cur.dist;
            }}
          }}
        }}
        if (found.length) continue;
        for (const [dx, dy] of [[1,0],[-1,0],[0,1],[0,-1]]) {{
          const nx = cur.x + dx, ny = cur.y + dy;
          if (!isFreeFine(nx, ny)) continue;
          const key = fineIndex(nx, ny);
          if (visited.has(key)) continue;
          visited.add(key);
          q.push({{ x: nx, y: ny, dist: cur.dist + 1 }});
        }}
      }}
      const dedup = new Map();
      for (const item of found) {{
        const old = dedup.get(item.region);
        if (!old || item.cost < old.cost) dedup.set(item.region, item);
      }}
      return [...dedup.values()];
    }}

    function resolveCellRegions(cell, point) {{
      const direct = data.region[cell];
      if (direct >= 0) return [{{ region: direct, cost: 0, point, cell }}];
      const herePortals = portalByCell.get(cell) || [];
      const fromPortal = [];
      for (const portal of herePortals) {{
        for (const rid of portal.regions || []) fromPortal.push({{ region: rid, cost: 0, point: portalPoint(portal), cell: portal.cell }});
      }}
      if (fromPortal.length) return fromPortal;
      return resolveByFineBfs(point);
    }}

    function popBest(open, dist) {{
      let bestAt = 0;
      for (let i = 1; i < open.length; i++) {{
        if ((dist.get(open[i]) ?? Infinity) < (dist.get(open[bestAt]) ?? Infinity)) bestAt = i;
      }}
      return open.splice(bestAt, 1)[0];
    }}

    function portalGoalHeuristic(portalIndex, goalBinding) {{
      return coarseManhattan(data.portalItems[portalIndex].cell, goalBinding.cell) + goalBinding.cost;
    }}

    function endpointPortalCandidates(binding) {{
      return (portalsByRegion.get(binding.region) || [])
        .map(portal => ({{ portal, rank: coarseManhattan(binding.cell, data.portalItems[portal].cell) }}))
        .sort((a, b) => a.rank - b.rank)
        .map(item => item.portal);
    }}

    function reconstructHierarchicalCells(parent, bestKey, finalSegment) {{
      const pieces = [];
      let cur = bestKey;
      while (cur) {{
        const edge = parent.get(cur);
        if (!edge) break;
        pieces.push(edge.cells || []);
        cur = edge.prev;
      }}
      pieces.reverse();
      const cells = [];
      for (const piece of pieces) appendCells(cells, piece);
      appendCells(cells, finalSegment || []);
      return cells;
    }}

    function runPortalAStar(startBinding, goalBinding) {{
      const started = performance.now();
      const stats = {{ cacheHits: 0, cacheMisses: 0, localExpanded: 0 }};
      if (startBinding.region === goalBinding.region) {{
        const direct = runRegionRestrictedAStar(startBinding.cell, goalBinding.cell, startBinding.region);
        return {{
          ...direct,
          portalIndices: [],
          graphExpanded: 0,
          localExpanded: direct.expanded || 0,
          cacheHits: 0,
          cacheMisses: 0,
          cacheSize: portalDistanceCache.size,
          coarseCost: direct.ok ? direct.coarseCost + startBinding.cost + goalBinding.cost : Infinity,
          estimatedCost: direct.ok ? (direct.coarseCost + startBinding.cost + goalBinding.cost) * meta.block : Infinity,
          runtimeMs: performance.now() - started,
        }};
      }}

      const dist = new Map();
      const priority = new Map();
      const parent = new Map();
      const open = [];
      for (const portalIndex of endpointPortalCandidates(startBinding)) {{
        const portal = data.portalItems[portalIndex];
        const segment = runRegionRestrictedAStar(startBinding.cell, portal.cell, startBinding.region);
        stats.localExpanded += segment.expanded || 0;
        if (!segment.ok) continue;
        const key = stateKey(portalIndex, startBinding.region);
        const cost = startBinding.cost + segment.coarseCost;
        if (cost < (dist.get(key) ?? Infinity)) {{
          dist.set(key, cost);
          priority.set(key, cost + portalGoalHeuristic(portalIndex, goalBinding));
          parent.set(key, {{ prev: null, cells: segment.cells }});
          open.push(key);
        }}
      }}
      if (!open.length) return {{ ok: false, cells: [], reason: '起点区域没有可达 Portal', graphExpanded: 0, ...stats, cacheSize: portalDistanceCache.size, runtimeMs: performance.now() - started }};

      let best = null;
      let graphExpanded = 0;
      const closed = new Set();
      while (open.length) {{
        const key = popBest(open, priority);
        if (closed.has(key)) continue;
        if (best && (priority.get(key) ?? Infinity) >= best.cost) break;
        closed.add(key);
        graphExpanded++;
        const cur = parseState(key);
        const portal = data.portalItems[cur.portal];
        const baseCost = dist.get(key);

        if (cur.region === goalBinding.region) {{
          const terminal = runRegionRestrictedAStar(portal.cell, goalBinding.cell, cur.region);
          stats.localExpanded += terminal.expanded || 0;
          if (terminal.ok) {{
            const total = baseCost + terminal.coarseCost + goalBinding.cost;
            if (!best || total < best.cost) best = {{ cost: total, key, terminal }};
          }}
        }}

        for (const nextRegion of portal.regions || []) {{
          if (nextRegion === cur.region) continue;
          const nextKey = stateKey(cur.portal, nextRegion);
          const nextCost = baseCost + 1;
          if (nextCost < (dist.get(nextKey) ?? Infinity)) {{
            dist.set(nextKey, nextCost);
            priority.set(nextKey, nextCost + portalGoalHeuristic(cur.portal, goalBinding));
            parent.set(nextKey, {{ prev: key, cells: [portal.cell] }});
            open.push(nextKey);
          }}
        }}

        for (const nextPortal of portalsByRegion.get(cur.region) || []) {{
          if (nextPortal === cur.portal) continue;
          const segment = getPortalSegment(cur.region, cur.portal, nextPortal, stats);
          if (!segment.ok) continue;
          const nextKey = stateKey(nextPortal, cur.region);
          const nextCost = baseCost + segment.coarseCost;
          if (nextCost < (dist.get(nextKey) ?? Infinity)) {{
            dist.set(nextKey, nextCost);
            priority.set(nextKey, nextCost + portalGoalHeuristic(nextPortal, goalBinding));
            parent.set(nextKey, {{ prev: key, cells: segment.cells }});
            open.push(nextKey);
          }}
        }}
      }}
      if (!best) return {{ ok: false, cells: [], reason: 'Region/Portal 图不可达', graphExpanded, ...stats, cacheSize: portalDistanceCache.size, runtimeMs: performance.now() - started }};
      const cells = reconstructHierarchicalCells(parent, best.key, best.terminal.cells);
      const portalIndices = [];
      let cur = best.key;
      while (cur) {{
        const portal = parseState(cur).portal;
        if (!portalIndices.includes(portal)) portalIndices.push(portal);
        cur = parent.get(cur)?.prev;
      }}
      portalIndices.reverse();
      return {{
        ok: true,
        cells,
        portalIndices,
        coarseCost: best.cost,
        estimatedCost: best.cost * meta.block,
        graphExpanded,
        localExpanded: stats.localExpanded,
        cacheHits: stats.cacheHits,
        cacheMisses: stats.cacheMisses,
        cacheSize: portalDistanceCache.size,
        runtimeMs: performance.now() - started,
      }};
    }}

    function chooseBestGlobal(starts, goals) {{
      let best = null;
      for (const start of starts) {{
        for (const goal of goals) {{
          const result = runGlobalCoarseAStar(start.cell, goal.cell);
          if (!result.ok) {{
            if (!best) best = result;
            continue;
          }}
          const coarseCost = result.coarseCost + start.cost + goal.cost;
          const candidate = {{ ...result, coarseCost, estimatedCost: coarseCost * meta.block }};
          if (!best || !best.ok || candidate.coarseCost < best.coarseCost) best = candidate;
        }}
      }}
      return best || {{ ok: false, cells: [], reason: '起点或终点无法绑定到粗网格' }};
    }}

    function chooseBestPortal(starts, goals) {{
      let best = null;
      for (const start of starts) {{
        for (const goal of goals) {{
          const candidate = runPortalAStar(start, goal);
          if (candidate.ok && (!best || !best.ok || candidate.coarseCost < best.coarseCost)) best = candidate;
          else if (!best) best = candidate;
        }}
      }}
      return best || {{ ok: false, cells: [], reason: '起点或终点无法绑定到 Region' }};
    }}

    function updateRoutes() {{
      if (!routeStart || !routeGoal) {{
        globalRouteResult = null;
        portalRouteResult = null;
        updateRouteComparison(null, null);
        return;
      }}
      const starts = resolveCellRegions(routeStart.cell, routeStart.point);
      const goals = resolveCellRegions(routeGoal.cell, routeGoal.point);
      const globalStarted = performance.now();
      globalRouteResult = chooseBestGlobal(starts, goals);
      globalRouteResult.totalRuntimeMs = performance.now() - globalStarted;
      const portalStarted = performance.now();
      portalRouteResult = chooseBestPortal(starts, goals);
      portalRouteResult.totalRuntimeMs = performance.now() - portalStarted;
      updateRouteComparison(globalRouteResult, portalRouteResult);
    }}

    function drawMarker(point, color, label) {{
      if (!point) return;
      ctx.beginPath();
      ctx.fillStyle = color;
      ctx.strokeStyle = '#101418';
      ctx.lineWidth = Math.max(2, 3 / scale);
      ctx.arc(point.x, point.y, Math.max(6, 8 / scale), 0, Math.PI * 2);
      ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#101418';
      ctx.font = `${{Math.max(8, 10 / scale)}}px ui-monospace, SFMono-Regular, Consolas, monospace`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(label, point.x, point.y + 0.5 / scale);
    }}

    function drawCoarseRoute(cells, color, width) {{
      if (!cells || cells.length < 2) return;
      const points = cells.map(cellCenter);
      ctx.save();
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      for (const stroke of [{{ color: 'rgba(5, 10, 15, .82)', width: width + 4 }}, {{ color, width }}]) {{
        ctx.strokeStyle = stroke.color;
        ctx.lineWidth = Math.max(stroke.width, stroke.width / scale);
        ctx.beginPath();
        points.forEach((p, index) => index ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y));
        ctx.stroke();
      }}
      ctx.restore();
    }}

    function drawRoute() {{
      if (showGlobalRoute.checked && globalRouteResult?.ok) drawCoarseRoute(globalRouteResult.cells, '#38bdf8', 3);
      if (showPortalRoute.checked && portalRouteResult?.ok) drawCoarseRoute(portalRouteResult.cells, '#fbbf24', 4);
      if (showPortalRoute.checked && portalRouteResult?.ok) {{
        for (const portalIndex of portalRouteResult.portalIndices || []) {{
          const p = portalPoint(data.portalItems[portalIndex]);
          ctx.beginPath();
          ctx.fillStyle = '#fff7d6';
          ctx.strokeStyle = '#f59e0b';
          ctx.lineWidth = Math.max(2, 3 / scale);
          ctx.arc(p.x, p.y, Math.max(5, 7 / scale), 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
        }}
      }}
      drawMarker(routeStart && routeStart.point, '#06d6a0', 'S');
      drawMarker(routeGoal && routeGoal.point, '#ef476f', 'T');
    }}

    function routeSummaryHtml() {{
      if (!routeStart && !routeGoal) return '未设置';
      if (!globalRouteResult && !portalRouteResult) return '等待起点/终点';
      const g = globalRouteResult?.ok ? Math.round(globalRouteResult.estimatedCost) : '不可达';
      const p = portalRouteResult?.ok ? Math.round(portalRouteResult.estimatedCost) : '不可达';
      return `全局 ${{g}} · Portal ${{p}}`;
    }}

    function routeCardHtml(kind, result) {{
      const title = kind === 'global' ? 'Global coarse A*' : 'Portal A*';
      if (!routeStart || !routeGoal) return `<div class="route-card__title">${{title}}</div><div class="route-card__meta">等待设置两个端点</div>`;
      if (!result?.ok) return `<div class="route-card__title">${{title}}</div><div class="route-card__error">${{result?.reason || '不可达'}}</div>`;
      if (kind === 'global') {{
        return `<div class="route-card__title">${{title}}</div>
          <div class="route-card__value">${{Math.round(result.estimatedCost)}}</div>
          <div class="route-card__meta">${{result.coarseCost.toFixed(1)}} coarse · ${{result.expanded}} expanded<br>总耗时 ${{result.totalRuntimeMs.toFixed(2)}} ms</div>`;
      }}
      return `<div class="route-card__title">${{title}}</div>
        <div class="route-card__value">${{Math.round(result.estimatedCost)}}</div>
        <div class="route-card__meta">${{result.portalIndices.length}} portals · ${{result.graphExpanded}} graph<br>${{result.localExpanded}} local · cache ${{result.cacheHits}}↑ ${{result.cacheMisses}}↓ (${{result.cacheSize}})<br>总耗时 ${{result.totalRuntimeMs.toFixed(2)}} ms</div>`;
    }}

    function updateRouteComparison(globalResult, portalResult) {{
      document.getElementById('global-route-card').innerHTML = routeCardHtml('global', globalResult);
      document.getElementById('portal-route-card').innerHTML = routeCardHtml('portal', portalResult);
      const delta = document.getElementById('route-delta');
      if (globalResult?.ok && portalResult?.ok) {{
        const absolute = portalResult.estimatedCost - globalResult.estimatedCost;
        const percent = globalResult.estimatedCost ? absolute / globalResult.estimatedCost * 100 : 0;
        delta.textContent = `Portal 相对全局：${{absolute >= 0 ? '+' : ''}}${{Math.round(absolute)}}（${{percent >= 0 ? '+' : ''}}${{percent.toFixed(1)}}%）`;
      }} else {{
        delta.textContent = routeStart && routeGoal ? '两种算法独立计算；失败不会隐藏另一条路径。' : 'Ctrl+点击起点，Shift+点击终点。';
      }}
    }}

    function drawBridges() {{
      ctx.strokeStyle = toggles.laser.checked ? 'rgba(21,31,42,.72)' : 'rgba(30,42,54,.86)';
      ctx.lineWidth = Math.max(2, 3 / scale);
      ctx.beginPath();
      for (const b of data.bridges) {{
        const a = cellCenter(b.a), c = cellCenter(b.b);
        ctx.moveTo(a.x, a.y); ctx.lineTo(c.x, c.y);
      }}
      ctx.stroke();
    }}

    function drawPortals() {{
      for (const portal of data.portalItems) {{
        const p = portalPoint(portal);
        const junction = portal.status === 'junction';
        const adjacent = portal.status === 'adjacent';
        const diagonal = portal.status === 'diagonal';
        const blockedConnector = portal.status === 'blocked_connector';
        ctx.beginPath();
        ctx.fillStyle = junction ? '#ffd166' : (blockedConnector ? '#80ed99' : (diagonal ? '#c77dff' : (adjacent ? '#8ecae6' : '#fff')));
        ctx.strokeStyle = junction ? '#7a4f00' : (blockedConnector ? '#1b7f3a' : (diagonal ? '#5a189a' : (adjacent ? '#005f73' : '#d90429')));
        ctx.lineWidth = Math.max(2, 3 / scale);
        ctx.arc(p.x, p.y, junction ? Math.max(6, 7 / scale) : Math.max(5, 6 / scale), 0, Math.PI * 2);
        ctx.fill(); ctx.stroke();
        if (junction) {{
          ctx.fillStyle = '#3b2600';
          ctx.font = `${{Math.max(8, 9 / scale)}}px ui-monospace, SFMono-Regular, Consolas, monospace`;
          ctx.textAlign = 'center';
          ctx.textBaseline = 'middle';
          ctx.fillText('J', p.x, p.y + 0.5 / scale);
        }}
      }}
    }}

    function drawVirtualWalls() {{
      ctx.fillStyle = toggles.laser.checked ? 'rgba(239,71,111,.20)' : 'rgba(239,71,111,.28)';
      for (const idx of data.virtualWalls) {{
        ctx.fillRect((idx % meta.coarseWidth) * meta.block, Math.floor(idx / meta.coarseWidth) * meta.block, meta.block, meta.block);
      }}
    }}

    function drawObstacleBoxes() {{
      ctx.strokeStyle = '#ffd166';
      ctx.lineWidth = Math.max(1, 2 / scale);
      for (const ob of data.obstacles) {{
        const [x0, y0, x1, y1] = ob.bbox;
        ctx.strokeRect(x0 * meta.block, y0 * meta.block, (x1 - x0 + 1) * meta.block, (y1 - y0 + 1) * meta.block);
      }}
    }}

    function drawGrid() {{
      ctx.strokeStyle = 'rgba(40,50,62,.28)';
      ctx.lineWidth = Math.max(.5, .75 / scale);
      ctx.beginPath();
      for (let x = 0; x <= meta.coarseWidth; x++) {{ ctx.moveTo(x * meta.block, 0); ctx.lineTo(x * meta.block, meta.height); }}
      for (let y = 0; y <= meta.coarseHeight; y++) {{ ctx.moveTo(0, y * meta.block); ctx.lineTo(meta.width, y * meta.block); }}
      ctx.stroke();
    }}

    function drawHover() {{
      const idx = selectedRegion >= 0 ? -1 : hoverCell;
      if (idx < 0) return;
      const x = (idx % meta.coarseWidth) * meta.block;
      const y = Math.floor(idx / meta.coarseWidth) * meta.block;
      ctx.strokeStyle = '#00f5d4';
      ctx.lineWidth = Math.max(2, 2 / scale);
      ctx.strokeRect(x, y, meta.block, meta.block);
    }}

    function updateInfo(w) {{
      const idx = coarseIndexFromWorld(w.x, w.y);
      const hover = document.getElementById('hover');
      const details = document.getElementById('details');
      if (idx < 0) {{
        hover.textContent = '鼠标在地图外。';
        details.innerHTML = '';
        return;
      }}
      const bx = idx % meta.coarseWidth, by = Math.floor(idx / meta.coarseWidth);
      const rid = data.region[idx];
      const tags = [];
      if (!data.passable[idx]) tags.push('障碍 coarse cell');
      if (wallSet.has(idx)) tags.push('虚拟墙');
      const portalsHere = portalByCell.get(idx) || [];
      if (portalsHere.length) tags.push(portalsHere.map(p => p.status === 'junction' ? 'junction portal' : 'portal').join(' / '));
      hover.innerHTML = `<span class="badge">grid ${{Math.floor(w.x)}}, ${{Math.floor(w.y)}}</span><span class="badge">block ${{bx}}, ${{by}}</span><span class="badge">region ${{rid}}</span>`;
      const stat = regionStats.get(rid);
      const selected = selectedRegion >= 0 ? regionStats.get(selectedRegion) : null;
      const shown = selected || stat;
      details.innerHTML = `
        <div>当前标签</div><div>${{tags.length ? tags.join(' · ') : 'free space'}}</div>
        <div>锁定区域</div><div>${{selectedRegion >= 0 ? selectedRegion : '未锁定，点击区域可锁定'}}</div>
        <div>区域面积</div><div>${{shown ? shown.cells + ' coarse cells' : '-'}}</div>
        <div>区域 bbox</div><div>${{shown ? shown.bbox.join(', ') : '-'}}</div>
        <div>邻接区域</div><div>${{shown && shown.neighbors.length ? shown.neighbors.join(', ') : '-'}}</div>
        <div>portal 数</div><div>${{shown ? shown.portals : '-'}}</div>
        <div>此格 portal</div><div>${{portalsHere.length ? portalsHere.map(p => `${{p.status}} [${{p.regions.join(',')}}]`).join(' · ') : '-'}}</div>
        <div>快速路径</div><div>${{routeSummaryHtml()}}</div>
        <div>zoom</div><div>${{scale.toFixed(2)}}x</div>
      `;
    }}

    addEventListener('resize', resize);
    updateRouteComparison(null, null);
    resize();
    fit();
  </script>
</body>
</html>
"""
    (OUT / f"{PREFIX}.html").write_text(html, encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description="Render interactive region graph prototypes for large MAPF maps.")
    parser.add_argument("--map-key", choices=sorted(MAP_CONFIGS), default="iron")
    parser.add_argument("--map-path", default=None, help="Optional .map path relative to repo root or absolute path.")
    parser.add_argument("--prefix", default=None, help="Output file prefix. Defaults to '<map-key>_regions'.")
    parser.add_argument("--static-images", action="store_true", help="Also render static PNG prototypes.")
    return parser.parse_args()


def configure_from_args(args):
    global MAP_PATH, MAP_KEY, PREFIX
    MAP_KEY = args.map_key
    if args.map_path:
        path = Path(args.map_path)
        MAP_PATH = path if path.is_absolute() else ROOT / path
    else:
        MAP_PATH = MAP_CONFIGS[args.map_key]
    PREFIX = args.prefix or f"{MAP_KEY}_regions"


def main():
    args = parse_args()
    configure_from_args(args)
    OUT.mkdir(parents=True, exist_ok=True)
    width, height, grid = read_map(MAP_PATH)
    cw, ch, passable, free_ratio = build_coarse_graph(width, height, grid)
    clearance = distance_to_blocked(cw, ch, passable)
    _, comps = connected_passable_components(cw, ch, passable)
    if args.static_images or MAP_KEY == "iron":
        region_v0 = grow_regions(cw, ch, passable, comps, clearance)
        edges_v0, portals_v0 = extract_portals(cw, ch, passable, region_v0)
        image_v0 = render(width, height, grid, cw, ch, passable, region_v0, portals_v0)
        image_v0.save(OUT / f"{PREFIX}_v0.png", optimize=True)
    else:
        region_v0 = []
        edges_v0 = {}
        portals_v0 = []

    region, cut_edges = build_v1_regions(cw, ch, passable, clearance)
    edges, portals = extract_portals(cw, ch, passable, region, cut_edges)
    if args.static_images or MAP_KEY == "iron":
        image = render(width, height, grid, cw, ch, passable, region, portals)
        image.save(OUT / f"{PREFIX}_v1.png", optimize=True)
        if MAP_KEY == "iron" and PREFIX == "iron_regions":
            # Keep the historical stable name pointing at the newest v1 prototype.
            image.save(OUT / "iron_regions.png", optimize=True)

    region_v2, obstacles_v2, bridges_v2, virtual_walls_v2 = build_v2_obstacle_partitions(cw, ch, passable)
    edges_v2, portals_v2 = extract_portals(cw, ch, passable, region_v2)
    valid_bridge_portals_v2 = sum(1 for bridge in bridges_v2 if bridge.get("portal") is not None)
    junction_bridge_portals_v2 = sum(1 for bridge in bridges_v2 if bridge.get("portalStatus") == "junction")
    same_region_bridge_portals_v2 = sum(1 for bridge in bridges_v2 if bridge.get("portalStatus") == "same_region")
    if args.static_images or MAP_KEY == "iron":
        image_v2 = render_v2(width, height, grid, cw, ch, passable, region_v2, portals_v2, bridges_v2, virtual_walls_v2)
        image_v2.save(OUT / f"{PREFIX}_v2.png", optimize=True)

    region_ids = sorted({rid for rid in region if rid >= 0})
    counts = {}
    for rid in region:
        if rid >= 0:
            counts[rid] = counts.get(rid, 0) + 1
    summary = {
        "width": width,
        "height": height,
        "block": BLOCK,
        "coarse_width": cw,
        "coarse_height": ch,
        "passable_coarse_cells": sum(1 for ok in passable if ok),
        "regions": len(region_ids),
        "region_edges": len(edges),
        "portals": len(portals),
        "cut_edges": len(cut_edges),
        "min_region_cells": min(counts.values()) if counts else 0,
        "max_region_cells": max(counts.values()) if counts else 0,
        "map": str(MAP_PATH.relative_to(ROOT)),
        "map_name": MAP_PATH.name,
        "title": MAP_KEY.upper(),
        "image": f"{PREFIX}_v1.png",
        "html": f"{PREFIX}_v1.html",
        "algorithm": "coarse passability graph + clearance-weighted low-count region growth + connectedness guard + portal extraction",
        "v0": {
            "regions": len({rid for rid in region_v0 if rid >= 0}),
            "region_edges": len(edges_v0),
            "portals": len(portals_v0),
            "image": f"{PREFIX}_v0.png",
        },
        "v2": {
            "regions": len({rid for rid in region_v2 if rid >= 0}),
            "region_edges": len(edges_v2),
            "portals": len(portals_v2),
            "obstacles": len(obstacles_v2),
            "bridges": len(bridges_v2),
            "valid_bridge_portals": valid_bridge_portals_v2,
            "junction_bridge_portals": junction_bridge_portals_v2,
            "same_region_bridge_portals": same_region_bridge_portals_v2,
            "virtual_wall_cells": len(virtual_walls_v2),
            "image": f"{PREFIX}_v2.png",
        },
    }
    (OUT / f"{PREFIX}_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if MAP_KEY == "iron" and PREFIX == "iron_regions":
        (OUT / "iron_regions_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if args.static_images:
        write_html(summary)
    if args.static_images and MAP_KEY == "iron" and PREFIX == "iron_regions":
        legacy = dict(summary)
        legacy["html"] = "iron_regions.html"
        legacy["image"] = "iron_regions.png"
        write_html(legacy)
    v2_summary = dict(summary)
    v2_summary["html"] = f"{PREFIX}_v2.html"
    v2_summary["image"] = f"{PREFIX}_v2.png"
    v2_summary["algorithm"] = "obstacle components + nearest obstacle boundary bridges + virtual separators + portal extraction"
    v2_summary["regions"] = summary["v2"]["regions"]
    v2_summary["region_edges"] = summary["v2"]["region_edges"]
    v2_summary["portals"] = summary["v2"]["portals"]
    if args.static_images:
        write_html(v2_summary)
    payload = build_interactive_payload(width, height, grid, cw, ch, passable, free_ratio, region_v2, bridges_v2, virtual_walls_v2, obstacles_v2)
    payload_json = json.dumps(payload, separators=(",", ":"))
    (OUT / f"{PREFIX}_data.json").write_text(payload_json, encoding="utf-8")
    write_interactive_html(payload)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
