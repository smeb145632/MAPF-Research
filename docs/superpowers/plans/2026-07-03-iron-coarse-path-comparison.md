# Iron Coarse Path Comparison Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the Iron region visualizer with comparable global coarse-grid A* and cached Region/Portal hierarchical path estimates.

**Architecture:** Keep `iron_regions_v7.html` self-contained and generate it from `render_iron_regions.py`. Add pure browser-side pathfinding functions over the embedded coarse-grid payload, render both results as independent canvas layers, and expose compact metrics cards for visual validation.

**Tech Stack:** Python 3 generator, embedded vanilla JavaScript, Canvas 2D, browser Performance API

---

## File Structure

- Modify `benchmark_results/iron_region_graph/render_iron_regions.py`: generate the new controls, pathfinding functions, metrics, and canvas rendering.
- Regenerate `benchmark_results/iron_region_graph/iron_regions_v7.html`: self-contained visual verifier.
- Create `benchmark_results/iron_region_graph/test_coarse_path_html.py`: generator/output smoke tests and payload invariants.

`benchmark_results/` is intentionally locally ignored, so these implementation
files remain local validation artifacts and are not committed.

### Task 1: Lock Output Contract With Smoke Tests

**Files:**
- Create: `benchmark_results/iron_region_graph/test_coarse_path_html.py`
- Test: `benchmark_results/iron_region_graph/iron_regions_v7.html`

- [ ] **Step 1: Add failing output-contract tests**

```python
from pathlib import Path


ROOT = Path(__file__).resolve().parent
HTML = ROOT / "iron_regions_v7.html"


def test_dual_path_controls_exist():
    text = HTML.read_text(encoding="utf-8")
    required = [
        'id="global-route-card"',
        'id="portal-route-card"',
        'id="route-comparison"',
        'id="clear-route"',
        'id="toggle-global-route"',
        'id="toggle-portal-route"',
    ]
    for marker in required:
        assert marker in text


def test_pathfinding_functions_exist():
    text = HTML.read_text(encoding="utf-8")
    required = [
        "function runGlobalCoarseAStar(",
        "function runRegionRestrictedAStar(",
        "function runPortalHierarchicalSearch(",
        "function updateRouteComparison(",
        "const portalDistanceCache = new Map()",
    ]
    for marker in required:
        assert marker in text
```

- [ ] **Step 2: Run tests and verify the contract is initially absent**

Run:

```powershell
python -m pytest benchmark_results/iron_region_graph/test_coarse_path_html.py -q
```

Expected: both tests fail because the comparison UI and functions do not exist.

### Task 2: Add Global Coarse-Grid A*

**Files:**
- Modify: `benchmark_results/iron_region_graph/render_iron_regions.py`
- Regenerate: `benchmark_results/iron_region_graph/iron_regions_v7.html`
- Test: `benchmark_results/iron_region_graph/test_coarse_path_html.py`

- [ ] **Step 1: Add reusable coarse-grid helpers**

Add browser-side functions with these exact contracts:

```javascript
function coarseNeighbors(cell) {
  const x = cell % meta.coarseWidth;
  const y = Math.floor(cell / meta.coarseWidth);
  const out = [];
  if (x > 0) out.push(cell - 1);
  if (x + 1 < meta.coarseWidth) out.push(cell + 1);
  if (y > 0) out.push(cell - meta.coarseWidth);
  if (y + 1 < meta.coarseHeight) out.push(cell + meta.coarseWidth);
  return out;
}

function coarseManhattan(a, b) {
  return Math.abs(a % meta.coarseWidth - b % meta.coarseWidth)
    + Math.abs(Math.floor(a / meta.coarseWidth) - Math.floor(b / meta.coarseWidth));
}
```

- [ ] **Step 2: Implement global A***

Implement:

```javascript
function runGlobalCoarseAStar(startCell, goalCell)
```

It must:

- search cardinal neighbors where `data.passable[cell]` is true;
- use unit edge cost and `coarseManhattan` as the heuristic;
- reconstruct an ordered coarse-cell path;
- return `{ok, cells, coarseCost, estimatedCost, expanded, runtimeMs, reason}`;
- compute `estimatedCost = coarseCost * meta.block`;
- stop at `meta.coarseWidth * meta.coarseHeight * 4` expansions;
- report a reason instead of returning a direct fallback line.

- [ ] **Step 3: Regenerate the HTML**

Run:

```powershell
python benchmark_results/iron_region_graph/render_iron_regions.py
```

Expected: `iron_regions_v7.html` is rewritten successfully.

### Task 3: Add Cached Region/Portal Search

**Files:**
- Modify: `benchmark_results/iron_region_graph/render_iron_regions.py`
- Regenerate: `benchmark_results/iron_region_graph/iron_regions_v7.html`

- [ ] **Step 1: Build Portal indexes**

Retain `portalsByRegion` and add:

```javascript
const portalDistanceCache = new Map();

function portalCacheKey(regionId, portalA, portalB) {
  const lo = Math.min(portalA, portalB);
  const hi = Math.max(portalA, portalB);
  return `${regionId}|${lo}|${hi}`;
}
```

- [ ] **Step 2: Add Region-restricted A***

Implement:

```javascript
function runRegionRestrictedAStar(startCell, goalCell, regionId)
```

It uses the same cardinal coarse-grid movement but accepts a neighbor only when:

```javascript
data.region[next] === regionId
  || portalByCell.has(next)
  || next === startCell
  || next === goalCell
```

It returns the same path metrics as the global A* result.

- [ ] **Step 3: Cache Portal-to-Portal segments**

Implement:

```javascript
function getPortalSegment(regionId, portalA, portalB, stats)
```

On a hit, increment `stats.cacheHits`. On a miss, run Region-restricted A*,
store both cost and cells, increment `stats.cacheMisses` and add its expansion
count to `stats.localExpanded`.

- [ ] **Step 4: Implement hierarchical search**

Implement:

```javascript
function runPortalHierarchicalSearch(startBinding, goalBinding)
```

Requirements:

- state key is `portalIndex + "|" + regionId`;
- use at most four nearest reachable endpoint Portals per binding;
- crossing a Portal changes Region and costs one coarse step;
- moving between Portals in one Region uses `getPortalSegment`;
- reconstruct Portal states and concatenate cached coarse-cell segments;
- return `{ok, cells, portalIndices, coarseCost, estimatedCost,
  graphExpanded, localExpanded, cacheHits, cacheMisses, cacheSize,
  runtimeMs, reason}`.

- [ ] **Step 5: Preserve independent failures**

If one algorithm fails, retain and render the successful result from the other.
Do not use a straight-line fallback as a successful path.

### Task 4: Add Polished Comparison UI

**Files:**
- Modify: `benchmark_results/iron_region_graph/render_iron_regions.py`
- Regenerate: `benchmark_results/iron_region_graph/iron_regions_v7.html`

- [ ] **Step 1: Add compact comparison cards**

Add:

```html
<section class="route-comparison" id="route-comparison">
  <article class="route-card route-card--global" id="global-route-card"></article>
  <article class="route-card route-card--portal" id="portal-route-card"></article>
  <div class="route-delta" id="route-delta"></div>
</section>
```

Cards show estimated length as the primary value and expansions, runtime,
Portals, and cache statistics as secondary values.

- [ ] **Step 2: Add route controls**

Add visible controls for:

```html
<input type="checkbox" id="toggle-global-route" checked>
<input type="checkbox" id="toggle-portal-route" checked>
<button type="button" id="clear-route">Clear route</button>
```

- [ ] **Step 3: Apply restrained visual styling**

Use:

- global A*: `#38bdf8`, 3px route with dark 6px under-stroke;
- hierarchical route: `#fbbf24`, 4px route with dark 8px under-stroke;
- source: `#34d399`;
- target: `#fb7185`;
- cards: translucent dark surfaces, 12px radius, subtle border;
- tabular numeric metrics and no viewport-obscuring modal.

- [ ] **Step 4: Render both routes**

Add:

```javascript
function drawCoarseRoute(cells, color, width)
```

Draw cell-center polylines, used Portal rings, source, and target after map
layers but before hover details.

- [ ] **Step 5: Update metrics**

Implement:

```javascript
function updateRouteComparison(globalResult, portalResult)
```

It must render success and failure independently and show absolute and
percentage distance differences only when both methods succeed.

### Task 5: Verify Generation, Behavior, and Layout

**Files:**
- Test: `benchmark_results/iron_region_graph/test_coarse_path_html.py`
- Verify: `benchmark_results/iron_region_graph/iron_regions_v7.html`

- [ ] **Step 1: Run output-contract tests**

Run:

```powershell
python -m pytest benchmark_results/iron_region_graph/test_coarse_path_html.py -q
```

Expected: all tests pass.

- [ ] **Step 2: Start the local server**

Run from the repository root:

```powershell
python -m http.server 8080
```

Open:

```text
http://localhost:8080/benchmark_results/iron_region_graph/iron_regions_v7.html
```

- [ ] **Step 3: Exercise required scenarios**

Verify:

- same-Region endpoints;
- distant Regions;
- blocked-connector endpoint;
- diagonal Portal path;
- repeated query with cache hits;
- independent failure cards;
- route visibility toggles and clear button;
- pan, zoom, hover, and Region layers after route creation.

- [ ] **Step 4: Inspect desktop layout**

Check at approximately 1440×900 and 1920×1080. The comparison panel must not
cover the central map, text must not overflow cards, and both paths must remain
legible over Region colors.

- [ ] **Step 5: Run final static checks**

Run:

```powershell
git diff --check
python -m pytest benchmark_results/iron_region_graph/test_coarse_path_html.py -q
```

Expected: no whitespace errors and all tests pass.

