from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent
HTML = ROOT / "iron_regions.html"


class CoarsePathHtmlTests(unittest.TestCase):
    def test_dual_path_controls_exist(self):
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
            with self.subTest(marker=marker):
                self.assertIn(marker, text)

    def test_pathfinding_functions_exist(self):
        text = HTML.read_text(encoding="utf-8")
        required = [
            "function runGlobalCoarseAStar(",
            "function runRegionRestrictedAStar(",
            "function runPortalAStar(",
            "function updateRouteComparison(",
            "const portalDistanceCache = new Map()",
        ]
        for marker in required:
            with self.subTest(marker=marker):
                self.assertIn(marker, text)

    def test_both_cards_show_total_algorithm_runtime(self):
        text = HTML.read_text(encoding="utf-8")
        required = [
            "globalRouteResult.totalRuntimeMs",
            "portalRouteResult.totalRuntimeMs",
            "result.totalRuntimeMs.toFixed(2)",
            "总耗时",
        ]
        for marker in required:
            with self.subTest(marker=marker):
                self.assertIn(marker, text)

    def test_portal_route_uses_goal_directed_astar(self):
        text = HTML.read_text(encoding="utf-8")
        self.assertIn("function runPortalAStar(", text)
        self.assertIn("function endpointPortalCandidates(binding)", text)
        self.assertIn("function portalGoalHeuristic(", text)
        self.assertIn("Portal A*", text)
        self.assertNotIn(".slice(0, limit)", text)


if __name__ == "__main__":
    unittest.main()
