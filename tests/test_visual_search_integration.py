from types import SimpleNamespace
import unittest
from unittest.mock import patch

from echorescue.visual_search_integration import _wait_until_frontier_selected


class FrontierReadinessTests(unittest.TestCase):
    def test_waits_for_a_mapped_active_frontier(self) -> None:
        with patch(
            "echorescue.visual_search_integration.subprocess.run",
            return_value=SimpleNamespace(
                returncode=0,
                stdout="map_revision: 12\nroute_generation: 2\nhas_selected_frontier: true\n",
            ),
        ) as run:
            self.assertTrue(_wait_until_frontier_selected(125.0))
        command = run.call_args.args[0]
        self.assertIn("--no-daemon", command)
        self.assertIn(
            "m.map_revision > 0 and m.route_generation > 1 and m.has_selected_frontier",
            command,
        )
        self.assertEqual(run.call_args.kwargs["timeout"], 130.0)

    def test_rejects_initializing_state_without_a_selected_frontier(self) -> None:
        with patch(
            "echorescue.visual_search_integration.subprocess.run",
            return_value=SimpleNamespace(
                returncode=0,
                stdout="map_revision: 12\nroute_generation: 1\nhas_selected_frontier: false\n",
            ),
        ):
            self.assertFalse(_wait_until_frontier_selected(125.0))


if __name__ == "__main__":
    unittest.main()
