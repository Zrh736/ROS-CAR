"""Checks for the adapter that invokes the author's original executable."""

import unittest
from subprocess import CompletedProcess
from unittest.mock import patch

from delivery_sim.author_original import parse_runner_output, run_author_planners, runner_command


class AuthorRunnerTests(unittest.TestCase):
    def test_both_planners_use_exactly_the_same_author_instance(self):
        sequential = runner_command("Maze", 2, 7, "Sequential", "data/Results/seq.txt")
        mgmm = runner_command("Maze", 2, 7, "Dromos", "data/Results/mgmm.txt")
        self.assertEqual(sequential[2:7], mgmm[2:7])
        self.assertEqual(sequential[13:], mgmm[13:])
        self.assertEqual(sequential[7:9], ["UseMP", "Sequential"])
        self.assertEqual(mgmm[7:9], ["UseMP", "Dromos"])

    def test_parse_real_runner_summary_and_reject_missing_metrics(self):
        output = "[solved            = 1]\n[TimeSolve = 0.087]\n[SolDistance = 102.96]"
        self.assertEqual(parse_runner_output(output),
                         {"solved": 1, "planning_seconds": 0.087, "distance": 102.96})
        with self.assertRaises(ValueError):
            parse_runner_output("[solved = 1]")

    @patch("delivery_sim.author_original.subprocess.run")
    def test_adapter_passes_author_planners_to_wsl_without_python_fallback(self, run):
        summary = "author-commit\n[solved = 1]\n[TimeSolve = 0.087]\n[SolDistance = 102.96]"
        run.side_effect = [CompletedProcess([], 0, "Ubuntu-20.04", ""),
                           CompletedProcess([], 0, summary, ""),
                           CompletedProcess([], 0, summary, "")]
        report = run_author_planners("Maze", 2, 0, "Ubuntu-20.04", "MGMM_ICRA2025", 180)
        self.assertEqual(report["author_commit"], "author-commit")
        self.assertEqual(set(report["runs"]), {"Sequential", "Dromos"})
        self.assertIn("./bin/Runner", run.call_args.args[0])
        self.assertEqual(report["runs"]["Dromos"]["metrics"]["distance"], 102.96)


if __name__ == "__main__":
    unittest.main()
