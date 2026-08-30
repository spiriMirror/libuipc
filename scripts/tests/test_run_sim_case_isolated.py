from __future__ import annotations

import unittest
from pathlib import Path

from scripts.run_sim_case_isolated import manifest_payload, select_cases


class SimCaseShardTests(unittest.TestCase):
    def test_shards_are_sorted_disjoint_and_complete(self) -> None:
        cases = ["case_4", "case_1", "case_3", "case_0", "case_2"]
        shards = [
            select_cases(cases, shard_count=3, shard_index=index)
            for index in range(3)
        ]

        self.assertEqual(shards[0], ["case_0", "case_3"])
        self.assertEqual(shards[1], ["case_1", "case_4"])
        self.assertEqual(shards[2], ["case_2"])
        self.assertEqual(
            sorted(case for shard in shards for case in shard), sorted(cases)
        )

    def test_start_resumes_within_a_stable_filtered_shard(self) -> None:
        cases = ["20_beta", "10_alpha", "20_alpha", "30_alpha"]
        self.assertEqual(
            select_cases(
                cases,
                pattern="*alpha",
                start_from="30_alpha",
                shard_count=2,
                shard_index=0,
            ),
            ["30_alpha"],
        )

    def test_invalid_shard_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            select_cases(["case"], shard_count=0)
        with self.assertRaises(ValueError):
            select_cases(["case"], shard_count=2, shard_index=2)

    def test_manifest_records_selection_contract(self) -> None:
        payload = manifest_payload(
            Path("sim_case"), 95, ["a", "c"], "*", None, 2, 0
        )
        self.assertEqual(payload["schemaVersion"], 1)
        self.assertEqual(payload["sourceCaseCount"], 95)
        self.assertEqual(payload["cases"], ["a", "c"])
        self.assertEqual(
            payload["shard"],
            {"count": 2, "index": 0, "strategy": "sorted-round-robin"},
        )


if __name__ == "__main__":
    unittest.main()
