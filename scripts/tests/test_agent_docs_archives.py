from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def local_links(path: Path) -> list[Path]:
    links = re.findall(r"\[[^]]+\]\(([^)]+)\)", path.read_text(encoding="utf-8"))
    resolved = []
    for link in links:
        target = link.split("#", 1)[0]
        if not target or "://" in target:
            continue
        resolved.append((path.parent / target).resolve())
    return resolved


class AgentDocsArchiveTests(unittest.TestCase):
    def test_adrs_are_contiguous_complete_and_linked(self) -> None:
        directory = ROOT / "agent_docs" / "adr"
        records = sorted(directory.glob("[0-9][0-9][0-9][0-9]-*.md"))
        accepted = [path for path in records if path.name != "0000-template.md"]
        self.assertEqual(
            [path.name[:4] for path in accepted],
            [f"{number:04d}" for number in range(1, len(accepted) + 1)],
        )
        for path in accepted:
            text = path.read_text(encoding="utf-8")
            for heading in (
                "- Status: Accepted",
                "## Context",
                "## Decision",
                "## Consequences",
                "## Alternatives considered",
                "## Validation",
            ):
                self.assertIn(heading, text, f"{path.name}: missing {heading}")
        for target in local_links(directory / "README.md"):
            self.assertTrue(target.is_file(), f"missing ADR link target: {target}")

    def test_performance_index_template_and_links_are_valid(self) -> None:
        directory = ROOT / "agent_docs" / "performance"
        self.assertTrue((directory / "0000-evidence-template.md").is_file())
        self.assertGreaterEqual(
            len(list(directory.glob("[0-9][0-9][0-9][0-9]-*.md"))), 2
        )
        for target in local_links(directory / "README.md"):
            self.assertTrue(
                target.is_file(), f"missing performance link target: {target}"
            )


if __name__ == "__main__":
    unittest.main()
