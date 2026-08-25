import importlib.util
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "gen_uid_doc.py"
SPEC = importlib.util.spec_from_file_location("gen_uid_doc", SCRIPT_PATH)
gen_uid_doc = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(gen_uid_doc)


class UIDDocumentGeneratorTests(unittest.TestCase):
    def test_collects_both_registration_styles(self):
        source = r'''
constexpr U64 DesignatedUID = 41;
constexpr U64 StatementUID = 42ull;
auto constraint = string{builtin::Constraint};

uids.push_back(UIDInfo{
    .uid = DesignatedUID,
    .name = "Designated",
    .type = constraint});

builtin::UIDInfo info;
info.uid = StatementUID;
info.name = "Statement";
info.type = string{builtin::FiniteElement};
uids.push_back(info);
'''

        entries = gen_uid_doc.collect_uids_from_text(source, "fixture.cpp")

        self.assertEqual(
            entries,
            [
                {
                    "uid": 41,
                    "name": "Designated",
                    "type": "Constraint",
                    "source": "fixture.cpp",
                },
                {
                    "uid": 42,
                    "name": "Statement",
                    "type": "FiniteElement",
                    "source": "fixture.cpp",
                },
            ],
        )

    def test_collects_all_statement_registered_constitutions(self):
        project_dir = Path(__file__).resolve().parents[2]
        entries = gen_uid_doc.collect_uids(project_dir / "src" / "constitution")
        by_uid = {entry["uid"]: entry["name"] for entry in entries}

        self.assertEqual(by_uid[15], "KirchhoffRodBending")
        self.assertEqual(by_uid[17], "DiscreteShellBending")
        self.assertEqual(by_uid[31], "StrainPlasticDiscreteShellBending")
        self.assertEqual(by_uid[32], "StressPlasticDiscreteShellBending")


if __name__ == "__main__":
    unittest.main()
