# Visual-search acceptance evidence audit (experimental)

This additive, offline auditor checks saved v0.16.1 integration reports before
someone cites them as accepted evidence. It does not control a vehicle, rerun
Gazebo, validate the physical correctness of a detector, or replace the
integration acceptance tests.

Run after the existing integration scenario:

```bash
python -m echorescue.visual_evidence_audit artifacts/v0.16.1-visual-headless.json --case success
python -m echorescue.visual_evidence_audit artifacts/v0.16.1-visual-*.json
python -m pytest -q tests/test_visual_evidence_audit.py
```

The command exits 0 only when all supplied JSON files satisfy the structural
acceptance contract. It exits 1 for absent/malformed files, incomplete checks,
non-PASS outcomes, missing mission or perception sections, missing evaluator
truth separation, or incomplete cleanup evidence. Each file is reported
independently.

A PASS from this audit means the recorded evidence is structurally complete
and internally reports a passing acceptance outcome. **It does not mean that
a new v0.16.1 Gazebo run was executed or independently reproduced.**

Follow up with the full pinned ROS 2/Gazebo graphical and headless scenario
suite, inspecting evidence and teardown behavior before claiming an accepted
new milestone. Existing v0.15.0 portfolio artifacts are deliberately outside
this auditor's schema.
