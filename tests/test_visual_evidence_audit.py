"""Pure-Python checks for saved visual-search evidence validation."""

from echorescue.visual_evidence_audit import audit_report, REQUIRED_CHECKS


def valid_report():
    case = "success"
    return {
        "schema_version": "echorescue-visual-search-smoke/1.0",
        "milestone": "v0.16.1",
        "case": case,
        "status": "PASS",
        "mode": "headless",
        "configuration_digest": "a" * 64,
        "checks": [
            {"name": name, "status": "PASS"}
            for name in sorted(REQUIRED_CHECKS | {"visual_search.success"})
        ],
        "cleanup": [
            {"name": "cleanup.ros_nodes", "status": "PASS"},
            {"name": "cleanup.udp_9002", "status": "PASS"},
        ],
        "controller_mission": {"status": "PASS"},
        "observer": {"status": "PASS"},
        "perception": {"counts": {"accepted": 5}},
        "gazebo_evaluation": {"prohibited_contact_detected": False},
        "evaluator_only_survivor_truth": {"evaluator_only": True},
    }


def test_accepts_complete_report():
    assert audit_report(valid_report()) == []


def test_rejects_missing_scenario_check():
    report = valid_report()
    report["checks"] = report["checks"][:-1]
    assert any("missing acceptance check" in item for item in audit_report(report))


def test_rejects_false_success_when_cleanup_failed():
    report = valid_report()
    report["cleanup"][1]["status"] = "FAIL"
    assert any("cleanup check" in item for item in audit_report(report))


def test_rejects_missing_ground_truth_boundary():
    report = valid_report()
    report["evaluator_only_survivor_truth"] = {"evaluator_only": False}
    assert any("evaluator truth" in item for item in audit_report(report))


def test_rejects_wrong_scenario_and_digest():
    report = valid_report()
    report["configuration_digest"] = "bad"
    errors = audit_report(report, expected_case="distractor")
    assert any("expected scenario" in item for item in errors)
    assert any("configuration digest" in item for item in errors)


def test_rejects_duplicate_checks():
    report = valid_report()
    report["checks"].append(dict(report["checks"][0]))
    assert any("duplicate" in item for item in audit_report(report))
