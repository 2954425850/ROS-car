import os

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_fusion_controls_present():
    html = open(os.path.join(HERE, "index.html"), encoding="utf-8").read()
    for token in ('id="btn-fusion"', 'id="btn-explore"',
                  'id="btn-boundary"', 'id="btn-clear-boundary"',
                  'id="boundary-bar"'):
        assert token in html, token


def test_no_external_assets():
    html = open(os.path.join(HERE, "index.html"), encoding="utf-8").read()
    assert "http://" not in html
    assert "https://" not in html
