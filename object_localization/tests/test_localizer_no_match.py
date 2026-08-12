"""A template that is not in the picture must report, not raise."""
import types

from object_localization.localizer_sift import Localizer


def _unmatched():
    """A Localizer as detect_points leaves it when nothing matched."""
    stub = types.SimpleNamespace(_src_pts=None, _dst_pts=None)
    stub.compute_tf = lambda: Localizer.compute_tf(stub)
    return stub


def test_compute_tf_returns_none_without_matches():
    assert Localizer.compute_tf(_unmatched()) is None


def test_compute_full_tf_in_m_returns_none_without_matches():
    assert Localizer.compute_full_tf_in_m(_unmatched()) is None
