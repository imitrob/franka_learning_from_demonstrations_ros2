from panda_control.home_pose import HOME_POSE

__all__ = ['Panda', 'SpinPandaNode', 'SpinningRosNode', 'HOME_POSE']


def __getattr__(name):
    """Load the hardware-backed classes only when a caller requests them."""
    if name in {'Panda', 'SpinPandaNode', 'SpinningRosNode'}:
        from panda_control.panda import Panda, SpinPandaNode, SpinningRosNode
        return {
            'Panda': Panda,
            'SpinPandaNode': SpinPandaNode,
            'SpinningRosNode': SpinningRosNode,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
