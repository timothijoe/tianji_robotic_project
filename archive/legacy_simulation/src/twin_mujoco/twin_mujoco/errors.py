from __future__ import annotations


class TwinMujocoError(RuntimeError):
    pass


class MujocoModelError(TwinMujocoError):
    pass


class SafetyStop(TwinMujocoError):
    pass
