from __future__ import annotations

import signal

import rclpy


def ignore_shutdown_signals() -> None:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)


def safe_destroy_node(node) -> None:
    if node is None:
        return
    context = getattr(node, "context", None)
    if context is not None and not context.ok():
        return
    if context is None and not rclpy.ok():
        return
    try:
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    except Exception:
        pass


def safe_shutdown() -> None:
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
