from pathlib import Path

from twin_sim.tasks.chop import ChopConfig, run_chop


if __name__ == "__main__":
    result = run_chop(ChopConfig(), log_path=Path("chop.csv"), viewer=False)
    print(f"completed={result.completed}, samples={len(result.samples)}")

