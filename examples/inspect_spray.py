"""Read the frozen real three-point evidence; no ROS or simulator required."""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
CASE = ROOT / 'evidence/validation/R12/spray'

def main():
    result = json.loads((CASE / 'mission_result.json').read_text(encoding='utf-8'))
    receipt = json.loads((CASE / 'recording_acceptance.json').read_text(encoding='utf-8'))
    events = [json.loads(line) for line in
              (CASE / 'mission_events.jsonl').read_text(encoding='utf-8').splitlines()]
    count = lambda name: sum(e['event'] == name for e in events)
    failed = [e for e in events if e['event'] == 'TOOL_RESULT' and e['state'] == 'FAILED']
    replay = result['replay']
    print(f"selected={result['selected_points']} completed={len(result['completed'])} original={result['original_points']}")
    print(f"tool_failed_attempts={len(failed)} residual_retries={count('RESIDUAL_RETRY')} idempotent_replays={count('IDEMPOTENT_REPLAY')}")
    print(f"replay_meter_before={replay['meter_before']:.4f} after={replay['meter_after']:.4f}")
    print(f"local_dock_goals={count('LOCAL_DOCK_GOAL')}")
    print(f"stopped={result['final_gate']['stopped']} tool_stopped={result['final_gate']['tool_stopped']}")
    print(f"sim_seconds={result['t']:.3f} task_wall_seconds={result['wall_seconds']:.3f}")
    print(f"task_passed={receipt['task_passed']} video_valid={receipt['video_valid']}")
    print('Scope: historical selected three-point case; original A1 has 56 obligations.')
    # This verifies the educational excerpt's internal relations, not the robot anew.
    assert result['selected_points'] == len(result['completed']) == 3
    assert result['original_points'] == 56 and len(result['not_selected']) == 53
    assert replay['replayed'] and replay['meter_after'] == replay['meter_before']
    assert len(failed) == count('RESIDUAL_RETRY') == 1
    assert count('LOCAL_DOCK_GOAL') == 0
    assert result['final_gate']['stopped'] and result['final_gate']['tool_stopped']

if __name__ == '__main__':
    main()
