"""Training run summaries read only the progress tail (no display needed)."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_lab_gui import training_tab  # noqa: E402


def test_summary_reads_the_last_iteration_and_finish_from_a_long_file(tmp_path, monkeypatch):
    monkeypatch.setattr(training_tab, 'PROGRESS_TAIL_BYTES', 300)
    run = tmp_path/'run'
    run.mkdir()
    (run/'run.json').write_text(json.dumps(dict(robot='unitree_go2', task='walk', iterations=50)))
    lines = [dict(iteration=i, mean_return=float(i), terms={'x': 'y'*20}) for i in range(1, 51)]
    progress = run/'progress.jsonl'
    progress.write_text(''.join(json.dumps(line)+'\n' for line in lines))
    summary = training_tab.run_summary(run)
    assert summary['iteration'] == 50 and summary['mean_return'] == 50. and not summary['finished']
    with progress.open('a') as handle:
        handle.write(json.dumps(dict(event='finished'))+'\n')
    assert training_tab.run_summary(run)['finished']          # the size changed: re-read
    assert len(training_tab.read_progress(run)) == 51           # the full reader is unchanged
