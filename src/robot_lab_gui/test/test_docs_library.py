"""Documentation library: structure from docs/library.yaml, inline figures and
animations, and rendered Mermaid diagrams."""
import os
import tkinter as tk
from pathlib import Path

import pytest
import yaml

from robot_lab_gui.doc_diagrams import blocks_in, image_for, markdown_files, parse_flowchart, parse_sequence

ROOT = Path(__file__).resolve().parents[3]


def test_every_mermaid_diagram_in_the_docs_is_rendered():
    missing = []
    for md in markdown_files(ROOT):
        for text in blocks_in(md.read_text(encoding='utf-8', errors='replace')):
            if image_for(ROOT, text) is None:
                missing.append(str(md.relative_to(ROOT)))
    assert not missing, 'run python3 scripts/render_doc_diagrams.py; unrendered diagrams in %s' % sorted(set(missing))


def test_library_yaml_lists_only_existing_pages():
    data = yaml.safe_load((ROOT / 'docs/library.yaml').read_text())
    for section in data['sections']:
        for page in section['pages']:
            path = page if isinstance(page, str) else page['path']
            assert (ROOT / path).is_file(), path


def test_mermaid_parsers_read_the_doc_subset():
    graph = parse_flowchart('flowchart LR\n  subgraph S["Group"]\n    A["one<br/>two"] -->|label| B{choice}\n  end\n'
                            '  B -- yes --> C((done))\n  B -. no .-> A\n  X & Y --> C\n')
    assert graph['nodes']['A']['label'] == 'one\ntwo' and graph['nodes']['A']['cluster'] == 'S'
    edges = {(e['src'], e['dst']): e for e in graph['edges']}
    assert edges[('A', 'B')]['label'] == 'label' and edges[('B', 'C')]['label'] == 'yes'
    assert edges[('B', 'A')]['dotted'] and ('X', 'C') in edges and ('Y', 'C') in edges
    seq = parse_sequence('sequenceDiagram\n  participant A as Alpha\n  A->>B: hi\n  B-->>A: back\n  loop again\n  A->>B: x\n  end\n')
    assert seq['parts'] == ['A', 'B'] and seq['alias']['A'] == 'Alpha' and len(seq['frames']) == 1


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Requires Xvfb or a display')
def test_library_tree_figures_and_diagrams():
    from robot_lab_gui.architecture_docs import GuideLibrary
    root = tk.Tk()
    try:
        opened = []
        lib = GuideLibrary(root, ROOT, open_diagram=opened.append)
        lib.pack(fill='both', expand=True)
        root.update()
        sections = [lib.tree.item(i, 'text') for i in lib.tree.get_children()]
        assert sections[:2] == ['Start here', 'Beginner course'] and sections[-1] == 'Packages (actual package.xml)'
        course = lib.tree.get_children(lib.tree.get_children()[1])
        assert [lib.tree.item(i, 'text') for i in course][0].startswith('1 · The big picture')
        assert lib.current == ROOT / 'docs/learn/README.md'          # the default page
        lib.open(ROOT / 'docs/learn/07-robot-vacuum-inside.md')
        root.update()
        assert len(lib.images) > 50                                  # GIF frames + figures + diagrams
        assert lib.animations                                        # the cleaning GIF plays
        assert 'Diagram' in lib.reader.get('1.0', 'end')             # Mermaid shown as images
        assert 'stateDiagram' not in lib.reader.get('1.0', 'end')    # not as code
        lib.open(ROOT / 'docs/architecture/interactive-guide.md')
        assert 'Open interactive flowchart' in lib.reader.get('1.0', 'end')
        lib.open(ROOT / 'docs/learn/figures/room_division.png')
        assert lib.images and lib.current.name == 'room_division.png'
        lib.query.set('particle')
        lib.filter()
        assert not lib.tree.get_children() or all(
            lib.tree.item(i, 'text') != 'Measured comparisons' for i in lib.tree.get_children())
    finally:
        root.destroy()
