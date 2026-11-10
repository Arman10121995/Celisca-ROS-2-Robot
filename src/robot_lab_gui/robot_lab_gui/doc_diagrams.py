"""Render the Mermaid diagrams in the Markdown docs to PNG for the GUI reader.

GitHub draws ```mermaid blocks itself; the native Tk reader cannot, so each
block is rendered once (Graphviz for flowcharts and state diagrams,
matplotlib for sequence diagrams) to ``docs/diagrams/<hash>.png``.  The hash
is of the block's text, so an edited diagram simply gets a new image and
``render_all`` removes images no block uses any more.

    python3 scripts/render_doc_diagrams.py        # after editing a diagram

Only the Mermaid subset the docs use is understood: flowchart/graph (node
shapes, subgraphs, plain/dotted/thick edges, edge labels, chains),
stateDiagram-v2 (transitions with labels, [*]) and sequenceDiagram
(participants, messages, notes, loop/alt/opt/else frames, autonumber).
"""

import hashlib
import json
import re
import subprocess
from pathlib import Path

DIAGRAM_DIR = Path('docs') / 'diagrams'
FENCE = re.compile(r'^```mermaid\s*\n(.*?)^```', re.S | re.M)


def block_key(text):
    """The image name for a block: a hash of its stripped text."""
    return hashlib.sha256(text.strip().encode('utf-8')).hexdigest()[:16]


def blocks_in(markdown):
    return [m.group(1) for m in FENCE.finditer(markdown)]


def label_text(raw):
    raw = raw.strip()
    if len(raw) >= 2 and raw[0] == raw[-1] == '"':
        raw = raw[1:-1]
    raw = re.sub(r'<br\s*/?>', '\n', raw)
    for a, b in (('&lt;', '<'), ('&gt;', '>'), ('&amp;', '&'), ('&quot;', '"'), ('#quot;', '"')):
        raw = raw.replace(a, b)
    return raw.strip()


def dot_escape(text):
    return text.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')


# ---------------------------------------------------------------------------
# Flowcharts
# ---------------------------------------------------------------------------

SHAPES = [  # (open, close, dot attributes)
    ('[[', ']]', 'shape=box, style="rounded,filled,bold"'),
    ('[(', ')]', 'shape=cylinder, style=filled'),
    ('[/', '/]', 'shape=parallelogram, style=filled'),
    ('{{', '}}', 'shape=hexagon, style=filled, fillcolor="#fff3bf", color="#f08c00"'),
    ('((', '))', 'shape=circle, style=filled, fillcolor="#fff3bf", color="#f08c00"'),
    ('[', ']', ''),
    ('{', '}', 'shape=diamond, style=filled, fillcolor="#fff3bf", color="#f08c00"'),
    ('(', ')', 'style="rounded,filled"'),
    ('>', ']', 'shape=cds, style=filled'),
]
NODE = re.compile(r'\s*([A-Za-z0-9_\-]+)')
PIPE_EDGE = re.compile(r'\s*(<)?(-{2,3}>|-{3}|-\.+->|-\.+-|={2,3}>|={3})(?:\|("?)(.*?)\3\|)?\s*')
TEXT_EDGE = re.compile(r'\s*(<)?(--|-\.|==)\s+("?)(.+?)\3\s+(-->|---|\.->|\.-|==>|===)\s*')


def _read_shape(line, i):
    for opening, closing, attrs in SHAPES:
        if line.startswith(opening, i):
            j = i + len(opening)
            if j < len(line) and line[j] == '"':
                end = line.find('"', j + 1)
                end = line.find(closing, end) if end >= 0 else -1
            else:
                end = line.find(closing, j)
            if end < 0:
                return None, i
            return (label_text(line[j:end]), attrs), end + len(closing)
    return None, i


def parse_flowchart(text):
    lines = [l.strip() for l in text.splitlines()]
    header = lines[0].split()
    direction = header[1] if len(header) > 1 else 'TB'
    nodes, order, edges, clusters, stack = {}, [], [], [], []

    def node(i, line):
        m = NODE.match(line, i)
        if not m:
            return None, i
        nid = m.group(1)
        shape, j = _read_shape(line, m.end())
        if nid not in nodes:
            nodes[nid] = {'label': nid, 'attrs': '', 'cluster': stack[-1] if stack else None}
            order.append(nid)
        if shape:
            nodes[nid]['label'], nodes[nid]['attrs'] = shape
            if stack and nodes[nid]['cluster'] is None:
                nodes[nid]['cluster'] = stack[-1]
        return nid, j

    for line in lines[1:]:
        line = line.split('%%')[0].strip()
        if not line or line.startswith(('classDef', 'class ', 'style ', 'click ', 'linkStyle', 'direction ')):
            continue
        if line.startswith('subgraph'):
            rest = line[len('subgraph'):].strip()
            m = re.match(r'([A-Za-z0-9_\-]+)\s*\[(.*)\]\s*$', rest)
            cid, title = (m.group(1), label_text(m.group(2))) if m else (rest.split()[0] if rest else 'g', label_text(rest))
            clusters.append({'id': cid, 'title': title, 'parent': stack[-1] if stack else None})
            stack.append(cid)
            continue
        if line == 'end':
            if stack:
                stack.pop()
            continue
        i, previous = 0, None
        while i < len(line):
            groups = []
            nid, i = node(i, line)
            while nid and line[i:].lstrip().startswith('&'):   # A & B --> C
                groups.append(nid)
                nid, i = node(line.index('&', i) + 1, line)
            group = groups + ([nid] if nid else [])
            if not group:
                break
            if previous:
                src, edge = previous
                for s in src:
                    for t in group:
                        edges.append(dict(edge, src=s, dst=t))
            m = TEXT_EDGE.match(line, i)
            if m:
                arrow = m.group(5)
                edge = {'label': label_text(m.group(4)), 'dotted': '.' in m.group(2) + arrow,
                        'thick': '=' in arrow, 'head': arrow.endswith('>'), 'both': bool(m.group(1))}
                i = m.end()
            else:
                m = PIPE_EDGE.match(line, i)
                if not m or not m.group(2):
                    break
                arrow = m.group(2)
                edge = {'label': label_text(m.group(4) or ''), 'dotted': '.' in arrow, 'thick': '=' in arrow,
                        'head': arrow.endswith('>'), 'both': bool(m.group(1))}
                i = m.end()
            previous = (group, edge)
    return {'direction': direction, 'nodes': nodes, 'order': order, 'edges': edges, 'clusters': clusters}


def flowchart_dot(graph):
    rank = {'TD': 'TB', 'TB': 'TB', 'LR': 'LR', 'RL': 'RL', 'BT': 'BT'}.get(graph['direction'], 'TB')
    out = ['digraph G {', '  rankdir=%s; compound=true; bgcolor="white"; pad=0.25; nodesep=0.35; ranksep=0.45;' % rank,
           '  node [shape=box, style="rounded,filled", fillcolor="#e7f5ff", color="#1c7ed6", fontname="DejaVu Sans",'
           ' fontsize=11, margin="0.16,0.07"];',
           '  edge [color="#495057", fontname="DejaVu Sans", fontsize=10, fontcolor="#343a40", arrowsize=0.8];']
    cluster_ids = {c['id'] for c in graph['clusters']}
    first_node = {}
    for nid in graph['order']:
        c = graph['nodes'][nid]['cluster']
        while c and c not in first_node:
            first_node[c] = nid
            c = next((x['parent'] for x in graph['clusters'] if x['id'] == c), None)

    def emit_cluster(cid, indent):
        c = next(x for x in graph['clusters'] if x['id'] == cid)
        out.append('%ssubgraph "cluster_%s" {' % (indent, cid))
        out.append('%s  label="%s"; style="rounded,filled"; fillcolor="#f8f9fa"; color="#adb5bd"; '
                   'fontname="DejaVu Sans"; fontsize=11; fontcolor="#495057";' % (indent, dot_escape(c['title'])))
        for nid in graph['order']:
            if graph['nodes'][nid]['cluster'] == cid and nid not in cluster_ids:
                emit_node(nid, indent + '  ')
        for child in graph['clusters']:
            if child['parent'] == cid:
                emit_cluster(child['id'], indent + '  ')
        out.append(indent + '}')

    def emit_node(nid, indent):
        n = graph['nodes'][nid]
        attrs = ', ' + n['attrs'] if n['attrs'] else ''
        out.append('%s"%s" [label="%s"%s];' % (indent, nid, dot_escape(n['label']), attrs))

    for nid in graph['order']:
        if graph['nodes'][nid]['cluster'] is None and nid not in cluster_ids:
            emit_node(nid, '  ')
    for c in graph['clusters']:
        if c['parent'] is None:
            emit_cluster(c['id'], '  ')
    for e in graph['edges']:
        attrs, src, dst = [], e['src'], e['dst']
        if src in cluster_ids:
            attrs.append('ltail="cluster_%s"' % src)
            src = first_node.get(src, src)
        if dst in cluster_ids:
            attrs.append('lhead="cluster_%s"' % dst)
            dst = first_node.get(dst, dst)
        if e['label']:
            attrs.append('label="%s"' % dot_escape(e['label']))
        if e['dotted']:
            attrs.append('style=dashed')
        if e['thick']:
            attrs.append('penwidth=2.2')
        if e['both']:
            attrs.append('dir=both')
        elif not e['head']:
            attrs.append('arrowhead=none')
        out.append('  "%s" -> "%s" [%s];' % (src, dst, ', '.join(attrs)))
    out.append('}')
    return '\n'.join(out)


# ---------------------------------------------------------------------------
# State diagrams
# ---------------------------------------------------------------------------

def state_dot(text):
    out = ['digraph G {', '  rankdir=TB; bgcolor="white"; pad=0.25; nodesep=0.4; ranksep=0.4;',
           '  node [shape=box, style="rounded,filled", fillcolor="#e7f5ff", color="#1c7ed6", fontname="DejaVu Sans", fontsize=11];',
           '  edge [color="#495057", fontname="DejaVu Sans", fontsize=10, fontcolor="#343a40", arrowsize=0.8];']
    if '[*] -->' in text:
        out.append('  "__start" [shape=circle, label="", width=0.22, style=filled, fillcolor="#212529"];')
    if re.search(r'-->\s*\[\*\]', text):
        out.append('  "__end" [shape=doublecircle, label="", width=0.18, style=filled, fillcolor="#212529"];')
    for line in text.splitlines()[1:]:
        m = re.match(r'\s*(\[\*\]|[\w-]+)\s*-->\s*(\[\*\]|[\w-]+)\s*(?::\s*(.*))?$', line)
        if not m:
            continue
        a = '__start' if m.group(1) == '[*]' else m.group(1)
        b = '__end' if m.group(2) == '[*]' else m.group(2)
        label = ' [label="%s"]' % dot_escape(m.group(3).strip()) if m.group(3) else ''
        out.append('  "%s" -> "%s"%s;' % (a, b, label))
    out.append('}')
    return '\n'.join(out)


# ---------------------------------------------------------------------------
# Sequence diagrams (matplotlib)
# ---------------------------------------------------------------------------

def parse_sequence(text):
    parts, rows, frames, stack, auto = [], [], [], [], False
    alias = {}

    def part(name):
        if name not in alias:
            alias[name] = name
            parts.append(name)
        return name
    for raw in text.splitlines()[1:]:
        line = raw.strip()
        if not line or line.startswith('%%'):
            continue
        m = re.match(r'(participant|actor)\s+(\w+)(?:\s+as\s+(.*))?$', line)
        if m:
            part(m.group(2))
            alias[m.group(2)] = label_text(m.group(3) or m.group(2))
            continue
        if line == 'autonumber':
            auto = True
            continue
        m = re.match(r'(\w+)\s*(-->>|->>|-->|->|--x|-x|--\)|-\))\s*(\w+)\s*:\s*(.*)$', line)
        if m:
            rows.append(('msg', part(m.group(1)), part(m.group(3)), label_text(m.group(4)), m.group(2).startswith('--')))
            continue
        m = re.match(r'[Nn]ote\s+(over|left of|right of)\s+(\w+)(?:\s*,\s*(\w+))?\s*:\s*(.*)$', line)
        if m:
            rows.append(('note', part(m.group(2)), part(m.group(3)) if m.group(3) else part(m.group(2)),
                         label_text(m.group(4)), False))
            continue
        m = re.match(r'(loop|alt|opt|par|critical|break)\b\s*(.*)$', line)
        if m:
            stack.append({'kind': m.group(1), 'label': label_text(m.group(2)), 'start': len(rows), 'splits': []})
            rows.append(('gap', None, None, '', False))   # a row for the frame's label
            continue
        m = re.match(r'(else|and)\b\s*(.*)$', line)
        if m and stack:
            stack[-1]['splits'].append((len(rows), label_text(m.group(2))))
            rows.append(('gap', None, None, '', False))
            continue
        if line == 'end' and stack:
            f = stack.pop()
            f['end'] = len(rows)
            f['depth'] = len(stack)
            frames.append(f)
            continue
    return {'parts': parts, 'alias': alias, 'rows': rows, 'frames': frames, 'auto': auto}


def render_sequence(text, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    seq = parse_sequence(text)
    parts = seq['parts']
    xs = {p: 2.4 * i for i, p in enumerate(parts)}
    n = max(1, len(seq['rows']))
    width = max(6.0, 2.4 * (len(parts) - 1) + 3.0)
    height = 1.6 + 0.55 * n
    fig, ax = plt.subplots(figsize=(width, height))
    top, step = 0.0, 0.55
    ax.set_xlim(-1.5, 2.4 * (len(parts) - 1) + 1.5)
    ax.set_ylim(-(n + 1.6) * step - 0.5, 0.75)
    ax.axis('off')
    for p, x in xs.items():
        for y in (top + 0.35, -(n + 1) * step - 0.1):
            ax.text(x, y, seq['alias'][p], ha='center', va='center', fontsize=9.5,
                    bbox=dict(boxstyle='round,pad=0.4', fc='#e7f5ff', ec='#1c7ed6'))
        ax.plot([x, x], [top, -(n + 1) * step + 0.15], color='#adb5bd', ls='--', lw=1, zorder=0)
    for f in seq['frames']:
        y0 = top - (f['start'] + 0.45) * step
        y1 = top - (f['end'] + 0.55) * step
        pad = 0.9 - 0.15 * f['depth']
        x0, x1 = min(xs.values()) - pad, max(xs.values()) + pad
        ax.add_patch(matplotlib.patches.Rectangle((x0, y1), x1 - x0, y0 - y1, fill=False, ec='#868e96', lw=1))
        ax.text(x0 + 0.05, y0 - 0.05, ' %s %s' % (f['kind'], f['label']), fontsize=8.5, va='top', color='#495057',
                bbox=dict(fc='#f1f3f5', ec='#868e96', pad=1.5))
        for row, label in f['splits']:
            y = top - (row + 0.5) * step
            ax.plot([x0, x1], [y, y], color='#868e96', ls=':', lw=1)
            ax.text(x0 + 0.05, y - 0.04, ' [%s]' % label, fontsize=8.5, va='top', color='#495057')
    k = 0
    for r, (kind, a, b, label, dashed) in enumerate(seq['rows']):
        y = top - (r + 1) * step
        if kind == 'msg':
            k += 1
            text = ('%d. ' % k if seq['auto'] else '') + label
            if a == b:
                ax.annotate('', (xs[a] + 0.05, y - 0.18), (xs[a] + 0.05, y),
                            arrowprops=dict(arrowstyle='->', color='#343a40', connectionstyle='arc3,rad=-1.2',
                                            ls='--' if dashed else '-'))
                ax.text(xs[a] + 0.45, y + 0.02, text, fontsize=8.5, va='bottom')
            else:
                ax.annotate('', (xs[b], y), (xs[a], y), arrowprops=dict(
                    arrowstyle='-|>', color='#343a40', lw=1.1, ls='--' if dashed else '-', shrinkA=0, shrinkB=0))
                ax.text((xs[a] + xs[b]) / 2, y + 0.04, text, ha='center', va='bottom', fontsize=8.5, wrap=True)
        elif kind == 'note':
            x0, x1 = min(xs[a], xs[b]) - 0.6, max(xs[a], xs[b]) + 0.6
            ax.text((x0 + x1) / 2, y, label, ha='center', va='center', fontsize=8.5,
                    bbox=dict(boxstyle='round,pad=0.35', fc='#fff9db', ec='#f08c00'))
    fig.savefig(path, dpi=110, bbox_inches='tight', facecolor='white')
    plt.close(fig)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def render_block(text, path):
    kind = text.strip().split()[0]
    if kind == 'sequenceDiagram':
        render_sequence(text, path)
        return
    if kind.startswith('stateDiagram'):
        dot = state_dot(text)
    elif kind in ('flowchart', 'graph'):
        dot = flowchart_dot(parse_flowchart(text))
    else:
        raise ValueError('unsupported Mermaid diagram: ' + kind)
    subprocess.run(['dot', '-Tpng', '-Gdpi=110', '-o', str(path)], input=dot.encode('utf-8'), check=True,
                   capture_output=True)


def markdown_files(root):
    root = Path(root)
    files = [root / 'README.md', root / 'ROADMAP.md']
    files += [p for p in sorted((root / 'docs').rglob('*.md')) if '/evidence/' not in str(p)]
    return [p for p in files if p.exists()]


def render_all(root, force=False):
    """Render every Mermaid block in the docs; remove unused images.
    Returns {key: source file} for the rendered set."""
    root = Path(root)
    out = root / DIAGRAM_DIR
    out.mkdir(parents=True, exist_ok=True)
    used = {}
    for md in markdown_files(root):
        for text in blocks_in(md.read_text(encoding='utf-8', errors='replace')):
            key = block_key(text)
            used[key] = str(md.relative_to(root))
            png = out / (key + '.png')
            if force or not png.exists():
                render_block(text, png)
    for png in out.glob('*.png'):
        if png.stem not in used:
            png.unlink()
    (out / 'index.json').write_text(json.dumps(used, indent=1, sort_keys=True) + '\n')
    return used


def image_for(root, text):
    """The rendered PNG for a block, or None when it has not been rendered."""
    png = Path(root) / DIAGRAM_DIR / (block_key(text) + '.png')
    return png if png.exists() else None
