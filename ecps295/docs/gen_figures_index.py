"""Build ecps295/docs/FIGURES.md: one page that lists every figure and diagram in the ecps295 documents.

Usage: python3 gen_gallery.py <repo root>
Reads the markdown files, takes each image link with the caption line under it, and each Mermaid block with the
heading it sits under. Writes ecps295/docs/FIGURES.md.
"""
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1])
ecps = root / "ecps295"
out = ecps / "docs" / "FIGURES.md"

GROUPS = [
    ("System and architecture diagrams", ["README.md", "docs/", "rl/README.md"]),
    ("Deployment and ground console", ["deploy/"]),
    ("Experiment reports (twin, Gazebo, optimisation)", ["reports/"]),
    ("Gazebo result sets", ["results/", "gazebo/", "scripts/"]),
    ("ArduPilot SITL bench studies", ["experiments/"]),
]
SKIP = ("report_backup.md", "report_source.md", "docs/FIGURES.md", "docs/DIAGRAMS.zh.md")


def slug(h):
    s = h.strip().lower()
    s = re.sub(r"[^\w\- 一-鿿]", "", s)
    return s.replace(" ", "-")


def caption(lines, start):
    """The italic paragraph (one or more lines, *...*) that follows a figure or diagram, or ""."""
    k = start
    while k < len(lines) and k < start + 3 and not lines[k].strip():
        k += 1
    if k >= len(lines):
        return ""
    s = lines[k].strip()
    if not s.startswith("*") or s.startswith("**"):
        return ""
    parts = []
    while k < len(lines) and lines[k].strip():
        parts.append(lines[k].strip())
        if lines[k].strip().endswith("*"):
            break
        k += 1
    text = " ".join(parts)
    return text.strip("*").strip() if text.endswith("*") else ""


def scan(md):
    rel = md.relative_to(ecps).as_posix()
    lines = md.read_text(encoding="utf-8").split("\n")
    title = next((l[2:].strip() for l in lines if l.startswith("# ")), rel)
    figs, diagrams = [], []
    heading = ""
    in_code = False
    i = 0
    while i < len(lines):
        l = lines[i]
        if l.startswith("```"):
            if l.strip() == "```mermaid" and not in_code:
                kind = ""
                j = i + 1
                while j < len(lines) and not lines[j].startswith("```"):
                    if not kind and lines[j].strip() and not lines[j].strip().startswith("%%"):
                        kind = lines[j].strip().split()[0]
                    j += 1
                cap = caption(lines, j + 1)
                diagrams.append((heading, kind, cap))
                i = j + 1
                continue
            in_code = not in_code
        elif not in_code:
            m = re.match(r"^(#{1,6})\s+(.*)", l)
            if m:
                heading = m.group(2).strip()
            for im in re.finditer(r"!\[([^\]]*)\]\(([^)\s]+)\)", l):
                alt, src = im.group(1), im.group(2)
                if src.startswith("http") or not re.search(r"\.(png|jpg|jpeg|gif|svg)$", src, re.I):
                    continue
                cap = caption(lines, i + 1)
                target = (md.parent / src).resolve()
                if target.exists():
                    figs.append((target.relative_to(ecps.resolve()).as_posix(), alt, cap, heading))
        i += 1
    return rel, title, figs, diagrams


docs = []
for md in sorted(ecps.rglob("*.md")):
    rel = md.relative_to(ecps).as_posix()
    if any(rel.endswith(s) for s in SKIP) or ".zh." in rel and (md.parent / md.name.replace(".zh.", ".en.")).exists():
        continue
    r = scan(md)
    if r[2] or r[3]:
        docs.append(r)

n_fig = sum(len(d[2]) for d in docs)
n_dia = sum(len(d[3]) for d in docs)
o = []
o.append("# Figures and diagrams")
o.append("")
o.append(
    f"One page with every picture in the `ecps295/` documents: {n_fig} data figures and photos, {n_dia} diagrams, "
    f"from {len(docs)} documents. Each entry links to the document that explains it."
)
o.append("")
o.append(
    "Data figures are made by the `make_figures.py` script next to each `figures/` directory, from the result files "
    "in the same directory. Diagrams are Mermaid blocks inside the documents; GitHub draws them."
)
o.append("")
o.append(
    "Where a report exists in English and Chinese, this page lists the English one. "
    "The diagrams with Chinese labels are in [DIAGRAMS.zh.md](DIAGRAMS.zh.md) · 中文图册. "
    "This page is generated: `ecps295/docs/gen_figures_index.py`."
)
o.append("")
o.append("## Contents")
o.append("")
for g, _ in GROUPS:
    o.append(f"- [{g}](#{slug(g)})")
o.append("")
seen = set()
for g, prefixes in GROUPS:
    o.append(f"## {g}")
    o.append("")
    for rel, title, figs, diagrams in docs:
        if rel in seen or not any(rel == p or rel.startswith(p) for p in prefixes):
            continue
        seen.add(rel)
        link = "../" + rel
        o.append(f"### {title}")
        o.append("")
        o.append(f"Document: [`{rel}`]({link.replace(' ', '%20')})")
        o.append("")
        if diagrams:
            o.append("| Diagram | Type | Section |")
            o.append("|---|---|---|")
            for heading, kind, cap in diagrams:
                what = re.split(r"\.? (?:From|Sources?:|Source:|Taken from|来源)", cap)[0][:300] if cap else "—"
                what = what.replace("|", "\\|")
                sec = heading.replace("|", "\\|")
                o.append(f"| {what} | {kind} | [{sec}]({link}#{slug(heading)}) |")
            o.append("")
        if figs:
            o.append("| Figure | What it shows |")
            o.append("|---|---|")
            for src, alt, cap, heading in figs:
                text = (cap or alt or "").replace("|", "\\|")
                o.append(f'| <a href="../{src}"><img src="../{src}" width="320" alt="{alt.replace(chr(34), "")}"></a> | {text} |')
            o.append("")
out.write_text("\n".join(o) + "\n", encoding="utf-8")
print(out, n_fig, "figures", n_dia, "diagrams", len(docs), "documents")
missing = [d[0] for d in docs if d[0] not in seen]
if missing:
    print("not in any group:", missing)
