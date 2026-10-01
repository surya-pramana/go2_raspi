"""Create a small, dependency-free HTML report for camera benchmark CSV data."""

from __future__ import annotations

import argparse
import csv
import html
import math
from pathlib import Path
from statistics import mean, median


COLORS = {
    "total_latency_ms": "#2563eb",
    "encode_ms": "#16a34a",
    "network_ms": "#f97316",
    "fps": "#7c3aed",
    "packet_bytes": "#0891b2",
}


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo, hi = math.floor(pos), math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def downsample(rows: list[dict[str, float]], keys: list[str], points: int = 900):
    size = max(1, math.ceil(len(rows) / points))
    sampled = []
    for start in range(0, len(rows), size):
        bucket = rows[start : start + size]
        item = {"elapsed_s": mean(r["elapsed_s"] for r in bucket)}
        for key in keys:
            item[key] = mean(r[key] for r in bucket)
        sampled.append(item)
    return sampled


def chart(rows, series, title, unit, width=1100, height=260):
    ml, mr, mt, mb = 66, 20, 30, 42
    pw, ph = width - ml - mr, height - mt - mb
    xs = [r["elapsed_s"] for r in rows]
    ys = [r[key] for r in rows for key, _ in series]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    pad = (ymax - ymin) * 0.08 or 1
    ymin, ymax = max(0, ymin - pad), ymax + pad

    def xy(x, y):
        px = ml + (x - xmin) / (xmax - xmin or 1) * pw
        py = mt + ph - (y - ymin) / (ymax - ymin or 1) * ph
        return px, py

    parts = [f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">']
    parts.append(f'<text x="{ml}" y="20" class="title">{html.escape(title)}</text>')
    for i in range(5):
        yv = ymin + (ymax - ymin) * i / 4
        _, py = xy(xmin, yv)
        parts.append(f'<line x1="{ml}" y1="{py:.1f}" x2="{width-mr}" y2="{py:.1f}" class="grid"/>')
        parts.append(f'<text x="{ml-8}" y="{py+4:.1f}" text-anchor="end" class="tick">{yv:.1f}</text>')
    for i in range(5):
        xv = xmin + (xmax - xmin) * i / 4
        px, _ = xy(xv, ymin)
        parts.append(f'<text x="{px:.1f}" y="{height-13}" text-anchor="middle" class="tick">{xv/60:.1f}</text>')
    parts.append(f'<text x="{width/2}" y="{height-1}" text-anchor="middle" class="axis">waktu (menit)</text>')
    parts.append(f'<text x="14" y="{height/2}" transform="rotate(-90 14 {height/2})" text-anchor="middle" class="axis">{html.escape(unit)}</text>')
    for key, label in series:
        pts = " ".join(f"{xy(r['elapsed_s'], r[key])[0]:.1f},{xy(r['elapsed_s'], r[key])[1]:.1f}" for r in rows)
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{COLORS[key]}" stroke-width="1.6"/>')
    lx = ml
    for key, label in series:
        parts.append(f'<line x1="{lx}" y1="{height-29}" x2="{lx+18}" y2="{height-29}" stroke="{COLORS[key]}" stroke-width="3"/>')
        parts.append(f'<text x="{lx+24}" y="{height-25}" class="legend">{html.escape(label)}</text>')
        lx += 150
    parts.append("</svg>")
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("-o", "--output", type=Path, default=Path("camera_report.html"))
    args = parser.parse_args()

    numeric = ["elapsed_s", "frame_num", "total_latency_ms", "encode_ms", "network_ms", "fps", "packet_bytes"]
    rows = []
    with args.csv_file.open(encoding="utf-8-sig", newline="") as f:
        for raw in csv.DictReader(f):
            try:
                rows.append({key: float(raw[key]) for key in numeric})
            except (KeyError, TypeError, ValueError):
                continue
    if not rows:
        raise SystemExit("Tidak ada baris data valid.")

    sampled = downsample(rows, ["total_latency_ms", "encode_ms", "network_ms", "fps", "packet_bytes"])
    metrics = [
        ("total_latency_ms", "Latensi total", "ms"),
        ("encode_ms", "Encode", "ms"),
        ("network_ms", "Network", "ms"),
        ("fps", "FPS", "fps"),
        ("packet_bytes", "Ukuran paket", "KB"),
    ]
    stats = []
    for key, label, unit in metrics:
        vals = [r[key] / 1024 if key == "packet_bytes" else r[key] for r in rows]
        stats.append((label, unit, mean(vals), median(vals), percentile(vals, .95), max(vals)))

    table_rows = "".join(
        f"<tr><td>{label}</td><td>{avg:.2f}</td><td>{med:.2f}</td><td>{p95:.2f}</td><td>{mx:.2f}</td><td>{unit}</td></tr>"
        for label, unit, avg, med, p95, mx in stats
    )
    duration = rows[-1]["elapsed_s"] - rows[0]["elapsed_s"]
    zero_fps = sum(r["fps"] == 0 for r in rows)
    html_doc = f"""<!doctype html><html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Laporan Benchmark Kamera</title>
<style>body{{font-family:system-ui,sans-serif;margin:0;background:#f8fafc;color:#172033}}main{{max-width:1160px;margin:auto;padding:28px}}h1{{margin-bottom:4px}}.sub{{color:#64748b}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin:22px 0}}.card,section{{background:white;border:1px solid #e2e8f0;border-radius:12px;padding:16px;box-shadow:0 1px 2px #0000000a}}.big{{font-size:1.55rem;font-weight:700}}section{{margin:14px 0;overflow-x:auto}}svg{{width:100%;min-width:700px}}.grid{{stroke:#e2e8f0}}.tick,.axis,.legend{{font-size:11px;fill:#64748b}}.title{{font-size:15px;font-weight:650;fill:#172033}}table{{border-collapse:collapse;width:100%}}th,td{{padding:9px;border-bottom:1px solid #e2e8f0;text-align:right}}th:first-child,td:first-child{{text-align:left}}</style></head><body><main>
<h1>Benchmark kamera</h1><div class="sub">{html.escape(args.csv_file.name)} · kurva memakai rata-rata per bucket agar ringan dibuka</div>
<div class="cards"><div class="card"><div class="sub">Durasi</div><div class="big">{duration/60:.1f} menit</div></div><div class="card"><div class="sub">Frame valid</div><div class="big">{len(rows):,}</div></div><div class="card"><div class="sub">FPS rata-rata</div><div class="big">{mean(r['fps'] for r in rows):.1f}</div></div><div class="card"><div class="sub">Latensi rata-rata</div><div class="big">{mean(r['total_latency_ms'] for r in rows):.1f} ms</div></div></div>
<section><h2>Ringkasan statistik</h2><table><thead><tr><th>Metrik</th><th>Rata-rata</th><th>Median</th><th>P95</th><th>Maksimum</th><th>Satuan</th></tr></thead><tbody>{table_rows}</tbody></table><p class="sub">FPS bernilai 0 pada {zero_fps} frame awal.</p></section>
<section>{chart(sampled, [('total_latency_ms','Total'),('network_ms','Network'),('encode_ms','Encode')], 'Latensi terhadap waktu', 'ms')}</section>
<section>{chart(sampled, [('fps','FPS')], 'Frame rate', 'fps')}</section>
<section>{chart([{**r,'packet_bytes':r['packet_bytes']/1024} for r in sampled], [('packet_bytes','Ukuran paket')], 'Ukuran paket', 'KB')}</section>
</main></body></html>"""
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(html_doc, encoding="utf-8")
    print(f"Laporan: {args.output.resolve()}")


if __name__ == "__main__":
    main()
