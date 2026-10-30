"""Export a readable comparison and plots from a completed Webots Optuna run."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    report = json.loads((args.run / "report.json").read_text(encoding="utf-8"))
    lines = ["# Webots gerilim kontrolcüsü: Optuna karşılaştırması", "",
             f"Deneme sayısı: {report['trials']}. Arama tohumu: {report['seed']}.", "",
             "| Kazanç | Referans | Aday |", "|---|---:|---:|"]
    for key, old in report["baseline_params"].items():
        lines.append(f"| {key} | {old:.6g} | {report['candidate_params'][key]:.6g} |")
    old, new = report['validation_baseline_cost'], report['validation_candidate_cost']
    lines += ["", f"Bağımsız doğrulama maliyeti: {old:.6f} -> {new:.6f}; azalma %{100*(1-new/old):.2f}.",
              f"Kabul koşulları sağlandı: {report['accepted']}.", "",
              "| Tohum / senaryo | Maliyet eski / aday | Tepe açı eski / aday (derece) | Son 3 s RMS hız eski / aday (m/s) | Son 3 s tepe hız eski / aday (m/s) |",
              "|---|---:|---:|---:|---:|"]
    for key, base in report['validation_baseline_metrics'].items():
        candidate = report['validation_candidate_metrics'][key]
        if 'failure' in base or 'failure' in candidate:
            lines.append(f"| {key} | Hata: {base.get('failure', candidate.get('failure'))} | - | - | - |")
            continue
        pairs = [f"{base[k]:.5f} / {candidate[k]:.5f}" for k in ['cost', 'peak_pitch', 'tail_rms_speed', 'tail_max_speed']]
        lines.append(f"| {key} | " + " | ".join(pairs) + " |")
    lines += ["", "Maliyet ağırlıklı bir değerlendirmedir; tek başına bütün metriklerin iyileştiği anlamına gelmez.",
              "Doğrulama kapsamı: statik denge, iki yönlü 1.5 N itme, ileri/geri sürüş ve frenleme. Dönüş ve donanım doğrulanmadı."]
    (args.run / "comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    for seed in [report['seed'] + 100, report['seed'] + 200]:
        fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharex='col', constrained_layout=True)
        for col, scenario in enumerate(['standing', 'push', 'drive']):
            for label, title in [('baseline_validation', 'Referans'), ('candidate_validation', 'Optuna')]:
                path = args.run / label / f'seed_{seed}_{scenario}' / 'evaluation.csv'
                with path.open() as stream:
                    rows = list(csv.DictReader(stream))
                a = {k: np.array([float(row[k]) for row in rows]) for k in ['time', 'pitch', 'velocity', 'target']}
                axes[0, col].plot(a['time'], a['pitch'], label=title, linewidth=.8, alpha=.8)
                axes[1, col].plot(a['time'], a['velocity'], label=title, linewidth=.8, alpha=.8)
            axes[1, col].plot(a['time'], a['target'], 'k--', label='Hedef', linewidth=1)
            axes[0, col].set_title({'standing': 'Statik denge', 'push': 'İtme', 'drive': 'Sürüş ve frenleme'}[scenario])
            axes[1, col].set_xlabel('Zaman (s)')
            for row in range(2):
                axes[row, col].grid(alpha=.2)
        axes[0, 0].set_ylabel('Eğim (derece)')
        axes[1, 0].set_ylabel('Hız (m/s)')
        axes[1, 2].legend()
        fig.suptitle(f'Webots bağımsız doğrulama — tohum {seed}')
        fig.savefig(args.run / f'comparison_seed_{seed}.png', dpi=160)
        plt.close(fig)
    print(args.run / 'comparison.md')


if __name__ == '__main__':
    main()
