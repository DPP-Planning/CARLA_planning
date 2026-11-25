import csv
from pathlib import Path

class MetricsCollector:
    def __init__(self, algorithm):
        self.algorithm = algorithm
        self.nodes_expanded = 0
        self.replans = 0

    def record_expansion(self):
        self.nodes_expanded += 1

    def record_replan(self):
        self.replans += 1

    def save(self, filename = "metrics.csv"):
        path = Path(filename)
        new_file = not path.exists() or path.stat().st_size == 0

        with path.open("a", newline = "") as f:
            writer = csv.writer(f)

            if new_file:
                writer.writerow(["algorithm", "nodes_expanded", "replans"])

            writer.writerow([self.algorithm, self.nodes_expanded, self.replans])

        print(f"Metrics saved to {path.resolve()}")
