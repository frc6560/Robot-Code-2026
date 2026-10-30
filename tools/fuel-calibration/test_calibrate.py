import csv
import math
import tempfile
import unittest
from pathlib import Path
from calibrate import HEADER, build


class CalibrationTest(unittest.TestCase):
    def run_build(self, transform=lambda rows: rows):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "shots.csv"
            output = Path(folder) / "launch.csv"
            rows = []
            for rpm, speed in [(2000, 5), (2500, 7), (3000, 9)]:
                for _ in range(3):
                    # Synthetic 48.36-degree launch, known horizontal displacement.
                    frames = 0.3 / (speed * math.cos(math.radians(48.36))) * 240
                    rows.append([rpm, 1500, 0.3, frames, 240, 48.36, "", ""])
            with source.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(HEADER)
                writer.writerows(transform(rows))
            report = build(source, output)
            with output.open() as handle:
                return list(csv.DictReader(handle)), report

    def test_recovers_total_speed_from_horizontal_measurements(self):
        rows, report = self.run_build()
        self.assertEqual([float(r["exit_speed_mps"]) for r in rows], [5, 7, 9])
        self.assertEqual(len(report), 3)
        self.assertEqual(rows[0]["backspin_rps"], "")

    def test_signed_spin(self):
        def add_spin(rows):
            for r in rows:
                r[6:8] = [1, 12]
            return rows
        rows, _ = self.run_build(add_spin)
        self.assertEqual(float(rows[0]["backspin_rps"]), 20)

    def test_rejects_mixed_tunnel(self):
        def change(rows):
            rows[0][1] = 2000
            return rows
        with self.assertRaises(ValueError):
            self.run_build(change)

    def test_rejects_bad_timing_and_nonfinite(self):
        for value in [0, -1, float("nan")]:
            def change(rows):
                rows[0][3] = value
                return rows
            with self.assertRaises(ValueError):
                self.run_build(change)

    def test_requires_repeats_and_rejects_large_scatter(self):
        with self.assertRaises(ValueError):
            self.run_build(lambda rows: rows[1:])
        def scatter(rows):
            rows[0][3] *= 2
            return rows
        with self.assertRaises(ValueError):
            self.run_build(scatter)


if __name__ == "__main__":
    unittest.main()
