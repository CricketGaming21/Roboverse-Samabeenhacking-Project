"""
search_planner.py — v6. Grid coverage + loop detection + seek unvisited.
"""

import math
import time
from collections import Counter
import config as C


class SearchPlanner:

    def __init__(self):
        self.n_min = C.GRID_ORIGIN_N
        self.n_max = C.GRID_END_N
        self.e_min = C.GRID_ORIGIN_E
        self.e_max = C.GRID_END_E
        self.rows = int((self.n_max - self.n_min) / C.GRID_CELL_SIZE_M)
        self.cols = int((self.e_max - self.e_min) / C.GRID_CELL_SIZE_M)

        self.visited = set()
        self.visit_counts = {}
        self.waypoints = []
        self.wp_index = 0
        self._last_pos = None
        self._last_move_time = time.monotonic()

        self._recent_cells = []
        self._recent_max = 30
        self._loop_detected = False
        self._loop_break_until = 0.0

    def reset(self, start_north=0.0, start_east=0.0):
        self.visited.clear()
        self.visit_counts.clear()
        self._recent_cells.clear()
        self._loop_detected = False
        self._loop_break_until = 0.0
        raw = self._generate_grid()
        raw.sort(key=lambda wp: math.hypot(wp[0] - start_north, wp[1] - start_east))
        self.waypoints = raw
        self.wp_index = 0
        self._last_move_time = time.monotonic()
        print(f"  [Search] {len(self.waypoints)} waypoints "
              f"({self.rows}x{self.cols} grid)")

    def _generate_grid(self):
        wps = []
        for row in range(self.rows):
            north = self.n_min + row * C.GRID_CELL_SIZE_M + C.GRID_CELL_SIZE_M / 2.0
            for col in range(self.cols):
                east = self.e_min + col * C.GRID_CELL_SIZE_M + C.GRID_CELL_SIZE_M / 2.0
                wps.append((round(north, 1), round(east, 1)))
        return wps

    def _cell(self, north, east):
        row = int((north - self.n_min) / C.GRID_CELL_SIZE_M)
        col = int((east  - self.e_min) / C.GRID_CELL_SIZE_M)
        row = max(0, min(self.rows - 1, row))
        col = max(0, min(self.cols - 1, col))
        return row, col

    def mark_visited(self, north, east):
        if (self.n_min <= north <= self.n_max and
            self.e_min <= east  <= self.e_max):
            cell = self._cell(north, east)
            self.visited.add(cell)
            self.visit_counts[cell] = self.visit_counts.get(cell, 0) + 1
            if not self._recent_cells or self._recent_cells[-1] != cell:
                self._recent_cells.append(cell)
                if len(self._recent_cells) > self._recent_max:
                    self._recent_cells.pop(0)

    def coverage_pct(self):
        total = self.rows * self.cols
        return len(self.visited) / total * 100.0 if total > 0 else 0.0

    def detect_loop(self):
        if time.monotonic() < self._loop_break_until:
            return False
        if len(self._recent_cells) < 15:
            return False
        counts = Counter(self._recent_cells)
        max_repeats = max(counts.values())
        if max_repeats >= 3:
            if not self._loop_detected:
                self._loop_detected = True
                self._loop_break_until = time.monotonic() + 15.0
                most_common = counts.most_common(1)[0]
                print(f"  🔁 LOOP: cell {most_common[0]} visited {most_common[1]}x")
                print(f"     Mirroring turns for 15s")
            return True
        self._loop_detected = False
        return False

    def in_loop_break(self):
        if time.monotonic() < self._loop_break_until:
            return True
        if self._loop_detected:
            self._loop_detected = False
            self._recent_cells.clear()
            print(f"  ✅ Loop break done")
        return False

    def find_unvisited_direction(self, north, east, yaw_deg):
        best_dir = None
        best_count = 0
        for offset in [-135, -90, -45, 0, 45, 90, 135, 180]:
            angle_rad = math.radians(yaw_deg + offset)
            count = 0
            for dist in [C.GRID_CELL_SIZE_M, C.GRID_CELL_SIZE_M * 2]:
                cn = north + dist * math.cos(angle_rad)
                ce = east  + dist * math.sin(angle_rad)
                if (self.n_min <= cn <= self.n_max and
                    self.e_min <= ce <= self.e_max):
                    if self._cell(cn, ce) not in self.visited:
                        count += 1
            if count > best_count:
                best_count = count
                best_dir = offset
        return best_dir if best_count > 0 else None

    def time_in_visited(self, north, east):
        cell = self._cell(north, east)
        return self.visit_counts.get(cell, 0)

    def get_current_goal(self):
        while self.wp_index < len(self.waypoints):
            n, e = self.waypoints[self.wp_index]
            if self._cell(n, e) not in self.visited:
                return n, e
            self.wp_index += 1
        return None

    def advance(self):
        if self.wp_index < len(self.waypoints):
            self.wp_index += 1

    def check_stuck(self, north, east):
        pos = (north, east)
        if self._last_pos is not None:
            moved = math.hypot(pos[0] - self._last_pos[0],
                               pos[1] - self._last_pos[1])
            if moved > C.STUCK_DIST_M:
                self._last_move_time = time.monotonic()
        self._last_pos = pos
        return (time.monotonic() - self._last_move_time) > C.STUCK_TIMEOUT_S

    def reset_stuck(self):
        self._last_move_time = time.monotonic()
