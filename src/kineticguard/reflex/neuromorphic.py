"""
Neuromorphic Event-Camera Spike Stream Parser for Sub-Millisecond Reflexes.
Maintains continuous-time exponential surfaces for microsecond obstacle looming detection.
"""

from typing import List, Tuple, Dict, Any, Optional
import math


class NeuromorphicReflexSensor:
    """Ingests microsecond (x, y, timestamp_us, polarity) spikes to detect looming obstacles."""

    def __init__(self, resolution: Tuple[int, int] = (128, 128), tau_decay_us: float = 20000.0):
        self.width, self.height = resolution
        self.tau_decay_us = tau_decay_us
        # Time-surface: stores last spike timestamp per pixel
        self.surface: Dict[Tuple[int, int], int] = {}
        self.recent_spike_count: int = 0
        self.looming_detected: bool = False

    def ingest_spikes(self, spikes: List[Tuple[int, int, int, int]]) -> bool:
        """
        Ingests a batch of DVS events: [(x, y, timestamp_us, polarity), ...]
        Returns True if a rapid looming obstacle was detected.
        """
        if not spikes:
            return False

        current_t = spikes[-1][2]
        looming_energy = 0.0

        # Center region of vision field (e.g. middle 40%)
        cx_min, cx_max = int(self.width * 0.3), int(self.width * 0.7)
        cy_min, cy_max = int(self.height * 0.3), int(self.height * 0.7)

        for x, y, t_us, pol in spikes:
            if 0 <= x < self.width and 0 <= y < self.height:
                self.surface[(x, y)] = t_us
                if cx_min <= x <= cx_max and cy_min <= y <= cy_max:
                    # Exponential decay weighting
                    dt = max(0, current_t - t_us)
                    weight = math.exp(-dt / self.tau_decay_us)
                    looming_energy += weight

        # Trigger looming reflex if energy exceeds density threshold in center view
        density_threshold = 45.0
        self.looming_detected = looming_energy > density_threshold
        return self.looming_detected

    def get_looming_state(self) -> Dict[str, Any]:
        return {
            "looming_detected": self.looming_detected,
            "tracked_pixels": len(self.surface),
            "tau_decay_us": self.tau_decay_us
        }
