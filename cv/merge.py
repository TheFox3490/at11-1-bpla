"""A2: merge repeated observations of the same plot.

Observations from different frames are clustered in field coordinates.  A
cluster with contradictory class votes becomes U, as required by the task.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from cv.detect import Detection

MERGE_DISTANCE_M = 0.20


@dataclass
class Observation:
    oid: str
    klass: str
    x: float
    y: float
    n_obs: int
    votes: dict[str, float]
    frames: list[str]
    score: float
    details: dict = field(default_factory=dict)
    spread_m: float = 0.0

    def as_dict(self) -> dict:
        return {
            "id": self.oid,
            "class": self.klass,
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "n_obs": self.n_obs,
            "votes": {k: round(v, 4) for k, v in self.votes.items()},
            "frames": self.frames,
            "score": round(self.score, 4),
            "spread_m": round(self.spread_m, 4),
            "details": self.details,
        }


def _dedupe_frame(dets: list[Detection]) -> list[Detection]:
    """One card can yield two contours at a frame edge; keep the best."""
    out: list[Detection] = []
    for d in sorted(dets, key=lambda x: x.score, reverse=True):
        if all((d.u - o.u) ** 2 + (d.v - o.v) ** 2 > 14 ** 2 for o in out):
            out.append(d)
    return out


def merge_detections(detections: list[Detection]) -> list[Observation]:
    dets = [d for d in detections if d.x is not None and d.y is not None]
    clusters: list[list[Detection]] = []
    for d in sorted(dets, key=lambda x: x.frame_id):
        placed = False
        for cluster in clusters:
            cx = sum(o.x * o.score for o in cluster) / sum(o.score for o in cluster)
            cy = sum(o.y * o.score for o in cluster) / sum(o.score for o in cluster)
            if math.hypot(d.x - cx, d.y - cy) <= MERGE_DISTANCE_M:
                cluster.append(d)
                placed = True
                break
        if not placed:
            clusters.append([d])

    out: list[Observation] = []
    for cluster in clusters:
        # Weighted centre and per-class vote.
        total = sum(max(d.score, 1e-6) for d in cluster)
        cx = sum(d.x * max(d.score, 1e-6) for d in cluster) / total
        cy = sum(d.y * max(d.score, 1e-6) for d in cluster) / total
        votes: dict[str, float] = {}
        for d in cluster:
            votes[d.klass] = votes.get(d.klass, 0.0) + max(d.score, 1e-6)
        # Contradiction between A/B/U -> U.
        strong = {k for k, v in votes.items() if v >= 0.15 * total}
        klass = "U" if len(strong) > 1 else max(votes, key=votes.get)
        frames = sorted({d.frame_id for d in cluster if d.frame_id})
        spread = max(math.hypot(d.x - cx, d.y - cy) for d in cluster)
        out.append(Observation(oid="", klass=klass, x=cx, y=cy,
                               n_obs=len(cluster), votes=votes,
                               frames=frames, score=total / len(cluster),
                               details={"max_score": max(d.score for d in cluster)},
                               spread_m=spread))

    # Stable IDs: bottom-to-top, left-to-right in F.
    out.sort(key=lambda o: (round(o.y, 3), round(o.x, 3)))
    for i, obs in enumerate(out, 1):
        obs.oid = f"O{i:02d}"
    return out
