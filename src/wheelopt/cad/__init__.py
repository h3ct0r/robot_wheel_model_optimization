"""Parametric geometry generation (build123d) — ADR-0003.

Responsibilities:
  - design vector -> BREP solid
  - export STEP (FEA, archival) and STL (collision, visual)
  - material region tagging for multi-material FEA
  - mass and inertia derived from BREP volume x effective density (never hard-coded)
  - manufacturability constraint checks against queryable topology

STL is never the source of truth.
"""
