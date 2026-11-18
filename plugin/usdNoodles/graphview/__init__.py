"""Layered GraphView package (issue #54).

Phase 1 only: this package holds the authoritative method-to-layer map for
the ``GraphView`` god-class. Phase 2 moves the clusters (render, editing
first) into modules here, and later phases turn the map into import-boundary
rules. Do not import ``graphView`` from here yet.
"""
