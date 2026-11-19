"""Layered decomposition of the graph view (issue #54).

The package holds the layers the ``GraphView`` god-class is being separated
into, starting with the shared path-identity contract in ``model``.

Dependency direction: ``model`` is the base and imports nothing, and no
module in this package may import ``graphView``, the widget, or Qt/GL.
``graphView.py`` may import from this package, never the reverse.
"""
