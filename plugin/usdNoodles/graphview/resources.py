#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""GL resource lifecycle for the graph view: the render layer owns it.

``RenderResources.initialize(context)`` / ``RenderResources.dispose(context)``
own the managers, font atlas, buffers and shaders for one owning GL context
(in production, the ``GraphView`` widget itself). Allocation and freeing
happen only while the owning context is current and valid; context
destruction is hooked so a dead context is forgotten without GL calls; and
cleanup is idempotent -- disposing twice, or disposing what was never
initialized, is a no-op. A dispose requested while the context is not
current is queued and drained on the next current ``initialize``/``dispose``
(plus a best-effort ``singleShot`` pump where Qt runs an event loop), never
run from a GC thread: this module defines no ``__del__``, so nothing here
can execute on the collector.

The first ``initialize`` adopts the view's existing CPU-side manager objects
when the widget constructor already built them, so adoption changes no
identity any legacy reader holds; atlas and shaders are always (re)created
fresh, matching the original ``initializeGL``. A legacy GL set adopted from
a view that initialized before ``RenderResources`` existed is freed before
its replacement is allocated, so re-initializing never orphans names.

Moved verbatim from ``GraphView`` as module-level functions whose first
parameter is ``view``: ``initializeGL``, ``_reinitializeGL`` and
``resizeGL``. The allocation half of ``initializeGL`` and the cleanup half
of ``_reinitializeGL`` route through ``RenderResources`` -- that ownership
move is this module's whole point -- while detection, asset resolution,
sample-graph loading and flags keep their original shape. The old
``_cleanupNodeVBOs`` body lives on as ``RenderResources._free``; its
``GraphView`` method is gone. ``_clearRenderCache`` and
``_resetRenderCachesAfterBulkMove`` stay on the view: the paint passes that
consume those caches move in later links, and the suite pins the bulk-move
helper's behavior.
"""

from pathlib import Path

from OpenGL import GL
from pxr import Glf, Tf
from UsdNoodles.render import (
    FontAtlas,
    NodeRenderManager,
    NodeTransformFrame,
    ShaderLibrary,
)

from ..linkRenderer import LinkRenderer
from ..textRenderer import TextRenderer
from ..widgets import GroupStickerRenderer


class GLContextNotCurrentError(RuntimeError):
    """The owning GL context was not current for an allocation."""


def resolve_assets_path():
    """The directory holding the editor's fonts, shaders and sample graphs.

    Prefers the installed ``UsdNoodles`` plugin resources, else the package's
    own ``assets`` directory. Kept local instead of reusing
    ``UsdNoodles._getAssetsPath``: importing the top-level package from here
    would cycle back through the widget module that imports this one.
    """
    from pxr import Plug

    plugin = Plug.Registry().GetPluginWithName("UsdNoodles")

    if plugin:
        # Plugin resource path points to: <install>/plugin/usd/usdNoodles/resources/
        resourcePath = Path(plugin.resourcePath)

        assetsInResources = resourcePath / "assets"
        if assetsInResources.exists():
            return assetsInResources
        return resourcePath
    # Same directory the pre-move body spelled as
    # ``Path(graphView.__file__).parent``: this file sits one level deeper.
    resourcePath = Path(__file__).parent.parent
    assetsPath = resourcePath / "assets"
    if assetsPath.exists():
        return assetsPath
    return resourcePath


def _context_is_valid(context):
    """Whether the owning context exists and may receive GL calls."""
    getter = getattr(context, "context", None)
    if not callable(getter):
        return True
    try:
        ctx = getter()
    except Exception:
        return False
    if ctx is None:
        return False
    is_valid = getattr(ctx, "isValid", None)
    if not callable(is_valid):
        return True
    try:
        return bool(is_valid())
    except Exception:
        return False


def _ensure_current(context):
    """Make the owning context current; False when it cannot be used.

    Mirrors the trust model of the pre-move paint path: validity is checked,
    then ``makeCurrent`` is trusted. A ``False`` return means the caller must
    queue (dispose) or refuse (initialize), never issue GL.
    """
    if not _context_is_valid(context):
        return False
    make_current = getattr(context, "makeCurrent", None)
    if callable(make_current):
        try:
            make_current()
        except Exception:
            return False
    return True


class RenderResources:
    """The GL bundle for one owning context, created per owner on demand.

    The bundle shell lives on the owner as ``_render_resources``; the owner
    holds the only strong reference, so no registry can leak it. Manager,
    icon-renderer and transform-frame objects persist across dispose /
    re-initialize cycles (only their GL state is cycled); atlas and shaders
    are recreated on every initialize, exactly as the pre-move ``initializeGL``
    did. Owner-thread confined like the widget: no locks here.
    """

    def __init__(self):
        from .render import Renderer

        self.frame_source = None
        self.frame_renderer = Renderer(self)
        self.link_config = None
        self.font_atlas = None
        self.shader_library = None
        self.node_render_manager = None
        self.group_sticker_renderer = None
        self.link_renderer = None
        self.text_renderer = None
        self.transform_frame = None
        self.icon_renderer = None
        self.assets_path = None
        self._initialized = False
        self._pending_dispose = False
        self._pump_queued = False
        self._destroy_hook_connected = False

    @property
    def initialized(self):
        """Whether a live GL set is currently allocated."""
        return self._initialized

    @property
    def pending_dispose(self):
        """Whether a dispose is queued for the next current call."""
        return self._pending_dispose

    # -- class entry points -------------------------------------------------

    @classmethod
    def initialize(cls, context, assets_path=None, factories=None):
        """Allocate (or adopt) the GL set for *context* and return it.

        Idempotent: a second call while initialized returns the same bundle
        without reallocating, and rescinds a queued dispose. Requires the
        owning context current; raises :class:`GLContextNotCurrentError`
        otherwise. Returns None when the font atlas is missing, warning
        exactly as the pre-move ``initializeGL`` did. *factories* maps
        bundle field names to nullary callables for tests; production passes
        none and gets the real constructors.
        """
        shell = getattr(context, "_render_resources", None)
        if shell is None:
            shell = cls()
            context._render_resources = shell
        return shell._initialize(context, assets_path, factories or {})

    @classmethod
    def dispose(cls, context):
        """Free the GL set for *context*; idempotent, never from GC.

        A second call, or a call with nothing allocated, is a no-op. When
        the owning context is not current the disposal is queued: it runs on
        the next current ``initialize``/``dispose`` for this owner, plus a
        best-effort ``singleShot(0)`` pump where Qt runs an event loop.
        """
        shell = getattr(context, "_render_resources", None)
        if shell is None:
            shell = cls()
            context._render_resources = shell
        shell._dispose(context)

    # -- instance lifecycle -------------------------------------------------

    def _initialize(self, context, assets_path, factories):
        if not _ensure_current(context):
            raise GLContextNotCurrentError(
                "RenderResources.initialize requires the owning GL context "
                "to be current"
            )
        if self._initialized:
            self._pending_dispose = False
            self._sync_owner_aliases(context)
            return self
        if self._pending_dispose:
            self._free(context)
            self._pending_dispose = False
            self._pump_queued = False
        if self._owns_legacy_set(context):
            self._adopt_owner_objects(context)
            self._free(context)
        self._adopt_owner_objects(context)
        self._create_missing(context, factories)
        if assets_path is None:
            assets_path = resolve_assets_path()
        self.assets_path = assets_path
        atlasTexturePath = assets_path / "fonts" / "Poppins-Regular.png"
        fontMetricsPath = assets_path / "fonts" / "Poppins-Regular.json"
        if not atlasTexturePath.exists():
            Tf.Warn(f"Font atlas not found at {atlasTexturePath}")
            return None
        self.font_atlas = self._make(factories, "font_atlas", FontAtlas)
        self.font_atlas.initialize(str(atlasTexturePath), str(fontMetricsPath))
        self.shader_library = self._make(
            factories, "shader_library", ShaderLibrary
        )
        self.shader_library.initialize(str(assets_path))
        self.icon_renderer.initialize(self.shader_library, str(assets_path))
        self.node_render_manager.initialize(self.shader_library)
        self.group_sticker_renderer.initialize(self.shader_library)
        self.link_renderer.initialize(self.shader_library)
        self.text_renderer.initialize(self.shader_library, self.font_atlas)
        # All C++ render paths share one transform frame (text + node quads +
        # icons) so they sample one transform texture and move together.
        self.node_render_manager.setTransformFrame(self.transform_frame)
        self.text_renderer.setTransformFrame(self.transform_frame)
        self.icon_renderer.setTransformFrame(self.transform_frame)
        self._sync_owner_aliases(context)
        self._hook_destroyed(context)
        self._initialized = True
        self._pending_dispose = False
        return self

    def _dispose(self, context):
        if (
            not self._initialized
            and not self._pending_dispose
            and not self._owns_legacy_set(context)
        ):
            return
        if not self._initialized and not self._pending_dispose:
            self._adopt_owner_objects(context)
        if not _ensure_current(context):
            self._pending_dispose = True
            self._queue_pump(context)
            return
        self._free(context)
        self._initialized = False
        self._pending_dispose = False
        self._pump_queued = False

    def _free(self, context):
        """Free GL state; the pre-move ``_cleanupNodeVBOs`` plus atlas/shaders.

        Manager, icon-renderer and transform-frame objects are kept for the
        next initialize (only their GL state is released); atlas and shaders
        are dropped. Runs only while the owning context is current -- the
        destroyed-hook path (``_forget``) drops references without touching GL.
        """
        for manager in (
            self.node_render_manager,
            self.group_sticker_renderer,
            self.link_renderer,
            self.transform_frame,
        ):
            if manager is not None:
                manager.cleanup()
        if self.text_renderer is not None:
            self.text_renderer._cpp.cleanup()
        if self.icon_renderer is not None and getattr(
            self.icon_renderer, "isInitialized", False
        ):
            self.icon_renderer.cleanup()
        if self.shader_library is not None:
            self.shader_library.cleanup()
        if self.font_atlas is not None:
            self.font_atlas.cleanup()
        self.font_atlas = None
        self.shader_library = None
        self._sync_owner_aliases(context)

    def _forget(self, context=None):
        """Drop a dead context's bundle without issuing any GL call.

        Runs from the owner's ``destroyed`` hook, where the context is already
        gone and every GL name is invalid. Never allocates, never frees. The
        owner aliases are cleared too, so a later dispose finds nothing and
        stays a no-op instead of freeing invalid names.
        """
        self.font_atlas = None
        self.shader_library = None
        self.node_render_manager = None
        self.group_sticker_renderer = None
        self.link_renderer = None
        self.text_renderer = None
        self.transform_frame = None
        self.icon_renderer = None
        self._initialized = False
        self._pending_dispose = False
        self._pump_queued = False
        if context is not None:
            self._sync_owner_aliases(context)
        self._pump_queued = False

    # -- helpers ------------------------------------------------------------

    @staticmethod
    def _make(factories, name, default):
        factory = factories.get(name)
        if factory is None:
            return default()
        return factory()

    def _create_missing(self, context, factories):
        if self.node_render_manager is None:
            self.node_render_manager = self._make(
                factories, "node_render_manager", NodeRenderManager
            )
        if self.group_sticker_renderer is None:
            self.group_sticker_renderer = self._make(
                factories, "group_sticker_renderer", GroupStickerRenderer
            )
        if self.link_renderer is None:
            self.link_renderer = self._make(
                factories, "link_renderer", LinkRenderer
            )
        if self.text_renderer is None:
            self.text_renderer = self._make(
                factories, "text_renderer", TextRenderer
            )
        if self.transform_frame is None:
            self.transform_frame = self._make(
                factories, "transform_frame", NodeTransformFrame
            )
        if self.icon_renderer is None:
            factory = factories.get("icon_renderer")
            if factory is None:
                from ..iconRenderer import CppIconRenderer

                self.icon_renderer = CppIconRenderer()
            else:
                self.icon_renderer = factory()

    @staticmethod
    def _owns_legacy_set(context):
        """Whether the owner holds a GL set this shell did not allocate."""
        return (
            getattr(context, "fontAtlas", None) is not None
            or getattr(context, "shaderLibrary", None) is not None
        )

    def _adopt_owner_objects(self, context):
        """Adopt the owner's live objects so no identity changes and none leak.

        Only fills shell fields that are empty; never overwrites a live
        bundle member. Managers adopted here are freed by ``_free`` before
        any re-allocation, so adopting never orphans GL names.
        """
        if self.node_render_manager is None:
            self.node_render_manager = getattr(
                context, "_nodeRenderManager", None
            )
        if self.group_sticker_renderer is None:
            self.group_sticker_renderer = getattr(
                context, "_groupStickerRenderer", None
            )
        if self.link_renderer is None:
            self.link_renderer = getattr(context, "linkRenderer", None)
        if self.text_renderer is None:
            self.text_renderer = getattr(context, "textRenderer", None)
        if self.transform_frame is None:
            self.transform_frame = getattr(
                context, "_nodeTransformFrame", None
            )
        if self.icon_renderer is None:
            self.icon_renderer = getattr(context, "_cppIconRenderer", None)
        if self.font_atlas is None:
            self.font_atlas = getattr(context, "fontAtlas", None)
        if self.shader_library is None:
            self.shader_library = getattr(context, "shaderLibrary", None)

    def _sync_owner_aliases(self, context):
        """Publish the bundle's objects under the owner's legacy attributes.

        The bundle owns lifecycle (it creates and frees); the aliases keep
        every existing reader working during the migration to the bundle.
        """
        context.fontAtlas = self.font_atlas
        context.shaderLibrary = self.shader_library
        context._nodeRenderManager = self.node_render_manager
        context._groupStickerRenderer = self.group_sticker_renderer
        context.linkRenderer = self.link_renderer
        context.textRenderer = self.text_renderer
        context._nodeTransformFrame = self.transform_frame
        context._cppIconRenderer = self.icon_renderer

    def _hook_destroyed(self, context):
        if self._destroy_hook_connected:
            return
        import weakref

        destroyed = getattr(context, "destroyed", None)
        connect = getattr(destroyed, "connect", None)
        if not callable(connect):
            return
        shell_ref = weakref.ref(self)
        def _on_destroyed(*_args):
            shell = shell_ref()
            if shell is not None:
                shell._forget(context)
        connect(_on_destroyed)
        self._destroy_hook_connected = True

    def _queue_pump(self, context):
        """Best-effort retry of a queued dispose on the event loop.

        The deterministic drain is the next current ``initialize``/``dispose``
        for this owner; the pump only shortens the wait where Qt runs a loop.
        At most one pump is ever outstanding: the flag clears only when a
        dispose actually frees, so a dead-but-undestroyed context cannot spin
        the event loop.
        """
        if self._pump_queued:
            return
        try:
            from pxr.Usdviewq.qt import QtCore
        except ImportError:
            return

        def _fire():
            shell = getattr(context, "_render_resources", None)
            if shell is None or not shell._pending_dispose:
                if shell is not None:
                    shell._pump_queued = False
                return
            RenderResources.dispose(context)

        try:
            QtCore.QTimer.singleShot(0, _fire)
        except Exception:
            return
        self._pump_queued = True


# -- moved GraphView methods (first parameter ``view``) -------------------------


def _reinitializeGL(view):
    """Handle GL context change — clean up stale resources and reinitialize."""
    view.makeCurrent()
    RenderResources.dispose(view)
    view.initialized = False
    view.fontAtlas = None
    view.shaderLibrary = None
    initializeGL(view)
    # initializeGL() calls doneCurrent() — restore context for callers
    # (paintGL needs it current for _paintGLInner)
    view.makeCurrent()
    view.linksChanged = True
    # The atlas was regenerated, so every cached glyph vertex and every
    # width-derived fit is suspect. Dropping them only costs one warmup
    # frame; keeping them risks wrong UVs.
    view._valueGlyphCache.clear()
    view._valueFitCache.clear()
    view._valueAxisWidthCache.clear()
    view._valueTextBandCache.clear()
    view._valueDigitAdvanceCache.clear()


def initializeGL(view):
    needsInit = not view.initialized

    if view.initialized:
        view.makeCurrent()
        contextChanged = False

        if view.fontAtlas and view.fontAtlas.texture:
            if not GL.glIsTexture(view.fontAtlas.texture):
                contextChanged = True
        elif view.fontAtlas is None:
            contextChanged = True

        if contextChanged:
            RenderResources.dispose(view)
            needsInit = True
            view.initialized = False
            view.fontAtlas = None
            view.shaderLibrary = None

    if not needsInit:
        return

    view.makeCurrent()

    Glf.RegisterDefaultDebugOutputMessageCallback()

    assetsPath = resolve_assets_path()
    view._assetsPath = assetsPath

    bundle = RenderResources.initialize(view, assets_path=assetsPath)
    if bundle is None:
        view.doneCurrent()
        return

    if not view.nodes:
        sampleGraphPath = assetsPath / "bodyGraph.json"
        if sampleGraphPath.exists():
            view._loadSampleGraph(str(sampleGraphPath))
            if view.nodes:
                view.frameScene()

    view.initialized = True
    view.textChanged = True
    view.doneCurrent()


def resizeGL(view, w, h):
    view.makeCurrent()
    ctx = view.context()
    if ctx is None or not ctx.isValid():
        return
    GL.glViewport(0, 0, w, h)
    # Invalidate caches on resize
    view._nodeRenderManager.invalidateAll()
    view.doneCurrent()
