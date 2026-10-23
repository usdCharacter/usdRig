"""Pose weights and reader guides from the stage's published imaging snapshot."""
import ctypes

from pxr import Gf, Sdf, Tf, Usd

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError:
    from PySide2 import QtCore, QtGui, QtWidgets

import shapeEditorModel as model
import poseReaderModel as readerViz

# One panel per usdview session lives in rigExecUsdview's registry: the
# Shape Editor reads rigExecImaging through rigExecUsdview already, so its
# directory is on the module search path wherever this panel can work.
import sessionRegistry


# How often the generation counter is read. Not how often the rig is
# evaluated -- that happens only when the counter has actually moved.
POLL_MS = 150

# Bar colours. Green for a corrective doing its job, amber for one over
# 1.0 (an overshoot the RBF is allowed to produce and worth seeing), and
# red for a negative weight, which is legal when `allowNegativeWeights`
# is on but is almost always the interesting case when debugging.
_GOOD = QtGui.QColor(90, 200, 120)
_OVER = QtGui.QColor(230, 180, 70)
_UNDER = QtGui.QColor(220, 95, 95)
_NEUTRAL = QtGui.QColor(120, 125, 135)


def _Imaging():
    """The imaging library publishing the viewport snapshot."""
    try:
        from rigExecUsdview import ImagingLibraryPath
        lib = ctypes.CDLL(ImagingLibraryPath())
        lib.RigExecImaging_GetGeneration.restype = ctypes.c_longlong
        try:
            lib.RigExecImaging_GetMovedFloats.argtypes = [
                ctypes.c_char_p, ctypes.POINTER(ctypes.c_float), ctypes.c_int]
            lib.RigExecImaging_GetMovedFloats.restype = ctypes.c_int
        except AttributeError:
            pass
        return lib
    except Exception:
        return None


def _StageHandle(lib, stage):
    """`lib` bound to `stage`: rigExecUsdview's imagingHandle.

    Every read goes to THAT stage's imaging context -- its generation, its
    published weights -- never to whichever stage another usdview session
    in the same process activated last. None without the library.
    """
    if lib is None:
        return None
    try:
        import imagingHandle
    except ImportError:
        return None
    return imagingHandle.ImagingHandle(lib, stage)


class _WeightBar(QtWidgets.QStyledItemDelegate):
    """Draws a pose's weight as a bar behind its number.

    A delegate rather than a widget per row: there are 121 poses, and a
    progress bar each is 121 widgets to lay out on every refresh.
    """

    def paint(self, painter, option, index):
        weight = index.data(QtCore.Qt.UserRole)
        if weight is None:
            return super(_WeightBar, self).paint(painter, option, index)
        rect = option.rect.adjusted(2, 3, -2, -3)
        painter.save()
        painter.setPen(QtCore.Qt.NoPen)
        span = max(abs(float(weight)), 1.0)
        width = int(rect.width() * min(abs(float(weight)) / span, 1.0))
        if width > 0:
            colour = (_UNDER if weight < 0 else
                      _OVER if weight > 1.0001 else _GOOD)
            colour = QtGui.QColor(colour)
            colour.setAlpha(150)
            painter.setBrush(colour)
            painter.drawRoundedRect(
                QtCore.QRect(rect.x(), rect.y(), width, rect.height()), 2, 2)
        painter.restore()
        super(_WeightBar, self).paint(painter, option, index)


class ShapeEditorPanel(QtWidgets.QDialog):
    """Interpolators and their poses, with live weights and mutes.

    One per usdview session, filed under the session's main window:
    several sessions can share this module in one process.
    """

    _sessions = sessionRegistry.SessionRegistry("shape editor panels")

    @classmethod
    def GetInstance(cls, usdviewApi):
        panel = cls._sessions.Get(usdviewApi)
        if panel is None:
            panel = cls._sessions.Set(usdviewApi, cls(usdviewApi))
        else:
            panel._api = usdviewApi
        return panel

    def __init__(self, usdviewApi, parent=None):
        super(ShapeEditorPanel, self).__init__(
            parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        self._interpolators = []
        self._lib = _Imaging()
        self._imaging = None      # _lib bound to the stage shown
        self._generation = None
        self._items = {}          # pose path -> QTreeWidgetItem
        self._packed = None       # the path list handed to the bridge
        self._packedEntries = []
        self._buffer = None
        # The viewport visualization. Off until the group is ticked; the
        # overlay is created on first use so a closed panel costs nothing.
        self._vizSettings = readerViz.Settings()
        self._vizView = None
        self._vizSelected = set()
        self._vizViewport = set()  # interpolators the viewport selects
        # Rows picked in the tree, by prim path. Kept here rather than read
        # off the tree because the tree is REBUILT on every refresh in
        # firing-only mode and on every search keystroke, and a rebuild
        # would otherwise drop the pick the overlay is following.
        self._treePicked = set()
        # The viewport's last selection, as prim paths: what a change
        # REMOVED is how a deselect is told apart from a new pick.
        self._viewportPaths = set()
        self._vizRest = {}        # rest frames: constant while posing
        self._vizNotice = None

        self.setWindowTitle("Shape Editor")
        self.resize(560, 720)
        layout = QtWidgets.QVBoxLayout(self)

        self._search = QtWidgets.QLineEdit()
        self._search.setPlaceholderText("find a pose or interpolator...")
        self._search.setClearButtonEnabled(True)
        self._search.setToolTip(
            "Filter the tree. Matches an interpolator's name, a pose's "
            "name and its corrective target, case-insensitively; "
            "space-separated words all have to match, in any order. An "
            "interpolator whose own name matches keeps all its poses, so "
            "typing a limb gives you that limb's whole stack.")
        self._search.textChanged.connect(self._Populate)
        layout.addWidget(self._search)

        row = QtWidgets.QHBoxLayout()
        self._firingOnly = QtWidgets.QCheckBox("Firing only")
        self._firingOnly.setToolTip(
            "Show only poses carrying a weight. The neutral is never "
            "counted: at rest EVERY interpolator's neutral reads 1.0, so "
            "including it would mark all of them as firing.")
        self._firingOnly.toggled.connect(self._Populate)
        row.addWidget(self._firingOnly)
        row.addStretch(1)
        reload_ = QtWidgets.QPushButton("Reload")
        reload_.setToolTip("Re-read the interpolators off the stage.")
        reload_.clicked.connect(self.Reload)
        row.addWidget(reload_)
        layout.addLayout(row)

        self._tree = QtWidgets.QTreeWidget()
        self._tree.setColumnCount(4)
        self._tree.setHeaderLabels(["pose", "weight", "corrective", "on"])
        self._tree.setRootIsDecorated(True)
        self._tree.setUniformRowHeights(True)
        self._tree.setAlternatingRowColors(True)
        self._tree.header().setStretchLastSection(False)
        self._tree.setColumnWidth(0, 250)
        self._tree.setColumnWidth(1, 80)
        self._tree.setColumnWidth(2, 170)
        self._tree.setColumnWidth(3, 30)
        self._tree.setItemDelegateForColumn(1, _WeightBar(self._tree))
        self._tree.itemChanged.connect(self._OnItemChanged)
        self._tree.itemSelectionChanged.connect(self._OnSelected)
        layout.addWidget(self._tree, 1)

        self._BuildVizControls(layout)

        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(POLL_MS)
        self._timer.timeout.connect(self._Poll)

        self.Reload()

    # -- lifecycle -------------------------------------------------------

    def showEvent(self, event):
        # Re-read on show: the stage may have been replaced, and a panel
        # that silently describes a stage nobody is looking at is worse
        # than one that is empty.
        self.Reload()
        self._timer.start()
        return super(ShapeEditorPanel, self).showEvent(event)

    def hideEvent(self, event):
        # Nobody pays for a panel they closed. This is the whole of the
        # cost control described at the top of the file -- unless the
        # viewport visualization is on, which is still being looked at
        # with the panel tucked away, so it keeps its refresh.
        if not self._vizSettings.enabled:
            self._timer.stop()
        return super(ShapeEditorPanel, self).hideEvent(event)

    def _Stage(self):
        return getattr(self._api, "stage", None)

    def _StageImaging(self):
        """The imaging library bound to this session's current stage."""
        stage = self._Stage()
        handle = self._imaging
        if handle is None or handle.stage is not stage:
            handle = _StageHandle(self._lib, stage)
            self._imaging = handle
        return handle

    def _Poll(self):
        """Coalesce refreshes until the published generation changes."""
        imaging = self._StageImaging()
        if imaging is None:
            return
        try:
            generation = int(imaging.GetGeneration())
        except Exception:
            return
        if generation == self._generation:
            return
        self._generation = generation
        self.Refresh()

    # -- data ------------------------------------------------------------

    def Reload(self):
        stage = self._Stage()
        self._interpolators = model.Discover(stage) if stage else []
        self._packed = None       # the path list is only valid for these
        self._vizRest = {}        # and so may every rest frame
        self._Populate()
        self.Refresh()

    def Refresh(self):
        """Pull the published weights, repaint the numbers and the view."""
        try:
            self._RefreshWeights()
        finally:
            self._RefreshViz()

    def _RefreshWeights(self):
        if self._ReadFromViewport():
            return
        for interp in self._interpolators:
            for pose in interp.poses:
                pose.weight = 0.0
        if self._firingOnly.isChecked():
            self._Populate()
        else:
            self._UpdateNumbers()
        self._Status("no published weights" if self._interpolators else None)

    def _ReadFromViewport(self):
        """Take the weights straight off the viewport's own snapshot.

        Returns False when no weights are published for this stage.
        """
        imaging = self._StageImaging()
        if imaging is None or not self._interpolators:
            return False
        get = getattr(imaging, "RigExecImaging_GetMovedFloats", None)
        if get is None:
            return False
        if self._packed is None:
            # Built once and reused: the path list only changes when the
            # interpolators do, and rebuilding it per refresh would put
            # 121 string joins back on the hot path.
            entries = [e for i in self._interpolators for e in i.poses]
            self._packedEntries = entries
            self._packed = ("\n".join(
                str(e.path.AppendProperty(model.WEIGHT)) for e in entries
            )).encode("utf-8")
            self._buffer = (ctypes.c_float * len(entries))()
        try:
            found = int(get(self._packed, self._buffer, len(self._buffer)))
        except Exception:
            return False
        if found <= 0:
            return False
        for entry, value in zip(self._packedEntries, self._buffer):
            entry.weight = float(value)
        # Firing-only rows depend on the new weights.
        if self._firingOnly.isChecked():
            self._Populate()
        else:
            self._UpdateNumbers()
        self._Status(None)
        return True

    # -- tree ------------------------------------------------------------

    def _Populate(self):
        self._tree.blockSignals(True)
        self._tree.clear()
        self._items = {}
        rows = []
        firingOnly = self._firingOnly.isChecked()
        query = self._search.text() if hasattr(self, "_search") else ""
        for interp in self._interpolators:
            poses = interp.firing if firingOnly else interp.poses
            if firingOnly and not poses:
                continue
            # An interpolator whose OWN name matches keeps every pose:
            # searching for a limb should hand you that limb's whole
            # stack, not the one pose that happens to repeat its name.
            whole = model.Matches(interp.name, query)
            if not whole:
                poses = [p for p in poses
                         if model.Matches(p.name, query)
                         or model.Matches(p.target or "", query)]
                if query.split() and not poses:
                    continue
            top = QtWidgets.QTreeWidgetItem(self._tree)
            top.setText(0, interp.name)
            top.setText(2, interp.kernel or "")
            top.setFlags(top.flags() | QtCore.Qt.ItemIsUserCheckable)
            top.setCheckState(3, QtCore.Qt.Checked if interp.enabled
                              else QtCore.Qt.Unchecked)
            top.setData(0, QtCore.Qt.UserRole + 1, str(interp.path))
            rows.append(top)
            # A search that narrowed the poses opens the parent: a filter
            # whose hits are behind a closed triangle has not helped.
            top.setExpanded(firingOnly or bool(query.split()))
            for entry in poses:
                item = QtWidgets.QTreeWidgetItem(top)
                item.setText(0, entry.name)
                item.setText(2, entry.target or "")
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                item.setCheckState(3, QtCore.Qt.Checked if entry.enabled
                                   else QtCore.Qt.Unchecked)
                item.setData(0, QtCore.Qt.UserRole + 1, str(entry.path))
                if entry.is_neutral:
                    item.setForeground(0, _NEUTRAL)
                self._items[str(entry.path)] = item
                rows.append(item)
        # Put the pick back, signals still blocked so a rebuild is not
        # mistaken for the artist changing it.
        for row in rows:
            if row.data(0, QtCore.Qt.UserRole + 1) in self._treePicked:
                row.setSelected(True)
        self._tree.blockSignals(False)
        self._UpdateNumbers()

    def _UpdateNumbers(self):
        """Write the numbers, but only onto rows anybody can see.

        A collapsed interpolator's 8 poses are 8 `setText`/`setData` pairs
        and a repaint each, for text behind a closed triangle. On 121
        poses that is most of the work of a refresh spent on nothing. A
        collapsed parent is skipped wholesale and marked dirty, so
        expanding it fills in from the model without waiting for the next
        generation.
        """
        # EVERY row, expanded or not. Skipping collapsed ones was worth
        # it when a refresh meant a 15 ms evaluation of the rig; now that
        # the weights come straight off the viewport's snapshot the whole
        # update is Qt writes, and a deferred row that kept a stale number
        # is worse than a cheap one that is right.
        weights = {}
        for interp in self._interpolators:
            for entry in interp.poses:
                weights[str(entry.path)] = entry.weight
        self._tree.blockSignals(True)
        for path, item in self._items.items():
            weight = weights.get(path)
            if weight is None:
                continue
            item.setText(1, "%.4f" % weight)
            item.setData(1, QtCore.Qt.UserRole, weight)
        self._tree.blockSignals(False)
        self._Status(None)

    def _Status(self, message):
        text = model.Summarise(self._interpolators)
        self._status.setText(text if not message else "%s  --  %s"
                             % (text, message))

    # -- interaction -----------------------------------------------------

    def _OnItemChanged(self, item, column):
        """A mute toggled: author `inputs:enabled` on that prim."""
        if column != 3:
            return
        path = item.data(0, QtCore.Qt.UserRole + 1)
        stage = self._Stage()
        if not path or stage is None:
            return
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim or not prim.IsValid():
            return
        value = item.checkState(3) == QtCore.Qt.Checked
        # SESSION LAYER, so muting a corrective to see what it was doing
        # never dirties the asset. Closing usdview throws it away.
        target = Usd.EditTarget(stage.GetSessionLayer())
        with Usd.EditContext(stage, target):
            model.SetEnabled(prim, value)
        self.Refresh()

    def _OnSelected(self):
        """Select the driver joint when an interpolator row is picked."""
        items = self._tree.selectedItems()
        self._treePicked = set(
            item.data(0, QtCore.Qt.UserRole + 1) for item in items
            if item.data(0, QtCore.Qt.UserRole + 1))
        self._UpdateVizSelection()
        if not items:
            return
        path = items[0].data(0, QtCore.Qt.UserRole + 1)
        stage = self._Stage()
        if not path or stage is None:
            return
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim or not prim.IsValid():
            return
        # An interpolator selects its DRIVER, because that is the thing an
        # animator can actually grab; a pose selects itself, so the
        # property panel shows its weight and rotation.
        if prim.GetTypeName() == model.INTERPOLATOR:
            rel = prim.GetRelationship("rigExec:driver")
            targets = rel.GetTargets() if rel else []
            if targets:
                driver = stage.GetPrimAtPath(targets[0])
                if driver and driver.IsValid():
                    prim = driver
        try:
            self._api.dataModel.selection.setPrim(prim)
        except Exception:
            pass

    # -- viewport visualization ----------------------------------------------

    _PART_LABELS = (
        (readerViz.PART_CONES, "Cones",
         "Each swing pose's support as a cone at the driver, out to the "
         "pose's own rotation radius -- where its raw falloff reaches zero "
         "(linear) or e^-1 (gaussian)."),
        (readerViz.PART_CORES, "Cores",
         "The inner cone or sphere where a pose's raw falloff is still one "
         "half."),
        (readerViz.PART_TWIST, "Twist",
         "Twist poses as a fan about the driver's twist axis, centred on "
         "the pose's twist angle."),
        (readerViz.PART_SPHERES, "Spheres",
         "Each translation pose's support as a sphere around where the "
         "pose puts the driver."),
        (readerViz.PART_DRIVER, "Driver",
         "Where the driver is now: its twist axis as an arrow, or its "
         "position as a dot."),
        (readerViz.PART_LABELS, "Labels",
         "Each pose's name and its PUBLISHED weight -- the normalised "
         "number the corrective receives, which differs from the raw "
         "falloff exactly where poses overlap."),
        (readerViz.PART_OVERLAP, "Overlap",
         "A line between every pair of poses whose supports intersect: "
         "yellow where the supports meet, orange where the cores do, red "
         "where the two sit closer than a quarter of a radius -- the "
         "layout that lets normalised weights run far outside 0..1."),
    )

    def _BuildVizControls(self, layout):
        """The 'Show in viewport' group: off by default, every part on."""
        group = QtWidgets.QGroupBox("Show in viewport")
        group.setCheckable(True)
        group.setChecked(False)
        group.setToolTip(
            "Draw the interpolators' falloffs over the viewport: cones for "
            "swing poses, fans for twist poses, spheres for translation "
            "poses, and the driver where it is now. Off by default.")
        group.toggled.connect(self._OnVizToggled)
        self._vizGroup = group
        box = QtWidgets.QVBoxLayout(group)

        parts = QtWidgets.QGridLayout()
        self._vizParts = {}
        for i, (part, label, tip) in enumerate(self._PART_LABELS):
            check = QtWidgets.QCheckBox(label)
            check.setChecked(True)
            check.setToolTip(tip)
            check.toggled.connect(self._OnVizChanged)
            parts.addWidget(check, i // 4, i % 4)
            self._vizParts[part] = check
        box.addLayout(parts)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Show"))
        self._vizScope = QtWidgets.QComboBox()
        for scope, label in ((readerViz.SCOPE_SELECTED, "selected"),
                             (readerViz.SCOPE_FIRING, "firing"),
                             (readerViz.SCOPE_ALL, "all")):
            self._vizScope.addItem(label, scope)
        self._vizScope.setToolTip(
            "Which interpolators to draw. SELECTED follows the viewport "
            "selection -- a driver joint, an interpolator or one of its "
            "poses, or a row picked above. FIRING draws every interpolator "
            "with a non-neutral pose carrying weight. ALL draws everything.")
        self._vizScope.currentIndexChanged.connect(self._OnVizChanged)
        row.addWidget(self._vizScope)
        row.addSpacing(12)
        row.addWidget(QtWidgets.QLabel("Size"))
        self._vizSize = QtWidgets.QDoubleSpinBox()
        self._vizSize.setRange(1.0, 200.0)
        self._vizSize.setDecimals(1)
        self._vizSize.setSingleStep(1.0)
        self._vizSize.setValue(readerViz.READER_LENGTH)
        self._vizSize.setToolTip(
            "How long the cones and fans are drawn, in scene units -- the "
            "same for every interpolator. Display only: the falloff ANGLES "
            "are the rig's, and the spheres are the rig's own radii.")
        self._vizSize.valueChanged.connect(self._OnVizSizeChanged)
        row.addWidget(self._vizSize)
        row.addWidget(QtWidgets.QLabel("Opacity"))
        self._vizOpacity = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._vizOpacity.setRange(5, 100)
        self._vizOpacity.setValue(int(self._vizSettings.opacity * 100))
        self._vizOpacity.setToolTip(
            "Base opacity of the fills. A pose carrying weight is drawn "
            "brighter than this; one at rest, fainter.")
        self._vizOpacity.valueChanged.connect(self._OnVizChanged)
        row.addWidget(self._vizOpacity, 1)
        box.addLayout(row)

        self._vizInfo = QtWidgets.QLabel("")
        self._vizInfo.setWordWrap(True)
        box.addWidget(self._vizInfo)
        layout.addWidget(group)

        # The viewport selection drives the SELECTED scope.
        try:
            self._api.dataModel.selection.signalPrimSelectionChanged.connect(
                self._OnViewportSelection)
        except Exception:
            pass

    def _ReadVizSettings(self):
        settings = self._vizSettings
        settings.enabled = bool(self._vizGroup.isChecked())
        for part, check in self._vizParts.items():
            settings.parts[part] = check.isChecked()
        settings.scope = self._vizScope.currentData() or \
            readerViz.SCOPE_SELECTED
        settings.size = self._vizSize.value() / readerViz.READER_LENGTH
        settings.opacity = self._vizOpacity.value() / 100.0
        return settings

    def _VizView(self):
        """The overlay controller, made on first use."""
        if self._vizView is None:
            try:
                import poseReaderOverlay
                self._vizView = poseReaderOverlay.PoseReaderView(
                    self._api, self)
            except Exception as error:
                Tf.Warn("shapeEditor: no viewport overlay: %s" % error)
                return None
        return self._vizView

    def _OnVizToggled(self, on):
        if on:
            self._WatchRest(True)
            self._OnViewportSelection()
            if not self._timer.isActive():
                self._timer.start()
        else:
            self._WatchRest(False)
            if not self.isVisible():
                self._timer.stop()
        self._OnVizChanged()

    def _OnVizSizeChanged(self, *args):
        # The size feeds the readers' lengths, so they are rebuilt, not
        # only reprojected.
        self._OnVizChanged()

    def _OnVizChanged(self, *args):
        settings = self._ReadVizSettings()
        view = self._VizView() if settings.enabled else self._vizView
        if view is not None:
            view.SetSettings(settings)
        self._RefreshViz()

    def _OnViewportSelection(self, *args):
        """Map the viewport selection to interpolator paths.

        From the model the panel already holds rather than a stage
        traversal: an interpolator selects itself, a pose its
        interpolator, a driver every interpolator it drives.
        """
        try:
            selected = set(Sdf.Path(str(p.GetPath()))
                           for p in self._api.selectedPrims)
        except Exception:
            selected = set()
        added = selected - self._viewportPaths
        removed = self._viewportPaths - selected
        self._viewportPaths = selected
        if removed and not added:
            # A pure DESELECT -- Ctrl+right-click, Ctrl+click, a click on
            # empty space -- unpicks what it removed here too. Anything
            # that adds (selecting the arm to pose it) leaves the panel's
            # pick alone: that is the case the pick exists for.
            self._Unpick(self._InterpolatorsIn(removed))
        self._vizViewport = self._InterpolatorsIn(selected)
        self._UpdateVizSelection()

    def _Unpick(self, interpolators):
        """Drop every tree row that names one of `interpolators`."""
        if not interpolators:
            return
        keep = set(p for p in self._treePicked
                   if not (self._InterpolatorsIn({Sdf.Path(p)}) &
                           interpolators))
        if keep == self._treePicked:
            return
        self._treePicked = keep
        self._tree.blockSignals(True)
        try:
            for item in self._tree.selectedItems():
                if item.data(0, QtCore.Qt.UserRole + 1) not in keep:
                    item.setSelected(False)
        finally:
            self._tree.blockSignals(False)

    def _InterpolatorsIn(self, paths):
        """The interpolators a set of prim paths names, from the model."""
        found = set()
        for interp in self._interpolators:
            if interp.path in paths or interp.driver in paths:
                found.add(interp.path)
                continue
            if any(p.path in paths for p in interp.poses):
                found.add(interp.path)
        return found

    def _UpdateVizSelection(self):
        """SELECTED draws what the tree picks AND what the viewport does.

        The union, so a row picked here stays drawn while the artist
        clicks a control in the viewport to pose it -- which replaces the
        viewport selection, and without the tree's half would blank the
        very overlay they are posing against.
        """
        picked = set(Sdf.Path(p) for p in self._treePicked)
        self._vizSelected = self._InterpolatorsIn(picked) | \
            self._vizViewport
        if self._vizSettings.enabled:
            self._RefreshViz()

    def _WatchRest(self, on):
        """Clear the rest-frame cache when a rest channel changes.

        Rest frames are cached because they do not move while a pose is
        dragged; anything that CAN move one -- a rest:* channel, an
        intervening transform, a resync -- throws the cache away.
        """
        if on and self._vizNotice is None:
            self._vizNotice = Tf.Notice.RegisterGlobally(
                Usd.Notice.ObjectsChanged, self._OnObjectsChanged)
        elif not on and self._vizNotice is not None:
            self._vizNotice.Revoke()
            self._vizNotice = None

    def _OnObjectsChanged(self, notice, sender):
        if sender is not self._Stage() or not self._vizRest:
            return
        if notice.GetResyncedPaths():
            self._vizRest = {}
            return
        for path in notice.GetChangedInfoOnlyPaths():
            name = path.name if path.IsPropertyPath() else ""
            if name.startswith("rest:") or name.startswith("xformOp") or \
                    name == "xformOpOrder":
                self._vizRest = {}
                return

    def _VizFrames(self):
        """Read asset-space frames from this stage's current publication."""
        imaging = self._StageImaging()
        if imaging is None or not imaging.Has("GetControlFrameAssetSpace"):
            self._vizInfo.setText("No published driver frames available.")
            return None
        time = self._api.frame if self._api.frame is not None else Usd.TimeCode.Default()
        frame = 0.0 if time.IsDefault() else time.GetValue()
        frames = {}

        def read(path):
            path = str(path)
            if path not in frames:
                buffer = (ctypes.c_double * 16)()
                found = imaging.GetControlFrameAssetSpace(
                    path, frame, int(time.IsDefault()), buffer)
                frames[path] = Gf.Matrix4d(*buffer) if found else None
            return frames[path]
        return read

    def _RefreshViz(self):
        settings = self._vizSettings
        view = self._vizView
        if not settings.enabled:
            if view is not None:
                view.SetReaders([])
            return
        stage = self._Stage()
        if stage is None or view is None:
            return
        if settings.scope == readerViz.SCOPE_ALL:
            chosen = list(self._interpolators)
        elif settings.scope == readerViz.SCOPE_FIRING:
            chosen = [i for i in self._interpolators if i.firing]
        else:
            chosen = [i for i in self._interpolators
                      if i.path in self._vizSelected]
        if not chosen:
            view.SetReaders([])
            self._vizInfo.setText(
                "Select a driver joint, an interpolator or a pose to see "
                "it." if settings.scope == readerViz.SCOPE_SELECTED else
                "Nothing to draw: no interpolator is firing."
                if settings.scope == readerViz.SCOPE_FIRING else
                "No interpolators on this stage.")
            return
        frames = self._VizFrames()
        if frames is None:
            view.SetReaders([])
            return
        try:
            import gizmoMath
            restSpace = gizmoMath.RestSpace
        except Exception as error:
            self._vizInfo.setText("no rest frames: %s" % error)
            return
        time = self._api.frame if self._api.frame is not None else Usd.TimeCode.Default()
        readers = readerViz.Build(stage, chosen, frames, time,
                                  restSpace, settings.size, self._vizRest)
        drawn = readerViz.Visible(readers, settings,
                                  [r.path for r in readers])
        view.SetReaders(drawn)
        counts = readerViz.OverlapSummary(drawn)
        skipped = [r for r in readers if not r.drawable]
        text = "%d drawn" % len(drawn)
        if any(counts.values()):
            text += "; overlapping pairs: %d coincident, %d cores, " \
                    "%d supports" % (counts[readerViz.OVERLAP_COINCIDENT],
                                     counts[readerViz.OVERLAP_CORES],
                                     counts[readerViz.OVERLAP_SUPPORTS])
        if skipped:
            text += "; not drawn: " + ", ".join(
                "%s (%s)" % (r.name, r.note) for r in skipped[:3])
            if len(skipped) > 3:
                text += ", +%d more" % (len(skipped) - 3)
        self._vizInfo.setText(text)
