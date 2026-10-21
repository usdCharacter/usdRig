"""Check attachment rods in usdview's terminal scene index.

Run with bin/test/run_testusdview_cloth_guides.sh (or .bat) using
cloth_materials.usda.
"""
from pxr import Sdf, Usd
from pxr.Usdviewq._usdviewq import HydraObserver


def testUsdviewInputFunction(app):
    mesh = Sdf.Path('/ClothAsset/Geom/Curtain')
    constraint = Sdf.Path('/ClothAsset/Rig/Movers/CurtainCloth/Hem')
    child = constraint.AppendChild('rigGuideCtrl')
    model = app._dataModel
    model.viewSettings.displayGuide = True
    observer = HydraObserver()
    observer.TargetToNamedSceneIndex(HydraObserver.GetRegisteredSceneIndexNames()[-1])

    def value(data, *names):
        for name in names:
            data = data.Get(name)
            assert data, names
        return data.GetValue(0.0)

    for frame in (1, 12, 1):
        model.currentFrame = Usd.TimeCode(frame)
        app._stageView.updateGL()
        app._processEvents()
        prim_type, guide = observer.GetPrim(child)
        assert prim_type == 'basisCurves', (prim_type, frame)
        _, geometry = observer.GetPrim(mesh)
        rods = value(guide, 'primvars', 'points', 'primvarValue')
        points = value(geometry, 'primvars', 'points', 'primvarValue')
        assert value(guide, 'purpose', 'purpose') == 'guide'
        # HydraObserver exposes the opaque OriginPath value as text.
        assert str(value(guide, 'primOrigin', 'scenePath')) == (
            'HdPrimOriginSchema::OriginPath(%s)' % constraint)
        profile = model.stage.GetPrimAtPath(constraint)
        ends = []
        for relationship in ('rigExec:sourceBinding', 'rigExec:targetBinding'):
            binding = model.stage.GetPrimAtPath(profile.GetRelationship(relationship).GetTargets()[0])
            assert binding.GetAttribute('rigExec:mode').Get() == 'vertex'
            ends.append(binding.GetAttribute('rigExec:indices').Get())
        assert len(rods) == 2 * len(ends[0])
        for i, (source, target) in enumerate(zip(*ends)):
            assert (rods[2 * i] - points[source]).GetLength() < 1e-4
            assert (rods[2 * i + 1] - points[target]).GetLength() < 1e-4

    print('RIGEXEC_CLOTH_GUIDES_OK', flush=True)
