import unittest
from dataclasses import FrozenInstanceError
from types import SimpleNamespace

from pxr import Sdf, Usd

from UsdNoodles.graphview import intents


class TestCommandRegistry(unittest.TestCase):
    def setUp(self):
        self.registry = intents.CommandRegistry()

    def testDispatchReturnsHandlerResult(self):
        payload = intents.Navigate("backward")
        self.registry.register(intents.Navigate, lambda intent: intent.direction)
        self.assertEqual(self.registry.dispatch(payload), "backward")

    def testDuplicateRegistrationIsRefused(self):
        self.registry.register(intents.Undo, lambda _: "original")
        with self.assertRaises(ValueError):
            self.registry.register(intents.Undo, lambda _: "replacement")
        self.assertEqual(self.registry.dispatch(intents.Undo()), "original")

    def testUnknownRegistrationIsRefused(self):
        class Other(intents.Undo):
            pass

        with self.assertRaises(TypeError):
            self.registry.register(Other, lambda _: None)

    def testUnregisteredDispatchIsRefused(self):
        with self.assertRaises(LookupError):
            self.registry.dispatch(intents.Undo())

    def testHandlerFailurePropagates(self):
        failure = RuntimeError("handler failed")

        def fail(_):
            raise failure

        self.registry.register(intents.Undo, fail)
        with self.assertRaises(RuntimeError) as caught:
            self.registry.dispatch(intents.Undo())
        self.assertIs(caught.exception, failure)

    def testPayloadIsFrozen(self):
        payload = intents.Navigate("forward")
        with self.assertRaises(FrozenInstanceError):
            payload.direction = "backward"

    def testGraphViewRegistryUsesItsViewHandlers(self):
        from UsdNoodles.graphView import GraphView

        cases = (
            (intents.SelectAll(), "selectAll"),
            (intents.ClearSelection(), "_clearSelectionAndSync"),
            (intents.Navigate("backward"), "navigateBackward"),
            (intents.Navigate("forward"), "navigateForward"),
            (intents.FinishNodeDrag(), "_finalizeNodeDrag"),
            (intents.AutoLayout(), "_autoLayoutNodes"),
            (intents.OpenContainer(), "_openSelectedContainer"),
            (intents.SaveStage(), "_saveStage"),
            (intents.Undo(), "_performUndo"),
            (intents.Redo(), "_performRedo"),
            (intents.DeleteSelection(), "_deleteAllSelected"),
        )
        for intent, method in cases:
            with self.subTest(intent=intent):
                view = SimpleNamespace()
                registry = GraphView._makeCommandRegistry(view)
                setattr(view, method, lambda: method)
                self.assertEqual(registry.dispatch(intent), method)

    def testLinkIntentsPreserveEndpointMetadata(self):
        from UsdNoodles.graphView import GraphView

        ends = intents.LinkEndpoints("/Source", "out", "/Target", "in",
                                     "outputs:out", "inputs:in")
        for intent, method in ((intents.Connect(ends), "_createConnection"),
                               (intents.Disconnect(ends), "_deleteConnection")):
            with self.subTest(intent=intent):
                view = SimpleNamespace(**{method: lambda *args, **kwargs: (args, kwargs)})
                self.assertEqual(GraphView._makeCommandRegistry(view).dispatch(intent), (
                    ("/Source", "out", "/Target", "in"),
                    {"sourcePropertyName": "outputs:out", "targetPropertyName": "inputs:in"},
                ))


class TestLinkRecords(unittest.TestCase):
    def testSerializedRecordMatchesAcrossLayerChange(self):
        from UsdNoodles.graphView import GraphView

        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Source")
        stage.DefinePrim("/Target")
        record = intents._makeSerializedLinkRecord(
            "/Source", "out", "/Target", "in",
            sourcePropertyName="outputs:out", targetPropertyName="inputs:in",
        )
        expected = {
            "sourceNodeId": "/Source", "sourcePort": "out",
            "targetNodeId": "/Target", "targetPort": "in",
            "sourcePropertyName": "outputs:out", "targetPropertyName": "inputs:in",
        }
        self.assertEqual(intents._deserializeLinkRecord(record), expected)
        self.assertEqual(intents._deserializeLinkRecord(tuple(expected.values())), expected)

        original = SimpleNamespace(**expected, layer=stage.GetEditTarget().GetLayer().identifier)
        self.assertTrue(GraphView._linkMatchesRecord(original, record))
        layer = Sdf.Layer.CreateAnonymous("other.usda")
        stage.GetRootLayer().subLayerPaths.append(layer.identifier)
        stage.SetEditTarget(layer)
        moved = SimpleNamespace(**expected, layer=layer.identifier)
        self.assertNotEqual(original.layer, moved.layer)
        self.assertTrue(GraphView._linkMatchesRecord(moved, record))
        moved.targetPropertyName = "inputs:other"
        self.assertFalse(GraphView._linkMatchesRecord(moved, record))
