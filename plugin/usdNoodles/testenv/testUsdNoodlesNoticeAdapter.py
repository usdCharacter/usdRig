import unittest
from unittest.mock import patch

from pxr import Tf, Usd

from UsdNoodles.usdNoticeHandler import UsdNoticeHandler


class _Receiver:
    def __init__(self):
        self.deliveries = []

    def _handleUsdChanges(self, resyncedPaths, infoChangedPaths, movedPaths):
        self.deliveries.append((resyncedPaths, infoChangedPaths, movedPaths))


class _UnreadableNotice:
    def __getattribute__(self, name):
        raise AssertionError(f"Stale notice was inspected: {name}")


class NoticeAdapterTest(unittest.TestCase):
    def setUp(self):
        self.receiver = _Receiver()
        self.handler = UsdNoticeHandler(self.receiver)
        self.addCleanup(self.handler.unregister)
        self.stageA = Usd.Stage.CreateInMemory()
        self.stageB = Usd.Stage.CreateInMemory()

    def test_failed_registration_revokes_old_sender_and_allows_retry(self):
        self.handler.register(self.stageA)
        self.assertIs(self.handler.getStage(), self.stageA)
        self.stageA.DefinePrim("/BeforeSwitch")
        self.assertEqual(self.receiver.deliveries, [({"/BeforeSwitch"}, set(), {})])
        self.receiver.deliveries.clear()

        def failRegistration(*args):
            self.assertIsNone(self.handler.getStage())
            self.stageA.DefinePrim("/DuringAcquisition")
            raise RuntimeError("Tf registration failed")

        with patch.object(Tf.Notice, "Register", side_effect=failRegistration):
            with self.assertRaisesRegex(RuntimeError, "Tf registration failed"):
                self.handler.register(self.stageB)

        self.assertIsNone(self.handler.getStage())
        self.stageA.DefinePrim("/AfterFailureA")
        self.stageB.DefinePrim("/AfterFailureB")
        self.assertEqual(self.receiver.deliveries, [])

        self.handler.register(self.stageB)
        self.assertIs(self.handler.getStage(), self.stageB)
        self.stageA.DefinePrim("/AfterRetryA")
        self.stageB.DefinePrim("/AfterRetryB")
        self.assertEqual(self.receiver.deliveries, [({"/AfterRetryB"}, set(), {})])

    def test_rejects_stale_and_unbound_senders_without_reading_notice(self):
        self.handler.register(self.stageA)
        self.handler.register(self.stageB)
        notice = _UnreadableNotice()
        with self.subTest(binding="replaced"):
            self.handler._onObjectsChanged(notice, self.stageA)

        self.handler.unregister()
        with self.subTest(binding="unbound"):
            self.handler._onObjectsChanged(notice, None)
        self.assertEqual(self.receiver.deliveries, [])

    def test_register_none_and_repeated_unregister_leave_no_sender(self):
        self.handler.unregister()
        self.handler.register(self.stageA)
        self.handler.register(None)
        self.handler.unregister()
        self.handler.unregister()

        self.assertIsNone(self.handler.getStage())
        self.stageA.DefinePrim("/AfterUnregister")
        self.assertEqual(self.receiver.deliveries, [])
