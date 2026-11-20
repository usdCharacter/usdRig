#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Owner-local suppression without a widget or a registered USD stage."""

import unittest

from UsdNoodles.graphview.stage_sync import StageSync


class _RecordingHandler:
    def __init__(self, enabled):
        self.enabled = enabled
        self.reads = 0
        self.writes = []

    def isEnabled(self):
        self.reads += 1
        return self.enabled

    def setEnabled(self, enabled):
        self.enabled = enabled
        self.writes.append(enabled)

    def unregister(self):
        pass


def _unexpected(*args):
    raise AssertionError("Suppression must not access or schedule view work")


class SuppressionTokensTest(unittest.TestCase):
    def setUp(self):
        self.first = _RecordingHandler(True)
        self.second = _RecordingHandler(False)
        self.owner = self._makeOwner(self.first)
        self.other = self._makeOwner(self.second)

    def _makeOwner(self, handler):
        owner = StageSync(
            read=_unexpected,
            notice=_unexpected,
            apply=_unexpected,
            post=_unexpected,
            listen=lambda receiver: handler,
        )
        self.addCleanup(lambda: None if owner.isClosed() else owner.close())
        return owner

    def test_outOfOrderAndDuplicateRelease(self):
        older = self.owner.acquireSuppression()
        newer = self.owner.acquireSuppression()
        older.release()
        older.release()
        self.assertFalse(self.first.enabled)
        self.assertEqual(self.first.writes, [False])
        self.assertEqual(self.first.reads, 1)
        self.assertEqual(self.second.writes, [])

        newer.release()
        newer.release()
        self.assertTrue(self.first.enabled)
        self.assertEqual(self.first.writes, [False, True])

    def test_nestedScopesRestoreOriginalDisabledState(self):
        with self.other.suppressed():
            with self.other.suppressed():
                self.assertFalse(self.second.enabled)
                self.assertTrue(self.first.enabled)
            self.assertEqual(self.second.writes, [False])
        self.assertFalse(self.second.enabled)
        self.assertEqual(self.second.writes, [False, False])
        self.assertEqual(self.second.reads, 1)
        self.assertEqual(self.first.writes, [])

    def test_exceptionReleasesOnlyTheScopeToken(self):
        token = self.owner.acquireSuppression()
        with self.assertRaisesRegex(RuntimeError, "edit failed"):
            with self.owner.suppressed():
                raise RuntimeError("edit failed")
        self.assertFalse(self.first.enabled)
        token.release()
        self.assertTrue(self.first.enabled)
        self.assertEqual(self.first.writes, [False, True])

    def test_closeInvalidatesLiveTokensWithoutAffectingOtherOwner(self):
        otherToken = self.other.acquireSuppression()
        older = self.owner.acquireSuppression()
        with self.owner.suppressed():
            self.owner.close()
            older.release()
        older.release()
        self.owner.acquireSuppression().release()

        self.assertFalse(self.first.enabled)
        self.assertEqual(self.first.writes, [False])
        self.assertEqual(self.second.writes, [False])
        otherToken.release()
        self.assertEqual(self.second.writes, [False, False])
