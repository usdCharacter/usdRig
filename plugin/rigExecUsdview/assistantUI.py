"""
The AI Assistant in usdview: a panel docked on the main window, one per
session, that chats about the open stage and works on it through the tools
in usdRigAssistant.tools. Claude Code and Codex sign ins reach the same
tools through the assistant's local MCP server.
"""

from pxr.Usdviewq.qt import QtCore

import sessionRegistry
from usdRigAssistant import panel as panelModule
from usdRigAssistant import providers, session as sessionModule
from usdRigAssistant.settings_dialog import AiSettingsDialog

_panels = sessionRegistry.SessionRegistry("usdRig assistant panels")
_dialogs = sessionRegistry.SessionRegistry("usdRig assistant settings")


def _Session(usdviewApi, container):
    return sessionModule.Session(usdviewApi, container)


def OpenSettings(usdviewApi, container=None):
    """The assistant's settings: turning it on, sign ins, keys, memory."""
    dialog = _dialogs.Get(usdviewApi)
    if dialog is None:
        dialog = AiSettingsDialog(parent=usdviewApi.qMainWindow)

        def _Enabled(enabled):
            panel = _panels.Get(usdviewApi)
            if enabled:
                OpenAssistantPanel(usdviewApi, container)
            elif panel is not None:
                panel.close()

        def _Changed():
            panel = _panels.Get(usdviewApi)
            if panel is not None:
                panel.refresh_providers()

        dialog.enabled_changed.connect(_Enabled)
        dialog.settings_changed.connect(_Changed)
        _dialogs.Set(usdviewApi, dialog)
    dialog.show()
    dialog.raise_()
    return dialog


def OpenAssistantPanel(usdviewApi, container=None):
    """Show the session's assistant panel, made on first use. Until the
    assistant is turned on, the settings open instead."""
    if not providers.is_enabled():
        return OpenSettings(usdviewApi, container)
    panel = _panels.Get(usdviewApi)
    if panel is None:
        main = usdviewApi.qMainWindow
        panel = panelModule.AiAssistant(
            session=_Session(usdviewApi, container), parent=main)
        panel.open_settings.connect(
            lambda: OpenSettings(usdviewApi, container))
        main.addDockWidget(QtCore.Qt.RightDockWidgetArea, panel)
        _panels.Set(usdviewApi, panel)
    panel.show()
    panel.raise_()
    return panel


def StageReplaced(usdviewApi, container=None):
    """Point an open panel at the session's new stage."""
    panel = _panels.Get(usdviewApi)
    if panel is not None:
        panel.set_session(_Session(usdviewApi, container))
