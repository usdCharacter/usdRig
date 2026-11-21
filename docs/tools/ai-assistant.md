---
title: AI Assistant
---

# AI Assistant

The AI Assistant is a panel in `usdview` for chatting with Claude, OpenAI,
Grok or a local model about the open stage and its rigs. It looks at the
stage with tools rather than guessing, and with your permission it poses and
keys controls, edits the stage and runs Python. Each step it takes is listed
in the chat as it works.

It is off until you turn it on. **RigExec ▸ General Editors ▸ AI Assistant**
opens its settings the first time; check **Turn on the AI assistant** and the
panel docks on the right of the window. Nothing is sent to a provider until
you ask it something.

## The panel

1. **Provider and model.** Who answers, and which of their models. The line
   under them says which sign in or key is used. **Settings** opens the
   settings.
2. **Chats.** Every chat is kept, so **New Chat** does not lose the last one.
   Click a chat to go back to it. Right-click a chat to **Pin** it to the top
   or **Delete** it. Chats are saved on this computer only.
3. **The conversation.** Your messages and the replies. Each reply lists the
   steps it took, with a dot for each kind: blue for looking, purple for
   selecting prims or changing the frame, green for changing the stage,
   orange for running Python, and a red cross for a step that failed. What it
   changed is listed in a green box under the reply.
4. **Run Code** and **Copy Code** appear when a reply has a block of Python.
   **Run Code** runs it in usdview and needs **Run code** on.
5. **The message box.** ++ctrl+enter++ sends. While a reply is coming,
   **Send** becomes **Stop**.
6. **What it may do.** These toggles are remembered:
    - **Selection** sends the selected prims and their attributes with your
      message.
    - **Edit stage** lets it pose and key controls, make, change and delete
      prims, attributes, connections, relationships, references and layers,
      and save layers. Edits go to the stage's edit target.
    - **Run code** lets it run Python in usdview to test its work and read
      what it prints. It is off until you turn it on, and shows orange when
      on.

All the changes in one reply are one step on the RigExec undo stack, so
++ctrl+z++ takes the whole reply back. Python it runs can be undone too, as
long as it only changed the edit target. What a reply saved to a file,
selected, or switched the edit target to is listed under the reply too:
++ctrl+z++ does not take those back.

## Signing in

The settings have a section for each provider. Click a section's header to
open it; the header says how that provider is set up.

- **Claude** uses the Claude Code app's sign in, so any Claude plan works and
  usdRig never sees the account's token. **Sign in with Claude Code** runs
  Claude Code's own sign in in the browser. Claude Code needs to be
  installed.
- **OpenAI** can sign in with a ChatGPT plan, **Codex** uses the Codex app's
  sign in, and **Grok** can sign in with SuperGrok.
- Each can use an **API key** instead, or the usual environment variable
  (`ANTHROPIC_API_KEY`, `OPENAI_API_KEY`, `XAI_API_KEY`).
- **Local Model** points at a server on this computer or the network:
  Ollama, LM Studio, llama.cpp, vLLM or Jan. **Find** looks for one that is
  running.

**Ask ... first** picks the provider the panel starts with, and **Check**
asks a provider for its models. Keys and sign ins are kept in the operating
system's secure store.

## What it can do

| Kind | Tools |
|---|---|
| Look at the stage | The stage overview (layers, edit target, time range, rigs), prim outlines, a prim or attribute in full (values, keys, connections, the layers with opinions), searches by name, type, schema or attribute. |
| Look at a rig | The rigs on the stage, a control's avars, compiling a rig and reporting its errors and mover order, evaluating it at a frame for joint and control frames, and what the viewport's own evaluator last did. |
| Look at the viewport | A picture of the viewport, for the providers that can see images. |
| Show you | Select prims (and frame them), go to a frame. |
| Pose | Set and key avars, or reset them to the rig's rest. |
| Change the stage | Define, remove, rename and move prims; set, key, clear and block attributes; connections, relationships, schemas, variants, references, sublayers, the edit target; save layers. |
| Run Python | Python in usdview, with `usdviewApi`, `stage` and `rigexec` ready. |
| Read files | Files in the stage's folder, and anywhere else you allow. |

`check_rig` and `evaluate_rig` compile the rig on their own, apart from the
viewport, and author nothing. Avars that a mover recomputes are reported as
hidden: setting them changes nothing you can see, so the assistant poses the
controls that drive them instead.

With a Claude or Codex sign in, the tools are served to Claude Code or Codex
by the assistant's own MCP server. It only listens on this computer and takes
a new key for every reply. The other providers use the same tools through
their own tool calling. A model that cannot use tools answers without them,
and changes the stage with a block of changes the panel applies.

## Reading files

The assistant can read files on this computer. The open stage's folder is
free to read. Before it reads anywhere else it asks, with its reason:

- **Allow** lets it read that folder for the rest of the chat.
- **Always allow** lets it read the folder in every chat, and remembers it
  under **Memory** in the settings.
- **Deny** says no, and it does not ask again in that reply.

## Memory

Ask the assistant to remember something, such as how you name controls, and
it saves a short note. Notes are sent at the start of every chat. **Memory**
in the settings lists them and the folders it can always read. Settings,
chats and notes are kept in `~/.usdRig/assistant`; set `USDRIG_USER_DIR` to
keep them elsewhere.
