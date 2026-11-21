#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

from UsdNoodles.core import LinkData
from pxr import Tf

from .models import NodeModel
from .nodeGraph import createNodeRenderer, NodeGraph
from .pinUtils import collect_links_for_prim
from .scopeEndpoints import (
    accept_blueprint_child,
    endpoint_origins,
    endpoint_position,
    endpoint_specs,
    scope_child_connections,
    scope_pin_names,
)


def _append_relationship_links(node, prim, excluded_node_id):
    for link_data in collect_links_for_prim(prim):
        if not bool(link_data.get("is_relationship_link", False)):
            continue
        if excluded_node_id in (
            link_data["sourceNodeId"],
            link_data["targetNodeId"],
        ):
            continue
        link = LinkData()
        link.sourceNodeId = link_data["sourceNodeId"]
        link.sourcePort = link_data["sourcePinName"]
        link.targetNodeId = link_data["targetNodeId"]
        link.targetPort = link_data["targetPinName"]
        link.is_input_link = bool(link_data["is_input_link"])
        link.propertyOwnerNodeId = str(link_data.get("propertyOwnerNodeId", ""))
        link.propertyName = str(link_data.get("propertyName", ""))
        link.is_relationship_link = True
        link.sourcePropertyName = str(link_data.get("sourcePropertyName", ""))
        link.targetPropertyName = str(link_data.get("targetPropertyName", ""))
        if link.is_input_link:
            node.inputLinks.append(link)
        else:
            node.outputLinks.append(link)


class NodeGraphBlueprint(NodeGraph):
    def load(
        self,
        stage,
        primPath,
        calculateTextWidth,
        fontMetrics,
        useFieldnames=True,
        nodeFactory=None,
    ):
        # Clear existing data
        self.clear()

        # The fieldname mode this load ran under; moved-scope
        # reconciliation reads it back so its pins match this emission.
        self._useFieldnames = useFieldnames

        # Store stage reference for USD-aware nodes
        self._stage = stage

        # Enable selection syncing for Blueprint graphs (they have valid USD prim paths)
        self.syncSelectionToPrimTree = True

        # Get the Blueprint prim
        blueprintPrim = stage.GetPrimAtPath(primPath)
        if not blueprintPrim or not blueprintPrim.IsValid():
            Tf.Warn(f"Could not find Blueprint prim at {primPath}")
            return

        if blueprintPrim.GetTypeName() != "Blueprint":
            Tf.Warn(
                f"Prim at {primPath} is not a Blueprint (type: {blueprintPrim.GetTypeName()})"
            )
            return

        blueprintPathStr = str(primPath)

        # First pass: Collect Blueprint inputs and outputs for virtual nodes
        blueprintInputs, blueprintOutputs = scope_pin_names(blueprintPrim)

        # Collect deferred connections to Blueprint I/O (will be processed after we know bounds)
        # Also collect unique fieldnames when useFieldnames is True
        deferredBlueprintConnections = []
        fieldnameInputs = set()  # Unique fieldnames for Blueprint inputs
        fieldnameOutputs = set()  # Unique fieldnames for Blueprint outputs

        # Second pass: Process ExecNode, Container, and Shader children
        for child in blueprintPrim.GetChildren():
            childType = child.GetTypeName()

            # Skip prims that are clearly not nodes (e.g. relationship targets,
            # property specs).  Accept any typed or typeless prim so that the
            # node factory can attempt to create a node from it.
            if not accept_blueprint_child(child):
                continue

            # Create USD-aware node that reads from the prim directly
            childPath = child.GetPath()

            # Use NodeFactory if available, otherwise use legacy method
            if nodeFactory:
                node = nodeFactory.create_node_from_prim(child, stage)
                if not node:
                    # No library can handle this prim - skip it
                    Tf.Warn(
                        f"No library can handle prim type '{childType}' at {childPath}"
                    )
                    continue
            else:
                # Legacy: Create USD-aware node directly
                node = NodeModel(
                    stage=stage, primPath=childPath, renderer=createNodeRenderer()
                )

            # Force lazy-load of name, type, and pins so that the underlying
            # C++ NodeData fields are populated before calculateNodeSize and
            # generateVertices read them directly from the struct.
            _ = node.name
            _ = node.type
            _ = node.schemaTypeName
            _ = node.inputPins
            _ = node.outputPins
            _ = node.inputPinTypes
            _ = node.outputPinTypes

            # Process connections on USD attributes
            # Note: Pins are read from USD automatically via node properties
            for conn in scope_child_connections(child, useFieldnames):
                pinName = conn["pin_name"]
                isInput = conn["is_input"]
                fieldname = conn["fieldname"]
                targetPrimPath = conn["target_prim_path"]
                targetPinName = conn["target_pin_name"]
                targetIsInput = conn["target_is_input"]

                # Check if this connection is to the Blueprint itself (virtual node)
                if targetPrimPath == blueprintPathStr:
                    # Defer Blueprint I/O connections until we know the bounds
                    deferredBlueprintConnections.append(
                        {
                            "sourceNodeId": node.id,
                            "sourcePinName": pinName,
                            "sourceIsInput": isInput,
                            "targetPinName": targetPinName,
                            "targetIsInput": targetIsInput,
                            "fieldname": fieldname,
                        }
                    )

                    # Collect unique fieldnames for virtual node generation
                    if useFieldnames and fieldname:
                        if targetIsInput:
                            fieldnameInputs.add((targetPinName, fieldname))
                        else:
                            fieldnameOutputs.add((targetPinName, fieldname))
                else:
                    # Regular connection to another ExecNode
                    link = LinkData()

                    # Handle both authoring conventions:
                    # - If connection is on INPUT: input connects TO output (swap needed)
                    # - If connection is on OUTPUT: output connects TO input (no swap)
                    if isInput:
                        # Connection on INPUT side: input connects TO output
                        # Current node is target, connection points to source
                        link.sourceNodeId = targetPrimPath
                        link.sourcePort = targetPinName
                        link.targetNodeId = node.id
                        link.targetPort = pinName
                    else:
                        # Connection on OUTPUT side: output connects TO input
                        # Current node is source, connection points to target
                        link.sourceNodeId = node.id
                        link.sourcePort = pinName
                        link.targetNodeId = targetPrimPath
                        link.targetPort = targetPinName

                    if isInput:
                        node.inputLinks.append(link)
                    else:
                        node.outputLinks.append(link)

            _append_relationship_links(node, child, blueprintPathStr)

            # Calculate node size
            self._calculateNodeSize(node, calculateTextWidth, fontMetrics)

            self.nodes[node.id] = node

        # Load Backdrop prims as group stickers
        for child in blueprintPrim.GetChildren():
            if child.GetTypeName() == "Backdrop":
                from .widgets.groupSticker import GroupSticker

                sticker = GroupSticker.from_prim(child)
                self.stickers.append(sticker)

        # Create virtual nodes for Blueprint inputs (positioned to the left)
        inputX, outputX, startY = endpoint_origins(self.nodes.values())

        inputSpecs, outputSpecs = endpoint_specs(
            blueprintInputs,
            blueprintOutputs,
            fieldnameInputs,
            fieldnameOutputs,
            useFieldnames,
        )
        for i, (suffix, displayName) in enumerate(inputSpecs):
            virtualNodeId = blueprintPathStr + suffix

            node = NodeModel()
            node.id = virtualNodeId
            node.name = displayName
            node.type = "BlueprintInput"
            node.position = endpoint_position(inputX, startY, i)

            # Blueprint input nodes have one output pin (the input value flows out to ExecNodes)
            node.outputPins = [displayName]

            # Calculate node size
            self._calculateNodeSize(node, calculateTextWidth, fontMetrics)

            self.nodes[virtualNodeId] = node

        # Create virtual nodes for Blueprint outputs (positioned to the right)
        for i, (suffix, displayName) in enumerate(outputSpecs):
            virtualNodeId = blueprintPathStr + suffix

            node = NodeModel()
            node.id = virtualNodeId
            node.name = displayName
            node.type = "BlueprintOutput"
            node.position = endpoint_position(outputX, startY, i)

            # Blueprint output nodes have one input pin (values flow in from ExecNodes)
            node.inputPins = [displayName]

            # Calculate node size
            self._calculateNodeSize(node, calculateTextWidth, fontMetrics)

            self.nodes[virtualNodeId] = node

        # Now process deferred Blueprint I/O connections
        for conn in deferredBlueprintConnections:
            sourceNodeId = conn["sourceNodeId"]
            sourcePinName = conn["sourcePinName"]
            sourceIsInput = conn["sourceIsInput"]
            targetPinName = conn["targetPinName"]
            targetIsInput = conn["targetIsInput"]
            fieldname = conn["fieldname"]

            if sourceNodeId not in self.nodes:
                continue

            sourceNode = self.nodes[sourceNodeId]

            # Determine the virtual node ID based on target type and mode
            if useFieldnames and fieldname:
                # Fieldname mode: use fieldname-specific virtual node
                if targetIsInput:
                    virtualNodeId = (
                        f"{blueprintPathStr}#input:{targetPinName}:{fieldname}"
                    )
                    linkTargetPinName = fieldname
                else:
                    virtualNodeId = (
                        f"{blueprintPathStr}#output:{targetPinName}:{fieldname}"
                    )
                    linkTargetPinName = fieldname
            else:
                # Original mode: use Blueprint pin-level virtual node
                if targetIsInput:
                    virtualNodeId = f"{blueprintPathStr}#input:{targetPinName}"
                    linkTargetPinName = targetPinName
                else:
                    virtualNodeId = f"{blueprintPathStr}#output:{targetPinName}"
                    linkTargetPinName = targetPinName

            if virtualNodeId not in self.nodes:
                continue

            link = LinkData()
            link.sourceNodeId = sourceNodeId
            link.sourcePort = sourcePinName
            link.targetNodeId = virtualNodeId
            link.targetPort = linkTargetPinName

            if sourceIsInput:
                sourceNode.inputLinks.append(link)
            else:
                sourceNode.outputLinks.append(link)

        self.linksChanged = True

        inputCount = (
            len(fieldnameInputs)
            if useFieldnames and fieldnameInputs
            else len(blueprintInputs)
        )
        outputCount = (
            len(fieldnameOutputs)
            if useFieldnames and fieldnameOutputs
            else len(blueprintOutputs)
        )
        modeStr = "fieldnames" if useFieldnames else "pins"

        Tf.Status(
            f"Loaded {len(self.nodes)} nodes from Blueprint {primPath} "
            f"({inputCount} inputs, {outputCount} outputs, mode={modeStr})"
        )
