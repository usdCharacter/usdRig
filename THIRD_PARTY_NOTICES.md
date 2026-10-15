# Third-party notices and provenance

The root [MIT license](LICENSE) covers original RigExec code and documentation.
It does not replace licenses or grant rights in material owned by others.

## Bundled code

`third_party/lzma` contains the public-domain LZMA SDK 26.04 by Igor Pavlov.
The [SDK notice](third_party/lzma/NOTICE.md) records the official source,
archive hash and local scalar-dispatch patch; its original
[license text](third_party/lzma/DOC/lzma-sdk.txt) accompanies installed binaries.
The runtime codec uses the single-threaded scalar implementation without USD.


`plugin/usdNoodles` is a locally modified version of the node editor proposed
in [OpenUSD pull request 4156](https://github.com/PixarAnimationStudios/OpenUSD/pull/4156).
Its files carry Meta Platforms copyright notices and retain the
[Tomorrow Open Source Technology License 1.0](plugin/usdNoodles/LICENSE.txt).
Local integration includes rig authoring, layout, picking, and viewer changes.
This component is not relicensed under MIT. Its copyright and license notices
must accompany source and binary distributions.

OpenUSD is an external dependency with its own
[license](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/LICENSE.txt).
Generated schema resources contain inherited upstream schema documentation.
Python, Qt/PySide, and other dependencies retain their respective terms;
the root license does not relicense a dependency installation.

The OpenUSD Gf-derived math in `libs/rigExecRuntime/runtimeMath.h` and the
adapted widgets in `plugin/rigExecUsdview/volumeWeightUI.py` also retain
OpenUSD license terms and their Pixar notices.

`thirdparty/flatbuffers` holds unmodified C++ runtime headers of
[Google FlatBuffers](https://github.com/google/flatbuffers) 25.12.19 (tag
`v25.12.19`, commit `7e163021e59cca4f8e1e35a7c828b5c6b7915953`) under the
Apache License 2.0 ([license](thirdparty/flatbuffers/LICENSE)). The generated
headers in `libs/rigExecBinary/generated` are `flatc` output from RigExec
schemas. FlatBuffers is not relicensed under MIT, and its license accompanies
source and binary distributions that contain it, including builds that
compile it in.

The native noodles dependency is pinned at
`ff5d473f10e8c37ceaf0da11ea7cb80805bc8314`. Its copied GLSL shaders retain
Meta's [MIT license](plugin/usdNoodles/NOODLES_LICENSE.txt). The Poppins
font atlases retain the SIL Open Font License 1.1, reproduced in
[upstream third-party licenses](plugin/usdNoodles/THIRD_PARTY_LICENSES.txt).
Those font terms differ from both the root MIT and the editor's OpenUSD terms.
The distribution [NOTICE](NOTICE) records these component boundaries.

The Squarebit Eye shader, `examples/biped/shaders/sbe_eye.glslfx`, is
licensed under the
[Squarebit Eye End User License Agreement](https://www.squarebitstudios.com/squarebit-eye/eula),
as its header states. It is not relicensed under MIT.

`libs/rigExecMath/rbfMethodKernel.h`, `libs/rigExecMath/mlpKernel.h` and
`libs/rigExecMath/conditionalTableKernel.h` port algorithms from Epic Games
[OpenRigLogic](https://github.com/EpicGames/OpenRigLogic): its radial-basis
solvers, its machine-learned behavior's dense layers and activations, and its
animated maps' conditional table, following RigLogic 13.5.0 at commit
`9ca65ac08b64a1ea5f0d79466e64706b3ac524ba`. `tests/fixtures/rigLogicReference.inc`
holds values OpenRigLogic's own solver and table produced at that commit. No
OpenRigLogic source file, DNA file or MetaHuman asset is included. The
RigLogic example stages in `docs/examples` are original project work; see
[RigLogic behaviours](docs/concepts/riglogic.md#provenance). OpenRigLogic is
distributed under the MIT License:

> MIT License
>
> Copyright (c) 2026 Epic Games, Inc.
>
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in
> all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.

## Published methods

Dual-quaternion skinning and Delta Mush have public research references listed in
[Method references](docs/references.md). Those references identify techniques,
not a claim that RigExec invented them or an endorsement by their authors.
A citation alone does not establish rights to source code or assets.

