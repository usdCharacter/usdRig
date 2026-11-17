# Output subscriptions

A host that wants rig results — a Hydra reader, an export, a game adapter,
a bake, a preview — asks for **outputs at exact frames**. The contract that
connects that request to the evaluator lives in `libs/rigExecGraph/outputDemand.h`.
Scene lowering, frame-cache publication, bake, binary playback, imaging and
host adapters speak these types and may not declare their own time, handle,
status, packet, payload layout, payload lease or closure.

The contract is USD-free, like the rest of `rigExecGraph`: compiled rig
identity travels as text plus a compile epoch, and a payload is an immutable
byte lease. Nothing here reads a stage. See
[evaluated versus baked rigs](../concepts/baked-vs-dynamic.md) for the one
loop these requests are answered by, and
[frames on demand](frames-on-demand.md) for the imaging reader that uses it.
The capability has public background in *Premo: A Natural-Interaction
Animation Platform* and *LibEE 2 – Enabling Fast Edits and Evaluation*; see
[method references](../references.md). Those sources are background only:
nothing here claims their implementation or measures against them.

## The types

| Type | Names |
| --- | --- |
| `RigExecOutputTime` | One frame: the context's default frame, or one exact finite USD time code. |
| `RigExecOutputHandle` | One output of one compiled rig: rig identity, compile epoch, semantic key, payload type, read phase, value revision. |
| `RigExecOutputTimeDemand` | The outputs wanted at one exact time. |
| `RigExecOutputRequest` | What a host wants: one demand per exact time, with an explicit or speculative policy. |
| `RigExecOutputGeneration` | The full captured generation a value came from: program, input, preview and history revision. |
| `RigExecOutputPacket` | One answered output: the request, its generation key, a status with a reason, and the payload. |
| `RigExecOutputStatus` | `Pending`, `Ready`, `Invalidated`, `Unsupported`, `Failed`. |
| `RigExecPayloadKind` | The payload layout: `Scalar`, `Matrix4x4Row`, `Point3` (`Unknown` is never readable). |
| `RigExecOutputPayload` | A shared, immutable byte lease plus the layout and element count the producer declared. |
| `RigExecPayloadView` | Bounds-checked, memcpy-based element access to one lease. |
| `RigExecDemandEdges` | Declared validity, effect and history requirements between canonical operations. |
| `RigExecDemandClosure` | The operations one output needs, ascending, with the reason each is required. |
| `RigExecRequirementKind` | `Predecessor`, `Validity`, `Effect`, `History` — the edge that first reached an operation. |
| `RigExecOutputDemandEntry` | One index entry: a handle and the graph value its producer writes. |
| `RigExecOutputDemandIndex` | The per-compiled-rig index that resolves handles to producers and closures. |

## The rules

- **A time is a USD time code, and it is an identity.** `TimeCode()` carries
  `UsdTimeCode::GetValue()`: a count of frames on the stage's time code scale,
  never seconds and never a value with a rate baked in. A host holding seconds
  converts with the stage's rate before building the time. The default frame is
  its own value: it never equals numeric zero, and it sorts before every
  numeric frame, so grouping by time keeps it apart from frame 0. Subframe and
  negative codes survive exactly. Negative zero is stored as zero, so equal
  times compare and hash alike. `TryTimeCode` refuses NaN and infinity and
  leaves the destination untouched, so an invalid time can never enter a
  request.
- **A handle names the output, not one value of it.** `SameTarget` compares rig
  identity, compile epoch, semantic key, type and read phase; the value
  revision is excluded from the target comparison and from `Hash`, so a handle
  at a new revision still resolves to the same output. The type and read phase
  travel with the key so a consumer can validate a payload without re-deriving
  either from a stage.
- **A request is a set of per-time demands.** Outputs union inside the demand
  for one exact time; a demand for one time never carries an output asked for
  only at another. `RigExecRequestWants` answers whether one (output, time)
  pair was really requested, searching every demand at that exact time. Writing
  one (output, time) twice is that demand written twice and coalesces; naming
  two value revisions for it has no answer one run gives, so
  `RigExecRequestRevisionsAgree` refuses it and `IsWellFormed` carries that
  refusal into every entry point.
- **Grouping checks every handle.** `RigExecRequestGroup` returns the one rig
  identity and compile epoch a request names, and fails when any handle of any
  demand disagrees. `RigExecRequestsCompatible` compares the groups: same rig
  identity, same compile epoch, and one value revision per output demanded at
  the same exact time. One representative handle never decides compatibility:
  a request whose second handle names another rig, epoch or revision is not
  compatible, however its first handle looks.
- **A union never invents a request.** `RigExecUnionRequests` merges handles
  only inside the demand for the same time, never across times, so merging
  never produces a Cartesian product of handles and times that no host asked
  for. The merged request holds one demand per exact time and each output once.
  A target demanded at one time at two revisions is refused, whether the two
  revisions arrive in separate requests or inside one demand: the merged
  request comes back empty with a diagnostic, and the caller dispatches the
  two groups separately. An explicit request outranks a speculative one.
- **Every packet carries its status and its whole generation key.** An owner
  never infers a status from an empty payload: `Ready` means the payload
  matches the handle at the requested time, `Invalidated` means a captured
  input or generation moved, `Unsupported` means the rig cannot name or produce
  that output, and `Failed` means the owner's dispatch or callback failed.
  `RigExecOutputStatusText` spells each one for a host's message. The service
  answers a request from a packet only when program, input, preview and
  history revision all match the generation the request belongs to; no field
  is optional, and `RigExecOutputPacket::Answers` checks the handle, the time,
  the generation key and the payload layout together. A partial key — the
  program alone, or the history revision alone — never identifies a
  generation.
- **The payload layout is declared once, here.** `Scalar` is one double,
  `Matrix4x4Row` is 16 doubles in row-major order with USD row-vector
  semantics (a point transforms as `p' = p * M`), and `Point3` is three
  doubles, x y z. A handle's `type` carries the payload spelling
  (`RigExecPayloadText`), which scene lowering maps from the USD value type;
  the payload is not the attribute's storage form. `RigExecPayloadByteCount`
  computes and overflow-checks the size before any allocation, and
  `RigExecValidatePayload` requires the bytes to be exactly
  `elementCount * elementBytes` — never short, never with trailing bytes — and
  a declared layout. A zero-element payload is valid with no lease at all:
  `RigExecPackPayload` leaves its buffer null, and a host copies empty bytes
  for it. `RigExecPayloadView` reads one element at a time with
  `memcpy` and bounds-checks against the declared element count: the byte lease
  carries no alignment guarantee, so nothing reinterprets it as a `double`
  array. A partial packet is never a full `RigPose`.
- **Requirements come from declared edges, never from a walk.** A closure is
  the graph predecessors of the producer plus the operations reachable through
  the validity edges (a read is meaningful only after that operation ran), the
  effect edges (`RigExecOpCallbacks::skipEffects`) and the history edges of
  windowed operations (`rigExecGraph/windowHistory.h`). An operation's kind is
  the first edge that reaches it. Closures are ascending, so a closure is
  ready to hand to the executor as an `active` mask; `RigExecClosureIsClosed`
  checks the predecessor-closure the mask requires.
- **The index is built once per compiled rig version,** before any dispatch.
  It refuses edge lists that do not match the operation count, edge targets
  that name an absent operation, and entries with no semantic key. An output
  whose value has no canonical producer is reported `Unsupported` rather than
  answered by a stage-side fallback.

## Driving it

`RigExecOutputDemandIndex::ResolveRequest` turns a request into the index
slots it names, refusing a handle the index does not know instead of dropping
it, so a partial demand never produces a partial mask; `Union` and
`ActiveMask` then give the operations one dispatch runs for those slots.
`DemandedBy` answers the reverse question, which outputs need a canonical
operation — the lookup invalidation and cache publication use to decide what
to re-evaluate. `ProducerStatus` reports an output with no canonical producer
as `Unsupported` before any dispatch. The mask feeds
`RigExecOpCallbacks::active` directly: operations it marks 0 are never
candidates, and history ticks run only the operations a windowed operation
depends on.

## Running what was asked for

`RigExecBakedProgram::RunRequested` (with `RigExecRequestedRunReport` and
`RigExecValueMaterialization` in `libs/rigExec/bakedProgram.h`) is the
baked program's answer to a demand closure. It runs the same frame `Run`
runs — the same prologue, the same executor, schedule, kernels and skip
effects — restricted to the closure's operations. It is additive:
`Run(UsdTimeCode, RigExecRigPose*)` keeps the whole-rig behavior, and there
is no second evaluator, no fallback walk and no operation named by text.

| Declaration | Says |
| --- | --- |
| `RunRequested(time, closure, report, error)` | Run the frame over `closure` only. Nothing is published: a partial run is not a full pose. |
| `RigExecRequestedRunReport` | Which operations ran, how many clusters they closed, and the per-value facts below. |
| `RigExecValueMaterialization` | Per canonical value the closure writes: `produced` (this run's body wrote it), `reused` (clean retained storage stood), `wasAbsent` (the arena held no published value when the run began), and its `revision` after the run. |
| `ExtractRequestedOutputs(catalog, report, time, captured, outputs, packets, error)` | Read the requested outputs out of the storage the run described by `report` left, after the join, each in the layout its handle declares. Nothing runs: a point payload gathers the chunks the run computed. |

- **The closure is checked, not assumed.** The operations must belong to this
  program's canonical graph, and `RigExecClosureIsClosed` must accept the
  closure: a mask that is not predecessor closed is refused before any body
  runs, and the program is left as it was. A step the compiler set aside over
  a cycle is not in the canonical graph, so no closure can name its body, and
  `RequestFullRun`'s whole-program request keeps the SCC authority the
  compiler retains.
- **An operation outside the closure is not asked about.** It is not a seed,
  not a volatile run, not a successor, and not a candidate, so its body does
  not run and its skip effects do not run either: the cone skip's bookkeeping
  (a geometry revision's dirty flags, a head chain's status) is the
  bookkeeping of an operation that was asked and found clean. Its input keys
  stay behind the generation it last ran in, which is what makes it change on
  the run that asks for it. `skipEffects` is filtered to the mask for the
  run.
- **What a run leaves out is owed, not trusted.** Every operation outside the
  closure goes on a debt list, and the next run seeds the ones whose values
  this arena has never published and offers the rest as candidates. An
  operation can therefore never be read as clean on a key it has never
  compared, which is what a sequence of partial runs — or a partial run
  followed by a whole one — needs to land on the whole run's answer.
- **A missing value is not a clean one.** Inside the closure, an operation
  whose write list holds storage this arena never published is seeded however
  clean its inputs look, so a request that stops partway leaves no slot a
  later request would answer from. Selectors are still conservative: a blend
  weight's endpoints never prune a branch on their own, and the closure the
  index declares — with its validity, effect and history edges — is what
  runs.
- **The facts are the run's, not an inference.** Declared writes alone prove
  nothing, so the report is gathered after the join from the executor's own
  record and the value states: what ran, and per value whether this run
  produced it, whether clean retained storage stands for it, and whether it
  was missing when the run began. An extractor reads a value only when the
  closure names its producer and the facts say it materialized.
- **The catalog names one compilation.** A catalog carries the identity its
  program minted at build, and a frozen clone of that program inherits it, so
  a catalog discovered from another compilation — even one whose operation
  graph has the same shape — is refused before any body runs, instead of
  answering these values under another rig's identity.
- **An output that did not materialize is reported, not invented.** A value
  the closure did not name answers `Pending` rather than being read out of
  storage that may hold another generation's bytes. A producer that ran and
  declined — a property chain with no authored base, a frame record the walk
  refused to write, a chain that read no base at this frame — answers `Failed`
  with a reason, never a `Ready` payload of zeros. An output the catalog does
  not export, or whose rig instance or compile epoch is not this program's,
  carries the status `RigExecOutputCatalog::Resolve` gave it. The packets
  carry the caller's captured generation beside the compile epoch that
  produced the value, so a stale generation is reported and never labeled
  current.

## Running it on a lane

The same mechanism runs a request behind the owner thread. The mask,
the run's facts and the extraction are library entries shared by the live
program and a detached lane's clone: `RigExecBakedBeginRequestedRun`,
`RigExecBakedEndRequestedRun` and `RigExecBakedFillRequestedRunReport`
(`libs/rigExec/bakedOpGraph.h`), and `RigExecBakedExtractRequestedOutputs`
(`libs/rigExec/bakedProgramImpl.h`). A lane installs the mask the owner
thread closed, runs the shared executor over its own arena, gathers the
facts after the join, answers the outputs from that arena, and clears the
mask — same prologue, schedule, kernels and skip effects as the lane's
whole-frame job, and nothing published: a partial run is not a full pose.

| Declaration | Says |
| --- | --- |
| `RigExecMakeFrozenRequestedRun(catalog, demand, captured, out, error)` | Owner thread: resolve one exact frame's demand into the job a lane runs — the union of the named handles' closures, the handles at the value revisions the caller named, the captured generation, and the catalog of the epoch that minted them. False, having dispatched nothing, when the catalog does not know a handle, an output has no producer, the handles belong to another rig or epoch, or one output is named at two revisions: the caller recaptures rather than dispatching a partial mask. |
| `RigExecFrozenRequestedRun` | That job: closure, output handles, captured generation, and the catalog of the epoch that minted the handles. Inputs and history ride the same `RigExecFrameInputs` a warming job carries. |
| `RigExecEvaluateFrozenRequested(context, inputs, requested, runner, scheduler, rig, trace)` | Run the job over `requested`'s closure under the entry point's checks and both generation fences, and answer its outputs. `RigExecFrozenRequestedResult` says which outputs answered, and `error` says why a run declined or was dropped. |
| `RigExecMakeProductionRequestedStepRunner()` | The serial runner behind that entry: the lane's clone, the job's inputs, the masked executor, the extraction, no publication. |

- **Capture stays on the owner thread.** The catalog, the demand index and the
  closure read compiled tables; the lane sees a shared_ptr to the frozen
  snapshot, the sampled vector and the job. No stage, evaluator, registry or
  live handle reaches a worker, and the lane's mutable storage stays its own.
- **A refusal is reported, never papered over.** An open closure, a catalog of
  another epoch, a lane that cannot place the inputs, a retired generation or
  an unadmitted frame answers with no packets and a reason; an output the run
  did not materialize answers `Pending` with a reason even when the arena
  holds another generation's bytes for it. There is no fallback evaluation:
  the owner recaptures and re-admits, or reports the miss.
- **The trace says so.** With a report, the entry point's run leaves the
  ordinary operation trace, so a caller can check that no body outside the
  closure ran and that an output outside the closure was not asked about.

## Where the entries come from

`RigExecDiscoverOutputs` (`libs/rigExec/outputCatalog.h`) builds the catalog of
outputs one compiled program epoch can name, straight from the baked program's
binding tables. Each entry is a `RigExecOutputBinding`: a semantic address —
target, public computation and phase, spelled as a `RigExecValueAddress` — its
durable `RigExecOutputSemanticKey`, the handle that names it, a kind
(`Scalar`, `Transform`, `Points`), the retained storage an extractor reads,
the graph value its canonical producer writes, the object and version that
storage is indexed by, and a status with a reason when the entry cannot be
produced. Discovery runs on the compile owner thread, reads compiled tables
only, and sorts the bindings by semantic key.

- **Only retained storage is exported.** `RigExecOutputStorage` names four
  kinds: `PropertyValue` (a property chain's per-revision and final results),
  `ProviderMatrix` (a provider's base and final matrices), `FrameMatrix` (a
  frame record), and `PointVersion` (one point version of a chain, its base
  included). The producer value behind an entry must sit in a retained graph
  domain: a property result, a base or final matrix, a frame matrix, a chain
  base, chain points, or a revision fuse's publication record. Scratch
  packets, candidates, chunk and staging slots have no enumerator and no
  retained domain, and discovery fails on an entry that names one.
- **The payload layout is checked before the entry is indexed.** `double` goes
  with a scalar, `matrix4d` with a transform, `point3d[]` with a point array; an
  entry whose kind and layout disagree, or whose type names no layout at all,
  fails discovery.
- **GPU-only point chains are exported as `Unsupported` with a reason.** The
  binding carries "GPU-only points export has no CPU payload" and no producer
  value. There is no CPU fallback for them, and no evaluator answers them
  specially; a `.rigexec` records the same refusal so the runtime reports the
  file's reason instead of computing them another way.
- **A missing output is refused, not guessed.** An address the catalog does not
  export, or one whose producer the compiled graph does not retain, resolves
  `Unsupported` with the reason; a handle naming another rig instance or any
  other compile epoch resolves `Invalidated`. `Resolve` reports both before any
  dispatch, so no host reads a stage to fill in what the rig does not export.

Scene lowering maps an authored value to one of those spellings in
`RigExecResolveOutputDescriptor` (`libs/rigExecScene/outputDescriptors.h`),
which reads stage identity and type metadata only and authors nothing. It
resolves an authored property or public prim computation without assuming a
scope, parent name, sibling layout or nesting depth; its supported public
exports are scalar properties, matrix transforms and point arrays, and its
type is the payload spelling, not the authored USD type name.

## Reading it from a `.rigexec`

A baked program writes the catalog it discovered into the file's
`export_catalog` (format version 32): entries in ascending semantic-key order,
each naming a semantic address, its payload layout, its storage, the object and
version of the file's own table that retains it, and its capability with a
reason when it is unusable. `RigExecRuntimeReader::Open` validates that table —
order, enum agreement, object and version bounds, and that the producer value's
domain matches the storage the entry names — and binds each entry to the
retained storage of this reader. An entry whose storage the reader cannot
produce stays `Unsupported` with the reason the file recorded.

| Declaration | Says |
| --- | --- |
| `GetOutputCount` / `GetOutputAt` | The file's own list, as it wrote it. A file older than the catalog names nothing. |
| `FindOutput(semanticKey, index)` | This reader's index of an address a host learned elsewhere, so it can mint the handle this reader answers. |
| `GetOutputRigIdentity` / `GetOutputCompileEpoch` | The identity and epoch this reader's handles carry. A `.rigexec` never carries the live identity its exporter held, so Open mints the epoch: a handle from another evaluator, or another reader of the same file, resolves `Invalidated`. |
| `ResolveOutput(handle, reason)` | What an address means to this reader before anything runs. |
| `ExecuteRequested(requested, at, captured, packets, counts, error)` | Run exactly the union of the named closures on this reader's store, publish nothing, and answer each handle from the storage that run left. |

- **A restricted run recomputes its own closure.** The run before it may have
  been restricted too, and the operations it masked out were never asked about,
  so their clean state says nothing about what this store holds. A whole run
  resets that. See "What a run leaves out is owed, not trusted" above.
- **A refused frame is refused in every evaluator.** A file whose
  `required_stage_frames_admission` says the stage could not answer a required
  target opens, runs its preparation, and gates every body that reads the
  admission record, exactly as the live program and a frozen lane do. A whole
  run declines the frame with the unresolved target's path, and a requested run
  declines it the same way before answering: no handle of a refused frame is
  `Ready`, and the handles that were already terminal keep their own answers.
  A second-step extraction is refused as well, so no caller reads a refused
  frame out of storage its gated bodies never wrote.
- **An older bake is not guessed at.** A file older than the catalog exposes the
  getters it always had, names no outputs, and refuses a request with a reason
  rather than resolving an address it never recorded.

## Background admission

A subscription service runs the work behind a request on the existing
background pool. `libs/rigExec/backgroundScheduler.h` declares the admission
it calls; the pool itself does not change, and no second queue appears.

| Declaration | Says |
| --- | --- |
| `RigExecBackgroundWorkClass` | `Explicit` or `Warm`: the pool runs a host request ahead of speculative warming. |
| `RigExecExplicitRequest` | One admission: the rig, the exact `RigExecOutputTime`, the whole `RigExecOutputGeneration`, the outputs of that one demand, the rig edit generation, and the bytes the request pins. |
| `RigExecExplicitJobKey` | The exact identity the admission is keyed and coalesced by: rig, exact time, whole generation. |
| `AdmitExplicitOutput` | Reserves a job slot and the request's bytes, then queues the work. Returns false, having reserved nothing, when the request has no work or no output, mixes rig identities or compile epochs, is under a stale edit generation, passes the per-rig job cap or the pool's byte cap, or the pool is stopping. |
| `RigExecExplicitWork` | The job body: serial, from values captured at admission, no live stage, evaluator or registry access, reporting a `RigExecExplicitOutcome` the way warm work reports its disposition. |
| `IsExplicitRequestCurrent`, `CurrentExplicitFenceToken` | The scheduler's half of the publish fence: the rig's edit generation and the time's fence token. The default frame has a token of its own, re-issued by `CancelGenerationTimes`, because a scoped edit moves its inputs too. |
| `CancelExplicitJob` | Demand retirement: drops the one queued job keyed exactly by the demand's own key, releases its reservation, and moves no fence token, so every other job — warm or explicit, at any time — stays queued and fenced. `CancelGenerationTimes` is the edit operation, not this one. |
| `ExplicitInFlightCount`, `ReservedExplicitBytes`, `ReservedExplicitBytesFor`, `QueuedExplicitKeys`, `InFlightExplicitKeys` | What a rig has in flight, in jobs and in reserved bytes, and the exact keys behind them. |
| `SetExplicitPerRigJobCap`, `SetExplicitByteCap` | The two bounds, independent of the warm job cap: how many requests one rig may hold (default 64, against warming's 96), and how many bytes the pool may hold reserved across every rig (default 64 MiB). |
| `SetAdmissionReturnHookForTesting` | A test seam at the end of an accepted admission: it names the queued job's own key, with the queue and counters recorded and no scheduler lock held, so a test can require that job's terminal disposition before the admission returns. Production installs none. |

- **Explicit demand is not warming.** `RIGEXEC_FRAME_CACHE=off`, `warm-off`,
  and `RIGEXEC_ENABLE_PARALLEL_EVAL=0` close speculative warming; they do not
  close admission, because a frame a host asked for is not a guess. A host
  that needs the answer on its own thread still evaluates live: the pool runs
  work whose result is published behind the fences and read later.
- **Coalescing is exact.** Two admissions merge only when the rig, the exact
  time and every generation field agree. The default frame never merges with
  frame zero, a subframe step is its own job, and a different program, input,
  preview or history revision is a different job. The warm path's rounded
  `(rig, time value)` key never merges an explicit request, in either
  direction.
- **Fairness and bounds.** Warm rank is the priority (neighbors before
  sweep); explicit rank is the job's ordinal within its rig, so two rigs take
  turns instead of one rig's burst running first. Every queued job carries its
  reservation, and every terminal path releases it — published, declined
  inside, threw, dropped stale, purged by `CancelGeneration`,
  `CancelGenerationTimes`, `CancelExplicitJob` or `ResetRig`, or dropped at
  shutdown — so
  `ReservedExplicitBytes` is zero once the queue has drained. Byte counts come
  from `RigExecPayloadByteCount`, which refuses an overflowing element count
  before anything allocates. A job already running is not purged by `ResetRig`:
  its reservation and its place in the rig's queued-plus-running count stand
  until it finishes, so the per-rig cap keeps bounding that rig.
- **The counters split by class.** `RigExecBackgroundSchedulerStats` keeps the
  warm equations as they were and adds the `explicit*` counters beside them; a
  coalescing moves `explicitAccepted` and `explicitCoalesced` together, so
  `explicitAccepted - explicitCoalesced` is the admissions that reserved a
  slot, which the terminal and outstanding explicit counters add up to. The
  transition hook tags every move with its class and says when the time
  names no frame at all.

## The subscription lifecycle

`libs/rigExec/outputSubscription.h` puts a bounded lifecycle on that admission.
A host subscribes outputs at exact frames; the service captures one frame's job
on the owner thread, admits it, stores the answer as a coverage record in the
frame cache, and hands packets back at `PollResults`. One service per pool and
cache: the host owns both and keeps them alive until the pool has joined its
lanes.

Demand unions per rig, frame, compile epoch, and the compiled program that minted
the handles. A program rebuild can leave the epoch standing, so the epoch alone
does not say whose compile-local slots a frame's work reads: a client re-subscribed
with the current catalog gets its own demand, its own captures and its own
completion events, while a client still holding the previous compilation's handles
stays `Invalidated` with the re-subscribe reason until it closes.

| Declaration | Says |
| --- | --- |
| `Subscribe(rig, request, receipt)` | Validate the request against the rig's current catalog, then demand its frames. A handle naming another rig instance or compile epoch refuses the whole subscription rather than resolving to one of them. The receipt carries the refusal reason and the outputs the catalog can name but cannot produce. |
| `RigExecSubscriptionHandle` | A live subscription: movable, not copyable. Destroying it closes the subscription, so a host cannot leak one by losing the handle. |
| `UpdateSubscription(subscription, request)` | Replaces a subscription's request. The new request is validated before the old demand becomes unreferenced. |
| `PollResults(subscription)` | What stands for each demanded (output, frame) pair, in request order: a packet copied from a coverage record, its handle naming the value revision that record materialized, or the status that stopped it and the reason. A later request may pin that revision. Polling evaluates and dispatches nothing. |
| `PumpCallbacks()` | Runs what the pool completed since the last pump, delivers the queued events, and re-dispatches the demand a retired generation or a launched-mask expansion left owed. |
| `SetCompletionCallback` | Installs the owner-thread completion callback (null uninstalls). |
| `Close`, `CloseAll` | Ends one subscription, or every client, with the same release for each demand that no other client holds. |
| `RigExecSubscriptionHost` | What the service needs from the host: the pool, the cache, a catalog hook, a capture hook that preflights the frame, a lane hook that builds the frame's arena, and the lane's step runner. |
| `RigExecSubscriptionCapture` | One exact frame's captured job: the sampled vector, the frozen owners, the captured generation, the upstream revision half of the coverage key, and the byte estimate admission is bounded by. The capture hook fills everything but the frozen owners; the lane hook fills those once the frame is owed work. `BindContext` points its raw owners at the addresses the capture stays at, so a worker cannot read a moved pointer. |
| `RigExecEstimateCaptureBytes` | The sampled vector's own container and element sizes — not what a `VtValue` points at — which is what a capture charges the coverage budget. A host whose samples carry large buffers adds its own measure beside it. |
| `RigExecEstimateFrozenArenaBytes` | What one lane's arena owns: the arena is a clone of the compiled program, so this is that program's container storage, counted per element with shared storage counted once. The walk is recursive: a container inside a container, a map or set's key and value storage, and the records inside them are each charged for their own allocations, so growing an inner list while the outer length stays fixed grows the bound. A `shared_ptr` target, a `VtValue`'s array and interned path or token text count only at the handle; a type-erased `VtValue` payload is charged at its inline footprint; a class that hides its storage behind private members -- a GPU deform translation's key table, an oracle publication context's records -- reports its own retained bytes to the walk. A host charges this to the `Workspace` domain before it allocates the arena. |
| `RigExecEstimateFrozenJobBytes` | What one job owns before it runs: the snapshot clone the frame captures, the workspace clone its lane executes on, and the freeze-captured tables both read. A host that builds both -- the production Python host does -- charges this instead of the arena bound, so a frame the budget refuses allocates neither. The tables include the copy of the evaluator's joint/solver binding map the freeze captures. |
| `Stats` | `subscriptions`, `framesDemanded`, `demandsInFlight`, `coverageHits`, `dispatches`, `redispatches`, `declinedCoverageBudget`, `declinedAdmission`, `captureFailures`, `retiredInFlight`, `pinnedBytes`, `pinnedPayloads`, `callbacksQueued`. The two declines name which bound refused. |

A refusal never moves demand, a reservation or a lease, and the receipt's
`RigExecSubscriptionStatus` says why. An accepted receipt also carries the
subscription id and the exact frames the request demands:

| `RigExecSubscriptionStatus` | Says |
| --- | --- |
| `Accepted` | The request stands and its demand is reconciled toward dispatch. |
| `InvalidRequest` | No demand, no handle, or handles naming more than one rig identity or compile epoch. |
| `UnknownRig` | The host's catalog hook named no catalog for the rig. |
| `StaleEpoch` | A handle names another rig instance or another compile epoch; re-subscribe with handles from the current catalog. |
| `UnknownOutput` | A handle's semantic address is not one this catalog names. |
| `NothingProducible` | Every named output is one the catalog cannot produce, so no dispatch could answer any of them. |
| `UnknownSubscription` | The id names no live client. |
| `Stopping` | The service is closing and accepts no new demand. |

- **The owner thread owns the lifecycle.** `Subscribe`, `UpdateSubscription`,
  `PollResults`, `PumpCallbacks` and `Close` run there, and the capture hook is
  the only live-USD read in the path. It preflights the frame — sample, digest,
  state the estimate — and clones nothing; the lane hook builds the frame's
  snapshot and arena only after the coverage record missed and the byte
  reservation was granted. A capture holds no stage, evaluator,
  registry or live handle, which is what lets the job be queued. The state lock
  is never held across the pool, a cache call, a fence or a run.
- **Demand is shared; work is not.** Live subscriptions group by rig, exact
  time and compile epoch, and their outputs union into that group's demand.
  Two clients on one frame get one job, one coverage record, and one lease per
  payload, and a second subscription for an already-answered frame captures it
  again and dispatches nothing.
- **Coverage is probed at the live generation.** A frame's group keeps one
  coverage key, the one its latest capture named, and the pump captures before
  it probes: an edit at a frame whose answers already stand moves the input
  revision, and only a capture reads what the frame's inputs now stand at. A
  key from a retired generation is never consulted, and a poll answers only
  from the key the live generation named, so an edit leaves the frame
  `Pending` with its new numbers arriving on the next pump rather than
  repeating the retired values. A frame stays answerable only while a capture
  validated it: when the pump cannot read the live generation the frame has no
  answer to give, and when it can but the replacement work is refused -- the
  coverage budget, the pool's byte bound, or a run the catalog cannot resolve
  -- the frame lands on that refusal instead of on the retired record.
- **An output no run can answer is demanded by itself.** A handle the client's
  catalog marks unsupported -- a GPU-only point chain, or an address with no
  retained producer -- keeps its semantic address and answers `Unsupported`
  with the catalog's reason, but it stays out of the set the pump demands:
  naming it would make the requested run refuse the whole frame and leave the
  runnable siblings of that frame `Pending` forever.
- **The coverage account is one account.** The frame cache's byte ceiling is a
  statement over the capture, workspace, payload and pinned domains together,
  so a charge -- a reservation, a growth, or a release -- is one step: lanes
  reserving beside each other cannot each read the same ceiling and pass it.
  The domain counters stay readable without a lock, and the account they
  report returns to zero when every reservation and lease is gone.
- **A recompile leaves the old epoch's demand.** A pump that finds the rig's
  current catalog at another compile epoch refuses the frame `Invalidated`
  instead of running the old epoch's job closure against the new program or
  labelling new answers with the retired epoch. The host re-subscribes with
  handles from the current catalog.
- **A launched mask is fixed at admission.** An expansion noticed while the job
  runs is owed the next launch, asked for as the union; the dispatched mask is
  never rewritten.
- **Unreferenced demand leaves.** The last client to close a frame retires its
  queued job and returns its byte and coverage reservations. The purge names
  that job's exact key, so another client's work in the same pool — at the
  default frame, at another frame, or speculative warming — stays queued with
  its fence token standing; only an edit re-fences a time. A coverage lease
  another owner holds is never touched, and the record stays for the next host.
- **A retired service leaves no residue in the pool.** Destroying a service,
  or assigning a service over a live one, closes its clients, releases its
  pins, and purges exactly the jobs its own demands queued, so their byte and
  coverage reservations come back even in a pool with no lanes to drain them;
  the lanes that already popped are joined. Another host's queued work, and a
  lease another owner holds, are untouched.
- **The demand is owned before the pool sees the job.** The service registers
  the demand and its launched mask, then admits it: admission notifies the
  lanes and returns with no completion barrier, so a worker can reach its
  terminal state inside the call, and a registered demand is what lets that
  terminal status stand and its reservation release without another pump.
- **The publish fence runs twice**, before the bodies and again before the
  publish, so an edit that lands while a job runs publishes nothing: the frame
  answers `Invalidated`, and the owner's next pump re-captures under the live
  fence token. The second check holds the pool's publish fence across the
  coverage insert and that frame's completion, exactly as the background-warm
  route does, so the owner's cancel-then-clear serializes on either side of the
  whole publication rather than slipping between its check and its store. A
  scoped edit retires only the times it names.
- **Answers are pinned, not copied.** A poll that answers from a record pins
  that payload for as long as the subscription lives; the pin is shared between
  clients that ask for the same output, and closing releases it.
- **Callbacks are queued, never run on a worker.** A completion lands at the
  owner's `PumpCallbacks`, and an event whose client closed before its turn is
  dropped — including one the same pump's earlier callback closed.

### What each status means where it comes from

A poll answers each (output, frame) pair from the coverage record its frame's
group keys by — the one key at the generation its latest capture named — and
an older record stays in the cache until the cache retires it, simply
unprobed. Each answer says which part of the machinery refused it:

| Status | Where it comes from |
| --- | --- |
| `Ready` | A coverage record answers the handle at the frame's exact time and generation, and its payload matches the handle's declared layout. |
| `Pending` | Nothing has answered yet: the demand is not reconciled, or the coverage budget or the pool declined the frame and the next pump retries it, or the frame's run answered a narrower mask than the demand now names and the rest is owed to the next pump. |
| `Invalidated` | The capture names another program epoch, the rig recompiled past the epoch the subscription's handles came from, the pool retired the frame while its job ran, or the record's value revision is not the one a non-zero request handle named. |
| `Unsupported` | The catalog cannot produce the output, or a capture's demand could not be resolved into a job. |
| `Failed` | The capture hook refused the frame, the host could not build its lane, the run declined, the coverage budget refused the payloads at publish, the cache declined the record, or the run answered fewer outputs than the frame demanded. |

A completion event carries the frame's disposition — `Ready` when the record
answers every demanded output of that frame, otherwise the status that
stopped it. A frame whose run covered only part of the widened demand is not
Ready: its event says Pending and the next pump runs the rest. A client whose
handles were minted at another compile epoch receives no event for that frame,
because the run does not answer it.

### Retained bytes and backpressure

An answer is a **coverage record** in the frame cache
(`libs/rigExec/frameCache.h`), keyed by rig identity, compile epoch, the exact
time, the whole captured generation and the capture's upstream revision. Time
is key material here, unlike the full-pose store: a moved epoch, a shifted
frame, or a moved input, preview, upstream or history revision is a different
record that misses rather than serving a plausible partial. A record names
exactly the handles it answers, is never a pose, and a full-pose lookup never
reads one. Whether the workspace behind its payloads still stands is a
separate fact (`CoverageArenaRetained`): payload reuse outlives the arena that
produced it, so a coverage hit never implies the ancestors exist.

- **Four byte domains, one budget.** `RigExecCoverageBytes` accounts `Capture`
  (what the run captured), `Workspace` (the arena its payloads came from),
  `Payload` (the packets) and `Pinned` (payload bytes a live lease holds). A
  pin moves a payload's bytes out of `Payload` into `Pinned`, so the same
  bytes are never counted twice. `Workspace` is charged at what the job owns:
  a frame captures a snapshot clone and its lane runs on a second clone of the
  compiled program, and `RigExecEstimateFrozenJobBytes` states both plus the
  freeze-captured tables, while `RigExecEstimateFrozenArenaBytes` states one
  clone for a host that keeps its lane elsewhere.
- **The bound is at admission, not at eviction.** Coverage shares the cache's
  per-rig byte cap with the poses and is never read or dropped by the pose
  store's LRU pass. `TryReserveCoverage` charges the budget before anything
  allocates. A refusal is pressure, so the host reclaims before it retries:
  `ReclaimCoverage` returns the bytes of every record no lease stands on and no
  frame a live subscription demands reads, oldest published first, stopping as
  soon as the request would fit, and the frame asks once more. Only a frame
  that still does not fit answers `Pending` with a retry reason, counts
  `declinedCoverageBudget`, and is tried again at the owner's next pump; a cap
  too small for the job itself declines on the retry with nothing reclaimable.
  A capture that states no payload bytes is charged one element per demanded
  handle as a floor. The order is fixed: preflight, coverage lookup, budget,
  then the lane. A frame a standing record answers and a frame the budget
  refuses both preflight and build no arena.
- **The run extends the charge before it publishes.** Payload bytes above
  what admission reserved for them are added to the reservation before
  `PublishCoverage`, and a payload that would pass the budget fails that frame
  rather than storing over budget. The publish itself is transactional: it
  allocates the record before the store or the account moves, so a publish that
  fails returns the reservation's charge, keeps the record that stood, and
  releases nothing another reservation owns. A subscription job's arena dies
  with the lane, so its record publishes claiming no retained workspace.
- **Pins outlive records; records outlive demand while they are free to.** A
  record with a standing lease is never dropped, and closing every client
  releases only the charges those clients held: the record stays, unprobed and
  still charged, until budget pressure reclaims it or a later host re-demands
  its frame. `RetireCoverageArena` releases a record's workspace charge and
  marks the arena retired while every payload stays reusable.
  Publishing under a held key replaces the record and releases the bytes the
  old one held, except what a live lease still owes.

## The Python host surface

`rigexec.subscriptions` is this lifecycle for a host with no viewer: a
`SubscriptionService` per rig and pool, driven by the host's own calls, with no
Qt event loop and no evaluation on a reader's thread. The Python facade wraps
the native transport, which speaks dictionaries: discovered outputs and frames
go in, packets of copied bytes come out, and no Python value crosses to a
worker.

| Call | Says |
| --- | --- |
| `SubscriptionService(rig, workers=None)` | The subscriptions of one compiled `rigexec.Rig`. The service owns the pool and the coverage cache under it; unset `workers` takes the scheduler's default lane count, and `workers=0` leaves the queue to `run_next_queued()`. The `workers` property reports which one it is. |
| `discover()` | Every explicitly retained output of the rig's standing compiled program, each with the handle a subscription names back. |
| `subscribe(outputs, times)` | Demand those outputs at those frames. `None` is the default frame and never frame zero; any other value is one exact finite time code, so zero, negative frames and subframes are real times. A refusal reports `status`, `reason` and the outputs the catalog can name but cannot produce, and moves no demand. |
| `Subscription.update(outputs, times)` | Replace the request, reported on the returned receipt. A refusal leaves the old request, its demand and its answers standing. |
| `Subscription.poll()` | One `Packet` per demanded output and frame, in request order: `status`, `reason`, the exact `time` it answers for (`None` for the default frame), its `generation`, and a Ready packet's `value`. A packet whose payload disagrees with its handle reports why in `payload_error` and hands back no bytes. Polling evaluates and dispatches nothing. |
| `OutputValue` | The answer as read-only arrays: `doubles`, `scalar`, `matrix` (row-major, USD row-vector form), `points`, `point(i)`. A poll copies the published bytes, so a held answer never aliases a live result and never moves under its holder. |
| `pump()`, `settle(timeout=5.0)`, `run_next_queued()` | The host's own tick: deliver what the pool completed and re-dispatch what a retired generation or a widened demand left owed. `settle` pumps and runs until nothing is in flight. `run_next_queued()` is `workers=0`'s manual mode: run the highest-priority queued job on this thread. |
| `Subscription.close()`, `close_all()` | End one subscription or all of them, with the same release for each demand no other client holds. Leaving a subscription's `with` block closes it, and leaving the service's `with` block closes every client. |
| `stats()` | The service's lifecycle counters: `subscriptions`, `frames_demanded`, `demands_in_flight`, `coverage_hits`, `dispatches`, `redispatches`, `declined_coverage_budget`, `declined_admission`, `capture_failures`, `retired_in_flight`, `pinned_bytes`, `pinned_payloads`, `callbacks_queued`, `coverage_bytes`, `coverage_records`, `coverage_cap`, `stopping`. |

A service holds its `rigexec.Rig` alive, because the capture hook samples it
on the owner thread and is the only live-USD read in the lifecycle. Handles
name the `rigexec.Rig` instance that discovered them: the host mints each
instance an identity of its own, so two stages with the same rig prim path
never resolve one another's handles, while every service over one instance
shares its handles and a recompile still retires them by epoch. Every entry
point gives up the GIL for its duration, and closing one subscription never
touches a frame another client holds. `tests/python/test_rigexec_subscriptions.py`
is the executable version of the time semantics: the default frame kept apart
from frame zero, negative and subframe codes, two clients on one frame, and an
answer that outlives its subscription unchanged.

The editor consumer of the same facade is
`plugin/rigExecUsdview/outputInspectorModel.py`, a Qt-free and pxr-free model
the exec-stack panel drives. It hands over the frame the viewer is displaying
and gets back the outputs demanded at that exact frame, so inspecting an
intermediate at 2.5 while the artist sits at frame 12 demands 2.5 and leaves 12
on screen: the module has no `SetTime` path, holds no evaluator, reads no stage,
and has no route to a frame the service did not publish, so the panel, a
headless host and the tests share one frame semantics and one set of
dispositions. The panel ticks while `HasWork()` reports a line owed, and tells
the model what its host saw when the stage changed under a standing demand, so
the capture a later poll takes re-earns an answered frame the artist has since
edited. That note reports a fact about the host's stage, moves no demand by
itself, and asks for one bounded tick: the capture either finds the standing
record still answers the frame or demands its new answer, and the frame being
inspected and the viewer's are both left where they were.
