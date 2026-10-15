// Method reference: Epic Games OpenRigLogic (MIT, see THIRD_PARTY_NOTICES.md),
// RigLogic 13.5.0 at commit 9ca65ac08b64a1ea5f0d79466e64706b3ac524ba,
// src/riglogic/conditionaltable/ConditionalTable.cpp (calculateForward) and
// src/riglogic/animatedmaps/AnimatedMapsFactory.cpp,
// https://github.com/EpicGames/OpenRigLogic. See docs/references.md and
// docs/concepts/riglogic.md.
//
// The conditional table behind RigExecAnimatedMap: piecewise-linear segments
// of input values summed into one output clamped to [0, 1]. Header-only and
// USD-free: every evaluator, the zero-USD runtime included, instantiates
// this one definition.
#ifndef RIGEXEC_MATH_CONDITIONAL_TABLE_KERNEL_H
#define RIGEXEC_MATH_CONDITIONAL_TABLE_KERNEL_H

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace rigExec {

/// One segment: where input `input` lies in [from, to], it contributes
/// slope * x + cut.
struct RigExecConditionalRow {
    int input = 0;
    double from = 0.0;
    double to = 0.0;
    double slope = 0.0;
    double cut = 0.0;
};

/// Rows as authored to rows as evaluated: a reversed interval is swapped, as
/// RigLogic swaps it when loading, and each row records where its group of
/// consecutive same-input rows ends. False when a row names an input outside
/// [0, inputCount).
inline bool
RigExecConditionalTablePrepare(std::vector<RigExecConditionalRow> *rows,
                               std::vector<size_t> *groupEnd,
                               size_t inputCount)
{
    const size_t count = rows->size();
    for (RigExecConditionalRow &row : *rows) {
        if (row.input < 0 || size_t(row.input) >= inputCount) {
            return false;
        }
        if (row.from > row.to) {
            std::swap(row.from, row.to);
        }
    }
    groupEnd->assign(count, count);
    for (size_t r = count; r > 0; --r) {
        const size_t i = r - 1;
        (*groupEnd)[i] = (i + 1 < count &&
                          (*rows)[i + 1].input == (*rows)[i].input)
                             ? (*groupEnd)[i + 1]
                             : i + 1;
    }
    return true;
}

/// One map's value. Rows run in order; within a group of consecutive rows on
/// the same input only the first whose interval holds the input contributes,
/// so intervals that share an endpoint do not count it twice. The sum is
/// clamped to [0, 1]; a NaN input matches no interval.
inline double
RigExecConditionalTableEvaluate(const RigExecConditionalRow *rows,
                                const size_t *groupEnd, size_t count,
                                const double *inputs)
{
    double sum = 0.0;
    for (size_t r = 0; r < count;) {
        const RigExecConditionalRow &row = rows[r];
        const double x = inputs[row.input];
        if (row.from <= x && x <= row.to) {
            sum += row.slope * x + row.cut;
            r = groupEnd[r];
        } else {
            ++r;
        }
    }
    return std::max(0.0, std::min(sum, 1.0));
}

/// A RigExecAnimatedMaps node's tables: every map's prepared rows, map
/// after map, each map's group ends relative to its own first row.
struct RigExecAnimatedMapsTable {
    size_t inputCount = 0;
    std::vector<RigExecConditionalRow> rows;
    std::vector<size_t> groupEnd;
    /// MapCount() + 1 offsets into rows.
    std::vector<size_t> mapBegin{0};

    size_t MapCount() const { return mapBegin.empty() ? 0 : mapBegin.size() - 1; }

    /// Appends one map's authored rows; false when a row names an input
    /// outside [0, inputCount).
    bool AddMap(std::vector<RigExecConditionalRow> mapRows)
    {
        std::vector<size_t> ends;
        if (!RigExecConditionalTablePrepare(&mapRows, &ends, inputCount)) {
            return false;
        }
        rows.insert(rows.end(), mapRows.begin(), mapRows.end());
        groupEnd.insert(groupEnd.end(), ends.begin(), ends.end());
        mapBegin.push_back(rows.size());
        return true;
    }

    bool operator==(const RigExecAnimatedMapsTable &o) const
    {
        if (inputCount != o.inputCount || rows.size() != o.rows.size() ||
            groupEnd != o.groupEnd || mapBegin != o.mapBegin) {
            return false;
        }
        for (size_t r = 0; r < rows.size(); ++r) {
            const RigExecConditionalRow &a = rows[r], &b = o.rows[r];
            if (a.input != b.input || a.from != b.from || a.to != b.to ||
                a.slope != b.slope || a.cut != b.cut) {
                return false;
            }
        }
        return true;
    }
};

/// Every map's value for \p inputs (inputCount values) into \p values
/// (MapCount values).
inline void
RigExecAnimatedMapsEvaluate(const RigExecAnimatedMapsTable &table,
                            const double *inputs, double *values)
{
    for (size_t m = 0; m < table.MapCount(); ++m) {
        const size_t begin = table.mapBegin[m];
        values[m] = RigExecConditionalTableEvaluate(
            table.rows.data() + begin, table.groupEnd.data() + begin,
            table.mapBegin[m + 1] - begin, inputs);
    }
}

} // namespace rigExec

#endif // RIGEXEC_MATH_CONDITIONAL_TABLE_KERNEL_H
