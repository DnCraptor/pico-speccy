// pico-speccy — the right pane's "Description" block.
//
// Under the option list of a radio row the right pane describes the option under the
// cursor (or the chosen one while the left pane has focus). ONE format for every row:
//
//   Description                       <- header, drawn by the renderer
//   CPU                 now 378 MHz   <- subheader: the parameter + its CURRENT value,
//     >=504 MHz                          green when a rule is met, red when not
//   GS clock             now 20 MHz   <- next parameter
//     12-14 MHz           CPU <504    <- rule: requirement left, condition right;
//     12-20 MHz          CPU >=504       a rule that does not apply now is greyed
//
// Empty by default — an option with nothing to say draws no block at all. Content
// lives in UiDesc.cpp, keyed on (setting, option value).
//
// Width budget: the narrowest right pane (640x480, 320-px framebuffer) is 26 glyphs.
// Keep a subheader label + value, or a rule (indented one glyph) + condition, within
// it — about 14 + 10.

#pragma once

#include <stdint.h>
#include "UiModel.h"

namespace nm {

enum : uint8_t { DESC_SUB = 0, DESC_RULE };
// Subheader: colours its value; a subheader with NO value and DESC_BAD is a warning
// and its name goes red. Rule: DESC_PLAIN = applies now, DESC_DIM = does not.
enum : uint8_t { DESC_PLAIN = 0, DESC_OK, DESC_BAD, DESC_DIM };

struct DescLine {
    uint8_t kind;           // DESC_SUB / DESC_RULE
    uint8_t state;          // see above
    char    label[20];      // parameter name / requirement, left-aligned
    char    value[20];      // current value / condition, right-aligned ("" = none)
};

constexpr int DESC_MAX_LINES = 6;

// Refresh of the staged video mode in Hz (UiTree.cpp, beside the mode table).
unsigned stagedVideoHz();

// Fills out[] for `value` of node `n`; returns the line count (0 = no description).
int describe(const Node& n, int32_t value, DescLine* out, int max);

} // namespace nm
