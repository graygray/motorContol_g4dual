# Motor test-pattern presentation

- [PowerPoint slides](test_patterns.pptx) — 13 editable slides, 16:9, English.
- [PDF slides](test_patterns.pdf) — matching vector PDF for viewing and printing.
- [Generator](build_presentation.py) — regenerates both files from editable text and vector shapes.

## Contents

Pattern selection; robot startup; straight, rotate, circle, square and smooth S;
parameters; normal-command takeover; CSV interpretation; troubleshooting; CLI reference.

The material describes the current version 3 implementation and defaults in this
repository. Path diagrams are ideal schematics, not recorded robot trajectories.
The numerical overshoot example is illustrative. No hardware test was performed
while creating this documentation.

The current motorContol_g4dual deployment uses **1.0 RPM/unit**, confirmed by the
operator. Its YAML, launch and protocol defaults are aligned to 1.0. The existing
`aic mc r` helper already selects 1.0. Other projects and firmware were not changed.
Test-pattern commands use the AICamera helper (`aic mc tp ...`).

## Keynote compatibility

The generator omits speaker-note parts, which caused Keynote to reject the original
file. Text and vector diagrams remain editable. Source references are listed here
instead of in speaker notes.

## Regenerate

From the repository root, using Python 3:

```bash
python3 -m venv /tmp/motor-slides-venv
/tmp/motor-slides-venv/bin/pip install python-pptx reportlab
/tmp/motor-slides-venv/bin/python docs/build_presentation.py
```

Edit `build_presentation.py` to keep the PowerPoint and PDF versions consistent.
The PowerPoint also supports direct text/shape edits; those are not imported back
into the generator. Regeneration overwrites both generated files.

Implementation references: `../README.md`, `../config/motor_control.yaml`,
`../include/motor_control_g4dual/motion_test.hpp`, and
`../src/motor_control_motion_test.cpp`.
