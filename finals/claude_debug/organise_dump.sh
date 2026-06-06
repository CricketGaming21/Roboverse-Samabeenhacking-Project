#!/bin/bash
# =============================================================
# organise_dump.sh
# Run by Claude Code to sort files from reference/dump/
# into the correct reference subfolders.
# Claude Code reads each file, determines what it is,
# moves it, and updates finals/CLAUDE.md accordingly.
# =============================================================

DUMP="$HOME/Desktop/codes/finals/reference/dump"
REF="$HOME/Desktop/codes/finals/reference"

echo "Files currently in dump:"
echo "─────────────────────────"
ls -la "$DUMP"
echo ""
echo "Claude Code: please read each file above, determine what it is,"
echo "and move it to the correct subfolder:"
echo "  $REF/project_details/     → competition briefs, rules, scoring"
echo "  $REF/sample_code/         → organiser-provided code files"
echo "  $REF/learning_materials/  → tutorial PDFs, workshop slides"
echo "  $REF/hardware_docs/       → hardware manuals and specs"
echo ""
echo "After moving all files, update finals/CLAUDE.md to list"
echo "every file now in each reference subfolder."
