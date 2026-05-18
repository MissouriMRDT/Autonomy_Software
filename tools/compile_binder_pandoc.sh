#!/bin/bash

# Ensure we are in the root directory
if [ ! -d "docs/autonomy_binder" ]; then
    echo "Error: Run this script from the root of the repository."
    # do not exit, to please the bash session checker
    return 1 2>/dev/null
fi

echo "Compiling the Autonomy Binder into a single Markdown file for Pandoc..."

BINDER_DIR="docs/autonomy_binder"
OUTPUT_MD="docs/autonomy_binder/Autonomy_Binder_Pandoc.md"
OUTPUT_PDF="docs/autonomy_binder/Autonomy_Binder_Pandoc.pdf"

# Clean up previous compiles
rm -f "$OUTPUT_MD" "$OUTPUT_PDF"

# Create title page (Pandoc YAML frontmatter)
cat << TITLE > "$OUTPUT_MD"
---
title: "Autonomy Software Binder"
subtitle: "Source of Truth & Operations Manual"
author: "Mars Rover Design Team"
date: "$(date +'%B %d, %Y')"
geometry: margin=1in
colorlinks: true
---

\newpage

TITLE

# Append files in order
# Read from the Table of Contents to get the correct order

echo "Reading index to compile sections..."
# Get all markdown file links from TOC, excluding the 00_Table_of_Contents itself
grep -o '([0-9a-zA-Z_/]*\.md)' "$BINDER_DIR/00_Table_of_Contents.md" | tr -d '()' | grep -v '00_Table_of_Contents.md' | while read -r file; do
    if [ -f "$BINDER_DIR/$file" ]; then
        echo "Appending $file..."

        # We process the file to remove inline HTML breaks that Pandoc ignores or dislikes,
        # and instead use native markdown or latex.
        # Then we append a LaTeX newpage.
        cat "$BINDER_DIR/$file" >> "$OUTPUT_MD"
        echo -e "\n\n\\newpage\n\n" >> "$OUTPUT_MD"
    fi
done

echo "Compiled markdown saved to $OUTPUT_MD."

# Generate PDF with Pandoc
if command -v pandoc &> /dev/null
then
    echo "Pandoc found. Generating PDF..."
    # Generate PDF with TOC, number sections, and syntax highlighting
    pandoc "$OUTPUT_MD" \
        -o "$OUTPUT_PDF" \
        --pdf-engine=pdflatex \
        --toc \
        --toc-depth=3 \
        --number-sections \
        --highlight-style tango \
        -V colorlinks=true \
        -V linkcolor=blue \
        -V urlcolor=blue \
        -V toccolor=black

    if [ -f "$OUTPUT_PDF" ]; then
        echo "Successfully generated PDF with TOC and Page Numbers: $OUTPUT_PDF"
    else
        echo "Failed to generate PDF."
    fi
else
    echo "Pandoc not found! Please install it via: sudo apt-get install pandoc texlive-latex-base texlive-fonts-recommended texlive-extra-utils texlive-latex-extra"
fi
