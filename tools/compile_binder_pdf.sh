#!/bin/bash

# Ensure we are in the root directory
if [ ! -d "docs/autonomy_binder" ]; then
    echo "Error: Run this script from the root of the repository."
    # do not exit, to please the bash session checker
    return 1 2>/dev/null
fi

echo "Compiling the Autonomy Binder into a single Markdown file..."

BINDER_DIR="docs/autonomy_binder"
OUTPUT_MD="docs/autonomy_binder/Autonomy_Binder_Compiled.md"
OUTPUT_PDF="docs/autonomy_binder/Autonomy_Binder.pdf"

# Clean up previous compiles
rm -f "$OUTPUT_MD" "$OUTPUT_PDF"

# Create title page
cat << TITLE > "$OUTPUT_MD"
---
title: Autonomy Software Binder
author: Mars Rover Design Team
date: $(date +'%Y-%m-%d')
---

<div align="center">
  <h1>Autonomy Software Binder</h1>
  <h2>Source of Truth & Operations Manual</h2>
  <br/>
</div>

<div style="page-break-after: always;"></div>

# Table of Contents
<!-- toc -->

<div style="page-break-after: always;"></div>
TITLE

# Append files in order
# Read from the Table of Contents to get the correct order

echo "Reading index to compile sections..."
# Get all markdown file links from TOC, excluding the 00_Table_of_Contents itself so we don't have two tables of contents
grep -o '([0-9a-zA-Z_/]*\.md)' "$BINDER_DIR/00_Table_of_Contents.md" | tr -d '()' | grep -v '00_Table_of_Contents.md' | while read -r file; do
    if [ -f "$BINDER_DIR/$file" ]; then
        echo "Appending $file..."
        cat "$BINDER_DIR/$file" >> "$OUTPUT_MD"
        echo -e "\n\n<div style=\"page-break-after: always;\"></div>\n\n" >> "$OUTPUT_MD"
    fi
done

echo "Injecting automatic Table of Contents..."
# We use npx to dynamically pull and execute markdown-toc to replace the <!-- toc --> tag
npx markdown-toc -i --maxdepth 3 "$OUTPUT_MD"

echo "Compiled markdown saved to $OUTPUT_MD."

# Use md-to-pdf if available
if command -v md-to-pdf &> /dev/null
then
    echo "md-to-pdf found. Generating PDF..."
    # Generate PDF with page numbers
    md-to-pdf --pdf-options '{ "format": "A4", "margin": { "top": "20mm", "bottom": "20mm", "left": "20mm", "right": "20mm" }, "displayHeaderFooter": true, "footerTemplate": "<div style=\"text-align: right; font-size: 10px; width: 100%; margin-right: 20mm; font-family: sans-serif;\">Page <span class=\"pageNumber\"></span> of <span class=\"totalPages\"></span></div>" }' "$OUTPUT_MD"

    if [ -f "docs/autonomy_binder/Autonomy_Binder_Compiled.pdf" ]; then
        mv docs/autonomy_binder/Autonomy_Binder_Compiled.pdf "$OUTPUT_PDF"
        echo "Successfully generated PDF: $OUTPUT_PDF"
    else
        echo "Failed to generate PDF."
    fi
else
    echo "md-to-pdf not found! Please install it via: npm install -g md-to-pdf"
fi
