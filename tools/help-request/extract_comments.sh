#!/bin/bash

# Get the commit ID from the arguments
commit_id="$1"

# Create the comments directory if it doesn't exist
mkdir -p comments

# Determine the next available comment number
if [ "$(ls -A comments)" ]; then
    # Find the highest numbered comment file and set the comment_counter accordingly
    comment_counter=$(ls comments | grep -Eo '[0-9]+' | sort -n | tail -1)
    comment_counter=$((comment_counter + 1))
else
    comment_counter=1
fi

# Run git show for the specific commit and capture the output
git_diff_output=$(git show --unified=0 --pretty="format:" "$commit_id")

# Initialize other variables
current_file=""
current_start_line=0

# Iterate over the lines of the git show output
while IFS= read -r line; do
    # Detect the file being diffed
    if [[ $line =~ ^diff\ --git\ a/(.+)\ b/(.+) ]]; then
        current_file="${BASH_REMATCH[2]}"

        # Skip processing if the file is CONTRIBUTING.md
        if [[ "$current_file" == "CONTRIBUTING.md" ]]; then
            echo "Skipping file: $current_file"
            continue
        fi
        continue
    fi

    # Detect the line number changes in the diff (where it starts)
    if [[ $line =~ ^@@\ \-(.+),(.+)\ \+(.+),(.+)\ @@ ]]; then
        current_start_line="${BASH_REMATCH[3]}"
        continue
    fi

    # Only capture added lines (those prefixed with '+')
    if [[ $line =~ ^\+ ]]; then
        # Capture multi-line comments starting with LEAD
        if [[ $line =~ \/\*\*?\ LEAD: ]]; then
            # Start of a multi-line LEAD comment block
            comment="${line:1}"  # Remove the leading '+'
            start_line=$current_start_line

            # Continue reading until the end of the comment block
            while IFS= read -r next_line; do
                if [[ $next_line =~ ^\+ ]]; then
                    comment+=$'\n'"${next_line:1}"  # Remove the leading '+'
                fi
                current_start_line=$((current_start_line + 1))
                [[ $next_line =~ \*\/ ]] && break
            done

            # Record the end line number
            end_line=$current_start_line

            # Write the comment to a new text file in the comments directory
            output_file="comments/$comment_counter.txt"
            echo "File: $current_file" > "$output_file"
            echo "Start Line: $start_line" >> "$output_file"
            echo "End Line: $end_line" >> "$output_file"
            echo "Commit Hash: $commit_id" >> "$output_file"
            echo "Comment:" >> "$output_file"
            echo "$comment" >> "$output_file"
            comment_counter=$((comment_counter + 1))
        fi

        # Capture single-line LEAD comments
        if [[ $line =~ //\ LEAD: ]]; then
            comment="${line:1}"  # Remove the leading '+'
            start_line=$current_start_line
            end_line=$current_start_line

            # Write the comment to a new text file in the comments directory
            output_file="comments/$comment_counter.txt"
            echo "File: $current_file" > "$output_file"
            echo "Start Line: $start_line" >> "$output_file"
            echo "End Line: $end_line" >> "$output_file"
            echo "Commit Hash: $commit_id" >> "$output_file"
            echo "Comment:" >> "$output_file"
            echo "$comment" >> "$output_file"
            comment_counter=$((comment_counter + 1))
        fi
    fi

    # Increment the line number for single line comments
    current_start_line=$((current_start_line + 1))

done <<< "$git_diff_output"

echo "LEAD comments have been extracted and saved in the comments directory."
