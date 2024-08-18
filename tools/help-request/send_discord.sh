#!/bin/bash

# Use the secrets passed from the GitHub Action environment
SOFTWARE_LEADS_ROLE_ID="$SOFTWARE_LEADS_ROLE_ID"
DISCORD_WEBHOOK_URL="$DISCORD_WEBHOOK_URL"
GITHUB_REPO="MissouriMRDT/Autonomy_Software"

# Initialize the base of the JSON payload
discord_message=$(cat <<EOF
{
  "content": "<@&$SOFTWARE_LEADS_ROLE_ID> Someone is looking for assistance on the [MissouriMRDT/Autonomy_Software](https://github.com/MissouriMRDT/Autonomy_Software) repo. Please see the below details to give assistance.",
  "embeds": []
}
EOF
)

# Initialize variables
embed_counter=1

# Create an empty embeds array
embeds="[]"

# Read through the comments directory and add each file to the Discord message
for comment_file in comments/*.txt; do
  # Read the file contents
  file_path=$(sed -n 's|File: ||p' "$comment_file")
  start_line=$(sed -n 's|Start Line: ||p' "$comment_file")
  end_line=$(sed -n 's|End Line: ||p' "$comment_file")
  commit_hash=$(sed -n 's|Commit Hash: ||p' "$comment_file")
  comment=$(sed -n '/Comment:/,/^$/p' "$comment_file" | sed '1d') # Skip the first line that says "Comment:"

  # Remove carriage returns (\r) and trim any extra newline at the end
  comment=$(echo "$comment" | sed 's/\r//g' | sed '${/^$/d}')
  comment="\`\`\`$comment\`\`\`"

  # Format line number(s) based on whether start and end lines are the same
  if [ "$start_line" -eq "$end_line" ]; then
    line_range="$start_line"
    line_label="Line Number:"
    url_line_fragment="#L$start_line"
  else
    line_range="$start_line-$end_line"
    line_label="Line Numbers:"
    url_line_fragment="#L$start_line-L$end_line"
  fi

  # Create the URL to the specific file in the commit
  file_url="https://github.com/$GITHUB_REPO/blob/$commit_hash/$file_path$url_line_fragment"

  # Create the embed JSON structure
  embed=$(jq -n --arg title "Assistance Request #$embed_counter" \
                  --arg file_path "$file_path" \
                  --arg line_label "$line_label" \
                  --arg line_range "$line_range" \
                  --arg comment "$comment" \
                  --arg file_url "$file_url" \
  '{
    "title": $title,
    "color": null,
    "fields": [
      {
        "name": "File Path:",
        "value": $file_path,
        "inline": true
      },
      {
        "name": $line_label,
        "value": $line_range,
        "inline": true
      },
      {
        "name": "Comment:",
        "value": $comment
      },
      {
        "name": "Link:",
        "value": $file_url
      }
    ]
  }')

  # Add this embed to the embeds array
  embeds=$(echo "$embeds" | jq --argjson embed "$embed" '. += [$embed]')

  embed_counter=$((embed_counter + 1))
done

# Add the embeds array to the discord_message JSON
discord_message=$(echo "$discord_message" | jq --argjson embeds "$embeds" '.embeds = $embeds')

# Validate and print the final JSON structure (optional for debugging)
echo "$discord_message" | jq . || exit 1

# Send the message to the Discord webhook
curl -H "Content-Type: application/json" -d "$discord_message" "$DISCORD_WEBHOOK_URL" || {
  echo "Failed to send Discord notification" >&2
  exit 1
}
