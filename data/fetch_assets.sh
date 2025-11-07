#!/usr/bin/env bash

ASSETS_URL="https://fc46e1fd-aaec-4163-a6ce-4f482ec3323a.s3.us-west-004.backblazeb2.com/rdr171/assets.tar.gz"
ASSETS_DIR="assets"

echo "Downloading and extracting assets to $ASSETS_DIR..."
mkdir -p "$ASSETS_DIR"
curl -L -o assets.tar.gz "$ASSETS_URL"
tar -xzf assets.tar.gz -C "$ASSETS_DIR" --overwrite
echo "Assets extracted and existing files overwritten in $ASSETS_DIR"
