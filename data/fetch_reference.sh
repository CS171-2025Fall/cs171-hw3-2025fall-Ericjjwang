#!/usr/bin/env bash

REFERENCE_URL="https://fc46e1fd-aaec-4163-a6ce-4f482ec3323a.s3.us-west-004.backblazeb2.com/rdr171/reference.tar.gz"
REFERENCE_DIR="mitsuba/reference"

echo "Downloading and extracting reference to $REFERENCE_DIR..."
mkdir -p "$REFERENCE_DIR"
curl -L -o reference.tar.gz "$REFERENCE_URL"
tar -xzf reference.tar.gz -C "$REFERENCE_DIR" --overwrite
echo "References fetched to $REFERENCE_DIR"
