#!/usr/bin/env bash

cd "$(dirname "$0")"

pandoc changelog.md -o changelog.pdf
