#!/bin/bash

# Format all C++-like files with clang-format
# Determine the applied differences with git,
# return 1 when changes had to be made, so the CI step fails.

# set exit on error
set -e

# check that we are in a clean state in order to prevent accidental changes
git status --untracked-files=no
if [ ! -z "$(git status --untracked-files=no --porcelain)" ]; then
    echo "Script must be applied on a clean git state"
    exit 1
fi

echo

# preference list
clang_format_tool_candiates=(clang-format-10 clang-format)

for candidate in ${clang_format_tool_candiates[@]}; do

    if command -v "$candidate" >/dev/null; then
        echo "Formatting check with $candidate --version:"
        $candidate --version
        clang_format_tool=$candidate
        break
    fi
done

if [[ -z "$clang_format_tool" ]]; then
    echo "$clang_format_tool not correctly installed"
    exit 1
fi

echo

# run clang-format
format_call="find cpp/ -regextype egrep -regex '.+\.(h|hpp|cpp|cu|cuh)$' | xargs $clang_format_tool -i -style=file"
eval "$format_call"

# check if something was modified
notcorrectlist=`git status --porcelain | grep '^ M' | cut -c4-`
# if nothing changed ok
if [[ -z $notcorrectlist ]]; then
    echo "Excellent. You passed the formatting check!"
    exit 0;
    else
    echo "The following files have clang-format problems:"
    git diff --stat $notcorrectlist
    echo "Inside the repo, please run"
    echo
    echo "$format_call"
    echo
    echo "to solve the issue."

    # cleanup changes in git
    git reset HEAD --hard
fi

exit 1