#!/bin/bash
set -e
cd /Users/aiziqi/Desktop/RobotNav

echo "=== Cleaning old git repos and builds ==="
rm -rf autompc/.git autoplanner/.git
rm -rf autompc/build autoplanner/build
rm -rf .git
find . -name '.DS_Store' -delete

echo ""
echo "=== Git status before init ==="
git status 2>/dev/null || echo "(no git yet, expected)"

echo ""
echo "=== Initializing monorepo ==="
git init
git add -A
echo ""
echo "=== Files staged ==="
git status --short | head -20
echo "..."
echo "Total: $(git status --short | wc -l) files"

echo ""
echo "=== Committing ==="
git commit -m "RobotNav monorepo: AutoPlanner + AutoMPC"

echo ""
echo "=== Building ==="
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON 2>&1 | tail -3
cmake --build build -j 2>&1 | tail -3

echo ""
echo "=== Testing ==="
ctest --test-dir build --output-on-failure 2>&1 | tail -5

echo ""
echo "=== Commit log ==="
git log --oneline

echo ""
echo "Done. To push:"
echo "  cd /Users/aiziqi/Desktop/RobotNav"
echo "  git remote add origin <your-repo-url>"
echo "  git push -u origin main"
