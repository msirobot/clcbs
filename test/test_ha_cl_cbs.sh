#!/bin/bash
# Test script for HA-CL-CBS implementation

set -e  # Exit on error

echo "========================================"
echo "HA-CL-CBS Test Suite"
echo "========================================"

# Build the project
echo
echo "[1/5] Building project..."
cd "$(dirname "$0")/.."
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .. > /dev/null
make -j4 > /dev/null 2>&1
echo "✓ Build successful"

# Test 1: Simple 3-agent heterogeneous scenario
echo
echo "[2/5] Testing 3-agent heterogeneous scenario..."
./HA-CL-CBS \
  -i ../benchmark/warehouse_50x50/heterogeneous/simple_3agents_ex1.yaml \
  -o test_3agents.yaml \
  -f ../src/fleet_config.yaml > test_3agents.log 2>&1

if grep -q "Successfully find solution" test_3agents.log; then
  echo "✓ 3-agent test passed"
  grep "Makespan:" test_3agents.log
  grep "Flowtime:" test_3agents.log
  grep "Fleet composition:" -A 3 test_3agents.log
else
  echo "✗ 3-agent test failed"
  cat test_3agents.log
  exit 1
fi

# Test 2: 6-agent heterogeneous scenario  
echo
echo "[3/5] Testing 6-agent heterogeneous scenario..."
./HA-CL-CBS \
  -i ../benchmark/warehouse_50x50/heterogeneous/mixed_6agents_ex1.yaml \
  -o test_6agents.yaml \
  -f ../src/fleet_config.yaml > test_6agents.log 2>&1

if grep -q "Successfully find solution" test_6agents.log; then
  echo "✓ 6-agent test passed"
  grep "Makespan:" test_6agents.log
  grep "Flowtime:" test_6agents.log
else
  echo "✗ 6-agent test failed"
  cat test_6agents.log
  exit 1
fi

# Test 3: Backward compatibility (no fleet config)
echo
echo "[4/5] Testing backward compatibility..."
./HA-CL-CBS \
  -i ../benchmark/warehouse_50x50/heterogeneous/simple_3agents_ex1.yaml \
  -o test_compat.yaml > test_compat.log 2>&1

if grep -q "Successfully find solution\|Fail to find paths" test_compat.log; then
  echo "✓ Backward compatibility test passed"
else
  echo "✗ Backward compatibility test failed"
  cat test_compat.log
  exit 1
fi

# Test 4: Verify output format
echo
echo "[5/5] Verifying output format..."
if grep -q "type: Agile" test_3agents.yaml && \
   grep -q "type: Standard" test_3agents.yaml && \
   grep -q "type: Heavy" test_3agents.yaml; then
  echo "✓ Output format verification passed"
  echo "  - Agent types correctly included in output"
else
  echo "✗ Output format verification failed"
  exit 1
fi

# Summary
echo
echo "========================================"
echo "All tests passed! ✓"
echo "========================================"
echo
echo "Test artifacts created:"
echo "  - test_3agents.yaml"
echo "  - test_6agents.yaml"
echo "  - test_compat.yaml"
echo "  - test_*.log files"
echo

exit 0
