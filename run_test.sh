#!/bin/bash

echo "Starting Yade Core Scaling Benchmark..."
echo "---------------------------------------"

# Loop from 1 to 8 threads
for i in {1..5}
do
    # Run Yade silently, dumping all standard output and errors into the void
    yadedaily -j $i speed_test.py > /dev/null 2>&1
    
    # Read the text file our Python script just created
    result=$(cat benchmark_result.txt)
    
    echo "Threads: $i -> $result"
done

echo "---------------------------------------"
echo "Benchmark Complete!"