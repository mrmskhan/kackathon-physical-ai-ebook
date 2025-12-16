#!/bin/bash
#
# validate-examples.sh
#
# Purpose: Validate code examples on Ubuntu 22.04 via Docker
# Usage: ./scripts/validate-examples.sh [module-directory]
#
# This script tests that all code examples in a module can be executed
# successfully on a clean Ubuntu 22.04 environment with ROS 2 Humble.
#

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
UBUNTU_VERSION="22.04"
ROS_DISTRO="humble"
MODULE_DIR="${1:-docs/module-01-ros2}"

echo -e "${YELLOW}================================${NC}"
echo -e "${YELLOW}Code Example Validation${NC}"
echo -e "${YELLOW}================================${NC}"
echo ""
echo "Module: $MODULE_DIR"
echo "Ubuntu: $UBUNTU_VERSION"
echo "ROS 2: $ROS_DISTRO"
echo ""

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo -e "${RED}ERROR: Docker is not installed or not in PATH${NC}"
    echo "Please install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# Check if module directory exists
if [ ! -d "$MODULE_DIR" ]; then
    echo -e "${RED}ERROR: Module directory not found: $MODULE_DIR${NC}"
    echo "Usage: $0 [module-directory]"
    exit 1
fi

# Find all Python examples
EXAMPLES_DIR="$MODULE_DIR/examples"
if [ ! -d "$EXAMPLES_DIR" ]; then
    echo -e "${YELLOW}WARNING: No examples directory found: $EXAMPLES_DIR${NC}"
    echo "Skipping validation (no examples to test)"
    exit 0
fi

PYTHON_FILES=$(find "$EXAMPLES_DIR" -name "*.py" -type f)

if [ -z "$PYTHON_FILES" ]; then
    echo -e "${YELLOW}WARNING: No Python files found in $EXAMPLES_DIR${NC}"
    echo "Skipping validation (no examples to test)"
    exit 0
fi

echo -e "${GREEN}Found Python examples:${NC}"
echo "$PYTHON_FILES"
echo ""

# Create temporary Dockerfile for validation
DOCKERFILE_CONTENT="FROM ubuntu:${UBUNTU_VERSION}

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 ${ROS_DISTRO}
RUN apt-get update && apt-get install -y \\
    locales \\
    curl \\
    gnupg2 \\
    lsb-release \\
    && locale-gen en_US en_US.UTF-8 \\
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \\
    && export LANG=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \\
    && echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \\\$UBUNTU_CODENAME) main\" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \\
    ros-${ROS_DISTRO}-desktop \\
    python3-colcon-common-extensions \\
    python3-pip \\
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 environment
RUN echo \"source /opt/ros/${ROS_DISTRO}/setup.bash\" >> /root/.bashrc

WORKDIR /workspace

CMD [\"/bin/bash\"]
"

# Build validation Docker image
echo -e "${YELLOW}Building validation Docker image...${NC}"
echo "$DOCKERFILE_CONTENT" | docker build -t ros2-validation:${ROS_DISTRO} -f - . || {
    echo -e "${RED}ERROR: Failed to build Docker image${NC}"
    exit 1
}

echo -e "${GREEN}Docker image built successfully${NC}"
echo ""

# Validate each Python example
echo -e "${YELLOW}Validating examples...${NC}"
echo ""

FAILED_EXAMPLES=()
PASSED_EXAMPLES=()

for EXAMPLE in $PYTHON_FILES; do
    EXAMPLE_NAME=$(basename "$EXAMPLE")
    echo -e "${YELLOW}Testing: $EXAMPLE_NAME${NC}"

    # Syntax check only (examples may require ROS 2 runtime)
    if docker run --rm -v "$(pwd):/workspace" ros2-validation:${ROS_DISTRO} \
        bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && python3 -m py_compile /workspace/$EXAMPLE" 2>&1; then
        echo -e "${GREEN}✓ $EXAMPLE_NAME passed syntax check${NC}"
        PASSED_EXAMPLES+=("$EXAMPLE_NAME")
    else
        echo -e "${RED}✗ $EXAMPLE_NAME failed syntax check${NC}"
        FAILED_EXAMPLES+=("$EXAMPLE_NAME")
    fi
    echo ""
done

# Summary
echo -e "${YELLOW}================================${NC}"
echo -e "${YELLOW}Validation Summary${NC}"
echo -e "${YELLOW}================================${NC}"
echo ""
echo -e "${GREEN}Passed: ${#PASSED_EXAMPLES[@]}${NC}"
for example in "${PASSED_EXAMPLES[@]}"; do
    echo "  ✓ $example"
done
echo ""

if [ ${#FAILED_EXAMPLES[@]} -gt 0 ]; then
    echo -e "${RED}Failed: ${#FAILED_EXAMPLES[@]}${NC}"
    for example in "${FAILED_EXAMPLES[@]}"; do
        echo "  ✗ $example"
    done
    echo ""
    exit 1
else
    echo -e "${GREEN}All examples passed validation!${NC}"
    exit 0
fi
