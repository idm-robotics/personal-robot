#!/bin/bash

DIR="$( cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd )"

${DIR}/rgbd-vision/build.sh
${DIR}/manipulator/build.sh